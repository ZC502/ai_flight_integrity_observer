#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Resource-Aware YOLO Adapter, powered by OBIO.

Modes:
  profile   : publish /obio/yolo_profile only
  throttle  : forward /camera/image_raw -> /obio/image_for_yolo with state-dependent frame skipping
  param     : update a target YOLO node through ROS 2 Parameter Service
  hybrid    : throttle + parameter control

Design notes:
  - Does not depend on a fixed DiagnosticStatus.name.
  - Parses diagnostic key/value pairs defensively.
  - Degrades immediately, but recovers only after a hysteresis window.
  - Topic throttling is the safe default because it does not require changing YOLO.
"""

import json
import re
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


STATES = ("GREEN", "YELLOW", "RED")
SEVERITY = {"GREEN": 0, "YELLOW": 1, "RED": 2}


class ObioYoloAdapter(Node):
    """Convert OBIO boundary-pressure diagnostics into YOLO runtime profiles."""

    def __init__(self) -> None:
        super().__init__("obio_yolo_adapter")

        # ------------------------- Parameters -------------------------
        self.declare_parameter("mode", "throttle")  # profile | throttle | param | hybrid
        self.declare_parameter("obio_diag_topic", "/diagnostics")
        self.declare_parameter("profile_topic", "/obio/yolo_profile")

        self.declare_parameter("input_image_topic", "/camera/image_raw")
        self.declare_parameter("output_image_topic", "/obio/image_for_yolo")

        self.declare_parameter("yellow_jitter_ms", 50.0)
        self.declare_parameter("red_jitter_ms", 100.0)
        self.declare_parameter("hysteresis_sec", 2.0)
        self.declare_parameter("profile_publish_hz", 1.0)

        self.declare_parameter("green_frame_stride", 1)
        self.declare_parameter("yellow_frame_stride", 2)
        self.declare_parameter("red_frame_stride", 4)

        self.declare_parameter("target_yolo_node", "/yolo_node")
        self.declare_parameter("imgsz_parameter_name", "imgsz")
        self.declare_parameter("green_imgsz", 960)
        self.declare_parameter("yellow_imgsz", 640)
        self.declare_parameter("red_imgsz", 320)

        self.declare_parameter("enable_color_log", False)

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in {"profile", "throttle", "param", "hybrid"}:
            self.get_logger().warning(
                f"Unknown mode '{self.mode}'. Falling back to 'profile'. "
                "Valid modes: profile, throttle, param, hybrid."
            )
            self.mode = "profile"

        self.diag_topic = str(self.get_parameter("obio_diag_topic").value)
        self.profile_topic = str(self.get_parameter("profile_topic").value)
        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.output_image_topic = str(self.get_parameter("output_image_topic").value)

        self.yellow_jitter_ms = float(self.get_parameter("yellow_jitter_ms").value)
        self.red_jitter_ms = float(self.get_parameter("red_jitter_ms").value)
        self.hysteresis_sec = max(0.0, float(self.get_parameter("hysteresis_sec").value))

        self.profile_publish_hz = float(self.get_parameter("profile_publish_hz").value)
        if self.profile_publish_hz <= 0.0:
            self.profile_publish_hz = 1.0

        self.frame_strides = {
            "GREEN": max(1, int(self.get_parameter("green_frame_stride").value)),
            "YELLOW": max(1, int(self.get_parameter("yellow_frame_stride").value)),
            "RED": max(1, int(self.get_parameter("red_frame_stride").value)),
        }
        self.imgsz_values = {
            "GREEN": max(1, int(self.get_parameter("green_imgsz").value)),
            "YELLOW": max(1, int(self.get_parameter("yellow_imgsz").value)),
            "RED": max(1, int(self.get_parameter("red_imgsz").value)),
        }

        self.target_yolo_node = str(self.get_parameter("target_yolo_node").value).strip()
        self.imgsz_parameter_name = str(self.get_parameter("imgsz_parameter_name").value).strip()
        self.enable_color_log = bool(self.get_parameter("enable_color_log").value)

        # ------------------------- State -------------------------
        self.current_state = "GREEN"
        self.recovery_candidate_state: Optional[str] = None
        self.recovery_candidate_since: Optional[float] = None

        self.last_jitter_ms = 0.0
        self.last_stale_stream = False
        self.last_stale_streams_raw = "None"
        self.last_dominant_cause = "NONE"
        self.last_transition_reason = "startup"
        self.last_diag_monotonic = 0.0

        self.frames_seen = 0
        self.frames_forwarded = 0
        self.frames_dropped = 0

        # ------------------------- Publishers / subscribers -------------------------
        profile_qos = QoSProfile(depth=10)
        profile_qos.reliability = ReliabilityPolicy.RELIABLE
        profile_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.profile_pub = self.create_publisher(String, self.profile_topic, profile_qos)

        self.diag_sub = self.create_subscription(
            DiagnosticArray,
            self.diag_topic,
            self.diag_callback,
            10,
        )

        self.image_pub = None
        self.image_sub = None
        if self.mode in {"throttle", "hybrid"}:
            # Subscribe using sensor-data QoS, publish using default reliable QoS for demo/tool compatibility.
            self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)
            self.image_sub = self.create_subscription(
                Image,
                self.input_image_topic,
                self.image_callback,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f"Topic throttler enabled: {self.input_image_topic} -> {self.output_image_topic}"
            )

        self.param_client = None
        if self.mode in {"param", "hybrid"}:
            node_name = self.target_yolo_node.rstrip("/")
            if not node_name:
                self.get_logger().warning("target_yolo_node is empty; parameter control disabled.")
            else:
                service_name = f"{node_name}/set_parameters"
                self.param_client = self.create_client(SetParameters, service_name)
                self.get_logger().info(
                    f"Parameter controller enabled: service={service_name}, parameter={self.imgsz_parameter_name}"
                )

        self.profile_timer = self.create_timer(1.0 / self.profile_publish_hz, self.publish_profile)

        self.get_logger().info(
            "Resource-Aware YOLO Adapter started | "
            f"mode={self.mode} | yellow_jitter_ms={self.yellow_jitter_ms:.1f} | "
            f"red_jitter_ms={self.red_jitter_ms:.1f} | hysteresis_sec={self.hysteresis_sec:.1f}"
        )
        self.publish_profile()

    # ---------------------------------------------------------------------
    # Diagnostics parsing
    # ---------------------------------------------------------------------
    @staticmethod
    def _norm_key(key: str) -> str:
        """Normalize diagnostic keys: setpointJitterMs -> setpointjitterms."""
        return "".join(ch for ch in key.strip().lower() if ch.isalnum())

    @staticmethod
    def _parse_float(value: str, default: float = 0.0) -> float:
        """Parse floats from strings such as '73.2', '73.2 ms', or 'jitter=73.2'."""
        if value is None:
            return default
        match = re.search(r"[-+]?\d+(?:\.\d+)?", str(value))
        if not match:
            return default
        try:
            return float(match.group(0))
        except ValueError:
            return default

    @staticmethod
    def _is_stale_value(value: str) -> Tuple[bool, str]:
        raw = "" if value is None else str(value).strip()
        normalized = raw.strip().lower()
        healthy_values = {"", "none", "false", "[]", "null", "nil", "0", "ok", "green"}
        return normalized not in healthy_values, raw if raw else "None"

    @staticmethod
    def _clean_cause(value: str) -> str:
        if value is None:
            return "NONE"
        cleaned = str(value).strip().upper()
        return cleaned if cleaned else "NONE"

    def diag_callback(self, msg: DiagnosticArray) -> None:
        """Parse OBIO DiagnosticArray without relying on DiagnosticStatus.name."""
        max_jitter_ms = 0.0
        stale_stream = False
        stale_streams_raw = "None"
        dominant_cause = "NONE"

        for status in msg.status:
            # Fallback: some diagnostics encode useful state in message rather than values.
            if getattr(status, "message", ""):
                message_cause = self._clean_cause(status.message)
                if any(token in message_cause for token in ("STALE", "SETPOINT_JITTER")):
                    dominant_cause = message_cause

            for kv in status.values:
                key = self._norm_key(kv.key)
                value = "" if kv.value is None else str(kv.value).strip()

                if key in {"setpointjitterms", "setpointjitter", "jitterms", "timingjitterms"}:
                    max_jitter_ms = max(max_jitter_ms, self._parse_float(value, 0.0))

                elif key in {"stalestreams", "stalestream", "streamstale", "missingstreams"}:
                    is_stale, raw = self._is_stale_value(value)
                    if is_stale:
                        stale_stream = True
                        stale_streams_raw = raw

                elif key in {"dominantcause", "cause", "dominantfault", "faultcause"}:
                    cause = self._clean_cause(value)
                    if cause != "NONE":
                        dominant_cause = cause

        self.last_jitter_ms = max_jitter_ms
        self.last_stale_stream = stale_stream
        self.last_stale_streams_raw = stale_streams_raw
        self.last_dominant_cause = dominant_cause
        self.last_diag_monotonic = time.monotonic()

        target_state, reason = self.compute_target_state(max_jitter_ms, stale_stream, dominant_cause)
        self.update_state_machine(target_state, reason)

    def compute_target_state(self, jitter_ms: float, stale_stream: bool, dominant_cause: str) -> Tuple[str, str]:
        cause = self._clean_cause(dominant_cause)

        red_causes = {
            "STALE_STREAM",
            "SETPOINT_STALE",
            "OFFBOARD_STALE",
            "ODOMETRY_STALE",
            "COMMAND_RESPONSE_MISMATCH",
            "POSITION_RESPONSE_MISMATCH",
        }
        yellow_causes = {
            "SETPOINT_JITTER",
            "TIMING_JITTER",
            "FLIGHT_RESIDUAL_WARNING",
        }

        if stale_stream:
            return "RED", f"stale_streams={self.last_stale_streams_raw}"
        if jitter_ms >= self.red_jitter_ms:
            return "RED", f"setpointJitterMs={jitter_ms:.1f} >= {self.red_jitter_ms:.1f}"
        if cause in red_causes or "STALE" in cause:
            return "RED", f"dominantCause={cause}"
        if jitter_ms >= self.yellow_jitter_ms:
            return "YELLOW", f"setpointJitterMs={jitter_ms:.1f} >= {self.yellow_jitter_ms:.1f}"
        if cause in yellow_causes:
            return "YELLOW", f"dominantCause={cause}"
        return "GREEN", "boundary_healthy"

    # ---------------------------------------------------------------------
    # State machine
    # ---------------------------------------------------------------------
    def update_state_machine(self, target_state: str, reason: str) -> None:
        if target_state not in SEVERITY:
            return

        now = time.monotonic()
        current_level = SEVERITY[self.current_state]
        target_level = SEVERITY[target_state]

        # Immediate degradation: GREEN->YELLOW, GREEN->RED, YELLOW->RED.
        if target_level > current_level:
            self.recovery_candidate_state = None
            self.recovery_candidate_since = None
            self.transition_to(target_state, reason)
            return

        # Same state: remain there and reset any recovery candidate.
        if target_level == current_level:
            self.recovery_candidate_state = None
            self.recovery_candidate_since = None
            return

        # Recovery path: RED->YELLOW, RED->GREEN, or YELLOW->GREEN requires hysteresis.
        if self.recovery_candidate_state != target_state:
            self.recovery_candidate_state = target_state
            self.recovery_candidate_since = now
            self.last_transition_reason = f"recovery_candidate={target_state}; waiting {self.hysteresis_sec:.1f}s"
            return

        assert self.recovery_candidate_since is not None
        stable_for = now - self.recovery_candidate_since
        if stable_for >= self.hysteresis_sec:
            self.transition_to(target_state, f"{reason}; stable_for={stable_for:.1f}s")
            self.recovery_candidate_state = None
            self.recovery_candidate_since = None

    def transition_to(self, new_state: str, reason: str) -> None:
        old_state = self.current_state
        self.current_state = new_state
        self.last_transition_reason = reason

        if self.enable_color_log:
            color = {"GREEN": "\033[92m", "YELLOW": "\033[93m", "RED": "\033[91m"}.get(new_state, "")
            reset = "\033[0m"
            state_text = f"{color}{old_state} -> {new_state}{reset}"
        else:
            state_text = f"{old_state} -> {new_state}"

        self.get_logger().info(
            f"OBIO-YOLO state transition: {state_text} | "
            f"jitter={self.last_jitter_ms:.1f}ms | cause={self.last_dominant_cause} | reason={reason}"
        )

        self.publish_profile()
        if self.mode in {"param", "hybrid"}:
            self.trigger_remote_imgsz_update()

    # ---------------------------------------------------------------------
    # Topic throttling
    # ---------------------------------------------------------------------
    def image_callback(self, msg: Image) -> None:
        if self.image_pub is None:
            return

        stride = max(1, int(self.frame_strides.get(self.current_state, 1)))
        should_forward = (self.frames_seen % stride) == 0
        self.frames_seen += 1

        if should_forward:
            self.frames_forwarded += 1
            self.image_pub.publish(msg)
        else:
            self.frames_dropped += 1

    # ---------------------------------------------------------------------
    # Optional parameter control
    # ---------------------------------------------------------------------
    def trigger_remote_imgsz_update(self) -> None:
        if self.param_client is None:
            return
        if not self.imgsz_parameter_name:
            self.get_logger().warning("imgsz_parameter_name is empty; skipping parameter update.")
            return
        if not self.param_client.service_is_ready():
            self.get_logger().warning(
                f"Parameter service for {self.target_yolo_node} is not ready; skipping imgsz update."
            )
            return

        target_imgsz = int(self.imgsz_values.get(self.current_state, 640))
        request = SetParameters.Request()
        request.parameters = [
            Parameter(self.imgsz_parameter_name, Parameter.Type.INTEGER, target_imgsz).to_parameter_msg()
        ]

        self.get_logger().info(
            f"Requesting YOLO parameter update: {self.imgsz_parameter_name}={target_imgsz} "
            f"on {self.target_yolo_node}"
        )
        future = self.param_client.call_async(request)
        future.add_done_callback(self._on_set_parameters_done)

    def _on_set_parameters_done(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS client futures may raise transport errors.
            self.get_logger().warning(f"YOLO parameter update failed: {exc}")
            return

        if not response or not getattr(response, "results", None):
            self.get_logger().warning("YOLO parameter update returned no result.")
            return

        result = response.results[0]
        if result.successful:
            self.get_logger().info("YOLO parameter update accepted.")
        else:
            reason = getattr(result, "reason", "")
            self.get_logger().warning(f"YOLO parameter update rejected: {reason}")

    # ---------------------------------------------------------------------
    # Profile stream
    # ---------------------------------------------------------------------
    def publish_profile(self) -> None:
        profile: Dict[str, Any] = {
            "state": self.current_state,
            "mode": self.mode,
            "imgsz": int(self.imgsz_values.get(self.current_state, 640)),
            "frame_stride": int(self.frame_strides.get(self.current_state, 1)),
            "hysteresis_sec": float(self.hysteresis_sec),
            "yellow_jitter_ms": float(self.yellow_jitter_ms),
            "red_jitter_ms": float(self.red_jitter_ms),
            "last_setpoint_jitter_ms": round(float(self.last_jitter_ms), 3),
            "last_stale_stream": bool(self.last_stale_stream),
            "last_stale_streams_raw": self.last_stale_streams_raw,
            "last_dominant_cause": self.last_dominant_cause,
            "reason": self.last_transition_reason,
            "frames_seen": int(self.frames_seen),
            "frames_forwarded": int(self.frames_forwarded),
            "frames_dropped": int(self.frames_dropped),
        }

        msg = String()
        msg.data = json.dumps(profile, sort_keys=True)
        self.profile_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObioYoloAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
