#!/usr/bin/env python3
"""
obio_gated_load_shedder.py

Demo-only external load-shedder for OBIO / AFIO.

It subscribes to /diagnostics, reads the OBIO diagnostic status, and toggles
/fake_slam_stressor load via /obio_demo/load_enabled.

This node intentionally does NOT perform perception, SLAM, flight control, or
PX4 failsafe logic. It demonstrates the integration pattern:

    OBIO boundary signal -> external autonomy manager -> shed non-critical work
"""

from __future__ import annotations

import json
import math
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Bool, String


def safe_float(value: Any, fallback: float = 0.0) -> float:
    try:
        x = float(value)
        return x if math.isfinite(x) else fallback
    except Exception:
        return fallback


def parse_boolish(value: Any) -> bool:
    s = str(value).strip().lower()
    return s in {"1", "true", "yes", "y", "stale", "error", "warn"}


class ObioGatedLoadShedder(Node):
    def __init__(self) -> None:
        super().__init__("obio_gated_load_shedder")

        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("target_name", "ai_flight_integrity/flight_execution_integrity")
        self.declare_parameter("load_control_topic", "/obio_demo/load_enabled")
        self.declare_parameter("status_topic", "/obio_demo/load_shedder_status")

        self.declare_parameter("setpoint_jitter_threshold_ms", 100.0)
        self.declare_parameter("setpoint_age_threshold_ms", 180.0)
        self.declare_parameter("flight_residual_threshold", 0.60)
        self.declare_parameter("pause_duration_sec", 2.0)
        self.declare_parameter("min_time_between_actions_sec", 1.0)

        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.target_name = str(self.get_parameter("target_name").value)
        self.load_control_topic = str(self.get_parameter("load_control_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.jitter_threshold = safe_float(self.get_parameter("setpoint_jitter_threshold_ms").value, 100.0)
        self.age_threshold = safe_float(self.get_parameter("setpoint_age_threshold_ms").value, 180.0)
        self.flight_residual_threshold = safe_float(self.get_parameter("flight_residual_threshold").value, 0.60)
        self.pause_duration_sec = max(0.1, safe_float(self.get_parameter("pause_duration_sec").value, 2.0))
        self.min_action_gap_sec = max(0.0, safe_float(self.get_parameter("min_time_between_actions_sec").value, 1.0))

        self.load_enabled = True
        self.pause_until = 0.0
        self.last_action_time = -1e9
        self.action_count = 0
        self.last_reason = "startup"
        self.last_metrics: Dict[str, Any] = {}

        self.diag_sub = self.create_subscription(
            DiagnosticArray,
            self.diagnostics_topic,
            self._diagnostics_callback,
            10,
        )
        self.control_pub = self.create_publisher(Bool, self.load_control_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.timer = self.create_timer(0.1, self._timer_callback)
        self.status_timer = self.create_timer(0.5, self._publish_status)

        # Start with load enabled.
        self._publish_load_enabled(True, reason="startup")

        self.get_logger().info(
            "OBIO gated load shedder started | "
            f"target={self.target_name} | jitter>{self.jitter_threshold:.1f}ms | "
            f"age>{self.age_threshold:.1f}ms | residual>{self.flight_residual_threshold:.3f} | "
            f"pause={self.pause_duration_sec:.1f}s"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_load_enabled(self, enabled: bool, reason: str) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.control_pub.publish(msg)
        self.load_enabled = bool(enabled)
        self.last_reason = reason

    def _extract_target(self, msg: DiagnosticArray) -> DiagnosticStatus | None:
        for status in msg.status:
            if status.name == self.target_name:
                return status
        return None

    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        status = self._extract_target(msg)
        if status is None:
            return

        values = {kv.key: kv.value for kv in status.values}
        sp_jitter = safe_float(values.get("setpointJitterMs", 0.0))
        sp_age = safe_float(values.get("setpointAgeMs", 0.0))
        flight_residual = safe_float(values.get("flightResidual", 0.0))
        stale_streams = parse_boolish(values.get("staleStreams", "false"))
        dominant = str(values.get("dominantCause", ""))
        level_name = str(values.get("diagnosticLevelName", "")) or str(status.message)

        trigger_reasons = []
        if sp_jitter >= self.jitter_threshold:
            trigger_reasons.append(f"setpointJitterMs={sp_jitter:.1f}")
        if sp_age >= self.age_threshold:
            trigger_reasons.append(f"setpointAgeMs={sp_age:.1f}")
        if flight_residual >= self.flight_residual_threshold:
            trigger_reasons.append(f"flightResidual={flight_residual:.3f}")
        if stale_streams or "STALE" in dominant:
            trigger_reasons.append(f"staleStreams/dominant={dominant}")
        if status.level >= DiagnosticStatus.ERROR:
            trigger_reasons.append(f"diagnosticLevel={level_name}")

        self.last_metrics = {
            "level": int(status.level),
            "message": status.message,
            "dominantCause": dominant,
            "setpointJitterMs": sp_jitter,
            "setpointAgeMs": sp_age,
            "flightResidual": flight_residual,
            "triggerReasons": trigger_reasons,
        }

        if trigger_reasons:
            self._trigger_shedding(trigger_reasons)

    def _trigger_shedding(self, reasons: list[str]) -> None:
        now = self._now_sec()
        if now - self.last_action_time < self.min_action_gap_sec:
            return

        self.pause_until = max(self.pause_until, now + self.pause_duration_sec)
        self.last_action_time = now
        self.action_count += 1
        reason = "; ".join(reasons[:3])
        self._publish_load_enabled(False, reason=f"shed: {reason}")
        self.get_logger().warn(
            f"OBIO: Shedding load for {self.pause_duration_sec:.1f}s | {reason}"
        )

    def _timer_callback(self) -> None:
        now = self._now_sec()
        if not self.load_enabled and now >= self.pause_until:
            self._publish_load_enabled(True, reason="re-enable load")
            self.get_logger().info("OBIO: Re-enabled fake SLAM workload")

    def _publish_status(self) -> None:
        payload = {
            "timestamp": self._now_sec(),
            "loadEnabled": self.load_enabled,
            "pauseRemainingSec": round(max(0.0, self.pause_until - self._now_sec()), 3),
            "actionCount": self.action_count,
            "lastReason": self.last_reason,
            "lastMetrics": self.last_metrics,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObioGatedLoadShedder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
