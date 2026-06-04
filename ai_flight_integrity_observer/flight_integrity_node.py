#!/usr/bin/env python3
"""
flight_integrity_node.py

AI Flight Integrity Observer v0.1.2+

Observe-only ROS 2 / PX4 flight execution-integrity observer.

Core idea
---------
Subscribe to PX4 offboard intent and vehicle feedback:

    /fmu/in/offboard_control_mode
    /fmu/in/trajectory_setpoint
    /fmu/out/vehicle_odometry

Evaluate whether offboard setpoint intent remains physically consistent
with vehicle odometry under companion-compute / AI load.

Publish the result as standard ROS diagnostics:

    /diagnostics
        ai_flight_integrity/flight_execution_integrity

This node does NOT:
- command the vehicle
- publish setpoints
- modify PX4
- replace failsafe logic
- intercept flight control

It only observes and reports.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import math

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleOdometry,
)

try:
    from px4_msgs.msg import EstimatorStatus
except Exception:
    EstimatorStatus = None

try:
    from .px4_qos import PX4_SENSOR_QOS
except ImportError:
    from px4_qos import PX4_SENSOR_QOS

try:
    from .flight_residual_core import (
        TREND_DECREASING,
        classify_flight_integrity,
        infer_control_semantic_mode,
        position_residual_trend,
        primary_residual_type_for_mode,
    )
except ImportError:
    from flight_residual_core import (
        TREND_DECREASING,
        classify_flight_integrity,
        infer_control_semantic_mode,
        position_residual_trend,
        primary_residual_type_for_mode,
    )


# ============================================================
# Helpers
# ============================================================

DIAG_OK = 0
DIAG_WARN = 1
DIAG_ERROR = 2
DIAG_STALE = 3


def set_diagnostic_level(status_msg: DiagnosticStatus, level: int) -> None:
    """
    Set DiagnosticStatus.level in a ROS 2 distro-compatible way.

    In ROS 2 Humble Python, DiagnosticStatus.level may be generated
    as a byte field and expects bytes([level]), not int(level).
    """
    level_int = int(level)
    level_int = max(0, min(3, level_int))

    try:
        status_msg.level = bytes([level_int])
    except (AssertionError, TypeError):
        status_msg.level = level_int


def key_value(key: str, value: Any) -> KeyValue:
    return KeyValue(key=str(key), value=str(value))


def finite_or(x: Any, fallback: float = 0.0) -> float:
    try:
        value = float(x)
        return value if math.isfinite(value) else fallback
    except Exception:
        return fallback


def is_finite(x: Any) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def vec3_from(value: Any, fallback: float = math.nan) -> List[float]:
    try:
        return [
            float(value[0]),
            float(value[1]),
            float(value[2]),
        ]
    except Exception:
        return [fallback, fallback, fallback]


def vec_norm(values: List[float]) -> float:
    total = 0.0
    valid = False
    for v in values:
        if is_finite(v):
            total += float(v) * float(v)
            valid = True
    return math.sqrt(total) if valid else 0.0


def masked_diff_norm(a: List[float], b: List[float]) -> float:
    total = 0.0
    valid = False

    for av, bv in zip(a, b):
        if is_finite(av) and is_finite(bv):
            d = float(av) - float(bv)
            total += d * d
            valid = True

    return math.sqrt(total) if valid else 0.0


def px4_timestamp_us_to_sec(timestamp_us: Any) -> float:
    try:
        return float(timestamp_us) * 1e-6
    except Exception:
        return 0.0


def any_offboard_axis_enabled(msg: OffboardControlMode) -> bool:
    fields = [
        "position",
        "velocity",
        "acceleration",
        "attitude",
        "body_rate",
        "actuator",
        "thrust_and_torque",
        "direct_actuator",
    ]

    for field in fields:
        if hasattr(msg, field):
            try:
                if bool(getattr(msg, field)):
                    return True
            except Exception:
                pass

    return False


# ============================================================
# Buffers
# ============================================================

@dataclass
class BufferedSetpoint:
    msg: TrajectorySetpoint
    receive_time: float
    timestamp_sec: float


@dataclass
class BufferedOdom:
    msg: VehicleOdometry
    receive_time: float
    timestamp_sec: float


@dataclass
class BufferedOffboardMode:
    msg: OffboardControlMode
    receive_time: float
    timestamp_sec: float


# ============================================================
# Node
# ============================================================

class FlightIntegrityNode(Node):
    def __init__(self) -> None:
        super().__init__("flight_integrity_node")

        # --------------------------------------------------------
        # Topics
        # --------------------------------------------------------
        self.declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("vehicle_odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("estimator_status_topic", "/fmu/out/estimator_status")
        self.declare_parameter("subscribe_estimator_status", False)
        self.declare_parameter("diagnostics_topic", "/diagnostics")

        # --------------------------------------------------------
        # Diagnostics identity
        # --------------------------------------------------------
        self.declare_parameter(
            "diagnostic_name",
            "ai_flight_integrity/flight_execution_integrity",
        )
        self.declare_parameter(
            "hardware_id",
            "px4_offboard_physics_boundary_observer",
        )

        # --------------------------------------------------------
        # Timing / rate
        # --------------------------------------------------------
        self.declare_parameter("evaluation_rate_hz", 20.0)
        self.declare_parameter("diagnostics_rate_hz", 10.0)

        # General missing/stale stream thresholds.
        self.declare_parameter("data_stale_warn_sec", 1.0)
        self.declare_parameter("data_stale_error_sec", 3.0)

        # Expected periods and jitter thresholds.
        self.declare_parameter("expected_setpoint_period_ms", 50.0)
        self.declare_parameter("expected_odometry_period_ms", 50.0)
        self.declare_parameter("expected_offboard_mode_period_ms", 50.0)

        self.declare_parameter("setpoint_jitter_warn_ms", 80.0)
        self.declare_parameter("setpoint_jitter_error_ms", 250.0)

        # Stream freshness thresholds.
        self.declare_parameter("setpoint_stale_warn_ms", 150.0)
        self.declare_parameter("setpoint_stale_error_ms", 500.0)
        self.declare_parameter("odometry_stale_warn_ms", 150.0)
        self.declare_parameter("odometry_stale_error_ms", 500.0)
        self.declare_parameter("offboard_stale_warn_ms", 300.0)
        self.declare_parameter("offboard_stale_error_ms", 800.0)

        # Residual thresholds.
        self.declare_parameter("velocity_residual_warn_mps", 0.50)
        self.declare_parameter("velocity_residual_error_mps", 1.00)
        # Mixed position+velocity mode only: absolute velocity residual
        # fuse. This catches actuator/impact/propulsion failures even while
        # position tracking is still in a permissible transient phase.
        self.declare_parameter("critical_velocity_residual_mps", 4.00)

        self.declare_parameter("position_residual_warn_m", 1.50)
        self.declare_parameter("position_residual_error_m", 3.00)

        # v0.1.2 mode-aware classification: in PX4 position mode,
        # decreasing position error during climb/descent is treated as a
        # tracking transient rather than a command-response failure.
        self.declare_parameter("position_residual_trend_deadband_m", 0.05)
        self.declare_parameter("position_residual_stable_count_threshold", 5)

        self.declare_parameter("position_jump_min_m", 1.00)
        self.declare_parameter("position_jump_margin_m", 0.20)
        self.declare_parameter("position_jump_ratio", 5.0)

        # Event hold for short-lived jump events.
        self.declare_parameter("diagnostic_event_hold_sec", 1.0)

        # Load metadata placeholders.
        self.declare_parameter("load_profile", "unknown")
        self.declare_parameter("cpu_load_percent", 0.0)
        self.declare_parameter("gpu_load_percent", 0.0)
        self.declare_parameter("npu_load_percent", 0.0)
        self.declare_parameter("ai_inference_latency_ms", 0.0)

        # --------------------------------------------------------
        # Read params
        # --------------------------------------------------------
        self.trajectory_setpoint_topic = str(
            self.get_parameter("trajectory_setpoint_topic").value
        )
        self.offboard_control_mode_topic = str(
            self.get_parameter("offboard_control_mode_topic").value
        )
        self.vehicle_odometry_topic = str(
            self.get_parameter("vehicle_odometry_topic").value
        )
        self.estimator_status_topic = str(
            self.get_parameter("estimator_status_topic").value
        )
        self.subscribe_estimator_status = bool(
            self.get_parameter("subscribe_estimator_status").value
        )

        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.diagnostic_name = str(self.get_parameter("diagnostic_name").value)
        self.hardware_id = str(self.get_parameter("hardware_id").value)

        self.evaluation_rate_hz = float(self.get_parameter("evaluation_rate_hz").value)
        self.diagnostics_rate_hz = float(self.get_parameter("diagnostics_rate_hz").value)

        self.data_stale_warn_sec = float(self.get_parameter("data_stale_warn_sec").value)
        self.data_stale_error_sec = float(self.get_parameter("data_stale_error_sec").value)

        self.expected_setpoint_period_ms = float(
            self.get_parameter("expected_setpoint_period_ms").value
        )
        self.expected_odometry_period_ms = float(
            self.get_parameter("expected_odometry_period_ms").value
        )
        self.expected_offboard_mode_period_ms = float(
            self.get_parameter("expected_offboard_mode_period_ms").value
        )

        self.setpoint_jitter_warn_ms = float(
            self.get_parameter("setpoint_jitter_warn_ms").value
        )
        self.setpoint_jitter_error_ms = float(
            self.get_parameter("setpoint_jitter_error_ms").value
        )

        self.setpoint_stale_warn_ms = float(
            self.get_parameter("setpoint_stale_warn_ms").value
        )
        self.setpoint_stale_error_ms = float(
            self.get_parameter("setpoint_stale_error_ms").value
        )
        self.odometry_stale_warn_ms = float(
            self.get_parameter("odometry_stale_warn_ms").value
        )
        self.odometry_stale_error_ms = float(
            self.get_parameter("odometry_stale_error_ms").value
        )
        self.offboard_stale_warn_ms = float(
            self.get_parameter("offboard_stale_warn_ms").value
        )
        self.offboard_stale_error_ms = float(
            self.get_parameter("offboard_stale_error_ms").value
        )

        self.velocity_residual_warn_mps = float(
            self.get_parameter("velocity_residual_warn_mps").value
        )
        self.velocity_residual_error_mps = float(
            self.get_parameter("velocity_residual_error_mps").value
        )
        self.critical_velocity_residual_mps = float(
            self.get_parameter("critical_velocity_residual_mps").value
        )
        self.position_residual_warn_m = float(
            self.get_parameter("position_residual_warn_m").value
        )
        self.position_residual_error_m = float(
            self.get_parameter("position_residual_error_m").value
        )
        self.position_residual_trend_deadband_m = float(
            self.get_parameter("position_residual_trend_deadband_m").value
        )
        self.position_residual_stable_count_threshold = int(
            self.get_parameter("position_residual_stable_count_threshold").value
        )
        self.position_jump_min_m = float(self.get_parameter("position_jump_min_m").value)
        self.position_jump_margin_m = float(
            self.get_parameter("position_jump_margin_m").value
        )
        self.position_jump_ratio = float(self.get_parameter("position_jump_ratio").value)

        self.diagnostic_event_hold_sec = float(
            self.get_parameter("diagnostic_event_hold_sec").value
        )

        self.load_profile = str(self.get_parameter("load_profile").value)
        self.cpu_load_percent = float(self.get_parameter("cpu_load_percent").value)
        self.gpu_load_percent = float(self.get_parameter("gpu_load_percent").value)
        self.npu_load_percent = float(self.get_parameter("npu_load_percent").value)
        self.ai_inference_latency_ms = float(
            self.get_parameter("ai_inference_latency_ms").value
        )

        # --------------------------------------------------------
        # Runtime state
        # --------------------------------------------------------
        self.node_start_time = self._now_sec()

        self.last_setpoint: Optional[BufferedSetpoint] = None
        self.prev_setpoint: Optional[BufferedSetpoint] = None

        self.last_odom: Optional[BufferedOdom] = None
        self.prev_odom: Optional[BufferedOdom] = None

        self.last_offboard_mode: Optional[BufferedOffboardMode] = None
        self.prev_offboard_mode: Optional[BufferedOffboardMode] = None

        self.last_estimator_status_msg: Optional[Any] = None
        self.last_estimator_status_receive_time: Optional[float] = None

        # v0.1.2: track residual trend and a short non-decreasing
        # counter to avoid diagnostic flicker during normal PX4
        # position-mode convergence.
        self.prev_position_tracking_residual: Optional[float] = None
        self.prev_control_semantic_mode: Optional[str] = None
        self.position_residual_stable_count = 0

        self.setpoint_interval_ms = 0.0
        self.setpoint_jitter_ms = 0.0

        self.odometry_interval_ms = 0.0
        self.odometry_jitter_ms = 0.0

        self.offboard_mode_interval_ms = 0.0
        self.offboard_mode_jitter_ms = 0.0

        self.last_error_payload: Optional[Dict[str, Any]] = None
        self.last_error_hold_until = 0.0

        self.stats = {
            "setpoint_received": 0,
            "offboard_mode_received": 0,
            "odometry_received": 0,
            "estimator_status_received": 0,
            "evaluations": 0,
            "waiting_count": 0,
            "diagnostics_published": 0,
        }

        self.last_payload: Dict[str, Any] = self._waiting_payload()

        # --------------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------------
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            self.diagnostics_topic,
            10,
        )

        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            self.trajectory_setpoint_topic,
            self._trajectory_setpoint_callback,
            PX4_SENSOR_QOS,
        )

        self.offboard_sub = self.create_subscription(
            OffboardControlMode,
            self.offboard_control_mode_topic,
            self._offboard_control_mode_callback,
            PX4_SENSOR_QOS,
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            self.vehicle_odometry_topic,
            self._vehicle_odometry_callback,
            PX4_SENSOR_QOS,
        )

        self.estimator_status_sub = None
        if self.subscribe_estimator_status and EstimatorStatus is not None:
            self.estimator_status_sub = self.create_subscription(
                EstimatorStatus,
                self.estimator_status_topic,
                self._estimator_status_callback,
                PX4_SENSOR_QOS,
            )
        elif self.subscribe_estimator_status and EstimatorStatus is None:
            self.get_logger().warn(
                "subscribe_estimator_status:=true, but px4_msgs.msg.EstimatorStatus "
                "could not be imported. Continuing without estimator_status."
            )

        self.evaluation_timer = self.create_timer(
            1.0 / max(self.evaluation_rate_hz, 1.0),
            self._evaluation_tick,
        )

        self.diagnostics_timer = self.create_timer(
            1.0 / max(self.diagnostics_rate_hz, 0.5),
            self._publish_diagnostics,
        )

        self.get_logger().info(
            "AI Flight Integrity Observer started | "
            f"setpoint={self.trajectory_setpoint_topic} | "
            f"offboard_mode={self.offboard_control_mode_topic} | "
            f"odometry={self.vehicle_odometry_topic} | "
            f"diagnostics={self.diagnostics_topic}"
        )

    # ============================================================
    # Time
    # ============================================================

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ============================================================
    # Callbacks
    # ============================================================

    def _update_interval(
        self,
        prev_receive_time: Optional[float],
        now: float,
        expected_ms: float,
    ) -> Tuple[float, float]:
        if prev_receive_time is None:
            return 0.0, 0.0

        interval_ms = (now - prev_receive_time) * 1000.0

        if expected_ms > 0.0:
            jitter_ms = abs(interval_ms - expected_ms)
        else:
            jitter_ms = 0.0

        return interval_ms, jitter_ms

    def _trajectory_setpoint_callback(self, msg: TrajectorySetpoint) -> None:
        now = self._now_sec()

        if self.last_setpoint is not None:
            self.prev_setpoint = self.last_setpoint
            self.setpoint_interval_ms, self.setpoint_jitter_ms = self._update_interval(
                self.prev_setpoint.receive_time,
                now,
                self.expected_setpoint_period_ms,
            )

        self.last_setpoint = BufferedSetpoint(
            msg=msg,
            receive_time=now,
            timestamp_sec=px4_timestamp_us_to_sec(getattr(msg, "timestamp", 0)),
        )

        self.stats["setpoint_received"] += 1

    def _offboard_control_mode_callback(self, msg: OffboardControlMode) -> None:
        now = self._now_sec()

        if self.last_offboard_mode is not None:
            self.prev_offboard_mode = self.last_offboard_mode
            (
                self.offboard_mode_interval_ms,
                self.offboard_mode_jitter_ms,
            ) = self._update_interval(
                self.prev_offboard_mode.receive_time,
                now,
                self.expected_offboard_mode_period_ms,
            )

        self.last_offboard_mode = BufferedOffboardMode(
            msg=msg,
            receive_time=now,
            timestamp_sec=px4_timestamp_us_to_sec(getattr(msg, "timestamp", 0)),
        )

        self.stats["offboard_mode_received"] += 1

    def _vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        now = self._now_sec()

        if self.last_odom is not None:
            self.prev_odom = self.last_odom
            self.odometry_interval_ms, self.odometry_jitter_ms = self._update_interval(
                self.prev_odom.receive_time,
                now,
                self.expected_odometry_period_ms,
            )

        self.last_odom = BufferedOdom(
            msg=msg,
            receive_time=now,
            timestamp_sec=px4_timestamp_us_to_sec(getattr(msg, "timestamp", 0)),
        )

        self.stats["odometry_received"] += 1

    def _estimator_status_callback(self, msg: Any) -> None:
        self.last_estimator_status_msg = msg
        self.last_estimator_status_receive_time = self._now_sec()
        self.stats["estimator_status_received"] += 1

    # ============================================================
    # Missing / stale streams
    # ============================================================

    def _stale_data_payload_if_needed(self, now: float) -> Optional[Dict[str, Any]]:
        node_age = now - self.node_start_time

        missing = []
        stale = []

        setpoint_age_sec = -1.0
        offboard_age_sec = -1.0
        odom_age_sec = -1.0

        if self.last_setpoint is None:
            missing.append("trajectory_setpoint")
        else:
            setpoint_age_sec = now - self.last_setpoint.receive_time
            if setpoint_age_sec * 1000.0 >= self.setpoint_stale_warn_ms:
                stale.append("trajectory_setpoint")

        if self.last_offboard_mode is None:
            missing.append("offboard_control_mode")
        else:
            offboard_age_sec = now - self.last_offboard_mode.receive_time
            if offboard_age_sec * 1000.0 >= self.offboard_stale_warn_ms:
                stale.append("offboard_control_mode")

        if self.last_odom is None:
            missing.append("vehicle_odometry")
        else:
            odom_age_sec = now - self.last_odom.receive_time
            if odom_age_sec * 1000.0 >= self.odometry_stale_warn_ms:
                stale.append("vehicle_odometry")

        if not missing and not stale:
            return None

        max_age_sec = max(setpoint_age_sec, offboard_age_sec, odom_age_sec, 0.0)

        if missing:
            if node_age >= self.data_stale_error_sec:
                status = "MISSING_STREAM_TIMEOUT"
                cause = "MISSING_STREAM"
                level_error = True
            else:
                status = "WAITING_FOR_DATA"
                cause = "MISSING_STREAM"
                level_error = False

        else:
            setpoint_error = (
                setpoint_age_sec >= 0.0
                and setpoint_age_sec * 1000.0 >= self.setpoint_stale_error_ms
            )
            offboard_error = (
                offboard_age_sec >= 0.0
                and offboard_age_sec * 1000.0 >= self.offboard_stale_error_ms
            )
            odom_error = (
                odom_age_sec >= 0.0
                and odom_age_sec * 1000.0 >= self.odometry_stale_error_ms
            )

            if setpoint_error:
                status = "RESYNCING"
                cause = "SETPOINT_STALE"
                level_error = True
            elif offboard_error:
                status = "RESYNCING"
                cause = "OFFBOARD_STALE"
                level_error = True
            elif odom_error:
                status = "RESYNCING"
                cause = "ODOMETRY_STALE"
                level_error = True
            else:
                status = "DEGRADED"
                cause = "STALE_STREAM"
                level_error = False

        return self._base_payload(
            now=now,
            status=status,
            dominant_cause=cause,
            total_residual=0.0,
            operator_attention=level_error,
            setpoint_age_sec=setpoint_age_sec,
            offboard_age_sec=offboard_age_sec,
            odom_age_sec=odom_age_sec,
            missing_streams=",".join(missing),
            stale_streams=",".join(stale),
        )

    # ============================================================
    # Evaluation
    # ============================================================

    def _evaluation_tick(self) -> None:
        now = self._now_sec()

        stale_payload = self._stale_data_payload_if_needed(now)
        if stale_payload is not None:
            self._set_last_payload_with_hold(stale_payload)
            return

        if self.last_setpoint is None or self.last_odom is None or self.last_offboard_mode is None:
            self.stats["waiting_count"] += 1
            self._set_last_payload_with_hold(self._waiting_payload())
            return

        try:
            payload = self._evaluate_integrity(now)
        except Exception as exc:
            self.get_logger().error(
                f"Flight integrity evaluation failed: {type(exc).__name__}: {exc}"
            )
            payload = self._evaluation_error_payload(now, exc)

        self._set_last_payload_with_hold(payload)

    def _evaluate_integrity(self, now: float) -> Dict[str, Any]:
        self.stats["evaluations"] += 1

        sp = self.last_setpoint.msg
        odom = self.last_odom.msg
        offboard = self.last_offboard_mode.msg

        setpoint_age_sec = now - self.last_setpoint.receive_time
        offboard_age_sec = now - self.last_offboard_mode.receive_time
        odom_age_sec = now - self.last_odom.receive_time

        offboard_active = any_offboard_axis_enabled(offboard)
        control_semantic_mode = infer_control_semantic_mode(offboard)
        primary_residual_type = primary_residual_type_for_mode(control_semantic_mode)

        sp_position = vec3_from(getattr(sp, "position", [math.nan, math.nan, math.nan]))
        sp_velocity = vec3_from(getattr(sp, "velocity", [math.nan, math.nan, math.nan]))

        odom_position = vec3_from(getattr(odom, "position", [math.nan, math.nan, math.nan]))
        odom_velocity = vec3_from(getattr(odom, "velocity", [math.nan, math.nan, math.nan]))

        velocity_tracking_residual = masked_diff_norm(sp_velocity, odom_velocity)
        position_tracking_residual = masked_diff_norm(sp_position, odom_position)
        previous_position_tracking_residual = self.prev_position_tracking_residual

        gps_vio_jump_metric = self._gps_vio_jump_metric()

        # Age residuals as normalized terms.
        setpoint_age_ms = setpoint_age_sec * 1000.0
        offboard_age_ms = offboard_age_sec * 1000.0
        odom_age_ms = odom_age_sec * 1000.0

        setpoint_age_residual = max(
            0.0,
            setpoint_age_ms / max(self.setpoint_stale_error_ms, 1e-6),
        )
        offboard_age_residual = max(
            0.0,
            offboard_age_ms / max(self.offboard_stale_error_ms, 1e-6),
        )
        odom_age_residual = max(
            0.0,
            odom_age_ms / max(self.odometry_stale_error_ms, 1e-6),
        )
        jitter_residual = max(
            0.0,
            self.setpoint_jitter_ms / max(self.setpoint_jitter_error_ms, 1e-6),
        )
        velocity_residual_norm = max(
            0.0,
            velocity_tracking_residual / max(self.velocity_residual_error_mps, 1e-6),
        )
        position_residual_norm = max(
            0.0,
            position_tracking_residual / max(self.position_residual_error_m, 1e-6),
        )
        jump_residual_norm = max(
            0.0,
            gps_vio_jump_metric / max(self.position_jump_min_m, 1e-6),
        )

        combined_tracking_residual = velocity_residual_norm + position_residual_norm

        if primary_residual_type == "position":
            mode_tracking_residual = position_residual_norm
        elif primary_residual_type == "velocity":
            mode_tracking_residual = velocity_residual_norm
        elif primary_residual_type == "mixed":
            mode_tracking_residual = combined_tracking_residual
        else:
            # Unsupported modes remain conservative but avoid overstating
            # velocity-only intent when PX4 is not in velocity mode.
            mode_tracking_residual = max(position_residual_norm, velocity_residual_norm)

        stream_residual = (
            max(setpoint_age_residual - 1.0, 0.0)
            + max(offboard_age_residual - 1.0, 0.0)
            + max(odom_age_residual - 1.0, 0.0)
        )

        total_residual = (
            combined_tracking_residual
            + jump_residual_norm
            + jitter_residual
            + stream_residual
        )

        flight_residual = (
            mode_tracking_residual
            + jump_residual_norm
            + jitter_residual
            + stream_residual
        )

        # v0.1.2 anti-chattering state:
        # A single noisy frame should not immediately turn normal PX4
        # position-mode convergence into RESYNCING. Count consecutive
        # non-decreasing position residual frames, and reset the count
        # whenever position error clearly decreases, returns below the
        # warning threshold, or the control semantic mode changes.
        trend_for_state, _ = position_residual_trend(
            position_tracking_residual,
            previous_position_tracking_residual,
            deadband_m=self.position_residual_trend_deadband_m,
        )

        if (
            previous_position_tracking_residual is None
            or control_semantic_mode != self.prev_control_semantic_mode
            or position_tracking_residual < self.position_residual_warn_m
            or trend_for_state == TREND_DECREASING
        ):
            self.position_residual_stable_count = 0
        else:
            self.position_residual_stable_count = min(
                self.position_residual_stable_count + 1,
                1_000_000,
            )

        self.prev_control_semantic_mode = control_semantic_mode

        classification = classify_flight_integrity(
            offboard_active=offboard_active,
            control_semantic_mode=control_semantic_mode,
            velocity_tracking_residual=velocity_tracking_residual,
            position_tracking_residual=position_tracking_residual,
            gps_vio_jump_metric=gps_vio_jump_metric,
            setpoint_age_ms=setpoint_age_ms,
            offboard_age_ms=offboard_age_ms,
            odom_age_ms=odom_age_ms,
            setpoint_jitter_ms=self.setpoint_jitter_ms,
            previous_position_tracking_residual=previous_position_tracking_residual,
            position_trend_deadband_m=self.position_residual_trend_deadband_m,
            position_residual_stable_count=self.position_residual_stable_count,
            stable_count_threshold=self.position_residual_stable_count_threshold,
            setpoint_stale_warn_ms=self.setpoint_stale_warn_ms,
            setpoint_stale_error_ms=self.setpoint_stale_error_ms,
            offboard_stale_warn_ms=self.offboard_stale_warn_ms,
            offboard_stale_error_ms=self.offboard_stale_error_ms,
            odometry_stale_warn_ms=self.odometry_stale_warn_ms,
            odometry_stale_error_ms=self.odometry_stale_error_ms,
            setpoint_jitter_warn_ms=self.setpoint_jitter_warn_ms,
            setpoint_jitter_error_ms=self.setpoint_jitter_error_ms,
            velocity_residual_warn_mps=self.velocity_residual_warn_mps,
            velocity_residual_error_mps=self.velocity_residual_error_mps,
            critical_velocity_residual_mps=self.critical_velocity_residual_mps,
            position_residual_warn_m=self.position_residual_warn_m,
            position_residual_error_m=self.position_residual_error_m,
        )

        self.prev_position_tracking_residual = position_tracking_residual

        status = classification.status
        cause = classification.dominant_cause
        candidate = classification.dominant_cause_candidate

        return self._base_payload(
            now=now,
            status=status,
            dominant_cause=cause,
            dominant_cause_candidate=candidate,
            total_residual=total_residual,
            flight_residual=flight_residual,
            operator_attention=(status == "RESYNCING"),
            setpoint_age_sec=setpoint_age_sec,
            offboard_age_sec=offboard_age_sec,
            odom_age_sec=odom_age_sec,
            missing_streams="",
            stale_streams="",
            offboard_active=offboard_active,
            setpoint_velocity=sp_velocity,
            vehicle_velocity=odom_velocity,
            setpoint_position=sp_position,
            vehicle_position=odom_position,
            velocity_tracking_residual=velocity_tracking_residual,
            position_tracking_residual=position_tracking_residual,
            gps_vio_jump_metric=gps_vio_jump_metric,
            control_semantic_mode=classification.control_semantic_mode,
            primary_residual_type=classification.primary_residual_type,
            position_residual_trend=classification.position_residual_trend,
            position_residual_delta=classification.position_residual_delta,
            tracking_transient=classification.tracking_transient,
            position_residual_stable_count=classification.position_residual_stable_count,
            critical_velocity_residual_mps=self.critical_velocity_residual_mps,
        )

    def _gps_vio_jump_metric(self) -> float:
        if self.prev_odom is None or self.last_odom is None:
            return 0.0

        prev_msg = self.prev_odom.msg
        curr_msg = self.last_odom.msg

        prev_pos = vec3_from(getattr(prev_msg, "position", [math.nan, math.nan, math.nan]))
        curr_pos = vec3_from(getattr(curr_msg, "position", [math.nan, math.nan, math.nan]))
        curr_vel = vec3_from(getattr(curr_msg, "velocity", [0.0, 0.0, 0.0]))

        step = masked_diff_norm(curr_pos, prev_pos)

        dt = max(
            1e-4,
            self.last_odom.receive_time - self.prev_odom.receive_time,
        )

        expected_step = vec_norm(curr_vel) * dt + self.position_jump_margin_m

        threshold = max(
            self.position_jump_min_m,
            expected_step * self.position_jump_ratio,
        )

        return max(0.0, step - threshold)

    def _classify_status_and_cause(
        self,
        offboard_active: bool,
        velocity_tracking_residual: float,
        position_tracking_residual: float,
        gps_vio_jump_metric: float,
        setpoint_age_ms: float,
        offboard_age_ms: float,
        odom_age_ms: float,
        setpoint_jitter_ms: float,
    ) -> Tuple[str, str, str]:
        if not offboard_active:
            return "DEGRADED", "OFFBOARD_INACTIVE", "OFFBOARD_INACTIVE"

        # ERROR priority.
        if setpoint_age_ms >= self.setpoint_stale_error_ms:
            return "RESYNCING", "SETPOINT_STALE", "SETPOINT_STALE"

        if offboard_age_ms >= self.offboard_stale_error_ms:
            return "RESYNCING", "OFFBOARD_STALE", "OFFBOARD_STALE"

        if odom_age_ms >= self.odometry_stale_error_ms:
            return "RESYNCING", "ODOMETRY_STALE", "ODOMETRY_STALE"

        if setpoint_jitter_ms >= self.setpoint_jitter_error_ms:
            return "RESYNCING", "SETPOINT_JITTER", "SETPOINT_JITTER"

        if gps_vio_jump_metric > 0.0:
            return "RESYNCING", "GPS_VIO_JUMP", "GPS_VIO_JUMP"

        if velocity_tracking_residual >= self.velocity_residual_error_mps:
            return "RESYNCING", "COMMAND_RESPONSE_MISMATCH", "COMMAND_RESPONSE_MISMATCH"

        if position_tracking_residual >= self.position_residual_error_m:
            return "RESYNCING", "POSITION_RESPONSE_MISMATCH", "POSITION_RESPONSE_MISMATCH"

        # WARN priority.
        if setpoint_age_ms >= self.setpoint_stale_warn_ms:
            return "DEGRADED", "SETPOINT_STALE", "SETPOINT_STALE"

        if offboard_age_ms >= self.offboard_stale_warn_ms:
            return "DEGRADED", "OFFBOARD_STALE", "OFFBOARD_STALE"

        if odom_age_ms >= self.odometry_stale_warn_ms:
            return "DEGRADED", "ODOMETRY_STALE", "ODOMETRY_STALE"

        if setpoint_jitter_ms >= self.setpoint_jitter_warn_ms:
            return "DEGRADED", "SETPOINT_JITTER", "SETPOINT_JITTER"

        if velocity_tracking_residual >= self.velocity_residual_warn_mps:
            return "DEGRADED", "COMMAND_RESPONSE_MISMATCH", "COMMAND_RESPONSE_MISMATCH"

        if position_tracking_residual >= self.position_residual_warn_m:
            return "DEGRADED", "POSITION_RESPONSE_MISMATCH", "POSITION_RESPONSE_MISMATCH"

        return "GREEN", "NONE", "NONE"

    # ============================================================
    # Payload
    # ============================================================

    def _base_payload(
        self,
        now: float,
        status: str,
        dominant_cause: str,
        total_residual: float,
        operator_attention: bool,
        setpoint_age_sec: float,
        offboard_age_sec: float,
        odom_age_sec: float,
        missing_streams: str,
        stale_streams: str,
        flight_residual: Optional[float] = None,
        dominant_cause_candidate: Optional[str] = None,
        offboard_active: bool = False,
        setpoint_velocity: Optional[List[float]] = None,
        vehicle_velocity: Optional[List[float]] = None,
        setpoint_position: Optional[List[float]] = None,
        vehicle_position: Optional[List[float]] = None,
        velocity_tracking_residual: float = 0.0,
        position_tracking_residual: float = 0.0,
        gps_vio_jump_metric: float = 0.0,
        control_semantic_mode: str = "UNKNOWN_MODE",
        primary_residual_type: str = "unknown",
        position_residual_trend: str = "unknown",
        position_residual_delta: float = 0.0,
        tracking_transient: bool = False,
        position_residual_stable_count: int = 0,
        critical_velocity_residual_mps: Optional[float] = None,
    ) -> Dict[str, Any]:
        if dominant_cause_candidate is None:
            dominant_cause_candidate = dominant_cause

        level = self._diagnostic_level(status)

        return {
            "timestamp": now,
            "status": status,
            "engineStatusRaw": status,
            "dominantCause": dominant_cause,
            "dominantCauseCandidate": dominant_cause_candidate,
            "causalAlignment": self._causal_alignment(status),
            "mode": "observe",
            "operatorAttentionRequired": bool(operator_attention or level == DIAG_ERROR),

            "totalResidual": finite_or(total_residual),
            "flightResidual": finite_or(total_residual if flight_residual is None else flight_residual),

            "offboardActive": bool(offboard_active),
            "controlSemanticMode": control_semantic_mode,
            "primaryResidualType": primary_residual_type,
            "positionResidualTrend": position_residual_trend,
            "positionResidualDelta": finite_or(position_residual_delta),
            "positionResidualStableCount": int(max(position_residual_stable_count, 0)),
            "positionResidualStableCountThreshold": int(
                max(self.position_residual_stable_count_threshold, 1)
            ),
            "criticalVelocityResidualMps": finite_or(
                self.critical_velocity_residual_mps
                if critical_velocity_residual_mps is None
                else critical_velocity_residual_mps
            ),
            "trackingTransient": bool(tracking_transient),

            "setpointAgeMs": finite_or(setpoint_age_sec * 1000.0, -1.0),
            "offboardModeAgeMs": finite_or(offboard_age_sec * 1000.0, -1.0),
            "odometryAgeMs": finite_or(odom_age_sec * 1000.0, -1.0),

            "setpointIntervalMs": self.setpoint_interval_ms,
            "setpointJitterMs": self.setpoint_jitter_ms,
            "odometryIntervalMs": self.odometry_interval_ms,
            "odometryJitterMs": self.odometry_jitter_ms,
            "offboardModeIntervalMs": self.offboard_mode_interval_ms,
            "offboardModeJitterMs": self.offboard_mode_jitter_ms,

            "velocityTrackingResidual": finite_or(velocity_tracking_residual),
            "positionTrackingResidual": finite_or(position_tracking_residual),
            "gpsVioJumpMetric": finite_or(gps_vio_jump_metric),

            "setpointVelocity": setpoint_velocity or [0.0, 0.0, 0.0],
            "vehicleVelocity": vehicle_velocity or [0.0, 0.0, 0.0],
            "setpointPosition": setpoint_position or [0.0, 0.0, 0.0],
            "vehiclePosition": vehicle_position or [0.0, 0.0, 0.0],

            "missingStreams": missing_streams,
            "staleStreams": stale_streams,
            "nodeAgeSec": now - self.node_start_time,

            "trajectorySetpointTopic": self.trajectory_setpoint_topic,
            "offboardControlModeTopic": self.offboard_control_mode_topic,
            "vehicleOdometryTopic": self.vehicle_odometry_topic,
            "estimatorStatusTopic": self.estimator_status_topic,
            "estimatorStatusEnabled": self.subscribe_estimator_status,

            "loadProfile": self.load_profile,
            "cpuLoadPercent": self.cpu_load_percent,
            "gpuLoadPercent": self.gpu_load_percent,
            "npuLoadPercent": self.npu_load_percent,
            "aiInferenceLatencyMs": self.ai_inference_latency_ms,

            "statsSetpointReceived": int(self.stats["setpoint_received"]),
            "statsOffboardModeReceived": int(self.stats["offboard_mode_received"]),
            "statsOdometryReceived": int(self.stats["odometry_received"]),
            "statsEstimatorStatusReceived": int(self.stats["estimator_status_received"]),
            "statsEvaluations": int(self.stats["evaluations"]),
        }

    def _waiting_payload(self) -> Dict[str, Any]:
        now = self._now_sec()
        return self._base_payload(
            now=now,
            status="WAITING_FOR_DATA",
            dominant_cause="WAITING_FOR_DATA",
            total_residual=0.0,
            operator_attention=False,
            setpoint_age_sec=-1.0,
            offboard_age_sec=-1.0,
            odom_age_sec=-1.0,
            missing_streams="",
            stale_streams="",
        )

    def _evaluation_error_payload(self, now: float, exc: Exception) -> Dict[str, Any]:
        payload = self._base_payload(
            now=now,
            status="EVALUATION_ERROR",
            dominant_cause="EVALUATION_EXCEPTION",
            total_residual=0.0,
            operator_attention=True,
            setpoint_age_sec=-1.0,
            offboard_age_sec=-1.0,
            odom_age_sec=-1.0,
            missing_streams="",
            stale_streams="",
        )

        payload["exceptionType"] = type(exc).__name__
        payload["exceptionMessage"] = str(exc)
        return payload

    def _set_last_payload_with_hold(self, payload: Dict[str, Any]) -> None:
        now = self._now_sec()
        level = self._diagnostic_level(str(payload.get("status", "")))

        # Hold short-lived non-stream ERROR events so Foxglove/rqt users
        # can actually see hard jump events.
        cause = str(payload.get("dominantCause", ""))

        holdable = level == DIAG_ERROR and cause not in {
            "MISSING_STREAM",
            "STALE_STREAM",
            "SETPOINT_STALE",
            "OFFBOARD_STALE",
            "ODOMETRY_STALE",
        }

        if holdable and self.diagnostic_event_hold_sec > 0.0:
            self.last_error_payload = dict(payload)
            self.last_error_hold_until = now + self.diagnostic_event_hold_sec
            self.last_payload = payload
            return

        if (
            level == DIAG_OK
            and self.last_error_payload is not None
            and now < self.last_error_hold_until
        ):
            held = dict(self.last_error_payload)
            held["heldEvent"] = True
            held["heldUntilSec"] = self.last_error_hold_until
            self.last_payload = held
            return

        self.last_payload = payload

    # ============================================================
    # Diagnostics
    # ============================================================

    def _causal_alignment(self, status: str) -> str:
        if status == "GREEN":
            return "ALIGNED"
        if status == "DEGRADED":
            return "DEGRADED"
        if status in {"RESYNCING", "EVALUATION_ERROR", "MISSING_STREAM_TIMEOUT"}:
            return "BROKEN"
        return "UNKNOWN"

    def _diagnostic_level(self, status: str) -> int:
        if status == "GREEN":
            return DIAG_OK

        if status in {
            "DEGRADED",
            "WAITING_FOR_DATA",
            "STALE_DATA",
        }:
            return DIAG_WARN

        if status in {
            "RESYNCING",
            "EVALUATION_ERROR",
            "MISSING_STREAM_TIMEOUT",
            "STALE_DATA_TIMEOUT",
        }:
            return DIAG_ERROR

        return DIAG_WARN

    def _diagnostic_level_name(self, level: int) -> str:
        level = int(level)

        if level == DIAG_OK:
            return "OK"
        if level == DIAG_WARN:
            return "WARN"
        if level == DIAG_ERROR:
            return "ERROR"
        if level == DIAG_STALE:
            return "STALE"

        return f"UNKNOWN_{level}"

    def _publish_diagnostics(self) -> None:
        payload = self.last_payload

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        st = DiagnosticStatus()
        st.name = self.diagnostic_name
        st.hardware_id = self.hardware_id

        status = str(payload.get("status", "UNKNOWN"))
        cause = str(payload.get("dominantCause", "NONE"))

        level = self._diagnostic_level(status)
        level_name = self._diagnostic_level_name(level)

        set_diagnostic_level(st, level)

        if cause in {"NONE", ""}:
            st.message = f"{level_name} | {status}: FLIGHT_ALIGNED"
        else:
            st.message = f"{level_name} | {status}: {cause}"

        st.values = [
            key_value("diagnosticLevelInt", int(level)),
            key_value("diagnosticLevelName", level_name),

            key_value("status", payload.get("status", "")),
            key_value("engineStatusRaw", payload.get("engineStatusRaw", "")),
            key_value("dominantCause", payload.get("dominantCause", "")),
            key_value("dominantCauseCandidate", payload.get("dominantCauseCandidate", "")),
            key_value("causalAlignment", payload.get("causalAlignment", "")),
            key_value("mode", payload.get("mode", "observe")),
            key_value(
                "operatorAttentionRequired",
                str(bool(payload.get("operatorAttentionRequired", False))).lower(),
            ),

            key_value("totalResidual", f"{finite_or(payload.get('totalResidual', 0.0)):.6f}"),
            key_value("flightResidual", f"{finite_or(payload.get('flightResidual', 0.0)):.6f}"),

            key_value("offboardActive", str(bool(payload.get("offboardActive", False))).lower()),
            key_value("controlSemanticMode", payload.get("controlSemanticMode", "UNKNOWN_MODE")),
            key_value("primaryResidualType", payload.get("primaryResidualType", "unknown")),
            key_value("positionResidualTrend", payload.get("positionResidualTrend", "unknown")),
            key_value("positionResidualDelta", f"{finite_or(payload.get('positionResidualDelta', 0.0)):.6f}"),
            key_value("positionResidualStableCount", payload.get("positionResidualStableCount", 0)),
            key_value(
                "positionResidualStableCountThreshold",
                payload.get("positionResidualStableCountThreshold", self.position_residual_stable_count_threshold),
            ),
            key_value(
                "criticalVelocityResidualMps",
                f"{finite_or(payload.get('criticalVelocityResidualMps', self.critical_velocity_residual_mps)):.2f}",
            ),
            key_value("trackingTransient", str(bool(payload.get("trackingTransient", False))).lower()),

            key_value("setpointAgeMs", f"{finite_or(payload.get('setpointAgeMs', -1.0)):.2f}"),
            key_value("offboardModeAgeMs", f"{finite_or(payload.get('offboardModeAgeMs', -1.0)):.2f}"),
            key_value("odometryAgeMs", f"{finite_or(payload.get('odometryAgeMs', -1.0)):.2f}"),

            key_value("setpointIntervalMs", f"{finite_or(payload.get('setpointIntervalMs', 0.0)):.2f}"),
            key_value("setpointJitterMs", f"{finite_or(payload.get('setpointJitterMs', 0.0)):.2f}"),
            key_value("odometryIntervalMs", f"{finite_or(payload.get('odometryIntervalMs', 0.0)):.2f}"),
            key_value("odometryJitterMs", f"{finite_or(payload.get('odometryJitterMs', 0.0)):.2f}"),
            key_value("offboardModeIntervalMs", f"{finite_or(payload.get('offboardModeIntervalMs', 0.0)):.2f}"),
            key_value("offboardModeJitterMs", f"{finite_or(payload.get('offboardModeJitterMs', 0.0)):.2f}"),

            key_value("velocityTrackingResidual", f"{finite_or(payload.get('velocityTrackingResidual', 0.0)):.6f}"),
            key_value("positionTrackingResidual", f"{finite_or(payload.get('positionTrackingResidual', 0.0)):.6f}"),
            key_value("gpsVioJumpMetric", f"{finite_or(payload.get('gpsVioJumpMetric', 0.0)):.6f}"),

            key_value("setpointVelocity", jsonish(payload.get("setpointVelocity", [0.0, 0.0, 0.0]))),
            key_value("vehicleVelocity", jsonish(payload.get("vehicleVelocity", [0.0, 0.0, 0.0]))),
            key_value("setpointPosition", jsonish(payload.get("setpointPosition", [0.0, 0.0, 0.0]))),
            key_value("vehiclePosition", jsonish(payload.get("vehiclePosition", [0.0, 0.0, 0.0]))),

            key_value("missingStreams", payload.get("missingStreams", "")),
            key_value("staleStreams", payload.get("staleStreams", "")),
            key_value("nodeAgeSec", f"{finite_or(payload.get('nodeAgeSec', 0.0)):.3f}"),

            key_value("trajectorySetpointTopic", payload.get("trajectorySetpointTopic", self.trajectory_setpoint_topic)),
            key_value("offboardControlModeTopic", payload.get("offboardControlModeTopic", self.offboard_control_mode_topic)),
            key_value("vehicleOdometryTopic", payload.get("vehicleOdometryTopic", self.vehicle_odometry_topic)),
            key_value("estimatorStatusTopic", payload.get("estimatorStatusTopic", self.estimator_status_topic)),
            key_value("estimatorStatusEnabled", str(bool(payload.get("estimatorStatusEnabled", False))).lower()),

            key_value("loadProfile", payload.get("loadProfile", self.load_profile)),
            key_value("cpuLoadPercent", f"{finite_or(payload.get('cpuLoadPercent', 0.0)):.2f}"),
            key_value("gpuLoadPercent", f"{finite_or(payload.get('gpuLoadPercent', 0.0)):.2f}"),
            key_value("npuLoadPercent", f"{finite_or(payload.get('npuLoadPercent', 0.0)):.2f}"),
            key_value("aiInferenceLatencyMs", f"{finite_or(payload.get('aiInferenceLatencyMs', 0.0)):.2f}"),

            key_value("heldEvent", str(bool(payload.get("heldEvent", False))).lower()),

            key_value("statsSetpointReceived", payload.get("statsSetpointReceived", 0)),
            key_value("statsOffboardModeReceived", payload.get("statsOffboardModeReceived", 0)),
            key_value("statsOdometryReceived", payload.get("statsOdometryReceived", 0)),
            key_value("statsEstimatorStatusReceived", payload.get("statsEstimatorStatusReceived", 0)),
            key_value("statsEvaluations", payload.get("statsEvaluations", 0)),

            key_value("exceptionType", payload.get("exceptionType", "")),
            key_value("exceptionMessage", payload.get("exceptionMessage", "")),
        ]

        msg.status.append(st)
        self.diagnostics_pub.publish(msg)
        self.stats["diagnostics_published"] += 1


def jsonish(value: Any) -> str:
    try:
        if isinstance(value, list):
            return "[" + ",".join(f"{finite_or(v):.4f}" for v in value) + "]"
        return str(value)
    except Exception:
        return str(value)


# ============================================================
# Main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightIntegrityNode()

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
