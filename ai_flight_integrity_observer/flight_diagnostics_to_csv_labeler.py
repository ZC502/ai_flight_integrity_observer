#!/usr/bin/env python3
"""
flight_diagnostics_to_csv_labeler.py

AI Flight Integrity Observer CSV labeler.

Listens to /diagnostics and exports flight execution-integrity events
into a machine-readable CSV file for:

- AI load vs flight integrity analysis
- Sim2Real failure mining
- offboard setpoint failure datasets
- OOD event detection
- regression testing under companion-compute load
- post-flight incident review

v0.1.2+ gated schema notes:
The labeler exposes mode-aware / gated-fuse fields as first-class CSV columns
instead of hiding them only inside extraValuesJson:

- controlSemanticMode
- primaryResidualType
- positionResidualTrend
- positionResidualDelta
- positionResidualStableCount
- positionResidualStableCountThreshold
- trackingTransient
- criticalVelocityResidualMps

Typical use:

    ros2 run ai_flight_integrity_observer flight_diagnostics_to_csv_labeler --ros-args \
      -p output_csv:=flight_integrity_labels.csv
"""

from __future__ import annotations

import csv
import json
import os
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray


# ============================================================
# Helpers
# ============================================================

def level_to_int(level: Any) -> int:
    """
    Normalize DiagnosticStatus.level across ROS 2 Python variants.

    In ROS 2 Humble, DiagnosticStatus.level may appear as bytes such as b"\\x02".
    In other contexts it may be an int.
    """
    if isinstance(level, (bytes, bytearray)):
        if len(level) == 0:
            return 0
        return int(level[0])

    try:
        return int(level)
    except Exception:
        return 0


def stamp_to_sec(stamp: Any) -> float:
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except Exception:
        return 0.0


def kv_to_dict(values: Any) -> Dict[str, str]:
    out: Dict[str, str] = {}
    try:
        for kv in values:
            out[str(kv.key)] = str(kv.value)
    except Exception:
        pass
    return out


def as_float(value: Optional[str], default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def as_int(value: Optional[str], default: int = 0) -> int:
    try:
        return int(float(value))
    except Exception:
        return default


def as_str(value: Optional[str], default: str = "") -> str:
    if value is None:
        return default
    return str(value)


# ============================================================
# Node
# ============================================================

class FlightDiagnosticsToCsvLabeler(Node):
    def __init__(self) -> None:
        super().__init__("flight_diagnostics_to_csv_labeler")

        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter(
            "target_name",
            "ai_flight_integrity/flight_execution_integrity",
        )
        self.declare_parameter("output_csv", "flight_integrity_labels.csv")

        # If false, only WARN / ERROR rows are exported by default.
        self.declare_parameter("include_ok", False)

        # 0 = OK, 1 = WARN, 2 = ERROR.
        self.declare_parameter("min_level", 1)

        # Keep stream-health rows such as SETPOINT_STALE / ODOMETRY_STALE.
        # Set false if you only want physical response mismatch labels.
        self.declare_parameter("include_stream_health", True)

        # Flush after every row. Slower, but safer for field tests.
        self.declare_parameter("flush_every_row", True)

        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.target_name = str(self.get_parameter("target_name").value)
        self.output_csv = str(self.get_parameter("output_csv").value)
        self.include_ok = bool(self.get_parameter("include_ok").value)
        self.min_level = int(self.get_parameter("min_level").value)
        self.include_stream_health = bool(
            self.get_parameter("include_stream_health").value
        )
        self.flush_every_row = bool(self.get_parameter("flush_every_row").value)

        self.rows_written = 0

        # Keep this schema stable and ML-friendly.
        self.fieldnames = [
            # Time
            "ros_time_sec",
            "wall_time_sec",

            # Diagnostic identity
            "diagnostic_name",
            "diagnostic_level_int",
            "diagnostic_level_name",
            "message",

            # Public state
            "status",
            "engineStatusRaw",
            "dominantCause",
            "dominantCauseCandidate",
            "causalAlignment",
            "operatorAttentionRequired",
            "offboardActive",

            # v0.1.2+ mode-aware / gated-fuse semantics
            "controlSemanticMode",
            "primaryResidualType",
            "positionResidualTrend",
            "positionResidualDelta",
            "positionResidualStableCount",
            "positionResidualStableCountThreshold",
            "trackingTransient",
            "criticalVelocityResidualMps",

            # Core residuals
            "totalResidual",
            "flightResidual",
            "velocityTrackingResidual",
            "positionTrackingResidual",
            "gpsVioJumpMetric",

            # Timing / freshness
            "setpointAgeMs",
            "offboardModeAgeMs",
            "odometryAgeMs",
            "setpointIntervalMs",
            "setpointJitterMs",
            "odometryIntervalMs",
            "odometryJitterMs",
            "offboardModeIntervalMs",
            "offboardModeJitterMs",

            # Vectors as stringified arrays
            "setpointVelocity",
            "vehicleVelocity",
            "setpointPosition",
            "vehiclePosition",

            # Stream state
            "missingStreams",
            "staleStreams",
            "nodeAgeSec",

            # Topic identity
            "trajectorySetpointTopic",
            "offboardControlModeTopic",
            "vehicleOdometryTopic",
            "estimatorStatusTopic",
            "estimatorStatusEnabled",

            # AI / platform load metadata
            "loadProfile",
            "cpuLoadPercent",
            "gpuLoadPercent",
            "npuLoadPercent",
            "aiInferenceLatencyMs",

            # Event hold
            "heldEvent",

            # Stats
            "statsSetpointReceived",
            "statsOffboardModeReceived",
            "statsOdometryReceived",
            "statsEstimatorStatusReceived",
            "statsEvaluations",

            # Exception fields
            "exceptionType",
            "exceptionMessage",

            # Preserve all additional diagnostic KeyValue fields.
            "extraValuesJson",
        ]

        output_dir = os.path.dirname(os.path.abspath(self.output_csv))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        self.csv_file = open(self.output_csv, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=self.fieldnames,
            extrasaction="ignore",
        )
        self.writer.writeheader()
        self.csv_file.flush()

        self.sub = self.create_subscription(
            DiagnosticArray,
            self.diagnostics_topic,
            self._diagnostics_callback,
            10,
        )

        self.get_logger().info(
            "AI Flight Integrity CSV labeler started | "
            f"diagnostics={self.diagnostics_topic} | "
            f"target={self.target_name} | "
            f"output={self.output_csv} | "
            f"include_ok={self.include_ok} | "
            f"min_level={self.min_level}"
        )

    # ============================================================
    # Filtering
    # ============================================================

    def _should_record(self, level_int: int, kv: Dict[str, str]) -> bool:
        if self.include_ok:
            return True

        if level_int < self.min_level:
            return False

        if self.include_stream_health:
            return True

        cause = kv.get("dominantCause", "")
        status = kv.get("status", "")

        stream_health_causes = {
            "MISSING_STREAM",
            "SETPOINT_STALE",
            "OFFBOARD_STALE",
            "ODOMETRY_STALE",
            "STALE_STREAM",
            "WAITING_FOR_DATA",
        }

        stream_health_statuses = {
            "MISSING_STREAM_TIMEOUT",
            "WAITING_FOR_DATA",
            "STALE_DATA",
            "STALE_DATA_TIMEOUT",
        }

        if cause in stream_health_causes:
            return False

        if status in stream_health_statuses:
            return False

        return True

    # ============================================================
    # Callback
    # ============================================================

    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        ros_time_sec = stamp_to_sec(msg.header.stamp)
        wall_time_sec = self.get_clock().now().nanoseconds * 1e-9

        for status_msg in msg.status:
            if status_msg.name != self.target_name:
                continue

            level_int = level_to_int(status_msg.level)
            kv = kv_to_dict(status_msg.values)

            if not self._should_record(level_int, kv):
                continue

            row = self._build_row(
                ros_time_sec=ros_time_sec,
                wall_time_sec=wall_time_sec,
                level_int=level_int,
                message=status_msg.message,
                diagnostic_name=status_msg.name,
                kv=kv,
            )

            self.writer.writerow(row)
            self.rows_written += 1

            if self.flush_every_row:
                self.csv_file.flush()

            if self.rows_written % 10 == 0:
                self.get_logger().info(
                    f"CSV labeler wrote {self.rows_written} rows -> {self.output_csv}"
                )

    def _build_row(
        self,
        ros_time_sec: float,
        wall_time_sec: float,
        level_int: int,
        message: str,
        diagnostic_name: str,
        kv: Dict[str, str],
    ) -> Dict[str, Any]:
        row: Dict[str, Any] = {
            # Time
            "ros_time_sec": f"{ros_time_sec:.9f}",
            "wall_time_sec": f"{wall_time_sec:.9f}",

            # Diagnostic identity
            "diagnostic_name": diagnostic_name,
            "diagnostic_level_int": level_int,
            "diagnostic_level_name": kv.get("diagnosticLevelName", ""),
            "message": message,

            # Public state
            "status": kv.get("status", ""),
            "engineStatusRaw": kv.get("engineStatusRaw", ""),
            "dominantCause": kv.get("dominantCause", ""),
            "dominantCauseCandidate": kv.get("dominantCauseCandidate", ""),
            "causalAlignment": kv.get("causalAlignment", ""),
            "operatorAttentionRequired": kv.get("operatorAttentionRequired", ""),
            "offboardActive": kv.get("offboardActive", ""),

            # v0.1.2+ mode-aware / gated-fuse semantics
            "controlSemanticMode": kv.get("controlSemanticMode", ""),
            "primaryResidualType": kv.get("primaryResidualType", ""),
            "positionResidualTrend": kv.get("positionResidualTrend", ""),
            "positionResidualDelta": as_float(kv.get("positionResidualDelta")),
            "positionResidualStableCount": as_int(kv.get("positionResidualStableCount")),
            "positionResidualStableCountThreshold": as_int(
                kv.get("positionResidualStableCountThreshold")
            ),
            "trackingTransient": kv.get("trackingTransient", ""),
            "criticalVelocityResidualMps": as_float(
                kv.get("criticalVelocityResidualMps")
            ),

            # Core residuals
            "totalResidual": as_float(kv.get("totalResidual")),
            "flightResidual": as_float(kv.get("flightResidual")),
            "velocityTrackingResidual": as_float(kv.get("velocityTrackingResidual")),
            "positionTrackingResidual": as_float(kv.get("positionTrackingResidual")),
            "gpsVioJumpMetric": as_float(kv.get("gpsVioJumpMetric")),

            # Timing / freshness
            "setpointAgeMs": as_float(kv.get("setpointAgeMs")),
            "offboardModeAgeMs": as_float(kv.get("offboardModeAgeMs")),
            "odometryAgeMs": as_float(kv.get("odometryAgeMs")),
            "setpointIntervalMs": as_float(kv.get("setpointIntervalMs")),
            "setpointJitterMs": as_float(kv.get("setpointJitterMs")),
            "odometryIntervalMs": as_float(kv.get("odometryIntervalMs")),
            "odometryJitterMs": as_float(kv.get("odometryJitterMs")),
            "offboardModeIntervalMs": as_float(kv.get("offboardModeIntervalMs")),
            "offboardModeJitterMs": as_float(kv.get("offboardModeJitterMs")),

            # Vectors
            "setpointVelocity": as_str(kv.get("setpointVelocity")),
            "vehicleVelocity": as_str(kv.get("vehicleVelocity")),
            "setpointPosition": as_str(kv.get("setpointPosition")),
            "vehiclePosition": as_str(kv.get("vehiclePosition")),

            # Stream state
            "missingStreams": kv.get("missingStreams", ""),
            "staleStreams": kv.get("staleStreams", ""),
            "nodeAgeSec": as_float(kv.get("nodeAgeSec")),

            # Topic identity
            "trajectorySetpointTopic": kv.get("trajectorySetpointTopic", ""),
            "offboardControlModeTopic": kv.get("offboardControlModeTopic", ""),
            "vehicleOdometryTopic": kv.get("vehicleOdometryTopic", ""),
            "estimatorStatusTopic": kv.get("estimatorStatusTopic", ""),
            "estimatorStatusEnabled": kv.get("estimatorStatusEnabled", ""),

            # AI / platform load metadata
            "loadProfile": kv.get("loadProfile", ""),
            "cpuLoadPercent": as_float(kv.get("cpuLoadPercent")),
            "gpuLoadPercent": as_float(kv.get("gpuLoadPercent")),
            "npuLoadPercent": as_float(kv.get("npuLoadPercent")),
            "aiInferenceLatencyMs": as_float(kv.get("aiInferenceLatencyMs")),

            # Event hold
            "heldEvent": kv.get("heldEvent", ""),

            # Stats
            "statsSetpointReceived": kv.get("statsSetpointReceived", ""),
            "statsOffboardModeReceived": kv.get("statsOffboardModeReceived", ""),
            "statsOdometryReceived": kv.get("statsOdometryReceived", ""),
            "statsEstimatorStatusReceived": kv.get("statsEstimatorStatusReceived", ""),
            "statsEvaluations": kv.get("statsEvaluations", ""),

            # Exception fields
            "exceptionType": kv.get("exceptionType", ""),
            "exceptionMessage": kv.get("exceptionMessage", ""),

            # Preserve all original diagnostic KV values.
            "extraValuesJson": json.dumps(kv, ensure_ascii=False, separators=(",", ":")),
        }

        return row

    # ============================================================
    # Cleanup
    # ============================================================

    def destroy_node(self) -> bool:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass

        return super().destroy_node()


# ============================================================
# Main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightDiagnosticsToCsvLabeler()

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
