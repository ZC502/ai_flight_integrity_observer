#!/usr/bin/env python3
"""
synthetic_px4_publisher.py

Synthetic PX4-style publisher for AI Flight Integrity Observer v0.1.2+.

No PX4 SITL required.
No Gazebo required.
No real UAV required.

It publishes synthetic PX4 ROS 2 topics:

    /fmu/in/offboard_control_mode
    /fmu/in/trajectory_setpoint
    /fmu/out/vehicle_odometry

Fault / validation profiles:

    normal
    setpoint_jitter
    command_response_mismatch
    gps_vio_jump

v0.1.2+ gated-fuse validation profiles:

    mixed_step_transient
        Mixed position+velocity offboard mode.
        Injects a large position step and a large feed-forward velocity phase
        mismatch while the position residual is still converging.
        Expected observer behavior:
            DEGRADED | POSITION_TRACKING_TRANSIENT
        It should NOT immediately trip CRITICAL_COMMAND_RESPONSE_MISMATCH.

    mixed_critical_stall
        Mixed position+velocity offboard mode.
        Injects a large position step and keeps the vehicle physically stuck
        while velocity residual remains above the critical ceiling.
        Expected observer behavior:
            First short transient window, then
            ERROR | RESYNCING: CRITICAL_COMMAND_RESPONSE_MISMATCH

This is a smoke-test and fault-injection tool. It validates topic wiring, QoS,
diagnostics, CSV label export, mode-aware classification, and gated critical
velocity fuse logic before connecting to real PX4 SITL.
"""

from __future__ import annotations

import json
import math
from typing import Any, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleOdometry,
)

try:
    from .px4_qos import PX4_SENSOR_QOS
except ImportError:
    from px4_qos import PX4_SENSOR_QOS


# ============================================================
# Helpers
# ============================================================

def finite_or(x: Any, fallback: float = 0.0) -> float:
    try:
        value = float(x)
        return value if math.isfinite(value) else fallback
    except Exception:
        return fallback


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def safe_set(msg: Any, field: str, value: Any) -> None:
    if hasattr(msg, field):
        try:
            setattr(msg, field, value)
        except Exception:
            pass


def safe_set_array(msg: Any, field: str, values: List[float]) -> None:
    if hasattr(msg, field):
        try:
            setattr(msg, field, [float(v) for v in values])
        except Exception:
            pass


# ============================================================
# Node
# ============================================================

class SyntheticPx4Publisher(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_px4_publisher")

        # --------------------------------------------------------
        # Topics
        # --------------------------------------------------------
        self.declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("vehicle_odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("status_topic", "/synthetic_px4/status")

        # --------------------------------------------------------
        # Simulation
        # --------------------------------------------------------
        self.declare_parameter("profile", "normal")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("fault_start_sec", 3.0)
        self.declare_parameter("fault_duration_sec", 8.0)

        # Offboard semantic mode for generic profiles:
        #   velocity | position | mixed
        # Mixed validation profiles force mixed mode automatically.
        self.declare_parameter("offboard_mode", "velocity")

        # Nominal setpoint velocity in NED frame.
        self.declare_parameter("vx_sp", 1.0)
        self.declare_parameter("vy_sp", 0.0)
        self.declare_parameter("vz_sp", 0.0)

        # Start position in NED frame.
        self.declare_parameter("x0", 0.0)
        self.declare_parameter("y0", 0.0)
        self.declare_parameter("z0", -5.0)

        # command_response_mismatch profile
        self.declare_parameter("response_ratio", 0.10)

        # setpoint_jitter profile
        self.declare_parameter("jitter_pause_sec", 0.35)
        self.declare_parameter("jitter_period_sec", 1.50)

        # gps_vio_jump profile
        self.declare_parameter("jump_distance_m", 2.0)
        self.declare_parameter("jump_period_sec", 3.0)

        # v0.1.2+ mixed-mode gated-fuse validation.
        self.declare_parameter("mixed_step_distance_m", 20.0)

        # mixed_step_transient:
        # Setpoint velocity and vehicle velocity deliberately differ by 4 m/s,
        # but vehicle is still moving toward the stepped position target.
        self.declare_parameter("mixed_transient_setpoint_vx", -2.0)
        self.declare_parameter("mixed_transient_actual_vx", 2.0)

        # mixed_critical_stall:
        # Setpoint asks for aggressive forward motion, vehicle is physically stuck.
        self.declare_parameter("mixed_critical_setpoint_vx", 4.5)
        self.declare_parameter("mixed_critical_actual_vx", 0.0)

        # Optional synthetic load metadata
        self.declare_parameter("load_profile", "synthetic")
        self.declare_parameter("cpu_load_percent", 0.0)
        self.declare_parameter("gpu_load_percent", 0.0)
        self.declare_parameter("npu_load_percent", 0.0)

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
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.profile = str(self.get_parameter("profile").value).lower().strip()
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.offboard_mode = str(self.get_parameter("offboard_mode").value).lower().strip()

        self.fault_start_sec = float(self.get_parameter("fault_start_sec").value)
        self.fault_duration_sec = float(self.get_parameter("fault_duration_sec").value)

        self.vx_sp = float(self.get_parameter("vx_sp").value)
        self.vy_sp = float(self.get_parameter("vy_sp").value)
        self.vz_sp = float(self.get_parameter("vz_sp").value)

        self.x_sp = float(self.get_parameter("x0").value)
        self.y_sp = float(self.get_parameter("y0").value)
        self.z_sp = float(self.get_parameter("z0").value)

        self.x = self.x_sp
        self.y = self.y_sp
        self.z = self.z_sp

        self.response_ratio = float(self.get_parameter("response_ratio").value)
        self.jitter_pause_sec = float(self.get_parameter("jitter_pause_sec").value)
        self.jitter_period_sec = float(self.get_parameter("jitter_period_sec").value)
        self.jump_distance_m = float(self.get_parameter("jump_distance_m").value)
        self.jump_period_sec = float(self.get_parameter("jump_period_sec").value)

        self.mixed_step_distance_m = float(
            self.get_parameter("mixed_step_distance_m").value
        )
        self.mixed_transient_setpoint_vx = float(
            self.get_parameter("mixed_transient_setpoint_vx").value
        )
        self.mixed_transient_actual_vx = float(
            self.get_parameter("mixed_transient_actual_vx").value
        )
        self.mixed_critical_setpoint_vx = float(
            self.get_parameter("mixed_critical_setpoint_vx").value
        )
        self.mixed_critical_actual_vx = float(
            self.get_parameter("mixed_critical_actual_vx").value
        )

        self.load_profile = str(self.get_parameter("load_profile").value)
        self.cpu_load_percent = float(self.get_parameter("cpu_load_percent").value)
        self.gpu_load_percent = float(self.get_parameter("gpu_load_percent").value)
        self.npu_load_percent = float(self.get_parameter("npu_load_percent").value)

        # --------------------------------------------------------
        # Runtime state
        # --------------------------------------------------------
        self.node_start_time = self._now_sec()
        self.last_loop_time = self.node_start_time

        self.next_jitter_time = self.node_start_time + self.fault_start_sec
        self.jitter_pause_until = 0.0

        self.next_jump_time = self.node_start_time + self.fault_start_sec
        self.jump_sign = 1.0

        self.mixed_step_injected = False

        self.last_fault_state = "NONE"
        self.last_setpoint_published = False

        # --------------------------------------------------------
        # ROS interfaces
        # --------------------------------------------------------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            self.offboard_control_mode_topic,
            PX4_SENSOR_QOS,
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            self.trajectory_setpoint_topic,
            PX4_SENSOR_QOS,
        )

        self.odometry_pub = self.create_publisher(
            VehicleOdometry,
            self.vehicle_odometry_topic,
            PX4_SENSOR_QOS,
        )

        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.timer = self.create_timer(
            1.0 / max(self.rate_hz, 1.0),
            self._loop,
        )

        self.get_logger().info(
            "Synthetic PX4 publisher started | "
            f"profile={self.profile} | "
            f"offboard_mode={self._effective_offboard_mode()} | "
            f"topics=({self.offboard_control_mode_topic}, "
            f"{self.trajectory_setpoint_topic}, {self.vehicle_odometry_topic}) | "
            f"rate={self.rate_hz:.1f}Hz"
        )

    # ============================================================
    # Time / mode
    # ============================================================

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _px4_timestamp_us(self, now: Optional[float] = None) -> int:
        if now is None:
            now = self._now_sec()
        return int(max(0.0, now - self.node_start_time) * 1e6)

    def _effective_offboard_mode(self) -> str:
        if self.profile in {"mixed_step_transient", "mixed_critical_stall"}:
            return "mixed"

        if self.offboard_mode in {"position", "velocity", "mixed"}:
            return self.offboard_mode

        return "velocity"

    # ============================================================
    # Main loop
    # ============================================================

    def _loop(self) -> None:
        now = self._now_sec()
        dt = clamp(now - self.last_loop_time, 1e-4, 0.20)
        self.last_loop_time = now

        elapsed = now - self.node_start_time
        fault_active = (
            self.fault_start_sec
            <= elapsed
            <= self.fault_start_sec + self.fault_duration_sec
        )

        fault_state = "NONE"

        setpoint_vx = self.vx_sp
        setpoint_vy = self.vy_sp
        setpoint_vz = self.vz_sp

        actual_vx = setpoint_vx
        actual_vy = setpoint_vy
        actual_vz = setpoint_vz

        publish_setpoint = True

        # --------------------------------------------------------
        # Fault profiles
        # --------------------------------------------------------
        if self.profile == "setpoint_jitter" and fault_active:
            fault_state = "SETPOINT_JITTER"

            if now >= self.next_jitter_time:
                self.jitter_pause_until = now + self.jitter_pause_sec
                self.next_jitter_time = now + self.jitter_period_sec

            if now <= self.jitter_pause_until:
                publish_setpoint = False

        elif self.profile == "command_response_mismatch" and fault_active:
            fault_state = "COMMAND_RESPONSE_MISMATCH"
            actual_vx = setpoint_vx * self.response_ratio
            actual_vy = setpoint_vy * self.response_ratio
            actual_vz = setpoint_vz * self.response_ratio

        elif self.profile == "gps_vio_jump" and fault_active:
            fault_state = "GPS_VIO_JUMP"

            if now >= self.next_jump_time:
                self.x += self.jump_sign * self.jump_distance_m
                self.jump_sign *= -1.0
                self.next_jump_time = now + self.jump_period_sec
                self.get_logger().warn(
                    "[SYNTHETIC_PX4] Injected GPS/VIO jump | "
                    f"distance={self.jump_distance_m:.2f}m"
                )

        elif self.profile == "mixed_step_transient" and fault_active:
            fault_state = "MIXED_STEP_TRANSIENT"

            if not self.mixed_step_injected:
                self.x_sp += self.mixed_step_distance_m
                self.mixed_step_injected = True
                self.get_logger().warn(
                    "[SYNTHETIC_PX4] Injected mixed-mode position step | "
                    f"distance={self.mixed_step_distance_m:.2f}m | "
                    "expected observer state: DEGRADED / POSITION_TRACKING_TRANSIENT"
                )

            # Large feed-forward phase mismatch, but vehicle is still moving
            # toward the stepped position target. This validates that the
            # gated critical fuse does not red-trigger during legitimate
            # transient capture.
            setpoint_vx = self.mixed_transient_setpoint_vx
            setpoint_vy = 0.0
            setpoint_vz = 0.0

            actual_vx = self.mixed_transient_actual_vx
            actual_vy = 0.0
            actual_vz = 0.0

        elif self.profile == "mixed_critical_stall" and fault_active:
            fault_state = "MIXED_CRITICAL_STALL"

            if not self.mixed_step_injected:
                self.x_sp += self.mixed_step_distance_m
                self.mixed_step_injected = True
                self.get_logger().warn(
                    "[SYNTHETIC_PX4] Injected mixed-mode critical stall | "
                    f"distance={self.mixed_step_distance_m:.2f}m | "
                    "expected observer state: transient first, then "
                    "ERROR / CRITICAL_COMMAND_RESPONSE_MISMATCH"
                )

            # Position target stays far away / keeps moving, while the vehicle
            # is physically stalled. After the position stable-count gate opens,
            # the critical velocity residual should trip red.
            setpoint_vx = self.mixed_critical_setpoint_vx
            setpoint_vy = 0.0
            setpoint_vz = 0.0

            actual_vx = self.mixed_critical_actual_vx
            actual_vy = 0.0
            actual_vz = 0.0

        # --------------------------------------------------------
        # Setpoint and vehicle integration
        # --------------------------------------------------------
        self.x_sp += setpoint_vx * dt
        self.y_sp += setpoint_vy * dt
        self.z_sp += setpoint_vz * dt

        self.x += actual_vx * dt
        self.y += actual_vy * dt
        self.z += actual_vz * dt

        # --------------------------------------------------------
        # Publish topics
        # --------------------------------------------------------
        timestamp_us = self._px4_timestamp_us(now)

        self._publish_offboard_control_mode(timestamp_us)

        if publish_setpoint:
            self._publish_trajectory_setpoint(
                timestamp_us,
                setpoint_vx=setpoint_vx,
                setpoint_vy=setpoint_vy,
                setpoint_vz=setpoint_vz,
            )

        self._publish_vehicle_odometry(
            timestamp_us=timestamp_us,
            vx=actual_vx,
            vy=actual_vy,
            vz=actual_vz,
        )

        self._publish_status(
            now=now,
            elapsed=elapsed,
            fault_state=fault_state,
            publish_setpoint=publish_setpoint,
            setpoint_vx=setpoint_vx,
            setpoint_vy=setpoint_vy,
            setpoint_vz=setpoint_vz,
            actual_vx=actual_vx,
            actual_vy=actual_vy,
            actual_vz=actual_vz,
        )

        if fault_state != self.last_fault_state:
            if fault_state != "NONE":
                self.get_logger().warn(f"[SYNTHETIC_PX4] {fault_state}_START")
            elif self.last_fault_state != "NONE":
                self.get_logger().info(f"[SYNTHETIC_PX4] {self.last_fault_state}_END")

            self.last_fault_state = fault_state

    # ============================================================
    # PX4 message publishing
    # ============================================================

    def _publish_offboard_control_mode(self, timestamp_us: int) -> None:
        msg = OffboardControlMode()
        safe_set(msg, "timestamp", int(timestamp_us))

        mode = self._effective_offboard_mode()

        safe_set(msg, "position", mode in {"position", "mixed"})
        safe_set(msg, "velocity", mode in {"velocity", "mixed"})
        safe_set(msg, "acceleration", False)
        safe_set(msg, "attitude", False)
        safe_set(msg, "body_rate", False)

        # PX4 message versions vary. Set only if the field exists.
        safe_set(msg, "actuator", False)
        safe_set(msg, "thrust_and_torque", False)
        safe_set(msg, "direct_actuator", False)

        self.offboard_pub.publish(msg)

    def _publish_trajectory_setpoint(
        self,
        timestamp_us: int,
        setpoint_vx: float,
        setpoint_vy: float,
        setpoint_vz: float,
    ) -> None:
        msg = TrajectorySetpoint()
        safe_set(msg, "timestamp", int(timestamp_us))

        # Keep position finite for easier diagnostic comparison.
        # In real PX4 velocity-only control, NaN position fields may be used.
        safe_set_array(msg, "position", [self.x_sp, self.y_sp, self.z_sp])
        safe_set_array(msg, "velocity", [setpoint_vx, setpoint_vy, setpoint_vz])
        safe_set_array(msg, "acceleration", [0.0, 0.0, 0.0])

        # Some px4_msgs versions include jerk.
        safe_set_array(msg, "jerk", [0.0, 0.0, 0.0])

        safe_set(msg, "yaw", 0.0)
        safe_set(msg, "yawspeed", 0.0)

        self.trajectory_pub.publish(msg)
        self.last_setpoint_published = True

    def _publish_vehicle_odometry(
        self,
        timestamp_us: int,
        vx: float,
        vy: float,
        vz: float,
    ) -> None:
        msg = VehicleOdometry()
        safe_set(msg, "timestamp", int(timestamp_us))
        safe_set(msg, "timestamp_sample", int(timestamp_us))

        # Use NED frame constants if present; otherwise use common fallback.
        safe_set(msg, "pose_frame", getattr(msg, "POSE_FRAME_NED", 1))
        safe_set(msg, "velocity_frame", getattr(msg, "VELOCITY_FRAME_NED", 1))

        safe_set_array(msg, "position", [self.x, self.y, self.z])

        # VehicleOdometry.q convention in PX4 is Hamilton quaternion.
        # Identity orientation: w, x, y, z.
        safe_set_array(msg, "q", [1.0, 0.0, 0.0, 0.0])

        safe_set_array(msg, "velocity", [vx, vy, vz])
        safe_set_array(msg, "angular_velocity", [0.0, 0.0, 0.0])

        safe_set_array(msg, "position_variance", [0.0, 0.0, 0.0])
        safe_set_array(msg, "orientation_variance", [0.0, 0.0, 0.0])
        safe_set_array(msg, "velocity_variance", [0.0, 0.0, 0.0])

        safe_set(msg, "reset_counter", 0)
        safe_set(msg, "quality", 100)

        self.odometry_pub.publish(msg)

    # ============================================================
    # Status publishing
    # ============================================================

    def _publish_status(
        self,
        now: float,
        elapsed: float,
        fault_state: str,
        publish_setpoint: bool,
        setpoint_vx: float,
        setpoint_vy: float,
        setpoint_vz: float,
        actual_vx: float,
        actual_vy: float,
        actual_vz: float,
    ) -> None:
        payload = {
            "timestamp": now,
            "profile": self.profile,
            "offboardMode": self._effective_offboard_mode(),
            "elapsedSec": elapsed,
            "faultState": fault_state,
            "publishedSetpoint": publish_setpoint,
            "setpoint": {
                "position": [self.x_sp, self.y_sp, self.z_sp],
                "velocity": [setpoint_vx, setpoint_vy, setpoint_vz],
            },
            "vehicle": {
                "position": [self.x, self.y, self.z],
                "velocity": [actual_vx, actual_vy, actual_vz],
            },
            "faultWindow": {
                "startSec": self.fault_start_sec,
                "endSec": self.fault_start_sec + self.fault_duration_sec,
                "active": (
                    self.fault_start_sec
                    <= elapsed
                    <= self.fault_start_sec + self.fault_duration_sec
                ),
            },
            "load": {
                "profile": self.load_profile,
                "cpuLoadPercent": self.cpu_load_percent,
                "gpuLoadPercent": self.gpu_load_percent,
                "npuLoadPercent": self.npu_load_percent,
            },
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(msg)


# ============================================================
# Main
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticPx4Publisher()

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
