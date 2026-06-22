#!/usr/bin/env python3
"""
fake_slam_stressor_node.py

Minimal reproducible compute-pressure demo for OBIO / AFIO.

This node sits between an upstream setpoint source and PX4:

    synthetic/offboard source -> /ai/raw_trajectory_setpoint
        -> fake_slam_stressor_node
        -> /fmu/in/trajectory_setpoint

It simulates SLAM/VIO/LVI side effects that do NOT necessarily show up as
high CPU%, but can still corrupt the Offboard setpoint timing boundary:

    - pass_through: healthy boundary
    - periodic_stall: sensor sync / loop-closure / mutex-style stalls

It is intentionally simple and deterministic. It does not modify PX4 and does
not publish odometry or offboard mode.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from px4_msgs.msg import TrajectorySetpoint

try:
    from .px4_qos import PX4_SENSOR_QOS
except Exception:
    from px4_qos import PX4_SENSOR_QOS


def finite_or(value: Any, fallback: float = 0.0) -> float:
    try:
        x = float(value)
        return x if math.isfinite(x) else fallback
    except Exception:
        return fallback


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class FakeSlamStressorNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_slam_stressor_node")

        self.declare_parameter("input_topic", "/ai/raw_trajectory_setpoint")
        self.declare_parameter("output_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("load_control_topic", "/obio_demo/load_enabled")
        self.declare_parameter("status_topic", "/obio_demo/fake_slam_status")

        # Modes:
        #   pass_through    - forward immediately
        #   periodic_stall  - periodically block callback before forwarding
        self.declare_parameter("mode", "pass_through")

        # Every stall_period_sec, block one callback for stall_duration_ms.
        # This emulates loop closure, feature matching, sensor sync wait,
        # lock contention, or executor starvation.
        self.declare_parameter("stall_period_sec", 1.5)
        self.declare_parameter("stall_duration_ms", 250.0)

        # If true, one message is published after the sleep. If false, the
        # triggering message is dropped. Publishing after sleep creates a stale
        # but syntactically valid setpoint; dropping creates a stream gap.
        self.declare_parameter("publish_after_stall", True)

        # Initial load state. The load shedder toggles this via Bool.
        self.declare_parameter("load_enabled", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.load_control_topic = str(self.get_parameter("load_control_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        self.stall_period_sec = max(0.05, finite_or(self.get_parameter("stall_period_sec").value, 1.5))
        self.stall_duration_ms = clamp(finite_or(self.get_parameter("stall_duration_ms").value, 250.0), 0.0, 2000.0)
        self.publish_after_stall = bool(self.get_parameter("publish_after_stall").value)
        self.load_enabled = bool(self.get_parameter("load_enabled").value)

        self.rx_count = 0
        self.tx_count = 0
        self.drop_count = 0
        self.stall_count = 0
        self.last_event = "startup"
        self.last_interval_ms = 0.0
        self.last_rx_time: float | None = None
        self.next_stall_time = self._now_sec() + self.stall_period_sec
        self.last_stall_duration_ms = 0.0

        self.sub = self.create_subscription(
            TrajectorySetpoint,
            self.input_topic,
            self._setpoint_callback,
            PX4_SENSOR_QOS,
        )
        self.pub = self.create_publisher(
            TrajectorySetpoint,
            self.output_topic,
            PX4_SENSOR_QOS,
        )
        self.control_sub = self.create_subscription(
            Bool,
            self.load_control_topic,
            self._load_control_callback,
            10,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.status_timer = self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            "fake_slam_stressor started | "
            f"input={self.input_topic} | output={self.output_topic} | "
            f"mode={self.mode} | stall_period_sec={self.stall_period_sec:.2f} | "
            f"stall_duration_ms={self.stall_duration_ms:.1f} | "
            f"load_enabled={self.load_enabled}"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _load_control_callback(self, msg: Bool) -> None:
        previous = self.load_enabled
        self.load_enabled = bool(msg.data)
        self.last_event = "load_enabled" if self.load_enabled else "load_shed_pause"
        if previous != self.load_enabled:
            self.get_logger().warn(
                f"load_enabled changed: {previous} -> {self.load_enabled}"
            )

    def _should_stall(self, now: float) -> bool:
        if self.mode not in {"periodic_stall", "stall", "lock_stall", "sensor_sync_stall"}:
            return False
        if not self.load_enabled:
            return False
        return now >= self.next_stall_time

    def _setpoint_callback(self, msg: TrajectorySetpoint) -> None:
        now = self._now_sec()
        self.rx_count += 1

        if self.last_rx_time is not None:
            self.last_interval_ms = max(0.0, (now - self.last_rx_time) * 1000.0)
        self.last_rx_time = now

        if self._should_stall(now):
            self.stall_count += 1
            self.last_event = "periodic_stall"
            self.last_stall_duration_ms = self.stall_duration_ms
            self.next_stall_time = now + self.stall_period_sec

            self.get_logger().warn(
                f"Fake SLAM stall #{self.stall_count}: sleeping {self.stall_duration_ms:.1f} ms "
                "before forwarding setpoint"
            )
            time.sleep(self.stall_duration_ms * 1e-3)

            if not self.publish_after_stall:
                self.drop_count += 1
                self.last_event = "drop_after_stall"
                return

        self.pub.publish(msg)
        self.tx_count += 1
        if self.last_event not in {"periodic_stall", "drop_after_stall", "load_shed_pause"}:
            self.last_event = "pass_through"

    def _publish_status(self) -> None:
        payload = {
            "timestamp": self._now_sec(),
            "mode": self.mode,
            "loadEnabled": self.load_enabled,
            "inputTopic": self.input_topic,
            "outputTopic": self.output_topic,
            "rxCount": self.rx_count,
            "txCount": self.tx_count,
            "dropCount": self.drop_count,
            "stallCount": self.stall_count,
            "lastEvent": self.last_event,
            "lastInputIntervalMs": round(self.last_interval_ms, 3),
            "stallPeriodSec": self.stall_period_sec,
            "stallDurationMs": self.stall_duration_ms,
            "nextStallInSec": round(max(0.0, self.next_stall_time - self._now_sec()), 3),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeSlamStressorNode()
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
