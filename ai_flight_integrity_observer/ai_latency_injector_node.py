#!/usr/bin/env python3
"""
ai_latency_injector_node.py

Controlled offboard setpoint latency injector for AI Flight Integrity Observer.

This node sits between an upstream AI/offboard planner and PX4:

    upstream planner -> /ai/raw_trajectory_setpoint
        -> ai_latency_injector_node
        -> /fmu/in/trajectory_setpoint
        -> PX4 SITL / real PX4 bridge

It does NOT publish odometry.
It does NOT modify PX4.
It does NOT replace the observer.
It only delays the trajectory setpoint stream to emulate AI/VIO/VLA inference
blocking on the companion-compute side.
"""

from __future__ import annotations

import json
import math
import random
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import TrajectorySetpoint

try:
    from .px4_qos import PX4_SENSOR_QOS
except ImportError:
    from px4_qos import PX4_SENSOR_QOS


def finite_or(value: Any, fallback: float = 0.0) -> float:
    try:
        x = float(value)
        return x if math.isfinite(x) else fallback
    except Exception:
        return fallback


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def safe_set(msg: Any, field: str, value: Any) -> None:
    if hasattr(msg, field):
        try:
            setattr(msg, field, value)
        except Exception:
            pass


class AILatencyInjectorNode(Node):
    def __init__(self) -> None:
        super().__init__("ai_latency_injector_node")

        self.declare_parameter("input_topic", "/ai/raw_trajectory_setpoint")
        self.declare_parameter("output_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("status_topic", "/ai_latency_injector/status")

        self.declare_parameter("ai_lag_ms", 0.0)
        self.declare_parameter("jitter_ms", 0.0)
        self.declare_parameter("drop_probability", 0.0)
        self.declare_parameter("burst_every_n", 0)
        self.declare_parameter("burst_lag_ms", 0.0)
        self.declare_parameter("max_lag_ms", 2000.0)

        # "blocking" intentionally sleeps inside the subscription callback.
        # This emulates an AI/offboard generation thread being occupied by inference.
        self.declare_parameter("mode", "blocking")

        # Default false preserves upstream timestamp for auditability.
        # Enable only if your PX4 bridge/pipeline requires fresh timestamps.
        self.declare_parameter("rewrite_timestamp_us", False)

        self.declare_parameter("load_profile", "controlled_ai_lag")
        self.declare_parameter("cpu_load_percent", 0.0)
        self.declare_parameter("gpu_load_percent", 0.0)
        self.declare_parameter("npu_load_percent", 0.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.ai_lag_ms = finite_or(self.get_parameter("ai_lag_ms").value, 0.0)
        self.jitter_ms = finite_or(self.get_parameter("jitter_ms").value, 0.0)
        self.drop_probability = clamp(
            finite_or(self.get_parameter("drop_probability").value, 0.0),
            0.0,
            1.0,
        )
        self.burst_every_n = int(finite_or(self.get_parameter("burst_every_n").value, 0.0))
        self.burst_lag_ms = finite_or(self.get_parameter("burst_lag_ms").value, 0.0)
        self.max_lag_ms = max(0.0, finite_or(self.get_parameter("max_lag_ms").value, 2000.0))
        self.mode = str(self.get_parameter("mode").value).lower().strip()
        self.rewrite_timestamp_us = bool(self.get_parameter("rewrite_timestamp_us").value)

        self.load_profile = str(self.get_parameter("load_profile").value)
        self.cpu_load_percent = finite_or(self.get_parameter("cpu_load_percent").value, 0.0)
        self.gpu_load_percent = finite_or(self.get_parameter("gpu_load_percent").value, 0.0)
        self.npu_load_percent = finite_or(self.get_parameter("npu_load_percent").value, 0.0)

        self.node_start_time = self._now_sec()
        self.rx_count = 0
        self.tx_count = 0
        self.drop_count = 0
        self.total_injected_lag_ms = 0.0
        self.max_injected_lag_ms_observed = 0.0
        self.last_rx_time = None
        self.last_tx_time = None
        self.last_injected_lag_ms = 0.0
        self.last_input_interval_ms = 0.0

        self.sub = self.create_subscription(
            TrajectorySetpoint,
            self.input_topic,
            self._trajectory_callback,
            PX4_SENSOR_QOS,
        )

        self.pub = self.create_publisher(
            TrajectorySetpoint,
            self.output_topic,
            PX4_SENSOR_QOS,
        )

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.status_timer = self.create_timer(1.0, self._publish_status_timer)

        self.get_logger().info(
            "AI latency injector started | "
            f"input={self.input_topic} | output={self.output_topic} | "
            f"mode={self.mode} | ai_lag_ms={self.ai_lag_ms:.1f} | "
            f"jitter_ms={self.jitter_ms:.1f} | drop_probability={self.drop_probability:.3f} | "
            f"burst_every_n={self.burst_every_n} | burst_lag_ms={self.burst_lag_ms:.1f}"
        )

        if self.output_topic == self.input_topic:
            self.get_logger().error(
                "input_topic and output_topic are identical. This can create a feedback loop. "
                "Use /ai/raw_trajectory_setpoint as input and /fmu/in/trajectory_setpoint as output."
            )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _px4_timestamp_us(self) -> int:
        now = self._now_sec()
        return int(max(0.0, now - self.node_start_time) * 1e6)

    def _compute_injected_lag_ms(self) -> float:
        lag_ms = max(0.0, self.ai_lag_ms)

        if self.jitter_ms > 0.0:
            lag_ms += random.uniform(-self.jitter_ms, self.jitter_ms)

        if self.burst_every_n > 0 and self.rx_count > 0:
            if self.rx_count % self.burst_every_n == 0:
                lag_ms += max(0.0, self.burst_lag_ms)

        return clamp(lag_ms, 0.0, self.max_lag_ms)

    def _trajectory_callback(self, msg: TrajectorySetpoint) -> None:
        rx_time = self._now_sec()
        self.rx_count += 1

        if self.last_rx_time is not None:
            self.last_input_interval_ms = max(0.0, (rx_time - self.last_rx_time) * 1000.0)
        self.last_rx_time = rx_time

        if self.drop_probability > 0.0 and random.random() < self.drop_probability:
            self.drop_count += 1
            self.last_injected_lag_ms = 0.0
            self._publish_status(event="drop", rx_time=rx_time, tx_time=None)
            return

        lag_ms = self._compute_injected_lag_ms()
        self.last_injected_lag_ms = lag_ms
        self.total_injected_lag_ms += lag_ms
        self.max_injected_lag_ms_observed = max(self.max_injected_lag_ms_observed, lag_ms)

        if self.mode == "blocking" and lag_ms > 0.0:
            time.sleep(lag_ms * 1e-3)

        out_msg = msg

        if self.rewrite_timestamp_us:
            safe_set(out_msg, "timestamp", int(self._px4_timestamp_us()))

        self.pub.publish(out_msg)

        tx_time = self._now_sec()
        self.last_tx_time = tx_time
        self.tx_count += 1

        self._publish_status(event="publish", rx_time=rx_time, tx_time=tx_time)

    def _publish_status_timer(self) -> None:
        self._publish_status(event="heartbeat", rx_time=None, tx_time=None)

    def _publish_status(self, event: str, rx_time: Any, tx_time: Any) -> None:
        now = self._now_sec()
        mean_lag = (
            self.total_injected_lag_ms / self.tx_count
            if self.tx_count > 0
            else 0.0
        )

        payload = {
            "timestamp": now,
            "event": event,
            "mode": self.mode,
            "inputTopic": self.input_topic,
            "outputTopic": self.output_topic,
            "aiLagMs": self.ai_lag_ms,
            "jitterMs": self.jitter_ms,
            "dropProbability": self.drop_probability,
            "burstEveryN": self.burst_every_n,
            "burstLagMs": self.burst_lag_ms,
            "maxLagMs": self.max_lag_ms,
            "rewriteTimestampUs": self.rewrite_timestamp_us,
            "lastInjectedLagMs": self.last_injected_lag_ms,
            "meanInjectedLagMs": mean_lag,
            "maxInjectedLagMsObserved": self.max_injected_lag_ms_observed,
            "lastInputIntervalMs": self.last_input_interval_ms,
            "rxCount": self.rx_count,
            "txCount": self.tx_count,
            "dropCount": self.drop_count,
            "rxTimeSec": rx_time,
            "txTimeSec": tx_time,
            "nodeAgeSec": max(0.0, now - self.node_start_time),
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AILatencyInjectorNode()

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
