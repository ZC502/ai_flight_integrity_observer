#!/usr/bin/env python3
"""
px4_qos.py

QoS helpers for PX4 / ROS 2 uXRCE-DDS topics.

PX4 high-rate topics are commonly exposed with sensor-style QoS.
For passive observers, BestEffort + Volatile is usually the safest
default to avoid the common "topic exists but callback never fires" trap.
"""

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)


def make_px4_sensor_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=int(depth),
    )


PX4_SENSOR_QOS = make_px4_sensor_qos(depth=10)
