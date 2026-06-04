#!/usr/bin/env python3
"""
flight_residual_core.py

Mode-aware flight residual classification helpers for AI Flight Integrity Observer.

This module keeps PX4 offboard-control semantics out of the ROS node plumbing:
- POSITION_MODE: position setpoint is primary; velocity can be only feed-forward / auxiliary.
- VELOCITY_MODE: velocity setpoint is primary.
- MIXED_POSITION_VELOCITY_MODE: both position and velocity are meaningful.

The goal of v0.1.1 is to avoid treating normal PX4 position-setpoint climb/descent
as COMMAND_RESPONSE_MISMATCH merely because vehicle velocity is non-zero while the
TrajectorySetpoint velocity vector is zero or unused.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional


POSITION_MODE = "POSITION_MODE"
VELOCITY_MODE = "VELOCITY_MODE"
MIXED_POSITION_VELOCITY_MODE = "MIXED_POSITION_VELOCITY_MODE"
ACCELERATION_MODE = "ACCELERATION_MODE"
ATTITUDE_MODE = "ATTITUDE_MODE"
BODY_RATE_MODE = "BODY_RATE_MODE"
ACTUATOR_MODE = "ACTUATOR_MODE"
UNKNOWN_MODE = "UNKNOWN_MODE"

PRIMARY_POSITION = "position"
PRIMARY_VELOCITY = "velocity"
PRIMARY_MIXED = "mixed"
PRIMARY_TIMING = "timing"
PRIMARY_UNKNOWN = "unknown"

TREND_DECREASING = "decreasing"
TREND_INCREASING = "increasing"
TREND_STABLE = "stable"
TREND_UNKNOWN = "unknown"


@dataclass(frozen=True)
class FlightIntegrityClassification:
    status: str
    dominant_cause: str
    dominant_cause_candidate: str
    control_semantic_mode: str
    primary_residual_type: str
    position_residual_trend: str
    position_residual_delta: float
    tracking_transient: bool


def _bool_field(msg: Any, field: str) -> bool:
    if not hasattr(msg, field):
        return False
    try:
        return bool(getattr(msg, field))
    except Exception:
        return False


def infer_control_semantic_mode(offboard_msg: Any) -> str:
    """Infer the active PX4 offboard control semantic from OffboardControlMode."""
    position = _bool_field(offboard_msg, "position")
    velocity = _bool_field(offboard_msg, "velocity")
    acceleration = _bool_field(offboard_msg, "acceleration")
    attitude = _bool_field(offboard_msg, "attitude")
    body_rate = _bool_field(offboard_msg, "body_rate")
    actuator = (
        _bool_field(offboard_msg, "actuator")
        or _bool_field(offboard_msg, "thrust_and_torque")
        or _bool_field(offboard_msg, "direct_actuator")
    )

    if position and velocity:
        return MIXED_POSITION_VELOCITY_MODE
    if position:
        return POSITION_MODE
    if velocity:
        return VELOCITY_MODE
    if acceleration:
        return ACCELERATION_MODE
    if attitude:
        return ATTITUDE_MODE
    if body_rate:
        return BODY_RATE_MODE
    if actuator:
        return ACTUATOR_MODE
    return UNKNOWN_MODE


def primary_residual_type_for_mode(control_semantic_mode: str) -> str:
    if control_semantic_mode == POSITION_MODE:
        return PRIMARY_POSITION
    if control_semantic_mode == VELOCITY_MODE:
        return PRIMARY_VELOCITY
    if control_semantic_mode == MIXED_POSITION_VELOCITY_MODE:
        return PRIMARY_MIXED
    if control_semantic_mode in {ACCELERATION_MODE, ATTITUDE_MODE, BODY_RATE_MODE, ACTUATOR_MODE}:
        return PRIMARY_UNKNOWN
    return PRIMARY_UNKNOWN


def position_residual_trend(
    current_position_residual: float,
    previous_position_residual: Optional[float],
    deadband_m: float = 0.05,
) -> tuple[str, float]:
    """Return (trend, delta), where delta = current - previous."""
    if previous_position_residual is None:
        return TREND_UNKNOWN, 0.0

    delta = float(current_position_residual) - float(previous_position_residual)

    if delta < -abs(deadband_m):
        return TREND_DECREASING, delta
    if delta > abs(deadband_m):
        return TREND_INCREASING, delta
    return TREND_STABLE, delta


def classify_flight_integrity(
    *,
    offboard_active: bool,
    control_semantic_mode: str,
    velocity_tracking_residual: float,
    position_tracking_residual: float,
    gps_vio_jump_metric: float,
    setpoint_age_ms: float,
    offboard_age_ms: float,
    odom_age_ms: float,
    setpoint_jitter_ms: float,
    previous_position_tracking_residual: Optional[float],
    position_trend_deadband_m: float,
    setpoint_stale_warn_ms: float,
    setpoint_stale_error_ms: float,
    offboard_stale_warn_ms: float,
    offboard_stale_error_ms: float,
    odometry_stale_warn_ms: float,
    odometry_stale_error_ms: float,
    setpoint_jitter_warn_ms: float,
    setpoint_jitter_error_ms: float,
    velocity_residual_warn_mps: float,
    velocity_residual_error_mps: float,
    position_residual_warn_m: float,
    position_residual_error_m: float,
) -> FlightIntegrityClassification:
    """Classify flight integrity with PX4 offboard-mode awareness."""
    primary_residual_type = primary_residual_type_for_mode(control_semantic_mode)
    trend, delta = position_residual_trend(
        position_tracking_residual,
        previous_position_tracking_residual,
        deadband_m=position_trend_deadband_m,
    )

    def result(status: str, cause: str, candidate: str, transient: bool = False):
        return FlightIntegrityClassification(
            status=status,
            dominant_cause=cause,
            dominant_cause_candidate=candidate,
            control_semantic_mode=control_semantic_mode,
            primary_residual_type=primary_residual_type,
            position_residual_trend=trend,
            position_residual_delta=delta,
            tracking_transient=transient,
        )

    if not offboard_active:
        return result("DEGRADED", "OFFBOARD_INACTIVE", "OFFBOARD_INACTIVE")

    # Stream freshness and hard timing faults remain mode-independent.
    if setpoint_age_ms >= setpoint_stale_error_ms:
        return result("RESYNCING", "SETPOINT_STALE", "SETPOINT_STALE")
    if offboard_age_ms >= offboard_stale_error_ms:
        return result("RESYNCING", "OFFBOARD_STALE", "OFFBOARD_STALE")
    if odom_age_ms >= odometry_stale_error_ms:
        return result("RESYNCING", "ODOMETRY_STALE", "ODOMETRY_STALE")
    if setpoint_jitter_ms >= setpoint_jitter_error_ms:
        return result("RESYNCING", "SETPOINT_JITTER", "SETPOINT_JITTER")
    if gps_vio_jump_metric > 0.0:
        return result("RESYNCING", "GPS_VIO_JUMP", "GPS_VIO_JUMP")

    # POSITION_MODE: velocity may be absent, zero, or used as feed-forward.
    # Do not let velocity residual alone trigger COMMAND_RESPONSE_MISMATCH.
    if control_semantic_mode == POSITION_MODE:
        if position_tracking_residual >= position_residual_error_m:
            if trend == TREND_DECREASING:
                return result(
                    "DEGRADED",
                    "POSITION_TRACKING_TRANSIENT",
                    "POSITION_RESPONSE_MISMATCH",
                    transient=True,
                )
            return result(
                "RESYNCING",
                "POSITION_RESPONSE_MISMATCH",
                "POSITION_RESPONSE_MISMATCH",
            )
        if position_tracking_residual >= position_residual_warn_m:
            return result(
                "DEGRADED",
                "POSITION_TRACKING_TRANSIENT" if trend == TREND_DECREASING else "POSITION_RESPONSE_MISMATCH",
                "POSITION_RESPONSE_MISMATCH",
                transient=(trend == TREND_DECREASING),
            )

    # VELOCITY_MODE: velocity residual is the primary execution signal.
    elif control_semantic_mode == VELOCITY_MODE:
        if velocity_tracking_residual >= velocity_residual_error_mps:
            return result("RESYNCING", "COMMAND_RESPONSE_MISMATCH", "COMMAND_RESPONSE_MISMATCH")
        if velocity_tracking_residual >= velocity_residual_warn_mps:
            return result("DEGRADED", "COMMAND_RESPONSE_MISMATCH", "COMMAND_RESPONSE_MISMATCH")

    # MIXED mode: both position and velocity are semantically meaningful.
    elif control_semantic_mode == MIXED_POSITION_VELOCITY_MODE:
        if velocity_tracking_residual >= velocity_residual_error_mps:
            return result("RESYNCING", "COMMAND_RESPONSE_MISMATCH", "COMMAND_RESPONSE_MISMATCH")
        if position_tracking_residual >= position_residual_error_m:
            if trend == TREND_DECREASING:
                return result(
                    "DEGRADED",
                    "POSITION_TRACKING_TRANSIENT",
                    "POSITION_RESPONSE_MISMATCH",
                    transient=True,
                )
            return result("RESYNCING", "POSITION_RESPONSE_MISMATCH", "POSITION_RESPONSE_MISMATCH")
        if velocity_tracking_residual >= velocity_residual_warn_mps:
            return result("DEGRADED", "COMMAND_RESPONSE_MISMATCH", "COMMAND_RESPONSE_MISMATCH")
        if position_tracking_residual >= position_residual_warn_m:
            return result(
                "DEGRADED",
                "POSITION_TRACKING_TRANSIENT" if trend == TREND_DECREASING else "POSITION_RESPONSE_MISMATCH",
                "POSITION_RESPONSE_MISMATCH",
                transient=(trend == TREND_DECREASING),
            )

    # Unsupported / unknown modes: conservative but avoid claiming a velocity-only cause.
    else:
        if position_tracking_residual >= position_residual_error_m:
            return result("RESYNCING", "POSITION_RESPONSE_MISMATCH", "POSITION_RESPONSE_MISMATCH")
        if velocity_tracking_residual >= velocity_residual_error_mps:
            return result("DEGRADED", "UNSUPPORTED_MODE_VELOCITY_RESIDUAL", "COMMAND_RESPONSE_MISMATCH")
        if position_tracking_residual >= position_residual_warn_m:
            return result("DEGRADED", "POSITION_RESPONSE_MISMATCH", "POSITION_RESPONSE_MISMATCH")

    # WARN priority for stream quality.
    if setpoint_age_ms >= setpoint_stale_warn_ms:
        return result("DEGRADED", "SETPOINT_STALE", "SETPOINT_STALE")
    if offboard_age_ms >= offboard_stale_warn_ms:
        return result("DEGRADED", "OFFBOARD_STALE", "OFFBOARD_STALE")
    if odom_age_ms >= odometry_stale_warn_ms:
        return result("DEGRADED", "ODOMETRY_STALE", "ODOMETRY_STALE")
    if setpoint_jitter_ms >= setpoint_jitter_warn_ms:
        return result("DEGRADED", "SETPOINT_JITTER", "SETPOINT_JITTER")

    return result("GREEN", "NONE", "NONE")
