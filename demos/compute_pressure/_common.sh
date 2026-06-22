#!/usr/bin/env bash
set -euo pipefail

PKG=${PKG:-ai_flight_integrity_observer}
DURATION_SEC=${DURATION_SEC:-120}
RAW_TOPIC=${RAW_TOPIC:-/ai/raw_trajectory_setpoint}
SETPOINT_TOPIC=${SETPOINT_TOPIC:-/fmu/in/trajectory_setpoint}
MODE_TOPIC=${MODE_TOPIC:-/fmu/in/offboard_control_mode}
ODOM_TOPIC=${ODOM_TOPIC:-/fmu/out/vehicle_odometry}

PIDS=()

cleanup() {
  echo ""
  echo "[cleanup] stopping demo processes..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  sleep 0.5
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

start_node() {
  echo "[start] $*"
  "$@" &
  PIDS+=("$!")
  sleep 0.8
}

start_observer_and_synthetic_source() {
  echo "[info] Starting OBIO/AFIO observer and synthetic PX4 source."
  echo "[info] No real drone required. PX4 SITL is optional for this synthetic demo."

  start_node ros2 run "$PKG" flight_integrity_node --ros-args \
    -p trajectory_setpoint_topic:="$SETPOINT_TOPIC" \
    -p offboard_control_mode_topic:="$MODE_TOPIC" \
    -p vehicle_odometry_topic:="$ODOM_TOPIC"

  # Synthetic source publishes odometry + offboard mode directly, but sends
  # trajectory setpoints to RAW_TOPIC. fake_slam_stressor forwards RAW_TOPIC
  # to SETPOINT_TOPIC with or without timing stalls.
  start_node ros2 run "$PKG" synthetic_px4_publisher --ros-args \
    -p profile:=normal \
    -p rate_hz:=50.0 \
    -p trajectory_setpoint_topic:="$RAW_TOPIC" \
    -p offboard_control_mode_topic:="$MODE_TOPIC" \
    -p vehicle_odometry_topic:="$ODOM_TOPIC"
}

watch_diagnostics_hint() {
  cat <<'TXT'

[optional watch terminal]
Run one of these in another terminal:

  ros2 topic echo /diagnostics --once --full-length

or, if installed:

  ros2 run ai_flight_integrity_observer afi_live_dashboard

TXT
}

wait_for_duration() {
  echo "[info] Demo running for ${DURATION_SEC}s. Press Ctrl-C to stop."
  sleep "$DURATION_SEC"
}
