#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_common.sh"

STALL_MS=${STALL_MS:-300}
STALL_PERIOD=${STALL_PERIOD:-1.5}
JITTER_THRESHOLD=${JITTER_THRESHOLD:-100}
PAUSE_SEC=${PAUSE_SEC:-2.0}

echo "============================================================"
echo "Scenario D: OBIO-gated load shedding"
echo "Expected: jitter spike -> load shedder pauses fake SLAM -> boundary recovers."
echo "============================================================"

start_observer_and_synthetic_source

start_node ros2 run "$PKG" fake_slam_stressor_node --ros-args \
  -p input_topic:="$RAW_TOPIC" \
  -p output_topic:="$SETPOINT_TOPIC" \
  -p mode:=periodic_stall \
  -p stall_period_sec:="$STALL_PERIOD" \
  -p stall_duration_ms:="$STALL_MS" \
  -p load_enabled:=true

start_node ros2 run "$PKG" obio_gated_load_shedder --ros-args \
  -p setpoint_jitter_threshold_ms:="$JITTER_THRESHOLD" \
  -p pause_duration_sec:="$PAUSE_SEC" \
  -p load_control_topic:=/obio_demo/load_enabled

watch_diagnostics_hint
wait_for_duration
