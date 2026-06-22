#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_common.sh"

echo "============================================================"
echo "Scenario A: Baseline — healthy Offboard stream"
echo "Expected: OBIO/AFIO remains GREEN."
echo "============================================================"

start_observer_and_synthetic_source

start_node ros2 run "$PKG" fake_slam_stressor_node --ros-args \
  -p input_topic:="$RAW_TOPIC" \
  -p output_topic:="$SETPOINT_TOPIC" \
  -p mode:=pass_through

watch_diagnostics_hint
wait_for_duration
