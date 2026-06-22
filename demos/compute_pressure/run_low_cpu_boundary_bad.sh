#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_common.sh"

STALL_MS=${STALL_MS:-300}
STALL_PERIOD=${STALL_PERIOD:-1.5}

echo "============================================================"
echo "Scenario C: Low/moderate CPU, boundary degraded"
echo "Expected: CPU may look fine, but OBIO/AFIO reports jitter/stale stream."
echo "============================================================"

start_observer_and_synthetic_source

start_node ros2 run "$PKG" fake_slam_stressor_node --ros-args \
  -p input_topic:="$RAW_TOPIC" \
  -p output_topic:="$SETPOINT_TOPIC" \
  -p mode:=periodic_stall \
  -p stall_period_sec:="$STALL_PERIOD" \
  -p stall_duration_ms:="$STALL_MS" \
  -p load_enabled:=true

watch_diagnostics_hint
wait_for_duration
