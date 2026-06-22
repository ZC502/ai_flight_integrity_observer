#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/_common.sh"

WORKERS=${WORKERS:-$(nproc)}
DUTY=${DUTY:-0.95}

echo "============================================================"
echo "Scenario B: High CPU, boundary still healthy"
echo "Expected: htop shows high CPU, but OBIO/AFIO remains GREEN."
echo "============================================================"

start_observer_and_synthetic_source

start_node ros2 run "$PKG" fake_slam_stressor_node --ros-args \
  -p input_topic:="$RAW_TOPIC" \
  -p output_topic:="$SETPOINT_TOPIC" \
  -p mode:=pass_through

# Start low-priority CPU burn. It should not touch the setpoint path.
echo "[start] CPU burner workers=$WORKERS duty=$DUTY"
python3 "$SCRIPT_DIR/background_cpu_burn.py" --duration-sec "$DURATION_SEC" --workers "$WORKERS" --duty-cycle "$DUTY" --nice 10 &
PIDS+=("$!")

watch_diagnostics_hint
wait_for_duration
