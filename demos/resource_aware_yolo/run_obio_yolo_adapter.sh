#!/usr/bin/env bash
# -*- coding: utf-8 -*-

# ==============================================================================
# Resource-Aware YOLO Demo, powered by OBIO
#
# Default:
#   ./run_obio_yolo_adapter.sh
#
# Optional OBIO core mode:
#   ./run_obio_yolo_adapter.sh obio-core
#
# This demo does not require a real camera, UAV, PX4 SITL, or GPU.
# It runs a synthetic 30 Hz camera stream and a deterministic OBIO pressure
# sequence to show GREEN -> YELLOW -> RED -> hysteresis recovery behavior.
# ==============================================================================

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_MODE="${1:-adapter-only}"   # adapter-only | obio-core
PACKAGE_NAME="ai_flight_integrity_observer"

HYSTERESIS_SEC="${HYSTERESIS_SEC:-2.0}"
GREEN_SEC="${GREEN_SEC:-6}"
YELLOW_SEC="${YELLOW_SEC:-6}"
RED_SEC="${RED_SEC:-6}"
RECOVERY_SEC="${RECOVERY_SEC:-8}"

PIDS=()
TMP_FILES=()

log()  { echo -e "\033[94m[OBIO INFO]\033[0m $*"; }
ok()   { echo -e "\033[92m[OBIO OK]\033[0m $*"; }
warn() { echo -e "\033[93m[OBIO WARN]\033[0m $*"; }
err()  { echo -e "\033[91m[OBIO ERROR]\033[0m $*"; }

cleanup() {
  trap - EXIT INT TERM
  echo
  warn "Cleaning up background demo processes..."

  # First pass: terminate process groups created by setsid.
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid}" ]]; then
      kill -TERM -- "-${pid}" >/dev/null 2>&1 || kill -TERM "${pid}" >/dev/null 2>&1 || true
    fi
  done

  sleep 1.0

  # Second pass: force-kill remaining process groups.
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid}" ]]; then
      kill -KILL -- "-${pid}" >/dev/null 2>&1 || kill -KILL "${pid}" >/dev/null 2>&1 || true
    fi
  done

  for f in "${TMP_FILES[@]:-}"; do
    [[ -f "${f}" ]] && rm -f "${f}" || true
  done

  ok "Cleanup complete."
}
trap cleanup EXIT INT TERM

source_ros() {
  if command -v ros2 >/dev/null 2>&1; then
    return 0
  fi

  for distro in humble jazzy iron rolling; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      # shellcheck disable=SC1090
      source "/opt/ros/${distro}/setup.bash"
      break
    fi
  done

  if ! command -v ros2 >/dev/null 2>&1; then
    err "ROS 2 is not sourced. Please source /opt/ros/<distro>/setup.bash first."
    exit 1
  fi
}

source_workspace() {
  if ros2 pkg prefix "${PACKAGE_NAME}" >/dev/null 2>&1; then
    return 0
  fi

  local candidates=(
    "${OBIO_WS:-}/install/setup.bash"
    "${SCRIPT_DIR}/../../../../install/setup.bash"
    "${HOME}/px4_ros2_ws/install/setup.bash"
    "${HOME}/ros2_ws/install/setup.bash"
  )

  for setup in "${candidates[@]}"; do
    if [[ -n "${setup}" && -f "${setup}" ]]; then
      # shellcheck disable=SC1090
      source "${setup}"
      if ros2 pkg prefix "${PACKAGE_NAME}" >/dev/null 2>&1; then
        return 0
      fi
    fi
  done

  err "Could not find ROS 2 package: ${PACKAGE_NAME}"
  echo
  echo "Build and source the workspace first, for example:"
  echo "  cd ~/px4_ros2_ws"
  echo "  colcon build --symlink-install --merge-install"
  echo "  source install/setup.bash"
  echo
  echo "Or set:"
  echo "  export OBIO_WS=/path/to/your/workspace"
  exit 1
}

start_bg() {
  local label="$1"
  shift

  log "Starting: ${label}"

  if command -v setsid >/dev/null 2>&1; then
    setsid "$@" &
  else
    warn "setsid not found; falling back to normal background process for ${label}"
    "$@" &
  fi

  local pid=$!
  PIDS+=("${pid}")
  sleep 0.8
}

start_bg_quiet() {
  local label="$1"
  shift

  local log_file="/tmp/obio_yolo_demo_${label//[^A-Za-z0-9_]/_}.log"
  log "Starting: ${label}  [log: ${log_file}]"

  if command -v setsid >/dev/null 2>&1; then
    setsid "$@" >"${log_file}" 2>&1 &
  else
    warn "setsid not found; falling back to normal background process for ${label}"
    "$@" >"${log_file}" 2>&1 &
  fi

  local pid=$!
  PIDS+=("${pid}")
  sleep 0.8
}

write_synthetic_stimulus_py() {
  local f
  f="$(mktemp /tmp/obio_yolo_stimulus_XXXXXX.py)"
  TMP_FILES+=("${f}")

  cat > "${f}" <<'PY'
#!/usr/bin/env python3
import os
import time
from array import array

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image


class ObioYoloSyntheticStimulus(Node):
    def __init__(self):
        super().__init__("obio_yolo_synthetic_stimulus")

        self.green_sec = float(os.environ.get("OBIO_DEMO_GREEN_SEC", "6"))
        self.yellow_sec = float(os.environ.get("OBIO_DEMO_YELLOW_SEC", "6"))
        self.red_sec = float(os.environ.get("OBIO_DEMO_RED_SEC", "6"))
        self.recovery_sec = float(os.environ.get("OBIO_DEMO_RECOVERY_SEC", "8"))

        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.phase_start = time.monotonic()
        self.loop_start = self.phase_start
        self.last_phase_print = None

        self.image_timer = self.create_timer(1.0 / 30.0, self.publish_image)
        self.diag_timer = self.create_timer(0.2, self.publish_diag)

        self.get_logger().info(
            "Synthetic stimulus started: 30 Hz camera + deterministic OBIO pressure sequence."
        )

    def current_phase(self):
        elapsed = (time.monotonic() - self.loop_start)
        total = self.green_sec + self.yellow_sec + self.red_sec + self.recovery_sec
        t = elapsed % total

        if t < self.green_sec:
            return "GREEN_BASELINE", t, self.green_sec, 0.0, "None", "FLIGHT_ALIGNED"

        t -= self.green_sec
        if t < self.yellow_sec:
            return "YELLOW_PRESSURE", t, self.yellow_sec, 70.0, "None", "SETPOINT_JITTER"

        t -= self.yellow_sec
        if t < self.red_sec:
            return "RED_STARVATION", t, self.red_sec, 140.0, "SETPOINT_STALE", "STALE_STREAM"

        t -= self.red_sec
        return "RECOVERY_HEALTHY", t, self.recovery_sec, 0.0, "None", "FLIGHT_ALIGNED"

    def publish_image(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "obio_demo_camera"
        msg.height = 1
        msg.width = 1
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = 3
        msg.data = array("B", [0, 0, 0])
        self.image_pub.publish(msg)

    def publish_diag(self):
        phase, t, dur, jitter, stale, cause = self.current_phase()

        status = DiagnosticStatus()
        status.name = "obio_demo/boundary_pressure"
        status.message = cause

        if cause == "FLIGHT_ALIGNED":
            status.level = DiagnosticStatus.OK
        elif cause == "SETPOINT_JITTER":
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.ERROR

        status.values = [
            KeyValue(key="setpointJitterMs", value=f"{jitter:.1f}"),
            KeyValue(key="staleStreams", value=str(stale)),
            KeyValue(key="dominantCause", value=str(cause)),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.diag_pub.publish(arr)

        # Print once per second so the user can see the intended phase.
        sec_left = int(max(0.0, dur - t))
        phase_print = (phase, sec_left)
        if phase_print != self.last_phase_print:
            self.last_phase_print = phase_print
            self.get_logger().info(
                f"Phase={phase} | jitter={jitter:.1f}ms | stale={stale} | cause={cause} | ~{sec_left}s left"
            )


def main():
    rclpy.init()
    node = ObioYoloSyntheticStimulus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
PY

  chmod +x "${f}"
  echo "${f}"
}

print_banner() {
  ok "=================================================================="
  ok " Resource-Aware YOLO Adaptive Scaling Demo  |  Powered by OBIO"
  ok "=================================================================="
  echo "Mode: ${DEMO_MODE}"
  echo "Hysteresis: ${HYSTERESIS_SEC}s"
  echo "Pressure sequence: GREEN ${GREEN_SEC}s -> YELLOW ${YELLOW_SEC}s -> RED ${RED_SEC}s -> RECOVERY ${RECOVERY_SEC}s"
  echo
}

source_ros
source_workspace
print_banner

# Start adapter first so it does not miss the initial synthetic diagnostics.
start_bg "obio_yolo_adapter" \
  ros2 run "${PACKAGE_NAME}" obio_yolo_adapter --ros-args \
    -p mode:=throttle \
    -p input_image_topic:=/camera/image_raw \
    -p output_image_topic:=/obio/image_for_yolo \
    -p green_frame_stride:=1 \
    -p yellow_frame_stride:=2 \
    -p red_frame_stride:=4 \
    -p hysteresis_sec:="${HYSTERESIS_SEC}" \
    -p profile_publish_hz:=1.0 \
    -p enable_color_log:=True

if [[ "${DEMO_MODE}" == "obio-core" ]]; then
  warn "Mode obio-core: launching OBIO core nodes. Boundary state depends on fake_slam_stressor_node behavior."

  start_bg_quiet "synthetic_camera_30hz" \
    ros2 topic pub --rate 30 /camera/image_raw sensor_msgs/msg/Image \
    "{header: {frame_id: 'demo_camera'}, height: 1, width: 1, encoding: 'rgb8', is_bigendian: 0, step: 3, data: [0, 0, 0]}"

  start_bg_quiet "flight_integrity_node" \
    ros2 run "${PACKAGE_NAME}" flight_integrity_node

  start_bg_quiet "synthetic_px4_publisher" \
    ros2 run "${PACKAGE_NAME}" synthetic_px4_publisher --ros-args -p profile:=normal

  start_bg "fake_slam_stressor_node" \
    ros2 run "${PACKAGE_NAME}" fake_slam_stressor_node

else
  warn "Mode adapter-only: launching deterministic synthetic camera + OBIO pressure stimulus."
  stimulus_py="$(write_synthetic_stimulus_py)"

  OBIO_DEMO_GREEN_SEC="${GREEN_SEC}" \
  OBIO_DEMO_YELLOW_SEC="${YELLOW_SEC}" \
  OBIO_DEMO_RED_SEC="${RED_SEC}" \
  OBIO_DEMO_RECOVERY_SEC="${RECOVERY_SEC}" \
  start_bg "synthetic_camera_and_pressure_stimulus" \
    python3 "${stimulus_py}"
fi

ok "------------------------------------------------------------------"
echo "Demo is running."
echo
echo "Open another terminal to inspect throttling:"
echo "  ros2 topic hz /camera/image_raw"
echo "  ros2 topic hz /obio/image_for_yolo"
echo
echo "Expected:"
echo "  GREEN    -> /obio/image_for_yolo ≈ 30 Hz"
echo "  YELLOW   -> /obio/image_for_yolo ≈ 15 Hz"
echo "  RED      -> /obio/image_for_yolo ≈ 7.5 Hz"
echo "  RECOVERY -> remains reduced until hysteresis passes, then returns to GREEN"
echo
echo "Foreground profile stream:"
ok "------------------------------------------------------------------"

# Foreground monitor. Ctrl+C exits and triggers cleanup.
ros2 topic echo /obio/yolo_profile
