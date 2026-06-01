# AI Flight Integrity Observer

> **A ROS 2 / PX4 runtime observer for flight execution integrity under AI and companion-compute load.**
> Quantify whether AI-generated offboard control intent is physically realized in vehicle response.

Modern drones increasingly rely on companion computers for:

```text
VIO
target tracking
obstacle avoidance
edge AI perception
semantic navigation
AI-assisted offboard autonomy
```

But heavy AI workloads can silently degrade the offboard control boundary:

```text
setpoint publish jitter
stale trajectory commands
delayed vehicle odometry
estimator lag
GPS / VIO jumps
command-response desynchronization
```

This project does **not** intercept flight control.

It observes whether offboard setpoints are physically realized by the vehicle, and exposes flight execution collapse through standard ROS 2 diagnostics.

````text
setpoint publish jitter
stale trajectory commands
delayed vehicle odometry
estimator lag
```
GPS /Initial target:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/out/vehicle_odometry
        ↓
/diagnostics
  ai_flight_integrity/flight_execution_integrity
````

---

## Current Status

`ai_flight_integrity_observer` is currently at **v0.1-alpha**.

The first version provides:

```text
observe-only flight integrity diagnostics
PX4-style synthetic publisher for smoke testing
standard ROS /diagnostics output
Best Effort / Volatile QoS for PX4-facing topics
load-metadata placeholders for future AI stress testing
```

It currently detects or exposes:

```text
SETPOINT_JITTER
SETPOINT_STALE
OFFBOARD_STALE
ODOMETRY_STALE
COMMAND_RESPONSE_MISMATCH
POSITION_RESPONSE_MISMATCH
GPS_VIO_JUMP
MISSING_STREAM
```

---

## What It Is Not

This project is not a flight controller.

It does not:

```text
publish flight commands
arm or disarm the vehicle
replace PX4 failsafe logic
modify PX4
modify offboard control code
guarantee flight safety
```

It is a passive runtime observer.

---

## Repository Layout

Expected package layout:

```text
ai_flight_integrity_observer/
├── README.md
├── package.xml
├── setup.py
├── resource/
│   └── ai_flight_integrity_observer
├── launch/
│   └── flight_integrity_observer.launch.py        # optional
└── ai_flight_integrity_observer/
    ├── __init__.py
    ├── px4_qos.py
    ├── flight_integrity_node.py
    └── synthetic_px4_publisher.py
```

Before building, make sure these files exist:

```text
resource/ai_flight_integrity_observer
ai_flight_integrity_observer/__init__.py
```

---

## Quick Start A: Synthetic Smoke Test

This test does **not** require PX4 SITL, Gazebo, Micro XRCE-DDS Agent, or a real UAV.

It only verifies that the observer, PX4-style messages, QoS, diagnostics, and fault profiles are wired correctly.

### 1. Create a ROS 2 workspace

```bash
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws/src
```

### 2. Clone dependencies and this repository

```bash
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/ZC502/ai_flight_integrity_observer.git
```

Make sure the `px4_msgs` branch matches the PX4 firmware / SITL version you plan to use later.

### 3. Build

```bash
cd ~/px4_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Terminal 1: Start the observer

```bash
ros2 run ai_flight_integrity_observer flight_integrity_node
```

### 5. Terminal 2: Start the synthetic PX4 publisher

Normal mode:

```bash
ros2 run ai_flight_integrity_observer synthetic_px4_publisher --ros-args \
  -p profile:=normal
```

Fault modes:

```bash
ros2 run ai_flight_integrity_observer synthetic_px4_publisher --ros-args \
  -p profile:=setpoint_jitter
```

```bash
ros2 run ai_flight_integrity_observer synthetic_px4_publisher --ros-args \
  -p profile:=command_response_mismatch
```

```bash
ros2 run ai_flight_integrity_observer synthetic_px4_publisher --ros-args \
  -p profile:=gps_vio_jump
```

### 6. Terminal 3: Inspect diagnostics

```bash
ros2 topic echo /diagnostics --once --full-length
```

A more compact diagnostic view:

```bash
ros2 topic echo /diagnostics --once --full-length \
| awk '
/message:/ {print}
/- key: diagnosticLevelName$/ ||
/- key: status$/ ||
/- key: dominantCause$/ ||
/- key: totalResidual$/ ||
/- key: flightResidual$/ ||
/- key: setpointJitterMs$/ ||
/- key: velocityTrackingResidual$/ ||
/- key: gpsVioJumpMetric$/ ||
/- key: staleStreams$/ ||
/- key: statsEvaluations$/ {
  print
  getline
  print
}'
```

Expected results:

```text
normal                    → OK    | GREEN: FLIGHT_ALIGNED
setpoint_jitter           → ERROR | RESYNCING: SETPOINT_JITTER
command_response_mismatch → ERROR | RESYNCING: COMMAND_RESPONSE_MISMATCH
gps_vio_jump              → ERROR | RESYNCING: GPS_VIO_JUMP
```

---

## Quick Start B: PX4 SITL Path

The synthetic test is only the first smoke test.

For real PX4 validation, you need:

```text
PX4 SITL
Micro XRCE-DDS Agent
ROS 2
px4_msgs matching your PX4 version
```

The intended PX4-facing topics are:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/out/vehicle_odometry
```

Optional future input:

```text
/fmu/out/estimator_status
```

Once PX4 SITL and the DDS bridge are running, verify that odometry is visible:

```bash
ros2 topic echo /fmu/out/vehicle_odometry --once
```

Then run the observer:

```bash
ros2 run ai_flight_integrity_observer flight_integrity_node --ros-args \
  -p trajectory_setpoint_topic:=/fmu/in/trajectory_setpoint \
  -p offboard_control_mode_topic:=/fmu/in/offboard_control_mode \
  -p vehicle_odometry_topic:=/fmu/out/vehicle_odometry
```

---

## PX4 QoS Warning

PX4 high-rate topics often use sensor-style QoS.

If your subscriber uses the default ROS 2 Reliable QoS, you may see the topic in:

```bash
ros2 topic list
```

but still receive no messages.

This observer explicitly uses:

```text
Best Effort
Volatile
Keep Last
```

for PX4-facing topics.

This is intentional.

---

## px4_msgs Dependency

This package depends on `px4_msgs`.

Clone and build `px4_msgs` in the same ROS 2 workspace:

```bash
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws/src

git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/ZC502/ai_flight_integrity_observer.git

cd ~/px4_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Important:

```text
The px4_msgs branch should match your PX4 firmware / SITL version.
```

Also note:

```text
Only PX4 topics listed in the firmware dds_topics.yaml are exposed through uXRCE-DDS.
```

If a topic is missing, check the PX4 `dds_topics.yaml` configuration.

---

## Time Synchronization Warning

Flight-integrity metrics involving latency, age, or jitter are only meaningful when timestamp semantics are understood.

PX4 messages often use PX4-side monotonic timestamps, while ROS 2 nodes may use companion-computer time.

This observer primarily reports ROS receive-time freshness and jitter for v0.1-alpha.

For hardware tests, ensure tight time synchronization between the flight controller and companion computer, or interpret absolute delay metrics as approximate.

Unsynchronized clocks can produce false diagnostics such as:

```text
SETPOINT_STALE
ODOMETRY_STALE
OFFBOARD_STALE
TIMING_JITTER
```

For hardware deployments, consider:

```text
Chrony
PTP
hardware timestamping
PX4 timesync analysis
```

---

## AI Load Metadata

The v0.1-alpha diagnostic schema includes placeholder fields for future AI-load correlation:

```text
loadProfile
cpuLoadPercent
gpuLoadPercent
npuLoadPercent
aiInferenceLatencyMs
```

In the current version these are manually supplied parameters.

Example:

```bash
ros2 run ai_flight_integrity_observer flight_integrity_node --ros-args \
  -p load_profile:=synthetic_yolo_stress \
  -p cpu_load_percent:=85.0 \
  -p gpu_load_percent:=70.0 \
  -p npu_load_percent:=60.0 \
  -p ai_inference_latency_ms:=45.0
```

Future versions may integrate with platform profilers or AI stress injectors.

---

## Diagnostic Output Example

Example `COMMAND_RESPONSE_MISMATCH`:

```yaml
message: "ERROR | RESYNCING: COMMAND_RESPONSE_MISMATCH"
name: "ai_flight_integrity/flight_execution_integrity"
hardware_id: "px4_offboard_physics_boundary_observer"
values:
  - key: "diagnosticLevelName"
    value: "ERROR"
  - key: "status"
    value: "RESYNCING"
  - key: "dominantCause"
    value: "COMMAND_RESPONSE_MISMATCH"
  - key: "totalResidual"
    value: "1.428000"
  - key: "flightResidual"
    value: "1.428000"
  - key: "velocityTrackingResidual"
    value: "0.900000"
  - key: "setpointJitterMs"
    value: "0.00"
  - key: "gpsVioJumpMetric"
    value: "0.000000"
  - key: "offboardActive"
    value: "true"
```

---

## Why This Exists

As UAVs integrate heavier companion-compute workloads for VIO, tracking, obstacle avoidance, and edge AI perception, the offboard control boundary becomes a critical failure surface.

The important question is not only:

```text
Is the AI model running?
```

but also:

```text
Is the vehicle physically realizing the setpoints produced under that AI load?
```

`AI Flight Integrity Observer` is designed to make that boundary visible.

---

## License

Apache-2.0

```
```
