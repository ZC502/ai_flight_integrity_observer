# Offboard Boundary Integrity Observer (OBIO)
*Formerly AFIO — Autonomy Flight Integrity Observer*
> **A deterministic ROS 2 / PX4 runtime assurance observer for compute-heavy Offboard autonomy stacks.**


[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue)](https://docs.ros.org/en/humble/index.html)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-v1.14%20%7C%20v1.15-orange)](https://px4.io/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](https://opensource.org/licenses/Apache-2.0)

### The Missing Trigger for Autonomy Load-Shedding

Modern UAV autonomy stacks increasingly run VIO, LVI, SLAM, mapping, visual tracking, neural planning, and dense perception workloads on edge hardware (Jetson, RK3588, NPU, custom companion computers).

When pushing these systems to their limits, autonomy managers need to know when to dynamically shed load (e.g., lower camera resolution, pause loop closure, switch to a lighter fallback model).

Currently, most adaptive compute policies use indirect hardware signals: `CPU %`, `GPU %`, `memory usage`, or `temperature`.

These signals are useful, but they do not answer the ultimate flight-control question. Using `CPU %` as the sole trigger for load-shedding is often lagging and fundamentally flawed:

- A well-optimized pipeline can run at **99% CPU** while the Offboard control stream remains perfectly healthy and on time.
- Conversely, a system might show only **60% CPU** load, but due to thread starvation, mutex lock contention, sensor synchronization bottlenecks, or bus stalls, the control publisher is blocked, delaying the Offboard setpoint by a lethal 150–300 ms.

**OBIO provides the missing semantic trigger**. Instead of asking "*Is the companion computer busy?*", OBIO asks: "*Has the companion workload begun to pollute the real-time flight control boundary?*"

### Low-Overhead Boundary Health Signals

OBIO does not perform scheduling, model pruning, perception, mapping, or flight control by itself. It is a low-overhead, zero-intrusive boundary observer.

It passively watches the ROS 2 / PX4 boundary:

- `/fmu/in/trajectory_setpoint`
- `/fmu/in/offboard_control_mode`
- `/fmu/out/vehicle_odometry`

It publishes standard `/diagnostics` and CSV labels that your existing lifecycle scripts or autonomy managers can consume:
- `setpointAgeMs`
- `setpointJitterMs`
- `staleStreams`
- `flightResidual` (NARH-inspired physical-temporal consistency score)
- `dominantCause`

### Use Case: LVI / VIO / SLAM Dynamic Load-Shedding

OBIO does not optimize LVI algorithms internally. It tells your autonomy manager exactly when the LVI load is violating the flight-control timing margins.

If `setpointJitterMs` **rises above threshold (Control-path pressure detected)**:
- Reduce visual tracking framerate or input resolution.
- Pause global SLAM loop closure.
- Restrict non-linear optimization iterations (e.g., in IESKF).
- Protect the RTOS scheduling priority of the control-intent publisher thread.

If `flightResidual` **spikes (Physical tracking divergence imminent)**:
- Transition into a conservative trajectory mode.
- Fall back to a lightweight state-estimation pipeline (e.g., pure IMU dead-reckoning).
- Flag the timestamp for post-flight incident analysis.

---

## Why This Exists: Two Reliability Surfaces

### A. Degraded Communication / RF / Telemetry Link Volatility

In degraded or contested communication environments, the Offboard stream may not simply disappear. It may become bursty, stale, or highly jittered while still producing syntactically valid messages.

AFIO exposes this early through:

```text
setpointAgeMs
setpointJitterMs
staleStreams
causalAlignment
```

External autonomy managers can use these signals to trigger runtime-assurance responses such as:

```text
slow down aggressive maneuvers
enter Hold / Loiter / Cruise mode
prioritize control telemetry over high-bandwidth video
switch to a backup control link
increase link robustness before hard stream loss
```

AFIO does not implement these recovery actions itself. It provides the observable boundary signal.

### B. Edge-AI / Vision Stack Overload

VIO, visual tracking, object detection, neural planning, mapping, and semantic navigation can create bursty companion-compute load. When perception inference latency spikes, the Offboard setpoint publisher may become delayed or irregular even while PX4 itself remains healthy.

AFIO turns that hidden compute-side degradation into visible diagnostics:

```text
SETPOINT_JITTER
SETPOINT_STALE
STALE_STREAM
COMMAND_RESPONSE_MISMATCH
POSITION_RESPONSE_MISMATCH
```

A companion-compute orchestrator can subscribe to `/diagnostics` and perform load-shedding:

```text
skip frames
reduce perception resolution
pause non-critical loop closure
switch from a heavy model to a lightweight fallback
reserve CPU affinity / scheduling priority for control publishing
```

---

## Architecture: Blind Boundary Observation

```text
+-------------------------------------------------------------+
|                     Companion Computer                      |
|                                                             |
|   VIO / tracking / neural planner / edge-AI perception      |
|                         │                                   |
|                         ▼                                   |
|              /ai/raw_trajectory_setpoint                    |
|                         │                                   |
|              [ ai_latency_injector_node ]                   |
|                         │                                   |
+-------------------------+-----------------------------------+
                          │
                          ▼
              /fmu/in/trajectory_setpoint
              /fmu/in/offboard_control_mode
                          │
+-------------------------+-----------------------------------+
|                    PX4 / ROS 2 Boundary                     |
|                                                             |
|        [ AFIO: flight_integrity_node ]                      |
|                                                             |
|        Observes:                                            |
|          /fmu/in/trajectory_setpoint                        |
|          /fmu/in/offboard_control_mode                      |
|          /fmu/out/vehicle_odometry                          |
|                                                             |
|        Publishes:                                           |
|          /diagnostics                                       |
|          CSV labels                                         |
+-------------------------+-----------------------------------+
                          │
                          ▼
+-------------------------------------------------------------+
|                         PX4 Autopilot                       |
|                 SITL or hardware flight core                |
+-------------------------------------------------------------+
```

The injector is only a test tool. The observer is blind to it. In a real deployment, the same observed degradation may come from AI inference stalls, overloaded companion compute, middleware jitter, telemetry loss, or estimator/odometry delays.

---

## NARH-Lite: Residual Auditing Principle

AFIO is inspired by the **Non-Associative Residual Hypothesis (NARH)**, originally formulated for discrete rigid-body simulation pipelines.

In the original NARH framing, a discrete solver may apply constraint or correction operators in different internal orders because of batching, projection, thread scheduling, or finite-precision effects. The resulting **Non-Associative Residual (NAR)** measures order-dependent deviation introduced by the numerical pipeline. It is not a claim that the physical state space itself is non-associative.

AFIO does not claim to inspect PX4's internal solver order. Instead, it applies the same residual-auditing idea at the ROS 2 / PX4 Offboard boundary:

```text
intent stream      = trajectory setpoint
feedback stream    = vehicle odometry
freshness stream   = message age, jitter, missing/stale status
semantic mode      = position / velocity / mixed Offboard mode
```

AFIO computes a boundary residual bundle:

```text
R_boundary = w_t R_timing
           + w_p R_position
           + w_v R_velocity
           + w_s R_stream
```

where:

```text
R_timing   → setpoint age / jitter / stale stream residual
R_position → position-tracking residual under position-mode semantics
R_velocity → velocity-tracking residual under velocity-mode semantics
R_stream   → missing or stale Offboard / odometry streams
```

The goal is not to prove that PX4 or ROS 2 is mathematically invalid. The goal is practical runtime assurance:

```text
Detect when a previously aligned Offboard boundary begins to exhibit structured residual growth.
```

---

## What OBIO Is Not

OBIO is not a flight controller and does not guarantee flight safety.

It does not:

```text
arm or disarm the vehicle
publish recovery setpoints
replace PX4 failsafe logic
modify PX4
modify the Offboard controller
claim real AI workload equivalence from synthetic latency alone
```

It is a passive observer and label generator for runtime assurance, regression testing, and post-flight analysis.

---

### Quick Start A: PX4 SITL Path

For real PX4 validation, you need:
```
PX4 SITL
Micro XRCE-DDS Agent
ROS 2
px4_msgs matching your PX4 version
```
The intended PX4-facing topics are:
```
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/out/vehicle_odometry
```
Optional future input:
```
/fmu/out/estimator_status
```
Once PX4 SITL and the DDS bridge are running, verify that odometry is visible:
```
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash

ros2 topic echo /fmu/out/vehicle_odometry --once
```
Then run the observer:
```
ros2 run ai_flight_integrity_observer flight_integrity_node --ros-args \
  -p trajectory_setpoint_topic:=/fmu/in/trajectory_setpoint \
  -p offboard_control_mode_topic:=/fmu/in/offboard_control_mode \
  -p vehicle_odometry_topic:=/fmu/out/vehicle_odometry
```
If no PX4 messages are received, check:
```
ros2 topic list | grep /fmu
ros2 topic info /fmu/out/vehicle_odometry -v
```
PX4-facing subscriptions in this observer use Best Effort / Volatile QoS to avoid the common PX4 QoS mismatch trap.

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

## Quick Start B: Synthetic Smoke Test

This test does **not** require PX4 SITL, Gazebo, Micro XRCE-DDS Agent, or a real UAV.

It verifies that the observer, PX4-style messages, QoS, diagnostics, fault profiles, and CSV label export are wired correctly.

**1. Create a ROS 2 workspace**
```
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws/src
```
**2. Clone dependencies and this repository**
```
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/ZC502/ai_flight_integrity_observer.git
```
Make sure the `px4_msgs` branch matches the PX4 firmware / SITL version you plan to use later.

**3. Build**
```
cd ~/px4_ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install --merge-install

source install/setup.bash
```
Verify that the package and executables are visible:
```bash
ros2 pkg list | grep ai_flight_integrity_observer
ros2 pkg executables ai_flight_integrity_observer
```
Expected executables:
```
ai_flight_integrity_observer flight_integrity_node
ai_flight_integrity_observer synthetic_px4_publisher
ai_flight_integrity_observer flight_diagnostics_to_csv_labeler
```
**4. Terminal 1: Start the observer**
```Bash
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run ai_flight_integrity_observer flight_integrity_node
```
Expected startup log:
```Plaintext
AI Flight Integrity Observer started | setpoint=/fmu/in/trajectory_setpoint | offboard_mode=/fmu/in/offboard_control_mode | odometry=/fmu/out/vehicle_odometry | diagnostics=/diagnostics
```
**5. Terminal 2: Start the synthetic PX4 publisher**

Normal mode:
```Bash
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run ai_flight_integrity_observer synthetic_px4_publisher --ros-args \
  -p profile:=normal
```
Fault modes:
```
ros2 run ai_flight_integrity_observer synthetic_px4_publisher --ros-args \
  -p profile:=setpoint_jitter \
  -p fault_duration_sec:=9999.0
ros2 run ai_flight_integrity_observer synthetic_px4_publisher --ros-args \
  -p profile:=command_response_mismatch \
  -p fault_duration_sec:=9999.0
ros2 run ai_flight_integrity_observer synthetic_px4_publisher --ros-args \
  -p profile:=gps_vio_jump \
  -p fault_duration_sec:=9999.0
```
Why `fault_duration_sec:=9999.0`?

Short fault windows can end before you inspect `/diagnostics`. A long fault duration makes the failure state easier to observe in the terminal or dashboard.

**6. Terminal 3: Inspect diagnostics**
```Bash
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo /diagnostics --once --full-length
```
A more compact diagnostic view:
```Bash
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
/- key: positionTrackingResidual$/ ||
/- key: gpsVioJumpMetric$/ ||
/- key: staleStreams$/ ||
/- key: statsEvaluations$/ {
  print
  getline
  print
}'
```
Expected results:
```
normal                    → OK    | GREEN: FLIGHT_ALIGNED
setpoint_jitter           → ERROR | RESYNCING: SETPOINT_JITTER
command_response_mismatch → ERROR | RESYNCING: COMMAND_RESPONSE_MISMATCH or POSITION_RESPONSE_MISMATCH
gps_vio_jump              → ERROR | RESYNCING: GPS_VIO_JUMP
```
Note on `command_response_mismatch`:

During the active fault window, the vehicle velocity intentionally fails to track the setpoint, so the observer may report:
```
COMMAND_RESPONSE_MISMATCH
```
After the velocity recovers, the vehicle may still lag behind the setpoint position. In that case, the observer correctly reports:
```
POSITION_RESPONSE_MISMATCH
```
This means the observer is detecting the remaining physical execution error, not that the test failed.

---

### CSV Failure Label Export

The observer publishes flight execution-integrity events to `/diagnostics`.

For ML / Sim2Real / AI-load regression workflows, the included CSV labeler converts those diagnostics into machine-readable failure labels.

**Terminal 4: Start the CSV labeler**
```Bash
source /opt/ros/humble/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run ai_flight_integrity_observer flight_diagnostics_to_csv_labeler --ros-args \
  -p output_csv:=flight_integrity_labels.csv
```
Inspect the CSV:
```Bash
head -n 5 flight_integrity_labels.csv
tail -f flight_integrity_labels.csv
```
Example labels:
```
ros_time_sec,diagnostic_level_name,status,dominantCause,totalResidual,flightResidual,velocityTrackingResidual,gpsVioJumpMetric,setpointJitterMs
1779800001.12,ERROR,RESYNCING,COMMAND_RESPONSE_MISMATCH,1.428000,1.428000,0.900000,0.000000,0.00
1779800004.54,ERROR,RESYNCING,GPS_VIO_JUMP,3.210000,3.210000,0.000000,2.100000,0.00
```
These labels can be used for:
- AI-load regression testing
- Sim2Real failure mining
- offboard setpoint failure datasets
- OOD event detection
- post-flight incident review
