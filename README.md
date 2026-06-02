# AI Flight Integrity Observer
>**A ROS 2 / PX4 runtime observer for flight execution integrity under AI and companion-compute load.**

**Quantify whether AI-generated offboard control intent is physically realized in vehicle response.**

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

GPS /Initial target:
```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/out/vehicle_odometry
        ↓
/diagnostics
  ai_flight_integrity/flight_execution_integrity
```

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

### Repository Layout

Expected package layout:
```
ai_flight_integrity_observer/
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── ai_flight_integrity_observer
├── launch/
│   └── flight_integrity_observer.launch.py        # optional
└── ai_flight_integrity_observer/
    ├── __init__.py
    ├── px4_qos.py
    ├── flight_integrity_node.py
    ├── synthetic_px4_publisher.py
    └── flight_diagnostics_to_csv_labeler.py
```
Before building, make sure these files exist:
```
setup.cfg
resource/ai_flight_integrity_observer
ai_flight_integrity_observer/__init__.py
```
`setup.cfg` is required for ROS 2 `ament_python` console scripts. Without it, the package may build successfully, but `ros2 run` may report:
```
No executable found
```

---

## Quick Start A: Synthetic Smoke Test

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

---

### Quick Start B: PX4 SITL Path

The synthetic test is only the first smoke test.

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
