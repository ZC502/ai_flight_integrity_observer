# Resource-Aware YOLO

_powered by OBIO — Offboard Boundary Integrity Observer_

**Run bigger YOLO when the robot is healthy.**  
**Back off before vision load starves control.**

OBIO turns the physical control boundary into a runtime pressure signal for edge-AI workloads. Instead of triggering load-shedding from CPU%, GPU%, or memory usage, OBIO watches the actual boundary between perception compute and physical execution: setpoint jitter, stale streams, offboard freshness, and command/response residuals.

The result is a simple deployment pattern:

```text
YOLO / VLM / vision workload
        ↓
Resource-Aware Adapter
        ↓
physical-boundary-aware load behavior
        ↓
robot, gate, AMR, drone, arm, or vehicle control path
```

## The Problem: Vision AI Is Usually Blind to Physical Consequences

YOLO pipelines are excellent at reading pixels, but they usually do not know whether their own compute spikes are damaging the downstream physical system.

In a normal digital pipeline, a 200–300 ms inference stall may look like a harmless delay. In a physical system, the same stall can starve a control stream, delay an actuator decision, or push a robot into an unsafe recovery state.

This shows up across many YOLO deployment surfaces:

```text
Drones and PX4 Offboard control
 perception spikes can starve fresh setpoints.

AMR / AGV / mobile robots
  vision load can compete with navigation and control callbacks.

Industrial inspection and robotic sorting
  delayed detections can desynchronize vision and actuator timing.

Security gates, logistics choke points, and crowd-flow systems
  camera AI may need to coordinate with physical gates, alarms, or access-control hardware.

Autonomous driving and edge perception stacks
  high-load scenes can create latency exactly when physical reaction time matters most.
```

Hardware utilization is not enough. A system can show moderate CPU usage while a ROS 2 executor, DDS path, mutex, callback queue, or setpoint publisher is already starved. OBIO is built for that gap.

## The Solution: Resource-Aware YOLO, Powered by OBIO

OBIO is a passive boundary observer. It does not optimize YOLO, fly the robot, control actuators, or replace failsafes. It provides the missing signal: **is the physical execution boundary still healthy while vision compute is running?**

The `obio_yolo_adapter` consumes OBIO diagnostics and turns any ROS 2 YOLO pipeline into a resource-aware pipeline:

```text
GREEN
  boundary is healthy
  full camera rate / high-resolution YOLO profile

YELLOW
  setpoint jitter or timing pressure is rising
  reduce image forwarding rate or move to a medium profile

RED
  stale stream or severe boundary pressure
  aggressive frame skipping or emergency low-load profile

RECOVERY
  return to GREEN only after the boundary remains healthy for a hysteresis window
```

The safe default is **topic throttling**:

```text
/camera/image_raw
        ↓
[ obio_yolo_adapter ]
        ↓
/obio/image_for_yolo
        ↓
YOLO node
```

YOLO does not need to know that OBIO exists. It simply subscribes to the adapter’s forwarded image topic. If a specific YOLO ROS node safely supports runtime parameter updates, the adapter can also use ROS 2 Parameter Service to adjust parameters such as `imgsz`.

## Demo Path

This repository provides two complementary demo layers:

```text
1. Resource-Aware YOLO Adapter Demo
   Shows the state machine, hysteresis, and image-topic throttling without a real camera or drone.

2. OBIO Core Boundary-Pressure Demo
   Shows why CPU% is a deceptive trigger: high CPU can be safe, and low CPU can still hide setpoint starvation.
```

The two demos should be read together:

```text
OBIO Core Demo proves the signal.
Resource-Aware YOLO Demo shows how to consume the signal.
```

---

## Integrating With YOLO

The default integration is zero-intrusive topic throttling.

Point your YOLO node to the adapter output instead of the raw camera topic:

```text
Before:
  YOLO subscribes to /camera/image_raw

After:
  YOLO subscribes to /obio/image_for_yolo
```

Run the adapter:

```bash
ros2 run ai_flight_integrity_observer obio_yolo_adapter --ros-args \
  -p mode:=throttle \
  -p input_image_topic:=/camera/image_raw \
  -p output_image_topic:=/obio/image_for_yolo
```

This works with any ROS 2 YOLO pipeline that consumes `sensor_msgs/msg/Image`. YOLO does not need to be modified.

For YOLO wrappers that safely support runtime parameter updates, the adapter can also run in optional parameter-control mode:

```bash
ros2 run ai_flight_integrity_observer obio_yolo_adapter --ros-args \
  -p mode:=param \
  -p target_yolo_node:=/yolo_node \
  -p imgsz_parameter_name:=imgsz
```

The recommended default is `throttle`. Parameter control is optional and wrapper-dependent.

**Non-ROS integrations can consume the same boundary-pressure idea through a custom bridge, but the current reference implementation targets ROS 2.**

---

## What OBIO Is Not

OBIO is not a flight controller, safety certification layer, actuator controller, or YOLO fork. It does not guarantee safety. It is a zero-intrusive observer and signal provider for runtime assurance, regression testing, and physical-edge-AI load behavior.

## NARH-Lite

OBIO is inspired by the Non-Associative Residual Hypothesis (NARH). In this repository, NARH is used conservatively: OBIO does not claim to inspect PX4 or ROS 2 internal solver order. It applies the residual-auditing idea at the external execution boundary, where intent streams, feedback streams, freshness, and physical response can be observed.

The practical goal is simple:

```text
Detect when a previously aligned physical execution boundary begins to show structured residual growth.
```

---

### Advanced Diagnostics

OBIO also publishes standard ROS 2 `/diagnostics` and can export CSV labels for regression testing, incident review, and offline analysis.

Detailed schema, compact inspection commands, and CSV export workflow:
```
docs/diagnostics_and_csv.md
```
For most users, start here:
```
cd demos/resource_aware_yolo
./run_obio_yolo_adapter.sh
```
To validate the core boundary-pressure signal:
```
cd demos/compute_pressure
./run_low_cpu_boundary_bad.sh
./run_obio_gated_recovery.sh
```
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue)](https://docs.ros.org/en/humble/index.html)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-v1.14%20%7C%20v1.15-orange)](https://px4.io/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](https://opensource.org/licenses/Apache-2.0)


**"YOLO should react to physical-boundary pressure.
OBIO provides that signal."**
