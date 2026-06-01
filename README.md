# AI Flight Integrity Observer

> A ROS 2 / PX4 runtime observer for flight execution integrity under AI and companion-compute load.

Modern drones increasingly rely on companion computers for VIO, target tracking, obstacle avoidance, and edge AI perception.

But heavy AI workloads can silently degrade the offboard control boundary:

- setpoint publish jitter
- stale trajectory commands
- delayed vehicle odometry
- estimator lag
- GPS / VIO jumps
- command-response desynchronization

This project does not intercept flight control.

It observes whether AI-generated offboard setpoints are physically realized by the vehicle, and exposes flight execution collapse through standard ROS 2 diagnostics.

Initial target:

```text
/fmu/in/trajectory_setpoint
/fmu/out/vehicle_odometry
        ↓
/diagnostics
  ai_flight_integrity/flight_execution_integrity
```
### Time synchronization warning

Flight-integrity metrics that involve latency, age, or jitter are only meaningful when timestamp semantics are understood.

PX4 messages often use PX4-side monotonic timestamps, while ROS 2 nodes may use companion-computer time. This observer reports both message timestamp-derived age and ROS receive-time age where possible.

For hardware tests, ensure tight time synchronization between the flight controller and companion computer, or interpret absolute delay metrics as approximate. Unsynchronized clocks can produce false `SETPOINT_STALE`, `ODOMETRY_STALE`, or `TIMING_JITTER` diagnostics.

### PX4 QoS warning

PX4 high-rate topics often use sensor-style QoS. If your subscriber uses the default ROS 2 reliable QoS, you may see the topic in `ros2 topic list` but receive no messages.

This observer explicitly uses Best Effort / Volatile QoS for PX4-facing topics.

### px4_msgs dependency

This package depends on `px4_msgs`.

Clone and build `px4_msgs` in the same ROS 2 workspace, and make sure its branch matches your PX4 firmware / SITL version.

Also note that only PX4 topics listed in the firmware `dds_topics.yaml` are exposed through uXRCE-DDS.

```Bash
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws/src

git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/ZC502/ai_flight_integrity_observer.git

cd ~/px4_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
Make sure the px4_msgs branch matches your PX4 firmware / SITL version.


