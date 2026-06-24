# Compute Pressure Demos: Why CPU% Is a Flawed Load-Shedding Trigger

This demo suite provides a minimal, reproducible way to show why indirect hardware metrics such as CPU%, GPU%, memory%, temperature, or FPS are not reliable triggers for UAV autonomy load-shedding.

The key claim:

```text
Do not trigger autonomy load-shedding only from CPU%.
Trigger it from Offboard boundary pressure.
```

OBIO / AFIO reports boundary pressure through metrics such as:

```text
setpointAgeMs
setpointJitterMs
staleStreams
positionTrackingResidual
velocityTrackingResidual
flightResidual
dominantCause
```

These demos intentionally separate **machine load** from **control-boundary health**.

## Interpretation

CPU%, GPU%, FPS, and memory usage are hardware-side signals. They are useful, but they are indirect.

OBIO measures the flight-control boundary directly:

```text
Is PX4 still receiving fresh, temporally consistent, physically realizable Offboard intent?
```

That is the signal an autonomy manager should use before deciding whether to pause loop closure, reduce perception resolution, skip frames, or protect the control publisher.

## Prerequisites

- ROS 2 Humble or Jazzy
- `px4_msgs`
- `ai_flight_integrity_observer` package built in the same workspace
- `htop` or `btop` for visual CPU observation

These demos can run without a real drone. They use `synthetic_px4_publisher` as the odometry/offboard-mode source and route trajectory setpoints through a fake SLAM stressor.

## Install the demo nodes

Copy these files into your package:

```text
ai_flight_integrity_observer/fake_slam_stressor_node.py
ai_flight_integrity_observer/obio_gated_load_shedder.py
demos/compute_pressure/
```

Add these console scripts to `setup.py`:

```python
"fake_slam_stressor_node = ai_flight_integrity_observer.fake_slam_stressor_node:main",
"obio_gated_load_shedder = ai_flight_integrity_observer.obio_gated_load_shedder:main",
```

Then rebuild:

```bash
cd ~/px4_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --merge-install
source install/setup.bash
```

## Terminal layout

Use two or three terminals:

```text
Left:  htop / btop
Right: OBIO live dashboard or /diagnostics watch
Bottom or another terminal: scenario script output
```

Optional watch command:

```bash
ros2 topic echo /diagnostics --once --full-length
```

## Scenario A: Baseline

```bash
cd demos/compute_pressure
./run_baseline.sh
```

Expected:

```text
CPU: low/normal
OBIO: GREEN
Takeaway: healthy Offboard stream
```

## Scenario B: High CPU, boundary still healthy

```bash
cd demos/compute_pressure
./run_cpu_high_boundary_ok.sh
```

Expected:

```text
CPU: high
OBIO: GREEN
Takeaway: high CPU does not automatically mean flight-control degradation
```

This scenario starts a low-priority CPU burner while keeping the setpoint forwarding path healthy.

## Scenario C: Low/moderate CPU, boundary degraded

```bash
cd demos/compute_pressure
./run_low_cpu_boundary_bad.sh
```

Expected:

```text
CPU: may look moderate
OBIO: SETPOINT_JITTER / STALE_STREAM
Takeaway: hardware metrics can look fine while the setpoint path is starving
```

This simulates lock contention, sensor synchronization wait, or loop-closure stalls by blocking the setpoint forwarding callback for 150–300 ms.

## Scenario D: OBIO-gated load shedding

```bash
cd demos/compute_pressure
./run_obio_gated_recovery.sh
```

Expected:

```text
jitter spike
  -> OBIO warning
  -> external load shedder pauses fake SLAM work
  -> boundary recovers
```

The shedder subscribes to `/diagnostics` and publishes to `/obio_demo/load_enabled`. It demonstrates the integration pattern only. OBIO itself does not schedule, prune, control, or recover the vehicle.

