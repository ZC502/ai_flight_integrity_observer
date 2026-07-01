# Resource-Aware YOLO Demo

Adaptive YOLO deployment for ROS 2 robots, powered by OBIO boundary-pressure signals.

This demo verifies the `obio_yolo_adapter` without a real camera, drone, PX4 SITL instance, or GPU. It publishes a synthetic camera stream and a deterministic OBIO-like pressure sequence, then shows the adapter changing the forwarded camera rate.

## What It Shows

```text
GREEN  -> full-rate image forwarding
YELLOW -> moderate frame skipping
RED    -> aggressive frame skipping
GREEN  -> restored only after hysteresis
```

The demo is intentionally small. It proves the adapter behavior first; a real YOLO node can be connected by subscribing to `/obio/image_for_yolo` instead of `/camera/image_raw`.

## Run

From this directory:

```bash
chmod +x run_obio_yolo_adapter.sh
./run_obio_yolo_adapter.sh
```

Expected profile cycle:

```json
{"state": "GREEN", "frame_stride": 1, "imgsz": 960}
{"state": "YELLOW", "frame_stride": 2, "imgsz": 640}
{"state": "RED", "frame_stride": 4, "imgsz": 320}
{"state": "GREEN", "frame_stride": 1, "imgsz": 960}
```

To watch input and output rates in separate terminals:

```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /obio/image_for_yolo
```

## Optional: Use Real OBIO Synthetic Core

The default mode uses synthetic OBIO-like diagnostics so that the adapter behavior is deterministic.

To use the real OBIO synthetic PX4/fake-stressor path instead:

```bash
./run_obio_yolo_adapter.sh obio-core
```

This starts:

```text
flight_integrity_node
synthetic_px4_publisher
fake_slam_stressor_node
obio_yolo_adapter
synthetic camera publisher
```

State changes in this mode depend on the current behavior and parameters of `fake_slam_stressor_node`.

## Connect a Real YOLO Node

Change your YOLO node input from:

```text
/camera/image_raw
```

to:

```text
/obio/image_for_yolo
```

Then launch the adapter:

```bash
ros2 run ai_flight_integrity_observer obio_yolo_adapter --ros-args \
  -p mode:=throttle \
  -p input_image_topic:=/camera/image_raw \
  -p output_image_topic:=/obio/image_for_yolo \
  -p green_frame_stride:=1 \
  -p yellow_frame_stride:=2 \
  -p red_frame_stride:=4 \
  -p hysteresis_sec:=2.0
```

## Optional Parameter-Control Mode

If your YOLO node safely supports runtime parameter updates, the adapter can request an `imgsz` change through ROS 2 Parameter Service:

```bash
ros2 run ai_flight_integrity_observer obio_yolo_adapter --ros-args \
  -p mode:=param \
  -p target_yolo_node:=/yolo_node \
  -p imgsz_parameter_name:=imgsz \
  -p green_imgsz:=960 \
  -p yellow_imgsz:=640 \
  -p red_imgsz:=320
```

This is optional. Topic throttling is the recommended default because it works without modifying YOLO.
