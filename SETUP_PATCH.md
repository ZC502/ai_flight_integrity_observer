# Setup patch

Add the following console scripts to `setup.py`:

```python
"fake_slam_stressor_node = ai_flight_integrity_observer.fake_slam_stressor_node:main",
"obio_gated_load_shedder = ai_flight_integrity_observer.obio_gated_load_shedder:main",
```

Example:

```python
entry_points={
    "console_scripts": [
        "flight_integrity_node = ai_flight_integrity_observer.flight_integrity_node:main",
        "synthetic_px4_publisher = ai_flight_integrity_observer.synthetic_px4_publisher:main",
        "flight_diagnostics_to_csv_labeler = ai_flight_integrity_observer.flight_diagnostics_to_csv_labeler:main",
        "fake_slam_stressor_node = ai_flight_integrity_observer.fake_slam_stressor_node:main",
        "obio_gated_load_shedder = ai_flight_integrity_observer.obio_gated_load_shedder:main",
    ],
},
```

No new ROS dependencies are required beyond `rclpy`, `std_msgs`, `diagnostic_msgs`, and `px4_msgs`.
