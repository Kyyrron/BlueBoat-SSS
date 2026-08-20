# BlueBoat GCS

Professional desktop application for live side-scan sonar surveys on the BlueBoat USV.
Replaces the matplotlib `processed_sss_listener.py` as the single software used during
experiments: live geo-referenced SSS mosaic over optional satellite imagery, robot
trajectory, AI detections and USBL pinger overlays, distance/inspection tools, and
START/STOP control of the processing pipeline.

## Quick start

```bash
pip install PySide6 pyyaml opencv-python numpy

# Demo / development — no ROS, no boat:
python -m blueboat_gcs.main --sim

# Field mode (ROS 2 environment with blueboat_interfaces sourced):
python -m blueboat_gcs.main [--config site.yaml]
```

## Layout

```
blueboat_gcs/        the application package (see docs/ARCHITECTURE.md)
launch/              SSS_processing_launch.py — processor-only launch file
                     started by the START button (install into blueboat_sss)
docs/ARCHITECTURE.md architecture + design decisions
docs/HANDOVER.md     placeholder topics, integration points, interpolation
                     recommendation, future work
```

Configuration: `blueboat_gcs/config/default.yaml` (topics, pipeline command, mosaic
resolution, interpolation limits, tile source, frame yaw offset).
