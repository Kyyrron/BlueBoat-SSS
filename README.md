# BlueBoat-SSS

Side-scan sonar acquisition, processing and operator display for the BlueBoat USV.
It drives the boat's two Cerulean Omniscan 450 SS units (port and starboard), turns their
pings into georeferenced rows, records SonarView-compatible `.svlog` files, and shows
everything live in the Ground Control Station (GCS) desktop app.

The module has two parts:

| Part | What it is |
|---|---|
| `blueboat_sss` | ROS 2 package: the sonar driver `sss_node.py` and the processor `sss_processor_node.py` (slant-range correction, bottom tracking, port/starboard merge, `.svlog` recording). Usage: [`blueboat_sss/README.md`](blueboat_sss/README.md) |
| `blueboat_gcs` | PySide6 desktop app: live mosaic over satellite tiles, waterfall, boat track, detection and USBL pinger overlays, recording sessions, seabed images for Perception, `.svlog` replay. Not a colcon package |

## Where it fits

```
sonar (this module) → Perception → Belief → Replanner → Control
```

- This module is the first link: it produces the processed pings and the seabed images.
- **All software runs on the operator laptop** (`~/ros2_ws`). The boat carries only hardware
  and firmware. The driver reaches the sonars over the BlueBoat Base Station WiFi
  (`192.168.2.92` port, `192.168.2.93` starboard, TCP 51200).
- The GCS runs no detector. It only draws the detections that BlueBoat-Perception publishes.
- In simulation, BlueBoat-SSS-Sim replaces the driver with a drop-in node of the same name
  and topics. The processor and the GCS run unchanged.

## Prerequisites

- ROS 2 Jazzy, workspace `~/ros2_ws`.
- `blueboat_interfaces` (message types), from BlueBoat-Control, built in the same workspace.
- Node dependencies: `simple_launch`, `scipy`, `mavros_msgs`, `geographic_msgs`;
  `rosbag2_py` for the `.svlog` → rosbag converter.
- GCS Python packages (PySide6, numpy, opencv-python, PyYAML):

```bash
pip install -r ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-SSS/requirements.txt
```

Keep numpy below 2: the ROS 2 Jazzy packages and scipy are built against numpy 1.x.
On an externally managed system python, add `--break-system-packages`.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select blueboat_interfaces   # first, if not built yet
colcon build --packages-select blueboat_sss
source install/setup.bash
```

The GCS needs no build: it runs from the source tree.

## Run

### The GCS (normal way)

Start it from the `blueboat_sss/` folder. It imports `blueboat_gcs` from there, and its
data path is relative to it.

```bash
cd ~/ros2_ws && source env.sh
cd src/BlueBoat-SideScanSonar/BlueBoat-SSS/blueboat_sss
python3 -m blueboat_gcs.main                      # live, with ROS
python3 -m blueboat_gcs.main --config site.yaml   # with a config override
python3 -m blueboat_gcs.main --sim                # built-in demo, no ROS needed
```

`build.sh` runs a full `colcon build`, sources `env.sh` and starts the GCS in one go.

What the operator does in the GCS:

1. At start-up the GCS launches the processing pipeline itself
   (`ros2 launch blueboat_sss SSS_processing_launch.py`).
2. The toolbar **Sonar** selector picks the source: **Simulation** (the simulator's sonar)
   or **Real sonars** (the GCS then also starts the driver). The choice is remembered.
3. **START** turns pinging on and starts the live view.
4. **Record ON / OFF** opens and closes a recording session.
5. **STOP** turns pinging off, closes the session and stops the nodes.
6. **Open SVLOG** opens a recorded `.svlog` in the replay window.

The right panel's **Acquisition → Sonar range** slider changes the range while running.

### The nodes by hand (without the GCS)

```bash
ros2 launch blueboat_sss SSS_processing_launch.py                        # processor only
ros2 launch blueboat_sss SSS_processing_launch.py with_acquisition:=True # + real driver
ros2 launch blueboat_sss SSS_simple_launch.py                            # driver only
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'
```

Pinging and recording are **off at start-up**. All commands, parameters and checks:
[`blueboat_sss/README.md`](blueboat_sss/README.md).

## Main topics

| Topic | Type | Producer → consumer |
|---|---|---|
| `/side_scan_sonar/{port,starboard}/profile` | `blueboat_interfaces/OmniscanProfile` | driver → processor, GCS |
| `/side_scan_sonar/{port,starboard}/raw` | `std_msgs/UInt8MultiArray` | driver → processor (feeds the `.svlog`) |
| `/side_scan_sonar/ping/enable` | `std_msgs/Bool` | GCS → driver (START / STOP) |
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | processor → GCS, Belief |
| `/sss_processor/log/enable` | `std_msgs/Bool` | GCS → processor (Record ON / OFF) |
| `/sss_ai/seabed_image`, `…/png`, `…/metadata` | `sensor_msgs/Image`, `sensor_msgs/Image`, `std_msgs/String` | GCS → Perception |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | Perception → GCS (drawn on the map) |

The GCS also reads `/blueboat/odom`, `/mavros/global_position/global`,
`/mavros/global_position/compass_hdg`, `/mavros/imu/data`, `/mavros/vfr_hud` and
`/blueboat/pinger_coordinates`. The sonar topics are `BEST_EFFORT`. Change a QoS at both
ends or not at all.

## Configuration

| File | Sets |
|---|---|
| `blueboat_sss/blueboat_gcs/config/default.yaml` | GCS: topic names, pipeline launch command, default sonar source, mosaic, display, seabed images, map tiles, range-slider limits, `data_root` |
| `--config <file>.yaml` | Your overrides on top of `default.yaml` (for example one per site) |
| Launch arguments | Driver acquisition (`range_length_mm`, `num_results`, …) and the processor's `transducer_x_offset_m`; see the package README |

## Data

Everything goes under `~/ros2_ws/data/SSS_data/` (`data_root`, resolved from the
`blueboat_sss/` folder).

```
data/SSS_data/
  sessions/<stamp>/          one folder per Record ON → OFF
    metadata.json            times, config, counters (+ the simulator's run record in simulation)
    *.svlog                  the sonar log of that session
    mosaic/  waterfall/  detections/  seabed_images/
  merged_sessions/<name>/    two logs merged from the replay window
```

Recorded `.svlog` files are primary field data: never edit or overwrite them.

## Layout

```
BlueBoat-SSS/
  requirements.txt           GCS Python packages
  build.sh                   build the workspace and start the GCS
  blueboat_sss/              ROS 2 package (ament_cmake)
    src/                     sss_node.py, sss_processor_node.py
    src/_custom_libraries/   omniscan_link (protocol), svlog / sss / math helpers
    launch/                  SSS_processing_launch.py, SSS_simple_launch.py
    custom_scripts/          offline tools (.svlog → rosbag, mosaic plotter)
    tests/                   pytest suite (driver, processor, GCS)
    terminals.txt            operator command sheet
    blueboat_gcs/            the GCS app
      main.py                entry point
      gui/ ros/ core/ mapping/ models/ utils/ sim/ analysis/ tools/
      config/default.yaml
      docs/                  architecture and background
```

## Tests

From `blueboat_sss/`, with the system python and **without** sourcing the workspace
(the workspace venv has no pytest):

```bash
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m pytest -p no:cacheprovider
```

Tests that need `blueboat_interfaces`, a ROS graph or the field `.svlog` corpus skip
when it is missing.

## Further reading

| Topic | File |
|---|---|
| Nodes, parameters, recording, health checks | [`blueboat_sss/README.md`](blueboat_sss/README.md) |
| GCS architecture and design | `blueboat_sss/blueboat_gcs/docs/ARCHITECTURE.md` |
| GCS integration points | `blueboat_sss/blueboat_gcs/docs/HANDOVER.md` |
| Seabed images for Perception | `blueboat_sss/blueboat_gcs/docs/PERCEPTION_INTEGRATION.md` |
| `.svlog` format and measured device settings | `blueboat_sss/blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md` |
| Side-scan background | `blueboat_sss/blueboat_gcs/docs/SCIENTIFIC_BACKGROUND.md` |
| The whole project | the superproject `README.md` and `docs/` |
