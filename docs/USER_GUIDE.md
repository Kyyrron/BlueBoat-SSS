# BlueBoat-SSS — user guide

How to use the Ground Control Station (GCS) and the sonar nodes in practice. For installation,
see the [README](../README.md). The two ROS nodes are described in detail in
[blueboat_sss/README.md](../blueboat_sss/README.md).

## 1. Start the GCS

Always from the `blueboat_sss/` folder: the app imports its package from there, and its data path
is relative to it.

```bash
cd ~/ros2_ws && source env.sh
cd src/BlueBoat-SideScanSonar/BlueBoat-SSS/blueboat_sss
python3 -m blueboat_gcs.main                      # live, with ROS
python3 -m blueboat_gcs.main --config site.yaml   # with your settings on top of the defaults
python3 -m blueboat_gcs.main --sim                # built-in demo, no ROS needed
```

## 2. A survey, step by step

1. **Start the GCS.** It launches the processing node itself
   (`ros2 launch blueboat_sss SSS_processing_launch.py`).
2. **Choose the sonar source** in the toolbar's **Sonar** selector:
   - **Simulation**: the simulator's sonar (BlueBoat-SSS-Sim) is already running in Gazebo;
   - **Real sonars**: the GCS also starts the real sonar driver.
   The choice is remembered for the next start.
3. **START** turns pinging on and starts the live view.
4. **Record ON** opens a recording session; **Record OFF** closes it. Everything you want to keep
   must be inside a session.
5. **STOP** turns pinging off, closes an open session and stops the nodes.

While running, the right panel's **Acquisition → Sonar range** slider changes the range. The GCS
stops pinging, sets the new range and starts pinging again. With the simulator's ray-cast sonar,
keep the range at 30 m or less (above 40 m the graphics card runs out of memory).

**Bag replay.** When the Mission Control Station replays a recorded real run in Gazebo, it starts
no sonar. With **Sonar: Simulation**, START notices the replay and starts the simulated sonar for
you, so you get pictures of the replayed run.

## 3. What the main window shows

- **Mosaic view**: the seabed placed on the satellite map, the boat's track, the sonar's swath,
  the acoustic pinger and the detections of BlueBoat-Perception.
- **Waterfall view**: one row per ping, port on the left, starboard on the right, the newest on
  top. Click a point to see where it is on the map and its GPS position.
- **Display settings**: colour map, contrast, brightness, mosaic priority (which pass wins where
  passes overlap), resolution and depth compensation.
- **The console** shows the messages of every node, so no extra terminal is needed. It also
  reports the stream's health: missing pings, one-sided rows.

The map stays hidden with "Waiting for GPS fix" until the first valid GPS fixes arrive.

## 4. The replay window

**Open SVLOG** opens a recorded `.svlog` (ours or SonarView's) in a second window:

| Button | What it does |
|---|---|
| **Render range** | Draws the chosen time window at once, as a mosaic or a waterfall |
| **Replay** | Plays the log at real time or faster |
| **Run AI** | Runs BlueBoat-Perception's detector on the log's seabed images, in the app |
| **Save pictures from the log** | Writes the seabed images next to the `.svlog`, in `seabed_images_<log name>/` |
| **Save as rosbag** | Converts the log into a ros2 bag |
| **Merge with another svlog** | Combines two logs into one new session folder; both originals stay untouched |

## 5. Settings

| File | Sets |
|---|---|
| `blueboat_sss/blueboat_gcs/config/default.yaml` | Topic names, the processing launch command, the default sonar source, mosaic, display, seabed images, map tiles, range slider limits, data folder |
| `--config <file>.yaml` | Your overrides on top of `default.yaml`, for example one file per site |
| Launch arguments of the nodes | The sonar settings (`range_length_mm`, `num_results`...) and the processor's `transducer_x_offset_m`; see [blueboat_sss/README.md](../blueboat_sss/README.md) |

## 6. Where the data goes

Everything goes under `~/ros2_ws/data/SSS_data/`:

```
data/SSS_data/
  sessions/<date-time>/      one folder per Record ON → OFF
    metadata.json            times, settings, counters (and the simulator's run record in simulation)
    *.svlog                  the sonar log of that session
    mosaic/                  sonar_mosaic.png and .npz, boat_trajectory.csv
    waterfall/               waterfall.png, waterfall_raw.npz (the full raw record)
    detections/              detections.csv
    seabed_images/           the seabed images sent to the detector, with their metadata
  merged_sessions/<name>/    two logs merged from the replay window
```

**Seabed images** are what the detector sees: each image is 256 rows of the waterfall with
square pixels, and its JSON metadata tells where every pixel lies on the map. Consecutive images
overlap by half, on purpose, so that every object appears whole in at least one image.

Recorded `.svlog` files are primary field data: never edit or overwrite them.

## 7. Main topics

| Topic | Type | From → to |
|---|---|---|
| `/side_scan_sonar/{port,starboard}/profile` | `blueboat_interfaces/OmniscanProfile` | driver → processor, GCS |
| `/side_scan_sonar/{port,starboard}/raw` | `std_msgs/UInt8MultiArray` | driver → processor (written into the `.svlog`) |
| `/side_scan_sonar/ping/enable` | `std_msgs/Bool` | GCS → driver (START / STOP) |
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | processor → GCS, Belief |
| `/sss_processor/log/enable` | `std_msgs/Bool` | GCS → processor (Record ON / OFF) |
| `/sss_ai/seabed_image`, `…/png`, `…/metadata` | `sensor_msgs/Image`, `sensor_msgs/Image`, `std_msgs/String` | GCS → Perception |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | Perception → GCS (drawn on the map) |

The GCS also reads `/blueboat/odom`, `/mavros/global_position/global`,
`/mavros/global_position/compass_hdg`, `/mavros/imu/data`, `/mavros/vfr_hud` and
`/blueboat/pinger_coordinates`. The sonar topics use "best effort" delivery: change a delivery
setting (QoS, quality of service) at both ends or not at all.

## 8. Folder layout

```
BlueBoat-SSS/
  requirements.txt           GCS Python packages
  build.sh                   build the workspace and start the GCS
  docs/                      this guide, the development guide, pictures
  blueboat_sss/              ROS 2 package
    src/                     sss_node.py (driver), sss_processor_node.py (processor)
    src/_custom_libraries/   sonar protocol, svlog, geometry and math helpers
    launch/                  SSS_processing_launch.py, SSS_simple_launch.py
    custom_scripts/          offline tools (.svlog → rosbag, mosaic plotter)
    tests/                   test suite (driver, processor, GCS)
    terminals.txt            operator command sheet
    blueboat_gcs/            the GCS app
      main.py                entry point
      config/default.yaml    default settings
      docs/                  architecture and background
```
