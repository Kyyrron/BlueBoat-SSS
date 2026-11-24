# blueboat_sss

ROS 2 package (`ament_cmake`) for the BlueBoat's two Cerulean Omniscan 450 SS side-scan
sonars. It has two nodes: a driver that talks to the sonars and a processor that turns
their pings into georeferenced rows and records `.svlog` files. Both run on the operator
laptop and reach the sonars over the BlueBoat Base Station WiFi. Nothing runs on the boat.

In the project chain (`sonar → Perception → Belief → Replanner → Control`) this package is
the sonar: its `/sss_processor/processed` feeds the Ground Control Station (GCS) and Belief.
The GCS app (`blueboat_gcs/`, in this folder) is described in the [module README](../README.md)
and the [user guide](../docs/USER_GUIDE.md).

## Nodes

| Node | Executable | Does |
|---|---|---|
| `side_scan_sonar` | `sss_node.py` | Drives both units over TCP (port `192.168.2.92:51200`, starboard `192.168.2.93:51200`). Publishes each profile, decoded and as the raw Ping Protocol frame |
| `sss_processor` | `sss_processor_node.py` | Slant-range correction, bottom tracking, port/starboard merge into one row, pose from `/blueboat/odom`, `.svlog` recording |

Pinging and logging are both **off at start-up**.

In simulation, the BlueBoat-SSS-Sim node takes the place of `side_scan_sonar` with the
same name, topics and parameters. Then start only the processor.

## Build

Needs `blueboat_interfaces` (from BlueBoat-Control) built first, plus `simple_launch`,
`scipy`, `mavros_msgs` and `geographic_msgs`.

```bash
cd ~/ros2_ws
colcon build --packages-select blueboat_sss
source install/setup.bash
```

## Launch

| Command | Starts |
|---|---|
| `ros2 launch blueboat_sss SSS_processing_launch.py` | Processor only (what the GCS starts with **Sonar: Simulation**) |
| `ros2 launch blueboat_sss SSS_processing_launch.py with_acquisition:=True` | Processor + driver (what the GCS starts with **Sonar: Real sonars**) |
| `ros2 launch blueboat_sss SSS_processing_launch.py will_use_rosbag:=True` | Processor only, for a rosbag replay (never starts the driver) |
| `ros2 launch blueboat_sss SSS_simple_launch.py` | Driver only |

Acquisition parameters can be given to either launch file, for example
`range_length_mm:=20000 gain_index:=4`. When a simulation `/clock` exists at launch,
`SSS_processing_launch.py` switches its nodes to sim time by itself.

## Start and stop

```bash
# pinging (driver)
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: false'

# .svlog recording (processor)
ros2 topic pub --once /sss_processor/log/enable std_msgs/msg/Bool 'data: true'
ros2 topic pub --once /sss_processor/log/enable std_msgs/msg/Bool 'data: false'
```

The GCS sends these for you (START / STOP, Record ON / OFF).

> Do not leave the Omniscans pinging for long out of water: the transmit transducer can
> overheat. A few minutes dry is fine.

## Topics

| Topic | Type | Dir | Node |
|---|---|---|---|
| `/side_scan_sonar/{port,starboard}/profile` | `blueboat_interfaces/OmniscanProfile` | pub | driver |
| `/side_scan_sonar/{port,starboard}/raw` | `std_msgs/UInt8MultiArray` (every device frame, verbatim) | pub | driver |
| `/side_scan_sonar/ping/enable` | `std_msgs/Bool` | sub | driver |
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | pub | processor |
| `/sss_processor/log/enable` | `std_msgs/Bool` | sub | processor |
| `/blueboat/odom` | `nav_msgs/Odometry` | sub | processor (required pose) |
| `/mavros/...` (IMU, GPS, compass, velocity, pose, VFR HUD, state) | MAVROS types | sub | processor (written into the `.svlog`) |

The sonar topics are `BEST_EFFORT` / `KEEP_LAST(10)`. A `RELIABLE` subscriber receives
nothing. Change a delivery setting (QoS, quality of service) at both ends or not at all.

In `ProcessedSSSPing`, the sign of `*_y` gives the side (+y port, −y starboard). Samples
are already slant-range corrected, with the water column removed.

## Acquisition parameters

Node parameters of `side_scan_sonar`, the same in both launch files. They are re-read on
every ping enable.

| Parameter | Default | Meaning |
|---|---|---|
| `range_start_mm` | 0 | start of the sampled window |
| `range_length_mm` | 20000 | range per side, mm |
| `msec_per_ping` | 50 | minimum ping interval (20 Hz, as SonarView); 0 = free run, diagnostic only |
| `gain_index` | −1 | −1 = device auto gain |
| `num_results` | 1200 | samples per ping (device maximum; 16.7 mm at 20 m) |
| `pulse_len_percent` | 0.00125 | transmit pulse as a fraction of 2R/c, the round-trip time at range R and sound speed c (stays at the 44.3 µs minimum up to ~27 m) |

The defaults are SonarView's settings and the project's survey setting (20 m per side).

- **Set the range from the water depth** (about 4× the deepest expected), not from the area
  you want to cover. Too long a range pushes the bottom return into the first few samples
  and bottom tracking fails.
- The ping rate is `max(msec_per_ping, 2R/c + ~3 ms)`: 20 Hz per side up to about 35 m.
- Free run (`msec_per_ping:=0`) pings faster, uses more WiFi and lost pings in the field.

Change a parameter while running: pinging off, set, pinging on.

```bash
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: false'
ros2 param set /side_scan_sonar range_length_mm 30000
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'
```

The GCS range slider does exactly this.

Processor parameter: `transducer_x_offset_m` (default −0.345, + forward of the
`/blueboat/odom` pose). It is published in every `ProcessedSSSPing.transducer_x_offset`.
To re-process a recording made before this arm existed, pass `transducer_x_offset_m:=0.0`.

## Transducer geometry

Constants at the top of `src/sss_processor_node.py`, the same values as the GCS and the
simulator:

| Constant | Value | Meaning |
|---|---|---|
| `TRANSDUCER_X_OFFSET_M` | −0.345 m | fore/aft arm from the odometry point (0.345 m aft) |
| `TRANSDUCER_Y_OFFSET_PORT_M` / `_STBD_M` | 0.285 m | lateral offset of each transducer |
| `TRANSDUCER_SUBMERSION_M` | 0.22 m | depth below the waterline |

These values are adopted from the simulator's profile, **not measured on the boat**.

## `.svlog` recording

- The processor writes the `.svlog`, rebuilt from the two `raw` topics. If `raw` stops,
  the log comes out empty.
- Output folder: `../../../../data/SSS_data`, **relative to the launch working directory**.
  Launched by the GCS from `BlueBoat-SSS/blueboat_sss/`, that is `~/ros2_ws/data/SSS_data`.
  If the folder cannot be created or written, the processor logs an error and does not record.
- Format: SonarView-compatible framed Ping Protocol, both channels in one file (told apart
  by each packet's `channel_number`), plus the MAVLink navigation stream. Opens directly in
  SonarView.
- Files roll at 500 MB. A name collision gets a `-001` suffix; nothing is overwritten.
- Recorded `.svlog` files are primary field data: never edit or overwrite them.

## Health check

```bash
ros2 node info /side_scan_sonar
ros2 node info /sss_processor
ros2 topic info /side_scan_sonar/ping/enable        # subscriber attached?
ros2 topic echo /side_scan_sonar/port/profile --field ping_number
ros2 topic hz   /side_scan_sonar/starboard/profile
ros2 topic hz   /sss_processor/processed
```

If the profiles arrive but `/sss_processor/processed` is at 0 Hz:

1. **No odometry.** The processor drops pings while `/blueboat/odom` is silent.
2. **QoS mismatch.** A `RELIABLE` subscriber on a sonar topic gets nothing.

While pinging, the driver logs a health line per side every 10 s (rate, missing ping
numbers, longest gap, checksum errors). A loss shown there happened before ROS.

## Offline tools

From `~/ros2_ws`, with the workspace sourced:

| Script | Does |
|---|---|
| `custom_scripts/svlog_to_rosbag.py` | Converts a `.svlog` into a rosbag the processor can replay. Set `INPUT_FILE` at the top of the script; it writes `./output_bag` (must not exist) |
| `custom_scripts/processed_sss_printer.py` | Re-plots a saved GCS mosaic. Options `--mosaic`, `--trajectory`, `--cmap` |

Replay a converted log through the processor:

```bash
ros2 launch blueboat_sss SSS_processing_launch.py will_use_rosbag:=True
ros2 bag play output_bag          # in another terminal, from the bag's folder
```

Enable logging first if you want a fresh `.svlog` from the replay.

## Layout

```
blueboat_sss/
  src/sss_node.py               driver
  src/sss_processor_node.py     processor
  src/_custom_libraries/        omniscan_link.py (Ping Protocol), svlog_helper.py,
                                sss_helper.py, math_helper.py
  launch/                       SSS_processing_launch.py, SSS_simple_launch.py
  custom_scripts/               offline tools
  tests/                        pytest suite (fake Omniscan devices included)
  terminals.txt                 operator command sheet
  blueboat_gcs/                 the GCS app (not installed by colcon)
```

All `src/` scripts are installed flat into `lib/blueboat_sss/`, so the nodes import the
helpers without a package prefix. Keep the executable names `sss_node.py` and
`sss_processor_node.py`: the GCS finds the running nodes by name.

## Tests

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-SSS/blueboat_sss
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m pytest -p no:cacheprovider
```

Use the system python and do not source the workspace first (its venv has no pytest).
Node tests skip when `blueboat_interfaces` is not found; corpus tests skip without the
field `.svlog` corpus.

## More

- The GCS and the whole module: [`../README.md`](../README.md), [`../docs/USER_GUIDE.md`](../docs/USER_GUIDE.md)
- `.svlog` format and the measured device settings: `blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md`
- Operator command sheet: `terminals.txt`
