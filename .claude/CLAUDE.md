# CLAUDE.md — BlueBoat-SSS

Side-scan sonar (SSS) acquisition, processing and ground-control for a BlueRobotics
BlueBoat USV. Two halves live in this submodule:

- **`blueboat_sss`** — ROS 2 package (`ament_cmake`) running on the robot: the Omniscan
  driver (`src/sss_node.py`) and the processing/logging node
  (`src/sss_processor_node.py`).
- **BlueBoat GCS** — a standalone PySide6 + rclpy operator application (package
  `blueboat_gcs`, entry point `blueboat_gcs.main`). Runs on the basestation laptop.
  It has **no ROS build dependency** and runs fully without ROS via `--sim`.

Research context: this supports a master's thesis on autonomous aspect-aware SSS
survey. Some rules below exist to protect the scientific argument, not just the code.

---

## Verification status of this document

Every claim below that can be checked against the files in this repository has been.
Facts needing a colcon build or the boat are either omitted — they live in `TODO.md`
with the reason — or carried with an explicit **UNVERIFIABLE HERE** marker where the
claim is load-bearing enough that dropping it would lose more than flagging it.

| Marker | Meaning |
|---|---|
| **VERIFIED** | Confirmed by reading the file named, in this tree. |
| **MEASURED** | Derived from direct measurement of a real recording (see `docs/SONARVIEW_SVLOG_ANALYSIS.md`). |
| **UNVERIFIABLE HERE** | Cannot be checked from this repository alone — it needs the field `.svlog` corpus, a sourced `blueboat_interfaces`, or the boat. |

Where this document and the repository ever disagree, the repository wins: prefer
reading a file to trusting a description of it here.

**What is UNVERIFIABLE HERE, and why.** Everything marked MEASURED rests on the field
`.svlog` corpus, which is not in this repository and not on this machine: the suite
looks for it at the hard-coded external mount
`/media/kyyrron/OS/Users/killi/Desktop/Research Kyutech/BlueBoat/allSvlogData`
(`tests/test_processor_assembly.py`), and every test that would reproduce a MEASURED
number skips when it is absent. The corpus-derived numbers below — the *Measured
acquisition settings* table, the 0 / −1 / +60 counter offsets, the boot-skew values,
the wrong-`src` percentages, the mosaic cell sizes, the 397.8 s session gap — are
therefore carried forward from when the corpus was mounted and were **not**
re-derived. The same applies to the two device addresses `192.168.2.92` /
`192.168.2.93` and the `sss_processor` ↔ hardware behaviour generally: the source is
verified, the boat is not reachable from here.

---

## Repository layout

```
BlueBoat-SSS/
├── requirements.txt                    GCS deps: PySide6, numpy, opencv-python, PyYAML
├── requirements-dev.txt                the above + pytest; numpy pinned <2 — GITIGNORED
├── README.md                           GCS quick-start
├── .githooks/pre-commit                runs the regression suite before a commit
├── .githooks/install.sh                sets core.hooksPath — the whole dir is GITIGNORED
└── blueboat_sss/                       the ROS 2 package root (ament_cmake)
    ├── package.xml                     name blueboat_sss, build_type ament_cmake
    ├── CMakeLists.txt                  installs launch/ + the five src/ scripts
    ├── build.sh                        colcon build in ~/ros2_ws, then launch the GCS
    ├── terminals.txt                   the operator copy-paste command sheet
    ├── pytest.ini                      rootdir for the suite; testpaths = tests
    ├── tests/                          regression suite: GCS half needs no ROS,
    │                                   robot-side half skips without a workspace
    ├── README.md                       robot-side quick start (nodes, topics, params)
    ├── launch/SSS_processing_launch.py processor, + acquisition on with_acquisition:=True
    ├── launch/SSS_simple_launch.py     acquisition only
    ├── src/sss_node.py                 node `side_scan_sonar`
    ├── src/sss_processor_node.py       node `sss_processor`
    ├── src/_custom_libraries/          sss_helper.py, svlog_helper.py, math_helper.py
    ├── custom_scripts/                 svlog_to_rosbag.py, processed_sss_printer.py
    ├── blueboat_sss/__init__.py        empty; the ament_python_install_package target
    └── blueboat_gcs/                   the GCS application package lives HERE
        ├── main.py
        ├── analysis/svlog_forensics.py  offline .svlog forensics (ROS-free)
        ├── config/  core/  gui/  mapping/  models/  ros/  sim/  tools/  utils/  docs/
```

**VERIFIED.** `blueboat_interfaces` is **not** in this submodule — it is an external
package supplied by BlueBoat-Control. The robot-side scripts are installed by
`CMakeLists.txt` as `PROGRAMS` into `lib/blueboat_sss/`, flat, which is why
`sss_processor_node.py` does `sys.path.insert(0, dirname(__file__))` and imports
`svlog_helper` / `sss_helper` / `math_helper` without a package prefix. The
`blueboat_sss/blueboat_sss/` Python package is empty; no GCS code is installed by
colcon.

---

## ROS 2 interface

This is the highest-value section: nearly every real bug in this project has been an
interface or framing mismatch. Types prefixed `blueboat_interfaces/` come from that
external interfaces package.

### `sss_node.py` — node name `side_scan_sonar`
Drives **two** Cerulean Omniscan 450 SS units over TCP via `brping.Omniscan450`
(one `OmniscanWorker` thread each, with a reconnect loop). Port is
`192.168.2.92:51200`, starboard `192.168.2.93:51200`, hard-coded in the node.
**VERIFIED.**

| Direction | Topic | Type | Other end |
|---|---|---|---|
| Pub | `~/port/profile` | `blueboat_interfaces/OmniscanProfile` | processor |
| Pub | `~/port/raw` | `std_msgs/UInt8MultiArray` | processor (svlog) |
| Pub | `~/starboard/profile` | `blueboat_interfaces/OmniscanProfile` | processor |
| Pub | `~/starboard/raw` | `std_msgs/UInt8MultiArray` | processor (svlog) |
| Sub | `~/ping/enable` | `std_msgs/Bool` | GCS START/STOP |

All four publishers use `BEST_EFFORT`, `KEEP_LAST`, depth 10. Pinging is **OFF at
startup**; acquisition parameters are re-read on every enable. **VERIFIED.**

`~/raw` carries the **already-framed Cerulean Ping Protocol packet** republished
verbatim from `brping` (`data.msg_data`). `OmniscanProfile` mirrors the decoded
header: `header` (`std_msgs/Header`), `side`, `ping_number`, `start_mm`, `length_mm`,
`timestamp_ms`, `ping_hz`, `gain_index`, `num_results`, `sos_dmps`, `channel_number`,
`pulse_duration_sec`, `analog_gain`, `max_pwr_db`, `min_pwr_db`,
`transducer_heading_deg`, `vehicle_heading_deg`, `pwr_results` (`uint16[]`).
**VERIFIED** against `BlueBoat-Control/blueboat_interfaces/msg/OmniscanProfile.msg`.

Parameters, declared identically in `sss_node.py` and both launch files
(**VERIFIED**): `range_start_mm` (0), `range_length_mm` (20000), `msec_per_ping` (0),
`gain_index` (−1 = device auto), `num_results` (600), `pulse_len_percent` (0.002).
`FILTER_DURATION_PERCENT` is pinned at 0.0015 and is not a parameter.

### `sss_processor_node.py` — node name `sss_processor`
Slant-range correction, bottom tracking, port/starboard merge, and `.svlog` writing.
The two responsibilities are independent: processing runs whether or not logging is on.

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Sub | `/side_scan_sonar/{port,starboard}/raw` | `std_msgs/UInt8MultiArray` | feeds `.svlog` |
| Sub | `/side_scan_sonar/{port,starboard}/profile` | `OmniscanProfile` | feeds processing |
| Sub | `/blueboat/odom` | `nav_msgs/Odometry` | pose snapped per ping, 5 s buffer |
| Sub | `/sss_processor/log/enable` | `std_msgs/Bool` | GCS Record ON/OFF |
| Pub | `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | GCS |

All of the above are **VERIFIED** against the source, and the three externally-visible
names match `blueboat_gcs/config/default.yaml` exactly.

Mavros telemetry, wrapped into `.svlog` packet-id-150 mavlink envelopes
(**VERIFIED** — these are the ROS topic names, not the mavlink message names):

| Topic | Type | Becomes |
|---|---|---|
| `/mavros/imu/data` | `sensor_msgs/Imu` | `ATTITUDE` (ENU to NED) |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `GLOBAL_POSITION_INT` |
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` | cached into `relative_alt` |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | cached into `hdg` (cdeg) |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `LOCAL_POSITION_NED` |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | cached, paired with the pose |
| `/mavros/home_position/home` | `mavros_msgs/HomePosition` | `HOME_POSITION` |
| `/mavros/global_position/gp_origin` | `geographic_msgs/GeoPointStamped` | `GPS_GLOBAL_ORIGIN` |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | `VFR_HUD` |

`time_boot_ms` is derived from each message `header.stamp` so messages from one source
burst keep an identical value — SonarView pairs ATTITUDE / GLOBAL_POSITION_INT /
LOCAL_POSITION_NED on it to compute heading-corrected position.

Sonar and odom subscriptions use `BEST_EFFORT`, `KEEP_LAST`, **depth 10**. **VERIFIED.**

`ProcessedSSSPing` fields (**VERIFIED**): `port_stamp`, `starboard_stamp`,
`port_ping_number`, `starboard_ping_number`, `robot_x`, `robot_y`,
`robot_orientation` (quaternion), `water_depth`, `transducer_x_offset`,
`port_intensity_db`, `port_y`, `starboard_intensity_db`, `starboard_y`.

The sign of `*_y` encodes the side: **+y = port, −y = starboard**, and it is the sign
that is authoritative, not the field name — the `port_*` fields carry whichever message
arrived on the port topic, whose own `channel_number` may say starboard, in which case
its `port_y` is negative. Consumers merge on the sign (the GCS concatenates both fields
and reads only that). Samples are already slant-range corrected and the water column is
already removed.

**One-sided rows.** A row carries whichever sides arrived. The absent side has
`*_ping_number = 0`, a zeroed `*_stamp`, and empty `*_intensity_db` / `*_y`;
**presence is `*_ping_number != 0`, not array length**, because `project_side`
legitimately yields an empty array for a *present* side when the altitude covers
the whole swath. **VERIFIED.**

**Rows are keyed by `ping_number`, normalised across the two devices.** The two
Omniscan units are independent devices with independent counters, so a row's
`port_ping_number` and `starboard_ping_number` differ by the boat's constant device
offset — **0, −1 and +60 all measured in the field corpus** (see
`blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md`). `sss_helper.PingCounterOffset`
recovers it by voting each ping against the opposite side's temporally nearest one;
`sss_processor_node` holds a short pre-roll until the offset is known so the first
pings of a mission are keyed correctly. What signals a torn row is the gap
*changing*, never the gap being non-zero. **MEASURED.**

**Pose is a hard gate.** `_emit_group` drops the ping outright if the odom buffer
is empty, and again if the nearest-stamp lookup returns nothing; both increment
`_dropped_no_odom`. There is no fallback pose on the robot side — the GCS mitigations
(see Key GCS design decisions) exist because of this. This is the **only** drop on the
robot side. **VERIFIED.**

**Altitude never gates emission.** `FBRTracker.update` returns the best value
available — locked, else provisional from this ping's own detections, else last known
— and `locked` reports quality. When nothing has ever been detected the processor
uses `0.0`: ground range = slant range, no water column removed, which is an identity
transform rather than a guessed altitude, and is the same value the GCS
`Depth comp. = off` selector applies. `_unlocked_pings` counts pings **emitted**
while unlocked; nothing is dropped for it. **VERIFIED.**

The four transducer-geometry constants (`TRANSDUCER_X_OFFSET_M`,
`TRANSDUCER_Y_OFFSET_PORT_M`, `TRANSDUCER_Y_OFFSET_STBD_M`,
`TRANSDUCER_SUBMERSION_M`) are all **0.0** and carry `TODO` comments. **VERIFIED.**

`.svlog` files are written to `../../../../data/SSS_data`, resolved relative to the
launch working directory — the same root the GCS uses as `data_root`. The node resolves
it to an absolute path once at construction and creates it there, so every path it
reports afterwards is absolute; `SvlogWriter.start` creates it again at Record ON. A
directory that cannot be created or written is logged at **error** level and leaves
recording off — the processor still runs, because processing and logging are
independent. **VERIFIED.**

### BlueBoat GCS
rclpy node name `blueboat_gcs`. Subscribes (**VERIFIED**; topic names configurable in
`config/default.yaml`, defaults mirrored in `config/settings.py`):

| Topic | Type | Notes |
|---|---|---|
| `/sss_processor/processed` | `ProcessedSSSPing` | `BEST_EFFORT`, depth **200** |
| `/blueboat/odom` | `nav_msgs/Odometry` | `BEST_EFFORT`, depth 10 |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `BEST_EFFORT`, depth 10 |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | depth 10 |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | optional; without `mavros_msgs`, speed comes from the odom twist |
| `/blueboat/pinger_coordinates` | `std_msgs/Float32MultiArray` | the listener reads `data[0:2]` and requires `len(data) >= 2`; the producer publishes `[x, y, z]`. **Vehicle frame** by default (`alignment.pinger_frame: robot`); `config/default.yaml`'s inline comment on this key still says world and is wrong |
| `/set_path` | `nav_msgs/Path` | from `path_publisher.py` |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | placeholder, not wired to a model |
| `/rosout` | `rcl_interfaces/Log` | depth 50; the GCS filters out its own node |

Publishes: `/side_scan_sonar/ping/enable` (Bool), `/sss_processor/log/enable` (Bool),
and `/sss_ai/seabed_analysis` (`std_msgs/String`, JSON, schema 1) — image metadata +
detections, **never pixels**.

The GCS **does not subscribe to `~/raw`.** **VERIFIED.**

Every listener guards its optional imports: a missing `blueboat_interfaces` disables the
sonar stream, a missing `vision_msgs` disables detections, a missing `mavros_msgs` falls
back to the odom twist for speed — each with a status message rather than a traceback.

---

## NON-NEGOTIABLE constraints

Violating any of these breaks another module, the hardware integration, or the thesis.

### Sonar data handling

1. **Side identity comes from the packet, never from the topic or `src` tag.** Use
   `channel_number` (0 = port, 1 = starboard), falling back to the sign of
   `transducer_heading_deg`. In the raw framed packet, `channel_number` is
   **byte 34** (8-byte frame header + payload offset 26), and
   `transducer_heading_deg` is **byte 52** (float32 LE) — **MEASURED**, zero
   mismatches. Real recordings carried a wrong `src` on up to 29.5 % of packets, which
   is what produced mirrored and swapped mosaics; two logs carry `channel_number = 255`
   on every packet, where only the heading fallback identifies the side. Obeyed by the
   GCS replay path (`core/svlog.py`) and, on both its paths, by `sss_processor_node.py`
   — `_src_from_packet` for the `src` byte written into `.svlog`, `_side_geometry` for
   the projected sign. The device id the raw callbacks pass is only a fallback.

2. **Never drop a ping.** Assemble rows by `ping_number`; emit one-sided rows rather
   than discarding them; never withhold a ping because the bottom tracker has not
   locked. Arrival-time pairing plus a strict depth-lock gate was silently destroying
   ~10 % of rows and the start of every mission. Obeyed by both paths — the GCS replay
   path and `sss_processor_node`. A missing `/blueboat/odom` pose is the one remaining
   drop, and it is BlueBoat-Control's (see `TODO.md`).

   **The key is the offset-normalised counter, not the raw one.** The two units number
   their pings independently; grouping on the raw counter merges halves acquired up to
   3.0 s apart on the +60 logs in the corpus. Any new consumer or reimplementation must
   align the counters first (`sss_helper.PingCounterOffset`,
   `blueboat_gcs.core.svlog.estimate_counter_offset`) — and must vote against the
   temporally *nearest* opposite ping, since voting against the most recent one is
   bimodal between the true offset and offset ±1.

3. **`~/raw` must keep being published.** The processor writes `.svlog` from it;
   disabling it produces empty logs.

4. **Do not set the GCS sonar subscriber to `RELIABLE` on its own.** The publisher is
   `BEST_EFFORT`; a `RELIABLE` subscriber is QoS-incompatible and receives *nothing*.
   Change both ends together or not at all.

5. **Set sonar range from water depth (~4x the deepest expected), not from the area to
   cover.** An 80 m range in shallow water pushed the bottom return to sample 49/600,
   which breaks bottom detection outright.

6. **Do not modify uploaded/recorded `.svlog` files.** They are primary field data.
   The writer enforces this on its own output too: a name collision — two recordings in
   the same second, or a 500 MB roll inside one — takes a suffixed name
   (`<stamp>-001.svlog`) rather than deleting the file already there.

### GCS structure

7. **`blueboat_gcs/tools/svlog_to_rosbag.py` and `blueboat_gcs/tools/svlog_helper.py`
   are byte-identical copies** of the robot-side files
   `custom_scripts/svlog_to_rosbag.py` and `src/_custom_libraries/svlog_helper.py`,
   which live in this same repository. SHA-256 identical, **VERIFIED** (26 037 B and
   14 665 B respectively; asserted by `tests/test_sweeps.py`). Update only by
   re-copying; never hand-edit. Both directions matter: the replay window puts `tools/` on `sys.path` and imports `svlog_to_rosbag`
   as a library for "Save as rosbag" (`gui/replay_window.py:464`), so its `Converter`
   API is load-bearing, not just its `main()`.

8. **The processor executable must stay named `sss_processor_node`** and keep
   `output='screen'` in the launch file — the GCS matches that string with `pgrep -f`
   to sweep orphan processes on STOP (`pipeline.leftover_process_patterns`), and pumps
   the launch tree stdout into the embedded console. **VERIFIED** at both ends.

9. **Data leaves the GCS only through recording sessions.** If no session was active,
   STOP and app-close export nothing. **VERIFIED** in `gui/main_window.py`.

10. **No `localStorage`/`sessionStorage`-style browser storage in artifacts**, and no
    ROS types past the signal bus — everything downstream of `ros/` consumes plain
    dataclasses so the GUI runs without ROS. **VERIFIED**: `ros/` is the only place
    `rclpy` is imported, and 49 of the 53 swept GCS modules import cleanly with no
    ROS at all (the four that do not are exactly the `ros/` listeners, which
    `main.py` imports lazily and only outside `--sim`). The sweep count excludes
    `tools/`, whose three files are checked by hash instead (NC #7); the package
    holds 56 `.py` files in total.

### Scientific validity (from the thesis plan)

11. **The headline detector trains on real beach imagery only.** Training it on
    synthetic data would inherit aspect sensitivity from the model instead of
    measuring it. Synthetic-trained detectors are for the sim-to-real gap experiment
    only.

12. **AI images are waterfall-domain and boat-relative**, never world-frame mosaic
    crops.

13. **The policy baseline is two-pass orthogonal**, not single-pass, and must be tuned
    seriously.

14. **AI iteration happens offline against recorded rosbags, never in the water.**

15. **The belief layer is the only object the replanner reads.** Detector, sensor and
    planner changes interact only through it.

16. **Headline results are stated as model-conditional** ("in a calibrated simulation
    environment…"), never as unqualified empirical findings.

---

## Key GCS design decisions

All **VERIFIED** against the current source.

**Signal bus.** One `AppSignals` instance; `rclpy` spins on a background thread
(`SingleThreadedExecutor`) and all data crosses to the GUI thread as queued Qt
signals. The GUI thread owns all state, so there are no locks. This is what makes the
replay window possible as a second, fully independent instance of the same stack.

**Console capture.** Three sources feed the embedded console: `core/logging_bus.py`
tees `stdout`/`stderr` and the `logging` root handler; `ros_manager` subscribes
`/rosout` (depth 50) for every *other* node ROS logger output; `pipeline_launcher`
pumps the launch subprocess stdout. The operator never needs an external terminal.

**Depth compensation** (`depth.mode`, and the `Depth comp.` selector) is the altitude
used for slant-range correction — the same concept SonarView exposes:
- `auto` — bottom detection (FBR tracker), returning locked, then provisional, then last-known
- `manual` — fixed altitude
- `off` — no correction, ground range = slant range

The waterfall changes when this changes because the waterfall is displayed in
**corrected ground range**, not raw slant range. Only intensity-vs-sample-index is
truly raw. On shallow data a wrong altitude is worse than none, so `off` is the robust
choice when bottom detection is unreliable.

**Mosaic resolution is adaptive.** `MosaicService._auto_tune_cell_size` derives the
ground-sample distance from the median across-track sample spacing over the outer half
of the swath, clamped by `mosaic.min/max_cell_size_m` (0.02–1.00 m). **MEASURED**:
21 mm on a 25.4 m/1200-sample log, 41 mm on a 20 m/600 one, 134 mm on an 80 m/600 one.
A `Resolution` selector (Auto / 2 / 5 / 10 / 15 / 25 / 50 cm) also offers fixed values;
changing it rebuilds the grid and clears accumulated data.

**Waterfall column scale** uses `SonarPing.slant_range_m` (the *configured* range),
not `max|y_local|`, which moves with the altitude estimate. Live, the configured range
is recovered exactly as `hypot(ground_max, water_depth)`; on replay it comes straight
from `length_mm`. The ring buffer is 1500 rows x 800 columns.

**Replay timeline is the log's own clock.** `load_svlog` stamps profiles from their
`timestamp_ms` and re-bases mavlink onto that clock through a measured constant: the
sonar and the autopilot each count from their own boot, and the offset between them is
per-file and arbitrary (2 644 886 / 58 363 / 56 171 / 57 566 ms across four corpus
logs) while the two tick at the same rate. `estimate_boot_skew` recovers it as the
median of `timestamp_ms − time_boot_ms`; one file-wide value suffices (p1–p99 spread
50–120 ms over a whole log). Replay at x1 is real time, measured against a wall clock.
`NS_PER_TICK` (20 ms) survives only as a fallback, and diverges deliberately from the
identically-named constant in `svlog_to_rosbag.Converter`, which must keep ticking
every packet to produce monotonic rosbag stamps. Two fallbacks, because the two clocks
fail separately:
- `timestamp_ms` unusable — zeros, or a backwards step beyond
  `STAMP_BACKSTEP_TOLERANCE_MS` (1 s) — puts the whole timeline on the tick and sets
  `SvlogMission.synthetic_clock`. The test is **per channel per segment**, never over
  file order: the writer batches by channel, so file order is non-monotonic on real
  logs while each channel's own sequence is not. The 1 s tolerance is what keeps a log
  that swaps 9 adjacent pings by one ping interval on its real clock.
- `time_boot_ms` frozen at a single value (one corpus log does this on all 2008 of its
  mavlink packets) leaves the sonar timeline intact and places poses on it instead.

**Sessions are segments.** Packet id 10 opens a recording session, and one `.svlog` may
hold several — Cerulean's harbour demo holds two, 213.8 s of acquisition either side of
a 397.8 s gap. `SvlogMission.segments` carries them (each with its own counter offset,
since a power cycle between sessions reassigns it), `duration_s` is the span and
`acquisition_s` the segments summed, and the dead time is a `MissionGap` event in
`events`. Every accumulator breaks on it rather than joining across: waterfall seam
(`WaterfallService.break_row`), rasterizer reset, trajectory subpath, and seabed window
flush (`seabed_imager.feed_pings(breaks=…)`, so no 256-row training tile spans two
sessions). Replay skips the gap and reports it; the slider keeps true mission time.

**Pose alignment** (`alignment.pose_source`, default `auto`): if embedded ping poses
sit frozen at the origin (`frozen_epsilon_m` 0.05 for `frozen_after_pings` 20 pings)
while GCS telemetry shows motion, pings are re-stamped from `RobotState`. A
GPS+compass dead-reckoning fallback (`alignment.gps_fallback`) covers a dead or
zero-frozen `/blueboat/odom`. Both are mitigations for a robot-side defect, not a fix
for it.

**Pinger frame** (`alignment.pinger_frame`, default `robot`): USBL fixes are treated
as vehicle-relative (x forward, y port) and rotated through the nearest robot pose.

**Stream health.** `SonarListener` counts three things and reports them in the
embedded console, so acquisition loss is never mistaken for a display bug:
`device_gaps` (holes in the device's own `port_ping_number`), `one_sided` (rows
carrying a single side), and `crossed_pairs` — rows whose `port_ping_number −
starboard_ping_number` left the value the *first* two-sided row established. The
expected gap is the boat's constant device offset, not zero, so a changing gap is
the defect (NC #2).

**Pipeline lifecycle.** START/STOP runs an explicit state machine with a
SIGINT, then SIGTERM, then SIGKILL ladder on the launch session group, plus an
unconditional leftover sweep after the launch process exits *and* at application
start. A `ros2 launch` that is SIGKILLed before forwarding shutdown orphans its
children; the sweep is the invariant that makes N start/stop cycles safe.

---

## Measured acquisition settings

Reference points from three real logs (**MEASURED**, and **UNVERIFIABLE HERE** —
the corpus that backs them is an external mount, see *Verification status*; full
analysis in `blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md`):

| | our sea trial | SonarView, same boat | Cerulean demo |
|---|---|---|---|
| Range | 80 m | 20 m | 25.4 m |
| Samples/ping | 600 | 600 | 1200 |
| Range sampling | 133 mm | 33 mm | 21 mm |
| Ping interval | 110 ms (9.1 Hz) | 50 ms (20 Hz) | 50 ms (20 Hz) |
| Transmit pulse | 213 µs | 66 µs | 44 µs |
| Bottom at sample | 49/600 | 269/600 | 137/1200 |
| Missing ping numbers | ~8 % | 0 | 0 |
| Wrong-`src` packets | 19.8 % | 0 | 0 |

Gain is auto on both stacks and changes on <1 % of pings — it is **not** a significant
contributor to banding. Transmit pulse length scales with range
(`pulse_len_percent x range`), so a long range degrades range resolution too.

**The default range is 20 m.** `range_length_mm` defaults to `20000` in all three
places that declare it: `src/sss_node.py`, `launch/SSS_processing_launch.py` and
`launch/SSS_simple_launch.py`, giving **33.3 mm range sampling** at the default 600
samples. This is the no-argument default, sized from water depth per NC #5.
`project_synthesis.md` §8.5's 30 m coverage pass and 15 m revisit pass remain the
experiment settings and must be passed explicitly per run. **`terminals.txt` does
not**: its Live section passes `range_length_mm:=20000`, i.e. the no-argument
default restated, and no §8.5 value appears anywhere in the file.

---

## Data produced

**`.svlog`** — SonarView-compatible stream of framed Cerulean Ping Protocol packets.
**VERIFIED** structure (`svlog_helper.py`): `BR` magic, u16 payload length, u16 packet
id, byte 6 = `src_device_id`, byte 7 = `dst_device_id`, payload, u16 checksum =
`sum(bytes[0 .. 7+N]) & 0xFFFF`. Packet ids: 10 (session JSON), 12 (view config), 150
(mavlink wrapper), 2194 (Omniscan status: 16-byte `<ffI3xB`, ~0.9 Hz per device, no
imagery — `decode_omniscan_status`; the two floats are housekeeping telemetry of
unestablished meaning), 2198 (`OS_MONO_PROFILE`). Device ids: port 1, starboard 2, platform
3. Session-metadata packets use `src=0`, `dst=0xFF`. Written by the processor; readable
by SonarView and by the GCS replay window. Files roll at 500 MB
(`MAX_LOG_SIZE_BYTES = 500 * 1000 * 1000`).

**Recording session** — `data_root/sessions/<stamp>/`, created when Record turns ON so
streaming artifacts land inside it:

```
<data_root>/sessions/2026_07_08-14_02_31/
    metadata.json          times, config snapshot, counters, topic table
    *.svlog                adopted from data_root by mtime window (+/- 10 s)
    mosaic/                sonar_mosaic.npz, sonar_mosaic.png, boat_trajectory.csv
    waterfall/             waterfall.png, waterfall_raw.npz  <- the dataset source
    detections/            detections.csv
    seabed_images/         written live, only while the session is active
```

Adopted `.svlog` files land at the **session root**, not in a `svlog/` subdirectory;
`core/recording_session.py` and `docs/HANDOVER.md` state the same layout. Anything
already filed under `sessions/` is never adopted again. **VERIFIED.**

**Seabed images** (AI, waterfall domain) — `seabed_XXXXX.png` plus
`metadata/seabed_XXXXX.json` (per-row pose/time/speed/altitude and the pixel-to-world
formula) and `metadata/seabed_XXXXX_world.npz` (per-pixel `world_x`/`world_y` grids
and the **raw float `intensity_db`**). The replay window "Save pictures from the log"
writes the identical artifacts to `seabed_images_<logname>/` next to the `.svlog`
instead.

> The PNG is display-normalized (per-image 2–98 %) for annotation tools. **Train on
> the `.npz` `intensity_db`, not on the PNG.**

Windowing: 256 rows, stride 128 (50 % overlap), 800 columns — the standard tiling
guarantee that an object smaller than the stride appears whole in at least one image.
A final truncated image flushes the remaining pings so no data is lost.

Row 0 = oldest ping, column 0 = +range (port). Pixel to world:
`y_local(i,j) = range[i]·(1 − 2j/(W−1))`, then rotate/translate by that row pose
(`world_x = x − sin(yaw)·y_local`, `world_y = y + cos(yaw)·y_local`).

---

## Commands

**GCS** — run with the working directory set to `BlueBoat-SSS/blueboat_sss/`, which is
the parent of the `blueboat_gcs` package (`build.sh` and `terminals.txt` both do this):

```bash
python -m blueboat_gcs.main --sim     # no ROS needed; full GUI on a laptop
python -m blueboat_gcs.main           # live, needs a sourced ROS 2 env
pip install -r ../requirements.txt    # or: pip install PySide6 --break-system-packages
```

**`.svlog` forensics** — offline, needs numpy + opencv only (no ROS, no display), and
run from the same directory:

```bash
python3 -m blueboat_gcs.analysis.svlog_forensics --out ~/svlog_reports LOG.svlog
python3 -m blueboat_gcs.analysis.svlog_forensics --out DIR --compare LOG... # or a directory
```
Writes `<out>/<stem>/report.md` plus a raw-slant / corrected-ground waterfall pair:
packet census (every id seen, not a whitelist), session segmentation and gaps,
per-channel acquisition parameters with mid-file change flags, PRI and ping-number gaps
computed **per channel per session**, `src`-vs-`channel_number` consistency, and the FBR
altitude distribution. Reports are deterministic apart from one `*Generated:*` line and
the PNGs are byte-stable. The `--out` is refused if it resolves inside a tree holding
`.svlog` files (NC #6). Also exposed as the `svlog-forensics` skill
(`.claude/skills/svlog-forensics/`).

**Robot (ROS 2)**
```bash
ros2 launch blueboat_sss SSS_processing_launch.py                    # processor only
ros2 launch blueboat_sss SSS_processing_launch.py with_acquisition:=True
ros2 launch blueboat_sss SSS_processing_launch.py with_acquisition:=True \
    range_length_mm:=20000 num_results:=1200
ros2 launch blueboat_sss SSS_processing_launch.py will_use_rosbag:=True
ros2 launch blueboat_sss SSS_simple_launch.py                        # acquisition only
```
`SSS_simple_launch.py` starts `sss_node.py` alone — its `sss_processor_node.py` block
is commented out. `SSS_processing_launch.py` is what the GCS START button runs
(`pipeline.launch_command`).

Build (**VERIFIED** from `build.sh`): the workspace is `~/ros2_ws`, with this
superproject checked out under `~/ros2_ws/src/BlueBoat-SideScanSonar/`.
`build.sh` runs a plain workspace-wide `colcon build`, sources a workspace-level
`env.sh`, then `cd`s to `blueboat_sss/` and starts the GCS. For this package alone:
`colcon build --packages-select blueboat_sss` then `source install/setup.bash`.

**Headless GUI testing** — the regression suite, run from `blueboat_sss/`:
```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q     # 110 tests: 85 GCS + 25 robot-side
pip install --user --break-system-packages -r ../requirements-dev.txt
```
Runtime depends entirely on what is available. With neither a sourced
`blueboat_interfaces` nor the field corpus, 50 pass and 60 skip in ~8 s; the ~160 s
figure is the full run with the corpus mounted. **The corpus path is hard-coded** in
`tests/test_processor_assembly.py` as
`/media/kyyrron/OS/Users/killi/Desktop/Research Kyutech/BlueBoat/allSvlogData`, an
external mount; every corpus test skips when it is absent, so a green suite is not
by itself evidence that the *Measured acquisition settings* numbers still hold.
The **GCS half** (`test_sweeps.py`, `test_sim_session.py`, `test_svlog_writer.py`,
`test_svlog_forensics.py`, `test_svlog_replay.py`, 85 tests) needs no ROS, no boat and
no display, and covers:
the compile sweep (**53 files, 0 failures**); the ROS-free import sweep (**49 of 53**,
the four failures being exactly `ros/{detections,pinger,sonar,telemetry}_listener`,
which take `rclpy` at module level — asserting their *identity* is what enforces
NC #10); NC #7 verbatim-copy hashes; NC #8 processor-name consistency; two `--sim`
START/STOP runs, one of them a full recording session that plants a `.svlog` in
`data_root` and asserts it is adopted to the session root; `SvlogWriter` itself —
directory creation, a reported failure on an unusable path or an unwritable file, and
NC #6 collision handling on both the start and the 500 MB roll; and the `.svlog`
forensics suite — 15 synthetic-file tests that run anywhere, plus 14 that pin the
*Measured acquisition settings* numbers against the field corpus and skip cleanly when
it is not mounted; and the replay-fidelity suite (`test_svlog_replay.py`, 38 tests —
17 synthetic, 21 corpus-gated) covering the real clock and both its fallbacks, session
segmentation and the gap breaks in every accumulator, and the id-2194 layout. Its field
assertions are cross-checked against `svlog_forensics.analyse` on the same bytes, so
each number has two independent derivations rather than one.

The **robot-side half** (`test_processor_assembly.py`, 25 tests) drives the real
`sss_processor_node` for NC #2 — row assembly, one-sided emission, the bounded flush,
the pre-roll, and the counter-offset estimator against the field `.svlog` corpus. It
needs a sourced ROS 2 workspace and skips cleanly without one, so the suite stays
laptop-runnable. It never enables logging and opens the corpus read-only (NC #6).

Availability is probed with `importlib.util.find_spec`, **not**
`pytest.importorskip`: a module-level `importorskip` raises `Skipped` during
collection, which on pytest 7.4.4 here aborts the whole session — the GCS tests
silently stop running and the suite still reports success. Use `pytestmark =
pytest.mark.skipif(...)` for any future environment-gated module.

The import sweep **blocks the ROS packages on `sys.meta_path`** rather than relying
on their absence. On the Linux development machine ROS 2 Jazzy is sourced globally,
so an observational sweep would report 53 of 53 and silently stop testing anything.

Writing a new GUI test: `QT_QPA_PLATFORM=offscreen`, drive the app with
`QTimer.singleShot` phases, end with `app.quit()`. Modal dialogs block offscreen runs
— use the `no_modal_dialogs` fixture (only `gui/replay_window.py` opens one today).
Point `data_root` at a temporary directory via `tmp_config`: `RecordingManager`
*moves* `.svlog` files into a session, so a test on the real root would destroy field
data (NC #6). Pings reach the mosaic/waterfall/imager only while `_viz_enabled` is
set, i.e. after START — a harness that calls `enable_pinging()` directly sees pings on
the bus and an empty mosaic.

**`--sim` end-to-end, as measured by the suite** (Linux, offscreen Qt): a 3 s
START/STOP cycle produces ~44 pings at the 15 Hz sim rate, each split evenly 220 `+y`
port / 220 `−y` starboard; the mosaic auto-tunes its cell size from the 0.10 m default
to 0.0753 m on the first ping; the waterfall buffer fills to `(44, 800)`; and STOP with
no active session exports nothing, as NC #9 requires. The simulator leaves
`slant_range_m` at 0.0, so the waterfall takes its documented `max|y_local|` fallback
there.

**Pre-commit hook.** `.githooks/pre-commit` runs the suite and blocks the commit on
failure. Install it once per clone with `./.githooks/install.sh` — this repository is a
git submodule, so its real hooks directory sits outside the worktree and
`core.hooksPath` is local config a clone does not inherit.

**The gate exists only in this working tree.** `.gitignore` excludes `.githooks/`,
`requirements-dev.txt` and `.claude/specs`, so a fresh clone gets neither the hook nor
the dev requirements that let the suite run. `blueboat_sss/tests/`,
`blueboat_sss/pytest.ini`, `blueboat_gcs/analysis/` and `.claude/skills/` are present
but **untracked**, so a fresh clone gets neither the suite nor the forensics tool.

No linter or type-checker is configured.

**Dependencies.**
- GCS (`requirements.txt`): `PySide6>=6.5`, `numpy>=1.24`, `opencv-python>=4.8`,
  `PyYAML>=6.0`; live mode additionally needs `rclpy` and `blueboat_interfaces`, and
  optionally `vision_msgs` / `mavros_msgs`.
- Tests (`requirements-dev.txt`): the above plus `pytest>=7.0`, with `numpy<2` and
  `opencv-python<5` pinned — ROS 2 Jazzy's Python packages and `scipy` 1.11 are built
  against the numpy 1.x ABI, and an unpinned install resolves numpy 2.x and leaves
  `scipy` outside its supported range.
- Robot side: `bluerobotics-ping` (`brping`), `simple_launch` (both launch files),
  `scipy` (`math_helper.py`), plus `mavros_msgs` and `geographic_msgs`. `rosbag2_py` is
  needed for `svlog_to_rosbag.py`. Nothing installed by `CMakeLists.txt` imports
  `matplotlib`; only the offline `custom_scripts/processed_sss_printer.py` does.

`pip` in some environments needs `--break-system-packages`.
