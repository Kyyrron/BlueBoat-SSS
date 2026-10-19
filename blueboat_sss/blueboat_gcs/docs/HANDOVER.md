# BlueBoat GCS — Handover

## 1. What you received

A complete PySide6 desktop application (`blueboat_gcs/`) replacing
`processed_sss_listener.py`, plus a new launch file
(`launch/SSS_processing_launch.py`) and this documentation. The app runs today in two
modes:

* `python -m blueboat_gcs.main --sim` — full GUI with a built-in simulator, no ROS;
* `python -m blueboat_gcs.main` — field mode, subscribing to the real topics.

Field prerequisites: `blueboat_interfaces` sourced in the environment; the toolbar's
**Sonar** selector on *Real sonars* (the pipeline then starts `sss_node.py` itself,
pinging off until START) — or on *Simulation*, where the simulator's `sss_sim_node`
is `/side_scan_sonar`; `pip install PySide6 pyyaml opencv-python` in the workspace venv.
Everything runs on the operator laptop.

## 2. Consumed topics (already implemented, no work needed)

| Topic (config key in `config/default.yaml`) | Type | Rate | Used for |
|---|---|---|---|
| `/sss_processor/processed` (`topics.processed_ping`) | `blueboat_interfaces/ProcessedSSSPing` | ~28 Hz | mosaic, depth, trajectory |
| `/blueboat/odom` (`topics.odom`) | `nav_msgs/Odometry` | ~20 Hz | robot pose/track (throttled to 5 Hz in GUI) |
| `/mavros/global_position/global` (`topics.navsat`) | `sensor_msgs/NavSatFix` | ~5 Hz | GPS origin binding + robot info |
| `/mavros/global_position/compass_hdg` (`topics.compass_hdg`) | `std_msgs/Float64` (deg) | ~10 Hz | heading display |
| `/mavros/vfr_hud` (`topics.vfr_hud`) | `mavros_msgs/VfrHud` | ~4 Hz | ground speed (optional; falls back to odom twist if `mavros_msgs` absent) |
| `/set_path` (`topics.planned_path`) | `nav_msgs/Path` | ~1 Hz | planned-path **fallback** only (the message `path_publisher.py` sends to RViz, a fixed `total_time` window). It is dropped while the path server can be asked directly (below) |

The planned mission path itself is **asked of the path server**, as the Mission
Control Station does for its preview (`config.planned_path`,
`ros/path_listener.py`, `core/deployed_path.py`):
1. `/path_generation/get_parameters` (`trajectory`, `yaml_path`) names the running mission.
2. ONE `/path_request` over that file's whole duration (dt 0.5 s) returns world/odom-frame
   poses, drawn as they come; each path fully replaces the previous one.
3. It is asked again only when the deployment changes: the file's mtime, a server restart,
   or the overlay being cleared.
4. While a `from_yaml` file is not deployed yet, the overlay is cleared and the status bar
   says it is waiting.

Published control topics: `std_msgs/Bool` on `/side_scan_sonar/ping/enable`
(START → true, STOP → false) and `/sss_processor/log/enable`.

**NavSatFix acceptance and MCS-simulated missions.** The listener gates fixes
with the pure `utils/geodesy.navsat_fix_ok`: only an explicit
`STATUS_NO_FIX` (-1), non-finite coordinates or the `(0, 0)` no-fix sentinel
are rejected. `STATUS_UNKNOWN` (-2) is **accepted** — since ROS 2 Iron it is
the message *default*, and it is exactly what the MCS bridge's simulated GPS
sends (it fills only lat/lon). This matters because in a Gazebo run of a
GPS-anchored mission launched from BlueBoat-MCS, **MCS is the only GPS
publisher in the graph** (there is no MAVROS): its bridge node synthesises
NavSatFix on `/mavros/global_position/global` (BEST_EFFORT, ~5 Hz) from the
sim odom, but only while MCS runs and only for a mission whose trajectory
file carries a Pattern-Designer `geo_anchor`. With that feed the GCS anchors
and shows the robot, trajectory and planned path without pressing START (no
SSS nodes needed). A non-anchored sim mission has no GPS anywhere by design —
the GCS map then stays gated ("Waiting for GPS fix"); `map.require_gps_anchor:
false` is the explicit identity-frame bypass, mirroring MCS's own
"GPS n/a (simulation)" mode. The console logs the first accepted and first
rejected fix, and `GeoService` logs when fixes arrive but cannot pair with a
fresh odom position, so a filtered or unpaired GPS feed is diagnosable instead
of looking like "no GPS at all".

**Recording sessions (one experiment = one folder).** The toolbar's
"Start recording" button (enabled only while the pipeline is running) opens a
recording session: it publishes `true` once on the processor's log/enable topic
(equivalent to `ros2 topic pub --once /sss_processor/log/enable std_msgs/msg/Bool
'data: true'`) and starts session bookkeeping. **STOP acquisition ends the session**
(it always publishes `false`, closing the .svlog) and automatically assembles:

```
<data_root>/sessions/2026_07_08-14_02_31/
    metadata.json            # times, ping/detection counts, config snapshot,
                             # priority mode, display settings, adopted svlogs
    *.svlog                  # adopted from the processor (see note)
    mosaic/
        sonar_mosaic.npz     # raw planes (legacy keys + closest/oldest/newest)
        sonar_mosaic.png     # quick-look through the display pipeline
        boat_trajectory.csv  # t_since_first_s, x_m, y_m, depth_m
    waterfall/
        waterfall.png        # quick-look
        waterfall_raw.npz    # untouched native-bin buffer (archival raw
                             # record; the AI feed is seabed_images/ PNGs
                             # + their metadata/ JSON — decided 2026-09-01)
    detections/detections.csv
```

Processing scripts can treat any `sessions/*/` directory as a complete, closed
experiment. Note on the `.svlog`: it is written by `sss_processor_node` wherever that
node decides; after the session ends, every `*.svlog` under `data_root` whose mtime
falls inside the session window is *moved* to the session root. The move is **deferred
by `recording.adopt_delay_s`** (default 1.5 s) after Record OFF, because the
`log_enable=False` message is asynchronous and moving the file while the processor
still holds it open recreates a headerless stub; STOP and app-close adopt
synchronously (pinging is already off there). A session that recorded pings but
adopted nothing raises a **visible warning** instead of silently writing
`adopted_svlogs: []`. Anything under `sessions/` **or `merged_sessions/`** is never
adopted. If your processor writes elsewhere, extend the sweep in
`core/recording_session.py::_adopt_svlogs`. If no recording session was active,
STOP and application close export **nothing** — data only leaves the application
through recording sessions.

**Record OFF / STOP no longer freeze the app (2026-09-15).** `RecordingManager.end()`
writes `metadata.json` and the detections synchronously, takes immutable snapshots of
the mosaic and the waterfall (`export_snapshot()` on each service — grid planes,
model items, trajectory; buffer, metadata, model, renderer settings) and hands them
to one job on the global `QThreadPool` (the pool the seabed-image saves already use);
the compressed `.npz` writes — 2.5–7.4 s on a full mission, the measured cause of the
"STOP freezes for seconds" field symptom — happen while pings keep arriving. The
console reports "Recording session saved to …" when the job finishes.
`RecordingManager.wait_for_exports()` blocks until the pool drains: the app's
`closeEvent` calls it (never exit under a write), and so must any script that lists a
session folder right after `end()`. `session_rebuild` (offline) still writes
synchronously through the same snapshots.

**Merged sessions** (`merged_sessions/<name>/`, next to `sessions/`): the replay
window's "Merge with another svlog…" button combines two recorded logs into one
new multi-session `.svlog` (sources untouched — NC #6) and regenerates every
session artifact from it offline with the same writers a live session uses
(`core/svlog_merge.py` + `core/session_rebuild.py`). The folder has the exact
layout above and opens in the replay window like any other log; the newer log's
clocks and poses are shifted so its first ping follows the older log's last.

**Acquisition lifecycle (current workflow).** The processing pipeline is launched
automatically at application startup; all visualization layers start disabled and no
recording is active. **START** = enable pinging + live visualization (no node
restart). **Record ON/OFF** (toolbar toggle) = open / close-and-save a recording
session, fully independent from visualization. **STOP** (or closing the app) =
pinging off, active session closed and saved, ROS 2 nodes gracefully terminated
(SIGINT→SIGTERM→SIGKILL escalation with `pipeline.stop_grace_s` /
`pipeline.stop_term_grace_s`, then a sweep killing anything matching
`pipeline.leftover_process_patterns`, default `sss_processor_node`; the sweep also
runs at startup and before each relaunch). **If no recording session was active,
nothing is exported.** START after STOP relaunches the pipeline automatically.
The **Sonar** selector (*Simulation* / *Real sonars*, remembered) decides whether that
launch also starts the real driver; switching it while the pipeline is up asks, runs
STOP and relaunches (`ros/sonar_source.py`).

**Embedded console.** The "Console" toolbar button opens the bottom console dock:
Python prints and app logging (via `core/logging_bus.py`), ROS 2 log messages from
every node via `/rosout`, and the raw stdout/stderr of the launch subprocess — the
operator never needs an external terminal. Removed control: the manual Min/Max dB
dynamic-range sliders are gone; real Omniscan data (uint16 `pwr_results`, per-gain
`min/max_pwr_db` e.g. +7…+64 dB) makes a fixed dB window meaningless, so the mapping
is the display model's (`core/display_model.py`, below), with Contrast (the model's
transfer exponent, seeded from `display.gamma`) / Brightness / Colormap / Opacity as
the operator controls. The `Range EQ (waterfall)` toggle is gone (2026-09-05). Panel base width is `PANEL_MIN_WIDTH` in
`gui/main_window.py`.

## 3. Integration points

### 3.1 USBL pinger — `ros/pinger_listener.py` (now the real interface)
* Topic: `topics.pinger` (default `/blueboat/pinger_coordinates`)
* **Frame (`alignment.pinger_frame`, default `robot`):** a USBL reports positions
  relative to its transducer, so `[x, y]` is interpreted as vehicle-frame
  (x forward, y port) and rotated through the robot pose nearest the fix
  (`utils/pose_alignment.robot_to_world`); set `world` if your USBL already publishes
  odom-frame coordinates. The left panel shows the pinger's world **and** GPS position
  plus its live distance to the robot, so the real-world alignment is checkable at a
  glance. If it appears offset in the wrong direction, that is the frame setting.
* Type: `std_msgs/Float32MultiArray`, `data = [x_world, y_world]` in the world/odom
  frame [m]; extra elements ignored, NaN/short messages dropped.
* Any rate works (last fix displayed). The left panel shows live pinger world
  coordinates and the continuously updated robot↔pinger distance;
  `DEFAULT_ACCURACY_M` draws the dashed ring (no covariance in the message).

### 3.1bis AI seabed imaging & the analysis topic (new)

**Live pipeline.** While visualization runs, `core/seabed_imager.py` builds a
waterfall-domain image every `seabed.stride` pings covering the last `seabed.rows`
pings (defaults 128/256 = 50 % overlap; see ARCHITECTURE §2.16 for the
justification). **Since 2026-10-16 the GCS detects nothing itself: its one
detection source is BlueBoat-Perception** (superproject module). Every image —
recording or not — is published for it (`core/perception_bridge.py`, one dedicated
publish thread): `topics.seabed_image` (`sensor_msgs/Image` 32FC1, the raw float dB),
`topics.seabed_image_png` (mono8, the PNG) and `topics.seabed_image_meta`
(`std_msgs/String`, the JSON envelope around `SeabedImage.metadata()`), keyed
`"<stream_id>/<image_id>"`, RELIABLE / KEEP_LAST 32 (contract:
`BlueBoat-Perception/docs/topics.md`). Perception's detections come back on
`/sss_ai/detections` (§3.2). The in-process `dummy_center_analyzer` is gone (the dummy
now lives in Perception, reproducing it exactly — `tests/test_perception_golden.py`), so a
detection can never be drawn twice. The analysis is still published on
`topics.seabed_analysis` (default `/sss_ai/seabed_analysis`, `std_msgs/String`, JSON),
with `detections` always `[]`:

```json
{"schema": 5,
 "image": {"image_id": 12, "t_start_s": ..., "t_end_s": ...,
           "rows": 256, "cols": 800,
           "png_path": ".../seabed_00012.png",         // null if not recording
           "metadata_path": ".../metadata/seabed_00012.json",
           "boat": {"mean_speed_mps": ..., "mean_altitude_m": ...,
                     "start_pose": [x,y,yaw], "end_pose": [x,y,yaw]}},
 "detections": []}
```

(`schema` follows the picture metadata's `SCHEMA_VERSION`: 4 until 2026-10-16, 5 since;
`t_start_s` is the **newest** row's time.) Never the pixels — only metadata. Rationale for JSON-over-String: the
detector contract is still moving; a schema-versioned JSON needs no interface-package
release. Once frozen, promote it 1:1 to `blueboat_interfaces/SeabedAnalysis.msg`.

**Files & georeferencing.** While a recording session is active, images stream into
`<session>/seabed_images/seabed_XXXXX.png` with `metadata/seabed_XXXXX.json`
(per-row pose/time/speed/altitude + the closed-form pixel→world formula) and
`metadata/seabed_XXXXX_world.npz` (per-pixel `world_x`/`world_y` float32 grids +
the raw float `intensity_db` for training — the PNG is display-normalized — and,
since 2026-09-15, `row_gain_db`, the per-row/per-side receive-gain offset the display
model removed; the JSON's `rows_data` carries the same as `gain_port_db` /
`gain_stbd_db`). A YOLO
bbox center `(row, col)` maps to the world with one lookup:
`world = (world_x[row, col], world_y[row, col])`. **Schema 5 (2026-10-16)** adds
`stream_id` (new at every START), `georef` — the grids are now **convention 2**,
`y = ±(0.285 + sqrt(max(s² − bottom_m², 0)))` (the processor's own geometry: tracked
bottom + transducer lever arm), where schema ≤ 4 pictures used convention 1
(`altitude_m`, no lever) and stay valid as written — and per row `roll_rad`,
`pitch_rad`, `yaw_rate_rad_s`, `attitude_age_s` (from `/mavros/imu/data`, `null` when
unknown; metadata only, never geometry) and `port_ping_number` / `stbd_ping_number`.

**Truncated tail & waterfall markers.** The imager's `flush()` (Record OFF, STOP,
end of every offline pass) emits one final image with the resting pings (< 256 rows)
so no data is wasted; a mission shorter than one window yields a single truncated
image. Every Perception detection carries its pixel row's time in its own
`header.stamp` and therefore also appears as a marker on the exact ping line in the
waterfall view (live, and in replay through Run AI); markers scroll out with the ring
buffer.

**Datasets from logs.** The replay window's "Save pictures from the log" runs the
identical imager over every ping of the loaded .svlog and writes
`seabed_images_<logname>/` (+ inner `metadata/`) next to the log file — raw
to-be-annotated images. **Run AI** runs BlueBoat-Perception's core in-process over the
same pictures (`core/perception_core.py`: the installed package, else the superproject
checkout; without either it says so and draws nothing).

### 3.1ter SVLOG replay window (new)

Toolbar "Open SVLOG" → a standalone replay window per log: same
colormap/priority/contrast/opacity/view controls as the main window (the RightPanel
is reused as-is), satellite tiles + trajectory + GPS readouts when the log contains
GLOBAL_POSITION_INT, and two consumption modes — **Render range** (dual-handle
slider, e.g. begin+5 s → end−10 s, rasterized at once) and **Replay** at ×1/×2/×4/×8
driving map, waterfall and altitude exactly as live. **Save as rosbag** converts the
log to a rosbag2 (mcap) folder next to it — name dialog pre-filled
`bag_<logname>` — using the byte-identical team converter duplicated in
`blueboat_gcs/tools/` (to update it, just re-copy the file from the robot repo);
this button needs a sourced ROS 2 environment and explains itself if one is
missing. **Run AI** recreates every seabed picture from the log (256-row windows,
50 % overlap, plus the truncated tail) behind a modal progress bar and shows the
detections on the map and in the waterfall view exactly as live. The offline
processing in
`core/svlog.py` is a port of the `sss_processor_node` pipeline with its constants
copied verbatim — if those are ever retuned in the processor, mirror them there.

### 3.1quater Sea-trial pose alignment — pings frozen at (0,0) with GPS on (new)

**Symptom.** In real trials every ping locked to (0, 0) although GPS was on; the
waterfall was fine and rosbag replays of SonarView `.svlog` files worked in the replay
window. **Root cause is upstream of this app.** `sss_processor_node` snaps each
ping's pose from `/blueboat/odom` (tolerance-free nearest-stamp lookup). Live, that
topic is either publishing zeros (EKF/robot_interface not yet fused) or stamping on a
clock different from the sonar profiles, so the lookup latches a boot-time zero.
Rosbag replays are immune because `svlog_to_rosbag` **synthesizes** `/blueboat/odom`
on the same synthetic clock from real `LOCAL_POSITION_NED` — which is exactly why
"bag works, live doesn't". **The real fix is at the source: `robot_interface` must publish a
valid `/blueboat/odom` (non-zero, correctly stamped on the sonar clock).**

**GCS-side mitigations** (so trials are usable before that is fixed), all in the
`alignment` config block and `utils/pose_alignment.py`:

* `pose_source: auto` (default) — when embedded ping poses stay frozen at the origin
  for `frozen_after_pings` while GCS telemetry shows the boat has moved,
  `main_window._align_ping_pose` re-stamps each ping with the time-nearest
  `RobotState` pose before it reaches the mosaic/imager. One console warning names the
  root cause. `embedded` trusts the ping pose (legacy); `gcs` always re-stamps.
* `gps_fallback: true` — if `/blueboat/odom` is silent (> 2 s) or zero-frozen while a
  GPS fix moves, `telemetry_listener` synthesizes `RobotState` from NavSatFix +
  compass heading (`GpsPoseSynthesizer`, first-fix ENU reference), so trajectory,
  origin binding and the re-stamped pings all align on the satellite map. Normal odom
  resumes automatically when it returns.

These are safety nets, not a substitute for a correct `/blueboat/odom`; the console
warning is deliberately explicit so the operator knows to fix the source.

### 3.1quinquies Sonar stream integrity, depth compensation & resolution (new)

Full measurements and the node-side patches are in
`docs/SONARVIEW_SVLOG_ANALYSIS.md`. Operational summary:

**Side routing.** Never trust the device/topic tag. Every profile packet carries its
own `channel_number` (0 = port, 1 = starboard) and `transducer_heading_deg`
(-90/+90 — the mounting angle, constant); on real dual-Omniscan logs these agreed with
each other on 100 % of packets but disagreed with the `src` tag on 19.8 %. The svlog
reader now routes on `channel_number` and assembles rows by `ping_number`, which
removed the mirrored and swapped rows entirely (11.4 % and 15.0 % of rows before).
The channel-255 fallback is the sign of the bearing **wrapped to (−180, 180]** (one
rule, `sss_helper.side_from_fields`, mirrored in `core/svlog.py` and
`core/live_native.py`, 2026-09-15): a writer that encodes an absolute bearing — the
simulator did — put a north-heading boat's port transducer at 270, and a raw sign test
mirrored it, which is what SonarView showed on the simulator logs. A bearing of 0
resolves nothing: the processor routes by the arrival topic on both its paths and
counts it; replay counts it in `SvlogMission.unresolved_side`.

**No ping is ever dropped.**
* The reader emits a row for every `ping_number`, including one-sided ones (that is
  also why single-transducer logs, like Cerulean's harbour demo, now open).
* The bottom tracker never withholds a ping: `FBRTracker.update` returns
  locked -> provisional -> last-known, and `resolve_altitude` falls back to 0.0
  (no correction). The only remaining drop reason is a genuinely missing pose.
* Live, `SonarListener` uses `sonar_stream.queue_depth` (200) because the processor
  publishes BEST_EFFORT and BEST_EFFORT never retransmits. Do **not** set the
  subscriber to RELIABLE alone - that is QoS-incompatible with a BEST_EFFORT
  publisher and receives nothing; change both ends together.
* `SonarListener` counts gaps in the device's own `ping_number` and mismatched
  port/starboard ping numbers, and reports both in the embedded console, so
  acquisition loss is never mistaken for a display bug.

**`~/raw` must stay published.** `sss_processor_node` subscribes to it and those
framed packets are what it writes into the `.svlog`. The GCS does not subscribe to it.
**Since 2026-09-05 the GCS subscribes to the two raw `~/profile` topics** (BEST_EFFORT,
depth 200): the profiles are cached per side by device `ping_number` and attached
verbatim to the matching processed row (`core/live_native.py`), so the live waterfall
and the AI pictures draw exactly the native bins the replay path decodes from the
`.svlog` — water column and ringing included. A profile that does not arrive within
`sonar_stream.profile_wait_ms` falls back to the re-projection for that side, and the
console warns once when more than 5 % of recent rows had to.

**Defaults (2026-09-03):** the shipped view is `Depth comp. = off`, mosaic overlap
`Priority = closest` — SonarView's conventions. The combos are seeded
from `config/default.yaml` (`depth.mode: "off"`, `mosaic.priority_mode: closest`) so
the widgets match the services. Every picture's look comes from the display model
(`mosaic.nadir_contrast: true`, below).

**Depth compensation** (`Depth comp.` in the right panel, `depth.mode` in config) is
the altitude used for slant-range correction - the same control SonarView exposes:
`auto` (bottom detect), `manual`, `off` (no correction, equal to SonarView's
"Manual / 0 m", the default). It governs the **mosaic** (ground range). The
**waterfall and the AI pictures are the raw slant-bin domain** and do not warp with
it: they carry the full raw dB, water column included. On shallow data a wrong
altitude is worse than none, so `off` is the safe choice when bottom detection is
unreliable. In the replay window, changing it re-processes the log.

**Waterfall orientation, true scale, range falloff & nadir (2026-09-05).** The
waterfall shows the **newest ping on top** (SonarView convention; the live view pins to
the top, the exported PNG and the seabed pictures are newest-first) and is drawn at
**true scale** (rows stretched by along-track metres per ping over the bin pitch,
"True scale" checkbox in the view; the metres per ping come from `core/along_track.py`
— ground speed over a ~5 s window × ping period, shared with the seabed pictures since
2026-09-15, because the raw per-ping displacement reads a quarter low at 20 Hz where
pings repeat a pose sample). Every picture — live waterfall, replay waterfall,
AI seabed pictures, mosaic — maps dB to grey through the window's ONE **display model**
(`core/display_model.py`; the science and the measurements behind it are in
`docs/SCIENTIFIC_BACKGROUND.md`): the two-way transmission loss `k·log10 r + 2αr` is
removed (the stream is pre-TVG: the seabed falls 60–70 dB per decade of slant range on
the field logs), then an empirical per-side seabed curve in normalised slant range
`r/h` (the **mode** of each bin's level histogram tied to a consensus physical line —
immune to shadows, walls and targets, even a wall shadowing the whole far range;
altitude- and range-invariant) is divided out — after the ping's own receive-gain step
`20·log10(analog_gain/74.55)` (per side; the auto-gain ladder is an amplitude gain and
each step is a level shift) — then a transfer: the power law
`u = 10^(γ(e−hi)/10)` with a robust window top, a soft highlight knee and **no low
handle** (default), or `display.transfer: db_window`, linear in dB over
`window_db/γ` below the top (SonarView's recorded view-state shape; the default is
open until a paired comparison on real data — `TODO.md`). Near and far
seabed read alike, shadows go black, and the water column and ringing core darken by
the extrapolated transmission loss — **nothing is erased**, everything is **losslessly
invertible** from the model stored in every JSON/npz (`display_model`, `db = e − TL(r) +
A(r/h)`). This replaced the 2026-09-03 seabed-referenced EGN (`core/contrast.py`,
deleted): its per-column *mean* reference and 5th-percentile low handle were both
contaminated by shadows, which is what produced the bright "nadir", the vertical bands
and the grey noise inside shadows. The narrow ringing blank still applies to the
**mosaic ground projection only**, so the track line stays clean in `off` mode; the
mosaic's planes are the normalised level (`value_domain: normalised_db` in its npz),
comparable across passes.

**Mosaic resolution** is no longer a fixed 0.25 m. `MosaicService` derives the
ground-sample distance from the median across-track sample spacing over the outer half
of the swath (21 mm on a 25.4 m/1200-sample log, 134 mm on an 80 m/600 one), clamped by
`mosaic.min/max_cell_size_m`. The `Resolution` selector offers Auto plus fixed values;
changing it rebuilds the grid and clears accumulated data.

**Acquisition settings matter more than any of the above.** Set the range from the
water depth (~4x the deepest water), not from the area you hope to cover. The 80 m used
in the sea trials cost 4x coarser sampling, half the ping rate, a 3x longer pulse, and
broke bottom detection outright. `launch/SSS_processing_launch.py` now defaults to
20 m.

### 3.2 AI detections — `ros/detections_listener.py` (BlueBoat-Perception)
* Topic: `topics.detections` (default `/sss_ai/detections`), produced by
  BlueBoat-Perception — the station's **one** detection source since 2026-10-16.
* Type: `vision_msgs/Detection2DArray`, one array per seabed image (possibly empty),
  per detection: `results[0].hypothesis.class_id` (class name), `.score` (confidence),
  `bbox.center.position.x/.y` = object position **in the local odom frame** [m],
  `bbox.size_x` = extent [m], `detections[i].id` = **stable uid** (`2000000 +
  16·image_id + k`) — republishing the same id after a revisit *updates* the marker and
  does not double-count it in the summary. `detections[i].header.stamp`, when non-zero,
  is the detection's row time and places its waterfall marker (additive, 2026-10-16; a
  zero stamp keeps the array's).
* Everything the station shows of a detection — map marker, waterfall marker, left-panel
  summary, the session's `detections/detections.csv` — comes from here. `vision_msgs` is
  guarded; the no-ROS `--sim` bench simulator still draws its own synthetic markers.

### 3.3 AI pipeline placement (decided 2026-10-16)
The detector is BlueBoat-Perception, a separate module and node fed the seabed
**pictures + metadata** (§3.1bis), never ping streams or mosaic patches (CM-10). Its
offline CLI runs the same core over a recorded session's `seabed_images/`.

## 4. Other integration notes

* **Frame alignment**: the converter assumes the odom frame is ENU (mavros
  convention). If your odom frame is heading-aligned at boot, set
  `map.frame_yaw_offset_deg` (this supersedes `math_helper.local_to_enu(yaw0)`).
* **Launch files**: `blueboat_sss` ships two — `launch/SSS_processing_launch.py`
  (processor, + `sss_node.py` with the *Real sonars* selector; what the app launches)
  and `launch/SSS_simple_launch.py` (acquisition only, for running the driver by hand).
  Both are installed by `install(DIRECTORY launch ...)` in `CMakeLists.txt`. The
  matplotlib listener node is gone; this app replaces it.
* **Transducer offsets**: since `994a74b` (2026-10-13) `sss_processor_node.py` uses
  `TRANSDUCER_Y_OFFSET_PORT/STBD_M = 0.285` (BR-101447 hull spacing / 2) and
  `TRANSDUCER_SUBMERSION_M = 0.15` (an estimate), mirrored in `core/svlog.py`; the
  fore/aft offset is still 0.0 (unmeasured). The seabed pictures, the waterfall clicks
  and the detection overlay use the same lever arm since 2026-10-16 (georef convention 2).
* **Tile cache**: browse the experiment area once with internet (dock Wi-Fi); tiles are
  cached in `~/.cache/blueboat_gcs/tiles` and work offline at sea. Respect OSM/Esri
  usage terms for anything beyond research use.
* **Outputs**: STOP (and window close) writes `data/SSS_data/<date>/sonar_mosaic.npz`
  (same keys as before: `mean_intensity`, `count`, `cell_size_m`, `x0`, `y0`),
  `sonar_mosaic.png`, `boat_trajectory.csv` — drop-in compatible with existing scripts.

## 5. Interpolation: should the AI see interpolated images?

**Recommendation: no — run detection on raw mosaics only.** Reasons:

1. *Statistical integrity.* The gap fill is a local mean: it invents plausible but
   fictitious texture, smooths exactly the high-frequency content (highlight/shadow
   pairs) the detector keys on, and its footprint correlates with boat speed and turn
   geometry — a detector trained or evaluated on filled images learns artefacts of the
   survey pattern, not of the seabed.
2. *Thesis integrity.* Contribution C3 evaluates detector localization accuracy;
   interpolated cells have no measurement behind them, so any detection centred on a
   filled cell would contaminate the (detection, USBL ground truth) statistics. C5
   promises an open dataset of real sonar data; the `.npz` therefore always stores raw
   data, and this must stay true.
3. *Nothing is lost.* Small along-track gaps are sub-object-size at survey speed; a
   YOLO-class detector is robust to a few missing pixels, and the adaptive replanner's
   revisit pass fills genuine coverage holes with *real* data — which is the whole point
   of the thesis.

Use interpolation for what it is: a **display aid for the human operator** (and for
figures, clearly labelled). If you ever experiment with feeding filled patches to the
detector, `fill_small_gaps` returns the fill mask — log it alongside so interpolated
detections can be excluded from any accuracy statistic.

**Waterfall domain and AI datasets.** The Waterfall view (`View` selector, right
panel) displays raw pings stacked in acquisition order — the domain the AI datasets
are generated from — and is fully interactive (wheel zoom, drag pan, vertical
scrolling through the whole buffer; it follows the newest ping while at the top and
releases the moment you scroll into history). A small in-view control strip adds
manual zoom −/+ buttons, "Fit file", the "True scale" toggle and an "AI detections"
checkbox that toggles the detection markers (drawn on the exact ping line each object
was seen on); the strip is part of the widget, so it is present in both the main and
the SVLOG replay windows. **The AI seabed pictures use the very same mapping as this
view** (one display model per window), and since 2026-09-05 their rows are
speed-corrected to square pixels (`seabed.row_geometry: square`, provisional — revert
with `ping`; see CLAUDE.md). For raw radiometry use the data upstream of any rendering:
`waterfall/waterfall_raw.npz` in a recording session (raw dB + the model), or each
picture's `_world.npz`. Interpolation never applies in the waterfall domain, and
the mosaic-side densification (`mapping/rasterizer.py`) only resamples between
adjacent real measurements — set `mosaic.densify: false` / `bilinear_splat: false`
for strictly legacy accumulation in A/B studies. The mosaic `.npz` keeps the legacy
keys and the `closest/oldest/newest` planes; `SSS opacity` and all other Display
controls are pure visualization and never touch stored data.

## 6. Suggested future improvements

* **Processor-side GPS**: add lat/lon (and roll/pitch) to `ProcessedSSSPing` so the
  mosaic could be georeferenced without the GUI-side origin binding, and to enable
  attitude-compensated projection (C4 / Lei et al. 2026 direction).
* **Rosbag replay tab**: the listeners already tolerate any publisher; a thin
  `ros2 bag play` wrapper in the toolbar would formalise post-mission review.
* **Dirty-rect rendering**: the 4 Hz full-raster render is fine to multi-km² at
  25 cm cells; if surveys grow past that, render only the changed bounding box.
* **Belief-grid / replanner layers**: when the adaptive replanner exists, its belief
  grid and planned waypoints are two more `map_layers` classes + two signals.
* **Click-to-export patch**: right-click → save the N×N metre raw patch around the
  cursor (useful for building the detection dataset).
* **Persist UI state** (checkboxes, last view, dock sizes) via `QSettings`.
