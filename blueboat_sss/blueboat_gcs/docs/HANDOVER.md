# BlueBoat GCS — Handover

## 1. What you received

A complete PySide6 desktop application (`blueboat_gcs/`) replacing
`processed_sss_listener.py`, plus a new launch file
(`launch/SSS_processing_launch.py`) and this documentation. The app runs today in two
modes:

* `python -m blueboat_gcs.main --sim` — full GUI with a built-in simulator, no ROS;
* `python -m blueboat_gcs.main` — field mode, subscribing to the real topics.

Field prerequisites: `blueboat_interfaces` sourced in the environment; the robot-side
`sss_node.py` running (pinging off — START enables it); `pip install PySide6 pyyaml
opencv-python` on the basestation.

## 2. Consumed topics (already implemented, no work needed)

| Topic (config key in `config/default.yaml`) | Type | Rate | Used for |
|---|---|---|---|
| `/sss_processor/processed` (`topics.processed_ping`) | `blueboat_interfaces/ProcessedSSSPing` | ~28 Hz | mosaic, depth, trajectory |
| `/blueboat/odom` (`topics.odom`) | `nav_msgs/Odometry` | ~20 Hz | robot pose/track (throttled to 5 Hz in GUI) |
| `/mavros/global_position/global` (`topics.navsat`) | `sensor_msgs/NavSatFix` | ~5 Hz | GPS origin binding + robot info |
| `/mavros/global_position/compass_hdg` (`topics.compass_hdg`) | `std_msgs/Float64` (deg) | ~10 Hz | heading display |
| `/mavros/vfr_hud` (`topics.vfr_hud`) | `mavros_msgs/VfrHud` | ~4 Hz | ground speed (optional; falls back to odom twist if `mavros_msgs` absent) |
| `/set_path` (`topics.planned_path`) | `nav_msgs/Path` | ~1 Hz | planned mission path overlay (same message `path_publisher.py` sends to RViz; poses in the local odom frame; each message fully replaces the displayed path) |

Published control topics: `std_msgs/Bool` on `/side_scan_sonar/ping/enable`
(START → true, STOP → false) and `/sss_processor/log/enable`.

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
    mosaic/
        sonar_mosaic.npz     # raw planes (legacy keys + closest/oldest/newest)
        sonar_mosaic.png     # quick-look through the display pipeline
        boat_trajectory.csv  # t_since_first_s, x_m, y_m, depth_m
    waterfall/
        waterfall.png        # quick-look
        waterfall_raw.npz    # untouched ping buffer -> AI dataset source
    detections/detections.csv
    svlog/*.svlog            # adopted from the processor (see note)
```

Processing scripts can treat any `sessions/*/` directory as a complete, closed
experiment. Note on the `.svlog`: it is written by `sss_processor_node` wherever that
node decides; after the session ends, every `*.svlog` under `data_root` whose mtime
falls inside the session window is *moved* into `svlog/`. If your processor writes
elsewhere, extend the sweep in `core/recording_session.py::_adopt_svlogs`. If no
recording session was active, STOP and application close export **nothing** — data
only leaves the application through recording sessions.

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

**Embedded console.** The "Console" toolbar button opens the bottom console dock:
Python prints and app logging (via `core/logging_bus.py`), ROS 2 log messages from
every node via `/rosout`, and the raw stdout/stderr of the launch subprocess — the
operator never needs an external terminal. Removed control: the manual Min/Max dB
dynamic-range sliders are gone; real Omniscan data (uint16 `pwr_results`, per-gain
`min/max_pwr_db` e.g. +7…+64 dB) makes a fixed dB window meaningless, so the range is
always derived from data percentiles, with Contrast/Brightness/Colormap/Opacity as
the operator controls. Panel base width is `PANEL_MIN_WIDTH` in
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
justification). Each image goes through the analyzer — currently
`dummy_center_analyzer`, which "detects" the image center; **replace that one
function with the real model**, keeping its contract: `SeabedImage -> [ {pixel:
[row, col], world: [x, y], class_name, confidence} ]`. Detections appear on the map
and the analysis is published on `topics.seabed_analysis`
(default `/sss_ai/seabed_analysis`, `std_msgs/String`, JSON):

```json
{"schema": 1,
 "image": {"image_id": 12, "t_start_s": ..., "t_end_s": ...,
           "rows": 256, "cols": 800,
           "png_path": ".../seabed_00012.png",         // null if not recording
           "metadata_path": ".../metadata/seabed_00012.json",
           "boat": {"mean_speed_mps": ..., "mean_altitude_m": ...,
                     "start_pose": [x,y,yaw], "end_pose": [x,y,yaw]}},
 "detections": [{"pixel": [128, 400], "world": [x, y],
                  "class_name": "dummy_center", "confidence": 0.5}]}
```

Never the pixels — only metadata + analysis. Rationale for JSON-over-String: the
detector contract is still moving; a schema-versioned JSON needs no interface-package
release. Once frozen, promote it 1:1 to `blueboat_interfaces/SeabedAnalysis.msg`.

**Files & georeferencing.** While a recording session is active, images stream into
`<session>/seabed_images/seabed_XXXXX.png` with `metadata/seabed_XXXXX.json`
(per-row pose/time/speed/altitude + the closed-form pixel→world formula) and
`metadata/seabed_XXXXX_world.npz` (per-pixel `world_x`/`world_y` float32 grids +
the raw float `intensity_db` for training — the PNG is display-normalized). A YOLO
bbox center `(row, col)` maps to the world with one lookup:
`world = (world_x[row, col], world_y[row, col])`.

**Truncated tail & waterfall markers.** The imager's `flush()` (Record OFF, STOP,
end of every offline pass) emits one final image with the resting pings (< 256 rows)
so no data is wasted; a mission shorter than one window yields a single truncated
image. Every detection is timestamped with its pixel row's ping time and therefore
also appears as a marker on the exact ping line in the waterfall view (live and in
replay); markers scroll out with the ring buffer.

**Datasets from logs.** The replay window's "Save pictures from the log" runs the
identical imager over every ping of the loaded .svlog and writes
`seabed_images_<logname>/` (+ inner `metadata/`) next to the log file — raw
to-be-annotated images, no analyzer.

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
copied verbatim — if those are ever retuned on the robot, mirror them there.

### 3.1quater Sea-trial pose alignment — pings frozen at (0,0) with GPS on (new)

**Symptom.** In real trials every ping locked to (0, 0) although GPS was on; the
waterfall was fine and rosbag replays of SonarView `.svlog` files worked in the replay
window. **Root cause is robot-side, not in this app.** `sss_processor_node` snaps each
ping's pose from `/blueboat/odom` (tolerance-free nearest-stamp lookup). Live, that
topic is either publishing zeros (EKF/robot_interface not yet fused) or stamping on a
clock different from the sonar profiles, so the lookup latches a boot-time zero.
Rosbag replays are immune because `svlog_to_rosbag` **synthesizes** `/blueboat/odom`
on the same synthetic clock from real `LOCAL_POSITION_NED` — which is exactly why
"bag works, live doesn't". **The real fix is on the robot: publish a valid
`/blueboat/odom` (non-zero, correctly stamped on the sonar clock).**

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

These are safety nets, not a substitute for correct robot-side odom; the console
warning is deliberately explicit so the operator knows to fix the source.

### 3.2 AI detections — `ros/detections_listener.py` (placeholder)
* Expected topic: `topics.detections` (default `/sss_ai/detections`)
* Expected type: `vision_msgs/Detection2DArray` with, per detection:
  `results[0].hypothesis.class_id` (class name), `.score` (confidence),
  `bbox.center.position.x/.y` = object position **in the local odom frame** [m],
  `bbox.size_x` = extent [m], `detections[i].id` = **stable uid** — republishing the
  same id after a revisit *updates* the marker and does not double-count it in the
  summary.
* Rate: event-driven. `vision_msgs` import is guarded; if the detection repo settles on
  a custom message, edit `_msg_to_detections()` only.

### 3.3 Future AI pipeline placement
The plan in `info.md` (feed local map patches to the detector) fits naturally as a
separate node subscribing to `/sss_processor/processed` (or to saved mosaic tiles) and
publishing `Detection2DArray` — the GUI needs no change. If you prefer in-process
inference on the basestation, add a `core/detector_service.py` consuming
`signals.sonar_ping` and emitting `signals.detection`; the bus makes both options
equivalent from the GUI's perspective.

## 4. Other integration notes

* **Frame alignment**: the converter assumes the odom frame is ENU (mavros
  convention). If your odom frame is heading-aligned at boot, set
  `map.frame_yaw_offset_deg` (this supersedes `math_helper.local_to_enu(yaw0)`).
* **Launch file installation**: add `launch/SSS_processing_launch.py` to the
  `blueboat_sss` package install rules, next to `SSS_launch.py`. The old
  `SSS_launch.py` remains valid for the legacy workflow; long-term, remove
  `processed_sss_listener.py` from it once the team has switched to the GCS.
* **Transducer offsets**: still `TODO = 0.0` in `sss_processor_node.py` — measure and
  fill before localization-accuracy experiments (C3); the GUI displays whatever the
  processor publishes.
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
panel) displays raw pings stacked in acquisition order — the domain from which future
AI datasets will primarily be generated — and is now fully interactive (wheel zoom,
drag pan, vertical scrolling through the whole ring buffer; it follows the newest
ping while at the bottom and releases the moment you scroll into history). A small
in-view control strip adds manual zoom −/+ buttons and an "AI detections" checkbox
that toggles the detection markers (which are drawn on the exact ping line each object
was seen on); the strip is part of the widget, so it is present in both the main and
the SVLOG replay windows. Its pixel
values go through the display pipeline for *viewing only*; for dataset generation use
the data upstream of any rendering: the per-ping `intensity_db`/`y_local` arrays from
`/sss_processor/processed`, or `waterfall/waterfall_raw.npz` in a recording session
(the untouched ring buffer). Interpolation never applies in the waterfall domain, and
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
