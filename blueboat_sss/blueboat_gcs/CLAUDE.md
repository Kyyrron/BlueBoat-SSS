# CLAUDE.md — BlueBoat GCS (`blueboat_gcs`)

Loaded when working under `blueboat_sss/blueboat_gcs/`. Moved verbatim from
`BlueBoat-SSS/.claude/CLAUDE.md` on 2026-10-08; that file keeps the module-wide
non-negotiables (NC #1–#16, which bind the GCS too), the node-side contract and the
commands — "NC #n" and "Verification status" refer to it.

## ROS 2 interface of the GCS

rclpy node name `blueboat_gcs`. Subscribes (**VERIFIED**; topic names configurable in
`config/default.yaml`, defaults mirrored in `config/settings.py`):

| Topic | Type | Notes |
|---|---|---|
| `/sss_processor/processed` | `ProcessedSSSPing` | `BEST_EFFORT`, depth **200** |
| `/side_scan_sonar/{port,starboard}/profile` | `OmniscanProfile` | `BEST_EFFORT`, depth **200** (`sonar_stream.profile_queue_depth`). **Since 2026-09-05**: the raw per-side profiles, cached by device `ping_number` and attached verbatim to the matching processed row (`core/live_native.py`), so the live waterfall and the AI pictures draw the same native bins the replay path decodes from the `.svlog` — water column, ringing and all. A profile missing after `profile_wait_ms` (60 ms) falls back to the re-projection for that side; the console warns once when more than 5 % of recent rows had to. **Bug fix 2026-10-16:** that re-projection now inverts the processor's own geometry (the 0.285 m lever arm and its altitude = `water_depth − 0.15`); since `994a74b` it had put a missed side on the wrong bins (20 m / 1200 bins at 1.2 m: bin 83 instead of 72) — `tests/test_fallback_lever_arm.py`. |
| `/blueboat/odom` | `nav_msgs/Odometry` | `BEST_EFFORT`, depth 10 |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `BEST_EFFORT`, depth 10. Gated by the pure `utils/geodesy.navsat_fix_ok`: **only** an explicit `STATUS_NO_FIX` (-1), non-finite coordinates, or the `(0, 0)` no-fix sentinel are rejected; `STATUS_UNKNOWN` (-2) is **accepted** — it is the ROS 2 Iron+ message *default*, and exactly what the MCS bridge's simulated GPS sends for Gazebo runs of GPS-anchored missions (MCS is the only GPS publisher in a sim graph; it fills lat/lon only). Rejecting all negative statuses silently discarded that whole feed — the "GCS says no GPS while MCS anchors" field bug. First accepted and first rejected fix are logged to the console |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | depth 10 |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | optional; without `mavros_msgs`, speed comes from the odom twist |
| `/blueboat/pinger_coordinates` | `std_msgs/Float32MultiArray` | gated by the pure `utils/pinger.parse_pinger`: **all-zero vectors are dropped** (the producer streams `zeros(3)` at ~20 Hz before any USBL detection — the field "marker rides the boat" bug), NaN/short arrays are dropped, and the frame is taken from the wire shape — `[x, y, z]` = vehicle frame (normal path), `[x, y]` = world frame (`fixed_pinger`). `alignment.pinger_frame: auto` (default) honours that; `robot`/`world` force one reading. The marker hides after `alignment.pinger_stale_after_s` (10 s) without a fix |
| `/set_path` | `nav_msgs/Path` | from `path_publisher.py` — the **fallback** since 2026-09-18. That publisher asks the path service for a fixed `total_time` window (1000 s), so it truncates any longer mission: the 1317 s survey of 2026-09-07 drew as its first 1000 s and stopped mid-water. It lives in `BlueBoat-Control`, frozen at v1.0, so the station reads the trajectory **it deployed itself** instead — `core/deployed_path.py`, the newest `~/.config/blueboat_mcs/trajectories/.deployed/*.yaml`, every sample of it, placed through the designer file's `geo_anchor` — and `/set_path` only draws when there is no deployed file, no anchor, or `planned_path.prefer_deployed: false` |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | **from BlueBoat-Perception, the station's one detection source** (2026-10-16): one array per seabed image; every map marker, waterfall marker, left-panel count and `detections.csv` row comes from here. A non-zero per-detection `header.stamp` (its row time) wins over the array's |
| `/mavros/imu/data` | `sensor_msgs/Imu` | `BEST_EFFORT`, depth 10 (2026-10-16). Roll / pitch / yaw rate attached to live pings at reception with their age (`utils/attitude.py`) for the seabed-image metadata — **never geometry**; the ping quaternion is not used (yaw-only on the real boat) |
| `/rosout` | `rcl_interfaces/Log` | depth 50; the GCS filters out its own node |

Publishes: `/side_scan_sonar/ping/enable` (Bool), `/sss_processor/log/enable` (Bool),
`/sss_ai/seabed_analysis` (`std_msgs/String`, JSON, schema = the picture metadata's,
5 since 2026-10-16; `detections` always `[]`) — image metadata, **never pixels** — and,
since 2026-10-16, **every seabed image for BlueBoat-Perception** (recording or not), keyed
`"<stream_id>/<image_id>"` (`Image.header.frame_id` / the JSON envelope), RELIABLE /
KEEP_LAST 32 / VOLATILE at both ends (CM-4): `/sss_ai/seabed_image` (`sensor_msgs/Image`
32FC1, the raw float dB = `_world.npz:intensity_db`), `/sss_ai/seabed_image/png` (mono8,
= the PNG), `/sss_ai/seabed_image/metadata` (`std_msgs/String`, the envelope around
`SeabedImage.metadata()`). Built by the ROS-free `core/perception_bridge.py`, sent by
`RosManager.publish_seabed_image` from the main window's one-thread publish pool.
Contract: `BlueBoat-Perception/docs/topics.md`; the SSS side:
`docs/PERCEPTION_INTEGRATION.md`.

The GCS **does not subscribe to `~/raw`.** **VERIFIED.** It subscribes to the two
`~/profile` topics (above) since 2026-09-05.

Every listener guards its optional imports: a missing `blueboat_interfaces` disables the
sonar stream, a missing `vision_msgs` disables detections, a missing `mavros_msgs` falls
back to the odom twist for speed — each with a status message rather than a traceback.

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

**Default view (2026-09-02):** `Depth comp. = off` and mosaic overlap
`Priority = closest` (SonarView's defaults) — set in
`config/default.yaml` (`depth.mode: "off"` — quoted, since bare `off` is YAML
boolean false; `mosaic.priority_mode: closest`); the right-panel combos are seeded
from that config so the widgets agree (`RightPanel(config=...)`). The look of every
picture comes from the **display model** below (`mosaic.nadir_contrast: true`); the
Contrast slider is seeded from `display.gamma`, both services seed their renderer from
the same value, and the window re-emits the panel right after wiring it
(`RightPanel.emit_display`, 2026-09-15) so the on-screen waterfall, the mosaic, the AI
pictures and `metadata.json`'s `display_settings_at_end` start on the same transfer
(the panel's construction-time emission reached nobody: screen γ 1.0 vs PNG γ 0.7,
metadata `null` — `project_review.md` gap G4; pinned by `tests/test_display_startup.py`).
**Completed 2026-09-18:** the slider reached the screen, the mosaic and the metadata
but **not** `DisplayModel.gamma`, which is what the seabed imager writes its PNGs
through — so a session run at the operator's contrast (0.48 on 2026-09-07) saved its
AI pictures at the config's 0.7. `_on_display_changed` now sets it, and the shipped
default moved 0.7 → **0.5** (amplitude): it is what the operator kept dialling in by
hand, and it is the transfer the shadow/highlight cue survives at a metre of altitude.
Every picture stores the gamma it was written with, so older sessions stay
interpretable.

**Depth compensation** (`depth.mode`, and the `Depth comp.` selector) is the altitude
used for slant-range correction — the same concept SonarView exposes:
- `auto` — bottom detection (FBR tracker), returning locked, then provisional, then last-known
- `manual` — fixed altitude
- `off` — no correction, ground range = slant range **(default)**

The **mosaic** changes when this changes; the **waterfall** does not: it draws the
native slant-bin domain — one column per device range bin, verbatim dB, SonarView's
raw convention — and on **both** paths (live since 2026-09-05, replay since
2026-09-02) carries the **full raw dB of every bin** (`port_bin0 = 0`, water column
and ringing included), so nothing is erased and the centre is a continuous water
column darkened by the model, never a NaN hole.

**One display model for every picture (2026-09-05, `core/display_model.py`;
background and measurements in `docs/SCIENTIFIC_BACKGROUND.md`).** The device
stream is pre-TVG: the seabed falls **60–70 dB per decade of slant range** on the field
logs, so no single raw window shows near and far seabed at once. Each window (main,
replay, session rebuild) owns ONE `DisplayModel`, fed by its waterfall service, and the
waterfall, the AI seabed pictures and the mosaic all map through it:

1. **Normalisation** `e = db + TL(r) − A_side(r/h) − g_side`: the two-way transmission
   loss `TL(r) = k·log10 r + 2αr` (`display.tl_k` 40, `display.tl_alpha_db_per_m` 0.1)
   removed, the **receive-gain step** `g = 20·log10(analog_gain / 74.55)` of the ping's
   own side removed (`display.gain_compensation`, reference = gain index 4, the
   calibration anchor; `analog_gain` rides in `SonarPing.port/stbd_analog_gain` from the
   raw profile live and from the `.svlog` on replay; measured: the device's downward
   gain steps match `20·log10` of the reported ratio to ~1 dB on two harbour logs,
   `project_review.md` §3.2 — the ladder is an amplitude gain; 0 = unknown = no
   correction), then an empirical seabed curve `A` in normalised slant range `x = r/h`
   (`h` = the row's tracked bottom, `bottom_slant_m`; the processor's altitude live)
   divided out. `A` is
   estimated **per side** as the **mode** of each log-spaced `x` bin's level histogram over
   seabed samples (`r ≥ max(h, mosaic.water_column_min_m)`), tied to physics by a
   **consensus straight line in `log10 x`** (each bin proposes a line through itself at
   every slope of the physical range `[−40, 0]` dB/decade, 5 dB steps; the proposal whose
   bins within `display.curve_tolerance_db` (6) form the **longest unbroken run** wins —
   the seabed is one continuous population across grazing angle, shadows, walls and a
   shore only interrupt it — and slope + level are refined on that run, clamped; only
   inlier bins refine the shape, every other bin takes the line), median-filtered across
   bins. **Fixed 2026-09-28:** the consensus used to run at the −20 dB/decade prior alone
   and count inliers anywhere; on a flat TL-compensated seabed (the simulator's, and a
   receiver compensating part of its range loss) with a low far band on ONE side (the
   shore in the shallow rows), a −20 line through the middle bins caught that band and
   won, the near-nadir bins of that side took the line 7–17 dB above their modes, and
   the side rendered a **dark band beside the nadir on every ping, U-turns included**
   (simulated sessions `2026_09_28-10_27_47` / `-15_49_34`: port 43–54 % of samples
   off the curve → 8–11 %; the field logs unchanged or better, 128 → 96 bins > 3 dB off
   their modes over 8 recordings; `test_a_one_sided_far_band_does_not_darken_that_
   sides_nadir`). Session pictures written before the fix keep their stored model (they
   invert exactly); a replay refits and shows the corrected picture. The mode is what makes it immune to shadows, walls and targets (a per-column
   *mean* and a 5th-percentile low handle were the measured cause of the 2026-09-03
   defects: the bright "nadir", the vertical bands, grey noise inside shadows); the
   line is what survives an enclosed basin where the whole far range is a wall's shadow
   on every ping (the noise floor's TL-compensated level rises with range and cannot
   share one line with the seabed, so the seabed ridge wins the vote); the `x` axis
   makes it invariant to altitude and to the range setting, so a range change needs no
   rebuild and a remap never touches it. For `x < 1` `A` holds `A(1)`, so the
   extrapolated `TL` sends the water column and the ringing core toward black **by
   physics** — no mask, nothing erased.
2. **Transfer** — two, selected by `display.transfer` (2026-09-15):
   `power` (**the shipped default**): `p = 10^(γ·(e − hi)/10)` with a soft knee above
   `display.knee` (0.7; highlights compress instead of clipping), `hi` = `display.hi_pct`
   (95) of the normalised seabed level, `γ` = `display.gamma` (**0.5** since
   2026-09-18, was 0.7; 1 = linear power,
   0.5 = amplitude — the Contrast slider). `db_window`: linear in dB,
   `u = clip((e − (hi − W)) / W, 0, 1)` with `W = display.window_db (40) / γ`, no knee —
   what SonarView's recorded view state is (its id-12 packets carry `gamma 0.2` and a
   `signal_range_min/max` window on the dB-linear per-ping counts, `project_review.md`
   §3.2/§3.5), so the two can be compared on the same log. **No low handle** in either:
   a shadow 15–40 dB below the seabed lands at `u < 0.03` on its own. The power default
   was calibrated on the 2026-09-04 simulation log and checked against SonarView's
   screenshot of the 2026-09-03 log rendered from the same `.svlog` (median seabed
   brightness matched; black nadir, black wall shadows, textured debris, no bands); it
   stays the default until a paired SonarView comparison on **real** data decides
   (`TODO.md`).
3. **Life cycle**: live, the model accumulates `display.warmup_rows` (300) rows, then is
   **re-published every `warmup_rows` rows and never frozen** (since 2026-09-19:
   `DisplayModel.observe_row`; the earlier freeze kept the launch area's curve for the
   whole mission), every tile re-rendering on each `version` bump; replay, `Run AI`,
   `Save pictures` and the session rebuild fit it in one pass (`DisplayModel.fit`, which
   freezes). `Clear SSS data` resets it.
4. **Invertibility**: every picture's JSON carries the model (`display_model`: TL
   parameters, `x_centers`, `a_port_db`/`a_stbd_db`, `hi_db`, `gamma`, `transfer`,
   `window_db`, `gain_reference_analog`) and per row `bottom_m`, `gain_port_db`,
   `gain_stbd_db`; the `_world.npz` and `waterfall_raw.npz` carry the curves, the raw
   float dB and `row_gain_db` (n, 2); `db = e − TL(r) + A(r/h) + g`,
   `e = hi + (10/γ)·log10(u)` (power) or `e = hi − W + u·W` (db_window).
   `sonar_mosaic.npz` is tagged `value_domain: normalised_db` (its planes are `e`,
   comparable across passes).
5. **Nothing else is done to the pictures**: no despeckling, no histogram equalisation,
   no bottom-tracked mask (`docs/SCIENTIFIC_BACKGROUND.md` §9 records why).

`core/contrast.py` (the 2026-09-03 seabed-referenced EGN) is deleted;
`mosaic.seabed_high_pct` / `seabed_low_pct` / `water_column_pct` are inert.

**The on-screen waterfall is drawn at true scale (2026-09-05; estimator replaced
2026-09-15).** The service reports the along-track metres per row (5th argument of
`layout_changed`) from the **one along-track estimator** `core/along_track.py`:
ground speed = median over a `along_track.window_s` (**2 s** since 2026-10-16, was 5 s; BlueBoat-Perception decision 6.3) window of chord speeds over
~`pair_s` (1 s) spans of the ping poses, × the median ping period. The previous
running median of consecutive row displacements was biased −24 % at 20 Hz on the field
log (18 % of pings repeat a pose sample, 26 % of windows read zero → "no true scale";
`project_review.md` §3.1). The seabed imager integrates the same estimator's speed into
the along-track position its square rows sample, so the picture and the screen agree
(`tests/test_along_track.py`). The view stretches rows by that over the bin pitch
(`WaterfallView.aspect`), so a picture is not squashed at 20 Hz. Clicks go through the
same transform.

**One scale, and it says which (2026-09-18).** The "True scale" checkbox is **gone**:
a waterfall two operators read differently is a waterfall neither can measure off, so
the physical aspect is not a preference. Two things replace it. The aspect is taken
from the rows **actually on screen** (`pitch_for_rows`, over breakpoints recorded
whenever the estimate moves by more than 5 %) rather than from the newest pitch
applied to the whole history — the 2026-09-07 session changed range mid-mission
(36 → 20 → 15 m), which one transform cannot draw correctly. And the state is written
in the corner: `50 cm/ping × 10 cm/bin` normally, `!along-track scale unavailable —
square pixels` when there is no speed estimate (a stationary boat, a NaN pose), and
`!mixed along-track scale 25–50 cm/ping — drawn at 50` when the screen holds rows from
two scales. The silent fallback to square pixels was how a picture could be off by a
factor of two with nothing on screen saying so (`project_review.md` G12;
`tests/test_waterfall_pick.py`).

**The ringing blank now applies to the ground/mosaic projection only.** `project_side`
(and its robot twin `sss_helper.project_side`) is **unchanged** — it still cuts the
transmit ringing (`depth.nadir_blank_m: 0.75`, `depth.blank_nadir`,
`depth.nadir_max_fraction`) from the ground samples that feed the mosaic, so the
ringing never splatters the boat track in `off` mode. Only the *native* slant-bin
payload the waterfall/pictures use changed (it no longer subtracts the cut). Full
rationale and the dB-vs-range table in
`blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md` §5.1.

**Waterfall orientation is newest-on-top (2026-09-02, SonarView convention).** The
service still stores rows oldest-first (all absolute-index contracts intact); the
live view maps `scene y = -(row)` (tiles mirrored vertically, follow pins to the
**top**), the exported `waterfall.png` is flipped (npz stays oldest-first, tagged
`row_order`/`png_row_order`), and the seabed pictures are built newest-first (row 0 =
newest; `pixel_convention` and the world grids flipped together so bbox→world stays
one lookup).

**Mosaic resolution is adaptive.** `MosaicService._auto_tune_cell_size` derives the
ground-sample distance from the median across-track sample spacing over the outer half
of the swath, clamped by `mosaic.min/max_cell_size_m` (0.02–1.00 m). **MEASURED**:
21 mm on a 25.4 m/1200-sample log, 41 mm on a 20 m/600 one, 134 mm on an 80 m/600 one.
A `Resolution` selector (Auto / 2 / 5 / 10 / 15 / 25 / 50 cm) also offers fixed values;
changing it rebuilds the grid and clears accumulated data.

**Waterfall column scale** uses `SonarPing.slant_range_m` (the *configured* range),
not `max|y_local|`, which moves with the altitude estimate. Live, the configured range
is recovered exactly as `hypot(ground_max, water_depth)`; on replay it comes straight
from `length_mm`.

**The waterfall buffer is growable and tiled, not a ring — and native slant-bin
(2026-09-01).** Rows are the device's own range bins, verbatim (no fixed 800-column
grid, no resampling, hole-free by construction; the old ground-range scatter left up
to ~25 % of every short-range row as NaN pepper, vertical dead columns and a fake
nadir band). Width adapts to the acquisition — a recording holding two range
settings re-lays the buffer to the biggest extent at the finest pitch, existing rows
remapped by pure pixel stretch. Genuine acquisition loss (`SonarPing.gap_before`, a
jump in the merged device counter = an instant where BOTH halves vanished) inserts
blank rows, capped at 5 per gap; single-side loss stays a half-dark one-sided row.
Rows live in 512-row tiles (float32 data + per-row
`(t, x, y, yaw, water_depth, h)` metadata — `h` is the row's altitude for the
display model, so a remap needs no re-estimation); a row's
index is its chronological ping index forever, renders touch dirty tiles only
(`layout_changed` + `tile_updated(first_row, QImage)` replaced the old whole-image
signal), and auto contrast is one **global** window applied over the rendered tiles
so they cannot band at their seams. By default (`mosaic.nadir_contrast`) the tiles
render through the window's display model (unit brightness, see above); with
`nadir_contrast` off it is the plain global raw-value histogram window. Memory is capped at
`mosaic.waterfall_max_rows`
(default 100 000; oldest whole tile evicted past it, absolute indices preserved);
the replay window calls `reserve(ping_count)` so an entire file is always
scrollable. The view keeps one pixmap item per tile — scene coords are absolute
buffer pixels — with a dynamic zoom floor and a "Fit file" button so the whole
mission fits the viewport. `waterfall_raw.npz` stays full-fidelity float32; only
the `waterfall.png` quick-look is decimated above 20 000 rows (stride recorded in
the npz as `png_row_stride`).

**Waterfall click → map point.** A click (not a drag — 5 px slop, same rule as the
map) emits `point_selected(row, col)`; both windows resolve it through the row's
stored pose with `seabed_imager.waterfall_pixel_to_world` (the single definition of
the documented pixel→world formula; the detection overlay applies its exact
inverse), mark it with `SelectionLayer` under the WorldRoot, center the map, and
show world + GPS coordinates (coordinate card in the main window, status bar in
replay). Gap rows and evicted history refuse with a status message. Since
2026-10-16 clicks and overlay use **georef convention 2**, as the pictures:
`WaterfallService.row_georef` (the row's tracked bottom `h`) and
`seabed_imager.LEVER_ARM_M` (0.285 m per side).

**Runtime sonar range.** The right panel's Acquisition group (main window only,
enabled while the pipeline is RUNNING) changes `range_length_mm` live:
`PipelineLauncher.set_range` performs the documented dance — ping/enable off,
`acquisition.settle_delay_s`, `set_parameters` on `/side_scan_sonar`, ping/enable
on — resuming pinging **unconditionally** on success or failure (result crosses
the bus as `sonar_params_result(bool, str)`). A successful change inserts a
waterfall `break_row()`: the column scale adapts per ping, so the seam marks where
the horizontal scale changed. The simulator refuses gracefully (fixed swath).

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
sit frozen at **any constant** (within `frozen_epsilon_m` 0.05 for
`frozen_after_pings` 20 pings) while GCS telemetry moved > 1 m, pings are re-stamped
from `RobotState` — the original origin-only test missed the second-session variant
where the processor latched an arbitrary stale pose, and actively disengaged on it.
Release now takes 5 consecutive moving pings (hysteresis), the detector is `reset()`
on every START along with the sonar listener's counter-offset expectations and the
seabed imager's buffers, and re-stamping is refused while GCS telemetry is itself
stale (re-stamping a frozen ping from a frozen `RobotState` recreates the pile-up).
A GPS+compass dead-reckoning fallback (`alignment.gps_fallback`) covers a dead or
zero-frozen `/blueboat/odom`. All are mitigations for an upstream `/blueboat/odom` defect, not a fix
for it — the processor's half is the odom buffer's nearest-stamp tolerance above.

**Pinger gating** (`alignment.pinger_frame`, default `auto`): all-zero vectors are
rejected (the robot publishes `zeros(3)` before any USBL detection), the frame comes
from the wire shape (3-vector = body, 2-vector = world; `robot`/`world` force one),
body fixes rotate through the nearest robot pose using the heading policy, and the
marker hides after `alignment.pinger_stale_after_s` (10 s) without a fix.

**GPS-anchored map** (ported from BlueBoat-MCS `GPS_MAP_ARCHITECTURE.md`): the map
scene is local east/north metres about the first accepted GPS fix, north-up, never
rotated. `mapping/geo.py` (verbatim MCS port) estimates the **translation-only** fit
`EN = world + t` online (`core/geo_service.py` pairs fixes at GPS rate with a < 0.5 s
wall-clock odom-freshness guard); every world-anchored layer lives under one
`WorldRoot` group whose position is the fit, so a refit moves everything in O(1) and
tiles — pinned to `(lat0, lon0)`, never to `t` — cannot slide. Until the fit is valid
the map is gated: `WorldRoot` hidden, "Waiting for GPS fix" notice, clicks refused
(`map.require_gps_anchor: false` bypasses with an identity anchor, no tiles). Replay
anchors instantly via `GeoService.set_fixed_fit` from the log's recorded origin.

**Heading policy** (`alignment.heading_source`, default `compass`): live ping/marker
yaw comes from `/mavros/global_position/compass_hdg`, converted **once** at ingestion
(`θ = wrap(radians(90 − hdg))`, `utils/pose_alignment.HeadingPolicy`), falling back to
odom yaw after `compass_stale_s` (2 s). This fixed the field's misaligned live SSS
pings; replay derives yaw from mavlink ATTITUDE and is untouched. `embedded` restores
the old odom-quaternion behaviour.

**Mosaic resolution is preserved across changes.** `MosaicService.set_cell_size`
resamples the old grid into the new one (`MosaicGrid.resample_from`) instead of
wiping: old data keeps its native resolution (SonarView-style non-constant effective
resolution), the mean plane transfers its weighted sums exactly, and `cleared` is not
emitted. Auto mode re-derives the data-supported GSD every ~20 pings and applies
**refinements only** — a coarser acquisition mid-mission never degrades the display.

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

**Sonar source** (2026-09-28, `ros/sonar_source.py`, toolbar **Sonar:** selector,
remembered in `QSettings`, default `pipeline.sonar_source: simulation`). *Real sonars*
makes the pipeline launch also start `sss_node.py`; *Simulation* starts the processor
only. Two rules keep one `/side_scan_sonar`: before a Real launch the **process table**
is read (a ROS graph query at application start would race discovery) and a running
`sss_sim_node`, or an `sss_node.py` this application did not start, makes the launch fall
back to the processor with the reason on the console; and every launched process
carries `BLUEBOAT_GCS_PIPELINE` in its environment, so the sweep of
`pipeline.owned_process_patterns` kills an orphaned driver of ours and never one an
operator started in a terminal. Switching while the pipeline is up asks, runs STOP
(saving an open session) and relaunches. START's `ping/enable` is volatile, so it is sent
once `/side_scan_sonar` subscribes to it (`ping_enable_wait_s`, 10 s, then anyway with
the likely cause for the selected source) — a relaunch, or a real driver still coming
up, used to swallow it.

**Recording lifecycle guards** (the "only the first session gets a .svlog" field
bug): `PipelineLauncher.start()` and `set_recording()` return bool, and the GUI
honours the refusals — a Record ON the processor never saw opens **no** session
(the button unchecks), and a refused START (launcher still in its stop ladder)
leaves the viz gate closed instead of greying the toolbar out with nothing
running. A pipeline drop while recording closes the session properly (the toolbar
uncheck alone left `RecordingManager` active, silently reusing the folder).
Adoption is **deferred** by `recording.adopt_delay_s` (1.5 s) after Record OFF —
moving the file while the processor's async `log_enable=False` was still in
flight made its next append recreate a headerless stub — and runs synchronously
on STOP/app-close (pinging already off); `merged_sessions/` is excluded from the
sweep, sub-frame stubs are skipped (never deleted), and an empty adoption on a
session with pings warns loudly.

## Data produced by the GCS

**Recording session** — `data_root/sessions/<stamp>/`, created when Record turns ON so
streaming artifacts land inside it:

```
<data_root>/sessions/2026_07_08-14_02_31/
    metadata.json          times, config snapshot, counters, topic table
    *.svlog                adopted from data_root by mtime window (+/- 10 s)
    mosaic/                sonar_mosaic.npz, sonar_mosaic.png, boat_trajectory.csv
    waterfall/             waterfall.png, waterfall_raw.npz  <- archival raw record
                           (the AI feed is the seabed PICTURES + metadata)
    detections/            detections.csv
    seabed_images/         written live, only while the session is active
```

Adopted `.svlog` files land at the **session root**, not in a `svlog/` subdirectory;
`core/recording_session.py` and `docs/HANDOVER.md` state the same layout. Anything
already filed under `sessions/` or `merged_sessions/` is never adopted again.
**VERIFIED.**

**Merged sessions** — `data_root/merged_sessions/<name>/`, next to `sessions/`.
The replay window's "Merge with another svlog…" button (`core/svlog_merge.py` +
`core/session_rebuild.py`) combines two recorded logs into ONE new multi-session
`.svlog` and regenerates every session artifact from it offline with the same
writers the live path uses, so the folder is a normal session in every way and
opens in the replay window unchanged. Mechanics: the older log (first id-10 wall
clock, else mtime) goes first byte-identical; the newer log keeps its own id-10
header (one is synthesized if absent) so it stays a separate segment — which is
what makes ping numbers collision-free and keeps `usable_stamps` per-clock — and
gets three constant shifts: sonar `timestamp_ms` += Δ so its first ping lands one
median PRI after the older log's last (the resulting one-PRI `MissionGap` still
breaks every accumulator), mavlink `time_boot_ms` += Δ + skew_B − skew_A so the
file-wide boot-skew median stays coherent, and `LOCAL_POSITION_NED` x/y shifted
onto the older log's frame via the two GPS origins when both exist (pass-through
+ warning otherwise; `GLOBAL_POSITION_INT` is absolute and untouched — the loader
takes the FIRST origin, the older log's). **NC #6: both sources are opened
read-only and never modified** (pinned by a SHA-256 regression test); the id-10
headers gain ignored provenance keys (`merged_from`, `merge_time_shift_ms`).
NC #9 posture: this is an offline transformation of data that already left
through sessions, not a new live-export path.

**Seabed images** (AI, waterfall domain) — `seabed_XXXXX.png` plus
`metadata/seabed_XXXXX.json` (per-row pose/time/speed/altitude/bottom/source ping, the
display model, the pixel-to-world formula) and `metadata/seabed_XXXXX_world.npz`
(per-pixel `world_x`/`world_y` grids, the **raw float `intensity_db`**, the model
curves, `row_ping_index`). The replay window "Save pictures from the log" writes the
identical artifacts to `seabed_images_<logname>/` next to the `.svlog` instead.

> The PNG is the **display model's transfer** (see *One display model* above): the same
> `e = db + TL(r) − A(r/h)` and `u = 10^(γ(e−hi)/10)` as the waterfall and the mosaic,
> one model for every tile of a mission → the same seabed reads at the same brightness
> (what a detector needs), and it is **losslessly invertible** from the JSON/npz. **The
> AI feed is the pictures + their JSON metadata (decided 2026-09-01)**; the `.npz` is an
> auxiliary record carrying the raw float `intensity_db`.

The tiles carry the **full raw dB, water column included**: nothing is erased, the
nadir is darkened by the model (extrapolated transmission loss), the thin first-bottom-
return line survives as a bright centre edge, as in SonarView. Tiles are hole-free
(CM-10), continuous, and invertible; no despeckling, no equalisation.

**Row geometry — `seabed.row_geometry: square` (decided 2026-09-05, PROVISIONAL).**
A picture row is one across-track bin pitch of along-track distance, so a pixel is the
same size along- and across-track and an object keeps its shape at any boat speed (the
sidescan literature's "speed correction"; SonarView draws true scale). Each row copies
exactly one ping's bins verbatim — the ping nearest to the row's along-track position
(`ping_index` per row in the JSON) — and its pose/time are interpolated along-track so
the world grids stay continuous. When the boat outruns the pitch some pings are absent
from the *picture* (they remain in `waterfall_raw.npz`); when it is slower a ping fills
several rows. `rows` (256) / `stride` (128) count picture rows: at 25–53 mm pitch a tile
is 6.4–13.6 m along-track. **If detector results are worse with square pixels, revert
with the one line `seabed.row_geometry: ping` in `config/default.yaml`** — that is the
previous contract (one row per ping, `rows`/`stride` in pings), kept in the code and in
`tests/test_seabed_pictures.py`; nothing else changes.

Windowing: a range change ends the window so every image is one homogeneous grid; the
50 % overlap is the standard tiling guarantee that an object smaller than the stride
appears whole in at least one image. **The overlap is deliberate**: consecutive images
sharing their middle half is the tiling guarantee working, not a registration bug — do
not "fix" it without weakening the detector dataset. A final truncated image flushes the
remaining rows so no data is lost.

Row 0 = **newest** (SonarView orientation; every row-indexed array and the world grids
are flipped together so bbox→world stays one lookup), column 0 = +range (port). Pixel to
world, **georef convention 2** (schema 5, 2026-10-16 — the processor's `project_side`
geometry): `k = (W/2−1−j)` or `(j−W/2)`, `s = (k+0.5)·bin_pitch_m`,
`y_local = ±(lever_side + sqrt(max(s² − bottom_i², 0)))` with `bottom_i` = the row's
`bottom_m` and `lever_side` = 0.285 m (`core/svlog.py` `TRANSDUCER_Y_OFFSET_*`), then
rotate/translate by that row pose (`world_x = x − sin(yaw)·y_local`,
`world_y = y + cos(yaw)·y_local`). The JSON says so in `georef`; a schema ≤ 4 picture
(no `georef`) used **convention 1** (`altitude_i` = `altitude_m`, the correction altitude +
submersion, and no lever) and its stored grids stay valid as written.
`waterfall_pixel_to_world`'s default arguments are convention 1 bit for bit
(`tests/test_perception_golden.py`).

**Schema 5 (2026-10-16), additive keys** (`tests/test_perception_golden.py` pins that
nothing else changed): `stream_id` (`gcs-<UTC>-<4 hex>`, new at every
`SeabedImager.reset()`, i.e. START; `image_id` restarts at 0 in each), `georef`, and per
row `roll_rad` / `pitch_rad` / `yaw_rate_rad_s` / `attitude_age_s` (the row's **source
ping's** — live the latest `/mavros/imu/data` sample at its reception, replay the latest
`.svlog` ATTITUDE before it; `null` when unknown) and `port_ping_number` /
`stbd_ping_number` (0 = side absent). `detections` is always `[]`: the station's
detector is BlueBoat-Perception. The replay window's **Run AI** runs Perception's core
in-process over the same pictures, through the live envelope
(`core/perception_core.py`).

## Testing and performance notes

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

**GCS live-pipeline budget (2026-09-03).** With every ping arriving as one
two-sided row at 20 Hz the app slowed and died: `MosaicService._derive_cell_size`
took the median spacing of the *merged* port+starboard `y_local` (interleaved by
any transducer asymmetry → the `min_cell_size_m` clamp, 3× the cells), and
`MosaicGrid._ensure_contains` padded seven planes with no cap before the budget
was checked (±200 m at 2 cm = 7 GB). Now: the cell size comes from one side; the
grid honours `max_grid_cells` **before** allocating by coarsening in place
(`MosaicGrid.coarsen`, block aggregation, never two grids resident) and refuses a
growth past `max_extent_m` (pose glitch); the mosaic re-colormaps only the dirty
sliver between full passes at `mosaic_render_hz` (2 Hz) with a 5 s auto-range
hold; the waterfall re-renders a window move `_STALE_PER_PASS` tiles per pass
(never the whole buffer), freezes its auto window after 2000 rows, debounces
the display sliders, resolves a detection's row once, caps memory in
**samples** (`waterfall_max_samples`, rows × native columns) and its view drops
pixmaps beyond `_MAX_ITEMS` tiles (re-requested on scroll); seabed-image saves
run on a pool thread; and the main window pauses rendering (never ingestion)
when the queue is more than `ping_lag_throttle` pings behind
(`AppSignals.sonar_latest_seq`). Measured 2026-09-05 with the shared display model
wired as in the main window (`python3 -m blueboat_gcs.analysis.ping_bench --pings
4000 --turn`): ingest 0.85 ms/ping, mosaic render ~3 ms (sliver) / 43 ms max,
waterfall ~18 ms per pass (the model's per-tile gather + power transfer), RSS 510 MB
flat after the caps. `tests/test_gcs_throughput.py` (9 tests) pins the caps.
