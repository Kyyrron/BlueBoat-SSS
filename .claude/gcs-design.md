# BlueBoat GCS — key design decisions (full text)

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

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
2026-10-16 clicks and overlay use the pictures' georef convention:
`WaterfallService.row_georef` (the row's tracked bottom `h`),
`seabed_imager.LEVER_ARM_M` (0.285 m per side) and, since 2026-11-15, the row's
fore/aft arm (below); both windows resolve a click with one call,
`WaterfallService.pixel_to_world(row, col)` (None on a gap or evicted row).

**Fore/aft arm: a ping is placed from the transducers' point (2026-11-15, CR-SSS-2).**
The transducers sit `a` = −0.345 m along the heading from the navigation pose (0.345 m
aft; adopted from the simulator's profile, not measured — the module `CLAUDE.md`,
*Fore/aft arm*). Until this date every placement started from the navigation point, so
one object seen on two opposite headings landed `2·|a|` = 0.69 m apart.
- **The rule, one definition:** `models/sonar.sensor_point(x, y, yaw, a)` =
  `(x + cos(yaw)·a, y + sin(yaw)·a)`; `SonarPing.x_offset_m` carries `a` (a non-finite
  value becomes 0.0 in `SonarPing.__post_init__`, so no consumer re-checks it) and
  `SonarPing.sensor_x` / `sensor_y` apply it to the ping's **final** pose (after the live
  re-stamping of pose and compass yaw). `a = 0` returns the pose itself, the same objects,
  so everything computed without an arm is bit for bit what it was.
- **Where `a` comes from:** live, the ping's own `ProcessedSSSPing.transducer_x_offset`
  (`core/live_native.transducer_x_offset_of`; the processor is the one source in a live
  graph, and one built before 2026-11-15 sends 0.0 = the old placement); replay, "Save
  pictures" and the session rebuild, `alignment.transducer_x_offset_m` (default −0.345 =
  `core/svlog.TRANSDUCER_X_OFFSET_M`), passed to `load_svlog(transducer_x_offset_m=…)` by
  `core/svlog.load_svlog_with_config`, the one call the replay window and the rebuild make.
  `0.0` there re-places a recording as before. The `--sim` bench simulator sends no arm.
- **Where it is applied:** the mosaic (`mapping/rasterizer.py` builds its ping line, and
  the virtual pings between two real ones, from the sensor point; the legacy scatter in
  `core/mosaic_service.py` too), the swath line (`gui/main_window.py`), the waterfall click
  (both windows: `WaterfallService.pixel_to_world`, which passes the row's arm as
  `waterfall_pixel_to_world(..., lever_x=…)`; the arm is the ninth per-row metadata column,
  in memory only) and the seabed-picture grids (below).
- **Where it is not, on purpose:** `boat_trajectory.csv` and every boat marker (the
  navigation point); the row poses in the pictures' `rows_data` (navigation pose, the arm
  is in `georef`); the along-track estimator and the square-row sampling (unchanged, so
  the detector's input is byte-identical; `TODO.md`); the detection overlay (its row is the
  detection's time, its column the across-track component: an arm along the heading
  changes neither).
- **Recorded:** `metadata.json` `transducer_x_offset_m` (the arm of the session's pings;
  a list if it changed, `null` without pings), every picture's `georef`.
- Pinned by `tests/test_fore_aft_arm.py`: a point seen heading east and heading west
  coincides with the arm and is 0.69 m apart without it, on every path above.

**Runtime sonar range.** The right panel's Acquisition group (main window only,
enabled while the pipeline is RUNNING) changes `range_length_mm` live:
`PipelineLauncher.set_range` performs the documented dance — ping/enable off,
`acquisition.settle_delay_s`, `set_parameters` on `/side_scan_sonar`, ping/enable
on — resuming pinging **unconditionally** on success or failure (result crosses
the bus as `sonar_params_result(bool, str)`). A successful change inserts a
waterfall `break_row()`: the column scale adapts per ping, so the seam marks where
the horizontal scale changed. The GCS's built-in `--sim` simulator refuses gracefully
(fixed swath). The simulated sonar node `sss_sim_node` accepts the change like the real
driver, re-reading its acquisition on the re-enable. With its **ray-cast** renderer, keep
the range ≤ 30 m: above 40 m the GPU runs out of memory and the node skips every cycle
(the simulator's README, "Range advice for the ray-cast", measured 2026-11-07).

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

**Stream clock** (2026-11-07, `core/stream_clock.py`). Every world-timing rule of the
GCS — the attitude / compass freshness attached to each ping (`attitude_age_s`,
`compass_stale_s`), telemetry staleness, the odom-dead GPS fallback, the geo pairing's
odom freshness, the odom emit throttle, mission time and a session's `duration_s` —
reads `stream_now()`, not `time.monotonic()`. In a real-robot graph (no `/clock`
publisher) it *is* `time.monotonic()`. In a simulation graph `ros_manager` feeds it
`/clock`, and it advances in sim seconds (continuous, interpolated, never backwards,
wall-paced after 3 s of `/clock` silence), so a simulation slowed below real time (the
ray-cast sonar) records what a real-speed one does. `started_utc` / `ended_utc` stay wall;
the profile join (`profile_wait_ms`) stays wall (a latency). `tests/test_stream_clock.py`.

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
up, used to swallow it. **Bag replay (2026-11-02):** an MCS "From a ros2bag" replay starts
no sonar (MCS owner decision, 2026-10-27), so a simulated session recorded during one held
only the replayed MAVLink — "no usable pings". With *Simulation*, START now reads the
process table (`sonar_source.find_replay`: a running `bag_replay.py` and the `gz sim
<folder>/world.sdf` a world folder with `scene.npz` runs) and, when nothing is
`/side_scan_sonar`, starts `ros2 launch blueboat_sss_sim sss_sim_launch.py
with_mavros_shim:=false auto_ping:=false world_dir:=<folder>`
(`pipeline.replay_simulator*`) as a second launch tree of ours (`OWNER_ENV`, stopped
with the pipeline, `sss_sim_node` swept owned-only), waiting up to
`replay_simulator_wait_s` (30 s) for it to load. The replay feeds it the recorded pose
(`/sim/sea_state/boat_pose`, 100 Hz) and the recorded `/mavros/*`. Verified end to end
2026-11-02 on `pool_1w` + the 12:13:18 pool bag: 727 two-sided pings in 39 s, all
placed (`tests/test_sonar_source.py`). The replay window's "no usable pings" dialog now
tells "no sonar profile at all" from "profiles without a pose" (`no_pings_message`).

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
session with pings warns loudly. Record ON waits for `sss_processor` to subscribe to
`log/enable` before publishing (2026-11-11, `_poll_log_enable`, the `ping/enable` rule):
"running" is a 2 s timer, and an enable sent 5 ms before the processor was up left a
session without a `.svlog`.
