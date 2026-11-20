# BlueBoat GCS — testing and performance notes

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

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
START/STOP cycle produces ~44 pings at the 15 Hz sim rate of the time (20 Hz since
2026-10-27, `SimConfig.ping_hz` = SonarView's / the driver's 50 ms; the test scales its
bound with the rate), each split evenly 220 `+y`
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
