# BlueBoat GCS — data produced by the GCS

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## Data produced by the GCS

**Recording session** — `data_root/sessions/<stamp>/`, created when Record turns ON so
streaming artifacts land inside it:

```
<data_root>/sessions/2026_07_08-14_02_31/
    metadata.json          times, config snapshot, counters, topic table, and
                           "simulation": the simulated sonar's run record (renderer,
                           profile sha256, world, seeds, build; its latched
                           /side_scan_sonar/ground_truth/run_info via core/sim_run_info.py,
                           2026-11-11) -- null on real water
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

**Late pings (2026-11-02).** A ping older than the newest buffered one (a burst
delivered ~1 s late over the WiFi hop: ~20 pings, measured on the 10-28 pool sessions,
where picture rows ran ~21 pings back in time) is inserted at its time order with its
along-track position interpolated in time, in both geometries
(`SeabedImager._late_index`; `test_late_pings_keep_picture_rows_chronological`).
Still open: at pivot turns the navigation point (GPS near the bow on the pool corpus)
outruns the transducers, so square rows repeat pings (30–65 % on `weight_1time`); the
fix needs the along-track estimator fed the transducers' own track. Every ping carries the
arm since 2026-11-15 (`sensor_x` / `sensor_y`), but the estimator still takes the
navigation point: changing it changes which ping each row copies, and the arm is adopted,
not measured (−0.345 m drawing vs −0.60 m from the pool's navigation fix). Each row carries
`yaw_rate_rad_s`.

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

**Georef convention 3 (2026-11-15): convention 2 placed from the transducers' point.**
A picture whose pings carry a fore/aft arm `a` (`SonarPing.x_offset_m`) computes
`world_x = x + cos(yaw)·a − sin(yaw)·y_local`, `world_y = y + sin(yaw)·a + cos(yaw)·y_local`
and writes `georef = {"convention": 3, "altitude": "bottom_m", "lever_arm_m": {"x": a,
"y_port": 0.285, "y_stbd": 0.285}}` with the convention-3 formula text
(`seabed_imager.georef_for`, `FORMULA_CONVENTION_3`). **No key is added and the schema
stays 5**: `lever_arm_m.x` existed (it was 0.0). The row pose in `rows_data` stays the
navigation pose; the dB, the PNG and which ping each row copies are unchanged. A picture
whose pings carry no arm is convention 2 exactly as before, grids and text, so the
Perception goldens are untouched and a recording re-processed with
`alignment.transducer_x_offset_m: 0.0` says what it said then. One arm per picture: pings
with another arm close the window, as a range change does. BlueBoat-Perception reads
`lever_arm_m.x` and (2026-11-18) applies it since 2026-11-15 (CR-PER-2, Perception `0d55d76`; re-checked by
session 12, `~/ros2_ws/sim_runs/perception/handoff_12_cr_per_2.md`: Perception's grids equal the station's
`_world.npz` bit for bit on 34/34 pictures), so its detections of a convention-3 picture are placed from the
transducers' point, as the GCS places them.

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
