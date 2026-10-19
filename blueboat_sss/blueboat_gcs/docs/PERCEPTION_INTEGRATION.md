# Perception integration — what changed in BlueBoat-SSS, and what stays

**Status: implemented 2026-10-16** (Phase 2 of the BlueBoat-Perception work). Every claim
below is pinned by a test named in §5.

BlueBoat-Perception (superproject module) consumes the GCS's seabed images and returns
detections. Its design and interface contract are in `BlueBoat-Perception/docs/DESIGN.md`
and `docs/topics.md`. This file is the SSS side: which files changed, how, and what is
proven unchanged.

Authority — the owner's orders:
- **Task B** (2026-10-15): small additive changes so the two modules work together.
- **Q6**: fix the seabed-image pixel→world now (convention 2). This is one of two
  behaviour changes.
- **Q8**: per-row attitude and device ping numbers in the image metadata.
- **2026-10-16, answer 3**: **one detection source only — Perception.** The station's
  in-process dummy and any source choice are removed, so detections can never be drawn
  twice. Everything else the station does stays as it is.
- **2026-10-16, answer 1**: the profile-miss fallback fix, as a separate, labelled bug fix
  (the other behaviour change).

---

## 1. What stays exactly as it is
- **Image formation, bit for bit.** Native slant bins verbatim, square or ping geometry,
  windowing, `break_window`, `flush`, the display model, the PNG, the session folder
  layout and NC #9 (nothing is exported without a session).
  - The raw dB, the PNG, the row times and poses, and the saved PNG bytes are identical
    to the pre-change code on 146 golden images: 2 row geometries × 2 model wirings
    (fitted and live), over a U-turn, heave, a gain step, one-sided pings, a range change,
    a session break and a target with its shadow.
- **Every existing topic** keeps its name, type, QoS and meaning.
  - `/sss_ai/seabed_analysis` still goes out for every image. It is identical apart from
    `schema` (4 → 5) and `detections` (now `[]`).
  - `/sss_ai/detections` is still subscribed with the same type and QoS.
- **Untouched:** the processor, `sss_node.py`, both launch files and every
  `ProcessedSSSPing` / `OmniscanProfile` use. No file under `src/` changed.
- **Metadata:** every pre-existing key and per-row field is identical, except the three
  ordered changes:
  - `schema`;
  - the pixel→world `formula` text (Q6);
  - `detections`.
- **The map, waterfall markers, left panel and `detections.csv`** still consume
  `signals.detection`. Only the producer changed: now only `DetectionsListener`.
- **`--sim`** (no ROS) is unchanged. Its bench simulator still draws its own synthetic
  markers; nothing is published.
- **NC #10:** no ROS types past `ros/`. The import sweep reports 64 of 68 modules ROS-free;
  the 4 that are not are the `ros/` listeners, as before.

## 2. One detection source (answer 3)
| File | Change |
|---|---|
| `core/seabed_imager.py` | `dummy_center_analyzer`, the `analyzer` argument of `SeabedImager` and `generate_from_pings(run_analyzer=…)` are removed. Images carry `detections: []`. |
| `gui/main_window.py` `_on_seabed_image` | Publishes `/sss_ai/seabed_analysis` as before (`detections: []`) and queues the image for Perception (§3). It emits **no** in-process `Detection`. |
| `ros/detections_listener.py` | Now the station's only detection input (docstring). A non-zero per-detection `header.stamp` (its row time) wins over the array stamp; a zero stamp keeps today's behaviour. |
| `gui/replay_window.py` `_run_ai`, new `core/perception_core.py` | **Run AI** runs Perception's core in-process, through the very envelope the live path publishes. The core comes from the installed package, else the superproject checkout; without either, the status bar says so and nothing is drawn. One detector implementation. The GCS *uses* Perception, which never imports the GCS (CM-3). |

Perception's dummy reproduces the former in-process dummy exactly: pixel, world under
convention 1, row time, class, confidence and extent, on all 146 golden images. Its uid
moved from the `1_000_000 + 16·id + k` range to `2_000_000 + 16·id + k`.

## 3. Image publishing (Task B, Q1)
| File | Change |
|---|---|
| `config/settings.py`, `config/default.yaml` | New `topics.seabed_image` (`/sss_ai/seabed_image`), `topics.seabed_image_png`, `topics.seabed_image_meta`, `topics.imu` (`/mavros/imu/data`). |
| new `core/perception_bridge.py` (ROS-free) | `image_key`, `stamp`, `envelope` (`{"schema": "blueboat_sss/seabed_image/1", key, stream_id, image_id, stamp, png_path, metadata_path, metadata}`), `image_arrays`, `publish_parts`. |
| `ros/ros_manager.py` | Three publishers, all **RELIABLE / KEEP_LAST 32 / VOLATILE**, the profile Perception subscribes with (CM-4): `sensor_msgs/Image` 32FC1 (raw float dB), mono8 (the PNG) and `std_msgs/String` (the envelope). `publish_seabed_image(key, stamp, db, png8, meta_json)`. The key rides in `header.frame_id`; `header.stamp` is the newest row time. `sensor_msgs` is imported inside `ros/` only. |
| `gui/main_window.py` | A dedicated single-thread `QThreadPool` and `_PerceptionPublishJob`. Every image, recording or not, is encoded and published off the GUI thread, in order. |
| `core/seabed_imager.py` | `stream_id` (`gcs-<UTC YYYYmmddTHHMMSSZ>-<4 hex>`) is generated at construction and at every `reset()` (START) and written into the metadata. |

## 4. Georeferencing convention 2 (Q6) and Q8

**The problem.** Since `994a74b` (2026-10-13) the processor projects
`y = ±(0.285 + sqrt(s² − h²))`, with `h` its altitude (= `water_depth − 0.15`). The picture
grids still used `sqrt(s² − water_depth²)` with no lever, so the two disagreed by the
lever arm across-track.

**The change:**
- `models/sonar.py` `slant_col_to_y_local` and `seabed_imager.waterfall_pixel_to_world`
  gained `lever_port` / `lever_stbd`. The defaults (0) are convention 1 bit for bit (golden
  test).
- The imager builds its grids from the row's tracked bottom `bottom_m` and
  `LEVER_ARM_M` = `core/svlog.py` `TRANSDUCER_Y_OFFSET_*` (0.285).
- `metadata()["georef"] = {"convention": 2, "altitude": "bottom_m", "lever_arm_m":
  {"x": 0.0, "y_port": 0.285, "y_stbd": 0.285}}`, and `SCHEMA_VERSION` 4 → **5**.
- Waterfall clicks (both windows) use the same convention, through
  `WaterfallService.row_georef` (the row's `h`), and so does the detection overlay's
  inverse (`_overlay`: `ground = |y| − lever`, `s = hypot(ground, h)`).
- Older sessions are not rewritten. Their pictures have no `georef` key, which means
  convention 1, and Perception reads them that way.

**Q8, metadata only, never geometry.**
- `SonarPing` gained optional `roll`, `pitch`, `yaw_rate`, `attitude_age_s` (NaN means
  unknown) and `port_ping_number`, `stbd_ping_number` (0 means absent). Defaults keep
  every constructor valid.
- **Live:**
  - `ros/telemetry_listener.py` subscribes `/mavros/imu/data` (BEST_EFFORT 10; it exists
    in both graphs: mavros on the boat, the simulator's shim in Gazebo) and emits
    `AppSignals.attitude`;
  - the ROS-free `utils/attitude.py` `AttitudeBuffer` attaches the latest sample to each
    ping at reception, with its age; a sample older than 2 s is not attached;
  - the ping quaternion is not used, because it is yaw-only on the real boat;
  - `SonarListener` passes the two device ping numbers.
- **Replay:** `core/svlog.py` takes roll, pitch and yaw rate from the latest ATTITUDE
  before each ping. It inverts the processor's NED/FRD encoding: roll, −pitch, −yawspeed.
  The ping numbers come from the profiles.
- **Per row:** each picture row writes its **source ping's** values (the ping whose bins
  it copies) as `roll_rad`, `pitch_rad`, `yaw_rate_rad_s`, `attitude_age_s` (`null` when
  unknown), `port_ping_number` and `stbd_ping_number`.

## 5. Separate bug fix (answer 1) — the profile-miss fallback
`core/live_native.attach_native` → `models/sonar.side_bins_from_ground`.

When a row's raw profile misses its 60 ms wait, the listener re-projects that side from
the processed row. It inverted with `h = water_depth` and no lever. On the same
processor output (20 m, 1200 bins, 1.2 m altitude) that put the row at **bin 83 instead of
72** (0.18 m too far out), with its far end at 20.16 m instead of 20.0 m. It now inverts
the processor's geometry: `s = hypot(|y| − lever, water_depth − 0.15)`. The device bins
are recovered exactly. `side_bins_from_ground`'s default (`lever=0`) is the old inverse
bit for bit. A row whose profile *was* attached never used this path. Noted in SSS
`CLAUDE.md` / `TODO.md` and the GCS `CLAUDE.md` profile row.

## 6. Data on disk
- Session pictures gain the additive keys. The PNG and npz formats are unchanged; the
  npz `world_x` / `world_y` values follow convention 2.
- **No existing session is rewritten** (NC #6, CM-7). Nothing new is written outside
  sessions: the published images are a live stream, and Perception writes into its own
  roots.

## 7. Tests (the gate: 353 pass / 0 fail / 50 skip on 2026-10-16, after §9)
- **Baseline before any edit: 298 / 4 fail / 50.** The four were stale expectations after
  `994a74b`, repaired as a labelled baseline repair (no behaviour change; listed in SSS
  `TODO.md`). Gate after the repair: 302 / 0 / 50.
- `tests/test_perception_golden.py` (29). It compares against
  `tests/fixtures/perception_golden.json`, captured by
  `tests/capture_perception_golden.py` from the unmodified code over the deterministic
  pings of `tests/golden_pings.py`, and checks:
  - the pictures and saved PNGs are bit-identical;
  - the metadata changed exactly as ordered;
  - the analysis payload changed only in `schema` and `detections`;
  - the default-argument grids equal the old grids;
  - no in-process `Detection` is emitted;
  - Perception's dummy equals the old one;
  - the live Perception path georeferences with convention 2.
- `tests/test_perception_bridge.py` (16) covers:
  - the envelope and arrays are what the session saves;
  - stamps; `stream_id`;
  - the convention-2 grids;
  - each row carries its source ping's attitude and ping numbers;
  - `null` and never NaN for unknown attitude;
  - `AttitudeBuffer`; `quat_to_roll_pitch`;
  - replay ATTITUDE → ENU;
  - the publish job; `RosManager.publish_seabed_image` builds bit-exact messages;
  - the listener's per-detection stamp;
  - replay Run AI through Perception, and its message when Perception is absent.
- `tests/test_fallback_lever_arm.py` (3): the row position before and after the fix, on
  the same input.
- **Tests changed because Q6 / answer 3 / the fix require it** (each change is commented
  in the test):
  - `test_seabed_pictures.py` (schema 5);
  - `test_waterfall_pick.py` (click and overlay in convention 2, both sides);
  - `test_live_native.py` (the processor's real geometry in its helper; the fallback
    lands on the device bins);
  - `test_sweeps.py` (+3 modules: `perception_bridge`, `perception_core`,
    `utils/attitude`).

## 8. Observed, not changed here (in `.claude/TODO.md`)
- The live listener's configured-range estimate ignores the lever arm. It is latent:
  nothing reads it.
- `native_bins`' reconstruction from `y_local` uses no lever arm.
- Station uids repeat across streams, as they did before.
- Live and offline pictures differ in the first 0.75 m of slant range.
- `t_start_s` is the newest row's time.
- ~~`SCIENTIFIC_BACKGROUND.md` §2's `tvg_compensation: 0` is stale~~ — corrected with §9:
  §2 now says the device's TVG is not identifiable from its output.
- The FBR tracker has no along-track median filter; Al-Rawi et al. §III recommends one
  (Perception `REFERENCES.md` R9).
- ~~The 5 s-median speed gives per-image along-track scale errors of up to ±30 %~~ —
  the window is 2 s since §9 (Perception decision 6.3).

## 9. After the Perception Phase 4 decisions (2026-10-16)
The owner decided Perception's `PREPROCESSING_STUDY.md` §6.1–§6.7. Only **6.3** changes this
module; the others live in Perception and leave the pictures, the PNG and the raw dB as
they are (6.1: SSS keeps forming the pictures).
- **6.3 — along-track speed window 5 s → 2 s** (`config/default.yaml`
  `along_track.window_s`, `config/settings.py` `AlongTrackConfig`). The one estimator
  (`core/along_track.py`) is now built in one place, `AlongTrackEstimator.from_config`,
  by the waterfall service and the seabed imager, so the screen and the pictures cannot
  use different windows. This changes the along-track scale of square-row pictures, by
  design; the georeferencing formula is unchanged.
  - Measured (Perception STUDY §5 M2 / M2b): per-picture scale p5–p95 0.88–1.10 →
    0.94–1.07 (real), 0.86–1.14 → 0.97–1.07 (sim); no added row-to-row jitter.
  - Tests: `test_along_track.py` +2 (the shipped value is 2 s and is read from one
    place; a speed step settles in 1.9 s instead of 3.35 s).
  - The golden (`capture_perception_golden.py` / `test_perception_golden.py`) pins the
    5 s it was captured with, so it keeps proving the pre-change image formation.
  - **Not validated:** 1 s. It measured "not worse", but the real logs' only reference
    is the autopilot's EKF pose.
- **TVG** (finding, docs only): `docs/SCIENTIFIC_BACKGROUND.md` §2 now states that the
  device's TVG is not identifiable from its output. The GCS's "pre-TVG" reading and the
  simulator's `tvg_compensation: 0.30` fit the same falloff (only the product
  `spreading · (1 − tvg)` is constrained). Open item in `.claude/TODO.md`.
- **Load-bearing for Perception from now on:** its default detector input is a fixed
  mapping of the 32FC1 dB and the per-row `gain_port_db` / `gain_stbd_db`
  (`fixed_gray_v1`). A change to those values, to the slant pitch or to the live 0.75 m
  nadir blank changes what its detector is trained and run on; the display model does
  not reach it.
