# TODO — BlueBoat-SSS

Open questions, unverified assumptions, and work not yet done. Nothing here is
settled; nothing here is restated as fact in CLAUDE.md.

Items checked against the repository carry the file and line where the condition was
confirmed. Items that need a sourced ROS 2 environment, the corpus or a field session are
marked with why.

---

## Node-side work (`blueboat_sss` ROS nodes) — still open

Every item below is in `blueboat_sss` and is required before live/recorded data is
correct at the source. All were re-confirmed present in the current code.

- [ ] **Live re-check of the 2026-09-03 row-tearing fix.** Offline replay through
  `_accept` and the 31 assembly tests pass; still to confirm on a live run: relaunch
  the sonar node (sim or real) under a running `sss_processor` and check
  `ros2 topic hz /sss_processor/processed` stays at the ping rate (not 2×) and
  `starboard_ping_number` is non-zero on every message after ≤ 24 pings.
- [ ] **Live 10-minute run after the 2026-09-03 GCS throughput fix.** The
  benchmark is offline; confirm on a real 20 Hz two-sided stream that the app's
  CPU and RSS stay flat (`top`), the waterfall follows without lag, the mosaic
  cell size reads ~0.026 m at 15 m / 600 bins, no "display N pings behind" status
  appears at rest, and Contrast-slider drags do not freeze the window.
- [ ] **Investigate the ~8 % ping loss.** Root cause unconfirmed. Candidates, in order:
      (a) `BEST_EFFORT` depth 10 on both hops — try `RELIABLE` depth 50 on *both* ends
      together; (b) confirmed still present — `msg.data = list(raw)` at
      `src/sss_node.py:328` and `list(data.pwr_results)` at `:320` build ~3700 Python
      objects per ping-pair inside the thread that must return to `wait_message` — try
      `array.array('B', …)` / `array.array('H', …)`; (c) enlarge `SO_RCVBUF`.
      *Root-causing needs a running system:* **NOT VERIFIABLE ON THIS MACHINE
      (Windows, no ROS2/colcon).**
- [ ] **Find out why the port device emits `channel_number=1` at all.** Whether both
      `OmniscanWorker`s can reach the same device/multiplexer, or `brping` shares
      parser state across instances, was never established. The processor now routes
      both the `.svlog` tag and the projection off the packet, so the symptom is
      masked at the consumer; this is still the actual defect. Two field logs
      (`No_sonarVNotOK_usOK_SimpleCurve`, `reflectionEvidenceFullharbour`, ~30 000
      profiles) are a second variant: `channel_number` is `255` on every packet and
      only `transducer_heading_deg` identifies the side. **BLOCKED: needs the two
      Omniscan units.**
- [ ] **Fix `/blueboat/odom`** so it is non-zero and stamped on the same clock as the
      sonar profiles. The GCS pose-alignment fallbacks are mitigations, not a fix. Note
      the processor-side consequence is severe: the processor treats pose as a hard gate
      and drops every ping that has no odom (`src/sss_processor_node.py`, `_emit_group`,
      `_dropped_no_odom`). Since assembly now emits one-sided rows and no longer
      withholds for bootstrap, this is the **only** remaining drop in the processor.
      Owned by BlueBoat-Control. **Needs the boat: a zero-reading `/blueboat/odom` is a
      robot_interface/hardware condition, not reproducible without a real-robot
      run.**

---

## Closed on 2026-09-19 (live vs replay: "replay is way better")

The operator reported the live waterfall and mosaic looking markedly worse
than the same data on replay. Root cause and fixes, all O(1) per ping:

- [x] **The live display model froze after `warmup_rows` and nothing unfroze
  it** (`core/display_model.py`). Everything after the first ~15 s was drawn
  through a curve fitted on the launch area — one altitude, one range
  setting, one bottom type — while replay fitted the whole file
  (`gui/replay_window.py` calls `DisplayModel.fit`). Worse, `observe_row`
  returns immediately when frozen, so a session whose bottom tracker had not
  locked during those first 300 rows froze an **empty** histogram and spent
  the rest of its life on the raw 2/98-percentile fallback — the
  near-white / far-black picture. It now **re-publishes** instead of
  freezing: the histogram was always accumulated per row, and the solve
  behind `version` is cached (`_snapshot`) and runs once per publish, once
  every `warmup_rows` rows. Gated by `test_warmup_publishes_and_keeps_-
  observing`, `test_a_late_regime_change_is_learned_live` and
  `test_streaming_matches_a_whole_pass_fit` — the last is the parity every
  existing test missed, because they all hold the model *fixed*.
- [x] **The live mosaic kept the 55 dB transmit ringing that replay blanks.**
  `depth.blank_nadir` / `nadir_blank_m` were read only by `core/svlog.py`,
  and the robot's processor passes no blank (`sss_helper.py`), so the
  ringing landed at ground range ≈ 0 and painted a saturated stripe down the
  track. `attach_native` takes the blank now, wired from the config through
  `SonarListener`. Gated by `test_live_nadir_blank_matches_replay`.
- [x] **The mosaic's warm-up hold-back waited for a freeze that no longer
  happens**, so it is released on readiness instead (`core/mosaic_service.py`).

## Still open from the same review

- [ ] **The two mosaics are not the same product.** Live is in **ground
  range** (the processor corrects with its tracked altitude and deletes the
  water column); replay defaults to **slant range** (`depth.mode: "off"`,
  the SonarView convention) and keeps it. And `MosaicService._normalised`
  reconstructs `r = hypot(|y|, h)`, which is exact live and *over-estimates*
  on the replay path where `y` is already slant. Pick one geometry for both
  deliberately and fix `_normalised` with it — the operator currently
  prefers the replay one, which is the mis-normalised one.
- [ ] **Fixed metres-per-pixel imagery.** The seabed pictures and the
  waterfall are one column per device range bin, so their across-track scale
  changes with `range_length_mm` (25.0 mm at 15 m, 33.3 at 20 m, 600 bins)
  and two runs at different range settings are not comparable at the same
  zoom. Resample each row onto a fixed ground pitch (`seabed.pixel_pitch_m`,
  0.025 m) with `native` kept as an option. **This changes the seabed-image
  contract** — the root `CLAUDE.md` §4.5 says "verbatim values, one column
  per device range bin" — so the row there moves in the same change. It
  stays CM-10-compliant (waterfall-domain, boat-relative).
- [ ] **Cache misses degrade one side of the live waterfall only.** On a
  miss `core/live_native.py` re-projects from the processor's ground samples:
  the centre gap widens on that side, the pitch becomes a median estimate
  and `analog_gain` is lost, so gain compensation silently switches off for
  that row/side. Tolerated up to 5 % without a word.
- [ ] **Cosmetic, live-only:** the render throttle above `ping_lag_throttle`;
  only two stale tiles re-rendered per pass after a model publish; replay not
  pushing the slider gamma onto `DisplayModel.gamma` (exported pictures only).

## Closed on 2026-09-18 (the simulator review's Part 6, GCS + processor half)

- [x] **`GLOBAL_POSITION_INT` velocities were hard-coded to zero** — so every
      `.svlog` this node writes claimed a stationary boat, and the along-track
      scale of a replayed waterfall is read from exactly those fields. They now
      come from `/mavros/local_position/velocity_local` (ENU → NED, cm/s), which
      the node already subscribed for its `LOCAL_POSITION_NED` twin; no velocity
      still means zeros, which the schema allows.
- [x] **The planned path was cut at 1000 s.** `path_publisher` (BlueBoat-Control,
      frozen at v1.0) asks the path service for a fixed `total_time` window, so the
      1317 s survey of 2026-09-07 drew as its first 1000 s. ~~The station now reads
      the trajectory it deployed itself (the newest `.deployed/*.yaml`, placed
      through the designer file's `geo_anchor`)~~ — **wrong on both counts, replaced
      2026-09-28** (below).

## Planned path (2026-09-28)

- [x] **The GCS drew another mission, 24.5 m off, in World Builder simulations.**
      The 2026-09-18 source read the *newest* `.deployed/*.yaml` once, at the first
      GPS anchor, and then ignored `/set_path` for the session. MCS deletes the
      current mission's deployed file at launch and rewrites it only after its GPS
      fit converges, and the sim anchors first, so "newest" was an older mission
      (`plage_lawnmower` during `plage_bathymetry2`). Even the right file was
      re-projected through the designer's `geo_anchor` although `deploy_mission`
      had already put its points in the run's world frame: a `design_offset`
      shift. Now the listener does what MCS does for its preview: it reads
      `path_generation`'s `trajectory` / `yaml_path` parameters and sizes ONE
      `/path_request` by that file's duration. It asks again only when the file's
      mtime changes, the server restarts or the overlay is cleared, and it draws the
      world-frame answer directly. `/set_path` draws only when no server answers.
      Code: `core/deployed_path.PlannedPathSource`, `ros/path_listener.py`,
      `config.planned_path`. Test: `tests/test_deployed_path.py`. Checked live
      against a real `path_generation` on an isolated domain: it waited for
      deployment, then drew 2636 poses of the 1317.6 s loop, starting at (0, 0).
- [x] **RViz drew the first 600 s (world mode) or 1000 s (empty Gazebo).**
      `path_publisher` builds its window once, at startup, when the deployed file
      does not exist yet. `blueboat_sss_sim`'s `full_mission_launch.py` and
      (post-v1.0, ordered by the owner) `blueboat_control`'s `Sim_launch.py` now take
      the duration from the designer file a `.deployed/` path names first.
- [ ] **Live check in an MCS-launched world simulation**: the GCS status line
      names the running mission, its first pose sits on the spawn point, and RViz
      shows the whole loop. This needs
      `colcon build --packages-select blueboat_sss_sim blueboat_control` first,
      because both launch files are installed copies.
- [x] **The waterfall's "True scale" toggle, and its silent fallback.** Removed:
      the aspect is now taken from the rows on screen (so a mission that changed
      range mid-survey is drawn in the scale of what you are looking at) and the
      state is written in the corner — including "scale unavailable" and "mixed
      scale", which were previously a silent reversion to square pixels.
- [x] **The Contrast slider did not reach the saved AI pictures.** It drove the
      screen, the mosaic and `metadata.json` but not `DisplayModel.gamma`, which is
      what `SeabedImager.to_png8` renders through: the 2026-09-07 session ran at
      γ 0.48 on screen and wrote its PNGs at 0.7. Fixed, and the shipped default
      moved to 0.5. `tests/test_display_startup.py`.

---

## Code/doc drift found in the tree

- [x] **`with_acquisition:=True` can never start the acquisition node.** **Fixed
      2026-09-15:** nested launch-time groups (`sl.group(unless_arg='will_use_rosbag')`
      → `sl.group(if_arg='with_acquisition')`), verified by evaluating the launch
      description under all four flag combinations. Original finding: confirmed by
      running it: `SSS_processing_launch.py:55` guarded the node with
      `if sl.arg('with_acquisition') and not sl.arg('will_use_rosbag')`, but outside an
      opaque function `simple_launch`'s `arg()` returns a `SimpleSubstitution` object,
      which is always truthy — so `not sl.arg('will_use_rosbag')` is always `False` and
      the branch is unreachable regardless of what is passed. `ros2 launch blueboat_sss
      SSS_processing_launch.py with_acquisition:=True` starts `sss_processor` alone.
      The same pattern would break `will_use_rosbag:=True`. Both flags are documented as
      working in `CLAUDE.md`'s Commands section and in the launch file's own docstring.
      Fix with launch-time conditions (`IfCondition` / `sl.group(if_arg=...)`) evaluated
      as substitutions, not with Python truthiness. `SSS_simple_launch.py` is unaffected
      — its equivalent guard at `:30` is commented out.

---

## Unresolved technical questions

- [ ] **Which transfer ships as the default — `power` (γ 0.7) or `db_window`?** Both
      exist since 2026-09-15 (`display.transfer`); the power law stays the default by
      decision until a **paired comparison on real data** exists: the same field
      `.svlog` rendered by SonarView (screenshot, its id-12 view state recorded:
      `gamma 0.2`, `signal_range_min/max`) and by the GCS under each transfer, judged on
      seabed CV, shadow contrast and target visibility (`project_review.md` §3.2/§3.5,
      §5). **BLOCKED — needs a SonarView machine** (none here).

- [ ] **Record one post-patch mission with our stack.** Every local `.svlog` was
      recorded by SonarLink or by the July processor; nothing in the corpus exercises
      the 2026-09-15 changes end to end (gain compensation with the real gain ladder,
      the along-track estimator at 20 Hz, the threaded export at STOP). One harbour
      session at 20 m / 20 Hz with Record ON, then `tests`' corpus items re-checked
      against it. **BLOCKED — needs the boat.**

- [x] **SonarView still renders better than us at identical range settings.**
      Resolved 2026-09-05 for the waterfall, the AI pictures and the mosaic by the one
      display model (`core/display_model.py`; `docs/SCIENTIFIC_BACKGROUND.md`): TL
      removed, per-side seabed curve in `r/h` (histogram mode), power-law transfer, no
      low handle; live path fed the raw profiles (`core/live_native.py`); true-scale
      view; square-pixel pictures. Rendered against SonarView's picture of the same
      2026-09-04 simulation log: black nadir with the thin bottom line, uniform seabed
      near-to-far, black shadows behind the walls, no vertical bands
      (`gamma 0.7`, `hi_pct 95` matched SonarView's median seabed brightness).
- [ ] **Live re-check of the 2026-09-05 raw-profile attach.** On a real 20 Hz two-sided
      stream (processor + GCS on the sim graph or in a real-robot run) confirm the console never
      prints the "rows had no raw profile in time" line at rest, `profile_misses`
      stays near 0 on the listener, the live waterfall shows the water column and the
      thin bottom line exactly like the replay of the recorded `.svlog`, and the
      model freezes after `display.warmup_rows` (every tile re-renders once). Needs a
      running processor — unverifiable from this repository.
- [ ] **Square-pixel seabed pictures are PROVISIONAL (`seabed.row_geometry: square`,
      2026-09-05).** Evaluate on the first real labelled set whether the detector does
      better than with one-row-per-ping tiles; if not, revert with the one config line
      `seabed.row_geometry: ping` (the old contract, kept in code and tests).
- [ ] **Display model on the field corpus.** `gamma` / `hi_pct` / `warmup_rows` were
      calibrated on the simulation log and two local field logs; check the per-side
      curves on the external corpus (80 m logs, low altitudes) and the mosaic's
      normalised planes against SonarView's mosaic at the same range/colormap.
      **Blocked on field data (see below).**
- [ ] **`RINGING_SEARCH_MAX` is a sample count, so its physical meaning moves with
      the range setting.** `src/sss_processor_node.py:114` fixes the ringing search
      horizon at 60 samples and `find_noise_window_start`'s fallback at 30
      (`src/_custom_libraries/sss_helper.py:44`), and `detect_fbr_slant_m` starts its
      bottom search at `nw_start + NOISE_FLOOR_WINDOW` — no earlier than sample ~50.
      That is a minimum detectable altitude of 1.67 m at the 20 m / 600 default, 2.5 m
      at 30 m / 600 and 6.7 m at 80 m / 600, which is why the 80 m sea trial (bottom at
      sample 49) could not detect bottom at all. Express the horizon and the fallback as
      distances converted through the ping's own `length_mm / num_results` instead. This
      changes detection behaviour, so it needs a recording to validate — pair it with
      the range-change `.svlog` under *Needed field data*.
- [ ] **Altitude estimate is unstable on long-range logs** (4.7 m to 45 m on the 80 m
      file, p10 4.67 / p90 15.23). May resolve itself at 20 m range; re-measure rather
      than assume. **Blocked on field data (see below).**

---

## Needed field data

All three are **blocked on a field session** — not attemptable from this machine.

- [ ] A short `.svlog` recorded **after** the range change (20 m) to confirm the
      predicted ping-rate and resolution gains on our own hardware.
- [ ] A `.svlog` recorded **after** the retag patch, to confirm 0 % mis-tagging.
- [ ] A same-day paired recording (ours vs SonarView, identical settings) for the
      quality comparison above.
- [ ] **A recording with the GPS velocities present** (2026-09-18): every corpus
      `.svlog` written by this node so far carries `vx = vy = vz = 0`, so no
      existing file exercises the fix, and a replay's along-track scale still falls
      back to the pose-chord estimator on all of them.
- [ ] **A fixed-gain pass, and a pass at ≥ 3 m altitude over the same ground**
      (from the simulator's review): the first separates the device's gain
      regulation from the level, the second separates shadow fill from TVG. Both
      are cheap to take on any beach day and neither can be substituted offline.

---

## Needs a real-sonar session

- [ ] **First real run with the GCS's Sonar selector on *Real sonars*
      (2026-09-28).** Confirm: the pipeline launch starts `sss_node.py` (console:
      "Sonar: Real — the pipeline also starts sss_node.py"), it connects to both
      units at 192.168.2.92 / .93 through the BlueBoat Base Station WiFi, START's
      enable is sent once the driver subscribes, STOP ends the driver and the sweep
      leaves no `sss_node.py` behind, and a switch to *Simulation* while running
      asks, stops and relaunches the processor alone. Also the first live run of the
      new defaults (1200 bins, pulse 0.00125): the reported `pulse_duration_sec`
      should read 44.3 µs at 20 m, raw frames 2462 B, and the live waterfall should
      match the replay of the recorded `.svlog`. Offline, `tests/test_sonar_source.py`
      covers the command, the guard, ownership and the deferred enable; none of this
      can be run without the boat.

## Needs a sourced ROS 2 environment

Listed so they are not mistaken for unchecked oversights. Each needs a sourced ROS 2
environment (`~/ros2_ws`, `source env.sh`).

- [x] Confirm `colcon build --packages-select blueboat_sss` succeeds and that
      `CMakeLists.txt` installs everything the nodes import at runtime (the flat-import
      trick in `sss_processor_node.py` depends on all five scripts landing in
      `lib/blueboat_sss/`). **Done** (ROS 2 Jazzy): build clean,
      all five scripts present in `install/blueboat_sss/lib/blueboat_sss/`.
- [x] Confirm every `ros2 launch` line in CLAUDE.md actually runs, and that
      `SSS_processing_launch.py` resolves `simple_launch`. **Done**: both launch files
      pass `--show-args` with the expected arguments, and `SSS_processing_launch.py`
      brings `sss_processor` up clean from the installed tree.
- [ ] Confirm the GCS START button end-to-end: launch subprocess, `/rosout` capture,
      ping-enable round trip, and the `pgrep -f sss_processor_node` orphan sweep
      (`pgrep` does not exist on Windows, so `_sweep_leftovers` is untestable here).
- [ ] Confirm `svlog_to_rosbag.py` still converts (needs `rclpy`, `rosbag2_py`,
      `blueboat_interfaces`, `mavros_msgs`, `geographic_msgs`). Note its `main()` has
      no argparse: `INPUT_FILE` and `OUTPUT_BAG` are hard-coded constants at `:89`–`:90`
      that must be edited before each standalone run. The GCS replay window drives the
      same `Converter` class with GUI-supplied paths instead.

---

## Project work not started

Tracked here so it is not mistaken for existing capability. From the thesis plan:
the beach dataset campaign (critical-path bottleneck), the detection module, the
belief layer implementation, the adaptive replanner, and the evaluation framework.
The aspect-response experiment is the scientific keystone and should be run early
enough that a negative result is actionable.

---

## Automation candidates

Not yet justified: anything around deployment, packaging, or CI — no recurring
evidence.
