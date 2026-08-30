# TODO — BlueBoat-SSS

Open questions, unverified assumptions, and work not yet done. Nothing here is
settled; nothing here is restated as fact in CLAUDE.md.

Items checked against the repository carry the file and line where the condition was
confirmed. Items that cannot be checked from a Windows laptop are marked with why.

---

## Robot-side work — still open

Every item below is in `blueboat_sss` and is required before live/recorded data is
correct at the source. All were re-confirmed present in the current code.

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
      the robot-side consequence is severe: the processor treats pose as a hard gate
      and drops every ping that has no odom (`src/sss_processor_node.py`, `_emit_group`,
      `_dropped_no_odom`). Since assembly now emits one-sided rows and no longer
      withholds for bootstrap, this is the **only** remaining drop on the robot side.
      Owned by BlueBoat-Control. **Needs the boat: a zero-reading `/blueboat/odom` is a
      robot_interface/hardware condition, not reproducible from the development
      machine.**

---

## Code/doc drift found in the tree

- [ ] **`with_acquisition:=True` can never start the acquisition node.** Confirmed by
      running it: `SSS_processing_launch.py:55` guards the node with
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

- [ ] **SonarView still renders better than us at identical range settings.** Not
      solved. Working hypothesis is that the residual gap was mostly the old fixed
      0.25 m mosaic grid (now adaptive) plus the 19.8 % mis-tagging — but this is an
      assumption. Needs a clean paired comparison after the retag patch, on one log,
      same range, same colormap. **Blocked on field data (see below).**
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

---

## Not verifiable from a Windows laptop

Listed so they are not mistaken for unchecked oversights. Each needs the Linux
development machine with a sourced ROS 2 environment.

- [x] Confirm `colcon build --packages-select blueboat_sss` succeeds and that
      `CMakeLists.txt` installs everything the nodes import at runtime (the flat-import
      trick in `sss_processor_node.py` depends on all five scripts landing in
      `lib/blueboat_sss/`). **Done** on the Linux machine (ROS 2 Jazzy): build clean,
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
