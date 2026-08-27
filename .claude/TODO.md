# TODO — BlueBoat-SSS

Open questions, unverified assumptions, and work not yet done. Nothing here is
settled; nothing here is restated as fact in CLAUDE.md.

Items checked against the repository carry the file and line where the condition was
confirmed. Items that cannot be checked from a Windows laptop are marked with why.

---

## Robot-side work — none of this is done yet

Every item below is in `blueboat_sss` and is required before live/recorded data is
correct at the source. All four were re-confirmed present in the current code.

- [ ] **`sss_processor_node._write_raw_with_src_tag` tags by topic, not by packet.**
      Confirmed at `src/sss_processor_node.py:534`, fed by `_on_port_raw` /
      `_on_starboard_raw` at `:290` / `:293`, which pass a constant `DEVICE_ID_PORT` /
      `DEVICE_ID_STBD`. This is where wrong `src` values get burned permanently into
      `.svlog`, which is why *SonarView* also renders our files with banded artifacts.
      Patch:
```python
      _CHANNEL_BYTE = 34   # 8-byte frame header + payload offset 26

      @staticmethod
      def _src_from_packet(raw: bytes, fallback: int) -> int:
          if (len(raw) > _CHANNEL_BYTE
                  and int.from_bytes(raw[4:6], "little") == OS_MONO_PROFILE_ID):
              ch = raw[_CHANNEL_BYTE]
              if ch in (0, 1):
                  return DEVICE_ID_PORT if ch == 0 else DEVICE_ID_STBD
          return fallback
```
      then tag with `self._src_from_packet(raw, fallback_src)`. Full derivation in
      `blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md` §9.1.
- [ ] **Live projection uses the subscription, not the message.** Confirmed: the
      `side_sign` arguments to `project_side` are hard-coded `+1.0` / `-1.0` at
      `src/sss_processor_node.py:638` / `:644`, and `msg.channel_number` is never read
      by the processor. Derive the sign from `msg.channel_number` (fallback: sign of
      `msg.transducer_heading_deg`). Until this lands, live mosaics keep the mirror;
      replayed `.svlog` in the GCS is already correct.
- [ ] **Assemble by `ping_number` instead of pairing within 50 ms.** Confirmed: the
      two-pointer matcher `_drain_matches` at `src/sss_processor_node.py:555` pairs on
      `header.stamp` within `TIME_MATCH_TOLERANCE_NS = 50_000_000` (`:123`) and
      `popleft()`s the unmatched side without emitting it. Also confirmed: the
      bootstrap gate at `:617` (`if altitude is None: ... return`) withholds every ping
      until the FBR tracker locks. Publish one-sided pings rather than dropping them,
      and stop withholding pings while the bottom tracker bootstraps — mirroring what
      `blueboat_gcs/core/svlog.py` already does.
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
      parser state across instances, was never established. The retag patch masks the
      symptom; this is the actual defect. **NOT VERIFIABLE ON THIS MACHINE (Windows,
      no ROS2/colcon; needs the two Omniscan units).**
- [ ] **Fix `/blueboat/odom`** so it is non-zero and stamped on the same clock as the
      sonar profiles. The GCS pose-alignment fallbacks are mitigations, not a fix. Note
      the robot-side consequence is severe: the processor treats pose as a hard gate
      and silently drops every ping pair that has no odom
      (`src/sss_processor_node.py:582` and `:650`). Owned by BlueBoat-Control.
      **NOT VERIFIABLE ON THIS MACHINE (Windows, no ROS2/colcon).**

---

## Code/doc drift found in the tree

- [ ] **`SSS_processing_launch.py` still defaults to 30 m.** `range_length_mm` is
      `30000` at `launch/SSS_processing_launch.py:48` (and `:25` of
      `SSS_simple_launch.py`, and `src/sss_node.py:344`), but
      `blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md:111` states the change to
      `20000` as "Fix (implemented)". Either apply the change or correct the doc — as
      it stands the recorded recommendation and the code disagree, and launching with
      defaults reproduces the setting that broke bottom detection.
- [ ] **`blueboat_sss/README.md` is stale.** It documents `/sss_node/ping/enable` and
      `/sss_node/log/enable` (the node is `side_scan_sonar`; log enable belongs to the
      processor), a `log_directory` parameter (does not exist), mcap `ros2 bag`
      recording by the processor (does not exist), `match_tolerance_ms` and
      `transducer_x_m` / `transducer_y_offset_m` / `transducer_z_m` parameters (none
      exist — the geometry is four module-level constants), and a `~/ping` output
      topic (it is `~/processed`). Rewrite or delete.
- [ ] **`SSS_launch.py` does not exist** but is still referenced by `terminals.txt:51`
      and `:81`, by the `SSS_simple_launch.py` docstring (`:4`–`:6`), by
      `SSS_processing_launch.py:24`, and by `blueboat_gcs/docs/HANDOVER.md:272`. The
      real files are `SSS_simple_launch.py` and `SSS_processing_launch.py`. Also
      `SSS_simple_launch.py:12` still describes `processed_sss_listener.py`, which was
      deleted from the tree (its node block at `:44`–`:52` is already commented out).
- [ ] **Session `.svlog` destination is ambiguous.**
      `RecordingManager._adopt_svlogs` moves adopted files to the session root —
      `dest = session` at `core/recording_session.py:160`, with the `svlog/`
      subdirectory line commented out — while its own module docstring (`:24`),
      `docs/HANDOVER.md:43` and `docs/ARCHITECTURE.md` all describe a `svlog/`
      subfolder. Pick one and make the three docs agree with the code.
- [ ] **Recording can fail silently if the log directory is missing.** Both `mkdir`
      calls on the write path are commented out —
      `src/_custom_libraries/svlog_helper.py:284` in `SvlogWriter.start` and
      `src/sss_processor_node.py:197` for `log_root`. If
      `../../../../data/SSS_data` does not exist relative to the launch cwd, the first
      `open(..., "ab")` raises `OSError`, which `SvlogWriter.write` swallows by setting
      `_active = False` (`:306`) — Record appears ON and nothing is written. Separately,
      `_roll_unlocked` calls `path.unlink()` on a same-named existing file (`:315`),
      which sits badly with NC #6 (recorded `.svlog` are primary field data).
- [ ] **`sss_helper.MosaicGrid` is dead code.** Its only consumer was the deleted
      `processed_sss_listener.py`; the GCS has its own `mapping/mosaic.py`. It is also
      the sole reason `sss_helper.py` imports `matplotlib`. Delete it, or state why it
      stays.

---

## Unresolved technical questions

- [ ] **SonarView still renders better than us at identical range settings.** Not
      solved. Working hypothesis is that the residual gap was mostly the old fixed
      0.25 m mosaic grid (now adaptive) plus the 19.8 % mis-tagging — but this is an
      assumption. Needs a clean paired comparison after the retag patch, on one log,
      same range, same colormap. **Blocked on field data (see below).**
- [ ] **Replay timeline is synthetic.** Confirmed still the case:
      `blueboat_gcs/core/svlog.py:56` defines `NS_PER_TICK = 20_000_000` and
      `load_svlog` calls `tick(None)` for every profile packet (`:384`), advancing a
      flat 20 ms per packet; the decoded `timestamp_ms` is carried in the dict (`:113`)
      but never used for timing. Mavlink packets do share a stamp when `time_boot_ms`
      matches (`:341`), so the clock is burst-aware but not real. A 612 s mission
      reports as ~252 s, so replay x1 is not real time. Fix: use real timestamps, keep
      the tick as fallback.
- [ ] **Segmented `.svlog` files.** Confirmed unhandled: `load_svlog` dispatches only
      on `OS_MONO_PROFILE_ID` and `MAVLINK_WRAPPER_ID` (`core/svlog.py:383`, `:400`);
      packet id 10 (session header) is decoded by `walk_packets` but never inspected,
      so a second session header is invisible. Cerulean demo file contains two session
      headers and a 398 s gap; the gap is neither surfaced nor handled.
- [ ] **Packet id 2194** (16 bytes at ~0.31 Hz) is only partly identified:
      `docs/SONARVIEW_SVLOG_ANALYSIS.md:116` calls it an Omniscan status packet
      associated with the `doppler_enable` device option, carrying no imagery. Nothing
      in the tree parses it. Harmless and ignored; identify it properly if Doppler/DVL
      is ever used.
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

- [ ] Confirm `colcon build --packages-select blueboat_sss` succeeds and that
      `CMakeLists.txt` installs everything the nodes import at runtime (the flat-import
      trick in `sss_processor_node.py` depends on all five scripts landing in
      `lib/blueboat_sss/`).
- [ ] Confirm every `ros2 launch` line in CLAUDE.md actually runs, and that
      `SSS_processing_launch.py` resolves `simple_launch`.
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

Justified by workflows that have demonstrably recurred:

- [ ] **`.svlog` forensic analysis** — packet census, per-channel parameter tables,
      PRI and ping-number gap statistics, `src`-vs-`channel_number` consistency, FBR
      altitude distribution, before/after waterfall rendering. This exact analysis was
      performed from scratch on three separate files. A **Skill** with a reusable
      script would pay for itself immediately, and every future field log needs it.
- [ ] **Headless GUI regression harness** — every single update ended with an ad-hoc
      `QT_QPA_PLATFORM=offscreen` script driving the app through `QTimer` phases with
      dialogs monkeypatched. This has caught real bugs repeatedly. Promote it to a
      committed `pytest` suite (there is currently no automated test suite at all),
      then a **hook** running it before commits. The bar to clear is low and already
      demonstrated: the compile sweep runs clean on all 51 GCS files, and 47 of them
      import with no ROS installed, so a laptop-only suite is viable today.

Not yet justified: anything around deployment, packaging, or CI — no recurring
evidence.
