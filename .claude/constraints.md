# BlueBoat-SSS — non-negotiable constraints (full text)

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## NON-NEGOTIABLE constraints

Violating any of these breaks another module, the hardware integration, or the thesis.

### Sonar data handling

1. **Side identity comes from the packet, never from the topic or `src` tag.** Use
   `channel_number` (0 = port, 1 = starboard), falling back to the sign of
   `transducer_heading_deg` **wrapped to (−180, 180]** (`sss_helper.side_from_fields`,
   the one rule; ported verbatim as `core/svlog.resolve_side` and
   `core/live_native.resolve_side_of_profile` — change all three together). The
   device reports the mounting angle (−90 port / +90 starboard on 100 % of field
   packets); the wrap is what keeps a writer that encodes an absolute bearing (the
   simulator until 2026-09-15) from mirroring half its headings, which is exactly what
   SonarView did to those logs (`project_review.md` §3.6). A bearing of exactly 0 or a
   non-finite one resolves nothing: the processor then routes by the **arrival topic on
   both its paths** (the message path used to call it port silently while the raw path
   took the topic, so the `.svlog` `src` byte and the row could disagree), counts it
   (`_unresolved_side`) and warns once; the GCS lands it on port and counts it
   (`SvlogMission.unresolved_side`, forensics `src.unresolved`). In the raw framed
   packet, `channel_number` is **byte 34** (8-byte frame header + payload offset 26),
   and `transducer_heading_deg` is **byte 52** (float32 LE) — **MEASURED**, zero
   mismatches. Real recordings carried a wrong `src` on up to 29.5 % of packets, which
   is what produced mirrored and swapped mosaics; two logs carry `channel_number = 255`
   on every packet, where only the heading fallback identifies the side. Obeyed by the
   GCS replay path (`core/svlog.py`) and, on both its paths, by `sss_processor_node.py`
   — `_src_from_packet` for the `src` byte written into `.svlog`, `_channel_of` for
   the row's side (`_side_geometry` takes that resolved side, never re-derives it).
   The device id the raw callbacks pass is only a fallback.

2. **Never drop a ping.** Assemble rows by `ping_number`; emit one-sided rows rather
   than discarding them; never withhold a ping because the bottom tracker has not
   locked. Arrival-time pairing plus a strict depth-lock gate was silently destroying
   ~10 % of rows and the start of every mission. Obeyed by both paths — the GCS replay
   path and `sss_processor_node`. A missing `/blueboat/odom` pose is the one remaining
   drop, and it is BlueBoat-Control's (see `TODO.md`).
   **Two assembly rules added 2026-09-03**, after a whole simulated session ran at
   100 % one-sided rows (every ping published as two half-rows — the GCS waterfall's
   "one black row in two per side"): (a) a key falling `COUNTER_RESTART_PINGS` (128)
   behind the newest key seen is a **device counter restart** (power cycle, or the
   simulator relaunched under a running processor), never a late arrival — the node
   drains what is pending, resets the high-water mark and re-learns the offset through
   the pre-roll, instead of evicting every new group on its next arrival for the rest
   of the run; (b) the flush timer judges "the stream stopped" on the **wall clock
   against the last arrival only** — never wall time against a message stamp, which is
   sim time under Gazebo (and device time on replay) and made every group look 1.8e9 s
   stale on every tick. **2026-11-07:** in a simulation graph the "wall clock" is the
   node clock: `SSS_processing_launch.py` uses `SimpleLauncher(use_sim_time='auto')`
   (true when a `/clock` publisher exists at launch, never in a real-robot graph), and
   `_arrival_ns()` then reads `/clock` for the arrival stamps, the stream-stopped check
   and the rate watchdog — arrival against arrival still, never against a stamp — so
   a simulation slowed below real time (the ray-cast sonar) does not tear rows.
   **2026-11-09:** the GCS starts the processor before Gazebo, so `auto` resolved to wall
   time and the first ray-cast harbour run reported wall-paced rates ("2.0 pings/s" for 20 Hz
   of sim time). A processor launched without sim time now switches to it when a `/clock`
   publisher appears (`_follow_clock`, a 1 Hz steady-clock timer; the arrival high-water mark
   and the rate window restart) and back to wall time after `CLOCK_FOLLOW_LOST_S` (5 s) without
   one; a real-robot graph has no `/clock`, so nothing changes there
   (`test_a_clock_appearing_after_start_switches_to_sim_time_and_back`). Both the node and the GCS `SonarListener` now warn once when
   more than 25 % of the last 100 rows are one-sided.

   **The key is the offset-normalised counter, not the raw one.** The two units number
   their pings independently; grouping on the raw counter merges halves acquired up to
   3.0 s apart on the +60 logs in the corpus. Any new consumer or reimplementation must
   align the counters first (`sss_helper.PingCounterOffset`,
   `blueboat_gcs.core.svlog.estimate_counter_offset`) — and must vote against the
   temporally *nearest* opposite ping, since voting against the most recent one is
   bimodal between the true offset and offset ±1.

3. **`~/raw` must keep being published.** The processor writes `.svlog` from it;
   disabling it produces empty logs.

3a. **One receive buffer per device; never share parser or socket state between the
   two Omniscan links** (2026-10-27). `brping`'s class-level `_input_buffer` made the
   port and starboard threads pop one deque: frames parsed by the wrong worker, lost to
   bad checksums, and a forced reconnect in the field (`pop from an empty deque`, the
   15:07 run). `OmniscanLink` owns everything per instance, and
   `tests/test_omniscan_link.py` pins it (two threads, two real sockets, zero loss, no
   crossing). Do not reintroduce `brping` at runtime.

3b. **Acquire as SonarView does** (owner, 2026-10-27): its acquisition parameters
   (`msec_per_ping` 50, auto gain, 1200 bins, pulse 0.00125; only the range may vary),
   its mavlink message rates, and its `.svlog` content — identically in this module and
   in BlueBoat-SSS-Sim (CM-2). SonarView's recordings are the reference: 0 % ping loss at
   20 pings/s against our 4–61 % free-running at 34.5 pings/s
   (`blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md` §11).

4. **Do not set the GCS sonar subscriber to `RELIABLE` on its own.** The publisher is
   `BEST_EFFORT`; a `RELIABLE` subscriber is QoS-incompatible and receives *nothing*.
   Change both ends together or not at all.

5. **Set sonar range from water depth (~4x the deepest expected), not from the area to
   cover.** An 80 m range in shallow water pushed the bottom return to sample 49/600,
   which breaks bottom detection outright.

6. **Do not modify uploaded/recorded `.svlog` files.** They are primary field data.
   The writer enforces this on its own output too: a name collision — two recordings in
   the same second, or a 500 MB roll inside one — takes a suffixed name
   (`<stamp>-001.svlog`) rather than deleting the file already there.

### GCS structure

7. **`blueboat_gcs/tools/svlog_to_rosbag.py` and `blueboat_gcs/tools/svlog_helper.py`
   are byte-identical copies** of the node-side files
   `custom_scripts/svlog_to_rosbag.py` and `src/_custom_libraries/svlog_helper.py`,
   which live in this same repository. SHA-256 identical, **VERIFIED** (26 037 B and
   14 665 B respectively; asserted by `tests/test_sweeps.py`). Update only by
   re-copying; never hand-edit. Both directions matter: the replay window puts `tools/` on `sys.path` and imports `svlog_to_rosbag`
   as a library for "Save as rosbag" (`gui/replay_window.py:464`), so its `Converter`
   API is load-bearing, not just its `main()`.

8. **The processor executable must stay named `sss_processor_node`** and keep
   `output='screen'` in the launch file — the GCS matches that string with `pgrep -f`
   to sweep orphan processes on STOP (`pipeline.leftover_process_patterns`), and pumps
   the launch tree stdout into the embedded console. **VERIFIED** at both ends. The
   driver's script name `sss_node.py` is load-bearing the same way
   (`pipeline.owned_process_patterns`, swept only when this application launched it —
   `tests/test_sonar_source.py`).

9. **Data leaves the GCS only through recording sessions.** If no session was active,
   STOP and app-close export nothing. **VERIFIED** in `gui/main_window.py`.

10. **No `localStorage`/`sessionStorage`-style browser storage in artifacts**, and no
    ROS types past the signal bus — everything downstream of `ros/` consumes plain
    dataclasses so the GUI runs without ROS (rcl_interfaces for the runtime range
    change is imported inside `ros/ros_manager.py` only, and its result crosses the
    bus as `(bool, str)`). **VERIFIED**: `ros/` is the only place `rclpy` is
    imported, and 66 of the 70 swept GCS modules import cleanly with no ROS at all
    (the four that do not are exactly the `ros/` listeners, which `main.py` imports
    lazily and only outside `--sim`; `tests/test_sweeps.py`, 2026-11-07: +`core/stream_clock.py`, 2026-11-11: +`core/sim_run_info.py`). The live
    listener's logic that can be tested without ROS lives in `core/live_native.py` for
    exactly that reason. The sweep count excludes `tools/`, whose three files are
    checked by hash instead (NC #7). `core/perception_core.py` imports
    BlueBoat-Perception's (ROS-free) core lazily, for the replay window's Run AI only.

### Scientific validity (from the thesis plan)

11. **(2026-11-18) Superseded by the owner's decision of 2026-10-20 (superproject CM-11):** the headline
    detector is a YOLO model in BlueBoat-Perception trained on *simulated* datasets and calibrated on
    held-out simulated worlds; a labelled real dataset blocks nothing. The consequence stays stated: a
    simulation-trained detector inherits its aspect sensitivity from the simulator's model, so
    detector-dependent results stay model-conditional. (The old rule read: "The headline detector trains on
    real beach imagery only.")

12. **AI images are waterfall-domain and boat-relative**, never world-frame mosaic
    crops.

13. **The policy baseline is two-pass orthogonal**, not single-pass, and must be tuned
    seriously.

14. **AI iteration happens offline against recorded rosbags, never in the water.**

15. **The belief layer is the only object the replanner reads.** Detector, sensor and
    planner changes interact only through it.

16. **Headline results are stated as model-conditional** ("in a calibrated simulation
    environment…"), never as unqualified empirical findings.

---
