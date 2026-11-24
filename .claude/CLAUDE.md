# CLAUDE.md — BlueBoat-SSS

Side-scan sonar (SSS) acquisition, processing and ground-control for a BlueRobotics
BlueBoat USV. Two halves live in this submodule:

- **`blueboat_sss`** — ROS 2 package (`ament_cmake`): the Omniscan driver
  (`src/sss_node.py`, which reaches the two units at 192.168.2.92 / .93, TCP 51200, over
  the BlueBoat Base Station WiFi) and the processing/logging node
  (`src/sss_processor_node.py`).
- **BlueBoat GCS** — a standalone PySide6 + rclpy operator application (package
  `blueboat_gcs`, entry point `blueboat_gcs.main`).
  It has **no ROS build dependency** and runs fully without ROS via `--sim`.

Both halves run on the operator laptop (`~/ros2_ws`), like every node of the project; the
boat carries only its hardware and its own firmware. In this document **node-side** means
the `blueboat_sss` ROS nodes and their helpers, as opposed to the GCS — never a place.

Research context: this supports a master's thesis on autonomous aspect-aware SSS
survey. Some rules below exist to protect the scientific argument, not just the code.

---

## Verification status of this document

Every claim below that can be checked against the files in this repository has been.
Facts needing a colcon build or the boat are either omitted — they live in `TODO.md`
with the reason — or carried with an explicit **UNVERIFIABLE HERE** marker where the
claim is load-bearing enough that dropping it would lose more than flagging it.

| Marker | Meaning |
|---|---|
| **VERIFIED** | Confirmed by reading the file named, in this tree. |
| **MEASURED** | Derived from direct measurement of a real recording (see `docs/SONARVIEW_SVLOG_ANALYSIS.md`). |
| **UNVERIFIABLE HERE** | Cannot be checked from this repository alone — it needs the field `.svlog` corpus, a sourced `blueboat_interfaces`, or the boat. |

Where this document and the repository ever disagree, the repository wins: prefer
reading a file to trusting a description of it here.

**What is UNVERIFIABLE HERE, and why.** Everything marked MEASURED rests on the field
`.svlog` corpus, which is not in this repository and not on this machine: the suite
looks for it at the hard-coded external mount
`/media/kyyrron/OS/Users/killi/Desktop/Research Kyutech/BlueBoat/allSvlogData`
(`tests/test_processor_assembly.py`), and every test that would reproduce a MEASURED
number skips when it is absent. The corpus-derived numbers below and in
`blueboat_sss/blueboat_gcs/CLAUDE.md` — the *Measured
acquisition settings* table, the 0 / −1 / +60 counter offsets, the boot-skew values,
the wrong-`src` percentages, the mosaic cell sizes, the 397.8 s session gap — are
therefore carried forward from when the corpus was mounted and were **not**
re-derived. The same applies to the two device addresses `192.168.2.92` /
`192.168.2.93` and the `sss_processor` ↔ hardware behaviour generally: the source is
verified, the boat is not reachable from here.

---

## Repository layout

Two halves: the ROS 2 package root `blueboat_sss/` (`ament_cmake`: `src/`, `launch/`,
`tests/`, `custom_scripts/`) and the GCS application package `blueboat_sss/blueboat_gcs/`
inside it. `.githooks/` and `requirements-dev.txt` are gitignored.

**VERIFIED.** `blueboat_interfaces` is **not** in this submodule — it is an external
package supplied by BlueBoat-Control, which is **frozen at tag `v1.0`** (2026-09-14) together
with BlueBoat-MCS: neither is edited from this module's sessions without an explicit user
order, so a `.msg` or launch change this module needs is a post-v1.0 request, not a patch
(root `CLAUDE.md` §3; `sim_review.md` Part 3.2). The node-side scripts are installed by
`CMakeLists.txt` as `PROGRAMS` into `lib/blueboat_sss/`, flat, which is why
`sss_processor_node.py` does `sys.path.insert(0, dirname(__file__))` and imports
`svlog_helper` / `sss_helper` / `math_helper` without a package prefix. The
`blueboat_sss/blueboat_sss/` Python package is empty; no GCS code is installed by
colcon.

---

Simulated results follow `../SIM_VALIDITY_CONDITIONS.md` (superproject root).

**Simulated sessions (simulator frozen 2026-11-18).**
- The processor's first-bottom-return lock on simulated pings is valid at the thesis setting, 1200 bins /
  44.3 µs (within 0.3 m to 11 m). At 600 bins or long pulses it locks late on simulated flat sand (+0.4 m at
  4.5 m, +2.5 m at 7 m of altitude). Never record results at 600 bins.
- An offline GCS rebuild of a recorded simulated session can run behind the live ping time (the waterfall's
  `row_t_s` was 5.11 s behind on the C2 mission). Align on the seabed images' `t_s` or the ground truth's
  `t_sim`. An offline replay does not reproduce the live chain object for object on barely visible objects.
- Simulated pictures are brighter and more uniform than real (+1 to +2 dB at 8–20 m ground range, no
  ping-to-ping level lines). Never quote a simulated level as a field level.

---

**Detail loads by itself:** each `.claude/rules/*.md` is a path-scoped rule that imports one `.claude/` topic
file when you read or edit a matching file; the tables below name the same files for reading by hand.

## ROS 2 interface (full: `.claude/interface.md`, `.claude/processor.md`)

Nearly every real bug in this project has been an interface or framing mismatch: read the full files before
touching a topic, QoS, frame or parameter.

- **`sss_node.py`**, node `side_scan_sonar`: two Omniscan 450 SS over TCP (`192.168.2.92` / `.93`, port
  51200), one `OmniscanLink` per device (no `brping` at runtime). Publishes `~/{port,starboard}/profile`
  (`blueboat_interfaces/OmniscanProfile`) and `~/{port,starboard}/raw` (every device frame, verbatim), all
  `BEST_EFFORT` / `KEEP_LAST` / 10; subscribes `~/ping/enable` (pinging **off at start-up**). Parameters and
  defaults (equal in the simulator): `range_start_mm` 0, `range_length_mm` 20000, `msec_per_ping` 50,
  `gain_index` −1, `num_results` 1200, `pulse_len_percent` 0.00125.
- **`sss_processor_node.py`**, node `sss_processor`: subscribes both `raw` and `profile` topics,
  `/blueboat/odom` and `/sss_processor/log/enable`; publishes `/sss_processor/processed`
  (`blueboat_interfaces/ProcessedSSSPing`); writes the `.svlog` with SonarView's mavlink rates. Rows keyed by
  normalised `ping_number`, one-sided rows emitted, pose interpolated and a hard gate, altitude never gates.
  Fore/aft arm `TRANSDUCER_X_OFFSET_M = -0.345` (adopted from the simulator, not measured: CR-OWN-1),
  published per ping as `transducer_x_offset`.
- **BlueBoat GCS**, rclpy node `blueboat_gcs`: subscribes `/sss_processor/processed` and the two `profile`
  topics (`BEST_EFFORT`, depth 200), never `raw`; publishes `/side_scan_sonar/ping/enable`,
  `/sss_processor/log/enable`, `/sss_ai/seabed_analysis`, and every seabed image for Perception
  (`/sss_ai/seabed_image`, `…/png`, `…/metadata`); draws only `/sss_ai/detections`. Detail:
  `blueboat_sss/blueboat_gcs/CLAUDE.md`.

## Non-negotiable constraints (full text: `.claude/constraints.md`)

1. Side identity comes from the packet, never from the topic or `src` tag (superproject CM-5).
2. Never drop a ping; one-sided rows are emitted (CM-6).
3. `~/raw` must keep being published (the `.svlog` is written from it).
4. Do not set the GCS sonar subscriber to `RELIABLE` on its own (CM-4).
5. Set sonar range from water depth (~4× the deepest expected), not from the area to cover.
6. Do not modify recorded `.svlog` files: primary field data (CM-7).
7. `blueboat_gcs/tools/svlog_to_rosbag.py` and `tools/svlog_helper.py` are byte-identical copies of the
   node-side files: update only by re-copying, never hand-edit.
8. The processor executable stays named `sss_processor_node` (and `sss_node.py` keeps its name): the GCS
   finds them with `pgrep -f`.
9. Data leaves the GCS only through recording sessions.
10. No browser-style storage in artifacts, and no ROS types past the signal bus: everything downstream of
    `ros/` consumes plain dataclasses, so the GUI runs without ROS.
11. The headline detector is trained on simulated data (CM-11, owner 2026-10-20; the old "real beach imagery
    only" rule is superseded).
12–16. AI images are waterfall-domain and boat-relative; the baseline is two-pass orthogonal; AI iteration is
    offline; the belief layer is the only thing the replanner reads; headline results are model-conditional
    (CM-10, CM-12, CM-14, CM-9, CM-13).

## Measured settings, data, commands

- Measured acquisition settings of the real logs and the `.svlog` format: `.claude/acquisition-data.md`
  (analysis `blueboat_sss/blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md`).
- Commands (GCS `python -m blueboat_gcs.main [--sim]` from `BlueBoat-SSS/blueboat_sss/`; `.svlog` forensics;
  `ros2 launch blueboat_sss SSS_processing_launch.py [with_acquisition:=True]`; the test suite and its
  ROS-shell pitfalls): `.claude/commands.md`, and the superproject skill `module-gates`.
- Which tool answers which `.svlog` question: superproject skill `svlog`; forensics: skill `svlog-forensics`.
- Open items: `.claude/TODO.md`; cross-repository: superproject `docs/OPEN_ITEMS.md`.

Superproject rules hold here: never `git commit`, `git push` or `git tag`; `BlueBoat-Control` (which owns
`blueboat_interfaces`) and `BlueBoat-MCS` are changed by additions only, on an explicit user order.
