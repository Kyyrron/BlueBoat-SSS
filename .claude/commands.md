# BlueBoat-SSS — commands

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

## Commands

**GCS** — run with the working directory set to `BlueBoat-SSS/blueboat_sss/`, the parent of
the `blueboat_gcs` package (`build.sh` and `terminals.txt` both do this):

```bash
python -m blueboat_gcs.main --sim     # no ROS needed; full GUI on a laptop
python -m blueboat_gcs.main           # live, needs a sourced ROS 2 env
```

**`.svlog` forensics** — offline (numpy + opencv, no ROS), from the same directory:
`python3 -m blueboat_gcs.analysis.svlog_forensics --out DIR LOG.svlog` (`--compare LOG...`
or a directory). The `--out` is refused inside a tree holding `.svlog` files (NC #6). Also
the `svlog-forensics` skill (`.claude/skills/svlog-forensics/`); which tool answers which
`.svlog` question: the superproject skill `svlog`.

**ROS 2 nodes** — `ros2 launch blueboat_sss SSS_processing_launch.py` (processor only;
`with_acquisition:=True` adds `sss_node.py`, `will_use_rosbag:=True` overrides;
`msec_per_ping:=0` free-runs the device, a diagnostic only;
`transducer_x_offset_m:=0.0` publishes no fore/aft arm, for a recording made before
2026-11-15). `SSS_simple_launch.py` = acquisition
only. `SSS_processing_launch.py` is what the GCS launches; its **Sonar: Real sonars**
selector appends `with_acquisition:=True`. The two flags are launch-time conditions
(nested `sl.group(unless_arg=…)` / `sl.group(if_arg=…)`) — a Python truthiness test on
substitutions made `with_acquisition:=True` a silent no-op before 2026-09-15; check a
launch change with all four flag combinations. Build: `colcon build --packages-select
blueboat_sss` from `~/ros2_ws`.

**Regression suite** — from `blueboat_sss/`:

```bash
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m pytest -p no:cacheprovider
```

- `pytest.ini` already passes `-q`; a second `-q` silences the pass/skip summary.
- Do not `source ~/ros2_ws/install/setup.bash` first: it switches `python3` to the
  workspace venv, which lacks pytest. The system python has pytest, rclpy, PySide6 and
  `blueboat_interfaces`.
- **452 pass / 52 skip on 2026-11-15** with the workspace's `blueboat_interfaces` on
  `PYTHONPATH` / `LD_LIBRARY_PATH`
  (`PYTHONPATH=~/ros2_ws/install/blueboat_interfaces/lib/python3.12/site-packages:$PYTHONPATH
  LD_LIBRARY_PATH=~/ros2_ws/install/blueboat_interfaces/lib:$LD_LIBRARY_PATH`), and
  **398 pass / 110 skip** with the bare command above (394 before the convention-3 twin in
  `test_perception_golden.py`, +4, BlueBoat-Perception R-5, same day), where the node-side files skip
  because `/usr/bin/python3` alone does not find `blueboat_interfaces` (measured the same
  day before the fore/aft work: 416 / 52 and 368 / 100; the 36 new tests are
  `test_fore_aft_arm.py`, 26, and `test_processor_fore_aft_arm.py`, 10, node-side).
  Earlier: 387 pass / 51 skip on 2026-10-27 (356 / 50 before the SonarView-parity work; the new
  skip is the brping oracle, `bluerobotics-ping` being absent from the system python;
  353 / 50 on 2026-10-16; 351 / 50 before the along-track window tests;
  297 / 50 on 2026-09-28; the gate was **red, 298 /
  4 fail / 50**, on 2026-10-16 before any edit: four stale expectations after
  `994a74b`'s constants, repaired as a labelled baseline repair, `TODO.md`); the skips
  are the corpus gates. **The corpus path is hard-coded** in
  `tests/test_processor_assembly.py` (an external mount, see *Verification status*); every
  corpus test skips without it, so a green suite is not by itself evidence that the
  *Measured acquisition settings* still hold.
- What each test file covers: read `tests/` (names are descriptive).

The **node-side half** (`test_processor_assembly.py`, 48 tests) drives the real
`sss_processor_node` for NC #2 — row assembly, one-sided emission, the bounded flush,
the pre-roll, the counter restart re-base, the wall-vs-stream clock separation of the
flush timer, the torn-row warning, the counter-offset estimator against the field
`.svlog` corpus, and the
odom buffer's nearest-stamp tolerance (fresh sample served, far sample refused with
the measured skew recorded, stale samples pruned at lookup) — and since 2026-10-27 the
SonarView parity: pose interpolation (yaw wrap included), the zero-stamp burst clock,
HEARTBEAT/STATUSTEXT shapes, the id-10 + id-12 header, the wrapper tag, the
message-interval request (sent while streaming, re-asserted, renewed on a low pose rate,
skipped without the service), the rate watchdog (window opened by the first logged
envelope), the odom buffer's clock-epoch drop and the once-only zero-stamp warning. It needs a sourced ROS 2
workspace and skips cleanly without one, so the suite stays laptop-runnable. It never
enables logging and opens the corpus read-only (NC #6).

The **driver** is covered by `test_omniscan_link.py` (ROS-free: framer under random
chunking, corruption and false syncs; command bytes = brping's; decode = the GCS
decoder; two links share nothing; two real sockets in two threads lose nothing and
never cross; a SonarView recording re-framed frame for frame when the local pool logs
exist) and `test_sss_node_driver.py` (two real `OmniscanWorker` threads against two
`tests/fake_omniscan.py` TCP devices: every ping on its own side, status frames on
`~/raw` only, 50 ms commanded, silent after stop, declared defaults = SonarView's).

Availability is probed with `importlib.util.find_spec`, **not**
`pytest.importorskip`: a module-level `importorskip` raises `Skipped` during
collection, which on pytest 7.4.4 here aborts the whole session — the GCS tests
silently stop running and the suite still reports success. Use `pytestmark =
pytest.mark.skipif(...)` for any future environment-gated module.

The import sweep **blocks the ROS packages on `sys.meta_path`** rather than relying
on their absence. On the Linux development machine ROS 2 Jazzy is sourced globally,
so an observational sweep would report 53 of 53 and silently stop testing anything.

**Pre-commit hook.** (2026-11-18: no `.githooks/` folder exists in this checkout; it is gitignored and
was never recreated here.) `.githooks/pre-commit` runs the suite and blocks the commit on
failure. Install it once per clone with `./.githooks/install.sh` — this repository is a
git submodule, so its real hooks directory sits outside the worktree and
`core.hooksPath` is local config a clone does not inherit. `.gitignore` excludes
`.githooks/` and `requirements-dev.txt`, so a fresh clone gets the suite (`tests/`,
`pytest.ini` are tracked) but neither the hook nor the dev requirements.

No linter or type-checker is configured.

**Dependencies.** `requirements-dev.txt` pins `numpy<2` and `opencv-python<5`: ROS 2
Jazzy's Python packages and `scipy` 1.11 are built against the numpy 1.x ABI, and an
unpinned install resolves numpy 2.x. Node side additionally needs `simple_launch`,
`scipy`, `mavros_msgs`, `geographic_msgs`; `rosbag2_py` for `svlog_to_rosbag.py`.
`bluerobotics-ping` (`brping`) is **test-only** since 2026-10-27 (the oracle for the
command bytes; the driver speaks the protocol itself, NC #3a). `pip` in some
environments needs `--break-system-packages`.
