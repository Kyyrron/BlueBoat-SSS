# BlueBoat-SSS — development guide

How to work on the sonar nodes and the Ground Control Station (GCS): the AI coding assistant
setup, the tests, the rules and what to improve next. For using the module, see the
[README](../README.md) and the [user guide](USER_GUIDE.md).

## 1. Install Claude Code

The project is developed with [Claude Code](https://docs.claude.com/en/docs/claude-code/overview),
an AI coding assistant that runs in the terminal.

```bash
curl -fsSL https://claude.ai/install.sh | bash        # native installer
# or, with Node.js 18 or newer:
npm install -g @anthropic-ai/claude-code
```

Then start it and log in with your Anthropic account:

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar               # the whole project: cross-module rules included
claude
# or only this module:
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-SSS && claude
```

Starting from the superproject is better: Claude then also reads the project-wide rules, the
interface contract between modules and the shared skills (test commands, file formats, and the
`svlog` skill that says which tool answers which question about a `.svlog`).

## 2. What Claude reads in this module

| File | Role |
|---|---|
| `.claude/CLAUDE.md` | Loaded at start: the two halves of the module, the interface summary and the non-negotiable rules |
| `.claude/interface.md`, `processor.md` | The driver's and the processor's full ROS interface |
| `.claude/constraints.md` | The full non-negotiable rules, with their reasons |
| `.claude/gcs-design.md`, `gcs-data.md`, `gcs-testing.md` | The GCS's design decisions, the data it writes, its tests and performance notes |
| `.claude/acquisition-data.md`, `commands.md` | Measured sonar settings; every command and the test pitfalls |
| `.claude/rules/*.md` | Load the right topic file automatically when Claude reads or edits a matching file |
| `.claude/skills/svlog-forensics/` | A skill that characterises a `.svlog`: packets, sessions, ping gaps, side identity |
| `.claude/TODO.md` | Open work |
| `blueboat_sss/blueboat_gcs/CLAUDE.md` | Notes for the GCS, loaded when Claude works there |

These files are written for Claude, but they are plain text and worth reading yourself before a
change: nearly every past bug in this module was an interface or framing mismatch, and the files
record how each was found.

## 3. What to set up after a fresh clone

- **Test dependencies** (`requirements-dev.txt` is not tracked by git). Install them into the
  system Python, which also runs the tests:
  ```bash
  /usr/bin/python3 -m pip install --user --break-system-packages "numpy<2" "opencv-python<5" "pytest>=7.0" PySide6 PyYAML
  ```
  numpy stays below 2: the ROS 2 Jazzy packages and scipy are built against numpy 1.
- **The optional pre-commit hook** (`.githooks/` is not tracked either). If you want the test suite to
  run before every commit, write a small `.githooks/pre-commit` that runs the test command below, and
  point git at it with `git config core.hooksPath .githooks`.
- **The field `.svlog` corpus** is not in the repository. Tests that reproduce measured numbers
  skip without it.
- No Claude hook is needed in this module.

## 4. Tests

From `blueboat_sss/`, with the **system Python** and **without** sourcing the workspace (the
workspace venv has no pytest):

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-SSS/blueboat_sss
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m pytest -p no:cacheprovider
```

The node tests skip when `blueboat_interfaces` is not found. To run them too, point the system
Python at the built package:

```bash
PYTHONPATH=~/ros2_ws/install/blueboat_interfaces/lib/python3.12/site-packages:$PYTHONPATH \
LD_LIBRARY_PATH=~/ros2_ws/install/blueboat_interfaces/lib:$LD_LIBRARY_PATH \
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m pytest -p no:cacheprovider
```

The test files are named after what they check (`tests/`). Two pieces of advice:

- **Point `data_root` at a temporary folder in a GUI test.** Recording moves `.svlog` files into
  a session, so a test on the real data folder would move field data.
- **Use `pytest.mark.skipif` for optional modules, never a module-level `importorskip`**: on this
  machine's pytest it aborts the whole run while still reporting success.

There is no linter or type checker.

## 5. Rules for changes

- **Side identity comes from the packet** (its channel number), never from the topic name.
- **Never drop a ping.** Rows are assembled by ping number; a row with one side only is still
  emitted.
- **Keep publishing the `raw` topics**: the `.svlog` is written from them.
- **Never change a delivery setting (QoS) on one end only.** The sonar topics are "best effort".
- **Never modify a recorded `.svlog`.** Tools that write must write elsewhere.
- **Two files exist twice on purpose**: `blueboat_gcs/tools/svlog_to_rosbag.py` and
  `blueboat_gcs/tools/svlog_helper.py` are byte-identical copies of the node-side files. Update
  them only by copying.
- **Keep the executable names** `sss_node.py` and `sss_processor_node`: the GCS finds the running
  nodes by those names.
- **No ROS types past the GCS's signal bus**: everything after `ros/` uses plain Python classes,
  so the app runs without ROS.
- **Do the same in the simulator**: acquisition settings, message rates and `.svlog` content must
  match between this module and BlueBoat-SSS-Sim. BlueBoat-Control owns the message types and
  changes by additions only.
- AI sessions never commit, push or tag: the owner does.

## 6. Next steps

From `.claude/TODO.md`, in plain words:

1. **Re-test the reworked sonar driver on the water.** The October 2026 recordings lost 4 to 61 %
   of pings; SonarView lost none the same day. The driver was rebuilt to work like SonarView
   (20 pings per second, one connection per sonar); it now needs a field session to confirm it.
2. **Measure the transducers' fore/aft position on the boat.** The value used (0.345 m behind the
   navigation point) is taken from the simulator's drawing, not measured.
3. **Find what resets the autopilot's message rates** about 9 s after each request. The position
   then arrives at 4 per second instead of about 13.
4. **Fix bottom tracking in deep water**: it fails on the deep harbour recordings.
5. **Record SonarView and this module side by side**, with identical settings, to settle the
   display's brightness mapping on real data.
