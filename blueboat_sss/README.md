# Side Scan Sonar — Usage Guide

Robot-side quick start for the `blueboat_sss` ROS 2 package. The full interface
reference is `../.claude/CLAUDE.md`; the operator copy-paste command sheet is
`terminals.txt`.

The package ships two nodes:

- **`side_scan_sonar`** (`src/sss_node.py`) — drives the pair of Cerulean
  Omniscan 450 SS units over TCP (port `192.168.2.92:51200`, starboard
  `192.168.2.93:51200`). Publishes the decoded header on `~/port/profile` and
  `~/starboard/profile` (`blueboat_interfaces/OmniscanProfile`), and the
  already-framed Cerulean Ping Protocol packet, verbatim, on `~/port/raw` and
  `~/starboard/raw` (`std_msgs/UInt8MultiArray`).
- **`sss_processor`** (`src/sss_processor_node.py`) — slant-range correction,
  bottom tracking (FBR), port/starboard merge, and `.svlog` writing. Publishes
  `blueboat_interfaces/ProcessedSSSPing` on `/sss_processor/processed`.

Pinging and logging are both **OFF at startup**.

## Launching

```bash
# Processor only — this is what the GCS START button runs
ros2 launch blueboat_sss SSS_processing_launch.py

# Acquisition only
ros2 launch blueboat_sss SSS_simple_launch.py
ros2 launch blueboat_sss SSS_simple_launch.py range_length_mm:=20000 gain_index:=4
```

## Start / stop pinging

Pinging fires the transducers and produces the data streams.

```bash
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: false'
```

> ⚠️ Don't leave the Omniscans pinging for long periods out of water —
> Cerulean's docs note the transmit transducer can heat up and be damaged.
> A few minutes dry is fine.

## Start / stop `.svlog` recording

Logging belongs to the **processor**, not the driver. This is the GCS
Record ON/OFF toggle.

```bash
ros2 topic pub --once /sss_processor/log/enable std_msgs/msg/Bool 'data: true'
ros2 topic pub --once /sss_processor/log/enable std_msgs/msg/Bool 'data: false'
```

The processor writes `.svlog` to `../../../../data/SSS_data`, resolved relative
to the **launch working directory** — the same root the GCS uses as `data_root`.
The node resolves that to an absolute path and creates the directory at startup,
and enabling logging creates it again if it has gone. If it cannot be created or
written, the processor logs an **error** (visible on `/rosout` and in the GCS
console) and recording stays off — it never reports a recording that is not
there. Every log line names the file, not just the directory.

The file is a SonarView-compatible stream of framed Ping Protocol packets: both
channels interleaved in one file, distinguished by the per-packet
`channel_number` field, so it opens directly in SonarView with both channels
visible. Files roll at 500 MB (`MAX_LOG_SIZE_BYTES` in
`src/_custom_libraries/svlog_helper.py`).

The `.svlog` is rebuilt from `~/port/raw` and `~/starboard/raw`. Those two
topics must keep being published, or the logs come out empty.

Recorded `.svlog` files are primary field data — never edit or overwrite them.
The writer holds to that too: a name collision rolls to `<stamp>-001.svlog`
rather than replacing the file already there.

## Acquisition parameters

Declared identically in `src/sss_node.py` and both launch files, and re-read on
every ping enable:

| parameter | default | meaning |
| ------------------- | ------- | ------------------------------------------ |
| `range_start_mm`    | 0       | start of the sampled window |
| `range_length_mm`   | 20000   | swath per side, in mm |
| `msec_per_ping`     | 0       | 0 = as fast as the hardware manages |
| `gain_index`        | -1      | -1 = device auto |
| `num_results`       | 600     | samples per ping |
| `pulse_len_percent` | 0.002   | transmit pulse as a fraction of range |

**Set the range from the water depth (~4x the deepest expected), not from the
area you hope to cover.** The 80 m used in the sea trials pushed the bottom
return to sample 49/600 and broke bottom detection outright. The 20 m default
gives 33.3 mm range sampling at 600 samples.

To change a parameter without restarting, toggle pinging off, set it, toggle
back on:

```bash
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: false'
ros2 param  set /side_scan_sonar range_length_mm 30000
ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'
```

### Publishing rate

One ROS message per ping packet received — no batching. `msec_per_ping = 0`
pings as fast as the hardware manages, which depends on range: two-way travel
time at ~1500 m/s sets the ceiling, roughly **20–25 Hz at 30 m**, **10 Hz at
75 m**, **~7 Hz at 100 m**. `msec_per_ping = N > 0` lower-bounds the period at
N ms — set `50` for a steady 20 Hz per side. Note that `pulse_len_percent`
scales the transmit pulse with the range, so a long range costs range
resolution as well as rate.

## Transducer geometry

Not launch parameters — four module-level constants at the top of
`src/sss_processor_node.py`, all currently `0.0` and carrying `TODO`s:
`TRANSDUCER_X_OFFSET_M`, `TRANSDUCER_Y_OFFSET_PORT_M`,
`TRANSDUCER_Y_OFFSET_STBD_M`, `TRANSDUCER_SUBMERSION_M`. Measure them on the
physical BlueBoat before any localization-accuracy work.

In the processed message the sign of `*_y` encodes the side: **+y = port,
-y = starboard**. Samples are already slant-range corrected and the water
column is already removed.

## Quick health check

```bash
# Are the nodes up?
ros2 node info /side_scan_sonar
ros2 node info /sss_processor

# Are subscribers attached to the control topics?
ros2 topic info /side_scan_sonar/ping/enable
ros2 topic info /sss_processor/log/enable

# Is data actually arriving when ping is on?
ros2 topic echo /side_scan_sonar/port/profile --field ping_number
ros2 topic hz   /side_scan_sonar/starboard/profile
ros2 topic hz   /sss_processor/processed
```

If the raw rates are fine but the processor reports 0 Hz, the most common cause
is a QoS mismatch. All four sonar publishers are `BEST_EFFORT` / `KEEP_LAST(10)`;
a `RELIABLE` subscriber is QoS-incompatible and receives *nothing*. Change both
ends together or not at all.

The processor also treats pose as a hard gate: it drops a ping pair outright if
`/blueboat/odom` has produced nothing, so a silent processor with healthy raw
topics can equally mean no odom.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select blueboat_sss
source install/setup.bash
```

`CMakeLists.txt` installs `launch/` plus the five `src/` scripts flat into
`lib/blueboat_sss/`; the flat layout is what lets `sss_processor_node.py` import
`sss_helper` / `svlog_helper` / `math_helper` without a package prefix.

Robot-side dependencies: `bluerobotics-ping` (`brping`), `simple_launch`,
`scipy`, `mavros_msgs`, `geographic_msgs`, and `blueboat_interfaces` (supplied
by BlueBoat-Control).
