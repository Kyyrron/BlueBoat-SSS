# Side Scan Sonar — Usage Guide

This package launches two nodes:

- **`sss_node`** — drives the pair of Cerulean Omniscan 450 devices and
  publishes raw `os_mono_profile` packets on `~/port/profile` and
  `~/starboard/profile`. Optionally writes a single SonarView-compatible
  `.svlog` containing both channels.
- **`sss_processor_node`** — subscribes to the raw topics, pairs port +
  starboard pings by ROS timestamp, descales power to dB, projects every
  sample to its `(x, y, z)` position in `base_link`, and publishes one
  `ProcessedSSSPing` per pair on `~/ping`. Optionally records the
  processed stream to a `ros2 bag` (mcap).

Both nodes have independent `~/ping/enable` and `~/log/enable` toggles
(processor only has logging, since "ping" is owned by `sss_node`).
Everything is OFF at startup.

## Launching the node

Standalone:

```bash
ros2 launch blueboat_control SSS_launch.py
```

With overrides at launch time (any of the parameters listed in
`SSS_launch.py`):

```bash
ros2 launch blueboat_control SSS_launch.py \
    range_length_mm:=50000 \
    gain_index:=4 \
    log_directory:=/userdata/sss_logs
```

From another launch file (for the future "boat + sss" composite):

```python
from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    # ...your other includes / nodes...
    sl.include('blueboat_control', 'SSS_launch.py')
    return sl.launch_description()
```

The node connects to both Omniscans on startup but **does not start
pinging or logging** — both are OFF until you flip them on with the
topics below.

## Start / stop pinging

Pinging is what actually fires the transducers and produces the data
streams on `/sss_node/port/profile` and `/sss_node/starboard/profile`.

```bash
# Start pinging (both sides at once)
ros2 topic pub --once /sss_node/ping/enable std_msgs/msg/Bool 'data: true'

# Stop pinging
ros2 topic pub --once /sss_node/ping/enable std_msgs/msg/Bool 'data: false'
```

Confirm data is flowing:

```bash
ros2 topic hz /sss_node/port/profile
ros2 topic hz /sss_node/starboard/profile
```

### Publishing rate

One ROS message is published per ping packet received — no batching.
The effective rate is set by the `msec_per_ping` parameter:

- `msec_per_ping = 0` (default) — pings as fast as the hardware can
  manage, which depends on range. Two-way travel time at 1500 m/s sets
  the ceiling: roughly **20–25 Hz at 30 m range**, **10 Hz at 75 m**,
  **~7 Hz at 100 m**.
- `msec_per_ping = N > 0` — lower-bounds the period at N ms. Set
  `msec_per_ping=50` for a steady **20 Hz** per side, `100` for 10 Hz,
  etc. Useful when downstream code expects a constant rate.

### Changing parameters mid-mission

The acquisition parameters (`range_length_mm`, `gain_index`,
`num_results`, `pulse_len_percent`, `msec_per_ping`, `range_start_mm`)
are read fresh on every "start". To change range without restarting:

```bash
ros2 topic pub --once /sss_node/ping/enable std_msgs/msg/Bool 'data: false'
ros2 param  set /sss_node range_length_mm 50000
ros2 topic pub --once /sss_node/ping/enable std_msgs/msg/Bool 'data: true'
```

> ⚠️ Don't leave the Omniscans pinging for long periods out of water —
> Cerulean's docs note the transmit transducer can heat up and be damaged.
> A few minutes dry is fine.

## Start / stop raw logging

Logging writes the raw packet stream to a `.svlog` file in the
**SonarView single-file format**: both sides interleaved in one file,
distinguished by the per-packet `channel_number` field. This is the
same layout SonarView produces when you click its **Record** button —
the file opens directly in SonarView with both channels visible.

```bash
# Start logging
ros2 topic pub --once /sss_node/log/enable std_msgs/msg/Bool 'data: true'

# Stop logging
ros2 topic pub --once /sss_node/log/enable std_msgs/msg/Bool 'data: false'
```

Logging only writes packets that are actually arriving from the
devices, so **enable pinging first** (or both at the same time).
Logging with pinging off creates the file with only its metadata
header — nothing else gets written until pings start.

Each off→on transition rolls a brand new `.svlog`. So a typical
recording session looks like:

```bash
ros2 topic pub --once /sss_node/ping/enable std_msgs/msg/Bool 'data: true'
ros2 topic pub --once /sss_node/log/enable  std_msgs/msg/Bool 'data: true'
# ...do the survey...
ros2 topic pub --once /sss_node/log/enable  std_msgs/msg/Bool 'data: false'
ros2 topic pub --once /sss_node/ping/enable std_msgs/msg/Bool 'data: false'
```

Files larger than 500 MB roll automatically into a continuation file
(same naming scheme, fresh metadata header).

## Choosing the log folder

The folder is the `log_directory` parameter. Default is `~/sss_logs`
(tilde expansion is handled by the node).

Override at launch time:

```bash
ros2 launch blueboat_control SSS_launch.py log_directory:=/userdata/sss_logs
```

Or in your composite launch file:

```python
sl.include('blueboat_control', 'SSS_launch.py',
           launch_arguments={'log_directory': '/userdata/sss_logs'})
```

The path can be absolute or use `~`. The directory is created on
startup if it doesn't exist. If you're running the node on the
BlueBoat onboard computer and want SonarView to pick up your logs
without copying, point this at `/userdata/SonarView` (the folder
SonarView watches by default — see the Blue Robotics Omniscan 450 SS
integration guide).

## What ends up on disk

One `.svlog` file per recording session, in the directory you
configured:

```
~/sss_logs/
├── 2026-05-08-14-03-27.svlog      # session 1 (started 14:03:27)
└── 2026-05-08-14-19-02.svlog      # session 2 (started 14:19:02)
```

Each file:

- Opens with a **JSON metadata packet** that declares both transducers
  in `session_devices` (`tcp://192.168.2.92:51200` and
  `tcp://192.168.2.93:51200` by default). SonarView reads this to set
  up the two display channels.
- Is then a stream of raw Ping-Protocol `os_mono_profile` packets from
  both devices, **interleaved in arrival order**. Each packet's
  `channel_number` field tells the player which side it came from.
- Each packet is framed exactly as Cerulean defines: `BR` sync bytes,
  little-endian payload length, message id (2198), device IDs, the
  52-byte fixed payload plus the `pwr_results` array (length =
  `num_results`), and a 2-byte checksum.

What you can do with it:

- **Open in SonarView** — desktop or BlueOS extension. Both channels
  show as a normal side-scan waterfall, exactly as if SonarView had
  recorded the session itself.
- **Export to XTF or SL2** from SonarView for use in other processing
  tools (ReefMaster, SonarWiz, etc.).
- **Replay through this node / `brping`** for offline ROS-side
  processing. The `examples/omniscan450Example.py` from
  `bluerobotics/ping-python` accepts a `.svlog` path as input and
  feeds packets through the same `wait_message()` API as live capture.

## Processed pings (sss_processor_node)

The processor consumes the raw topics, pairs port + starboard pings by
their ROS arrival timestamps (a two-pointer match within
`match_tolerance_ms`, default 50 ms), descales the u16 power samples to
dB, projects each sample to its `(x, y, z)` in `base_link` under a
horizontal-beam assumption (no bathymetry yet), and publishes one
`blueboat_interfaces/ProcessedSSSPing` per pair on
`/sss_processor_node/ping`.

The output message holds, for each side, `num_samples` parallel arrays
of `x`, `y`, `z`, `intensity_db` — so the consumer reads "intensity at
this point in the boat frame" without any conversion. In REP-103:
port samples sit at `+y`, starboard at `-y`.

### Transducer geometry

Three parameters control where the samples land in `base_link`:

| parameter               | meaning                             | default |
| ----------------------- | ----------------------------------- | ------- |
| `transducer_x_m`        | longitudinal offset (forward = +)   | 0.0     |
| `transducer_y_offset_m` | lateral offset from centerline      | 0.0     |
| `transducer_z_m`        | vertical offset                     | 0.0     |

Sample i on each side sits at slant range `r_i = (start_mm + i *
length_mm / (num_results - 1)) / 1000` in meters. Port → `(x, +y +
r_i, z)`, starboard → `(x, -y - r_i, z)`. When you have bathymetry, the
processor can be upgraded to project onto the seafloor instead of a
flat plane.

### Start / stop processed-ping logging

This is independent from the raw `.svlog` toggle:

```bash
# Start recording processed pings (mcap bag)
ros2 topic pub --once /sss_processor_node/log/enable std_msgs/msg/Bool 'data: true'

# Stop
ros2 topic pub --once /sss_processor_node/log/enable std_msgs/msg/Bool 'data: false'
```

The processor spawns a `ros2 bag record --storage mcap` subprocess in
its own process group, so it won't die when the node receives a
SIGINT. On stop, the subprocess gets a clean SIGINT and the bag closes
properly.

### What ends up on disk for processed pings

`ros2 bag` writes a **folder** per session (mcap convention), placed
next to the raw `.svlog` files:

```
~/sss_logs/
├── 2026-05-08-14-03-27.svlog              # raw, from sss_node
├── 2026-05-08-14-19-02.svlog
├── processed-2026-05-08-14-03-30/         # processed bag
│   ├── metadata.yaml
│   └── processed-2026-05-08-14-03-30_0.mcap
└── processed-2026-05-08-14-19-05/
    ├── metadata.yaml
    └── processed-2026-05-08-14-19-05_0.mcap
```

Raw and processed get **independent timestamps** (you toggle each
recorder separately, so they may not start at the exact same moment).

What you can do with a processed bag:

- **`ros2 bag play <folder>`** — replays the `ProcessedSSSPing` topic
  through ROS exactly as it was published, so downstream nodes (AI
  patches, georeferencing) can consume it offline without any of the
  upstream ones running.
- **`ros2 bag info <folder>`** — total messages, duration, topic list.
- **Read with `rosbag2_py` in Python** — for analysis notebooks. The
  mcap format also opens directly in Foxglove Studio.

## Quick health check

Useful one-liners for sanity-checking that the node is alive:

```bash
# Are the nodes up?
ros2 node info /sss_node
ros2 node info /sss_processor_node

# Are subscribers attached to the control topics?
ros2 topic info /sss_node/ping/enable
ros2 topic info /sss_node/log/enable
ros2 topic info /sss_processor_node/log/enable

# Is data actually arriving when ping is on?
ros2 topic echo /sss_node/port/profile --field ping_number
ros2 topic hz   /sss_node/starboard/profile
ros2 topic hz   /sss_processor_node/ping
```

If `ros2 topic hz /sss_processor_node/ping` is markedly lower than the
raw-topic rates, the pairing tolerance is probably too tight — bump
`match_tolerance_ms` up to ~½ of the actual ping period.

If raw rates are fine but the processor reports 0 Hz, the most common
cause is a QoS mismatch. Subscribers to the raw topics must use
`BEST_EFFORT` / `KEEP_LAST(10)` to connect (this node already does).
