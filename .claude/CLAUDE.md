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

## ROS 2 interface

This is the highest-value section: nearly every real bug in this project has been an
interface or framing mismatch. Types prefixed `blueboat_interfaces/` come from that
external interfaces package.

### `sss_node.py` — node name `side_scan_sonar`
Drives **two** Cerulean Omniscan 450 SS units over TCP, one `OmniscanWorker` thread
each with a reconnect loop, each owning one `OmniscanLink`
(`src/_custom_libraries/omniscan_link.py`, ROS-free: its own socket, receive buffer
and framer — blocking `recv_into`, checksum-verified frames, resync past corruption).
Port is `192.168.2.92:51200`, starboard `192.168.2.93:51200`, hard-coded in the node.
**VERIFIED.** `brping` is no longer used at runtime (2026-10-27): its
`PingDevice._input_buffer` is a **class** attribute, so the two devices of one process
read both TCP streams into one deque from two threads (the 15:07 field log of
2026-10-27: `[starboard] connection lost: pop from an empty deque`, a forced reconnect
mid-survey); its TCP `read_io` busy-spins a core per idle thread; it parses byte by byte
and drops message ids it has no template for. It stays the test oracle for the command
bytes (`tests/test_omniscan_link.py`).

| Direction | Topic | Type | Other end |
|---|---|---|---|
| Pub | `~/port/profile` | `blueboat_interfaces/OmniscanProfile` | processor |
| Pub | `~/port/raw` | `std_msgs/UInt8MultiArray` | processor (svlog) |
| Pub | `~/starboard/profile` | `blueboat_interfaces/OmniscanProfile` | processor |
| Pub | `~/starboard/raw` | `std_msgs/UInt8MultiArray` | processor (svlog) |
| Sub | `~/ping/enable` | `std_msgs/Bool` | GCS START/STOP |

All four publishers use `BEST_EFFORT`, `KEEP_LAST`, depth 10. Pinging is **OFF at
startup**; acquisition parameters are re-read on every enable. **VERIFIED.**

`~/raw` carries the **already-framed Cerulean Ping Protocol packet**, verbatim as the
device sent it — **every** device frame, not only profiles: the Omniscan status frames
(id 2194, ~0.9 Hz per device in SonarView logs) and any id newer firmware adds go to
`~/raw` only (into the `.svlog`, as SonarView records them); replies to our own
commands (ack, nack, device_information) are not forwarded, and a NACK is logged as a
warning. `OmniscanProfile` mirrors the decoded header (fields:
`BlueBoat-Control/blueboat_interfaces/msg/OmniscanProfile.msg`; `pwr_results` is
`uint16[]`, normalised per ping onto `[min_pwr_db, max_pwr_db]`), filled from one C
array (`array('H')`), no per-sample Python objects.

**Health line** every 10 s per side while pinging: profiles and rate, missing ping
numbers, longest gap, checksum errors, resync bytes, other ids seen (warning level when
anything was lost). A ping missing *there* was lost before ROS (device, WiFi, TCP);
the processor's and GCS's own counts can only add DDS losses — the split the
2026-10-27 analysis needed and did not have.

Parameters, declared identically in `sss_node.py` and both launch files
(**VERIFIED**): `range_start_mm` (0), `range_length_mm` (20000), `msec_per_ping`
(**50**, SonarView's command since 2026-10-27), `gain_index` (−1 = device auto),
`num_results` (1200), `pulse_len_percent` (0.00125) — SonarView's acquisition exactly
(only the range may differ by the owner's order). `msec_per_ping` is a lower bound:
period = max(it, 2R/c + ~3 ms), so 20 pings/s up to ~35 m; `0` free-runs the device at
2R/c + 2.3 ms, **MEASURED** 29.0 ms (34.5 Hz) at 20 m on every 2026-10-27 recording,
1.7× the link bandwidth and a 2.3 ms range-ambiguity margin. A stop repeats the last
configuration with `enable=0`. `FILTER_DURATION_PERCENT` is pinned at 0.0015 and is not
a parameter (SonarView's value is not observable in any log: UNVERIFIABLE HERE). 1200 /
0.00125 = the device's best range resolution (owner's decision 2026-09-28; the transmit
pulse stays on its **MEASURED** 44.3 µs floor up to ~27 m). The simulator's
`sss_sim_node` declares the same six defaults (its smoke test asserts the equality).

### `sss_processor_node.py` — node name `sss_processor`
Slant-range correction, bottom tracking, port/starboard merge, and `.svlog` writing.
The two responsibilities are independent: processing runs whether or not logging is on.

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Sub | `/side_scan_sonar/{port,starboard}/raw` | `std_msgs/UInt8MultiArray` | feeds `.svlog` |
| Sub | `/side_scan_sonar/{port,starboard}/profile` | `OmniscanProfile` | feeds processing |
| Sub | `/blueboat/odom` | `nav_msgs/Odometry` | pose interpolated to each ping, 5 s buffer |
| Sub | `/sss_processor/log/enable` | `std_msgs/Bool` | GCS Record ON/OFF |
| Pub | `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | GCS |
| Cli | `/mavros/set_message_interval` | `mavros_msgs/MessageInterval` | SonarView's mavlink rates, below |

All of the above are **VERIFIED** against the source, and the three externally-visible
names match `blueboat_gcs/config/default.yaml` exactly.

**SonarView's mavlink rates (2026-10-27, owner's order: "as close to SonarView as
possible").** While profiles arrive (one within 5 s), the processor requests from the
autopilot exactly the message intervals SonarView writes in its id-10 header
(`svlog_helper.DEFAULT_PERIODIC_MESSAGES`: HEARTBEAT 1 Hz; ATTITUDE, LOCAL_POSITION_NED,
GLOBAL_POSITION_INT 30 Hz; SCALED_PRESSURE2, VFR_HUD 3 Hz; GPS_GLOBAL_ORIGIN 1 Hz;
AUTOPILOT_VERSION 1/60 Hz), asynchronously, and repeats it every 30 s (an autopilot
reboot resets intervals). Absent the service (simulation) it logs once and carries on.
Without the request the autopilot streams **4 Hz** on the mavros link; with it,
SonarView's own recordings carry **12.8 Hz** per message (the autopilot's delivered
ceiling, measured on the boat; cause UNVERIFIABLE HERE), and run bags made while
SonarView was open show `/mavros/local_position/odom` at 10.7–13 Hz. **The request is
autopilot-wide — `/blueboat/odom` and BlueBoat-Control's 20 Hz loop get the faster pose
too** (owner accepted, 2026-10-27: fresher pose, a smooth PID derivative instead of a
5× spike every fifth tick, gains may need a retune, `/mavros/*` in run bags ~3×; root
`CLAUDE.md` §4.4).

**Rate watchdog.** While recording, every 10 s, it warns once when ATTITUDE, GPI or LPN
arrive below 10 Hz, or when a side's ping rate is more than 10 % off SonarView's law
(`sonarview_ping_hz`: max(50 ms, 2R/c + 3 ms)), and says when the rates recover.

Mavros telemetry, wrapped into `.svlog` packet-id-150 mavlink envelopes
(**VERIFIED** — these are the ROS topic names, not the mavlink message names):

| Topic | Type | Becomes |
|---|---|---|
| `/mavros/imu/data` | `sensor_msgs/Imu` | `ATTITUDE` (ENU to NED) |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `GLOBAL_POSITION_INT` |
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` | cached into `relative_alt` |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | cached into `hdg` (cdeg) |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `LOCAL_POSITION_NED` |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | cached, paired with the pose, and converted (ENU → NED, cm/s) into `GLOBAL_POSITION_INT`'s `vx/vy/vz` — a replayed waterfall's along-track scale is read from those fields; `.svlog` files written before 2026-09-18 carry zeros there |
| `/mavros/home_position/home` | `mavros_msgs/HomePosition` | `HOME_POSITION` |
| `/mavros/global_position/gp_origin` | `geographic_msgs/GeoPointStamped` | `GPS_GLOBAL_ORIGIN` |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | `VFR_HUD` |
| `/mavros/state` | `mavros_msgs/State` | `HEARTBEAT` (SonarView's shape: Rover `custom_mode`, `base_mode` bits, `MAV_TYPE_SURFACE_BOAT`) |
| `/mavros/statustext/recv` | `mavros_msgs/StatusText` | `STATUSTEXT` (50-char array, as SonarView records it — the link guardian's messages land in the `.svlog`) |

`time_boot_ms` is derived from each message `header.stamp` so messages from one source
burst keep an identical value — SonarView pairs ATTITUDE / GLOBAL_POSITION_INT /
LOCAL_POSITION_NED on it to compute heading-corrected position. **A zero stamp** (a
real-robot launch: mavros on `use_sim_time` with no `/clock`) uses the arrival time
instead, grouped by `_BurstClock` (arrivals within 5 ms share one value). Before
2026-10-27 every pose of every field `.svlog` carried one frozen value,
4288904828 = (0 − node boot) mod 2³². `HOME_POSITION` / `GPS_GLOBAL_ORIGIN` carry
`time_usec` on the same clock.

**Wire format as SonarView writes it** (`svlog_helper`, both copies, NC #7): the mavlink
wrapper is tagged `src=0xFF`, `dst=0` (it was `src=3` until 2026-10-27; no reader here
routes mavlink by `src`), its mavlink2rest JSON has sorted keys and a 2-space indent;
the file header is the id-10 session metadata followed by SonarView's id-12 view config
(`DEFAULT_VIEW_CONFIG`), and `doppler_enable` is `true` as SonarView writes it (its
meaning is UNVERIFIABLE HERE).

Sonar and odom subscriptions use `BEST_EFFORT`, `KEEP_LAST`, **depth 10**. **VERIFIED.**

`ProcessedSSSPing` fields: `BlueBoat-Control/blueboat_interfaces/msg/ProcessedSSSPing.msg`.

The sign of `*_y` encodes the side: **+y = port, −y = starboard**. Rows are keyed by the
side resolved from the packet (`_accept` → `_channel_of`, NC #1; the arrival topic only when
the packet resolves nothing), so the `port_*` fields carry the packet's port ping and
`port_y` is never negative, whichever topic delivered it (`_side_geometry`; verified
2026-10-19). The topic-routed behaviour this paragraph once described predates the
packet-based routing. Consumers still merge on the sign (the GCS concatenates both fields
and reads only that), which stays correct either way. Samples are already slant-range corrected and the water column is
already removed.

**One-sided rows.** A row carries whichever sides arrived. The absent side has
`*_ping_number = 0`, a zeroed `*_stamp`, and empty `*_intensity_db` / `*_y`;
**presence is `*_ping_number != 0`, not array length**, because `project_side`
legitimately yields an empty array for a *present* side when the altitude covers
the whole swath. **VERIFIED.**

**Rows are keyed by `ping_number`, normalised across the two devices.** The two
Omniscan units are independent devices with independent counters, so a row's
`port_ping_number` and `starboard_ping_number` differ by the boat's constant device
offset — **0, −1 and +60 all measured in the field corpus** (see
`blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md`). `sss_helper.PingCounterOffset`
recovers it by voting each ping against the opposite side's temporally nearest one;
`sss_processor_node` holds a short pre-roll until the offset is known so the first
pings of a mission are keyed correctly. What signals a torn row is the gap
*changing*, never the gap being non-zero. **MEASURED.**

**Pose is interpolated** (2026-10-27): `_OdomBuffer.pose_at` places each ping between
the two odom samples that bracket its stamp (position linear, orientation a normalised
lerp on the short arc), falling back to `nearest()` with the same tolerance; at 4 Hz
odom the nearest-sample snap stepped the track by up to 25 cm at 1 m/s.

**Pose is a hard gate.** `_emit_group` drops the ping outright if the odom buffer
is empty, and again if the pose lookup returns nothing; both increment
`_dropped_no_odom`. The lookup refuses a sample farther than
`ODOM_NEAREST_TOLERANCE_S` (1 s) from the profile stamp and prunes stale entries at
lookup time too — previously a dead or clock-mismatched `/blueboat/odom` left the
buffer's last sample latched and every ping was stamped with that one stale pose
(the "pings pile on one point" field bug); now such pings take the sanctioned drop
with a warning naming the measured skew. There is no fallback pose in the
processor — the GCS mitigations (`blueboat_gcs/CLAUDE.md`) exist because of this.
This is the **only** drop in the processor. **VERIFIED.**

**Altitude never gates emission.** `FBRTracker.update` returns the best value
available — locked, else provisional from this ping's own detections, else last known
— and `locked` reports quality. When nothing has ever been detected the processor
uses `0.0`: ground range = slant range, no water column removed, which is an identity
transform rather than a guessed altitude, and is the same value the GCS
`Depth comp. = off` selector applies. `_unlocked_pings` counts pings **emitted**
while unlocked; nothing is dropped for it. **VERIFIED.**

The four transducer-geometry constants: since `994a74b` (2026-10-13)
`TRANSDUCER_Y_OFFSET_PORT_M` / `_STBD_M` = **0.285** (BR-101447 hull spacing 570 mm / 2)
and `TRANSDUCER_SUBMERSION_M` = **0.22** since 2026-11-02 (**measured**, SSS-Sim
field-calibration pass 3: the rear-most hull holes 0.030 m above the keel, the waterline
0.216–0.241 m at 16–22 kg, and the 0.90 m pool reproduces every run's FBR altitude to
±0.03 m; = the simulator's `sensor_depth_m`; was 0.15, an estimate), mirrored in the
GCS's `core/svlog.py`; `TRANSDUCER_X_OFFSET_M` is still 0.0: the drawing says −0.345 m
from the hull centre, the pool walls −0.60 m from the navigation fix (GPS near the bow),
and it only rides in `ProcessedSSSPing.transducer_x_offset` — no georeference uses a
fore/aft arm yet (a georef convention change, Perception's contract; planned). So
`water_depth = altitude + 0.22` and `y = ±(0.285 + sqrt(s² − altitude²))`. **VERIFIED**
(`sss_processor_node.py:136-139`). The golden pings keep their frozen 0.15 convention
(`tests/golden_pings.py`).

`.svlog` files are written to `../../../../data/SSS_data`, resolved relative to the
launch working directory — the same root the GCS uses as `data_root`. The node resolves
it to an absolute path once at construction and creates it there, so every path it
reports afterwards is absolute; `SvlogWriter.start` creates it again at Record ON. A
directory that cannot be created or written is logged at **error** level and leaves
recording off — the processor still runs, because processing and logging are
independent. **VERIFIED.**

### BlueBoat GCS

rclpy node name `blueboat_gcs`; its full subscription table, gating rules and design
decisions are in **`blueboat_sss/blueboat_gcs/CLAUDE.md`** (loads when working there).
Cross-module essentials: it subscribes `/sss_processor/processed` and the two `~/profile`
topics (`BEST_EFFORT`, depth **200**) — **never `~/raw`** — and publishes
`/side_scan_sonar/ping/enable`, `/sss_processor/log/enable` and `/sss_ai/seabed_analysis`
(JSON, never pixels; `detections: []`). Everything downstream of `ros/` consumes plain
dataclasses (NC #10).

**BlueBoat-Perception (2026-10-16, by the owner's order) is the station's one detection
source.** The GCS publishes every seabed image for it — `/sss_ai/seabed_image` (32FC1),
`/sss_ai/seabed_image/png` (mono8), `/sss_ai/seabed_image/metadata` (JSON), RELIABLE /
KEEP_LAST 32 — and draws only what comes back on `/sss_ai/detections`; it runs no detector
of its own (the former in-process dummy now lives in Perception, reproduced exactly). It
also subscribes `/mavros/imu/data` for per-row attitude metadata. What changed in this
module, and what is pinned unchanged: `blueboat_gcs/docs/PERCEPTION_INTEGRATION.md`.
Perception's default detector input (`fixed_gray`, its decision 6.4) is a fixed mapping of
the **32FC1 dB** and the per-row `gain_port_db` / `gain_stbd_db`, not of the display PNG:
a change to those values, to the pictures' slant pitch or to the live 0.75 m nadir blank
changes what its detector is trained and run on. The display model does not reach it.

---

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
   stale on every tick. Both the node and the GCS `SonarListener` now warn once when
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
    imported, and 64 of the 68 swept GCS modules import cleanly with no ROS at all
    (the four that do not are exactly the `ros/` listeners, which `main.py` imports
    lazily and only outside `--sim`; `tests/test_sweeps.py`, 2026-10-16). The live
    listener's logic that can be tested without ROS lives in `core/live_native.py` for
    exactly that reason. The sweep count excludes `tools/`, whose three files are
    checked by hash instead (NC #7). `core/perception_core.py` imports
    BlueBoat-Perception's (ROS-free) core lazily, for the replay window's Run AI only.

### Scientific validity (from the thesis plan)

11. **The headline detector trains on real beach imagery only.** Training it on
    synthetic data would inherit aspect sensitivity from the model instead of
    measuring it. Synthetic-trained detectors are for the sim-to-real gap experiment
    only.

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

## Measured acquisition settings

Reference points from three real logs (**MEASURED**, and **UNVERIFIABLE HERE** —
the corpus that backs them is an external mount, see *Verification status*; full
analysis in `blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md`):

| | our sea trial | SonarView, same boat | Cerulean demo |
|---|---|---|---|
| Range | 80 m | 20 m | 25.4 m |
| Samples/ping | 600 | 600 | 1200 |
| Range sampling | 133 mm | 33 mm | 21 mm |
| Ping interval | 110 ms (9.1 Hz) | 50 ms (20 Hz) | 50 ms (20 Hz) |
| Transmit pulse | 213 µs | 66 µs | 44 µs |
| Bottom at sample | 49/600 | 269/600 | 137/1200 |
| Missing ping numbers | ~8 % | 0 | 0 |
| Wrong-`src` packets | 19.8 % | 0 | 0 |

Gain is auto on both stacks and changes on <1 % of pings — it is **not** a significant
contributor to banding. Transmit pulse length scales with range
(`max(44.3 µs, pulse_len_percent × 2R/c)`), so a long range degrades range resolution too.

**2026-10-27, same boat, same day, same 20 m / 1200 bins / 44.3 µs / auto gain**
(**MEASURED** on files local to this machine, `~/ros2_ws/data/SSS_data/sessions/`;
`blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md` §11):

| | SonarView pool ×2 | ours, pool 10:30 | ours, harbour 15:07 / 15:54 / 16:07 |
|---|---|---|---|
| Recorder | SonarLink on BlueOS (the boat's Pi) | `sss_processor_node`, laptop over WiFi | same |
| Ping interval | **50 ms** (20 Hz) | 29 ms (34.5 Hz, free run) | 29 ms |
| Missing ping numbers | **0 %** | 5.2 % | 30.9 / 60.6 / 4.0 % (stalls to 7.1 s) |
| Pose (ATTITUDE/GPI/LPN) | 12.8 Hz each | 4 Hz | 4 Hz (2 Hz during the 60.6 % run) |
| `time_boot_ms` | advancing | frozen | frozen |
| Status frames (2194) | ~0.9 Hz/device | none | none |
| `channel_number` | 0/1 | 255 | 0/1 |

The pings missing from our `.svlog` (processor, `~/raw` depth 10) are exactly the ones
missing from the GCS's records (`~/profile` depth 200) in all four sessions: lost before
ROS — the brping receive path (NC #3a) and the WiFi link, which SonarLink never crosses.
Everything in that table except the recorder location is fixed in this module since
2026-10-27; the field re-test is `TODO.md`'s.

**The default range is 20 m.** `range_length_mm` defaults to `20000` in all three
places that declare it: `src/sss_node.py`, `launch/SSS_processing_launch.py` and
`launch/SSS_simple_launch.py`, giving **16.7 mm range sampling** at the default 1200
samples (since 2026-09-28; 33.3 mm at the former 600). This is the no-argument default,
sized from water depth per NC #5, and `project_synthesis.md` §8.5's setting for every
survey condition (owner, 2026-10-20; the old 30 m coverage / 15 m revisit passes are
gone). `msec_per_ping` defaults to **50** in the same three places (SonarView's
interval, 2026-10-27).

---

## Data produced

**`.svlog`** — SonarView-compatible stream of framed Cerulean Ping Protocol packets.
**VERIFIED** structure (`svlog_helper.py`): `BR` magic, u16 payload length, u16 packet
id, byte 6 = `src_device_id`, byte 7 = `dst_device_id`, payload, u16 checksum =
`sum(bytes[0 .. 7+N]) & 0xFFFF`. Packet ids: 10 (session JSON), 12 (view config), 150
(mavlink wrapper), 2194 (Omniscan status: 16-byte `<ffI3xB`, ~0.9 Hz per device, no
imagery — `decode_omniscan_status`; the two floats are housekeeping telemetry of
unestablished meaning; forwarded by our driver since 2026-10-27 whenever the device sends
them), 2198 (`OS_MONO_PROFILE`). Device ids: port 1, starboard 2, platform 3 (declared in
the session JSON). Session-metadata packets (10, then 12) use `src=0`, `dst=0xFF`; mavlink
wrappers `src=0xFF`, `dst=0`, as SonarView writes them. Written by the processor; readable
by SonarView and by the GCS replay window. Files roll at 500 MB
(`MAX_LOG_SIZE_BYTES = 500 * 1000 * 1000`).

**Recording sessions, merged sessions and seabed images** (all GCS outputs): format and
contract in `blueboat_sss/blueboat_gcs/CLAUDE.md` *Data produced by the GCS*. The two
rules that bind everyone: data leaves the GCS only through recording sessions (NC #9),
and the AI feed is the seabed **pictures + their JSON metadata** (decided 2026-09-01).

---

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
`msec_per_ping:=0` free-runs the device, a diagnostic only). `SSS_simple_launch.py` = acquisition
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
- **387 pass / 51 skip on 2026-10-27** with the workspace's `blueboat_interfaces` on
  `PYTHONPATH` / `LD_LIBRARY_PATH` (356 / 50 before the SonarView-parity work; the new
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
message-interval request (sent while streaming, re-asserted, skipped without the
service) and the rate watchdog. It needs a sourced ROS 2
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

**Pre-commit hook.** `.githooks/pre-commit` runs the suite and blocks the commit on
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
