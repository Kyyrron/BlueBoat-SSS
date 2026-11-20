# BlueBoat-SSS — `sss_processor_node` (node `sss_processor`)

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

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
reboot resets intervals). Since 2026-11-03 it also renews the request whenever the
slowest of ATTITUDE / GPI / LPN, measured on arrival over the last 3 s (recording or
not), is below `POSE_RATE_WARN_HZ` (10), at most once per 5 s; the 30 s re-assert stays
the fallback (`_PoseRateMeter`, `_assert_mavlink_rates`). Reason: on the 2026-10-28 water
run the 30 Hz held for only ~9 s of every 30 s period, 4 Hz otherwise (calibration pass 4
report, owner's machine; UNVERIFIABLE HERE). Something resets the intervals; what is
`TODO.md`'s, and the renewal is unverified on water. Absent the service (simulation) it
logs once and carries on; nothing is requested.
Without the request the autopilot streams **4 Hz** on the mavros link; with it,
SonarView's own recordings carry **12.8 Hz** per message (the autopilot's delivered
ceiling, measured on the boat; cause UNVERIFIABLE HERE), and run bags made while
SonarView was open show `/mavros/local_position/odom` at 10.7–13 Hz. Our own request did
not hold steadily on the 2026-10-28 run (above), so "~13 Hz while the sonar streams" is
the intent, not a measured result. **The request is
autopilot-wide — `/blueboat/odom` and BlueBoat-Control's 20 Hz loop get the faster pose
too, for as long as it holds** (owner accepted, 2026-10-27: fresher pose, a smooth PID derivative instead of a
5× spike every fifth tick, gains may need a retune, `/mavros/*` in run bags ~3×; root
`CLAUDE.md` §4.4).

**Rate watchdog.** While recording, every 10 s (sim seconds under `use_sim_time`), it warns once when ATTITUDE, GPI or LPN
arrive below 10 Hz, or when a side's ping rate is more than 10 % off SonarView's law
(`sonarview_ping_hz`: max(50 ms, 2R/c + 3 ms)), and says when the rates recover. Its
window opens at the first logged pose envelope after Record ON (or at the first tick if
none was logged), and a window under 5 s is carried into the next tick: before
2026-11-03 the first window included the time before the `.svlog` opened (2.8 Hz
reported on 2026-10-28).

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
GCS's `core/svlog.py`. So `water_depth = altitude + 0.22` and
`y = ±(0.285 + sqrt(s² − altitude²))`. **VERIFIED** (`sss_processor_node.py`, the
*Transducer geometry* block). The golden pings keep their frozen 0.15 convention
(`tests/golden_pings.py`).

**Fore/aft arm (2026-11-15, owner's decision of 2026-11-14; CR-SSS-1 / CR-SSS-2).**
`TRANSDUCER_X_OFFSET_M` = **−0.345** (+ forward: 0.345 m aft of the `/blueboat/odom`
pose; was 0.0), = the simulator's `model.mount_x_m`, mirrored in the GCS's `core/svlog.py`.
**Adopted from the simulator's `config/default_sonar.yaml`, not measured**
(`~/ros2_ws/sim_runs/sim_acceptance/owner_measurements.yaml` says so; the drawing gives
−0.345 m from the hull centre, the pool walls −0.60 m from the navigation fix with the GPS
near the bow: open, `TODO.md`). `tests/test_fore_aft_arm.py` holds the processor's four
constants, the GCS mirror, the config and launch defaults and the simulator's file together.
- **Processor:** the constant is the default of the node parameter
  `transducer_x_offset_m` (the node's only declared parameter; dynamic typing, so `:=0`
  is accepted; a value that is not a finite number logs an error and keeps the default),
  forwarded by `SSS_processing_launch.py`'s argument of the same name. It is published
  unchanged in every `ProcessedSSSPing.transducer_x_offset` and changes nothing else in
  the row: the pose stays the navigation pose, `*_y` are untouched
  (`tests/test_processor_fore_aft_arm.py`).
- **Consumers place a ping from `pose + R(yaw)·(arm, 0)`.** The GCS does, everywhere
  (`blueboat_gcs/CLAUDE.md`, *Fore/aft arm*). BlueBoat-Belief does since 2026-11-15 (its
  `DESIGN.md` D-12: it reads `transducer_x_offset` from every processed ping; the installed node
  predates it until `~/ros2_ws/install` is rebuilt). BlueBoat-Perception does too: (2026-11-18) Perception applies it since 2026-11-15 (CR-PER-2, Perception `0d55d76`; re-checked by session 12, `~/ros2_ws/sim_runs/perception/handoff_12_cr_per_2.md`).
- **Old sessions:** `transducer_x_offset_m:=0.0` on the launch line (processor) and
  `alignment.transducer_x_offset_m: 0.0` in the GCS config re-process a recording exactly
  as it was placed before 2026-11-15. A parameter, not a code fork.
- **`~/ros2_ws/install` must be rebuilt** after a change of the constant
  (`colcon build --packages-select blueboat_sss`): the GCS takes the arm of a live ping from
  the message, so a stale installed processor silently keeps the old value. Its start-up
  line says which arm it runs with.

`.svlog` files are written to `../../../../data/SSS_data`, resolved relative to the
launch working directory — the same root the GCS uses as `data_root`. The node resolves
it to an absolute path once at construction and creates it there, so every path it
reports afterwards is absolute; `SvlogWriter.start` creates it again at Record ON. A
directory that cannot be created or written is logged at **error** level and leaves
recording off — the processor still runs, because processing and logging are
independent. **VERIFIED.**
