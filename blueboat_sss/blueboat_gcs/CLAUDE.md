# CLAUDE.md — BlueBoat GCS (`blueboat_gcs`)

Loaded when working under `blueboat_sss/blueboat_gcs/`. Moved verbatim from
`BlueBoat-SSS/.claude/CLAUDE.md` on 2026-10-08; that file keeps the module-wide
non-negotiables (NC #1–#16, which bind the GCS too), the node-side contract and the
commands — "NC #n" and "Verification status" refer to it. On 2026-11-18 the design decisions, the data
formats and the testing notes moved verbatim to `BlueBoat-SSS/.claude/gcs-design.md`, `gcs-data.md` and
`gcs-testing.md`; read the relevant one before changing that part of the GCS.

## ROS 2 interface of the GCS

rclpy node name `blueboat_gcs`. Subscribes (**VERIFIED**; topic names configurable in
`config/default.yaml`, defaults mirrored in `config/settings.py`):

| Topic | Type | Notes |
|---|---|---|
| `/sss_processor/processed` | `ProcessedSSSPing` | `BEST_EFFORT`, depth **200** |
| `/side_scan_sonar/{port,starboard}/profile` | `OmniscanProfile` | `BEST_EFFORT`, depth **200** (`sonar_stream.profile_queue_depth`). **Since 2026-09-05**: the raw per-side profiles, cached by device `ping_number` and attached verbatim to the matching processed row (`core/live_native.py`), so the live waterfall and the AI pictures draw the same native bins the replay path decodes from the `.svlog` — water column, ringing and all. A profile missing after `profile_wait_ms` (60 ms) falls back to the re-projection for that side; the console warns once when more than 5 % of recent rows had to. **Bug fix 2026-10-16:** that re-projection now inverts the processor's own geometry (the 0.285 m lever arm and its altitude = `water_depth − TRANSDUCER_SUBMERSION_M`, 0.22 m since 2026-11-02, was 0.15); since `994a74b` it had put a missed side on the wrong bins (20 m / 1200 bins at 1.2 m: bin 83 instead of 72) — `tests/test_fallback_lever_arm.py`. |
| `/blueboat/odom` | `nav_msgs/Odometry` | `BEST_EFFORT`, depth 10 |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `BEST_EFFORT`, depth 10. Gated by the pure `utils/geodesy.navsat_fix_ok`: **only** an explicit `STATUS_NO_FIX` (-1), non-finite coordinates, or the `(0, 0)` no-fix sentinel are rejected; `STATUS_UNKNOWN` (-2) is **accepted** — it is the ROS 2 Iron+ message *default*, and exactly what the MCS bridge's simulated GPS sends for Gazebo runs of GPS-anchored missions (MCS is the only GPS publisher in a sim graph; it fills lat/lon only). Rejecting all negative statuses silently discarded that whole feed — the "GCS says no GPS while MCS anchors" field bug. First accepted and first rejected fix are logged to the console |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | depth 10 |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | optional; without `mavros_msgs`, speed comes from the odom twist |
| `/blueboat/pinger_coordinates` | `std_msgs/Float32MultiArray` | gated by the pure `utils/pinger.parse_pinger`: **all-zero vectors are dropped** (the producer streams `zeros(3)` at ~20 Hz before any USBL detection — the field "marker rides the boat" bug), NaN/short arrays are dropped, and the frame is taken from the wire shape — `[x, y, z]` = vehicle frame (normal path), `[x, y]` = world frame (`fixed_pinger`). `alignment.pinger_frame: auto` (default) honours that; `robot`/`world` force one reading. The marker hides after `alignment.pinger_stale_after_s` (10 s) without a fix |
| `/set_path` | `nav_msgs/Path` | from `path_publisher.py` — the **fallback** since 2026-09-18. That publisher asks the path service for a fixed `total_time` window (1000 s), so it truncates any longer mission: the 1317 s survey of 2026-09-07 drew as its first 1000 s and stopped mid-water. It lives in `BlueBoat-Control`, frozen at v1.0, so the station reads the trajectory **it deployed itself** instead — `core/deployed_path.py`, the newest `~/.config/blueboat_mcs/trajectories/.deployed/*.yaml`, every sample of it, placed through the designer file's `geo_anchor` — and `/set_path` only draws when there is no deployed file, no anchor, or `planned_path.prefer_deployed: false` |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | **from BlueBoat-Perception, the station's one detection source** (2026-10-16): one array per seabed image; every map marker, waterfall marker, left-panel count and `detections.csv` row comes from here. A non-zero per-detection `header.stamp` (its row time) wins over the array's |
| `/mavros/imu/data` | `sensor_msgs/Imu` | `BEST_EFFORT`, depth 10 (2026-10-16). Roll / pitch / yaw rate attached to live pings at reception with their age (`utils/attitude.py`) for the seabed-image metadata — **never geometry**; the ping quaternion is not used (yaw-only on the real boat) |
| `/rosout` | `rcl_interfaces/Log` | depth 50; the GCS filters out its own node |

Publishes: `/side_scan_sonar/ping/enable` (Bool), `/sss_processor/log/enable` (Bool),
`/sss_ai/seabed_analysis` (`std_msgs/String`, JSON, schema = the picture metadata's,
5 since 2026-10-16; `detections` always `[]`) — image metadata, **never pixels** — and,
since 2026-10-16, **every seabed image for BlueBoat-Perception** (recording or not), keyed
`"<stream_id>/<image_id>"` (`Image.header.frame_id` / the JSON envelope), RELIABLE /
KEEP_LAST 32 / VOLATILE at both ends (CM-4): `/sss_ai/seabed_image` (`sensor_msgs/Image`
32FC1, the raw float dB = `_world.npz:intensity_db`), `/sss_ai/seabed_image/png` (mono8,
= the PNG), `/sss_ai/seabed_image/metadata` (`std_msgs/String`, the envelope around
`SeabedImage.metadata()`). Built by the ROS-free `core/perception_bridge.py`, sent by
`RosManager.publish_seabed_image` from the main window's one-thread publish pool.
Contract: `BlueBoat-Perception/docs/topics.md`; the SSS side:
`docs/PERCEPTION_INTEGRATION.md`.

The GCS **does not subscribe to `~/raw`.** **VERIFIED.** It subscribes to the two
`~/profile` topics (above) since 2026-09-05.

Every listener guards its optional imports: a missing `blueboat_interfaces` disables the
sonar stream, a missing `vision_msgs` disables detections, a missing `mavros_msgs` falls
back to the odom twist for speed — each with a status message rather than a traceback.

## Key GCS design decisions (full text: `BlueBoat-SSS/.claude/gcs-design.md`)

Read the full text before changing any of these. The topics, in file order:

- **Signal bus** (one `AppSignals`; `rclpy` spins on a background thread) and **console capture**.
- **Default view** and **depth compensation** (the display model, nadir handling, the ringing blank on the
  ground / mosaic projection only).
- **One scale, and it says which** (2026-09-18: the "True scale" checkbox is gone).
- **Waterfall orientation** newest-on-top (SonarView convention) and **column scale**.
- **Mosaic resolution** is adaptive (`MosaicService._auto_tune_cell_size`) and preserved across changes.
- **Waterfall click → map point** (5 px slop).
- **Fore/aft arm** (2026-11-15, CR-SSS-2): a ping is placed from the transducers' point.
- **Runtime sonar range** (the Acquisition group, main window only).
- **Replay timeline** is the log's own clock; **sessions are segments** (packet id 10 opens a session).
- **Pose alignment**, **stream clock**, **pinger gating**, **GPS-anchored map**, **heading policy**.
- **Stream health** (three counters), **pipeline lifecycle** (an explicit START/STOP state machine),
  **sonar source** (real sonars, simulation, and the GCS starting the simulated sonar beside a bag replay),
  **recording lifecycle guards**.

## Data produced by the GCS (full text: `BlueBoat-SSS/.claude/gcs-data.md`)

Recording sessions (`.svlog`, `metadata.json`, waterfall, mosaic, seabed images), merged sessions, and the
seabed images with their JSON metadata (schema 5), which are the AI feed. Data leaves the GCS only through
recording sessions (NC #9). Late pings (2026-11-02) and the georef conventions 1–3 are described there.
**Simulated sessions:** `metadata.json["simulation"]` holds the simulator's run record (profile, world digest,
seeds, install = source); simulated results follow the superproject's `SIM_VALIDITY_CONDITIONS.md`. An
offline rebuild of a simulated session can run behind the live ping time (5.11 s on the C2 mission): align on
the seabed images' `t_s`.

## Testing and performance notes (full text: `BlueBoat-SSS/.claude/gcs-testing.md`)

The GCS test suite, the live-pipeline budget (2026-09-03) and the performance measurements. Exact commands and
pass counts: superproject skill `module-gates`.
