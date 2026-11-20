# BlueBoat-SSS — ROS 2 interface: driver and GCS

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

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
Since 2026-11-15 a picture built from pings that carry the fore/aft arm says georef
**convention 3** (`lever_arm_m.x` = the arm, same keys, schema still 5) and its grids
include the arm; the dB, the PNG and the row poses are unchanged, so the detector's input
is too. Perception reads the key and applies it: (2026-11-18) Perception applies it since 2026-11-15 (CR-PER-2, Perception `0d55d76`; re-checked by session 12, `~/ros2_ws/sim_runs/perception/handoff_12_cr_per_2.md`).

---
