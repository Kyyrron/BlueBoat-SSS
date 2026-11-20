# BlueBoat-SSS — measured acquisition settings and data produced

*Moved verbatim from `CLAUDE.md` on 2026-11-18 (session 11), which keeps a short summary and points here. Lines marked "(2026-11-18)" were corrected after the move.*

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
