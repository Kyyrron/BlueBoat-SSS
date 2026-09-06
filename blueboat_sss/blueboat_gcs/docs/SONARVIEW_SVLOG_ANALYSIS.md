# SonarView vs. our GCS — measured differences, root causes, fixes

Evidence base (all numbers measured, nothing assumed):

| File | Recorded by | Content |
|---|---|---|
| `reflection_evidence.svlog` | **our** stack | 16.8 MB, 5250 profiles, dual Omniscan |
| `diffDepthCompensation.svlog` | **SonarView** | 9.8 MB, 4900 profiles, dual Omniscan, same boat |
| `harbor_scan_combined.svlog` | Cerulean demo | 18 MB, 4278 profiles, single Omniscan |

`reflection_evidence.svlog` is a shorthand used throughout this document; on disk the
file is `ShiraishiJima/MainReflectionEvidence+misspingsWaterfall/2026-07-23-11-45-00.svlog`.

Every number below is reproducible with
`python3 -m blueboat_gcs.analysis.svlog_forensics --compare <the three files>`.

---

## 1. The headline result: our raw acoustics are fine

Sorting each channel by `ping_number` and correlating consecutive pings beyond the
nadir gives **median neighbour correlation 0.84 (port) / 0.86 (starboard), with zero
decorrelated rows**. The two transducers fire at a rock-stable **−42 ms** offset
(σ = 0.5 ms) and `gain_index` changes on only 0.6–0.9 % of pings.

**Nothing is corrupted acoustically.** Every artifact below comes from *labelling,
ordering and settings* — all fixable in software, which is why SonarView can render
our own file better than we could.

---

## 2. The mirror — root cause found

Each profile packet carries its own `channel_number` (0 = port, 1 = starboard) and
`transducer_heading_deg` (−90 / +90). In our file these two **always agree with each
other**, but they disagree with the packet's `src` device tag on **1040 of 5250
packets (19.8 %)**, in runs of up to 24 consecutive pings — exactly the banded "weird
zones". SonarView's own recording is 100 % consistent (2450/2450 per side).

We routed pings by the device tag and paired the two sides on **arrival time**
(50 ms). Measured consequences on `reflection_evidence.svlog`:

| Defect | Share of emitted rows |
|---|---|
| Same physical side drawn on **both** left and right (the mirror) | **11.4 %** |
| Port and starboard fully **swapped** | **15.0 %** |
| The two halves came from **different pings** | **27.3 %** |
| Rows discarded because one side was unmatched | 260 (**10.4 %**) |

**Fix (implemented).** Route by the packet's own `channel_number` (falling back to the
sign of `transducer_heading_deg`), and group by `ping_number` instead of arrival time.
Result: 0 % mirrored, 0 % swapped, 0 % cross-ping, and 2755 rows emitted instead of
2495. Visual before/after: `docs/waterfall_fix_comparison.png`.

The residual black bands in the "after" image are rows where the device genuinely
delivered only one side. They are now shown honestly instead of being hidden by
dropping the row.

---

## 3. Real ping loss (robot-side, not fixable inside the GCS)

`ping_number` gaps in our file, routed by `channel_number`: **211 missing on channel 0
(7.7 %) and 49 on channel 1 (1.8 %)** against a span of 2755 pings per side — 260 in
total, which is the same 260 one-sided rows §2 counts. PRI is a rock-stable 110 ms
(p99 110 ms) interrupted by eight stalls over 500 ms, the longest 3287 ms on channel 0
and 4382 ms on channel 1. SonarView's recording on the same hardware has **zero**
missing ping numbers and a PRI of 50.0 ms (p95 51 ms).

> Routing matters for this statistic too. Keyed on the `src` tag — the routing §9.1
> replaced — the same file reports 356 and 192 (548 total), because the 19.8 % of
> packets carrying a wrong tag are counted as gaps on both sides at once. The
> channel-routed figures above are the real loss.

**Correction to an earlier note in this document:** `~/raw` must NOT be disabled.
`sss_processor_node` subscribes to `/side_scan_sonar/{port,starboard}/raw` and those
framed packets are exactly what it writes into the `.svlog` (`_write_raw_with_src_tag`).
Turning the raw publication off would produce empty log files. The GCS itself does not
subscribe to `~/raw` at all — verified: its only sonar subscription is
`/sss_processor/processed`.

So the loss must be attacked without removing the topic:

1. **QoS.** Both hops use `BEST_EFFORT` with `depth=10`. BEST_EFFORT never
   retransmits, and 10 slots is 0.5 s at 20 Hz — one slow mosaic re-render overruns
   it. SonarView has no lossy hop at all: it reads the device over TCP.
   *GCS side (done):* subscriber depth raised to **200** (`sonar_stream.queue_depth`).
   A RELIABLE subscriber is **not** an option unless the publisher changes too —
   RELIABLE-subscriber/BEST_EFFORT-publisher is QoS-incompatible and would receive
   nothing. *Robot side (recommended):* make both `sss_node`'s publishers and the
   processor's subscriptions RELIABLE with `depth=50`; at ~1.2 kB and 20 Hz per side
   this is cheap and removes the middleware as a loss source.
2. **Serialization cost in the hot loop.** `_publish_raw` does
   `msg.data = list(raw)`, building a ~1250-element Python list per ping per side, and
   `_publish_profile` does `list(data.pwr_results)` (600 more). That is ~3700
   Python-level object conversions per ping-pair, inside the same thread that must
   return to `wait_message` before the device's next packet. Use buffer protocol
   types instead — `array.array('B', raw)` and `array.array('H', data.pwr_results)`
   are accepted by rclpy for `uint8[]` / `uint16[]` and avoid the per-element boxing.
3. **Socket buffer.** Enlarge `SO_RCVBUF` on the device socket so a scheduling hiccup
   does not overflow the kernel queue.

The GCS no longer *adds* to this loss, and it now measures it: `SonarListener` tracks
gaps in the device's own `ping_number` and reports them in the embedded console
("N ping(s) lost upstream of the GCS"), so acquisition loss can never again be
mistaken for a display problem.

## 4. Settings: the single biggest quality lever

| | ours (sea trial) | SonarView | Cerulean demo |
|---|---|---|---|
| Range | **80 m** | 20 m | 25.4 m |
| Samples/ping | 600 | 600 | 1200 |
| Range sampling | **133 mm** | 33 mm | 21 mm |
| PRI | **110 ms (9.1 Hz)** | 50 ms (20 Hz) | 50 ms (20 Hz) |
| Transmit pulse | **213 µs** | 66 µs | 44 µs |
| Bottom return at sample | **49 / 600** | 269 / 600 | 137 / 1200 |

The 80 m range is the cause of four separate symptoms at once: 4× coarser sampling,
half the ping rate, a 3× longer pulse (16 cm vs 5 cm range resolution), and a bottom
return so close to the transmit ringing that **bottom detection stops working** —
which is precisely why SonarView shows `Detected N/A` on our file no matter which
source is selected.

The last symptom is not merely "close to the ringing", it is structural.
`find_noise_window_start` searches only the first `RINGING_SEARCH_MAX = 60` samples,
and `detect_fbr_slant_m` begins its bottom search at `nw_start + NOISE_FLOOR_WINDOW`
— no earlier than sample ≈50 with the fallback `nw_start = 30`. Because that horizon
is a **sample count**, its physical meaning scales with `range_length_mm /
num_results`, which sets a minimum detectable altitude per range setting (600
samples):

| `range_length_mm` | mm/sample | FBR floor (≈sample 50) |
|---|---|---|
| 80000 (our sea trial) | 133 | **6.7 m** — bottom measured at sample 49 ≈ 6.5 m, inside the noise-floor window and therefore unfindable |
| 30000 | 50 | 2.5 m |
| **20000 (current default)** | **33.3** | **1.67 m** |
| 15000 | 25 | 1.25 m |

Against the site depths in `project_synthesis.md` §3.3 — 0.5–5 m at the beach test
site, 2–6 m in the marina — a 30 m range puts the floor above the shallow end of
both; 20 m clears it.

**Fix (implemented).** The default range is **20000 mm** at all three sites that
declare it — `src/sss_node.py`, `launch/SSS_processing_launch.py` and
`launch/SSS_simple_launch.py`. At the default `num_results = 600` that gives **33.3 mm
per sample**, matching the SonarView reference column above. Rule: set the range from
the water depth (~4× the deepest water expected), never from the area you hope to
cover.

This is the no-argument default, chosen to be safe at the shallow end of our sites. It
does not replace `project_synthesis.md` §8.5, which reserves **30 m per side for
coverage passes and 15 m for revisit passes**; those remain the experiment settings and
are passed explicitly per run, as `terminals.txt` already does.

### Packet id 2194 — Omniscan status

Established from the bytes, across all three reference logs (`core/svlog.py`
`decode_omniscan_status`, pinned by `tests/test_svlog_replay.py`):

* 26-byte frame, **16-byte payload**, layout `<ffI3xB`;
* the `uint32` is on the **same device clock as `timestamp_ms`** — inside that
  channel's profile time range, monotonic, and never more than half a ping interval
  from the nearest profile (median 11–13 ms, max 25 ms against a 50 ms PRI). It is
  sampled independently on that clock, not copied from a profile: only ~3 % land
  exactly on one;
* byte 15 is `channel_number`, agreeing with the frame's `src` tag on every packet seen;
* **~0.9 Hz per device** (1.11 s median interval), i.e. one per ~22 pings at 20 Hz.
  Counts: 192 (Cerulean demo), 218 (SonarView 20 m), 418 (Tire4 25 m).

The earlier figure of **0.31 Hz was wrong**, and wrong for an instructive reason: it
divided the Cerulean demo's 192 packets by the file's whole 611.5 s span, which
contains a 397.8 s gap in which nothing was recorded. Against the 213.8 s the sonar
was actually acquiring it is 0.90 Hz. That is the segmentation defect below, showing
up as a rate.

**Not established: what the two floats are.** They are device-specific, span ~45–75
with the pair 17–19 apart, and drift by under 1 over a whole log. They correlate with
nothing acoustic — |r| < 0.2 against the same channel's `max_pwr_db`, `min_pwr_db`,
`gain_index` and raw power statistics — so they are housekeeping telemetry rather
than signal statistics. `doppler_enable` in the device options is Cerulean's
speed-over-ground (Doppler) processing and this packet appears alongside it, but
nothing measured here ties the two together; the association is a guess and is
recorded as one. The packet carries no imagery, so nothing downstream needs it. The
loader counts it (`SvlogMission.unparsed_packets`) rather than skipping it silently.

Gain: both stacks run auto-gain. SonarView's settles at `gain_index` 4 for 96 % of
pings; ours moved across 4–7 but changed on <1 % of pings, so gain is **not** a
significant contributor to the banding.

---

## 5. Depth compensation — what it is, and why theirs looked better

SonarView's "Depth Compensation" is simply **the altitude used for slant-range
correction**, with a selectable source (port sonar / starboard sonar / manual). It is
the same quantity our FBR tracker estimates.

Two questions answered:

* **"Why does the waterfall change when I change it?"** In our app it **no longer
  does** (2026-09-02): the waterfall and the AI pictures draw the **raw slant-bin
  domain** (one column per device bin, verbatim dB), so the `Depth comp.` selector
  governs only the **mosaic** (ground range = `sqrt(slant² − h²)`). Only the mosaic
  warps with altitude now.
* **"Why does Manual / 0 m look better?"** Because with `h = 0` no warping is
  applied. For shallow water (`h << R`) ground range ≈ slant range anyway, so a
  *wrong* altitude is far more damaging than *no* correction: an over-estimated
  altitude both deletes real samples and compresses the near range. With our 80 m logs
  the altitude estimate wandered between 4.7 m and 45 m (p10 4.67, p90 15.23), so
  "off" genuinely was the better choice on that data.

**Fix (implemented).** A `Depth comp.` selector in the right panel with the same three
options — `Auto (bottom detect)`, `Manual`, `Off (no correction)` — wired to both the
live view and the replay window. Changing it in the replay window re-processes the log,
mirroring SonarView's behaviour.

### 5.1 The transmit ringing is blanked; the water column is not

Right under the transducer the profile leaves at **55 dB**, and the *brightest*
seabed return anywhere in the file is 54.8 dB (p99). That near-field spike is
transmit ringing, not seabed, and it sat as a bright core down the middle of every
`off` image and splatted onto the track line in the mosaic.

Everything past it is real. Measured on `diffDepthCompensation.svlog`:

| slant range | median dB | |
|---|---|---|
| 0.00 m | 55.2 | ringing, brighter than any seabed return |
| 0.30 m | 43.3 | still above the median seabed |
| **0.73 m** | **36.5** | **crosses the median seabed level (36.5 dB)** |
| 1.00 m | 33.2 | now darker than the seabed |
| 2.00 m | 27.6 | water column proper |
| 6.00 m | 16.5 | water-column floor |
| ~9.4 m | 27.9 → 42.6 | the bottom return |
| 13.4 / 20 m | 39.5 / 25.6 | seabed |

Note the crossing at 20 m: the seabed's own far return is **25.6 dB**, *darker* than
the water column at 2 m. So the water column cannot be separated from the seabed by
level, and no contrast window can black it out without blacking out the outer swath
too — its mid-tone appearance is honest, and it is what SonarView shows as well.

**Fix — one display model (2026-09-05; supersedes the 2026-09-03 seabed-referenced
EGN and the interim 2026-09-02 raw single-window).** The table above is the whole
story: the seabed's far return (25.6 dB at 20 m) is *darker* than the water column at
2 m (27.6 dB), so **no single window over raw dB can separate them** — whatever blacks
out the water column also blacks out the far seabed. SonarView's answer, and now ours,
is to **flatten the range falloff first**, then window. The 2026-09-03 attempt did that
with a per-column *mean* seabed reference and a 5th-percentile low handle; measured on
the 2026-09-04 simulation log both were contaminated by the 5–14 % of samples that are
acoustic shadows (15–40 dB below the seabed), which put the low handle ~30 dB under the
seabed — the water column (only 15–35 dB under) rendered at 70–90 % brightness with the
ringing gradient on top (the "weird nadir"), shadows rendered as grey noise, and any
wall or shadow at a fixed range biased its column's mean into a vertical band.
`core/display_model.py` replaces it (background, references and measurements in
`SCIENTIFIC_BACKGROUND.md`):

1. **Normalisation** `e = db + TL(r) − A_side(r/h)`: the deterministic two-way
   transmission loss `TL(r) = 40·log10 r + 0.2·r` removed, then an empirical seabed
   curve `A` in normalised slant range `x = r/h` (`h` the tracked bottom) divided out —
   per side, the **mode** of each log-spaced `x` bin's level histogram over seabed
   samples, median-filtered across bins. The mode is immune to shadows, walls and
   targets as long as plain seabed holds the plurality of a bin; the `x` axis makes the
   curve altitude- and range-invariant. For `x < 1` `A` holds `A(1)`, so the
   extrapolated `TL` sends the water column and the ringing toward black by physics.
2. **Transfer** `u = clip(10^(γ(e − hi)/10), 0, 1)`: `hi` a robust high percentile of the
   normalised seabed level, `γ` the Contrast slider (0.7 default; 1 = linear power).
   **No low handle.**

One model per window, stored in every seabed picture's JSON (`display_model`) and
`_world.npz`, in `waterfall_raw.npz` and in the session `metadata.json`, so a picture is
losslessly invertible: `e = hi + (10/γ)·log10(u)`, `db = e − TL(r) + A(r/h)`. The live
path draws the same raw bins as replay since the GCS subscribes to the raw profiles
(`core/live_native.py`), so this analysis applies to both.

The earlier sample-removal **nadir blank still exists, but only on the ground/mosaic
projection**: `project_side` (unchanged) cuts the transmit ringing
(`depth.nadir_blank_m = 0.75`, the measured 0.73 m crossing, `max(correction, blank)`)
from the ground samples that feed the mosaic, so the ringing never splatters the boat
track in `off` mode. Those samples are removed rather than NaN'd because
`MosaicGrid.add_samples` has no finiteness filter (one NaN poisons a cell forever) —
this constraint is unchanged. The *native* slant-bin payload the waterfall/pictures
use no longer applies that cut.

The FBR tracker is advanced on **every** ping in every mode (`resolve_altitude` used
to early-return for `off`/`manual`), which is what makes the bottom estimate available
to the contrast split — carried on each ping as `SonarPing.bottom_slant_m` — even when
the applied `water_depth` is 0.

## 6. FBR bootstrap was throwing data away

The old tracker returned `None` until 10 consecutive detections agreed within 0.30 m,
and the caller **dropped every ping** until then — losing the start of every mission
and arbitrary chunks whenever lock was lost.

**Fix (implemented, both stacks).** `FBRTracker.update` now always returns the best
available altitude (locked → provisional → last known) and exposes `locked`
separately, so quality can be reported without discarding data; the caller falls back
to 0.0 (no correction) when nothing has ever been detected. **No ping is ever dropped
for lack of a depth lock.** On `reflection_evidence.svlog`, 593 pings that were
previously discarded are now displayed with a provisional altitude. The robot side
carries the same behaviour in `sss_helper.FBRTracker`, and counts pings emitted while
unlocked (`_unlocked_pings`) rather than pings dropped.

The only remaining drop reason is a genuinely missing pose (no `LOCAL_POSITION_NED`
before the sonar data): 0 pings in our file, 6 in SonarView's.

---

## 7. Mosaic resolution is no longer fixed

The mosaic was rasterized on a hard-coded 0.25 m grid, which threw away most of the
sensor's resolution — a large part of the apparent sharpness gap.

**Fix (implemented).** `MosaicService` derives the ground-sample distance from the
data: the median across-track sample spacing over the outer half of the swath, clamped
to `[min_cell_size_m, max_cell_size_m]`. Measured results:

| Log | theoretical spacing | auto cell | previously |
|---|---|---|---|
| ours, 80 m / 600 | 133 mm | **134 mm** | 250 mm |
| SonarView, 20 m / 600 | 33 mm | **41 mm** | 250 mm |
| Cerulean, 25.4 m / 1200 | 21 mm | **21 mm** | 250 mm |

A `Resolution` selector (Auto / 2 / 5 / 10 / 15 / 25 / 50 cm) is exposed in the right
panel. Changing it rebuilds the grid, so accumulated data is cleared.

---

## 8. Waterfall geometry stabilised

Columns were scaled by `max|y_local|`, which depends on the altitude estimate. With an
estimate wandering 4.7 → 45 m every row got a different scale and the image rippled.
`SonarPing` now carries `slant_range_m` (the *configured* range, constant for a given
setting) and the waterfall scales by that, falling back to the old behaviour for logs
and the simulator that do not provide it.

---

## 9. Robot-side state

### 9.1 Side identity is read from the packet (done)

`_write_raw_with_src_tag()` used to tag each packet with the device id implied by **the
topic it arrived on**, burning a wrong `src` permanently into the `.svlog` — which is
why SonarView also rendered our files with banded artifacts. It now derives the tag from
the packet: for `OS_MONO_PROFILE` (id 2198), `channel_number` is payload byte 26, i.e.
**byte 34 of the framed packet**, with `transducer_heading_deg` (payload byte 44, **byte
52** of the frame, float32 LE) as the fallback where `channel_number` is outside `(0, 1)`.
The device id the raw callbacks pass is only the fallback.

```python
CHANNEL_BYTE = 34           # 8-byte frame header + payload offset 26
HEADING_BYTE = 52           # 8-byte frame header + payload offset 44

@staticmethod
def _src_from_packet(raw: bytes, fallback: int) -> int:
    """Device id from the packet itself; the topic is only a fallback."""
    if (len(raw) > CHANNEL_BYTE
            and int.from_bytes(raw[4:6], "little") == OS_MONO_PROFILE_ID):
        ch = raw[CHANNEL_BYTE]
        if ch in (0, 1):
            return DEVICE_ID_PORT if ch == 0 else DEVICE_ID_STBD
        if len(raw) >= HEADING_BYTE + 4:
            heading, = struct.unpack_from("<f", raw, HEADING_BYTE)
            if heading:
                return DEVICE_ID_STBD if heading > 0 else DEVICE_ID_PORT
    return fallback
```

Measured over the whole corpus: `channel_number` and `transducer_heading_deg` agree on
**100 %** of packets in every file (`ch 0 ⟺ hdg −90`, `ch 1 ⟺ hdg +90`). Three of our
recordings carry a wrong `src` on 10.6 / 19.8 / 29.5 % of packets; two more carry
`channel_number = 255` on every packet, so only the heading fallback identifies their
side — and on those two the `src` tag is close to a **coin flip**, disagreeing with the
transducer heading on **51.6 %** (`2026-07-21-15-50-26`) and **41.1 %**
(`2026-07-22-13-43-08`) of packets. They are the worst files in the corpus, not clean
ones: routing by the tag would scramble about half of each.

Replaying `reflection_evidence.svlog` (5250 profiles, 19.8 % mis-tagged) through the
processor and re-recording gives **0.0 % mismatch with all 5250 profiles preserved**.

### 9.2 Live projection (done) and assembly (done)

The live path is projected on the robot, so the mirror had to be fixed there too.
`_emit_group` resolves both the sign and the transducer offset per message through
`_side_geometry`, applying the identical rule:

```python
ch = msg.channel_number
if ch not in (0, 1):                    # defensive fallback
    ch = 1 if msg.transducer_heading_deg > 0 else 0
side_sign = +1.0 if ch == 0 else -1.0
```

Field assignment is unchanged: a message that arrived on the port topic still fills
`port_y` / `port_intensity_db`, with a negative `y` when the packet says starboard.
Consumers merge on the sign, not the field name.

Assembly now matches the replay path: rows are keyed by `ping_number`, one-sided rows
are published rather than dropped, and no ping is withheld while the bottom tracker
bootstraps. A missing `/blueboat/odom` pose is the only remaining robot-side drop.

### 9.3 The two devices do not share a ping counter

Assembling on the raw `ping_number` is **not** safe, and this was measured only after
the assembly work started. The two Omniscan 450 units are independent devices with
independent counters. Their relative offset is constant for a power-up cycle but
otherwise arbitrary. Over the whole corpus (18 files, 16 two-sided), pairing each ping
with the opposite side's temporally nearest one:

| `port_pn − stbd_pn` | logs |
|---|---|
| `0` | 10 |
| `−1` | 4 — `2026-07-22-14-18-55`, `HarbourCleanExample`, `example-disturbances`, `toOptimize-harbourScan` |
| `+60` | 2 — `diffDepthCompensation`, `2026-07-21-15-50-26` |

On the `+60` logs, grouping by the raw counter merges halves that are **seconds** apart:

| log | halves merged, raw key | halves merged, aligned key |
|---|---|---|
| `diffDepthCompensation.svlog` | median 2982 ms, max 3217 ms | max 20 ms |
| `2026-07-21-15-50-26.svlog` | median 1737 ms, max 1740 ms | max 6 ms |
| `toOptimize-harbourScan.svlog` | median 40 ms | max 116 ms |

At survey speed that is metres of boat travel inside one row. This is **not** the
`channel_number = 255` anomaly — `diffDepthCompensation.svlog` has clean
`channel_number` 0/1 and still shows `+60`.

The offset is recovered by voting each ping against the opposite side's *temporally
nearest* one (`sss_helper.PingCounterOffset`,
`blueboat_gcs.core.svlog.estimate_counter_offset`). Voting against the most recently
*seen* opposite ping instead is **bimodal**, splitting roughly evenly between the true
offset and offset ±1 depending on interleave phase (`Tire4-25m`: 0×4687 vs −1×4561),
so each vote is deferred by four arrivals until both neighbours are available. With
that deferral the winning mode holds ≥ 94 % of votes on every two-sided log, and 100 %
on ten of them.

Both stacks apply the correction: `sss_processor_node` learns it incrementally and
holds a short pre-roll so the first pings of a mission are keyed correctly; the GCS
replay path establishes it in a pre-pass over the file, **per session segment** — a
device power cycle between two recordings in one file reassigns the offset.

### 9.4 The devices do not share a clock with the autopilot either

A `.svlog` carries two independent clocks, and neither is wall time:

| | counts from | measured range |
|---|---|---|
| `OS_MONO_PROFILE.timestamp_ms` | the **sonar's** boot | e.g. 3 003 574 → 3 615 124 ms |
| mavlink `time_boot_ms` | the **autopilot's** boot | e.g. 358 667 → 970 247 ms |

The offset between them is per-file and arbitrary — **2 644 886, 58 363, 56 171 and
57 566 ms** on four corpus logs — while the two tick at the same rate: their spans
agree to 30 ms over 611 s. So one measured constant puts both streams on one timeline
(`estimate_boot_skew`, the median of `timestamp_ms − time_boot_ms` taken against the
most recent profile). It is stable enough for a single file-wide value: the p1–p99
spread is 50–120 ms over a whole log, and the two segments of the Cerulean demo agree
to 20 ms.

Two failure modes are real in the corpus and are handled separately, because the two
clocks fail separately:

* **`timestamp_ms` unusable** (zeros, or a backwards jump beyond
  `STAMP_BACKSTEP_TOLERANCE_MS` = 1 s) → the whole timeline falls back to the 20 ms
  tick. The test is applied **per channel per segment**, never over file order: the
  writer batches by channel, so file order is non-monotonic on real logs (145, 125 and
  1076 inversions on three files) while each channel's own sequence is monotonic on
  every log. One log does swap 9 adjacent pings out of ~17 000, each by exactly one
  29 ms ping interval — a writer artefact the final event sort absorbs, and far too
  small to justify discarding a good clock, which is what the 1 s tolerance encodes.
* **`time_boot_ms` frozen** — `No_sonarVNotOK_usOK_SimpleCurve` carries one single
  value on all 2008 of its mavlink packets. Anchoring poses to it would collapse every
  `RobotState` onto one instant. There the sonar clock still carries the timeline and
  poses ride it instead, rather than the good clock being discarded along with the bad.

### 9.5 One file can hold several recording sessions

Packet id 10 marks the start of a session. Cerulean's own harbour demo holds **two**:
165.1 s of acquisition, a **397.8 s** gap, then 48.7 s — 611.5 s of span for 213.8 s
of data. The loader used to dispatch only ids 150 and 2198, so id 10 was decoded and
discarded and the two sessions were concatenated. Consequences, all of them silent:

* the mission reported **251.6 s** on the flat tick instead of 611.5 s;
* the waterfall stacked the last row of session 1 against the first of session 2 as
  neighbours, which reads as continuous seabed;
* the mosaic rasterizer would densify a swath straight across the join wherever the
  boat had not moved more than its 2.5 m guard;
* two seabed image tiles spanned the gap — 256-row windows whose rows are 397.8 s
  apart, with a fictitious speed spike, and nothing in the PNG to show it.

Sessions are now `SvlogMission.segments`, the dead time between them is a `MissionGap`
event, and every accumulator breaks on it (waterfall seam, rasterizer reset,
trajectory subpath, seabed window flush). Replay skips the gap and says so, while the
slider keeps true mission time. `acquisition_s` (segments summed) and `duration_s`
(span, gaps included) are both reported, and agree with `svlog_forensics` on every
corpus log.

## 10. What I still need

* A short `.svlog` recorded **after** the range change (20 m) to confirm the ping-rate
  and resolution gains on our own hardware.
* A `.svlog` recorded on the two Omniscan units after the §9.1 retag patch. Replay
  confirms 0 % mis-tagging; only real acquisition can confirm it at the source.
* Whether SonarView still out-renders us at *identical* range settings once the
  mis-tagging is gone — my current assumption is that the residual gap was mostly the
  fixed 0.25 m mosaic grid (now adaptive) plus the mis-tagged 19.8 %, but that needs a
  clean paired comparison to confirm rather than assume.
