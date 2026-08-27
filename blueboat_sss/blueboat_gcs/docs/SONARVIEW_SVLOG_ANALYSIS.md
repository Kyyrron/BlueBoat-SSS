# SonarView vs. our GCS — measured differences, root causes, fixes

Evidence base (all numbers measured, nothing assumed):

| File | Recorded by | Content |
|---|---|---|
| `reflection_evidence.svlog` | **our** stack | 16.8 MB, 5250 profiles, dual Omniscan |
| `diffDepthCompensation.svlog` | **SonarView** | 9.8 MB, 4900 profiles, dual Omniscan, same boat |
| `harbor_scan_combined.svlog` | Cerulean demo | 18 MB, 4278 profiles, single Omniscan |

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

`ping_number` gaps in our file: **356 missing on channel 0, 192 on channel 1**
(~8 %), with PRI stalls up to **548 ms**. SonarView's recording on the same hardware
has **zero** missing ping numbers and a PRI of 50.0 ms (p95 51 ms).

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
| Bottom return at sample | **49 / 600** | 270 / 600 | 137 / 1200 |

The 80 m range is the cause of four separate symptoms at once: 4× coarser sampling,
half the ping rate, a 3× longer pulse (16 cm vs 5 cm range resolution), and a bottom
return so close to the transmit ringing that **bottom detection stops working** —
which is precisely why SonarView shows `Detected N/A` on our file no matter which
source is selected.

**Fix (implemented).** `launch/SSS_processing_launch.py` default range is now
**20000 mm**. Rule: set the range from the water depth (~4× the deepest water
expected), never from the area you hope to cover.

`doppler_enable` in the device options is Cerulean's speed-over-ground (Doppler)
processing; it appears alongside a 16-byte, 0.31 Hz status packet (`id 2194`) that we
do not parse and do not need — it carries no imagery.

Gain: both stacks run auto-gain. SonarView's settles at `gain_index` 4 for 96 % of
pings; ours moved across 4–7 but changed on <1 % of pings, so gain is **not** a
significant contributor to the banding.

---

## 5. Depth compensation — what it is, and why theirs looked better

SonarView's "Depth Compensation" is simply **the altitude used for slant-range
correction**, with a selectable source (port sonar / starboard sonar / manual). It is
the same quantity our FBR tracker estimates.

Two questions answered:

* **"Why does the waterfall change when I change it?"** Because the waterfall is *not*
  raw data — in SonarView, as in our app, it is displayed in **corrected ground
  range**. Changing the altitude changes `ground = sqrt(slant² − h²)` and therefore
  every column position. Only `intensity_db` vs slant sample index is truly raw.
* **"Why does Manual / 0 m look better?"** Because with `h = 0` no samples are
  discarded and no warping is applied. For shallow water (`h << R`) ground range ≈
  slant range anyway, so a *wrong* altitude is far more damaging than *no* correction:
  an over-estimated altitude both deletes real samples and compresses the near range.
  With our 80 m logs the altitude estimate wandered between 4.7 m and 45 m
  (p10 4.67, p90 15.23), so "off" genuinely was the better choice on that data.

**Fix (implemented).** A `Depth comp.` selector in the right panel with the same three
options — `Auto (bottom detect)`, `Manual`, `Off (no correction)` — wired to both the
live view and the replay window. Changing it in the replay window re-processes the log,
mirroring SonarView's behaviour.

---

## 6. FBR bootstrap was throwing data away

The old tracker returned `None` until 10 consecutive detections agreed within 0.30 m,
and the caller **dropped every ping** until then — losing the start of every mission
and arbitrary chunks whenever lock was lost.

**Fix (implemented).** `FBRTracker.update` now always returns the best available
altitude (locked → provisional → last known) and exposes `locked` separately, so
quality can be reported without discarding data. `resolve_altitude` falls back to
0.0 (no correction) when nothing has ever been detected. **No ping is ever dropped
for lack of a depth lock.** On `reflection_evidence.svlog`, 593 pings that were
previously discarded are now displayed with a provisional altitude.

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

## 9. Robot-side patches still required

### 9.1 The mis-tagging is written by the processor

`_write_raw_with_src_tag()` tags each packet with the device id implied by **the topic
it arrived on**, not by the packet's own identity:

```python
def _on_port_raw(self, msg):      self._write_raw_with_src_tag(msg, DEVICE_ID_PORT)
def _on_starboard_raw(self, msg): self._write_raw_with_src_tag(msg, DEVICE_ID_STBD)
```

Whenever a packet reaches the wrong topic, the wrong `src` is burned into the `.svlog`
permanently — which is why **SonarView also renders our files with banded artifacts**.
The packet already knows which channel it is: for `OS_MONO_PROFILE` (id 2198),
`channel_number` is payload byte 26, i.e. **byte 34 of the framed packet** (verified
against all 5250 packets in `reflection_evidence.svlog`, zero mismatches).

```python
from .svlog_helper import OS_MONO_PROFILE_ID, DEVICE_ID_PORT, DEVICE_ID_STBD

_CHANNEL_BYTE = 34          # 8-byte frame header + payload offset 26

@staticmethod
def _src_from_packet(raw: bytes, fallback: int) -> int:
    """Device id from the packet itself; the topic is only a fallback."""
    if (len(raw) > _CHANNEL_BYTE
            and int.from_bytes(raw[4:6], "little") == OS_MONO_PROFILE_ID):
        ch = raw[_CHANNEL_BYTE]
        if ch in (0, 1):
            return DEVICE_ID_PORT if ch == 0 else DEVICE_ID_STBD
    return fallback

def _write_raw_with_src_tag(self, msg, fallback_src: int) -> None:
    if not self._svlog.active:
        return
    raw = bytes(bytearray(msg.data))
    try:
        self._svlog.write(retag_packet_src_device_id(
            raw, self._src_from_packet(raw, fallback_src)))
    except ValueError as exc:
        self.get_logger().warn(f"dropping malformed raw packet: {exc}")
```

This alone makes every future `.svlog` correct in **both** our GCS and SonarView.

### 9.2 Live projection and assembly

The live path is projected on the robot, so the mirror must also be fixed there:

```python
# side from the message, not from the subscription it arrived on
side_sign = +1.0 if msg.channel_number == 0 else -1.0
if msg.channel_number not in (0, 1):                    # defensive fallback
    side_sign = -1.0 if msg.transducer_heading_deg > 0 else +1.0
```

and pings should be assembled by `ping_number` instead of paired within 50 ms, with
one-sided pings published rather than dropped, and no ping withheld while the bottom
tracker bootstraps. Until that lands, the GCS reports the symptom (`port/starboard
halves come from different pings`) in the console.

**Recorded `.svlog` files replayed in the GCS are already correct**, because the replay
path does its own routing and assembly.

## 10. What I still need

* A short `.svlog` recorded **after** the range change (20 m) to confirm the ping-rate
  and resolution gains on our own hardware.
* A `.svlog` recorded after the §9.1 retag patch, to confirm 0 % mis-tagging at the
  source.
* Whether SonarView still out-renders us at *identical* range settings once the
  mis-tagging is gone — my current assumption is that the residual gap was mostly the
  fixed 0.25 m mosaic grid (now adaptive) plus the mis-tagged 19.8 %, but that needs a
  clean paired comparison to confirm rather than assume.
