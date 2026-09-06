"""Forensic analysis of a recorded ``.svlog``.

One run over a file produces a Markdown report plus a before/after waterfall
pair: packet census, session segmentation, per-channel acquisition parameters,
PRI and ping-number gap statistics, ``src``-vs-``channel_number`` consistency,
and the FBR altitude distribution. With more than one input it also writes the
cross-file comparison table.

    python3 -m blueboat_gcs.analysis.svlog_forensics [--out DIR] LOG...

Design constraints, each measured rather than assumed:

* **Read-only.** Recorded ``.svlog`` files are primary field data (CLAUDE.md
  NC #6 / root CM-7). Inputs are read whole and never reopened for writing, and
  the output directory is refused if it resolves anywhere inside a tree that
  holds ``.svlog`` files.
* **Side identity comes from the packet** (NC #1 / CM-5) — ``core.svlog.side_of``,
  i.e. ``channel_number`` with the sign of ``transducer_heading_deg`` as the
  fallback. The ``src`` byte is a *metric* here, never an input: two logs in the
  corpus carry ``channel_number = 255`` on every packet and are identified by the
  fallback alone.
* **PRI comes from ``timestamp_ms`` sorted per channel**, never from the file
  order and never from ``core.svlog.load_svlog``. Sixteen of the eighteen corpus
  logs have non-monotonic stamps in file order (up to 5537 inversions) because
  the writer batches by channel; and ``load_svlog``'s clock is the synthetic
  20 ms ``NS_PER_TICK``, so timing derived from it would be circular.
* **Ping-number gaps are counted per session segment.** The counter jumps across
  a session boundary: on the Cerulean demo, whole-file counting reports 65 %
  missing where the two segments are individually clean.
* **The packet census enumerates every id seen**, never a whitelist — id 0
  occurs in four corpus logs as well-formed zero-payload frames.

The parser is not re-implemented: ``core/svlog.py`` owns ``walk_packets``,
``decode_os_mono_profile``, ``side_of``, ``estimate_counter_offset`` and the
processing chain, and this module reads them.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import struct
import sys
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from ..core.svlog import (DEVICE_ID_PORT, DEVICE_ID_STBD, JSON_WRAPPER_ID,
                          MAVLINK_WRAPPER_ID, OMNISCAN_STATUS_ID,
                          OS_MONO_PROFILE_ID, FBRTracker,
                          decode_os_mono_profile, decode_session_header,
                          detect_fbr_slant_m, estimate_counter_offset,
                          project_side, scale_to_db, side_of, walk_packets)

#: Across-track columns in the rendered waterfalls, matching
#: ``mosaic.waterfall_columns`` in the shipped GCS config.
WATERFALL_COLUMNS = 800

#: Rendered rows are capped and strided so the image stays viewable on a
#: 19 000-ping log. The stride is reported so the picture is never mistaken
#: for the full record.
MAX_WATERFALL_ROWS = 3000

#: Plain percentile stretch for the raw forensic before/after waterfalls.
#: This is the legacy 2-98 % fallback only — the app's PNGs now go through
#: the display model (``core.display_model``: TL + seabed curve + transfer);
#: forensics stays raw on purpose (see ``_to_png8``).
CONTRAST_PERCENTILES = (2.0, 98.0)

#: Known packet ids, for annotation only. An id absent from this table is
#: reported as "unknown", never dropped.
PACKET_ID_NAMES: Dict[int, str] = {
    0: "empty frame (zero-payload)",
    JSON_WRAPPER_ID: "session metadata (JSON)",
    12: "SonarView view config",
    MAVLINK_WRAPPER_ID: "mavlink wrapper",
    OMNISCAN_STATUS_ID: "Omniscan status (~0.9 Hz per device)",
    OS_MONO_PROFILE_ID: "OS_MONO_PROFILE",
}

SIDE_NAMES = {0: "port", 1: "starboard"}
EXPECTED_SRC = {0: DEVICE_ID_PORT, 1: DEVICE_ID_STBD}


class SvlogForensicsError(Exception):
    """A file that cannot be analysed at all (empty, truncated, not a svlog)."""


# ---------------------------------------------------------------------------
# Frame iteration
# ---------------------------------------------------------------------------
def iter_frames(data: bytes) -> Iterator[Tuple[int, bytes]]:
    """Yield ``(byte offset, framed packet)`` in file order.

    ``walk_packets`` stays the authoritative parser; the offset is recovered by
    searching forward from the end of the previous frame, which is exact because
    packets are yielded in order and the cursor never moves backwards.
    """
    cursor = 0
    for pkt in walk_packets(data):
        off = data.find(pkt, cursor)
        if off < 0:                       # unreachable in practice; stay honest
            return
        yield off, pkt
        cursor = off + len(pkt)


def packet_id(pkt: bytes) -> int:
    return int(struct.unpack_from("<H", pkt, 4)[0])


# ---------------------------------------------------------------------------
# Result records
# ---------------------------------------------------------------------------
@dataclass
class Census:
    """Packet population of the whole file."""

    file_bytes: int = 0
    frames: int = 0
    framed_bytes: int = 0
    trailing_bytes: int = 0             # bytes after the last complete frame
    unframed_bytes: int = 0             # junk skipped + trailing
    counts: Counter = field(default_factory=Counter)
    id_bytes: Counter = field(default_factory=Counter)

    @property
    def truncated_tail(self) -> bool:
        return self.trailing_bytes > 0


@dataclass
class ChannelStats:
    """One channel inside one session segment."""

    channel: int
    n: int = 0
    distinct: int = 0
    pn_min: int = 0
    pn_max: int = 0
    pri_median_ms: float = 0.0
    pri_p95_ms: float = 0.0
    pri_max_ms: float = 0.0

    @property
    def span(self) -> int:
        return self.pn_max - self.pn_min + 1 if self.n else 0

    @property
    def missing(self) -> int:
        return max(self.span - self.distinct, 0)

    @property
    def missing_pct(self) -> float:
        return 100.0 * self.missing / self.span if self.span else 0.0

    @property
    def rate_hz(self) -> float:
        return 1000.0 / self.pri_median_ms if self.pri_median_ms > 0 else 0.0


@dataclass
class Segment:
    """One recording session: everything between two id-10 headers."""

    index: int
    byte_offset: int
    wall_clock: Optional[str] = None
    session_id: Optional[str] = None
    devices: Optional[str] = None
    profiles: int = 0
    ts_min: Optional[int] = None
    ts_max: Optional[int] = None
    channels: Dict[int, ChannelStats] = field(default_factory=dict)
    counter_offset: int = 0
    counter_confidence: float = 0.0

    @property
    def duration_s(self) -> float:
        if self.ts_min is None or self.ts_max is None:
            return 0.0
        return (self.ts_max - self.ts_min) / 1000.0


@dataclass
class ParamStat:
    """One acquisition parameter on one channel, over the whole file.

    ``transitions`` — changes between *consecutive* pings — is reported next to
    the value histogram because the two answer different questions. Auto-gain
    takes four or five distinct ``gain_index`` values on most field logs while
    changing on under 1 % of pings; a range change mid-survey takes two values
    and transitions once. A histogram alone cannot tell those apart.
    """

    name: str
    counts: Counter
    transitions: int = 0
    pings: int = 0

    @property
    def mode(self):
        return self.counts.most_common(1)[0][0] if self.counts else None

    @property
    def changed(self) -> bool:
        return len(self.counts) > 1

    @property
    def transition_pct(self) -> float:
        return 100.0 * self.transitions / self.pings if self.pings else 0.0

    def summary(self) -> str:
        if not self.counts:
            return "-"
        if not self.changed:
            return _fmt_value(self.mode)
        top = self.counts.most_common(3)
        parts = [f"{_fmt_value(v)}x{n}" for v, n in top]
        if len(self.counts) > 3:
            parts.append(f"+{len(self.counts) - 3} more")
        return (f"**{_fmt_value(self.mode)}** ({', '.join(parts)}; "
                f"{self.transitions} transition(s), "
                f"{self.transition_pct:.1f} % of pings)")


@dataclass
class SrcConsistency:
    """``src`` tag versus the packet's own side identity."""

    profiles: int = 0
    mismatches: int = 0
    longest_run: int = 0
    channel_values: Counter = field(default_factory=Counter)

    @property
    def pct(self) -> float:
        return 100.0 * self.mismatches / self.profiles if self.profiles else 0.0

    @property
    def heading_fallback_pct(self) -> float:
        """Share of packets whose side came from the heading, not channel_number."""
        if not self.profiles:
            return 0.0
        exotic = sum(n for v, n in self.channel_values.items() if v not in (0, 1))
        return 100.0 * exotic / self.profiles


@dataclass
class FbrStats:
    """Raw per-ping bottom detections, and the tracker's view of them."""

    n: int = 0
    detected: int = 0
    min_m: float = 0.0
    max_m: float = 0.0
    p10_m: float = 0.0
    p50_m: float = 0.0
    p90_m: float = 0.0
    bottom_sample: Optional[int] = None
    bottom_sample_modal: Optional[int] = None   # restricted to the modal range
    num_results: int = 0
    groups: int = 0
    locked_groups: int = 0

    @property
    def detected_pct(self) -> float:
        return 100.0 * self.detected / self.n if self.n else 0.0

    @property
    def locked_pct(self) -> float:
        return 100.0 * self.locked_groups / self.groups if self.groups else 0.0


@dataclass
class Forensics:
    """Everything one file yielded."""

    path: Path
    sha256: str
    census: Census
    segments: List[Segment]
    params: Dict[int, Dict[str, ParamStat]]
    src: SrcConsistency
    fbr: FbrStats
    non_monotonic: int = 0
    counter_offset: int = 0
    counter_confidence: float = 0.0
    waterfall_stride: int = 1
    images: Dict[str, Path] = field(default_factory=dict)

    # -- convenience accessors used by the comparison table and the tests ----
    @property
    def profiles(self) -> int:
        return self.census.counts.get(OS_MONO_PROFILE_ID, 0)

    @property
    def acquisition_s(self) -> float:
        """Time the sonar was actually pinging: segments summed, gaps excluded."""
        return sum(s.duration_s for s in self.segments)

    @property
    def span_s(self) -> float:
        """First to last ping, gaps included."""
        stamps = [s for seg in self.segments
                  for s in (seg.ts_min, seg.ts_max) if s is not None]
        return (max(stamps) - min(stamps)) / 1000.0 if stamps else 0.0

    @property
    def gaps_s(self) -> List[float]:
        out = []
        for a, b in zip(self.segments, self.segments[1:]):
            if a.ts_max is not None and b.ts_min is not None:
                out.append((b.ts_min - a.ts_max) / 1000.0)
        return out

    def param(self, name: str):
        """Modal value of ``name`` over whichever channel carries it."""
        for by_name in self.params.values():
            stat = by_name.get(name)
            if stat is not None and stat.counts:
                return stat.mode
        return None

    @property
    def mm_per_sample(self) -> float:
        length, n = self.param("length_mm"), self.param("num_results")
        return length / n if length and n else 0.0

    @property
    def pri_median_ms(self) -> float:
        vals = [c.pri_median_ms for s in self.segments
                for c in s.channels.values() if c.pri_median_ms > 0]
        return float(np.median(vals)) if vals else 0.0

    @property
    def missing_pct(self) -> float:
        """Worst per-channel, per-segment missing share — the headline loss."""
        vals = [c.missing_pct for s in self.segments for c in s.channels.values()]
        return max(vals) if vals else 0.0

    @property
    def missing_total(self) -> int:
        return sum(c.missing for s in self.segments for c in s.channels.values())


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
@dataclass(slots=True)
class _Head:
    """One decoded profile header, kept for the second pass."""

    index: int
    segment: int
    side: int
    pn: int
    ts: int
    start_mm: int
    length_mm: int
    num_results: int
    min_db: float
    max_db: float


def _percentile(values: Sequence[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def _fmt_value(v) -> str:
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)


def analyse(path: Path, *, render: bool = True,
            out_dir: Optional[Path] = None) -> Forensics:
    """Analyse one ``.svlog``. The file is read once, never written."""
    path = Path(path)
    if not path.is_file():
        raise SvlogForensicsError(f"{path}: not a file")
    data = path.read_bytes()
    if not data:
        raise SvlogForensicsError(f"{path.name}: file is empty")
    sha = hashlib.sha256(data).hexdigest()

    census = Census(file_bytes=len(data))
    segments: List[Segment] = []
    heads: List[_Head] = []
    params: Dict[int, Dict[str, Counter]] = {}
    prev_param: Dict[int, Dict[str, object]] = {}
    transitions: Dict[int, Counter] = {}
    src = SrcConsistency()

    seg_index = -1
    run = 0
    last_end = 0
    prev_ts: Optional[int] = None
    non_monotonic = 0

    # ---- pass 1: headers only. No pwr decode, so this is cheap. ------------
    packets = list(walk_packets(data))
    if not packets:
        raise SvlogForensicsError(
            f"{path.name}: no framed packets found — not a .svlog, or the "
            f"'BR' framing is corrupt ({len(data)} bytes read)")

    for off, pkt in iter_frames(data):
        last_end = off + len(pkt)
        pid = packet_id(pkt)
        census.frames += 1
        census.framed_bytes += len(pkt)
        census.counts[pid] += 1
        census.id_bytes[pid] += len(pkt)

        if pid == JSON_WRAPPER_ID:
            seg_index += 1
            hdr = decode_session_header(pkt[8:-2])
            segments.append(Segment(index=seg_index, byte_offset=off,
                                    wall_clock=hdr["wall_clock"],
                                    session_id=hdr["session_id"],
                                    devices=hdr["devices"]))
            continue
        if pid != OS_MONO_PROFILE_ID:
            continue

        try:
            head = struct.unpack_from("<IIIIIHHHBBffffff", pkt, 8)
        except struct.error:
            continue
        (pn, start_mm, length_mm, ts, _ping_hz, gain_index, num_results,
         sos_dmps, channel_number, _res, pulse_s, _ag, max_db, min_db,
         heading, _vh) = head
        if len(pkt) < 8 + struct.calcsize("<IIIIIHHHBBffffff") + 2 * num_results:
            continue                       # truncated payload; not a profile

        # The side rule itself lives in core.svlog.side_of and is called, not
        # restated here: it encodes NC #1 / CM-5, and a second copy of it in
        # this module is exactly the drift that rule exists to prevent.
        ch = side_of({"channel_number": channel_number,
                      "transducer_heading_deg": heading})

        if seg_index < 0:                  # profiles before any session header
            seg_index = 0
            segments.append(Segment(index=0, byte_offset=off))
        seg = segments[seg_index]
        seg.profiles += 1
        seg.ts_min = ts if seg.ts_min is None else min(seg.ts_min, ts)
        seg.ts_max = ts if seg.ts_max is None else max(seg.ts_max, ts)

        if prev_ts is not None and ts < prev_ts:
            non_monotonic += 1
        prev_ts = ts

        src.profiles += 1
        src.channel_values[channel_number] += 1
        if pkt[6] != EXPECTED_SRC[ch]:
            src.mismatches += 1
            run += 1
            src.longest_run = max(src.longest_run, run)
        else:
            run = 0

        by_name = params.setdefault(ch, {})
        prev_by_name = prev_param.setdefault(ch, {})
        for name, value in (("start_mm", start_mm), ("length_mm", length_mm),
                            ("num_results", num_results),
                            ("gain_index", gain_index),
                            ("pulse_duration_us", round(pulse_s * 1e6, 3)),
                            ("sos_dmps", sos_dmps)):
            by_name.setdefault(name, Counter())[value] += 1
            if name in prev_by_name and prev_by_name[name] != value:
                transitions.setdefault(ch, Counter())[name] += 1
            prev_by_name[name] = value

        heads.append(_Head(len(heads), seg_index, ch, pn, ts, start_mm,
                           length_mm, num_results, min_db, max_db))

    census.trailing_bytes = len(data) - last_end
    census.unframed_bytes = len(data) - census.framed_bytes

    if not heads:
        raise SvlogForensicsError(
            f"{path.name}: {census.frames} framed packets but no decodable "
            f"OS_MONO_PROFILE (id {OS_MONO_PROFILE_ID}) — nothing to analyse")

    # ---- per-segment timing, gaps and counter offset -----------------------
    for seg in segments:
        seg_heads = [h for h in heads if h.segment == seg.index]
        for ch in sorted({h.side for h in seg_heads}):
            rows = [h for h in seg_heads if h.side == ch]
            pns = [h.pn for h in rows]
            stamps = np.sort(np.asarray([h.ts for h in rows], dtype=np.float64))
            deltas = np.diff(stamps)
            deltas = deltas[deltas > 0]
            seg.channels[ch] = ChannelStats(
                channel=ch, n=len(rows), distinct=len(set(pns)),
                pn_min=min(pns), pn_max=max(pns),
                pri_median_ms=float(np.median(deltas)) if deltas.size else 0.0,
                pri_p95_ms=float(np.percentile(deltas, 95)) if deltas.size else 0.0,
                pri_max_ms=float(deltas.max()) if deltas.size else 0.0)
        seg.counter_offset, seg.counter_confidence = estimate_counter_offset(
            [(h.side, h.pn, h.ts) for h in seg_heads])

    offset, confidence = estimate_counter_offset(
        [(h.side, h.pn, h.ts) for h in heads])

    param_stats = {
        ch: {n: ParamStat(n, c,
                          transitions.get(ch, Counter()).get(n, 0),
                          sum(c.values()))
             for n, c in by_name.items()}
        for ch, by_name in params.items()}

    result = Forensics(path=path, sha256=sha, census=census, segments=segments,
                       params=param_stats, src=src, fbr=FbrStats(),
                       non_monotonic=non_monotonic, counter_offset=offset,
                       counter_confidence=confidence)

    # ---- pass 2: bottom detection over every ping --------------------------
    fbr_per_head, bottoms = _detect_bottoms(packets, heads)
    detections = [v for v in fbr_per_head if v is not None]
    result.fbr = FbrStats(
        n=len(heads), detected=len(detections),
        num_results=int(result.param("num_results") or 0))
    if detections:
        result.fbr.min_m = float(min(detections))
        result.fbr.max_m = float(max(detections))
        result.fbr.p10_m = _percentile(detections, 10)
        result.fbr.p50_m = _percentile(detections, 50)
        result.fbr.p90_m = _percentile(detections, 90)
    every = [v for vals in bottoms.values() for v in vals]
    if every:
        result.fbr.bottom_sample = int(np.median(every))
        modal_len = result.param("length_mm")
        if modal_len in bottoms and len(bottoms) > 1:
            result.fbr.bottom_sample_modal = int(np.median(bottoms[modal_len]))

    # ---- grouping, tracker, and the waterfall pair -------------------------
    groups = _group_heads(heads, offset)
    altitudes, locked = _track_altitudes(groups, fbr_per_head)
    result.fbr.groups = len(groups)
    result.fbr.locked_groups = sum(1 for v in locked if v)

    if render and out_dir is not None:
        stride = max(1, -(-len(groups) // MAX_WATERFALL_ROWS))
        result.waterfall_stride = stride
        result.images = _render_waterfalls(packets, heads, groups, altitudes,
                                           stride, out_dir)
    return result


def _decode_profiles(packets: Sequence[bytes],
                     heads: Sequence[_Head]) -> Iterator[Tuple[_Head, np.ndarray]]:
    """Yield ``(head, intensity_db)`` for each profile, in head order.

    ``heads`` was built from the same packet list in the same order, so walking
    both together needs no index bookkeeping beyond skipping non-profile frames.
    """
    it = iter(heads)
    head = next(it, None)
    for pkt in packets:
        if head is None:
            return
        if packet_id(pkt) != OS_MONO_PROFILE_ID:
            continue
        try:
            d = decode_os_mono_profile(pkt[8:-2])
        except ValueError:
            continue                       # skipped in pass 1 too
        yield head, scale_to_db(d["pwr"], d["min_pwr_db"], d["max_pwr_db"])
        head = next(it, None)


def _detect_bottoms(packets: Sequence[bytes], heads: Sequence[_Head]
                    ) -> Tuple[List[Optional[float]], Dict[int, List[int]]]:
    """Raw per-ping FBR slant range, and the sample index it landed on.

    Bottom samples are bucketed by ``length_mm``: the index is a *sample* index,
    so on a file that changes range mid-survey the whole-file median blends two
    incomparable populations (on ``diffDepthCompensation.svlog``, 280 at 20 m
    and 120 at 35 m). Both are reported.
    """
    fbr: List[Optional[float]] = [None] * len(heads)
    bottoms: Dict[int, List[int]] = {}
    for head, db in _decode_profiles(packets, heads):
        alt = detect_fbr_slant_m(db, head.start_mm, head.length_mm,
                                 head.num_results)
        fbr[head.index] = alt
        if alt is not None:
            mm_per_sample = head.length_mm / max(head.num_results - 1, 1)
            bottoms.setdefault(head.length_mm, []).append(
                int(round((alt * 1000.0 - head.start_mm) / mm_per_sample)))
    return fbr, bottoms


def _group_heads(heads: Sequence[_Head], offset: int
                 ) -> List[Tuple[int, Dict[int, _Head]]]:
    """Rows keyed by the offset-normalised ping counter (NC #2 / CM-6).

    The two devices number independently — 0, -1 and +60 all occur in the field
    corpus — so the raw counter is not a shared key. Grouping is per segment, so
    a counter reset at a session boundary cannot merge two sessions' pings.
    """
    order: List[Tuple[int, int]] = []
    groups: Dict[Tuple[int, int], Dict[int, _Head]] = {}
    for h in heads:
        key = (h.segment, h.pn if h.side == 0 else h.pn + offset)
        g = groups.get(key)
        if g is None:
            g = groups[key] = {}
            order.append(key)
        g[h.side] = h
    return [(k[1], groups[k]) for k in order]


def _track_altitudes(groups: Sequence[Tuple[int, Dict[int, _Head]]],
                     fbr: Sequence[Optional[float]]
                     ) -> Tuple[List[float], List[bool]]:
    """Feed the real ``FBRTracker`` in row order -> per-row altitude and lock."""
    tracker = FBRTracker()
    altitudes: List[float] = []
    locked: List[bool] = []
    for _key, members in groups:
        port = fbr[members[0].index] if 0 in members else None
        stbd = fbr[members[1].index] if 1 in members else None
        alt = tracker.update(port, stbd)
        altitudes.append(0.0 if alt is None else float(alt))
        locked.append(bool(tracker.locked))
    return altitudes, locked


# ---------------------------------------------------------------------------
# Waterfall rendering
# ---------------------------------------------------------------------------
def _to_png8(img: np.ndarray) -> np.ndarray:
    """Plain 2-98 % display stretch for the forensic before/after pair.

    Deliberately *raw*: this diagnostic shows the untouched slant vs
    corrected-ground geometry (and the pre-TVG range falloff) as-is, so it
    does NOT apply the app's seabed-referenced EGN + nadir-aware window
    (``core.display_model`` / ``SeabedImage.to_png8``). Do not "fix" it to match
    the app — the raw stretch is the point of a forensics view."""
    finite = np.isfinite(img)
    if not finite.any():
        return np.zeros(img.shape, np.uint8)
    lo, hi = np.percentile(img[finite], CONTRAST_PERCENTILES)
    hi = max(hi, lo + 1e-6)
    out = np.zeros(img.shape, np.float32)
    out[finite] = np.clip((img[finite] - lo) / (hi - lo), 0, 1)
    return (out * 255).astype(np.uint8)


def _place(row: np.ndarray, y: np.ndarray, db: np.ndarray, half: float) -> None:
    """Port (+y) on the LEFT, matching ``core.waterfall_service``."""
    if half <= 0 or y.size == 0:
        return
    col = np.clip(((half - y) / (2.0 * half) * (WATERFALL_COLUMNS - 1)
                   ).astype(np.int32), 0, WATERFALL_COLUMNS - 1)
    row[col] = db


def _render_waterfalls(packets: Sequence[bytes], heads: Sequence[_Head],
                       groups: Sequence[Tuple[int, Dict[int, _Head]]],
                       altitudes: Sequence[float], stride: int,
                       out_dir: Path) -> Dict[str, Path]:
    """The before/after pair: raw slant range vs corrected ground range."""
    # Both sides of one row must land on the same image line, so the mapping is
    # built per group and then looked up per profile.
    row_of: Dict[int, int] = {}
    alt_of: Dict[int, float] = {}
    for row_i in range(0, len(groups), stride):
        line = row_i // stride
        for h in groups[row_i][1].values():
            row_of[h.index] = line
            alt_of[h.index] = altitudes[row_i]

    n_rows = -(-len(groups) // stride)
    slant = np.full((n_rows, WATERFALL_COLUMNS), np.nan, np.float32)
    ground = np.full((n_rows, WATERFALL_COLUMNS), np.nan, np.float32)

    for head, db in _decode_profiles(packets, heads):
        line = row_of.get(head.index)
        if line is None:
            continue
        sign = 1.0 if head.side == 0 else -1.0
        alt = alt_of[head.index]
        # Slant: no correction at all — the truly raw geometry.
        y_s, db_s = project_side(db, head.start_mm, head.length_mm,
                                 head.num_results, 0.0, 0.0, sign)
        _place(slant[line], y_s, db_s, head.length_mm / 1000.0)
        # Ground: the tracked altitude, water column removed.
        y_g, db_g = project_side(db, head.start_mm, head.length_mm,
                                 head.num_results, alt, 0.0, sign)
        _place(ground[line], y_g, db_g, head.length_mm / 1000.0)

    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {}
    for name, img in (("waterfall_slant.png", slant),
                      ("waterfall_ground.png", ground)):
        dest = out_dir / name
        cv2.imwrite(str(dest), _to_png8(img))
        paths[name.replace(".png", "")] = dest
    return paths


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
def _census_table(r: Forensics) -> List[str]:
    span = r.span_s
    acq = r.acquisition_s
    out = ["| packet id | name | count | bytes | % bytes | rate over acq. | rate over span |",
           "|---|---|---:|---:|---:|---:|---:|"]
    for pid in sorted(r.census.counts):
        n = r.census.counts[pid]
        b = r.census.id_bytes[pid]
        out.append(
            f"| {pid} | {PACKET_ID_NAMES.get(pid, '**unknown**')} | {n} | {b} | "
            f"{100.0 * b / max(r.census.file_bytes, 1):.1f} % | "
            f"{n / acq if acq else 0:.2f} Hz | {n / span if span else 0:.2f} Hz |")
    return out


def _segment_table(r: Forensics) -> List[str]:
    out = ["| # | byte offset | wall clock | profiles | duration | gap before |",
           "|---:|---:|---|---:|---:|---:|"]
    gaps = [None] + r.gaps_s
    for seg, gap in zip(r.segments, gaps):
        out.append(f"| {seg.index} | {seg.byte_offset} | "
                   f"{seg.wall_clock or '-'} | {seg.profiles} | "
                   f"{seg.duration_s:.1f} s | "
                   f"{'-' if gap is None else f'{gap:.1f} s'} |")
    return out


def _param_table(r: Forensics) -> List[str]:
    names = ["start_mm", "length_mm", "num_results", "gain_index",
             "pulse_duration_us", "sos_dmps"]
    channels = sorted(r.params)
    out = ["| parameter | " + " | ".join(f"channel {c} ({SIDE_NAMES[c]})"
                                         for c in channels) + " |",
           "|---|" + "---|" * len(channels)]
    for name in names:
        cells = []
        for c in channels:
            stat = r.params[c].get(name)
            cells.append(stat.summary() if stat else "-")
        out.append(f"| `{name}` | " + " | ".join(cells) + " |")
    length, n = r.param("length_mm"), r.param("num_results")
    if length and n:
        out.append(f"| *derived* mm/sample | "
                   + " | ".join([f"{length / n:.1f}"] * len(channels)) + " |")
    return out


def _timing_table(r: Forensics) -> List[str]:
    out = ["| segment | channel | pings | distinct | pn span | missing | "
           "PRI median | PRI p95 | PRI max |",
           "|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for seg in r.segments:
        for ch in sorted(seg.channels):
            c = seg.channels[ch]
            out.append(
                f"| {seg.index} | {ch} ({SIDE_NAMES[ch]}) | {c.n} | "
                f"{c.distinct} | {c.span} | {c.missing} ({c.missing_pct:.1f} %) | "
                f"{c.pri_median_ms:.1f} ms ({c.rate_hz:.1f} Hz) | "
                f"{c.pri_p95_ms:.1f} ms | {c.pri_max_ms:.1f} ms |")
    return out


def render_report(r: Forensics, *, generated: Optional[str] = None) -> str:
    """The Markdown report. Everything but the one 'Generated' line is stable."""
    stamp = generated or datetime.now(timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%SZ")
    L: List[str] = []
    L.append(f"# `.svlog` forensics — {r.path.name}")
    L.append("")
    L.append(f"*Generated: {stamp}*")
    L.append("")
    L.append(f"- Source: `{r.path}`")
    L.append(f"- SHA-256: `{r.sha256}`")
    L.append(f"- Size: {r.census.file_bytes} bytes "
             f"({r.census.file_bytes / 1e6:.1f} MB)")
    L.append(f"- Profiles: {r.profiles} · sessions: {len(r.segments)}")
    L.append(f"- Acquisition: {r.acquisition_s:.1f} s "
             f"(file span {r.span_s:.1f} s, "
             f"{len(r.gaps_s)} inter-session gap(s))")
    L.append("")

    warn = []
    if r.census.truncated_tail:
        warn.append(f"{r.census.trailing_bytes} trailing bytes after the last "
                    f"complete frame — the file is truncated.")
    if r.census.unframed_bytes - r.census.trailing_bytes > 0:
        warn.append(f"{r.census.unframed_bytes - r.census.trailing_bytes} "
                    f"bytes skipped between frames.")
    unknown = [p for p in r.census.counts if p not in PACKET_ID_NAMES]
    if unknown:
        warn.append(f"unrecognised packet id(s): {sorted(unknown)}.")
    if warn:
        L.append("> **Warnings.** " + " ".join(warn))
        L.append("")

    L.append("## 1. Packet census")
    L.append("")
    L += _census_table(r)
    L.append("")
    L.append(f"{r.census.frames} framed packets, {r.census.framed_bytes} framed "
             f"bytes, {r.census.unframed_bytes} unframed "
             f"({r.census.trailing_bytes} of them trailing).")
    L.append("")

    L.append("## 2. Sessions")
    L.append("")
    L += _segment_table(r)
    L.append("")
    if len(r.segments) > 1:
        L.append("More than one session header: ping counters restart across "
                 "the boundary, so every gap statistic below is **per segment**. "
                 "A whole-file count would report the counter jump as loss.")
    if r.segments and r.segments[0].devices:
        L.append(f"Devices (session 0): {r.segments[0].devices}")
    L.append("")

    L.append("## 3. Acquisition parameters")
    L.append("")
    L += _param_table(r)
    L.append("")
    changed = sorted({n for by in r.params.values()
                      for n, s in by.items() if s.changed})
    settings = [c for c in changed if c != "gain_index"]
    if settings:
        L.append(f"**Acquisition settings changed mid-file:** "
                 f"{', '.join('`%s`' % c for c in settings)}. The bolded value "
                 f"is the mode, and every single-value summary of this file — "
                 f"including the comparison table — quotes the mode.")
    else:
        L.append("No acquisition-setting changes mid-file.")
    if "gain_index" in changed:
        L.append("")
        L.append("`gain_index` varies because both stacks run auto-gain; the "
                 "transition rate above, not the number of distinct values, is "
                 "what says whether gain is a meaningful contributor to banding.")
    L.append("")

    L.append("## 4. PRI and ping-number gaps")
    L.append("")
    L += _timing_table(r)
    L.append("")
    L.append(f"Ping counter offset (port − starboard): **{r.counter_offset}** "
             f"at {r.counter_confidence:.2f} confidence"
             + (" — single-sided log, nothing to align."
                if r.counter_confidence == 0.0 else "."))
    L.append("")
    L.append(f"`timestamp_ms` is non-monotonic in file order on "
             f"**{r.non_monotonic}** of {r.profiles} profiles "
             f"({100.0 * r.non_monotonic / max(r.profiles, 1):.1f} %); the "
             f"writer batches by channel, so every PRI above is computed on "
             f"stamps sorted **per channel**.")
    L.append("")

    L.append("## 5. `src` vs `channel_number`")
    L.append("")
    L.append(f"**{r.src.mismatches} of {r.src.profiles} profiles "
             f"({r.src.pct:.1f} %)** carry a frame `src` that disagrees with the "
             f"packet's own side identity. Longest consecutive run: "
             f"{r.src.longest_run}.")
    L.append("")
    L.append("| `channel_number` | packets | share |")
    L.append("|---|---:|---:|")
    for value in sorted(r.src.channel_values):
        n = r.src.channel_values[value]
        label = SIDE_NAMES.get(value, f"{value} (outside 0/1)")
        L.append(f"| {value} — {label} | {n} | "
                 f"{100.0 * n / max(r.src.profiles, 1):.1f} % |")
    L.append("")
    if r.src.heading_fallback_pct > 0:
        L.append(f"{r.src.heading_fallback_pct:.1f} % of packets carry a "
                 f"`channel_number` outside (0, 1); their side comes from the "
                 f"sign of `transducer_heading_deg` (NC #1 / CM-5).")
        L.append("")

    L.append("## 6. Bottom detection (FBR)")
    L.append("")
    L.append(f"- Detections: {r.fbr.detected} of {r.fbr.n} pings "
             f"({r.fbr.detected_pct:.1f} %)")
    if r.fbr.detected:
        L.append(f"- Slant altitude: min {r.fbr.min_m:.2f} m · "
                 f"p10 {r.fbr.p10_m:.2f} · p50 {r.fbr.p50_m:.2f} · "
                 f"p90 {r.fbr.p90_m:.2f} · max {r.fbr.max_m:.2f} m")
    if r.fbr.bottom_sample is not None:
        L.append(f"- Median bottom return at sample "
                 f"**{r.fbr.bottom_sample}/{r.fbr.num_results}**")
    if r.fbr.bottom_sample_modal is not None:
        L.append(f"- ...restricted to the modal `length_mm` "
                 f"({r.param('length_mm')} mm): "
                 f"**{r.fbr.bottom_sample_modal}/{r.fbr.num_results}**. The "
                 f"index is a sample number, so the two are not comparable "
                 f"across a mid-file range change.")
    L.append(f"- Tracker locked on {r.fbr.locked_groups} of {r.fbr.groups} rows "
             f"({r.fbr.locked_pct:.1f} %)")
    L.append("")
    L.append("The percentiles are the *raw per-ping detector*; the lock share is "
             "the `FBRTracker`'s view of the same detections. They are reported "
             "separately because a wide raw spread with a high lock share means "
             "something different from either alone.")
    L.append("")

    if r.images:
        L.append("## 7. Waterfall — raw slant range vs corrected ground range")
        L.append("")
        L.append(f"![slant]({r.images['waterfall_slant'].name}) "
                 f"![ground]({r.images['waterfall_ground'].name})")
        L.append("")
        L.append(f"`waterfall_slant.png` applies no correction and **keeps the "
                 f"water column** — deliberately the truly raw geometry, which is "
                 f"what makes the nadir noise visible here rather than masked; "
                 f"`waterfall_ground.png` uses the tracked altitude and drops the "
                 f"water column. (The GCS display also blanks the first 0.75 m of "
                 f"slant range in every mode — the transmit ringing — which this "
                 f"pair does not, so the two images stay a before/after of the "
                 f"correction alone.) "
                 f"{WATERFALL_COLUMNS} columns, port (+y) on the "
                 f"left, row stride {r.waterfall_stride}, 2–98 % display "
                 f"stretch.")
        L.append("")

    L.append("---")
    L.append("")
    L.append("Input opened read-only; recorded `.svlog` files are primary field "
             "data (CLAUDE.md NC #6 / root CM-7).")
    return "\n".join(L) + "\n"


def render_comparison(results: Sequence[Forensics],
                      *, generated: Optional[str] = None) -> str:
    """The cross-file table — the *Measured acquisition settings* shape."""
    stamp = generated or datetime.now(timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%SZ")
    names = [r.path.name for r in results]
    L = ["# `.svlog` forensics — comparison", "", f"*Generated: {stamp}*", "",
         "| | " + " | ".join(names) + " |", "|---|" + "---|" * len(results)]

    def row(label, fn):
        L.append(f"| {label} | " + " | ".join(fn(r) for r in results) + " |")

    row("Range", lambda r: f"{(r.param('length_mm') or 0) / 1000:.1f} m")
    row("Samples/ping", lambda r: str(r.param("num_results") or "-"))
    row("Range sampling", lambda r: f"{r.mm_per_sample:.0f} mm")
    row("Ping interval", lambda r: (f"{r.pri_median_ms:.0f} ms "
                                    f"({1000 / r.pri_median_ms:.1f} Hz)"
                                    if r.pri_median_ms else "-"))
    row("Transmit pulse", lambda r: f"{r.param('pulse_duration_us') or 0:.0f} µs")
    row("Bottom at sample", lambda r: (f"{r.fbr.bottom_sample}/{r.fbr.num_results}"
                                       if r.fbr.bottom_sample is not None else "-"))
    row("Missing ping numbers", lambda r: f"{r.missing_pct:.1f} %")
    row("Wrong-`src` packets", lambda r: f"{r.src.pct:.1f} %")
    row("Sessions", lambda r: str(len(r.segments)))
    row("Profiles", lambda r: str(r.profiles))
    L.append("")
    L.append("Range, samples, sampling and pulse are **modal** values: a file "
             "that changes its range mid-survey is flagged in its own report.")
    L.append("")
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _collect_inputs(paths: Sequence[str]) -> List[Path]:
    out: List[Path] = []
    for raw in paths:
        p = Path(raw)
        if p.is_dir():
            out.extend(sorted(p.rglob("*.svlog")))
        else:
            out.append(p)
    return out


def _check_output_is_outside(out: Path, inputs: Sequence[Path]) -> None:
    """NC #6 / CM-7: never write inside the tree holding primary field data.

    The test is against the **common root** of every input, not against each
    input's own parent, and it refuses in both directions:

    * ``out`` inside the root — the obvious case, and the one that catches a
      sibling directory such as ``<corpus>/reports`` when the logs themselves
      sit in ``<corpus>/Shiraishi/Tires/``;
    * ``out`` an ancestor of the root — the same violation from the other end,
      since ``--out <corpus>`` with an input at ``<corpus>/Tires/x.svlog``
      still deposits reports inside the corpus.

    Inputs with no common ancestor (separate mounts) are checked individually.

    A third rule catches what the path arithmetic cannot: with a single deep
    input the common root is that file's own directory, so a sibling of a
    *grandparent* — ``<corpus>/reports`` for an input in
    ``<corpus>/Shiraishi/Tires/`` — looks unrelated to it. Walking ``out``'s
    ancestors and refusing any that directly contains a ``.svlog`` closes that,
    because the corpus root does.
    """
    out = out.resolve()
    parents = [p.resolve().parent for p in inputs]
    try:
        roots = [Path(os.path.commonpath([str(p) for p in parents]))]
    except ValueError:                      # different drives / no common root
        roots = parents
    for root in roots + parents:
        if out == root or root in out.parents or out in root.parents:
            raise SvlogForensicsError(
                f"refusing to write into {out} — it holds, or sits inside, "
                f"{root}, which holds the .svlog input(s). Recorded .svlog "
                f"files are primary field data (NC #6 / CM-7); choose an "
                f"--out elsewhere.")
    for ancestor in (out, *out.parents):
        if any(ancestor.glob("*.svlog")):
            raise SvlogForensicsError(
                f"refusing to write into {out} — {ancestor} holds .svlog "
                f"files, so the output would land inside a tree carrying "
                f"primary field data (NC #6 / CM-7). Choose an --out "
                f"elsewhere.")


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        prog="svlog_forensics",
        description="Packet census, timing, side-identity and bottom-detection "
                    "forensics for recorded .svlog files. Read-only.")
    ap.add_argument("paths", nargs="+", metavar="LOG",
                    help=".svlog files, or directories to search recursively")
    ap.add_argument("--out", default="svlog_forensics", type=Path,
                    help="output directory (default: ./svlog_forensics)")
    ap.add_argument("--no-images", action="store_true",
                    help="skip the waterfall pair (much faster)")
    ap.add_argument("--compare", action="store_true",
                    help="also write comparison.md across all inputs")
    args = ap.parse_args(argv)

    try:
        inputs = _collect_inputs(args.paths)
        if not inputs:
            raise SvlogForensicsError("no .svlog files matched")
        _check_output_is_outside(args.out, inputs)
    except SvlogForensicsError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    args.out.mkdir(parents=True, exist_ok=True)
    results: List[Forensics] = []
    failures = 0
    for path in inputs:
        stem = path.stem
        target = args.out / stem
        try:
            r = analyse(path, render=not args.no_images, out_dir=target)
        except SvlogForensicsError as exc:
            print(f"error: {exc}", file=sys.stderr)
            failures += 1
            continue
        target.mkdir(parents=True, exist_ok=True)
        report = target / "report.md"
        report.write_text(render_report(r), encoding="utf-8")
        results.append(r)
        print(f"{path.name}: {r.profiles} profiles, {len(r.segments)} session(s), "
              f"wrong-src {r.src.pct:.1f} %, worst missing {r.missing_pct:.1f} % "
              f"-> {report}")

    if args.compare and len(results) > 1:
        dest = args.out / "comparison.md"
        dest.write_text(render_comparison(results), encoding="utf-8")
        print(f"comparison -> {dest}")

    if not results:
        return 2
    return 1 if failures else 0


if __name__ == "__main__":                                  # pragma: no cover
    raise SystemExit(main())
