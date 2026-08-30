"""Offline .svlog reading and processing.

Turns a SonarView .svlog (as written by sss_processor_node via
svlog_helper.py) into the same model stream the live application
consumes — SonarPing + RobotState + GPS origin — so the replay window
and the dataset generator reuse the entire existing GUI/service stack
unchanged.

Format layer (ports from the team's ``svlog_to_rosbag.py``, kept
byte-identical in behaviour):

* ``walk_packets``            — BR-framed Cerulean Ping Protocol stream;
* ``decode_os_mono_profile``  — OS_MONO_PROFILE (id 2198) payload,
  ``<IIIIIHHHBBffffff`` head + u16 pwr_results;
* burst-aware synthetic clock — mavlink messages sharing a
  ``time_boot_ms`` share a stamp; any new tick advances by 20 ms
  (``Converter._tick`` semantics);
* NED→ENU position and attitude conversion (REP-103/105, identical
  formulas).

Processing layer (faithful port of the ``sss_processor_node`` pipeline +
``sss_helper.py``, constants copied verbatim — keep in sync with the
robot-side repo if those are ever retuned):

    raw u16 → scale_to_db → ringing-adaptive noise window → FBR
    detection per side → dual-side FBRTracker (independent bootstrap,
    max() fusion) → slant-range correction dropping the water column →
    assembly by offset-normalised ping_number → pose snap from the
    synthesized odom.
"""

from __future__ import annotations

import json
import math
import struct
from collections import Counter, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import (Callable, Deque, Iterator, List, Optional, Sequence,
                    Tuple)

import numpy as np

from ..models.robot_state import RobotState
from ..models.sonar import SonarPing
from ..utils.geodesy import yaw_to_compass_deg

# ---------------------------------------------------------------------------
# Packet constants (svlog_helper.py conventions)
# ---------------------------------------------------------------------------
JSON_WRAPPER_ID = 10
MAVLINK_WRAPPER_ID = 150
OMNISCAN_STATUS_ID = 2194
OS_MONO_PROFILE_ID = 2198
DEVICE_ID_PORT = 1
DEVICE_ID_STBD = 2

#: Fallback clock step, used only where a channel-segment's own ``timestamp_ms``
#: is unusable (see ``load_svlog``). The name and value come from
#: ``svlog_to_rosbag.Converter``, but the two clocks have **deliberately
#: diverged**: the converter ticks *every* packet because its output is a
#: rosbag whose stamps must be monotonic ROS time with mavlink bursts sharing
#: one, feeding the live processor's 50 ms pairing tolerance. This module's
#: output is a replay timeline, which must be the log's own wall clock. The
#: converter is a NC #7 byte-identical mirror and is not edited from here.
NS_PER_TICK = 20_000_000          # 20 ms
_HEAD_FMT = "<IIIIIHHHBBffffff"   # OS_MONO_PROFILE head
_HEAD_SIZE = struct.calcsize(_HEAD_FMT)

#: Omniscan status payload (id 2194): two float32, the ``timestamp_ms`` of a
#: profile on the same channel, three zero bytes, and the channel number.
#: 16 bytes, ~0.9 Hz per device. See ``decode_omniscan_status``.
_STATUS_FMT = "<ffI3xB"
_STATUS_SIZE = struct.calcsize(_STATUS_FMT)

# Processing constants — verbatim from sss_processor_node.py.
NOISE_FLOOR_WINDOW = 20
FBR_THRESHOLD_DELTA_DB = 8.0
WITHIN_PING_PERSISTENCE = 3
RINGING_SEARCH_MAX = 60
RINGING_DROP_DB = 10.0
RINGING_PERSISTENCE = 5
BOOTSTRAP_PINGS = 10
ALTITUDE_AGREEMENT_TOL_M = 0.30
OUTLIER_TOL_M = 1.0
RELOCK_AFTER = 15
TRANSDUCER_SUBMERSION_M = 0.0
TRANSDUCER_Y_OFFSET_PORT_M = 0.0
TRANSDUCER_Y_OFFSET_STBD_M = 0.0
PAIR_TOLERANCE_NS = 50_000_000    # 50 ms processor tolerance


# ---------------------------------------------------------------------------
# Format layer
# ---------------------------------------------------------------------------
def walk_packets(data: bytes) -> Iterator[bytes]:
    """Yield framed packets from a svlog stream, skipping junk bytes."""
    pos, n = 0, len(data)
    while pos < n - 8:
        if data[pos:pos + 2] != b"BR":
            pos += 1
            continue
        plen = struct.unpack_from("<H", data, pos + 2)[0]
        total = 8 + plen + 2
        if pos + total > n:
            return                                  # truncated tail
        yield data[pos:pos + total]
        pos += total


def decode_os_mono_profile(payload: bytes) -> dict:
    """OS_MONO_PROFILE payload -> named fields (raises ValueError)."""
    if len(payload) < _HEAD_SIZE:
        raise ValueError("payload too short")
    (ping_number, start_mm, length_mm, timestamp_ms, ping_hz,
     gain_index, num_results, sos_dmps, channel_number, _res,
     pulse_duration_sec, analog_gain, max_pwr_db, min_pwr_db,
     transducer_heading_deg, vehicle_heading_deg) = struct.unpack(
        _HEAD_FMT, payload[:_HEAD_SIZE])
    expected = _HEAD_SIZE + 2 * num_results
    if len(payload) < expected:
        raise ValueError("payload truncated")
    pwr = np.frombuffer(payload, dtype="<u2", count=num_results,
                        offset=_HEAD_SIZE).astype(np.float32)
    return {"start_mm": start_mm, "length_mm": length_mm,
            "num_results": num_results, "max_pwr_db": max_pwr_db,
            "min_pwr_db": min_pwr_db, "pwr": pwr,
            "timestamp_ms": timestamp_ms, "ping_number": ping_number,
            # Authoritative side identity (see load_svlog): the device's
            # own channel tag, plus the transducer bearing as fallback.
            "channel_number": channel_number, "gain_index": gain_index,
            "transducer_heading_deg": transducer_heading_deg}


def decode_omniscan_status(payload: bytes) -> dict:
    """Omniscan status payload (id 2194) -> named fields (raises ValueError).

    **Established from the bytes** across three reference logs: the 16-byte
    payload is ``<ffI3xB`` — two ``float32``; a ``uint32`` on the **same device
    clock as** ``OS_MONO_PROFILE.timestamp_ms`` (inside that channel's profile
    time range, monotonic, and within half a ping interval of the nearest
    profile — median 11-13 ms, max 25 ms against a 50 ms PRI — so it is
    independently sampled on that clock, not a copy of a profile stamp); three
    zero bytes; and the channel number, which agrees with the frame's ``src``
    tag on every packet seen. One packet per device at ~0.9 Hz (1.11 s median
    interval), so roughly one per 22 pings at 20 Hz. It carries no imagery.

    **Not established:** what the two floats are. They are device-specific,
    span ~45-75 with the pair 17-19 apart, drift by under 1 over a whole log,
    and correlate with nothing acoustic — ``|r| < 0.2`` against the same
    channel's ``max_pwr_db``, ``min_pwr_db``, ``gain_index`` and raw power
    percentiles. They are housekeeping telemetry, not signal statistics; the
    quantity itself is unnamed here rather than guessed at.
    """
    if len(payload) < _STATUS_SIZE:
        raise ValueError("status payload too short")
    f0, f1, timestamp_ms, channel_number = struct.unpack(
        _STATUS_FMT, payload[:_STATUS_SIZE])
    return {"value_0": f0, "value_1": f1, "timestamp_ms": timestamp_ms,
            "channel_number": channel_number}


def decode_session_header(payload: bytes) -> dict:
    """Session-metadata payload (id 10) -> ``{wall_clock, session_id, devices}``.

    Junk, non-JSON or a non-object document yields all-``None`` rather than
    raising: a header that cannot be read still opens a segment, because the
    *packet's presence* is what marks the session boundary.

    ``analysis/svlog_forensics.py`` reads this from here; the parser has one
    home, in the same direction as ``walk_packets`` and ``side_of``.
    """
    try:
        doc = json.loads(payload.decode("utf-8", "replace").rstrip("\x00"))
    except (ValueError, UnicodeDecodeError):
        doc = None
    if not isinstance(doc, dict):
        return {"wall_clock": None, "session_id": None, "devices": None}
    return {"wall_clock": doc.get("timestamp"),
            "session_id": doc.get("session_id"),
            "devices": describe_session_devices(doc.get("session_devices"))}


def describe_session_devices(devices) -> Optional[str]:
    """One line per device: nickname, product, device_id, url.

    ``session_devices`` is a single object on the Cerulean logs and a list on
    others, so both shapes are flattened to the same summary.
    """
    if devices is None:
        return None
    items = devices if isinstance(devices, (list, tuple)) else [devices]
    parts = []
    for item in items:
        if not isinstance(item, dict):
            parts.append(str(item))
            continue
        bits = [str(item[k]) for k in ("nickname", "product_id", "device_id")
                if item.get(k) is not None]
        url = item.get("url")
        parts.append(" ".join(bits) + (f" @ {url}" if url else ""))
    return "; ".join(p for p in parts if p) or None


def side_of(d: dict) -> int:
    """Authoritative side identity for a decoded profile: 0 = port, 1 = stbd.

    The packet's own ``channel_number``, with the transducer bearing as the
    fallback for devices that leave it outside (0, 1) -- two logs in the
    corpus carry 255 on every packet.
    """
    ch = d["channel_number"]
    if ch not in (0, 1):
        ch = 1 if d["transducer_heading_deg"] > 0 else 0
    return ch


def estimate_counter_offset(profiles: Sequence[Tuple[int, int, int]],
                            defer: int = 4) -> Tuple[int, float]:
    """Constant offset between the two devices' ping counters -> (offset, confidence).

    Port of ``sss_helper.PingCounterOffset``, run as a single pass over a
    whole file rather than incrementally.

    The two Omniscan 450 units are independent devices with independent
    ``ping_number`` counters; the offset between them is arbitrary but
    constant for a power-up cycle. Measured over this project's field corpus
    it is 0 on ten logs, -1 on four and +60 on two, so ``ping_number`` cannot
    be used directly as a cross-side assembly key: on a +60 log it pairs a
    port ping with a starboard ping acquired ~1.7 s earlier.

    ``profiles`` is ``(channel, ping_number, timestamp)`` in file order. Each
    ping votes on the counter difference against the opposite side's
    *temporally nearest* ping. Voting against the most recently *seen*
    opposite ping is bimodal -- it splits between the true offset and
    offset +-1 depending on interleave phase -- so a vote is deferred by
    ``defer`` arrivals until both neighbours are available. Confidence is the
    winning share; it exceeds 0.94 on every two-sided log in the corpus.
    """
    recent = {0: deque(maxlen=2 * defer + 1), 1: deque(maxlen=2 * defer + 1)}
    pending: Deque[Tuple[int, int, int]] = deque()
    votes: Counter = Counter()
    for ch, pn, ts in profiles:
        recent[ch].append((ts, pn))
        pending.append((ch, pn, ts))
        if len(pending) <= defer:
            continue
        pch, ppn, pts = pending.popleft()
        opposite = recent[1 - pch]
        if not opposite:
            continue
        _, opn = min(opposite, key=lambda e: abs(e[0] - pts))
        votes[(ppn - opn) if pch == 0 else (opn - ppn)] += 1
    if not votes:
        return 0, 0.0                       # single-sided log; nothing to align
    offset, n = votes.most_common(1)[0]
    return offset, n / sum(votes.values())


def estimate_boot_skew(votes: Sequence[int]) -> Optional[int]:
    """Constant offset [ms] from autopilot ``time_boot_ms`` to sonar ``timestamp_ms``.

    The sonar and the autopilot each stamp from *their own* boot, so the two
    streams in one ``.svlog`` sit on different clocks. Measured across the field
    corpus the offset between them is large and per-file arbitrary — 2 644 886,
    58 363, 56 171 and 57 566 ms on four logs — while the two clocks tick at the
    same rate: their spans agree to 30 ms over 611 s. So one constant per file
    puts both streams on one timeline, and that constant has to be measured, not
    assumed.

    ``votes`` is ``most-recent profile timestamp_ms - time_boot_ms``, one per
    mavlink packet in file order. Because the writer interleaves the two streams
    in acquisition order, each vote is right to within the local packet spacing;
    the median cancels that. Its stability is what licenses a single file-wide
    value: the p1-p99 spread is 50-120 ms over a whole log, and the two segments
    of the Cerulean demo agree to 20 ms.

    Returns ``None`` when there is nothing to measure (no mavlink, or no profile
    before any mavlink packet), which puts the mavlink stream back on the tick.
    """
    if not votes:
        return None
    return int(np.median(np.asarray(votes, dtype=np.int64)))


#: A backwards step larger than this rejects a channel's ``timestamp_ms``.
#: Sized from the corpus rather than from taste. Real logs contain *local*
#: inversions — one file swaps 9 adjacent pings out of ~17 000, each by exactly
#: one 29 ms ping interval, which is the writer emitting a pair out of order and
#: is fully absorbed by the final ``events.sort()``. What genuinely breaks a
#: timeline is a clock *reset*, which moves backwards by seconds or minutes.
#: 1 s sits ~34 ping intervals above the artefact and orders of magnitude below
#: a reset, so it separates the two without straddling either.
STAMP_BACKSTEP_TOLERANCE_MS = 1000


def usable_stamps(stamps: Sequence[int]) -> bool:
    """Is this channel-segment's own ``timestamp_ms`` sequence usable as a clock?

    Tested **per channel per segment**, never over file order. The writer
    batches profiles by channel, so file order is non-monotonic on real logs
    — 145, 125 and 1076 inversions measured on three corpus files — while each
    channel's own sequence is monotonic to within the tolerance above on every
    log in the corpus. A file-order monotonicity test would reject the real
    clock almost everywhere and silently re-synthesize the timeline this
    function exists to protect.
    """
    if len(stamps) < 2:
        return bool(stamps) and stamps[0] > 0
    arr = np.asarray(stamps, dtype=np.int64)
    if (arr <= 0).any():
        return False
    if np.diff(arr).min() < -STAMP_BACKSTEP_TOLERANCE_MS:
        return False
    return bool(arr[-1] > arr[0])


def ned_to_enu_xyz(x_n: float, y_e: float, z_d: float):
    return (y_e, x_n, -z_d)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    return math.pi / 2.0 - yaw_ned


# ---------------------------------------------------------------------------
# Processing layer (ports of sss_helper.py)
# ---------------------------------------------------------------------------
def scale_to_db(pwr: np.ndarray, min_db: float, max_db: float) -> np.ndarray:
    return (min_db + (pwr / 65535.0) * (max_db - min_db)).astype(np.float32)


def find_noise_window_start(db: np.ndarray, search_max=RINGING_SEARCH_MAX,
                            drop_db=RINGING_DROP_DB,
                            persistence=RINGING_PERSISTENCE,
                            fallback=30) -> int:
    n = len(db)
    if n < search_max + persistence:
        return fallback
    target = float(db[:search_max].max()) - drop_db
    below = db < target
    for i in range(search_max - persistence + 1):
        if below[i:i + persistence].all():
            return i
    return fallback


def detect_fbr_slant_m(db: np.ndarray, start_mm: int, length_mm: int,
                       num_results: int) -> Optional[float]:
    nw_start = find_noise_window_start(db)
    nw_end = nw_start + NOISE_FLOOR_WINDOW
    if len(db) < nw_end + WITHIN_PING_PERSISTENCE:
        return None
    threshold = float(db[nw_start:nw_end].mean()) + FBR_THRESHOLD_DELTA_DB
    above = db > threshold
    for i in range(nw_end, len(db) - WITHIN_PING_PERSISTENCE + 1):
        if above[i:i + WITHIN_PING_PERSISTENCE].all():
            slant_mm = start_mm + (i / max(num_results - 1, 1)) * length_mm
            return slant_mm / 1000.0
    return None


def project_side(db: np.ndarray, start_mm: int, length_mm: int,
                 num_results: int, altitude_m: float,
                 y_offset_m: float, side_sign: float
                 ) -> Tuple[np.ndarray, np.ndarray]:
    """Slant-range correction dropping the water column (vectorized)."""
    i = np.arange(len(db), dtype=np.float64)
    slant = start_mm / 1000.0 + (i / max(num_results - 1, 1)) * (length_mm / 1000.0)
    keep = slant > altitude_m
    ground = np.sqrt(np.maximum(slant[keep] ** 2 - altitude_m ** 2, 0.0))
    y = side_sign * (y_offset_m + ground)
    return y.astype(np.float64), db[keep].astype(np.float32)


class _SideTracker:
    def __init__(self) -> None:
        self._window: Deque[float] = deque(maxlen=BOOTSTRAP_PINGS)
        self._altitude: Optional[float] = None
        self._reject = 0
        self._miss = 0

    def update(self, fbr: Optional[float]) -> Optional[float]:
        if self._altitude is None:
            if fbr is None:
                self._miss += 1
                if self._miss >= RELOCK_AFTER:
                    self._window.clear()
                    self._miss = 0
                return None
            self._miss = 0
            self._window.append(fbr)
            if (len(self._window) == BOOTSTRAP_PINGS
                    and max(self._window) - min(self._window)
                    <= ALTITUDE_AGREEMENT_TOL_M):
                self._altitude = sum(self._window) / len(self._window)
            return self._altitude
        if fbr is not None and abs(fbr - self._altitude) <= OUTLIER_TOL_M:
            self._altitude = fbr
            self._reject = 0
        else:
            self._reject += 1
        if self._reject >= RELOCK_AFTER:
            self._altitude = None
            self._window.clear()
            self._reject = self._miss = 0
        return self._altitude


class FBRTracker:
    """Dual-side altitude tracker with a *provisional* output.

    The original tracker returned ``None`` until ten consecutive
    detections agreed to within 0.30 m, and the caller dropped every
    ping until then — which silently discarded the start of every
    mission and, whenever the lock was lost, arbitrary chunks in the
    middle. SonarView displays data from the very first ping, so we
    match that: ``update`` always returns the best altitude available
    (locked > provisional > last known), and ``locked`` reports whether
    the strict agreement criterion is currently satisfied, so callers
    can flag quality without throwing data away.
    """

    def __init__(self) -> None:
        self._port = _SideTracker()
        self._stbd = _SideTracker()
        self._altitude: Optional[float] = None
        self._last_known: Optional[float] = None
        self.locked = False

    def update(self, port_alt, stbd_alt) -> Optional[float]:
        p, s = self._port.update(port_alt), self._stbd.update(stbd_alt)
        if p is not None and s is not None:
            self._altitude = max(p, s)
        elif p is not None:
            self._altitude = p
        elif s is not None:
            self._altitude = s
        else:
            self._altitude = None
        self.locked = self._altitude is not None
        if self._altitude is not None:
            self._last_known = self._altitude
            return self._altitude
        # Not locked: provisional estimate from the raw detections so the
        # ping is still usable, then fall back to the last known value.
        raw = [v for v in (port_alt, stbd_alt) if v is not None]
        if raw:
            self._last_known = max(raw)
            return self._last_known
        return self._last_known


def resolve_altitude(tracker: "FBRTracker", port_alt, stbd_alt,
                     mode: str = "auto",
                     manual_m: float = 0.0) -> Tuple[float, bool]:
    """Depth-compensation policy -> (altitude_m, locked).

    Mirrors SonarView's "Depth Compensation" source selector:

    * ``auto``   — bottom detection (our FBR tracker), provisional value
      accepted so no ping is ever discarded; falls back to 0.0 (no
      correction) while nothing has ever been detected;
    * ``manual`` — a fixed operator-supplied altitude;
    * ``off``    — altitude 0: ground range = slant range, no water
      column removed. This is SonarView's "Manual / 0 m" mode, which on
      shallow data (h << R) is very close to the corrected geometry and
      is far more robust than a wrong altitude, because an over-estimated
      altitude both deletes real samples and warps the near range.
    """
    if mode == "off":
        return 0.0, True
    if mode == "manual":
        return float(manual_m), True
    alt = tracker.update(port_alt, stbd_alt)
    if alt is None:
        return 0.0, False
    return float(alt), tracker.locked


# ---------------------------------------------------------------------------
# Mission model + reader
# ---------------------------------------------------------------------------
@dataclass
@dataclass(frozen=True)
class MissionSegment:
    """One recording session inside the file: everything between two id-10 headers.

    A ``.svlog`` may hold several. Cerulean's own harbour demo holds two,
    213.8 s of acquisition separated by a 397.8 s gap; before segments existed
    the loader concatenated them and reported one 611 s run of continuous data.
    """

    index: int
    t_start: float = 0.0            # mission seconds, first ping of the segment
    t_end: float = 0.0              # mission seconds, last ping of the segment
    ping_count: int = 0
    wall_clock: Optional[str] = None
    session_id: Optional[str] = None
    devices: Optional[str] = None
    counter_offset: int = 0
    counter_confidence: float = 0.0

    @property
    def duration_s(self) -> float:
        return max(self.t_end - self.t_start, 0.0)


@dataclass(frozen=True)
class MissionGap:
    """Dead time between two segments — the sonar was not acquiring.

    Emitted into ``events`` so every accumulator downstream can break rather
    than join across it: the waterfall would otherwise stack rows minutes apart
    as neighbours, and the mosaic rasterizer would densify a straight swath
    between the two positions.
    """

    t: float                        # mission seconds, where the gap opens
    seconds: float                  # how long nothing was acquired
    from_segment: int
    to_segment: int


@dataclass
class SvlogMission:
    """Fully decoded + processed mission, ready to feed the GUI stack.

    ``events`` is time-sorted: ("ping", t_s, SonarPing), ("state", t_s,
    RobotState) and ("gap", t_s, MissionGap). ``t_s`` is seconds from the first
    event on the **log's own clock** — profiles carry their ``timestamp_ms`` and
    mavlink is re-based onto it through the measured boot skew, so replay at x1
    is real time. ``synthetic_clock`` reports the fallback (see ``load_svlog``).
    """

    path: Path
    events: List[tuple] = field(default_factory=list)
    duration_s: float = 0.0
    origin: Optional[Tuple[float, float]] = None    # (lat, lon) at (x0, y0)
    origin_xy: Tuple[float, float] = (0.0, 0.0)
    ping_count: int = 0
    dropped_bootstrap: int = 0      # legacy field, kept for compatibility
    dropped_no_pose: int = 0        # no odom had arrived yet
    unlocked_pings: int = 0         # emitted with a provisional altitude
    both_sides: int = 0             # rows carrying port AND starboard
    counter_offset: int = 0         # stbd ping_number + this = port numbering
    counter_confidence: float = 0.0 # share of votes behind counter_offset
    segments: List[MissionSegment] = field(default_factory=list)
    boot_skew_ms: Optional[int] = None   # timestamp_ms - time_boot_ms
    synthetic_clock: bool = False        # any stream fell back to NS_PER_TICK
    clock_notes: List[str] = field(default_factory=list)
    unparsed_packets: Counter = field(default_factory=Counter)

    @property
    def pings(self) -> List[SonarPing]:
        return [e[2] for e in self.events if e[0] == "ping"]

    @property
    def gaps(self) -> List[MissionGap]:
        return [e[2] for e in self.events if e[0] == "gap"]

    @property
    def gap_times(self) -> List[float]:
        """Mission times at which a session boundary opens.

        The break list consumers pass to ``seabed_imager.feed_pings`` so no
        image window straddles two sessions.
        """
        return [g.t for g in self.gaps]

    @property
    def acquisition_s(self) -> float:
        """Time the sonar was actually pinging: segments summed, gaps excluded."""
        return sum(s.duration_s for s in self.segments)


def load_svlog(path: Path,
               progress: Optional[Callable[[float], None]] = None,
               depth_mode: str = "auto",
               manual_depth_m: float = 0.0) -> SvlogMission:
    """Read + process an entire .svlog into a SvlogMission.

    Five behaviours differ deliberately from the original implementation,
    all driven by measurements on real logs (see
    docs/SONARVIEW_SVLOG_ANALYSIS.md):

    1. **Side routing uses the packet's own ``channel_number``**, never
       the device/``src`` tag. In a real dual-Omniscan recording 19.8 %
       of packets carried the wrong ``src``, which put starboard data on
       the port side (and vice-versa) and produced the mirrored mosaic.
       ``channel_number`` and ``transducer_heading_deg`` agree on 100 %
       of packets in every log inspected, so the packet is trusted over
       its envelope.
    2. **Pings are assembled, never paired-and-dropped.** Profiles are
       grouped by ``ping_number`` and every group is emitted, even when
       only one side is present (that also makes single-transducer logs,
       like Cerulean's own harbour demo, load normally). The previous
       50 ms pairing threw away 10.4 % of the available rows.
    3. **The two counters are aligned before grouping.** The units number
       their pings independently, with an arbitrary constant offset
       (``estimate_counter_offset``), so the raw counter is not a shared
       key: on the two +60 logs in the corpus, grouping on it merged
       halves acquired ~1.7 s apart.
    4. **The timeline is the log's own clock.** Profiles stamp from their
       ``timestamp_ms``; mavlink stamps from ``time_boot_ms`` re-based by
       the measured ``estimate_boot_skew`` — the sonar and the autopilot
       count from their own boots, so without that constant the two
       streams are minutes apart. ``NS_PER_TICK`` survives only as a
       per-channel-per-segment fallback (``usable_stamps``). A flat tick
       reported Cerulean's 611.5 s demo as 251.6 s.
    5. **Session headers open segments.** A ``.svlog`` may hold several
       recordings; packet id 10 marks each. Segments are surfaced and the
       dead time between them is emitted as a ``MissionGap`` event, so no
       consumer joins across it.

    ``progress`` (0..1) is called periodically for GUI progress dialogs.
    """
    data = Path(path).read_bytes()
    mission = SvlogMission(path=Path(path))

    clock_ns = 0
    last_boot_ms: Optional[int] = None

    def tick(boot_ms: Optional[int]) -> int:
        """Fallback clock: mavlink bursts share a stamp, anything else advances."""
        nonlocal clock_ns, last_boot_ms
        if boot_ms is not None and boot_ms == last_boot_ms:
            return clock_ns
        clock_ns += NS_PER_TICK
        last_boot_ms = boot_ms
        return clock_ns

    # Pose synthesis state (ATTITUDE + LOCAL_POSITION_NED -> odom).
    latest_yaw: Optional[float] = None
    latest_lpn: Optional[dict] = None
    last_state_emit_ns = -10**18
    state_period_ns = int(1e9 / 5.0)                # 5 Hz, like live GUI
    cur_pose: Optional[Tuple[float, float, float]] = None
    cur_speed = 0.0

    #: (segment, normalised ping_number) -> {channel: profile, t_ns, pose, seg};
    #: pose and clock are captured on arrival.
    groups: dict = {}
    order: List[Tuple[int, int]] = []
    #: segment -> [first t_ns, last t_ns, emitted pings]
    seg_span: dict = {}

    def emit_state(t_ns: int) -> None:
        nonlocal last_state_emit_ns
        if cur_pose is None or t_ns - last_state_emit_ns < state_period_ns:
            return
        last_state_emit_ns = t_ns
        x, y, yaw = cur_pose
        lat = lon = None
        if mission.origin is not None:
            from ..utils.geodesy import enu_to_gps
            lat, lon = enu_to_gps(mission.origin[0], mission.origin[1],
                                  x - mission.origin_xy[0],
                                  y - mission.origin_xy[1])
        mission.events.append(("state", t_ns / 1e9, RobotState(
            t=t_ns / 1e9, x=x, y=y, yaw=yaw, lat=lat, lon=lon,
            heading_deg=yaw_to_compass_deg(yaw), speed_mps=cur_speed)))

    packets = list(walk_packets(data))

    # ---- pre-pass: segments, counter offsets and the two clocks ------------
    # Everything the main pass needs to key and stamp a packet is established
    # here, so that the main pass never has to converge on a value mid-file:
    # a group must sit under one key, and a stamp must not depend on how much
    # of the file has been read.
    #
    # Only heads are decoded — no pwr — so this stays cheap on a 19 000-ping log.
    seg_heads: dict = {}            # segment -> [(channel, ping_number, ts)]
    seg_meta: List[dict] = []       # segment -> id-10 fields
    skew_votes: List[int] = []
    boot_values: set = set()        # distinct time_boot_ms seen
    seg_of_packet: List[int] = []   # parallel to `packets`
    seg_index = -1
    last_profile_ts: Optional[int] = None
    for pkt in packets:
        pid = struct.unpack_from("<H", pkt, 4)[0]
        if pid == JSON_WRAPPER_ID:
            seg_index += 1
            seg_meta.append(decode_session_header(pkt[8:-2]))
        elif pid == OS_MONO_PROFILE_ID:
            if seg_index < 0:       # profiles before any header: segment 0
                seg_index = 0
                seg_meta.append({"wall_clock": None, "session_id": None,
                                 "devices": None})
            try:
                d = decode_os_mono_profile(pkt[8:-2])
            except ValueError:
                seg_of_packet.append(max(seg_index, 0))
                continue
            last_profile_ts = d["timestamp_ms"]
            seg_heads.setdefault(seg_index, []).append(
                (side_of(d), d["ping_number"], last_profile_ts))
        elif pid == MAVLINK_WRAPPER_ID:
            try:
                m = json.loads(pkt[8:-2].decode("utf-8")).get("message", {})
            except (ValueError, UnicodeDecodeError):
                m = {}
            boot = m.get("time_boot_ms")
            if boot is not None:
                boot_values.add(int(boot))
                if last_profile_ts is not None:
                    skew_votes.append(last_profile_ts - int(boot))
        seg_of_packet.append(max(seg_index, 0))

    if not seg_meta:                # no header and no profile
        seg_meta.append({"wall_clock": None, "session_id": None,
                         "devices": None})

    # Per-segment counter offset. The devices may be power-cycled between
    # sessions, which reassigns the offset; the whole-file value would then be
    # wrong for at least one segment. The mission-level value stays the first
    # segment's, which is what every existing caller means by it.
    seg_offset: dict = {}
    for i in range(len(seg_meta)):
        seg_offset[i] = estimate_counter_offset(seg_heads.get(i, []))
    mission.counter_offset, mission.counter_confidence = seg_offset.get(
        0, (0, 0.0))

    # Clock usability, per channel per segment (never over file order).
    stamps_ok: dict = {}
    for i, rows in seg_heads.items():
        for ch in {r[0] for r in rows}:
            ok = usable_stamps([r[2] for r in rows if r[0] == ch])
            stamps_ok[(i, ch)] = ok
            if not ok:
                mission.clock_notes.append(
                    f"session {i + 1}, {'port' if ch == 0 else 'starboard'}: "
                    f"unusable timestamp_ms — falling back to the "
                    f"{NS_PER_TICK // 1_000_000} ms tick")

    # The sonar clock carries the timeline. Two clocks cannot be mixed in one:
    # a real stamp and a tick count are not comparable, and interleaving them
    # reorders events rather than merely coarsening them. So the *detection* is
    # per channel per segment (which is what keeps it from firing on the
    # writer's channel batching) while the *fallback* is whole-file.
    real_clock = bool(stamps_ok) and all(stamps_ok.values())
    mission.synthetic_clock = not real_clock

    # The autopilot clock is a separate question, and it fails separately: one
    # corpus log carries a single frozen ``time_boot_ms`` on all 2008 of its
    # mavlink packets. Anchoring poses to that would collapse every RobotState
    # onto one instant. Where it is unusable but the sonar clock is good, poses
    # ride the sonar clock instead — the running profile stamp — which keeps the
    # real timeline rather than discarding it over the weaker of the two clocks.
    mission.boot_skew_ms = (estimate_boot_skew(skew_votes)
                            if len(boot_values) > 1 else None)
    if real_clock and mission.boot_skew_ms is None and boot_values:
        mission.clock_notes.append(
            "mavlink time_boot_ms is frozen at a single value — poses are "
            "placed on the sonar clock instead")
    skew_ms = mission.boot_skew_ms or 0
    mavlink_on_boot = real_clock and mission.boot_skew_ms is not None

    sonar_ns = 0                       # most recent profile stamp, for poses

    def stamp_profile(d: dict) -> int:
        nonlocal sonar_ns
        if not real_clock:
            return tick(None)
        sonar_ns = int(d["timestamp_ms"]) * 1_000_000
        return sonar_ns

    def stamp_mavlink(boot_ms) -> int:
        if mavlink_on_boot and boot_ms is not None:
            # Messages sharing a time_boot_ms share a stamp, as they must:
            # SonarView pairs ATTITUDE / GLOBAL_POSITION_INT /
            # LOCAL_POSITION_NED on exactly that equality.
            return (int(boot_ms) + skew_ms) * 1_000_000
        if real_clock:
            return sonar_ns
        return tick(boot_ms)

    for k, pkt in enumerate(packets):
        if progress is not None and k % 2000 == 0:
            progress(0.5 * k / max(len(packets), 1))
        pid = struct.unpack_from("<H", pkt, 4)[0]
        payload = pkt[8:-2]
        seg = seg_of_packet[k]
        if pid == OS_MONO_PROFILE_ID:
            try:
                d = decode_os_mono_profile(payload)
            except ValueError:
                continue
            t_ns = stamp_profile(d)
            # Authoritative side: the packet's own channel_number, with
            # the transducer bearing as a fallback for exotic writers.
            ch = side_of(d)
            # Both counters expressed on the port device's numbering, and
            # scoped to the segment: a power cycle between sessions restarts
            # both the counter and the offset between the two devices.
            offset = seg_offset.get(seg, (0, 0.0))[0]
            key = (seg, d["ping_number"] if ch == 0
                   else d["ping_number"] + offset)
            g = groups.get(key)
            if g is None:
                g = groups[key] = {"t_ns": t_ns, "pose": cur_pose, "seg": seg}
                order.append(key)
            g[ch] = d
        elif pid == MAVLINK_WRAPPER_ID:
            try:
                m = json.loads(payload.decode("utf-8")).get("message", {})
            except (ValueError, UnicodeDecodeError):
                continue
            t_ns = stamp_mavlink(m.get("time_boot_ms"))
            mtype = m.get("type")
            if mtype == "ATTITUDE":
                latest_yaw = yaw_ned_to_enu(float(m.get("yaw", 0.0)))
            elif mtype == "LOCAL_POSITION_NED":
                latest_lpn = m
            elif mtype == "GLOBAL_POSITION_INT" and mission.origin is None:
                lat = float(m.get("lat", 0)) / 1e7
                lon = float(m.get("lon", 0)) / 1e7
                if abs(lat) > 1e-6 and cur_pose is not None:
                    mission.origin = (lat, lon)
                    mission.origin_xy = (cur_pose[0], cur_pose[1])
            if latest_yaw is not None and latest_lpn is not None:
                px, py, _ = ned_to_enu_xyz(float(latest_lpn.get("x", 0.0)),
                                           float(latest_lpn.get("y", 0.0)),
                                           float(latest_lpn.get("z", 0.0)))
                vx, vy, _ = ned_to_enu_xyz(float(latest_lpn.get("vx", 0.0)),
                                           float(latest_lpn.get("vy", 0.0)),
                                           float(latest_lpn.get("vz", 0.0)))
                cur_pose = (px, py, latest_yaw)
                cur_speed = math.hypot(vx, vy)
                emit_state(t_ns)
        elif pid != JSON_WRAPPER_ID:
            # Counted, not silently skipped — a census is how an unrecognised
            # id ever gets identified. id 2194 (Omniscan status) is the one
            # this project has characterised; see decode_omniscan_status.
            mission.unparsed_packets[pid] += 1

    # ---- assemble: one row per ping_number, one-sided rows included ----
    fbr = FBRTracker()
    order.sort()                       # restores acquisition order exactly
    for i, key in enumerate(order):
        if progress is not None and i % 500 == 0:
            progress(0.5 + 0.5 * i / max(len(order), 1))
        g = groups[key]
        pose = g["pose"]
        if pose is None:
            mission.dropped_no_pose += 1
            continue                   # genuinely unplaceable (no odom yet)
        p, s = g.get(0), g.get(1)
        db_p = alt_p = db_s = alt_s = None
        if p is not None:
            db_p = scale_to_db(p["pwr"], p["min_pwr_db"], p["max_pwr_db"])
            alt_p = detect_fbr_slant_m(db_p, p["start_mm"], p["length_mm"],
                                       p["num_results"])
        if s is not None:
            db_s = scale_to_db(s["pwr"], s["min_pwr_db"], s["max_pwr_db"])
            alt_s = detect_fbr_slant_m(db_s, s["start_mm"], s["length_mm"],
                                       s["num_results"])
        altitude, locked = resolve_altitude(fbr, alt_p, alt_s,
                                            depth_mode, manual_depth_m)
        if not locked:
            mission.unlocked_pings += 1
        ys, ins = [], []
        if p is not None:
            y, iv = project_side(db_p, p["start_mm"], p["length_mm"],
                                 p["num_results"], altitude,
                                 TRANSDUCER_Y_OFFSET_PORT_M, +1.0)
            ys.append(y); ins.append(iv)
        if s is not None:
            y, iv = project_side(db_s, s["start_mm"], s["length_mm"],
                                 s["num_results"], altitude,
                                 TRANSDUCER_Y_OFFSET_STBD_M, -1.0)
            ys.append(y); ins.append(iv)
        if not ys:
            continue
        ref = p if p is not None else s
        x, y0, yaw = pose
        mission.events.append(("ping", g["t_ns"] / 1e9, SonarPing(
            t=g["t_ns"] / 1e9, robot_x=x, robot_y=y0, yaw=yaw,
            water_depth=altitude + TRANSDUCER_SUBMERSION_M,
            y_local=np.concatenate(ys),
            intensity_db=np.concatenate(ins),
            slant_range_m=ref["length_mm"] / 1000.0,
            sides=("both" if (p is not None and s is not None)
                   else ("port" if p is not None else "starboard")))))
        mission.ping_count += 1
        if p is not None and s is not None:
            mission.both_sides += 1
        span = seg_span.setdefault(g["seg"], [g["t_ns"], g["t_ns"], 0])
        span[0] = min(span[0], g["t_ns"])
        span[1] = max(span[1], g["t_ns"])
        span[2] += 1

    mission.events.sort(key=lambda e: e[1])
    if mission.events:
        from dataclasses import replace as dc_replace
        t0 = mission.events[0][1]
        mission.events = [(kind, t - t0, dc_replace(obj, t=obj.t - t0))
                          for kind, t, obj in mission.events]
        mission.duration_s = mission.events[-1][1]
    else:
        t0 = 0.0

    # ---- segments and the dead time between them ---------------------------
    # Only segments that actually produced a placeable ping become a segment:
    # a header with nothing behind it is a writer artefact, not a session.
    for idx in sorted(seg_span):
        lo, hi, n = seg_span[idx]
        offset, confidence = seg_offset.get(idx, (0, 0.0))
        meta = seg_meta[idx] if idx < len(seg_meta) else {}
        mission.segments.append(MissionSegment(
            index=len(mission.segments),
            t_start=lo / 1e9 - t0, t_end=hi / 1e9 - t0, ping_count=n,
            wall_clock=meta.get("wall_clock"),
            session_id=meta.get("session_id"), devices=meta.get("devices"),
            counter_offset=offset, counter_confidence=confidence))

    for a, b in zip(mission.segments, mission.segments[1:]):
        gap = b.t_start - a.t_end
        if gap <= 0:
            continue                   # sessions that abut; nothing to break
        mission.events.append(("gap", a.t_end, MissionGap(
            t=a.t_end, seconds=gap,
            from_segment=a.index, to_segment=b.index)))
    if len(mission.segments) > 1:
        mission.events.sort(key=lambda e: e[1])

    if progress is not None:
        progress(1.0)
    return mission
