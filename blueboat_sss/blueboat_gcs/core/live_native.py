"""ROS-free half of the live sonar listener: attach the raw native bins.

``ProcessedSSSPing`` carries slant-corrected ground samples with the
water column already deleted, so a waterfall built from it can never
show the nadir the way the replay path (raw ``.svlog``) does, and the
re-projection back to slant bins is approximate. The GCS therefore also
subscribes to the two raw ``OmniscanProfile`` topics the processor
consumes, caches them per side by device ``ping_number`` (the processed
message carries both raw counters), and attaches the **verbatim** native
payload to each row — exactly what ``core/svlog.py`` builds on replay,
through the same :func:`native_from_profile`. When a profile is missing
(BEST_EFFORT loss, or a processed row that arrived before its profiles
and outlived ``profile_wait_ms``) that side falls back to the
re-projection, so no row is ever withheld (CM-6).

Everything here is plain numpy so the parity test runs on a laptop
without ROS; ``ros/sonar_listener.py`` is the thin adapter that feeds
:class:`ProfileCache` from the subscriptions and drains
:class:`PendingRows` from a node timer. All of it runs on the executor
thread; the GUI thread only ever sees the emitted ``SonarPing``.
"""

from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np

from ..models.sonar import native_from_profile, side_bins_from_ground
from .svlog import TRANSDUCER_SUBMERSION_M, scale_to_db

#: A key this far behind the newest one seen is a device counter restart
#: (same rule as the processor's COUNTER_RESTART_PINGS).
COUNTER_RESTART_PINGS = 128


@dataclass
class CachedProfile:
    db: np.ndarray            # float32, full raw dB from bin 0
    start_mm: int
    length_mm: int
    num_results: int
    gain_index: int


def side_of_profile(channel_number: int, transducer_heading_deg: float) -> int:
    """CM-5: side from the packet — ``channel_number`` (0 port / 1 stbd),
    falling back to the sign of the transducer bearing."""
    if channel_number in (0, 1):
        return int(channel_number)
    return 1 if float(transducer_heading_deg) > 0.0 else 0


def profile_from_msg(msg: Any) -> Tuple[int, int, CachedProfile]:
    """(side, ping_number, profile) from an ``OmniscanProfile`` message
    (duck-typed: any object with the message's attribute names)."""
    pwr = np.asarray(msg.pwr_results, dtype=np.float64)
    db = scale_to_db(pwr, float(msg.min_pwr_db), float(msg.max_pwr_db))
    side = side_of_profile(int(msg.channel_number),
                           float(getattr(msg, "transducer_heading_deg", 0.0)))
    prof = CachedProfile(db=np.asarray(db, dtype=np.float32),
                         start_mm=int(msg.start_mm), length_mm=int(msg.length_mm),
                         num_results=int(msg.num_results),
                         gain_index=int(getattr(msg, "gain_index", -1)))
    return side, int(msg.ping_number), prof


class ProfileCache:
    """Bounded per-side store of raw profiles keyed by device ping number."""

    def __init__(self, per_side: int = 512,
                 restart_pings: int = COUNTER_RESTART_PINGS) -> None:
        self._per_side = max(int(per_side), 2)
        self._restart = int(restart_pings)
        self._store: List["OrderedDict[int, CachedProfile]"] = [
            OrderedDict(), OrderedDict()]
        self._newest = [None, None]        # type: List[Optional[int]]
        self.puts = 0
        self.evicted = 0
        self.restarts = 0

    def put(self, side: int, ping_number: int, prof: CachedProfile) -> None:
        store = self._store[side]
        newest = self._newest[side]
        if newest is not None and ping_number < newest - self._restart:
            # The device (or the simulator) restarted its counter: every
            # cached key belongs to the previous power-up.
            store.clear()
            self.restarts += 1
            newest = None
        store[ping_number] = prof
        store.move_to_end(ping_number)
        self._newest[side] = ping_number if newest is None else max(newest, ping_number)
        self.puts += 1
        while len(store) > self._per_side:
            store.popitem(last=False)
            self.evicted += 1

    def take(self, side: int, ping_number: int) -> Optional[CachedProfile]:
        return self._store[side].pop(ping_number, None)

    def has(self, side: int, ping_number: int) -> bool:
        return ping_number in self._store[side]

    def __len__(self) -> int:
        return len(self._store[0]) + len(self._store[1])

    def clear(self) -> None:
        for s in self._store:
            s.clear()
        self._newest = [None, None]


@dataclass
class ProcessedFields:
    """The parts of a ``ProcessedSSSPing`` the attach step needs (already
    converted to numpy so the ROS message can be released)."""

    port_pn: int
    stbd_pn: int
    water_depth: float
    port_y: np.ndarray
    port_db: np.ndarray
    stbd_y: np.ndarray
    stbd_db: np.ndarray


def attach_native(f: ProcessedFields, cache: ProfileCache
                  ) -> Tuple[Dict[str, Any], Tuple[bool, bool]]:
    """SonarPing keyword fields for the native payload of one row.

    Returns ``(fields, (port_hit, stbd_hit))``: a hit means the side's
    verbatim profile was attached; a miss means the re-projection
    fallback (``side_bins_from_ground``) was used for that side. An
    absent side (``*_pn == 0``) is neither.
    """
    depth = max(float(f.water_depth), 0.0)
    out: Dict[str, Any] = {"bin_size_m": 0.0, "port_bin0": 0, "port_db": None,
                           "stbd_bin0": 0, "stbd_db": None,
                           "stbd_bin_size_m": 0.0, "gain_index": -1,
                           # Exact parity with replay's ``fbr.bottom``:
                           # water_depth = altitude + submersion.
                           "bottom_slant_m": max(depth - TRANSDUCER_SUBMERSION_M, 0.0)}
    hits = [False, False]
    sizes = [0.0, 0.0]
    for side, pn, y, db in ((0, f.port_pn, f.port_y, f.port_db),
                            (1, f.stbd_pn, f.stbd_y, f.stbd_db)):
        if not pn:
            continue
        prof = cache.take(side, pn)
        if prof is not None:
            bin0, size = native_from_profile(prof.start_mm, prof.length_mm,
                                             prof.num_results)
            vals = prof.db
            hits[side] = True
            if prof.gain_index >= 0:
                out["gain_index"] = prof.gain_index
        else:
            bin0, vals, size = side_bins_from_ground(np.abs(y), db, depth)
            if size <= 0.0:
                continue
        sizes[side] = size
        if side == 0:
            out["port_bin0"], out["port_db"] = bin0, vals
        else:
            out["stbd_bin0"], out["stbd_db"] = bin0, vals
    out["bin_size_m"] = sizes[0] if sizes[0] > 0 else sizes[1]
    out["stbd_bin_size_m"] = sizes[1] if (sizes[1] > 0 and sizes[0] > 0
                                         and abs(sizes[1] - sizes[0]) > 1e-12) else 0.0
    if out["bin_size_m"] <= 0.0:
        out["port_db"] = out["stbd_db"] = None
    return out, (hits[0], hits[1])


class PendingRows:
    """FIFO of processed rows waiting (briefly) for their profiles.

    Rows are released strictly in arrival order: a row is ready when
    every present side has its profile cached, or when it has waited
    ``wait_s``. Normally the profiles arrive *before* the processed row
    (the processor consumed them to build it), so the queue is empty and
    :meth:`push` returns the row at once.
    """

    def __init__(self, cache: ProfileCache, wait_s: float = 0.06) -> None:
        self._cache = cache
        self._wait = float(wait_s)
        self._q: Deque[Tuple[float, Any, ProcessedFields]] = deque()

    def _ready(self, f: ProcessedFields) -> bool:
        return ((not f.port_pn or self._cache.has(0, f.port_pn))
                and (not f.stbd_pn or self._cache.has(1, f.stbd_pn)))

    def push(self, now: float, item: Any, f: ProcessedFields) -> List[Tuple[Any, ProcessedFields, bool]]:
        """Enqueue and return whatever is releasable now, in order, as
        ``(item, fields, timed_out)`` triples."""
        self._q.append((now, item, f))
        return self.drain(now)

    def drain(self, now: float) -> List[Tuple[Any, ProcessedFields, bool]]:
        out = []
        while self._q:
            t0, item, f = self._q[0]
            timed_out = now - t0 >= self._wait
            if self._ready(f) or timed_out:
                self._q.popleft()
                out.append((item, f, timed_out and not self._ready(f)))
            else:
                break
        return out

    def __len__(self) -> int:
        return len(self._q)
