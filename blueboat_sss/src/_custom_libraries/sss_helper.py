#!/usr/bin/env python3
"""Pure-math helpers and trackers for side scan sonar post-processing.
"""

from __future__ import annotations

import math
import numpy as np
from collections import deque
from typing import Deque, List, Optional, Sequence, Tuple

def project_to_world(
    robot_x: float, robot_y: float, yaw: float,
    y_local: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Rotate a ping's lateral samples into world coordinates.

    The ping is purely lateral in the boat frame: each sample sits at
    (x_body=0, y_body=y_local[i]). REP-103 conventions:
      * +y_body = port  -> processor publishes positive `port_y[i]`
      * -y_body = stbd  -> processor publishes negative `starboard_y[i]`

    The 2D rotation by yaw of (0, y_body) into world frame is:
        x_w = -sin(yaw) * y_body
        y_w =  cos(yaw) * y_body
    """
    x_world = robot_x - math.sin(yaw) * y_local
    y_world = robot_y + math.cos(yaw) * y_local
    return x_world, y_world


def scale_to_db(
    pwr_u16: Sequence[int], min_pwr_db: float, max_pwr_db: float
) -> List[float]:
    """Convert raw u16 power samples to dB.

    Formula (Cerulean Omniscan 450 template):
        db = min_pwr_db + (raw / 65535) * (max_pwr_db - min_pwr_db)

    See: github.com/bluerobotics/ping-python/blob/master/generate/templates/omniscan450.py.in
    """
    span = max_pwr_db - min_pwr_db
    return [min_pwr_db + (s / 65535.0) * span for s in pwr_u16]

def find_noise_window_start(
    pwr_db,
    search_max: int = 60,
    drop_db: float = 10.0,
    persistence: int = 5,
    fallback: int = 30,
) -> int:
    """Locate the first sample past the transmit-pulse ringing.

    The transmit pulse and its ringing tail occupy the first samples of
    every ping with intensity well above the post-ringing water-column
    noise. This function returns the index at which ringing has decayed
    enough to safely estimate the noise floor.

    Algorithm
    ---------
    1. peak = max(pwr_db[:search_max])     # ringing peak (near sample 0-1)
    2. target = peak - drop_db
    3. Return the first index i in [0, search_max - persistence] such that
       pwr_db[i : i + persistence] are all strictly below target.
    4. If no such index is found, return `fallback` (defensive default;
       see note below).

    Why this adapts to the environment
    ----------------------------------
    The algorithm only uses values relative to the ping's own ringing peak.
    A pool with long ringing tails, an open-water run with short ringing,
    or a configuration with different hardware gain all produce a different
    `target` and a different settle index — derived from the data, not
    from any tuned constant.

    Parameters
    ----------
    pwr_db : sequence of float
        Per-sample intensity in dB (already scaled from u16 via scale_to_db).
    search_max : int
        How many samples from the start to consider. This is a SAMPLE
        COUNT, so the slant range it covers scales with
        `length_mm / num_results`: 60 samples is 2.0 m at the 33.3 mm/sample
        default (20 m / 600), 3.0 m at 30 m / 600, and 8.0 m at 80 m / 600.
        It must stay smaller than the expected sample index of the
        shallowest plausible FBR, so lower it — or shorten the range — for
        deployments whose altitude falls below that reach.
    drop_db : float
        How far below the ringing peak we consider "settled". 10 dB is the
        empirical value from the pool run and matches the literature
        (pulse-to-noise contrast is typically 15-25 dB).
    persistence : int
        How many consecutive samples must be below target to count as
        settled. Guards against single-sample dips inside the ringing tail.
    fallback : int
        Returned when no settle is found in [0, search_max). Also a sample
        count: 30 ≈ 1.0 m at the 33.3 mm/sample default (20 m / 600) — past
        any sensible ringing tail, before any sensible bottom return.

    Returns
    -------
    int : index where the noise window should start.
    """
    n = len(pwr_db)
    if n < search_max + persistence:
        return fallback
    peak = max(pwr_db[:search_max])
    target = peak - drop_db
    for i in range(search_max - persistence + 1):
        if all(pwr_db[i + k] < target for k in range(persistence)):
            return i
    return fallback

def detect_fbr_slant_m(
    pwr_db, start_mm, length_mm, num_results,
    *, noise_floor_window, threshold_delta_db, persistence,
    ringing_search_max: int = 60,
    ringing_drop_db: float = 10.0,
    ringing_persistence: int = 5,
):
    nw_start = find_noise_window_start(
        pwr_db,
        search_max=ringing_search_max,
        drop_db=ringing_drop_db,
        persistence=ringing_persistence,
    )
    nw_end = nw_start + noise_floor_window
    if len(pwr_db) < nw_end + persistence:
        return None
    noise = sum(pwr_db[nw_start:nw_end]) / noise_floor_window
    threshold = noise + threshold_delta_db
    for i in range(nw_end, len(pwr_db) - persistence + 1):
        if pwr_db[i] <= threshold:
            continue
        if all(pwr_db[i + k] > threshold for k in range(persistence)):
            # existing slant-range conversion — keep whatever you have here:
            slant_mm = start_mm + (i / max(num_results - 1, 1)) * length_mm
            return slant_mm / 1000.0
    return None


def project_side(
    pwr_db: Sequence[float],
    start_mm: int,
    length_mm: int,
    num_results: int,
    altitude_m: float,
    transducer_y_offset_m: float,
    side_sign: float,
) -> Tuple[List[float], List[float]]:
    """Slant-range-correct one side, drop water-column samples.

    Returns (y_in_base_link, intensities_db); sample i of one list pairs
    with sample i of the other. `side_sign` = +1 for port, -1 for starboard.

    Assumes the seabed is locally flat under each ping (standard SSS
    practice with single-beam data).
    """
    y_out: List[float] = []
    db_out: List[float] = []
    start_m = start_mm / 1000.0
    length_m = length_mm / 1000.0
    denom = max(num_results - 1, 1)
    for i in range(len(pwr_db)):
        slant = start_m + (i / denom) * length_m
        if slant <= altitude_m:
            continue  # water column
        ground = math.sqrt(slant * slant - altitude_m * altitude_m)
        y_out.append(float(side_sign * (transducer_y_offset_m + ground)))
        db_out.append(float(pwr_db[i]))
    return y_out, db_out


class PingCounterOffset:
    """Estimator for the constant offset between the two devices' ping counters.

    The two Omniscan 450 units are independent devices with independent
    `ping_number` counters, and the offset between them is arbitrary but
    constant for a power-up cycle. Measured across the project's field
    `.svlog` corpus it is 0 on ten logs, -1 on four and +60 on two, so the
    raw counter cannot be used directly as a cross-side assembly key: on a
    +60 log it would pair a port ping with a starboard ping acquired ~1.7 s
    earlier.

    The offset is recovered by voting: each ping is compared against the
    opposite side's *temporally nearest* ping, and the mode of
    `port_ping_number - starboard_ping_number` over a rolling window is the
    estimate.

    Voting against the most recent opposite-side ping instead of the nearest
    one is **bimodal** -- it splits roughly evenly between the true offset
    and offset +-1 depending on interleave phase -- so a vote is deferred by
    `defer` arrivals, until both the earlier and the later opposite-side
    neighbours are available. With that deferral the winning mode holds
    >=94 % of votes on every two-sided log in the corpus.
    """

    def __init__(self, window: int = 64, defer: int = 4) -> None:
        self._defer = defer
        self._recent: dict = {0: deque(maxlen=2 * defer + 1),
                              1: deque(maxlen=2 * defer + 1)}
        self._pending: Deque[Tuple[int, int, int]] = deque()
        self._votes: Deque[int] = deque(maxlen=window)
        self._offset = 0

    @property
    def offset(self) -> int:
        """Value to add to a starboard `ping_number` to get the port key."""
        return self._offset

    @property
    def votes(self) -> int:
        return len(self._votes)

    @property
    def confidence(self) -> float:
        """Share of the rolling window agreeing with the current estimate."""
        if not self._votes:
            return 0.0
        return self._votes.count(self._offset) / len(self._votes)

    def observe(self, channel: int, ping_number: int, stamp_ns: int) -> None:
        """Feed one arriving ping. `channel` is 0 = port, 1 = starboard."""
        self._recent[channel].append((stamp_ns, ping_number))
        self._pending.append((channel, ping_number, stamp_ns))
        if len(self._pending) <= self._defer:
            return
        ch, pn, ts = self._pending.popleft()
        opposite = self._recent[1 - ch]
        if not opposite:
            return
        _, opn = min(opposite, key=lambda e: abs(e[0] - ts))
        self._votes.append((pn - opn) if ch == 0 else (opn - pn))
        # Mode of the rolling window; ties resolve to the incumbent, which
        # keeps the estimate from flapping between two equally-supported
        # values.
        best, best_n = self._offset, self._votes.count(self._offset)
        for cand in set(self._votes):
            n = self._votes.count(cand)
            if n > best_n:
                best, best_n = cand, n
        self._offset = best

    def key(self, channel: int, ping_number: int) -> int:
        """Assembly key: the ping number expressed on the port counter."""
        return ping_number if channel == 0 else ping_number + self._offset


class _SideTracker:
    """Single-side altitude tracker bootstrapped from temporal self-consistency.

    A side is considered locked once `bootstrap_pings` of its most recent
    detections all fall within `agreement_tol_m` of each other (a stable,
    plausible seabed return) -- it does NOT need to agree with the other
    side. This makes the system robust to one transducer (e.g. a low-gain
    channel) returning noise: the good side carries the estimate alone.

    After lock, each new detection within `outlier_tol_m` of the current
    altitude updates it; detections further away are rejected as outliers
    (a fish, a rock edge, a noise spike) and the last value is held. A
    long run of rejects (longer than `relock_after`) forces a re-bootstrap,
    so the tracker recovers if the seabed genuinely steps.
    """

    def __init__(self, bootstrap_pings: int, agreement_tol_m: float,
                 outlier_tol_m: float, relock_after: int) -> None:
        self._bootstrap_pings = bootstrap_pings
        self._agreement_tol_m = agreement_tol_m
        self._outlier_tol_m = outlier_tol_m
        self._relock_after = relock_after
        self._window: Deque[float] = deque(maxlen=bootstrap_pings)
        self._altitude: Optional[float] = None
        self._reject_streak = 0
        self._miss_streak = 0

    @property
    def altitude(self) -> Optional[float]:
        return self._altitude

    @property
    def locked(self) -> bool:
        return self._altitude is not None

    def update(self, fbr: Optional[float]) -> Optional[float]:
        if self._altitude is None:
            # --- bootstrap phase ---
            # An occasional missed detection (None) shouldn't wipe progress;
            # we just don't add to the window. Because the window is a
            # fixed-size sliding deque, stale early values age out on their
            # own, so a slowly-drifting seabed still locks once the most
            # recent `bootstrap_pings` detections are mutually consistent.
            if fbr is None:
                self._miss_streak += 1
                # Only a long blackout (no bottom at all) clears progress.
                if self._miss_streak >= self._relock_after:
                    self._window.clear()
                    self._miss_streak = 0
                return None
            self._miss_streak = 0
            self._window.append(fbr)
            if len(self._window) == self._bootstrap_pings:
                if (max(self._window) - min(self._window)) <= self._agreement_tol_m:
                    self._altitude = sum(self._window) / len(self._window)
            return self._altitude

        # --- operational phase ---
        if fbr is None:
            self._reject_streak += 1
        elif abs(fbr - self._altitude) <= self._outlier_tol_m:
            self._altitude = fbr
            self._reject_streak = 0
        else:
            self._reject_streak += 1

        if self._reject_streak >= self._relock_after:
            # Lost the bottom -- drop the lock and re-bootstrap.
            self._altitude = None
            self._window.clear()
            self._reject_streak = 0
            self._miss_streak = 0
        return self._altitude


class FBRTracker:
    """Dual-side altitude estimator: each side bootstraps independently.

    Per-ping inputs are the FBR slant ranges from each side. Each side runs
    its own `_SideTracker`; the fused altitude is the max() of whichever
    sides are currently locked (max so the seabed is never under-estimated,
    which would push samples into the water column and create false holes).

    The critical property vs the old design: bootstrap no longer requires
    the two sides to agree with each other. If one transducer is low-gain
    and returns noise, the other side bootstraps and carries the estimate
    by itself. The system produces an altitude as soon as EITHER side is
    self-consistent for `bootstrap_pings` detections.

    `update` never withholds a usable value: it returns the best altitude
    available -- locked, else provisional (this ping's own raw detections),
    else the last known one -- and `locked` reports whether the strict
    agreement criterion is currently met. Callers flag quality with
    `locked`; they do not gate emission on it, because withholding pings
    until lock silently discards the start of every mission (NON-NEGOTIABLE
    #2). `None` comes back only when nothing has ever been detected, and
    means "apply no slant correction", not "drop this ping".
    """

    def __init__(self, bootstrap_pings: int, agreement_tol_m: float,
                 outlier_tol_m: float = 1.0, relock_after: int = 15) -> None:
        self._port = _SideTracker(bootstrap_pings, agreement_tol_m,
                                  outlier_tol_m, relock_after)
        self._stbd = _SideTracker(bootstrap_pings, agreement_tol_m,
                                  outlier_tol_m, relock_after)
        self._altitude: Optional[float] = None
        self._last_known: Optional[float] = None
        self.locked = False

    @property
    def is_bootstrapped(self) -> bool:
        return self._altitude is not None

    @property
    def altitude(self) -> Optional[float]:
        return self._altitude

    def update(
        self, port_alt: Optional[float], stbd_alt: Optional[float]
    ) -> Optional[float]:
        p = self._port.update(port_alt)
        s = self._stbd.update(stbd_alt)

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

        # Not locked. Prefer a provisional estimate from this ping's own raw
        # detections over holding a stale value, then fall back to the last
        # known altitude. Either keeps the ping usable; neither is presented
        # as a locked estimate.
        raw = [v for v in (port_alt, stbd_alt) if v is not None]
        if raw:
            self._last_known = max(raw)
        return self._last_known
