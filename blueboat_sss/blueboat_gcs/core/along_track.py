"""One along-track estimator for every consumer of "metres per ping".

The on-screen waterfall stretches its rows to true scale and the AI
seabed pictures resample their rows to square pixels; both need the
boat's along-track progress per ping. Until 2026-09-15 they estimated it
independently, from the raw displacement between consecutive ping poses:
the waterfall took a running median of those steps, the imager summed
them. Measured on the field logs (``project_review.md`` §3.1) neither
holds at the project's own 20 Hz setting: consecutive pings often share
a pose sample (18 % of pings on the SonarLink-recorded 20 m log), so the
median of steps was biased low by a quarter and fell to zero in 26 % of
its windows — which the view turned into "no true scale" — while the
summed steps are inflated by rectified pose noise.

This estimator uses the ping stream only (identical live and in replay)
and separates the two quantities that were being confused:

* **ground speed** — the median, over the last ``window_s`` seconds, of
  chord speeds measured over spans of about ``pair_s`` seconds (the most
  recent sample at least ``pair_s`` old is the reference), so repeated
  pose samples and centimetre jitter average out while a turn is still
  followed within a second;
* **ping period** — the median inter-ping interval over the same window.

``row_pitch_m = speed_mps * ping_period_s`` is what one ping advances
along-track; :meth:`advance` integrates the same speed over the ping
interval into a cumulative along-track position (monotone
non-decreasing, never inflated by jitter). Before the window spans
``pair_s`` the reference is the oldest sample held, so a constant-speed
fixture reads exactly from its second ping.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional, Tuple


class AlongTrackEstimator:
    """Ground speed, ping period and cumulative along-track from ping poses."""

    def __init__(self, window_s: float = 5.0, pair_s: float = 1.0) -> None:
        self._window_s = max(float(window_s), 1e-3)
        self._pair_s = max(min(float(pair_s), self._window_s), 1e-3)
        self._samples: Deque[Tuple[float, float, float]] = deque()
        self._speeds: Deque[Tuple[float, float]] = deque()   # (t, chord speed)
        self._dts: Deque[Tuple[float, float]] = deque()      # (t, dt)
        self._t_prev: Optional[float] = None
        self._along = 0.0

    # ---- ingestion ---------------------------------------------------------------
    def observe(self, t: float, x: float, y: float) -> None:
        """Record one ping pose. Non-finite input is ignored; a time that
        does not advance contributes no interval."""
        if not (math.isfinite(t) and math.isfinite(x) and math.isfinite(y)):
            return
        if self._samples:
            dt = t - self._samples[-1][0]
            if dt > 0.0:
                self._dts.append((t, dt))
        self._samples.append((t, x, y))
        oldest_kept = t - self._window_s
        while len(self._samples) > 2 and self._samples[0][0] < oldest_kept:
            self._samples.popleft()
        while self._dts and self._dts[0][0] < oldest_kept:
            self._dts.popleft()
        while self._speeds and self._speeds[0][0] < oldest_kept:
            self._speeds.popleft()
        ref = self._reference(t)
        if ref is not None and t - ref[0] > 0.0:
            v = math.hypot(x - ref[1], y - ref[2]) / (t - ref[0])
            self._speeds.append((t, v))

    def _reference(self, t: float) -> Optional[Tuple[float, float, float]]:
        """The most recent sample at least ``pair_s`` older than ``t``, or
        the oldest sample held while the window is still filling."""
        if len(self._samples) < 2:
            return None
        cut = t - self._pair_s
        ref = None
        for s in reversed(self._samples):
            if s[0] <= cut:
                ref = s
                break
        return ref if ref is not None else self._samples[0]

    def mark_break(self) -> None:
        """A seam (session gap, acquisition loss): no chord and no interval
        may span it. Speed and period history is kept, so the reported
        pitch does not collapse across a few blank rows."""
        self._samples.clear()
        self._t_prev = None

    def reset(self) -> None:
        self._samples.clear()
        self._speeds.clear()
        self._dts.clear()
        self._t_prev = None
        self._along = 0.0

    # ---- estimates -------------------------------------------------------------------
    @staticmethod
    def _median(values: Deque[Tuple[float, float]]) -> float:
        if not values:
            return 0.0
        v = sorted(s for _, s in values)
        n = len(v)
        mid = n // 2
        return float(v[mid] if n % 2 else 0.5 * (v[mid - 1] + v[mid]))

    @property
    def speed_mps(self) -> float:
        """Median chord speed over the window (0 until two samples)."""
        return self._median(self._speeds)

    @property
    def ping_period_s(self) -> float:
        """Median interval between pings over the window (0 until two)."""
        return self._median(self._dts)

    @property
    def row_pitch_m(self) -> float:
        """Along-track metres one ping advances: speed × period."""
        return self.speed_mps * self.ping_period_s

    @property
    def along_m(self) -> float:
        """Cumulative along-track position [m] as of the last :meth:`advance`."""
        return self._along

    def advance(self, t: float) -> float:
        """Integrate the current speed over the interval since the previous
        call and return the cumulative along-track position."""
        if self._t_prev is not None and math.isfinite(t) and t > self._t_prev:
            self._along += self.speed_mps * (t - self._t_prev)
        if math.isfinite(t):
            self._t_prev = t
        return self._along
