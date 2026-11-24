"""The GCS's world-timing clock: wall monotonic, or sim-paced under /clock.

Why (owner, 2026-11-07). The simulation may run slower than real time (Gazebo
real-time factor 0.01-0.5) so the ray-cast sonar can render each ping inside
its sim-time period. Every stream then arrives at the slowed wall pace, and
every GCS rule written in wall seconds -- the attitude / compass freshness
attached to each ping, telemetry staleness, the odom-dead GPS fallback, the
recording's ``duration_s`` -- would judge or record a slowed run differently
from a real-speed one. Those rules read :func:`now` instead of
``time.monotonic()``.

Semantics:

* **Real-robot graph** (no ``/clock`` publisher; mavros runs ``use_sim_time``
  with none): never fed, so :func:`now` *is* ``time.monotonic()`` -- nothing
  changes.
* **Simulation graph**: ``ros/ros_manager`` feeds every ``/clock`` sample.
  :func:`now` stays continuous with the monotonic value it had when the feed
  started and from then on advances by **sim seconds**, interpolated between
  samples by the measured sim/wall ratio. It never goes backwards.
* **Stalled feed** (``/clock`` silent for ``stall_s`` wall seconds: Gazebo
  closed, paused or hung): it advances at the wall rate again, so staleness
  shows, and :attr:`stalled` says so.
* A sim clock that jumps back (Gazebo restarted under the GCS) re-bases;
  :func:`now` still never decreases.

Qt-free and ROS-free (``tests/test_stream_clock.py``).
"""

from __future__ import annotations

import threading
import time
from typing import Optional

#: Wall seconds of /clock silence after which the clock falls back to wall pace.
DEFAULT_STALL_S = 3.0
#: Time constant [wall s] of the measured sim/wall ratio's smoothing.
RATIO_TAU_S = 2.0


class StreamClock:
    """See module docstring. Thread-safe: fed from the ROS thread, read
    from the GUI thread."""

    def __init__(self, stall_s: float = DEFAULT_STALL_S) -> None:
        self._stall_s = float(stall_s)
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        """Back to pure wall time (no feed)."""
        with self._lock:
            self._fed = False
            self._v_ref = 0.0      # clock value at the reference sample
            self._s_ref = 0.0      # sim time of the reference sample
            self._s_last = 0.0     # newest sim sample
            self._m_last = 0.0     # monotonic time it arrived
            self._ratio = 1.0      # measured d(sim)/d(wall), smoothed
            self._floor = -float("inf")

    # ----------------------------------------------------------------- feed
    def feed(self, sim_s: float, mono_s: Optional[float] = None) -> None:
        """One ``/clock`` sample (sim seconds) received at ``mono_s``
        (``time.monotonic()`` when omitted)."""
        m = time.monotonic() if mono_s is None else float(mono_s)
        s = float(sim_s)
        with self._lock:
            if not self._fed:
                self._v_ref, self._s_ref = max(m, self._floor), s
                self._s_last, self._m_last = s, m
                self._fed = True
                return
            if s < self._s_last - 1e-9:
                # Sim restarted: continue from where the clock stands.
                self._v_ref = max(self._value(m), self._floor)
                self._s_ref = s
                self._s_last, self._m_last = s, m
                return
            dm = m - self._m_last
            if dm > 1e-6:
                inst = (s - self._s_last) / dm
                a = min(1.0, dm / RATIO_TAU_S)
                self._ratio = max(0.0, min(2.0, (1.0 - a) * self._ratio + a * inst))
            self._s_last, self._m_last = s, m

    # ----------------------------------------------------------------- read
    def now(self, mono_s: Optional[float] = None) -> float:
        """The clock value [s]; ``time.monotonic()`` until the first feed."""
        m = time.monotonic() if mono_s is None else float(mono_s)
        with self._lock:
            if not self._fed:
                return m
            v = max(self._value(m), self._floor)
            self._floor = v
            return v

    def _value(self, m: float) -> float:
        base = self._v_ref + (self._s_last - self._s_ref)
        gap = max(0.0, m - self._m_last)
        interp = min(gap, self._stall_s)
        return base + interp * min(self._ratio, 1.5) + max(0.0, gap - self._stall_s)

    @property
    def sim_paced(self) -> bool:
        """True once a /clock feed drives the clock."""
        with self._lock:
            return self._fed

    @property
    def stalled(self) -> bool:
        """True while a fed clock has heard no /clock for ``stall_s``."""
        with self._lock:
            return self._fed and time.monotonic() - self._m_last > self._stall_s

    @property
    def ratio(self) -> Optional[float]:
        """The measured sim/wall ratio (the real-time factor), None unfed."""
        with self._lock:
            return self._ratio if self._fed else None


_CLOCK = StreamClock()


def stream_clock() -> StreamClock:
    """The process-wide clock (one GCS per process)."""
    return _CLOCK


def now() -> float:
    """``stream_clock().now()``: what every world-timing rule reads."""
    return _CLOCK.now()
