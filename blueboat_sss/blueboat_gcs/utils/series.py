"""Timestamped, growable numpy series with efficient time-window slicing.

Ported verbatim from BlueBoat-MCS (``mcs/core/series.py``) as part of the
GPS-anchored map architecture. Appending is amortised O(1) (capacity
doubling); windowed reads return numpy views, never copies of the whole
history.
"""

from __future__ import annotations

import numpy as np


class TimeSeries:
    """A (t, value[dim]) series ordered by monotonically increasing time."""

    def __init__(self, dim: int, initial_capacity: int = 4096) -> None:
        self._dim = dim
        self._t = np.empty(initial_capacity, dtype=np.float64)
        self._v = np.empty((initial_capacity, dim), dtype=np.float64)
        self._n = 0

    # ------------------------------------------------------------------ API
    def __len__(self) -> int:
        return self._n

    @property
    def dim(self) -> int:
        return self._dim

    def clear(self) -> None:
        self._n = 0

    def append(self, t: float, value) -> None:
        if self._n == len(self._t):
            self._grow()
        self._t[self._n] = t
        self._v[self._n] = value
        self._n += 1

    def t(self) -> np.ndarray:
        """View of all timestamps."""
        return self._t[: self._n]

    def v(self) -> np.ndarray:
        """View of all values, shape (n, dim)."""
        return self._v[: self._n]

    def last(self) -> tuple[float, np.ndarray] | None:
        if self._n == 0:
            return None
        i = self._n - 1
        return float(self._t[i]), self._v[i]

    def t_range(self) -> tuple[float, float] | None:
        if self._n == 0:
            return None
        return float(self._t[0]), float(self._t[self._n - 1])

    def window(self, t0: float, t1: float) -> tuple[np.ndarray, np.ndarray]:
        """Views of (timestamps, values) with t0 <= t <= t1 (binary search)."""
        ts = self.t()
        i0 = int(np.searchsorted(ts, t0, side="left"))
        i1 = int(np.searchsorted(ts, t1, side="right"))
        return ts[i0:i1], self._v[i0:i1]

    def decimated_window(
        self, t0: float, t1: float, max_points: int
    ) -> tuple[np.ndarray, np.ndarray]:
        """Windowed views, stride-decimated so at most ``max_points`` remain."""
        ts, vs = self.window(t0, t1)
        n = len(ts)
        if n <= max_points or max_points <= 0:
            return ts, vs
        stride = int(np.ceil(n / max_points))
        return ts[::stride], vs[::stride]

    # ------------------------------------------------------------- internal
    def _grow(self) -> None:
        cap = max(len(self._t) * 2, 4096)
        t = np.empty(cap, dtype=np.float64)
        v = np.empty((cap, self._dim), dtype=np.float64)
        t[: self._n] = self._t[: self._n]
        v[: self._n] = self._v[: self._n]
        self._t, self._v = t, v
