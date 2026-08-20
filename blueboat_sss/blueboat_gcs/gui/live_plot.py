"""Lightweight real-time scrolling plot (no external plotting library).

A bounded time series painted as a polyline in ``paintEvent``: appends
are O(1) into a deque, repaints are coalesced to ``MAX_FPS``, and the y
axis autoscales with hysteresis so the trace doesn't jitter. Designed as
a reusable building block — the "Robot Altitude" plot is simply
``LivePlot(title="Robot Altitude", unit="m", window_s=60)``; future live
plots (speed, SNR, ping rate…) instantiate the same class and connect a
different signal.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Deque, Optional, Tuple

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QSizePolicy, QWidget

from . import theme

MAX_FPS = 8.0                 # repaint coalescing
_MARGIN_L, _MARGIN_B = 34, 14


class LivePlot(QWidget):
    """Scrolling y(t) plot over the last ``window_s`` seconds."""

    def __init__(self, title: str, unit: str = "", window_s: float = 60.0,
                 color: QColor = None,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._title = title
        self._unit = unit
        self._window = window_s
        self._color = color or QColor("#4dd0e1")
        self._data: Deque[Tuple[float, float]] = deque(maxlen=4096)
        self._t0: Optional[float] = None
        self._last_paint = 0.0
        self._ylim: Optional[Tuple[float, float]] = None
        self.setMinimumHeight(96)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    # ---- data ------------------------------------------------------------------
    def append(self, value: float) -> None:
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        self._data.append((now, float(value)))
        # Drop samples that scrolled out of the window.
        cutoff = now - self._window
        while self._data and self._data[0][0] < cutoff:
            self._data.popleft()
        if now - self._last_paint >= 1.0 / MAX_FPS:
            self._last_paint = now
            self.update()

    def clear(self) -> None:
        self._data.clear()
        self._ylim = None
        self.update()

    # ---- painting ---------------------------------------------------------------
    def _y_limits(self) -> Tuple[float, float]:
        values = [v for _, v in self._data]
        lo, hi = min(values), max(values)
        if hi - lo < 0.2:                       # flat trace: give it air
            mid = 0.5 * (hi + lo)
            lo, hi = mid - 0.15, mid + 0.15
        pad = 0.12 * (hi - lo)
        lo, hi = lo - pad, hi + pad
        # Hysteresis: only rescale when the trace leaves the current
        # limits or uses less than half of them — no per-frame jitter.
        if self._ylim is not None:
            plo, phi = self._ylim
            if lo >= plo and hi <= phi and (hi - lo) > 0.4 * (phi - plo):
                return self._ylim
        self._ylim = (lo, hi)
        return self._ylim

    def paintEvent(self, event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor("#10151b"))
        p.setPen(QPen(theme.COLOR_GRID_TEXT))
        p.setFont(QFont("DejaVu Sans", 8))
        p.drawText(_MARGIN_L, 12, self._title)
        if len(self._data) < 2:
            p.drawText(_MARGIN_L, h // 2, "waiting for data…")
            return

        lo, hi = self._y_limits()
        now = self._data[-1][0]
        x0, y0 = _MARGIN_L, 16
        pw, ph = w - x0 - 6, h - y0 - _MARGIN_B

        # Axis labels + gridlines (min / max / latest).
        grid_pen = QPen(QColor(255, 255, 255, 18))
        for frac, val in ((0.0, hi), (1.0, lo)):
            yy = y0 + frac * ph
            p.setPen(grid_pen)
            p.drawLine(x0, yy, x0 + pw, yy)
            p.setPen(QPen(theme.COLOR_GRID_TEXT))
            p.drawText(2, int(yy) + 4, f"{val:.1f}")
        p.drawText(x0, h - 2, f"last {self._window:.0f} s")
        latest = self._data[-1][1]
        txt = f"{latest:.2f} {self._unit}".strip()
        p.setPen(QPen(self._color))
        p.drawText(w - p.fontMetrics().horizontalAdvance(txt) - 6, 12, txt)

        # Trace.
        pts = []
        span = max(hi - lo, 1e-9)
        for t, v in self._data:
            fx = 1.0 - (now - t) / self._window
            if fx < 0.0:
                continue
            pts.append(QPointF(x0 + fx * pw,
                               y0 + (1.0 - (v - lo) / span) * ph))
        pen = QPen(self._color, 1.4)
        pen.setCosmetic(True)
        p.setPen(pen)
        p.drawPolyline(pts)
