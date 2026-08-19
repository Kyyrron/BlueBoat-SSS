"""Dual-handle range slider (start/end selection on one bar).

Qt has no built-in two-handle slider; this is a compact custom widget:
a groove with two draggable handles bounding a highlighted span.
Values are integers in [minimum, maximum]; ``range_changed(lo, hi)`` is
emitted on every change; handles cannot cross.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QRect, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QSizePolicy, QWidget

_HANDLE_W = 12
_GROOVE_H = 6


class RangeSlider(QWidget):
    """Horizontal slider with two handles selecting [low, high]."""

    range_changed = Signal(int, int)

    def __init__(self, minimum: int = 0, maximum: int = 1000,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._min, self._max = minimum, maximum
        self._lo, self._hi = minimum, maximum
        self._drag: Optional[str] = None            # "lo" | "hi"
        self.setMinimumHeight(26)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setCursor(Qt.PointingHandCursor)

    # ---- API -------------------------------------------------------------------
    def set_range(self, minimum: int, maximum: int) -> None:
        self._min, self._max = minimum, max(maximum, minimum + 1)
        self._lo, self._hi = self._min, self._max
        self.update()
        self.range_changed.emit(self._lo, self._hi)

    def values(self) -> tuple:
        return self._lo, self._hi

    def set_values(self, lo: int, hi: int) -> None:
        lo = max(self._min, min(lo, self._max))
        hi = max(self._min, min(hi, self._max))
        self._lo, self._hi = min(lo, hi), max(lo, hi)
        self.update()
        self.range_changed.emit(self._lo, self._hi)

    # ---- geometry ---------------------------------------------------------------
    def _x_of(self, value: int) -> int:
        span = max(self._max - self._min, 1)
        usable = self.width() - 2 * _HANDLE_W
        return _HANDLE_W + int((value - self._min) / span * usable)

    def _value_of(self, x: int) -> int:
        usable = max(self.width() - 2 * _HANDLE_W, 1)
        frac = (x - _HANDLE_W) / usable
        return int(round(self._min + max(0.0, min(1.0, frac))
                         * (self._max - self._min)))

    # ---- painting --------------------------------------------------------------------
    def paintEvent(self, event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cy = self.height() // 2
        groove = QRect(_HANDLE_W, cy - _GROOVE_H // 2,
                       self.width() - 2 * _HANDLE_W, _GROOVE_H)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#233041"))
        p.drawRoundedRect(groove, 3, 3)
        x_lo, x_hi = self._x_of(self._lo), self._x_of(self._hi)
        p.setBrush(QColor("#3a76d6"))
        p.drawRoundedRect(QRect(x_lo, cy - _GROOVE_H // 2,
                                max(x_hi - x_lo, 2), _GROOVE_H), 3, 3)
        for x in (x_lo, x_hi):
            p.setBrush(QColor("#c7d0d9"))
            p.setPen(QPen(QColor("#10151b"), 1))
            p.drawEllipse(x - _HANDLE_W // 2, cy - _HANDLE_W // 2,
                          _HANDLE_W, _HANDLE_W)

    # ---- interaction ----------------------------------------------------------------
    def mousePressEvent(self, event) -> None:  # noqa: N802
        x = int(event.position().x())
        # Grab whichever handle is nearer (ties -> the one that can move).
        d_lo, d_hi = abs(x - self._x_of(self._lo)), abs(x - self._x_of(self._hi))
        self._drag = "lo" if d_lo < d_hi else "hi"
        self.mouseMoveEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if self._drag is None:
            return
        v = self._value_of(int(event.position().x()))
        if self._drag == "lo":
            self._lo = min(max(self._min, v), self._hi)
        else:
            self._hi = max(min(self._max, v), self._lo)
        self.update()
        self.range_changed.emit(self._lo, self._hi)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        self._drag = None
