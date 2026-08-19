"""Central map widget.

A ``QGraphicsView`` whose scene lives in world metres (scene y = -world y,
see gui/map_layers.py). This is the decision that makes everything else
simple: pan/zoom are native view transforms, click -> world coordinates is
``mapToScene``, and layers are independent QGraphicsItems — the same model
used by professional GCS tools.

Rendering smoothness: the view uses full-viewport updates with
antialiasing and smooth pixmap sampling; raster refreshes swap a pixmap
on an existing item (no scene rebuild), so there is no flicker.

Interactions
------------
* wheel        : zoom, anchored under the cursor
* left drag    : pan (hand cursor)
* left click   : inspect point (``point_clicked`` -> panels show robot +
                 GPS coordinates with copy buttons)
* measure mode : two clicks -> line + distance (``measure_done``)
"""

from __future__ import annotations

import enum
import math
from typing import Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QFont, QPainter, QPen
from PySide6.QtWidgets import QGraphicsScene, QGraphicsView

from . import theme
from .map_layers import s2w, w2s

_ZOOM_STEP = 1.25
_MIN_SCALE = 0.02    # px per metre (whole-km view)
_MAX_SCALE = 400.0   # px per metre (cm-level inspection)
_CLICK_SLOP_PX = 5   # press/release within this distance counts as a click
_GRID_STEPS_M = (0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000)


class MapMode(enum.Enum):
    NAVIGATE = enum.auto()
    MEASURE = enum.auto()


class MapView(QGraphicsView):
    """Interactive world-frame view hosting all map layers."""

    point_clicked = Signal(float, float)                       # world x, y
    measure_started = Signal(float, float)                     # first point
    measure_done = Signal(float, float, float, float, float)   # x1 y1 x2 y2 d
    viewport_changed = Signal(QRectF, float)  # world rect (y-up), m per px
    mode_changed = Signal(object)             # MapMode

    def __init__(self) -> None:
        super().__init__()
        self.setScene(QGraphicsScene(self))
        self.setBackgroundBrush(theme.COLOR_BACKGROUND)
        self.setRenderHints(QPainter.Antialiasing |
                            QPainter.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setMouseTracking(True)
        # A generous fixed scene rect keeps free panning possible even when
        # few items exist yet (auto sceneRect would clamp the view).
        self.scene().setSceneRect(-20000, -20000, 40000, 40000)
        self.scale(6.0, 6.0)  # ~6 px per metre initial view

        self._mode = MapMode.NAVIGATE
        self._press_pos = None  # type: Optional[QPointF]
        self._measure_first: Optional[Tuple[float, float]] = None

    # ---- public API -----------------------------------------------------------
    @property
    def mode(self) -> MapMode:
        return self._mode

    def set_mode(self, mode: MapMode) -> None:
        if mode is self._mode:
            return
        self._mode = mode
        self._measure_first = None
        self.setCursor(Qt.CrossCursor if mode is MapMode.MEASURE
                       else Qt.ArrowCursor)
        self.mode_changed.emit(mode)

    def metres_per_pixel(self) -> float:
        return 1.0 / self.transform().m11()

    def zoom(self, factor: float) -> None:
        new_scale = self.transform().m11() * factor
        if _MIN_SCALE <= new_scale <= _MAX_SCALE:
            self.scale(factor, factor)
            self._emit_viewport()

    def zoom_in(self) -> None:
        self.zoom(_ZOOM_STEP)

    def zoom_out(self) -> None:
        self.zoom(1.0 / _ZOOM_STEP)

    def center_on_world(self, x: float, y: float) -> None:
        """One-shot centering (used by 'Center robot'); the camera stays
        free afterwards — there is deliberately no follow mode."""
        self.centerOn(w2s(x, y))
        self._emit_viewport()

    def world_viewport_rect(self) -> QRectF:
        """Visible area in world metres, y-up (normalized rect)."""
        scene_rect = self.mapToScene(self.viewport().rect()).boundingRect()
        x0, y_top = s2w(scene_rect.topLeft())
        x1, y_bot = s2w(scene_rect.bottomRight())
        return QRectF(QPointF(x0, min(y_top, y_bot)),
                      QPointF(x1, max(y_top, y_bot))).normalized()

    # ---- events -----------------------------------------------------------------
    def wheelEvent(self, event) -> None:  # noqa: N802 (Qt override)
        self.zoom(_ZOOM_STEP if event.angleDelta().y() > 0 else 1 / _ZOOM_STEP)

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if event.button() == Qt.LeftButton:
            self._press_pos = event.position()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        super().mouseReleaseEvent(event)
        if event.button() != Qt.LeftButton or self._press_pos is None:
            return
        moved = (event.position() - self._press_pos).manhattanLength()
        self._press_pos = None
        if moved > _CLICK_SLOP_PX:
            self._emit_viewport()  # it was a pan
            return
        wx, wy = s2w(self.mapToScene(event.position().toPoint()))
        if self._mode is MapMode.MEASURE:
            self._handle_measure_click(wx, wy)
        else:
            self.point_clicked.emit(wx, wy)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        super().mouseMoveEvent(event)
        if self._press_pos is not None:  # panning in progress
            self._emit_viewport()

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._emit_viewport()

    def _handle_measure_click(self, wx: float, wy: float) -> None:
        if self._measure_first is None:
            self._measure_first = (wx, wy)
            self.measure_started.emit(wx, wy)
        else:
            x1, y1 = self._measure_first
            self._measure_first = None
            dist = math.hypot(wx - x1, wy - y1)
            self.measure_done.emit(x1, y1, wx, wy, dist)

    def _emit_viewport(self) -> None:
        self.viewport_changed.emit(self.world_viewport_rect(),
                                   self.metres_per_pixel())

    # ---- adaptive X/Y grid overlay -----------------------------------------------
    def drawForeground(self, painter: QPainter, rect: QRectF) -> None:  # noqa: N802
        step = self._grid_step()
        pen = QPen(theme.COLOR_GRID, 0)
        pen.setCosmetic(True)
        painter.setPen(pen)

        x0 = math.floor(rect.left() / step) * step
        y0 = math.floor(rect.top() / step) * step
        xs, ys = [], []
        x = x0
        while x <= rect.right():
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
            xs.append(x)
            x += step
        y = y0
        while y <= rect.bottom():
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            ys.append(y)
            y += step

        # Labels in device space so they stay a constant size on screen.
        painter.save()
        painter.resetTransform()
        painter.setPen(QPen(theme.COLOR_GRID_TEXT))
        painter.setFont(QFont("DejaVu Sans", 7))
        h = self.viewport().height()
        for x in xs:
            p = self.mapFromScene(QPointF(x, 0))
            painter.drawText(p.x() + 3, h - 6, self._fmt(x))
        for y in ys:
            p = self.mapFromScene(QPointF(0, y))
            painter.drawText(4, p.y() - 3, self._fmt(-y))  # scene->world y
        painter.restore()

    def _grid_step(self) -> float:
        target_px = 90.0  # aim for a line roughly every ~90 px
        target_m = target_px * self.metres_per_pixel()
        for step in _GRID_STEPS_M:
            if step >= target_m:
                return step
        return _GRID_STEPS_M[-1]

    @staticmethod
    def _fmt(v: float) -> str:
        return f"{v:.1f}" if abs(v) < 10 else f"{v:.0f}"
