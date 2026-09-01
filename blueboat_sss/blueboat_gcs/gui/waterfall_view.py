"""Interactive waterfall view: raw pings stacked in acquisition order.

Built on ``QGraphicsView`` to give the interaction professional sonar
software offers:

* mouse-wheel zoom anchored under the cursor, down to a dynamic minimum
  that always lets the WHOLE mission fit the viewport (plus a "Fit file"
  button that jumps there);
* drag pan + scrollbars through the entire buffered history — the
  service grows with the mission, so scrolling reaches the first ping;
* **pin-to-newest**: while the view is at the bottom it follows the
  incoming pings like a paper recorder; the moment the operator scrolls
  up to inspect history the pinning releases, and scrolling back to the
  bottom re-engages it — no fighting the user for the camera;
* **click to locate**: a click (as opposed to a drag) emits
  ``point_selected(row, col)`` in absolute buffer coordinates; the
  windows resolve it to a world position + GPS and mark it on the map.
  The selected point is drawn as a crosshair via :meth:`set_selected`.

Display model: the service emits fixed-height tiles
(``tile_updated(first_row, image)``), each backed by its own pixmap
item positioned at its absolute row, plus ``layout_changed(row0,
total_rows, cols, range_m)`` for the scene rect. Scene coordinates ==
absolute buffer pixels: x = column, y = ping index — which is what
keeps ``mapToScene`` on a click a direct row/col lookup.

The overlay (port/starboard labels, current range, nadir line, follow
state, detection markers, selection crosshair) is drawn in
``drawForeground`` in device coordinates so it never scales with the
imagery.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import (QColor, QFont, QImage, QMouseEvent, QPainter,
                           QPen, QPixmap, QWheelEvent)
from PySide6.QtWidgets import (QGraphicsPixmapItem, QGraphicsScene,
                               QGraphicsView)

from . import theme

_ABS_MIN_SCALE = 1e-4       # hard floor under the dynamic fit-file minimum
_MIN_SCALE = 0.25           # minimum when the content is small
_MAX_SCALE = 16.0
_PIN_TOLERANCE_PX = 8       # "at the bottom" slack before unpinning
_CLICK_SLOP_PX = 5          # press->release travel below this is a click


class WaterfallView(QGraphicsView):
    """Zoomable / scrollable display of the WaterfallService output."""

    follow_changed = Signal(bool)
    #: A genuine click (not a pan): absolute buffer (row, col).
    point_selected = Signal(int, int)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        #: absolute first_row -> pixmap item, one per service tile.
        self._items: Dict[int, QGraphicsPixmapItem] = {}

        self.setBackgroundBrush(theme.COLOR_BACKGROUND)
        self.setRenderHints(QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.SmartViewportUpdate)

        self._range_m = 0.0
        self._row0 = 0
        self._total = 0
        self._cols = 0
        self._follow = True
        self._fitted_once = False
        self._press_pos = None
        self._detections: list = []      # {"row", "col", "label"} absolute px
        self._show_detections = True
        self._selected: Optional[Tuple[int, int]] = None
        self._build_controls()

    def _build_controls(self) -> None:
        """Corner control strip: manual zoom −/+, fit-file, and the
        AI-detections toggle. Child widgets of the view, so both the main
        window's and the replay window's waterfalls get them with zero
        extra wiring."""
        from PySide6.QtWidgets import (QCheckBox, QHBoxLayout, QPushButton,
                                       QWidget)
        self._controls = QWidget(self)
        lay = QHBoxLayout(self._controls)
        lay.setContentsMargins(4, 2, 4, 2)
        lay.setSpacing(4)
        zoom_out = QPushButton("−")
        zoom_in = QPushButton("+")
        fit_btn = QPushButton("Fit file")
        for b, tip in ((zoom_out, "Zoom out"), (zoom_in, "Zoom in")):
            b.setFixedSize(26, 22)
            b.setToolTip(tip)
        fit_btn.setFixedHeight(22)
        fit_btn.setToolTip(
            "Zoom out until the entire buffered mission is visible.")
        zoom_in.clicked.connect(self.zoom_in)
        zoom_out.clicked.connect(self.zoom_out)
        fit_btn.clicked.connect(self.fit_file)
        self._det_check = QCheckBox("AI detections")
        self._det_check.setChecked(True)
        self._det_check.setToolTip(
            "Show / hide AI detection markers on the waterfall.")
        self._det_check.toggled.connect(self._set_show_detections)
        lay.addWidget(zoom_out)
        lay.addWidget(zoom_in)
        lay.addWidget(fit_btn)
        lay.addWidget(self._det_check)
        self._controls.setStyleSheet(
            "QWidget{background: rgba(16,21,27,190); border-radius: 4px;}")
        self._controls.adjustSize()
        self._controls.move(8, 24)
        self._controls.raise_()

    # ---- manual zoom -----------------------------------------------------------
    def _min_scale(self) -> float:
        """Dynamic zoom floor: never below what fits the whole mission.

        The old fixed 0.25 minimum made a long log impossible to survey
        at a glance; the floor now follows the scene height."""
        rect = self._scene.sceneRect()
        if rect.height() <= 0 or rect.width() <= 0:
            return _MIN_SCALE
        margin = 16
        fit = min((self.viewport().width() - margin) / rect.width(),
                  (self.viewport().height() - margin) / rect.height())
        return max(_ABS_MIN_SCALE, min(_MIN_SCALE, fit))

    def _apply_zoom(self, step: float) -> None:
        current = self.transform().m11()
        target = max(self._min_scale(), min(_MAX_SCALE, current * step))
        factor = target / current
        if abs(factor - 1.0) > 1e-9:
            self.scale(factor, factor)
        self._update_follow_from_scrollbar()

    def zoom_in(self) -> None:
        self._apply_zoom(1.25)

    def zoom_out(self) -> None:
        self._apply_zoom(1 / 1.25)

    def fit_file(self) -> None:
        """One shot: the entire buffered mission inside the viewport."""
        rect = self._scene.sceneRect()
        if rect.isEmpty():
            return
        self.resetTransform()
        factor = self._min_scale()
        self.scale(factor, factor)
        self.centerOn(rect.center())
        self._update_follow_from_scrollbar()

    def _set_show_detections(self, on: bool) -> None:
        self._show_detections = on
        self.viewport().update()

    def on_detections(self, dets: list) -> None:
        """Detection overlay from WaterfallService (absolute buffer px)."""
        self._detections = list(dets)
        self.viewport().update()

    # ---- selection --------------------------------------------------------------
    def set_selected(self, row: Optional[int], col: Optional[int] = None
                     ) -> None:
        """Show (or clear, with row=None) the selected-point crosshair."""
        self._selected = None if row is None else (int(row), int(col))
        self.viewport().update()

    # ---- data slots -----------------------------------------------------------
    def on_layout(self, row0: int, total: int, cols: int,
                  range_m: float) -> None:
        """Buffer geometry from the service; scene y = absolute row."""
        self._range_m = range_m
        self._row0, self._total, self._cols = row0, total, cols
        if total <= row0:                     # cleared
            for item in self._items.values():
                self._scene.removeItem(item)
            self._items.clear()
            self._scene.setSceneRect(QRectF())
            self._fitted_once = False
            self._selected = None
            self.viewport().update()
            return
        # Tiles evicted by the memory cap: drop their items.
        for first_row in [k for k in self._items if k < row0]:
            self._scene.removeItem(self._items.pop(first_row))
        self._scene.setSceneRect(QRectF(0, row0, cols, total - row0))
        if not self._fitted_once:
            self._fit_width()
            self._fitted_once = True
        if self._follow:
            self._scroll_to_bottom()
        self.viewport().update()

    def on_tile(self, first_row: int, image: QImage) -> None:
        """One re-rendered tile from the service."""
        item = self._items.get(first_row)
        if item is None:
            item = QGraphicsPixmapItem()
            item.setTransformationMode(Qt.SmoothTransformation)
            item.setPos(QPointF(0, first_row))
            self._scene.addItem(item)
            self._items[first_row] = item
        item.setPixmap(QPixmap.fromImage(image))
        self.viewport().update()

    def set_follow(self, follow: bool) -> None:
        self._follow = follow
        if follow:
            self._scroll_to_bottom()
        self.follow_changed.emit(follow)

    # ---- interaction -----------------------------------------------------------
    def wheelEvent(self, event: QWheelEvent) -> None:  # noqa: N802
        self._apply_zoom(1.25 if event.angleDelta().y() > 0 else 1 / 1.25)

    def mousePressEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        if event.button() == Qt.LeftButton:
            self._press_pos = event.position().toPoint()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:  # noqa: N802
        # Click-vs-drag discrimination, same slop rule as MapView: the
        # hand-drag pan consumes larger movements; a short press-release
        # is a point selection.
        super().mouseReleaseEvent(event)
        if event.button() != Qt.LeftButton or self._press_pos is None:
            return
        pos = event.position().toPoint()
        moved = (pos - self._press_pos).manhattanLength()
        self._press_pos = None
        if moved > _CLICK_SLOP_PX or self._total <= self._row0:
            return
        sp = self.mapToScene(pos)
        row = int(sp.y())
        col = int(sp.x())
        if (self._row0 <= row < self._total
                and 0 <= col < self._cols):
            self.point_selected.emit(row, col)

    def scrollContentsBy(self, dx: int, dy: int) -> None:  # noqa: N802
        super().scrollContentsBy(dx, dy)
        self._update_follow_from_scrollbar()

    def _update_follow_from_scrollbar(self) -> None:
        """Pin when at the bottom, release when the user scrolls away."""
        bar = self.verticalScrollBar()
        at_bottom = bar.value() >= bar.maximum() - _PIN_TOLERANCE_PX
        if at_bottom != self._follow:
            self._follow = at_bottom
            self.follow_changed.emit(at_bottom)

    def _scroll_to_bottom(self) -> None:
        bar = self.verticalScrollBar()
        bar.setValue(bar.maximum())

    def _fit_width(self) -> None:
        rect = self._scene.sceneRect()
        if rect.width() <= 0:
            return
        self.resetTransform()
        margin = 16
        factor = max(self._min_scale(), min(
            _MAX_SCALE,
            (self.viewport().width() - margin) / rect.width()))
        self.scale(factor, factor)

    # ---- overlay ------------------------------------------------------------------
    def drawForeground(self, painter: QPainter, rect) -> None:  # noqa: N802
        if not self._items:
            painter.resetTransform()
            painter.setPen(QPen(theme.COLOR_GRID_TEXT))
            painter.drawText(self.viewport().rect(), Qt.AlignCenter,
                             "Waterfall — waiting for sonar pings…")
            return
        # Nadir line follows the imagery (scene coordinates).
        mid_x = self._scene.sceneRect().center().x()
        pen = QPen(QColor(255, 255, 255, 60), 0)
        pen.setCosmetic(True)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.drawLine(mid_x, rect.top(), mid_x, rect.bottom())
        # Detection markers: positioned in image (scene) coordinates,
        # drawn at fixed device size so they never scale with zoom —
        # same visual language as the map's DetectionLayer.
        if self._detections and self._show_detections:
            painter.save()
            painter.resetTransform()
            painter.setFont(QFont("DejaVu Sans", 8))
            for det in self._detections:
                pt = self.mapFromScene(det["col"] + 0.5, det["row"] + 0.5)
                pen = QPen(theme.COLOR_DETECTION, 1.6)
                painter.setPen(pen)
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(pt, 9, 9)
                painter.drawLine(pt.x() - 13, pt.y(), pt.x() - 5, pt.y())
                painter.drawLine(pt.x() + 5, pt.y(), pt.x() + 13, pt.y())
                painter.drawText(pt.x() + 12, pt.y() - 8, det["label"])
            painter.restore()
        # Selected-point crosshair (waterfall click <-> map selection).
        if self._selected is not None:
            painter.save()
            painter.resetTransform()
            pt = self.mapFromScene(self._selected[1] + 0.5,
                                   self._selected[0] + 0.5)
            pen = QPen(QColor(80, 220, 255), 1.8)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(pt, 7, 7)
            painter.drawLine(pt.x() - 14, pt.y(), pt.x() - 4, pt.y())
            painter.drawLine(pt.x() + 4, pt.y(), pt.x() + 14, pt.y())
            painter.drawLine(pt.x(), pt.y() - 14, pt.x(), pt.y() - 4)
            painter.drawLine(pt.x(), pt.y() + 4, pt.x(), pt.y() + 14)
            painter.restore()
        # Labels in device coordinates (never scale with zoom).
        painter.resetTransform()
        painter.setPen(QPen(theme.COLOR_GRID_TEXT))
        painter.setFont(QFont("DejaVu Sans", 9))
        w = self.viewport().width()
        h = self.viewport().height()
        painter.drawText(8, 16, f"PORT  ⟵  {self._range_m:.0f} m")
        txt = f"{self._range_m:.0f} m  ⟶  STARBOARD"
        painter.drawText(w - painter.fontMetrics().horizontalAdvance(txt) - 8,
                         16, txt)
        state = ("following newest ping ↓" if self._follow
                 else "history view — scroll to bottom to follow")
        painter.drawText(8, h - 8, state)
