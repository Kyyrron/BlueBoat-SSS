"""Interactive waterfall view: raw pings stacked in acquisition order.

Rebuilt on ``QGraphicsView`` (previously a static paint widget) to give
the interaction professional sonar software offers:

* mouse-wheel zoom anchored under the cursor (0.25×–16×);
* drag pan + vertical scrollbar through the whole ring buffer;
* **pin-to-newest**: while the view is at the bottom it follows the
  incoming pings like a paper recorder; the moment the operator scrolls
  up to inspect history the pinning releases, and clicking "Follow" (or
  scrolling back to the bottom) re-engages it — no fighting the user
  for the camera, same philosophy as the map's one-shot centring.

Axes remain ping index (vertical) × across-track (horizontal); the
overlay (port/starboard labels, current range, nadir line, follow state)
is drawn in ``drawForeground`` in device coordinates so it never scales
with the imagery.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QRectF, Qt, Signal
from PySide6.QtGui import (QColor, QFont, QImage, QPainter, QPen, QPixmap,
                           QWheelEvent)
from PySide6.QtWidgets import (QGraphicsPixmapItem, QGraphicsScene,
                               QGraphicsView)

from . import theme

_MIN_SCALE = 0.25
_MAX_SCALE = 16.0
_PIN_TOLERANCE_PX = 8       # "at the bottom" slack before unpinning


class WaterfallView(QGraphicsView):
    """Zoomable / scrollable display of the WaterfallService output."""

    follow_changed = Signal(bool)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._item = QGraphicsPixmapItem()
        self._item.setTransformationMode(Qt.SmoothTransformation)
        self._scene.addItem(self._item)

        self.setBackgroundBrush(theme.COLOR_BACKGROUND)
        self.setRenderHints(QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.SmartViewportUpdate)

        self._range_m = 0.0
        self._follow = True
        self._fitted_once = False
        self._detections: list = []      # {"row", "col", "label"} in px
        self._show_detections = True
        self._build_controls()

    def _build_controls(self) -> None:
        """Corner control strip: manual zoom −/+ and the AI-detections
        toggle. Child widgets of the view, so both the main window's and
        the replay window's waterfalls get them with zero extra wiring."""
        from PySide6.QtWidgets import (QCheckBox, QHBoxLayout, QPushButton,
                                       QWidget)
        self._controls = QWidget(self)
        lay = QHBoxLayout(self._controls)
        lay.setContentsMargins(4, 2, 4, 2)
        lay.setSpacing(4)
        zoom_out = QPushButton("−")
        zoom_in = QPushButton("+")
        for b, tip in ((zoom_out, "Zoom out"), (zoom_in, "Zoom in")):
            b.setFixedSize(26, 22)
            b.setToolTip(tip)
        zoom_in.clicked.connect(self.zoom_in)
        zoom_out.clicked.connect(self.zoom_out)
        self._det_check = QCheckBox("AI detections")
        self._det_check.setChecked(True)
        self._det_check.setToolTip(
            "Show / hide AI detection markers on the waterfall.")
        self._det_check.toggled.connect(self._set_show_detections)
        lay.addWidget(zoom_out)
        lay.addWidget(zoom_in)
        lay.addWidget(self._det_check)
        self._controls.setStyleSheet(
            "QWidget{background: rgba(16,21,27,190); border-radius: 4px;}")
        self._controls.adjustSize()
        self._controls.move(8, 24)
        self._controls.raise_()

    # ---- manual zoom -----------------------------------------------------------
    def _apply_zoom(self, step: float) -> None:
        current = self.transform().m11()
        target = max(_MIN_SCALE, min(_MAX_SCALE, current * step))
        factor = target / current
        if abs(factor - 1.0) > 1e-9:
            self.scale(factor, factor)
        self._update_follow_from_scrollbar()

    def zoom_in(self) -> None:
        self._apply_zoom(1.25)

    def zoom_out(self) -> None:
        self._apply_zoom(1 / 1.25)

    def _set_show_detections(self, on: bool) -> None:
        self._show_detections = on
        self.viewport().update()

    def on_detections(self, dets: list) -> None:
        """Detection overlay from WaterfallService (buffer pixel coords)."""
        self._detections = list(dets)
        self.viewport().update()

    # ---- data slot -----------------------------------------------------------
    def on_image(self, image: QImage, range_m: float) -> None:
        if image.isNull():                    # "Clear SSS data"
            self._item.setPixmap(QPixmap())
            self._scene.setSceneRect(QRectF())
            self._fitted_once = False
            self.viewport().update()
            return
        self._range_m = range_m
        self._item.setPixmap(QPixmap.fromImage(image))
        self._scene.setSceneRect(self._item.boundingRect())
        if not self._fitted_once:
            self._fit_width()
            self._fitted_once = True
        if self._follow:
            self._scroll_to_bottom()
        self.viewport().update()

    def set_follow(self, follow: bool) -> None:
        self._follow = follow
        if follow:
            self._scroll_to_bottom()
        self.follow_changed.emit(follow)

    # ---- interaction -----------------------------------------------------------
    def wheelEvent(self, event: QWheelEvent) -> None:  # noqa: N802
        self._apply_zoom(1.25 if event.angleDelta().y() > 0 else 1 / 1.25)

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
        factor = max(_MIN_SCALE, min(
            _MAX_SCALE,
            (self.viewport().width() - margin) / rect.width()))
        self.scale(factor, factor)

    # ---- overlay ------------------------------------------------------------------
    def drawForeground(self, painter: QPainter, rect) -> None:  # noqa: N802
        if self._item.pixmap().isNull():
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
