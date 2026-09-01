"""QGraphicsScene layers composing the central map.

Scene convention (GPS-anchored, ported from BlueBoat-MCS): the scene is
**local east/north metres about the first accepted GPS fix**, north-up,
never rotated; scene = (east, -north) so north points up on screen.
Every layer converts through the module-level ``w2s``/``s2w`` helpers to
keep the y-flip in one place.

World-anchored layers (mosaic, trajectory, planned path, swath,
detections, pinger) keep drawing in the robot's world/odom metres but
live under one :class:`WorldRoot` group whose scene position IS the
odom->EN translation (``EN = world + t``, mapping/geo.py) — a refit
moves everything coherently in O(1), and nothing is shown until the
anchor is valid. Tiles and measurements live directly in the EN scene.

Layer stacking (z-values): satellite tiles are always *below* the SSS
mosaic — the sonar data remains the primary layer per the specification —
and annotations (trajectory, detections, pinger, measurements) sit above.
"""

from __future__ import annotations

import math
from typing import Callable, Dict, List, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (QBrush, QColor, QFont, QImage, QPainterPath, QPen,
                           QPixmap, QPolygonF, QTransform)
from PySide6.QtWidgets import (QGraphicsEllipseItem, QGraphicsItem,
                               QGraphicsItemGroup, QGraphicsLineItem,
                               QGraphicsPathItem, QGraphicsPixmapItem,
                               QGraphicsPolygonItem, QGraphicsScene,
                               QGraphicsSimpleTextItem)

from ..mapping.geo import latlon_to_local_en, local_en_to_latlon
from ..mapping.tiles import (TILE_SIZE_PX, TileFetcher, TileKey,
                             latlon_to_tile, tile_to_latlon,
                             zoom_for_resolution)
from ..models.detection import Detection, PingerFix
from . import theme

Z_TILES = -20.0
Z_MOSAIC = 0.0
Z_PLANNED_PATH = 5.0     # below the executed trajectory
Z_SWATH = 8.0            # sonar range line, just below the trajectory
Z_TRAJECTORY = 10.0
Z_ROBOT = 15.0
Z_DETECTIONS = 20.0
Z_PINGER = 25.0
Z_MEASURE = 30.0
Z_SELECTION = 35.0       # waterfall-selected point, above everything


def w2s(x: float, y: float) -> QPointF:
    """World metres -> scene coordinates."""
    return QPointF(x, -y)


def s2w(p: QPointF) -> Tuple[float, float]:
    """Scene coordinates -> world metres."""
    return p.x(), -p.y()


def _attach(scene: QGraphicsScene, item: "QGraphicsItem",
            parent: Optional["QGraphicsItem"]) -> None:
    """Add ``item`` to the scene, under ``parent`` when one is given."""
    if parent is not None:
        item.setParentItem(parent)
    else:
        scene.addItem(item)


# ---------------------------------------------------------------------------
class WorldRoot:
    """One parent item for every world-anchored layer.

    Its scene position IS the geo-fit translation: ``EN = world + t``
    (mapping/geo.py) becomes ``setPos(tx, -ty)`` under the scene's
    y-flip, so children keep drawing in plain world metres through
    ``w2s`` and a refit moves the mosaic, trajectory, detections,
    pinger, swath and planned path coherently in O(1).

    Its visibility is the map's readiness gate: hidden (with everything
    under it) until the GPS anchor is valid.
    """

    def __init__(self, scene: QGraphicsScene) -> None:
        self._group = QGraphicsItemGroup()
        self._group.setHandlesChildEvents(False)
        self._group.setVisible(False)      # gated until the anchor opens
        scene.addItem(self._group)

    @property
    def item(self) -> QGraphicsItemGroup:
        return self._group

    def set_translation(self, tx: float, ty: float) -> None:
        self._group.setPos(tx, -ty)

    def set_ready(self, ready: bool) -> None:
        self._group.setVisible(ready)


# ---------------------------------------------------------------------------
class MosaicLayer:
    """The processed SSS raster — primary layer of the application."""

    def __init__(self, scene: QGraphicsScene,
                 parent: Optional[QGraphicsItem] = None) -> None:
        self._item = QGraphicsPixmapItem()
        self._item.setZValue(Z_MOSAIC)
        self._item.setTransformationMode(Qt.SmoothTransformation)
        _attach(scene, self._item, parent)

    def update(self, image: QImage,
               extent: Tuple[float, float, float, float],
               cell_size_m: float) -> None:
        """Place the rendered raster at its world footprint."""
        xmin, _xmax, _ymin, ymax = extent
        self._item.setPixmap(QPixmap.fromImage(image))
        # Image row 0 is the top = world ymax -> scene y = -ymax.
        self._item.setPos(w2s(xmin, ymax))
        self._item.setTransform(QTransform.fromScale(cell_size_m, cell_size_m))

    def set_visible(self, visible: bool) -> None:
        self._item.setVisible(visible)

    def set_opacity(self, opacity: float) -> None:
        """SSS transparency over the map background. Pure compositing
        (QGraphicsItem opacity): real-time, smooth, data untouched."""
        self._item.setOpacity(max(0.0, min(1.0, opacity)))

    def clear(self) -> None:
        self._item.setPixmap(QPixmap())


# ---------------------------------------------------------------------------
class TrajectoryLayer:
    """Robot track since START + heading marker at the current pose."""

    _MARKER = QPolygonF([QPointF(1.4, 0.0), QPointF(-0.9, 0.7),
                         QPointF(-0.5, 0.0), QPointF(-0.9, -0.7)])

    def __init__(self, scene: QGraphicsScene,
                 parent: Optional[QGraphicsItem] = None) -> None:
        pen = QPen(theme.COLOR_TRAJECTORY, 0)  # cosmetic: 1 px at any zoom
        pen.setCosmetic(True)
        pen.setWidthF(1.6)
        self._path_item = QGraphicsPathItem()
        self._path_item.setPen(pen)
        self._path_item.setZValue(Z_TRAJECTORY)
        _attach(scene, self._path_item, parent)

        self._marker = QGraphicsPolygonItem(self._MARKER)
        self._marker.setBrush(QBrush(theme.COLOR_ROBOT))
        mpen = QPen(theme.COLOR_ROBOT_OUTLINE, 0)
        mpen.setCosmetic(True)
        mpen.setWidthF(1.5)
        self._marker.setPen(mpen)
        self._marker.setZValue(Z_ROBOT)
        _attach(scene, self._marker, parent)

        self._points: List[Tuple[float, float]] = []
        self._visible = True
        self._break_next = False

    def begin_new_segment(self) -> None:
        """Break the polyline before the next pose.

        Armed on START *and* by the telemetry staleness watchdog: if the
        boat moved while no data was flowing, no phantom straight
        segment is drawn between the old and the new position — the
        displayed history is preserved and the marker re-syncs on the
        next message, exactly as if the application had just launched.
        """
        self._break_next = True

    def set_stale(self, stale: bool) -> None:
        """Dim the robot marker while telemetry is not flowing, so the
        operator can see the displayed pose is no longer live."""
        self._marker.setOpacity(0.35 if stale else 1.0)

    #: A pose jump larger than this starts a new polyline segment even if
    #: no state machine armed a break — the stateless safety net that
    #: guarantees recovery (teleport, no phantom line) whenever
    #: localization resumes after a silent displacement.
    JUMP_BREAK_M = 5.0

    def add_pose(self, x: float, y: float, yaw: float) -> None:
        jump = (self._points
                and math.hypot(x - self._points[-1][0],
                               y - self._points[-1][1]) > self.JUMP_BREAK_M)
        self._points.append((x, y))
        # Rebuilding a QPainterPath for tens of thousands of points every
        # ping would be wasteful; append incrementally instead.
        path = self._path_item.path()
        if path.elementCount() == 0 or self._break_next or jump:
            path.moveTo(w2s(x, y))     # start a new subpath (no joining line)
            self._break_next = False
        else:
            path.lineTo(w2s(x, y))
        self._path_item.setPath(path)
        self._marker.setPos(w2s(x, y))
        # Scene y is flipped, so the on-screen rotation is -yaw.
        self._marker.setRotation(-math.degrees(yaw))

    def current_pos(self) -> Optional[Tuple[float, float]]:
        return self._points[-1] if self._points else None

    def clear(self) -> None:
        self._points.clear()
        self._path_item.setPath(QPainterPath())

    def set_visible(self, visible: bool) -> None:
        self._visible = visible
        self._path_item.setVisible(visible)
        # The robot marker stays visible: hiding the *trajectory* should not
        # hide the boat itself (operators always want to see the boat).


# ---------------------------------------------------------------------------
class DetectionLayer:
    """AI detection markers. Fully functional; fed by the placeholder
    listener (ros/detections_listener.py) or the simulator."""

    def __init__(self, scene: QGraphicsScene,
                 parent: Optional[QGraphicsItem] = None) -> None:
        self._scene = scene
        self._group = QGraphicsItemGroup()
        self._group.setZValue(Z_DETECTIONS)
        _attach(scene, self._group, parent)
        self._items: Dict[int, QGraphicsItemGroup] = {}

    def upsert(self, det: Detection) -> None:
        """Add a detection, replacing any previous one with the same uid
        (revisits refine positions, so uid-based replacement is required)."""
        old = self._items.pop(det.uid, None)
        if old is not None:
            self._scene.removeItem(old)

        r = max(det.extent_m, 0.5)
        circle = QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
        pen = QPen(theme.COLOR_DETECTION, 0)
        pen.setCosmetic(True)
        pen.setWidthF(2.0)
        circle.setPen(pen)
        circle.setBrush(QBrush(QColor(255, 202, 40, 40)))

        label = QGraphicsSimpleTextItem(
            f"{det.class_name} {det.confidence:.0%}")
        label.setBrush(QBrush(theme.COLOR_DETECTION))
        label.setFont(QFont("DejaVu Sans", 8))
        # Keep the label readable at any zoom level.
        label.setFlag(QGraphicsSimpleTextItem.ItemIgnoresTransformations)
        label.setPos(r * 0.8, -r * 0.8)

        g = QGraphicsItemGroup()
        g.addToGroup(circle)
        g.addToGroup(label)
        g.setPos(w2s(det.x, det.y))
        g.setParentItem(self._group)
        self._items[det.uid] = g

    def clear(self) -> None:
        for item in self._items.values():
            self._scene.removeItem(item)
        self._items.clear()

    def set_visible(self, visible: bool) -> None:
        self._group.setVisible(visible)


# ---------------------------------------------------------------------------
class PingerLayer:
    """Last known USBL pinger position (single highlighted marker)."""

    def __init__(self, scene: QGraphicsScene,
                 parent: Optional[QGraphicsItem] = None) -> None:
        self._group = QGraphicsItemGroup()
        self._group.setZValue(Z_PINGER)
        _attach(scene, self._group, parent)

        pen = QPen(theme.COLOR_PINGER, 0)
        pen.setCosmetic(True)
        pen.setWidthF(2.0)

        self._accuracy = QGraphicsEllipseItem()
        acc_pen = QPen(theme.COLOR_PINGER, 0)
        acc_pen.setCosmetic(True)
        acc_pen.setStyle(Qt.DashLine)
        self._accuracy.setPen(acc_pen)
        self._accuracy.setBrush(QBrush(QColor(105, 240, 174, 25)))
        self._group.addToGroup(self._accuracy)

        self._cross_a = QGraphicsLineItem()
        self._cross_b = QGraphicsLineItem()
        for line in (self._cross_a, self._cross_b):
            line.setPen(pen)
            self._group.addToGroup(line)

        self._group.setVisible(False)  # nothing to show until a fix arrives
        self._has_fix = False
        self._enabled = True

    def update(self, fix: PingerFix) -> None:
        s = 1.2  # cross half-size, metres
        p = w2s(fix.x, fix.y)
        self._cross_a.setLine(p.x() - s, p.y() - s, p.x() + s, p.y() + s)
        self._cross_b.setLine(p.x() - s, p.y() + s, p.x() + s, p.y() - s)
        r = fix.accuracy_m or 0.0
        self._accuracy.setRect(p.x() - r, p.y() - r, 2 * r, 2 * r)
        self._accuracy.setVisible(r > 0.0)
        self._has_fix = True
        self._group.setVisible(self._enabled)

    def clear(self) -> None:
        """Hide the marker until the next fix arrives."""
        self._has_fix = False
        self._group.setVisible(False)

    def set_visible(self, visible: bool) -> None:
        self._enabled = visible
        self._group.setVisible(visible and self._has_fix)


# ---------------------------------------------------------------------------
class SelectionLayer:
    """The point picked in the waterfall view, shown on the world map.

    One diamond + crosshair marker with a lat/lon label, parented to
    WorldRoot so it takes plain world metres (same convention as
    PingerLayer) and follows anchor refits for free.
    """

    _COLOR = QColor(80, 220, 255)      # matches the waterfall crosshair

    def __init__(self, scene: QGraphicsScene,
                 parent: Optional[QGraphicsItem] = None) -> None:
        self._group = QGraphicsItemGroup()
        self._group.setZValue(Z_SELECTION)
        _attach(scene, self._group, parent)

        pen = QPen(self._COLOR, 0)
        pen.setCosmetic(True)
        pen.setWidthF(2.0)
        s = 1.0                        # diamond half-size, metres
        self._diamond = QGraphicsPolygonItem(QPolygonF(
            [QPointF(0, -s), QPointF(s, 0), QPointF(0, s), QPointF(-s, 0)]))
        self._diamond.setPen(pen)
        self._diamond.setBrush(QBrush(QColor(80, 220, 255, 40)))
        self._cross_h = QGraphicsLineItem(-1.8 * s, 0, 1.8 * s, 0)
        self._cross_v = QGraphicsLineItem(0, -1.8 * s, 0, 1.8 * s)
        for line in (self._cross_h, self._cross_v):
            line.setPen(pen)
        self._label = QGraphicsSimpleTextItem()
        self._label.setBrush(QBrush(self._COLOR))
        self._label.setFont(QFont("DejaVu Sans", 8))
        self._label.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self._label.setPos(1.4 * s, -2.2 * s)
        for item in (self._diamond, self._cross_h, self._cross_v,
                     self._label):
            self._group.addToGroup(item)
        self._group.setVisible(False)

    def show_at(self, x: float, y: float, label: str = "") -> None:
        """Place the marker at world (x, y) with an optional text label."""
        self._group.setPos(w2s(x, y))
        self._label.setText(label)
        self._group.setVisible(True)

    def clear(self) -> None:
        self._group.setVisible(False)


# ---------------------------------------------------------------------------
class MeasureLayer:
    """Two-click distance measurement overlay."""

    def __init__(self, scene: QGraphicsScene) -> None:
        pen = QPen(theme.COLOR_MEASURE, 0)
        pen.setCosmetic(True)
        pen.setWidthF(2.0)
        self._line = QGraphicsLineItem()
        self._line.setPen(pen)
        self._line.setZValue(Z_MEASURE)
        scene.addItem(self._line)

        self._label = QGraphicsSimpleTextItem()
        self._label.setBrush(QBrush(theme.COLOR_MEASURE))
        self._label.setFont(QFont("DejaVu Sans", 9, QFont.Bold))
        self._label.setFlag(QGraphicsSimpleTextItem.ItemIgnoresTransformations)
        self._label.setZValue(Z_MEASURE)
        scene.addItem(self._label)

        self._marks: List[QGraphicsEllipseItem] = []
        for _ in range(2):
            m = QGraphicsEllipseItem(-0.3, -0.3, 0.6, 0.6)
            m.setPen(pen)
            m.setBrush(QBrush(theme.COLOR_MEASURE))
            m.setZValue(Z_MEASURE)
            scene.addItem(m)
            self._marks.append(m)
        self.clear()

    def show_first(self, x: float, y: float) -> None:
        self.clear()
        self._marks[0].setPos(w2s(x, y))
        self._marks[0].setVisible(True)

    def show_measurement(self, p1: Tuple[float, float],
                         p2: Tuple[float, float], distance_m: float) -> None:
        s1, s2 = w2s(*p1), w2s(*p2)
        self._line.setLine(s1.x(), s1.y(), s2.x(), s2.y())
        self._line.setVisible(True)
        self._marks[0].setPos(s1)
        self._marks[1].setPos(s2)
        for m in self._marks:
            m.setVisible(True)
        mid = QPointF((s1.x() + s2.x()) / 2, (s1.y() + s2.y()) / 2)
        self._label.setText(f"{distance_m:.2f} m")
        self._label.setPos(mid)
        self._label.setVisible(True)

    def clear(self) -> None:
        for item in (self._line, self._label, *self._marks):
            item.setVisible(False)


# ---------------------------------------------------------------------------
class SwathLayer:
    """Current sonar acquisition range: a thin white line through the robot,
    perpendicular to its heading, spanning ±range.

    The extent is derived from the actual samples of every ping
    (max |y_local|), so it always reflects the *current* sonar
    configuration and updates automatically the moment the range changes
    — no configuration duplication in the GUI.
    """

    def __init__(self, scene: QGraphicsScene,
                 parent: Optional[QGraphicsItem] = None) -> None:
        pen = QPen(QColor(255, 255, 255, 210), 0)
        pen.setCosmetic(True)          # thin (1 px) at any zoom level
        pen.setWidthF(1.0)
        self._line = QGraphicsLineItem()
        self._line.setPen(pen)
        self._line.setZValue(Z_SWATH)
        self._line.setVisible(False)   # nothing to show until a ping arrives
        _attach(scene, self._line, parent)
        self._enabled = True
        self._has_data = False

    def update(self, robot_x: float, robot_y: float, yaw: float,
               range_m: float) -> None:
        if range_m <= 0.0:
            return
        # The ping is purely lateral: same geometry as project_to_world
        # evaluated at y_local = ±range.
        dx, dy = -math.sin(yaw) * range_m, math.cos(yaw) * range_m
        p1 = w2s(robot_x + dx, robot_y + dy)   # port tip
        p2 = w2s(robot_x - dx, robot_y - dy)   # starboard tip
        self._line.setLine(p1.x(), p1.y(), p2.x(), p2.y())
        self._has_data = True
        self._line.setVisible(self._enabled)

    def clear(self) -> None:
        self._has_data = False
        self._line.setVisible(False)

    def set_visible(self, visible: bool) -> None:
        self._enabled = visible
        self._line.setVisible(visible and self._has_data)


# ---------------------------------------------------------------------------
class PlannedPathLayer:
    """Planned mission path (nav_msgs/Path from path_publisher.py).

    Thin dark-blue line, deliberately distinct from the cyan executed
    trajectory and drawn *below* it. A new message fully replaces the
    previous path.
    """

    def __init__(self, scene: QGraphicsScene,
                 parent: Optional[QGraphicsItem] = None) -> None:
        pen = QPen(theme.COLOR_PLANNED_PATH, 0)
        pen.setCosmetic(True)
        pen.setWidthF(1.2)
        self._item = QGraphicsPathItem()
        self._item.setPen(pen)
        self._item.setZValue(Z_PLANNED_PATH)
        _attach(scene, self._item, parent)
        self._last_points = None

    def set_path(self, points) -> None:
        """Replace the displayed path with ((x, y), ...) in world metres."""
        if points == self._last_points:
            return                # verbatim re-send: no rebuild, no repaint
        self._last_points = points
        path = QPainterPath()
        for i, (x, y) in enumerate(points):
            (path.moveTo if i == 0 else path.lineTo)(w2s(x, y))
        self._item.setPath(path)

    def clear(self) -> None:
        self._last_points = None
        self._item.setPath(QPainterPath())

    def set_visible(self, visible: bool) -> None:
        self._item.setVisible(visible)


# ---------------------------------------------------------------------------
class TileLayer:
    """Satellite / street background, pinned to the geographic origin.

    Placed directly in the EN scene from ``(lat0, lon0)`` alone — never
    through the odom translation ``t`` — so imagery never slides when
    the anchor refits (the MCS rule). Only active once the anchor
    exists. Tiles for the current zoom level replace tiles of the
    previous one as they arrive, which keeps zoom transitions smooth.
    """

    MAX_TILES_PER_UPDATE = 96

    def __init__(self, scene: QGraphicsScene, fetcher: TileFetcher,
                 origin_provider: Callable[[], Optional[Tuple[float, float]]]
                 ) -> None:
        self._scene = scene
        self._fetcher = fetcher
        self._origin = origin_provider     # () -> (lat0, lon0) | None
        self._items: Dict[TileKey, QGraphicsPixmapItem] = {}
        self._zoom: Optional[int] = None
        self._visible = True
        fetcher.tile_ready.connect(self._on_tile_ready)

    # -- viewport driven update ------------------------------------------------
    def update_viewport(self, en_rect: QRectF, metres_per_px: float) -> None:
        """Ensure tiles covering ``en_rect`` (EN metres, y-up) exist."""
        origin = self._origin()
        if not (self._visible and origin is not None):
            return
        lat0, lon0 = origin
        z = zoom_for_resolution(lat0, metres_per_px)

        if z != self._zoom:
            self._drop_other_zooms(z)
            self._zoom = z

        # EN rect corners -> lat/lon -> tile index range.
        corners = [(en_rect.left(), en_rect.top()),
                   (en_rect.right(), en_rect.bottom())]
        txs, tys = [], []
        for east, north in corners:
            lat, lon = local_en_to_latlon(east, north, lat0, lon0)
            tx, ty = latlon_to_tile(lat, lon, z)
            txs.append(tx)
            tys.append(ty)
        x0, x1 = int(math.floor(min(txs))), int(math.floor(max(txs)))
        y0, y1 = int(math.floor(min(tys))), int(math.floor(max(tys)))
        n = 2 ** z

        count = 0
        for tx in range(max(0, x0), min(n - 1, x1) + 1):
            for ty in range(max(0, y0), min(n - 1, y1) + 1):
                if count >= self.MAX_TILES_PER_UPDATE:
                    return
                count += 1
                key = (z, tx, ty)
                if key in self._items:
                    continue
                img = self._fetcher.request(key)
                if img is not None:
                    self._place_tile(key, img)

    def _on_tile_ready(self, z: int, x: int, y: int, img: QImage) -> None:
        if z == self._zoom and (z, x, y) not in self._items:
            self._place_tile((z, x, y), img)

    def _place_tile(self, key: TileKey, img: QImage) -> None:
        origin = self._origin()
        if origin is None:
            return
        lat0, lon0 = origin
        z, tx, ty = key
        nw = latlon_to_local_en(*tile_to_latlon(tx, ty, z), lat0, lon0)
        se = latlon_to_local_en(*tile_to_latlon(tx + 1, ty + 1, z),
                                lat0, lon0)
        item = QGraphicsPixmapItem(QPixmap.fromImage(img))
        item.setZValue(Z_TILES)
        item.setTransformationMode(Qt.SmoothTransformation)
        item.setPos(w2s(nw[0], nw[1]))  # NW corner; north decreases southward
        sx = (se[0] - nw[0]) / TILE_SIZE_PX
        sy = (nw[1] - se[1]) / TILE_SIZE_PX  # scene y grows downward
        item.setTransform(QTransform.fromScale(sx, sy))
        item.setVisible(self._visible)
        self._scene.addItem(item)
        self._items[key] = item

    def _drop_other_zooms(self, keep_zoom: int) -> None:
        for key in [k for k in self._items if k[0] != keep_zoom]:
            self._scene.removeItem(self._items.pop(key))

    def set_visible(self, visible: bool) -> None:
        self._visible = visible
        for item in self._items.values():
            item.setVisible(visible)
