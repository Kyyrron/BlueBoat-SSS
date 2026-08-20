"""Background map tiles (OSM / Esri satellite) without extra dependencies.

* Standard slippy-map (Web-Mercator) tile scheme, fetched with Qt's own
  ``QNetworkAccessManager`` — asynchronous, so the GUI never blocks on
  the network — and cached on disk, so the app keeps working offline at
  sea for any area browsed once at the dock.
* Tiles are *positioned in the local metric frame* by converting their
  corner lat/lon through the ``CoordinateConverter``: at harbour scale
  the equirectangular local frame and Web Mercator agree to well under a
  pixel, so tiles can simply be scaled to their local-frame footprint.
* The tile layer is strictly a background: it only exists once the GPS
  origin is bound, and it renders below the SSS mosaic (z-order).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Optional, Set, Tuple

from PySide6.QtCore import QObject, QUrl, Signal
from PySide6.QtGui import QImage
from PySide6.QtNetwork import (QNetworkAccessManager, QNetworkReply,
                               QNetworkRequest)

TILE_SIZE_PX = 256
MIN_ZOOM, MAX_ZOOM = 3, 19

TileKey = Tuple[int, int, int]  # (z, x, y)


# ---- slippy math -------------------------------------------------------------
def latlon_to_tile(lat: float, lon: float, z: int) -> Tuple[float, float]:
    """WGS-84 -> fractional tile coordinates at zoom z."""
    lat_r = math.radians(max(-85.05112878, min(85.05112878, lat)))
    n = 2.0 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n
    return x, y


def tile_to_latlon(x: float, y: float, z: int) -> Tuple[float, float]:
    """Fractional tile coordinates -> WGS-84 of the tile's NW corner."""
    n = 2.0 ** z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return lat, lon


def ground_resolution(lat: float, z: int) -> float:
    """Metres per tile pixel at latitude ``lat`` and zoom ``z``."""
    return 156543.03392 * math.cos(math.radians(lat)) / (2.0 ** z)


def zoom_for_resolution(lat: float, metres_per_px: float) -> int:
    """Smallest zoom whose tiles are at least as sharp as the view."""
    for z in range(MAX_ZOOM, MIN_ZOOM - 1, -1):
        if ground_resolution(lat, z) >= metres_per_px * 0.75:
            return z
    return MAX_ZOOM


# ---- fetcher ------------------------------------------------------------------
class TileFetcher(QObject):
    """Async tile downloads with a persistent disk cache.

    Emits ``tile_ready(z, x, y, QImage)``. Failed downloads are dropped
    silently (the layer simply shows nothing there); a tile is re-requested
    on the next viewport change.
    """

    tile_ready = Signal(int, int, int, QImage)

    def __init__(self, url_template: str, cache_dir: Path,
                 max_concurrent: int = 6, parent: Optional[QObject] = None
                 ) -> None:
        super().__init__(parent)
        self._template = url_template
        self._cache_dir = cache_dir
        self._cache_dir.mkdir(parents=True, exist_ok=True)
        self._nam = QNetworkAccessManager(self)
        self._pending: Set[TileKey] = set()
        self._queue: list[TileKey] = []
        self._max_concurrent = max_concurrent
        self._inflight = 0

    # -- public ---------------------------------------------------------------
    def request(self, key: TileKey) -> Optional[QImage]:
        """Return the tile immediately if cached, else schedule a download."""
        img = self._load_cached(key)
        if img is not None:
            return img
        if key not in self._pending:
            self._pending.add(key)
            self._queue.append(key)
            self._pump()
        return None

    # -- internals ----------------------------------------------------------
    def _cache_path(self, key: TileKey) -> Path:
        z, x, y = key
        return self._cache_dir / str(z) / str(x) / f"{y}.png"

    def _load_cached(self, key: TileKey) -> Optional[QImage]:
        p = self._cache_path(key)
        if p.is_file():
            img = QImage(str(p))
            if not img.isNull():
                return img
        return None

    def _pump(self) -> None:
        while self._queue and self._inflight < self._max_concurrent:
            key = self._queue.pop(0)
            z, x, y = key
            url = self._template.format(z=z, x=x, y=y)
            req = QNetworkRequest(QUrl(url))
            # OSM usage policy requires an identifying user agent.
            req.setHeader(QNetworkRequest.UserAgentHeader,
                          "BlueBoatGCS/1.0 (research USV survey GUI)")
            reply = self._nam.get(req)
            self._inflight += 1
            reply.finished.connect(lambda r=reply, k=key: self._on_reply(r, k))

    def _on_reply(self, reply: QNetworkReply, key: TileKey) -> None:
        self._inflight -= 1
        self._pending.discard(key)
        try:
            if reply.error() == QNetworkReply.NetworkError.NoError:
                data = bytes(reply.readAll())
                img = QImage.fromData(data)
                if not img.isNull():
                    path = self._cache_path(key)
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_bytes(data)
                    self.tile_ready.emit(key[0], key[1], key[2], img)
        finally:
            reply.deleteLater()
            self._pump()
