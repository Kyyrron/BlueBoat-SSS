"""Waterfall service: raw pings stacked in acquisition order.

Rendering strategy (decision)
-----------------------------
The waterfall is *not* a map layer: its axes are ping index (vertical,
time) × across-track distance (horizontal), i.e. the raw acquisition
domain — the domain in which future AI datasets will be generated. It
therefore gets its own numpy buffer and its own view widget instead of
being forced into the georeferenced QGraphicsScene.

Growable tiled buffer (replaces the old 1500-row ring)
------------------------------------------------------
The original ring buffer silently discarded the head of any mission
longer than ``waterfall_rows`` pings, and every render rebuilt and
re-colormapped the *whole* image (``np.roll`` + LUT) at 4 Hz. Both are
gone:

* Rows live in fixed-height **tiles** of ``_TILE_ROWS`` × columns
  (float32 data + float64 per-row metadata), appended as the mission
  grows. A row's index is its chronological ping index forever, which
  is what makes row → pose lookups (:meth:`row_meta`) trivial.
* Each render pass re-colormaps **dirty tiles only** and emits them
  individually (:attr:`tile_updated`); the view keeps one pixmap item
  per tile, positioned at its absolute row. Scene y == ping index.
* Memory is bounded by ``mosaic.waterfall_max_rows`` (float32 data +
  meta ≈ 3.6 KB/row at 800 columns). Past the cap the **oldest whole
  tile** is dropped and ``_row0`` advances, so later indices stay
  valid. Replay calls :meth:`reserve` with the mission's ping count so
  an entire file is scrollable.
* Contrast in auto mode is **global**: an incremental histogram over
  every ingested sample yields one (p_lo, p_hi) window for all tiles —
  per-tile percentiles would band at tile seams. When the window moves
  more than 3 % of its span, every tile is re-rendered once.

The pixel pipeline itself is still the shared
``MosaicRenderer``/``DisplaySettings`` path, so the display controls
behave identically in both views. Raw data is never modified.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QImage

from ..config.settings import AppConfig
from ..mapping.renderer import DisplaySettings, MosaicRenderer
from ..models.sonar import SonarPing

#: Rows per tile. 512 × 800 float32 ≈ 1.6 MB data + 20 KB meta; small
#: enough that re-colormapping the dirty tail tile at 4 Hz is trivial.
_TILE_ROWS = 512

#: Fixed histogram span for the global auto-contrast window [dB]. Sonar
#: intensities (scale_to_db output, simulator, field corpus) all live
#: well inside it; out-of-range samples clamp to the edge bins.
_HIST_LO, _HIST_HI, _HIST_BINS = -140.0, 40.0, 720

#: Re-render every tile when the auto window moved more than this
#: fraction of its own span since the last full pass.
_LIMITS_REFRESH_FRACTION = 0.03

#: PNG quick-look decimation threshold (export_into).
_PNG_MAX_ROWS = 20_000


class WaterfallService(QObject):
    """Growable tiled buffer of all pings, rendered on a throttle."""

    #: Buffer geometry: (row0, total_rows, columns, current range [m]).
    #: Rows [row0, total_rows) exist; row0 > 0 means the oldest rows
    #: were evicted by the memory cap. total_rows == row0 == 0 after
    #: clear() — the view drops everything.
    layout_changed = Signal(int, int, int, float)
    #: One re-rendered tile: (absolute first row, QImage of the tile).
    tile_updated = Signal(int, QImage)
    #: Detection overlay in ABSOLUTE buffer coordinates:
    #: list of {"row": int, "col": int, "label": str}.
    detections_updated = Signal(object)

    def __init__(self, config: AppConfig) -> None:
        super().__init__()
        self._cols = int(config.mosaic.waterfall_columns)
        self._capacity = int(config.mosaic.waterfall_max_rows)
        # Tiles: parallel lists of (data, meta) arrays. meta columns are
        # (t, x, y, yaw, half-range) — needed for detections overlay and
        # the waterfall-click → world lookup.
        self._tiles: List[np.ndarray] = []
        self._metas: List[np.ndarray] = []
        self._row0 = 0                  # absolute index of the oldest row
        self._total = 0                 # absolute index one past the newest
        self._dirty_tiles: set = set()  # tile list indices needing render
        self._detections: list = []     # dicts: t_s, x, y, label
        self._range_m = 0.0             # current per-side swath [m]
        self._dirty = False
        self._layout_dirty = False
        self._enabled = False           # render only while the view is shown
        self._renderer = MosaicRenderer(
            percentiles=tuple(config.mosaic.contrast_percentiles))
        self._pcts = tuple(config.mosaic.contrast_percentiles)
        # Global auto-contrast histogram over every ingested sample.
        self._hist = np.zeros(_HIST_BINS, dtype=np.int64)
        self._hist_edges = np.linspace(_HIST_LO, _HIST_HI, _HIST_BINS + 1)
        self._last_limits: Optional[Tuple[float, float]] = None

        self._timer = QTimer(self)
        self._timer.setInterval(int(1000.0 / config.mosaic.render_hz))
        self._timer.timeout.connect(self._render_if_dirty)
        self._timer.start()

    # ---- geometry helpers ------------------------------------------------------
    @property
    def total_rows(self) -> int:
        return self._total

    @property
    def first_row(self) -> int:
        return self._row0

    def reserve(self, rows: int) -> int:
        """Raise the row capacity (never lowers it); returns the cap.

        The replay window calls this with the mission's ping count so a
        whole file stays scrollable instead of losing its head to the
        live-mode memory cap.
        """
        self._capacity = max(self._capacity, int(rows))
        return self._capacity

    def _tail_slot(self) -> Tuple[np.ndarray, np.ndarray, int]:
        """(data tile, meta tile, offset) for the next row to write."""
        idx = self._total - self._row0
        tile_i, off = divmod(idx, _TILE_ROWS)
        if tile_i == len(self._tiles):
            self._tiles.append(np.full((_TILE_ROWS, self._cols), np.nan,
                                       dtype=np.float32))
            self._metas.append(np.full((_TILE_ROWS, 5), np.nan,
                                       dtype=np.float64))
        return self._tiles[tile_i], self._metas[tile_i], off

    def _advance(self) -> None:
        self._total += 1
        self._dirty_tiles.add((self._total - 1 - self._row0) // _TILE_ROWS)
        self._dirty = True
        self._layout_dirty = True
        # Memory cap: drop the oldest whole tile. Absolute indices keep
        # their meaning — row_meta() for evicted rows returns None.
        if self._total - self._row0 > self._capacity and len(self._tiles) > 1:
            self._tiles.pop(0)
            self._metas.pop(0)
            self._row0 += _TILE_ROWS
            self._dirty_tiles = {i - 1 for i in self._dirty_tiles if i > 0}

    # ---- ingestion -------------------------------------------------------------
    def on_sonar_ping(self, ping: SonarPing) -> None:
        y = ping.y_local
        if y.size < 2:
            return
        # Column scale: prefer the sonar's CONFIGURED slant range, which
        # is constant for a given setting. max|y_local| depends on the
        # altitude estimate (ground = sqrt(slant^2 - h^2)), so when the
        # bottom detection wobbles — 4.7 m to 45 m on real sea-trial
        # data — every row gets a different scale and the waterfall
        # ripples. Falling back to max|y_local| keeps older logs and the
        # simulator working.
        r = float(ping.slant_range_m) if ping.slant_range_m > 0.0 else 0.0
        if r <= 0.0:
            r = float(np.abs(y).max())
        if r <= 0.0:
            return
        self._range_m = r
        # Across-track column: port (+y) on the LEFT, starboard on the right.
        col = np.clip(((r - y) / (2.0 * r) * (self._cols - 1)).astype(np.int32),
                      0, self._cols - 1)
        buf, meta, off = self._tail_slot()
        row = buf[off]
        row.fill(np.nan)
        row[col] = ping.intensity_db          # duplicates: last sample wins
        meta[off] = (ping.t, ping.robot_x, ping.robot_y, ping.yaw, r)
        finite = ping.intensity_db[np.isfinite(ping.intensity_db)]
        if finite.size:
            self._hist += np.histogram(finite, bins=self._hist_edges)[0]
        self._advance()

    def break_row(self) -> None:
        """Insert one blank row, so the rows either side are not neighbours.

        The waterfall's vertical axis is ping index, not time, so nothing in it
        can express a pause on its own: pings minutes apart would stack as
        adjacent rows and read as continuous seabed. A ``.svlog`` holding two
        recording sessions is exactly that case — Cerulean's harbour demo has a
        397.8 s gap — and the replay window calls this on the ``MissionGap``
        event. An all-NaN row renders as background, i.e. a visible seam.
        """
        buf, meta, off = self._tail_slot()
        buf[off].fill(np.nan)
        meta[off].fill(np.nan)
        self._advance()

    # ---- row → pose ------------------------------------------------------------
    def row_meta(self, row: int) -> Optional[Tuple[float, float, float,
                                                   float, float]]:
        """(t, robot_x, robot_y, yaw, half_range) of an absolute row, or
        None for a gap row, an evicted row, or an out-of-range index."""
        if not (self._row0 <= row < self._total):
            return None
        idx = row - self._row0
        m = self._metas[idx // _TILE_ROWS][idx % _TILE_ROWS]
        if not np.all(np.isfinite(m)):
            return None
        return tuple(float(v) for v in m)

    # ---- display -----------------------------------------------------------------
    def set_enabled(self, enabled: bool) -> None:
        """Called when the view mode switches; renders eagerly on entry."""
        self._enabled = enabled
        if enabled:
            self._mark_all_dirty()
            self._force_render()

    def clear(self) -> None:
        """'Clear SSS data': drop the buffered pings; keep streaming."""
        self._tiles.clear()
        self._metas.clear()
        self._detections.clear()
        self._row0 = 0
        self._total = 0
        self._dirty_tiles.clear()
        self._hist.fill(0)
        self._last_limits = None
        self._dirty = False
        self._layout_dirty = False
        self.layout_changed.emit(0, 0, self._cols, self._range_m)
        self.detections_updated.emit([])

    # ---- detections -------------------------------------------------------------
    def add_detection(self, t_s: float, x: float, y: float,
                      label: str) -> None:
        """Register a world-frame detection for waterfall display.

        ``t_s`` should be the ping time of the row the object was seen on
        (SeabedImager stamps each detection with its pixel row's time).
        """
        self._detections.append({"t_s": float(t_s), "x": float(x),
                                 "y": float(y), "label": label})
        if len(self._detections) > 500:
            self._detections.pop(0)
        self._dirty = True
        if self._enabled:
            self._force_render()

    def clear_detections(self) -> None:
        self._detections.clear()
        self.detections_updated.emit([])
        if self._enabled:
            self._force_render()

    def _overlay(self) -> list:
        """Map world detections to ABSOLUTE (row, col) buffer coordinates.

        Row: the ping whose time matches the detection's row time.
        Column: exact inversion of the pixel->world formula —
            y_local = (yw - y_r)·cos(yaw) − (xw − x_r)·sin(yaw)
            col     = (r − y_local) / (2 r) · (W − 1)
        Detections outside the buffered time span or the row's swath are
        skipped (they were evicted or lie off-swath at that instant).
        """
        out = []
        if self._total == self._row0 or not self._detections:
            return out
        meta = self._meta_chrono()
        times = meta[:, 0]
        finite = np.isfinite(times)
        if not finite.any():
            return out
        lo, hi = times[finite][0], times[finite][-1]
        for det in self._detections:
            if not (lo <= det["t_s"] <= hi):
                continue
            # Nearest row by time: gap rows are NaN, so a binary search
            # is off the table — nanargmin is O(n) but only runs while
            # detections exist, per render pass.
            i = int(np.nanargmin(np.abs(times - det["t_s"])))
            _t, rx, ry, yaw, r = meta[i]
            if not np.isfinite(r) or r <= 0:
                continue
            y_local = ((det["y"] - ry) * np.cos(yaw)
                       - (det["x"] - rx) * np.sin(yaw))
            if abs(y_local) > r * 1.02:
                continue
            col = int(round((r - y_local) / (2 * r) * (self._cols - 1)))
            out.append({"row": self._row0 + i,
                        "col": min(max(col, 0), self._cols - 1),
                        "label": det["label"]})
        return out

    # ---- persistence -----------------------------------------------------------
    def chronological(self) -> Optional[np.ndarray]:
        """All buffered rows, oldest first (None if empty)."""
        if self._total == self._row0:
            return None
        n = self._total - self._row0
        return np.concatenate(self._tiles, axis=0)[:n].copy()

    def _meta_chrono(self) -> np.ndarray:
        n = self._total - self._row0
        return np.concatenate(self._metas, axis=0)[:n]

    def export_into(self, target) -> bool:
        """Write waterfall.png (display pipeline) + waterfall_raw.npz
        (untouched buffer, for AI dataset generation) into ``target``.

        The npz always carries the full-fidelity float32 buffer. The PNG
        is a quick-look: above ``_PNG_MAX_ROWS`` rows it is vertically
        decimated (every Nth ping) and the stride is recorded in the npz
        as ``png_row_stride`` so nobody mistakes it for the data.
        """
        chrono = self.chronological()
        if chrono is None:
            return False
        from pathlib import Path
        import cv2
        target = Path(target)
        target.mkdir(parents=True, exist_ok=True)
        stride = max(1, int(np.ceil(chrono.shape[0] / _PNG_MAX_ROWS)))
        rgba = self._renderer.to_rgba(chrono[::stride])
        cv2.imwrite(str(target / "waterfall.png"),
                    cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
        np.savez_compressed(target / "waterfall_raw.npz",
                            intensity_db=chrono,
                            swath_half_range_m=self._range_m,
                            columns=self._cols,
                            png_row_stride=stride)
        return True

    def set_display(self, settings: DisplaySettings) -> None:
        self._renderer.settings = settings
        self._mark_all_dirty()
        if self._enabled:
            self._force_render()

    # ---- rendering -------------------------------------------------------------
    def _mark_all_dirty(self) -> None:
        self._dirty_tiles.update(range(len(self._tiles)))
        self._dirty = self._layout_dirty = bool(self._tiles)

    def _global_limits(self) -> Optional[Tuple[float, float]]:
        """One contrast window for every tile, from the global histogram
        (auto mode only — the manual window comes from DisplaySettings)."""
        if not self._renderer.settings.auto_range:
            return None
        total = int(self._hist.sum())
        if total == 0:
            return None
        cum = np.cumsum(self._hist) / total * 100.0
        lo_i = int(np.searchsorted(cum, self._pcts[0]))
        hi_i = int(np.searchsorted(cum, self._pcts[1]))
        centers = (self._hist_edges[:-1] + self._hist_edges[1:]) / 2.0
        lo = float(centers[min(lo_i, _HIST_BINS - 1)])
        hi = float(centers[min(hi_i, _HIST_BINS - 1)])
        if hi - lo < 1e-6:
            hi = lo + 1e-6
        return lo, hi

    def _render_if_dirty(self) -> None:
        if self._enabled and self._dirty:
            self._force_render()

    def _force_render(self) -> None:
        self._dirty = False
        if self._total == self._row0:
            return
        limits = self._global_limits()
        if limits is not None:
            last = self._last_limits
            span = limits[1] - limits[0]
            if (last is None
                    or abs(limits[0] - last[0]) > _LIMITS_REFRESH_FRACTION * span
                    or abs(limits[1] - last[1]) > _LIMITS_REFRESH_FRACTION * span):
                # The global window moved: every tile is stale at once.
                self._last_limits = limits
                self._dirty_tiles.update(range(len(self._tiles)))
            else:
                limits = last          # hold the window steady
        if self._layout_dirty:
            self._layout_dirty = False
            self.layout_changed.emit(self._row0, self._total, self._cols,
                                     self._range_m)
        for tile_i in sorted(self._dirty_tiles):
            if tile_i >= len(self._tiles):
                continue
            img = self._renderer.to_qimage(self._tiles[tile_i], flip=False,
                                           limits=limits)
            self.tile_updated.emit(self._row0 + tile_i * _TILE_ROWS, img)
        self._dirty_tiles.clear()
        self.detections_updated.emit(self._overlay())
