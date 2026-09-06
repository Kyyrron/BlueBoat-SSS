"""Waterfall service: raw pings stacked in acquisition order.

Rendering strategy (decision)
-----------------------------
The waterfall is *not* a map layer: its axes are ping index (vertical,
time) × across-track slant range (horizontal) — the raw acquisition
domain, exactly as SonarView draws it. It therefore gets its own numpy
buffer and its own view widget instead of being forced into the
georeferenced QGraphicsScene.

Native slant-bin rows (replaces the fixed-width ground scatter)
---------------------------------------------------------------
The old implementation scattered each ping's slant-corrected ground
samples into a fixed 800-column grid. Ground-range spacing is not
uniform (it stretches toward nadir), so at short range up to ~25 % of
every row's columns received no sample and rendered as dark holes:
pepper noise, persistent vertical dark stripes, and a wide fake
"nadir gap" band — none of it in the data. All of that is gone:

* A row is the ping's **native device range bins, verbatim** — one
  column per bin, no resampling, no interpolation, hole-free by
  construction (`models.sonar.build_slant_row`). Column width adapts to
  the acquisition: nothing is forced to 800 columns any more.
* The centre gap that remains is the **physical water column** (bins
  with slant range below the tracked altitude, cut upstream): its width
  follows per ping from the geometry, not from column aliasing.
* When the range setting changes mid-mission the buffer re-lays itself
  out to the largest extent seen (finest bin pitch wins); rows at a
  coarser pitch paint by pure pixel stretch — values verbatim.
* Genuine acquisition loss (a jump in the device's own ping counter,
  `SonarPing.gap_before`) is drawn as blank lines, so real dropouts are
  visible and distinguishable from display artifacts.

Growable tiled buffer
---------------------
* Rows live in fixed-height **tiles** of ``_TILE_ROWS`` × columns
  (float32 data + float64 per-row metadata), appended as the mission
  grows. A row's index is its chronological ping index forever, which
  is what makes row → pose lookups (:meth:`row_meta`) trivial.
* Each render pass re-colormaps **dirty tiles only** and emits them
  individually (:attr:`tile_updated`); the view keeps one pixmap item
  per tile, positioned at its absolute row. Scene y == ping index.
* Memory is bounded by ``mosaic.waterfall_max_rows``. Past the cap the
  **oldest whole tile** is dropped and ``_row0`` advances, so later
  indices stay valid. Replay calls :meth:`reserve` with the mission's
  ping count so an entire file is scrollable.

Display normalization
---------------------
The device stream is pre-TVG (60–70 dB of range falloff across a ping on
the field logs), so a single linear dB->grey window over the raw values
cannot show near and far seabed at once. By default (``mosaic.
nadir_contrast``) every tile is rendered through the window's ONE
``DisplayModel`` (``core/display_model``): two-way transmission loss
removed, the empirical seabed curve in normalised slant range divided
out, then a power-law transfer with no low handle — the same model the
AI pictures and the mosaic use, so all three read alike and every export
inverts back to dB. The buffered dB is never modified; the model is
recorded on export. With ``nadir_contrast`` off the plain raw-value
histogram window applies and no normalisation is done.
"""

from __future__ import annotations

import math
from collections import deque
from typing import List, Optional, Tuple

import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QImage

from ..config.settings import AppConfig
from ..mapping.renderer import DisplaySettings, MosaicRenderer
from ..models.sonar import SonarPing, build_slant_row, native_bins
from .display_model import DisplayModel

#: Rows per tile. 512 × ~1200 float32 ≈ 2.5 MB data + 20 KB meta; small
#: enough that re-colormapping the dirty tail tile at 4 Hz is trivial.
_TILE_ROWS = 512

#: Fixed histogram span for the plain raw-value fallback window [dB]
#: (nadir_contrast off). Sonar intensities (scale_to_db output, simulator,
#: field corpus) all live well inside it; out-of-range samples clamp to
#: the edge bins.
_HIST_LO, _HIST_HI, _HIST_BINS = -140.0, 100.0, 960

#: Re-render every tile when the auto window moved more than this
#: fraction of its own span since the last full pass.
_LIMITS_REFRESH_FRACTION = 0.03

#: Cap on blank rows inserted per detected ping-counter gap: loss stays
#: visible without a large gap scrolling the image away.
_MAX_GAP_ROWS = 5

#: PNG quick-look decimation threshold (export_into).
_PNG_MAX_ROWS = 20_000


class WaterfallService(QObject):
    """Growable tiled buffer of all pings, rendered on a throttle."""

    #: Buffer geometry: (row0, total_rows, columns, half-extent [m],
    #: along-track metres per row). Rows [row0, total_rows) exist; row0 > 0
    #: means the oldest rows were evicted by the memory cap. total_rows ==
    #: row0 == 0 after clear() — the view drops everything. The last value
    #: (running median of consecutive row displacements) lets the view
    #: draw true scale; 0 until two placed rows exist.
    layout_changed = Signal(int, int, int, float, float)
    #: One re-rendered tile: (absolute first row, QImage of the tile).
    tile_updated = Signal(int, QImage)
    #: Detection overlay in ABSOLUTE buffer coordinates:
    #: list of {"row": int, "col": int, "label": str}.
    detections_updated = Signal(object)

    #: Stale (window-moved) tiles re-rendered per pass, newest first: the
    #: rest follow on later passes, so a window move never freezes the
    #: GUI for a full-buffer re-colormap (586 ms at 15 k rows, seconds at
    #: the cap).
    _STALE_PER_PASS = 2
    #: Past this many buffered rows the auto window is frozen unless it
    #: moves by _LIMITS_REFRESH_FRACTION_LATE: the EGN statistics have
    #: converged and every later move would only re-render the world.
    _LIMITS_FREEZE_ROWS = 2000
    _LIMITS_REFRESH_FRACTION_LATE = 0.15
    _DISPLAY_DEBOUNCE_MS = 120

    def __init__(self, config: AppConfig,
                 model: Optional[DisplayModel] = None) -> None:
        super().__init__()
        self._max_rows = int(config.mosaic.waterfall_max_rows)
        self._max_samples = int(getattr(config.mosaic, "waterfall_max_samples", 0))
        self._capacity = self._max_rows
        self._reserved = 0
        self._stale_tiles: set = set()  # window moved: re-render lazily
        self._throttled = False
        self._pending_settings = None
        # Layout: slant-bin pitch [m/column] and columns per side. Both
        # adapt to the data (0 until the first ping arrives).
        self._pitch = 0.0
        self._half = 0
        # Tiles: parallel lists of (data, meta) arrays. meta columns are
        # (t, x, y, yaw, water_depth, bottom_slant_m) — the first five feed
        # the detections overlay and the waterfall-click → world lookup;
        # bottom_slant_m lets a remap re-classify seabed vs water column
        # so the seabed-referenced EGN can be rebuilt exactly.
        self._tiles: List[np.ndarray] = []
        self._metas: List[np.ndarray] = []
        self._row0 = 0                  # absolute index of the oldest row
        self._total = 0                 # absolute index one past the newest
        self._dirty_tiles: set = set()  # tile list indices needing render
        self._detections: list = []     # dicts: t_s, x, y, label
        self._dirty = False
        self._layout_dirty = False
        self._enabled = False           # render only while the view is shown
        self._renderer = MosaicRenderer(
            percentiles=tuple(config.mosaic.contrast_percentiles))
        self._pcts = tuple(config.mosaic.contrast_percentiles)
        # Plain raw-value histogram: the fallback window when the
        # nadir-aware seabed EGN is disabled (nadir_contrast off).
        self._hist = np.zeros(_HIST_BINS, dtype=np.int64)
        self._hist_edges = np.linspace(_HIST_LO, _HIST_HI, _HIST_BINS + 1)
        # The window's ONE display model (core/display_model.py): injected
        # by the window so the waterfall, the AI imager and the mosaic map
        # dB to grey identically; owned here only when nothing is injected
        # (bench / tests). It lives in physical units (slant range, the
        # row's altitude), so a layout remap never touches it.
        self._model = model if model is not None else DisplayModel(config)
        self._owns_model = model is None
        self._nadir_contrast = bool(config.mosaic.nadir_contrast)
        self._rendered_version: Optional[Tuple[int, bool]] = None
        # Unit-domain renderer for the model path: the model applies the
        # transfer (gamma), the renderer only colours [0, 1] + brightness.
        self._unit_renderer = MosaicRenderer()
        self._unit_renderer.settings = DisplaySettings(gamma=1.0)
        self._last_limits: Optional[Tuple[float, float]] = None
        # Along-track metres per row (true-scale view): running median of
        # the displacement between consecutive placed rows.
        self._row_steps: deque = deque(maxlen=256)
        self._last_xy: Optional[Tuple[float, float]] = None

        self._timer = QTimer(self)
        self._timer.setInterval(int(1000.0 / config.mosaic.render_hz))
        self._timer.timeout.connect(self._render_if_dirty)
        self._timer.start()
        self._display_timer = QTimer(self)
        self._display_timer.setSingleShot(True)
        self._display_timer.setInterval(self._DISPLAY_DEBOUNCE_MS)
        self._display_timer.timeout.connect(self._apply_pending_display)

    def throttle(self, on: bool) -> None:
        """Skip renders while the GUI thread is behind the ping stream;
        rows still enter the buffer (never dropped)."""
        self._throttled = bool(on)

    def _recompute_capacity(self) -> None:
        """Row cap = min(row cap, sample cap / columns), never below a
        replay reservation."""
        cap = self._max_rows
        if self._max_samples > 0 and self.columns > 0:
            cap = min(cap, max(self._max_samples // self.columns, 2 * _TILE_ROWS))
        self._capacity = max(cap, self._reserved)

    # ---- geometry helpers ------------------------------------------------------
    @property
    def total_rows(self) -> int:
        return self._total

    @property
    def first_row(self) -> int:
        return self._row0

    @property
    def columns(self) -> int:
        """Current buffer width (0 before the first ping)."""
        return 2 * self._half

    @property
    def pitch_m(self) -> float:
        """Slant metres per column (0 before the first ping)."""
        return self._pitch

    @property
    def half_extent_m(self) -> float:
        return self._half * self._pitch

    @property
    def model(self) -> DisplayModel:
        """The display model this service renders through."""
        return self._model

    @property
    def row_pitch_m(self) -> float:
        """Along-track metres per row (0 until two placed rows exist)."""
        if not self._row_steps:
            return 0.0
        return float(np.median(np.fromiter(self._row_steps, dtype=np.float64)))

    def reserve(self, rows: int) -> int:
        """Raise the row capacity (never lowers it); returns the cap.

        The replay window calls this with the mission's ping count so a
        whole file stays scrollable instead of losing its head to the
        live-mode memory cap.
        """
        self._reserved = max(self._reserved, int(rows))
        self._capacity = max(self._capacity, self._reserved)
        return self._capacity

    def _tail_slot(self) -> Tuple[np.ndarray, np.ndarray, int]:
        """(data tile, meta tile, offset) for the next row to write."""
        idx = self._total - self._row0
        tile_i, off = divmod(idx, _TILE_ROWS)
        if tile_i == len(self._tiles):
            self._tiles.append(np.full((_TILE_ROWS, self.columns), np.nan,
                                       dtype=np.float32))
            self._metas.append(np.full((_TILE_ROWS, 6), np.nan,
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
            self._stale_tiles = {i - 1 for i in self._stale_tiles if i > 0}

    # ---- adaptive layout -------------------------------------------------------
    def _ensure_layout(self, bin_size_m: float, extent_bins: int) -> None:
        """Adopt / widen the (pitch, half-columns) layout for a new row.

        The buffer always spans the **largest range seen** at the
        **finest bin pitch seen** (the user-facing rule: a recording
        holding two range settings gets a picture sized to the bigger
        one). Both changes remap the existing tiles by pure pixel
        stretch — verbatim values, no interpolation.
        """
        if bin_size_m <= 0 or extent_bins <= 0:
            return
        extent_m = bin_size_m * extent_bins
        if self._pitch == 0.0:
            self._pitch = bin_size_m
            self._half = extent_bins
            # Rows buffered before the first ping (pre-layout break_row
            # seams) were zero-width; give them the adopted width.
            for i, tile in enumerate(self._tiles):
                if tile.shape[1] != self.columns:
                    self._tiles[i] = np.full((tile.shape[0], self.columns),
                                             np.nan, dtype=np.float32)
            self._reset_stats()
            self._recompute_capacity()
            return
        new_pitch = self._pitch
        if bin_size_m < self._pitch * (1.0 - 1e-6):
            new_pitch = bin_size_m
        new_half = max(self._half if new_pitch == self._pitch else 0,
                       int(math.ceil(max(extent_m, self.half_extent_m)
                                     / new_pitch - 1e-9)))
        if new_pitch == self._pitch and new_half == self._half:
            return
        self._remap(new_pitch, new_half)

    def _remap(self, new_pitch: float, new_half: int) -> None:
        """Re-lay existing tiles onto a new (pitch, half) grid."""
        old_pitch, old_half = self._pitch, self._half
        w_new = 2 * new_half
        # For each new column, the old column whose slant interval covers
        # it (or -1 = outside the old buffer → NaN).
        k_new = np.arange(new_half, dtype=np.float64)
        k_old = np.floor((k_new + 0.5) * new_pitch / old_pitch).astype(np.int64)
        valid = k_old < old_half
        port_src = np.where(valid, old_half - 1 - k_old, 0)
        stbd_src = np.where(valid, old_half + k_old, 0)
        for i, tile in enumerate(self._tiles):
            out = np.full((tile.shape[0], w_new), np.nan, dtype=np.float32)
            port = tile[:, port_src]
            port[:, ~valid] = np.nan
            stbd = tile[:, stbd_src]
            stbd[:, ~valid] = np.nan
            out[:, :new_half] = port[:, ::-1]
            out[:, new_half:] = stbd
            self._tiles[i] = out
        self._pitch, self._half = new_pitch, new_half
        self._reset_stats(rebuild=True)
        self._recompute_capacity()
        self._dirty_tiles.update(range(len(self._tiles)))
        self._dirty = self._layout_dirty = True

    def _reset_stats(self, rebuild: bool = False) -> None:
        """Reset (and optionally rebuild from the tiles) the raw-value
        histogram of the legacy window. The display model is NOT touched:
        it lives in physical units, so a remap leaves it valid."""
        self._hist.fill(0)
        self._last_limits = None
        if not rebuild:
            return
        for tile in self._tiles:
            finite = np.isfinite(tile)
            if finite.any():
                self._hist += np.histogram(tile[finite], bins=self._hist_edges)[0]

    # ---- ingestion -------------------------------------------------------------
    def on_sonar_ping(self, ping: SonarPing) -> None:
        nb = native_bins(ping)
        if nb is None or nb.extent_bins == 0:
            return
        # Genuine acquisition loss (device counter jumped): blank lines.
        for _ in range(min(int(ping.gap_before), _MAX_GAP_ROWS)):
            self.break_row()
        self._ensure_layout(nb.bin_size_m, nb.extent_bins)
        if self._half == 0:
            return
        row = build_slant_row(nb, self._pitch, self._half)
        buf, meta, off = self._tail_slot()
        buf[off] = row
        # The row's altitude for the display model: the tracked bottom,
        # else the processor's own correction altitude (its tracker).
        h = float(ping.bottom_slant_m)
        if h <= 0.0:
            h = max(float(ping.water_depth), 0.0)
        meta[off] = (ping.t, ping.robot_x, ping.robot_y, ping.yaw,
                     max(float(ping.water_depth), 0.0), h)
        if self._last_xy is not None:
            self._row_steps.append(math.hypot(ping.robot_x - self._last_xy[0],
                                              ping.robot_y - self._last_xy[1]))
        self._last_xy = (float(ping.robot_x), float(ping.robot_y))
        finite = np.isfinite(row)
        if finite.any():
            self._hist += np.histogram(row[finite], bins=self._hist_edges)[0]
            self._model.observe_row(row, self._pitch, h)
        self._advance()

    def break_row(self) -> None:
        """Insert one blank row, so the rows either side are not neighbours.

        The waterfall's vertical axis is ping index, not time, so nothing in it
        can express a pause on its own: pings minutes apart would stack as
        adjacent rows and read as continuous seabed. A ``.svlog`` holding two
        recording sessions is exactly that case — Cerulean's harbour demo has a
        397.8 s gap — and the replay window calls this on the ``MissionGap``
        event; live mode inserts one per ping the device counter says was
        lost. An all-NaN row renders as background, i.e. a visible seam.
        A pre-layout break (no ping seen yet) still counts a row: the
        zero-width tiles are re-sized when the first ping fixes the
        layout.
        """
        buf, meta, off = self._tail_slot()
        buf[off].fill(np.nan)
        meta[off].fill(np.nan)
        self._last_xy = None                 # no displacement across a seam
        self._advance()

    # ---- row → pose ------------------------------------------------------------
    def row_meta(self, row: int) -> Optional[Tuple[float, float, float,
                                                   float, float]]:
        """(t, robot_x, robot_y, yaw, water_depth) of an absolute row, or
        None for a gap row, an evicted row, or an out-of-range index."""
        if not (self._row0 <= row < self._total):
            return None
        idx = row - self._row0
        m = self._metas[idx // _TILE_ROWS][idx % _TILE_ROWS]
        if not np.all(np.isfinite(m[:5])):
            return None
        return tuple(float(v) for v in m[:5])

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
        self._pitch = 0.0
        self._half = 0
        self._dirty_tiles.clear()
        self._stale_tiles.clear()
        self._reset_stats()
        if self._owns_model:
            self._model.reset()
        self._row_steps.clear()
        self._last_xy = None
        self._dirty = False
        self._layout_dirty = False
        self.layout_changed.emit(0, 0, 0, 0.0, 0.0)
        self.detections_updated.emit([])

    # ---- detections -------------------------------------------------------------
    def add_detection(self, t_s: float, x: float, y: float,
                      label: str) -> None:
        """Register a world-frame detection for waterfall display.

        ``t_s`` should be the ping time of the row the object was seen on
        (SeabedImager stamps each detection with its pixel row's time).
        """
        det = {"t_s": float(t_s), "x": float(x), "y": float(y),
               "label": label, "row": self._row_for_time(float(t_s))}
        self._detections.append(det)
        if len(self._detections) > 500:
            self._detections.pop(0)
        # The timer's next pass draws it: no synchronous full render per
        # detection (one image = one detection = one 20-45 ms render).
        self._dirty = True

    def _row_for_time(self, t_s: float) -> Optional[int]:
        """Absolute row of the ping nearest ``t_s``, resolved ONCE at
        registration (the mapping never changes afterwards; scanning the
        whole buffer per detection per render pass was O(dets x rows))."""
        if self._total == self._row0:
            return None
        meta = self._meta_chrono()
        times = meta[:, 0]
        finite = np.isfinite(times)
        if not finite.any():
            return None
        lo, hi = times[finite][0], times[finite][-1]
        if not (lo <= t_s <= hi):
            return None
        return self._row0 + int(np.nanargmin(np.abs(times - t_s)))

    def clear_detections(self) -> None:
        self._detections.clear()
        self.detections_updated.emit([])
        if self._enabled:
            self._force_render()

    def _overlay(self) -> list:
        """Map world detections to ABSOLUTE (row, col) buffer coordinates.

        Row: the ping whose time matches the detection's row time.
        Column: invert the slant layout — ``y_local`` from the row pose,
        ``s = hypot(y_local, depth)``, ``k = s / pitch``. Detections
        outside the buffered time span or the row's swath are skipped
        (they were evicted or lie off-swath at that instant).
        """
        out = []
        if self._total == self._row0 or not self._detections or self._half == 0:
            return out
        for det in self._detections:
            row = det.get("row")
            if row is None:
                row = det["row"] = self._row_for_time(det["t_s"])
                if row is None:
                    continue
            if not (self._row0 <= row < self._total):
                continue                    # evicted by the memory cap
            i = row - self._row0
            tile, off = divmod(i, _TILE_ROWS)
            _t, rx, ry, yaw, depth = self._metas[tile][off, :5]
            if not np.isfinite(depth):
                continue
            y_local = ((det["y"] - ry) * np.cos(yaw)
                       - (det["x"] - rx) * np.sin(yaw))
            s = math.hypot(y_local, depth)
            k = int(s / self._pitch)
            if k >= self._half:
                continue
            col = (self._half - 1 - k) if y_local >= 0 else (self._half + k)
            out.append({"row": self._row0 + i,
                        "col": col,
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
        (the untouched native-bin buffer — an archival raw record; the
        AI feed is the seabed **pictures**) into ``target``.

        The npz carries the full-fidelity float32 buffer in the native
        slant-bin layout (``slant_pitch_m`` metres per column, transducer
        at the centre seam). The PNG is a quick-look through the display
        pipeline: above ``_PNG_MAX_ROWS`` rows it is vertically decimated
        (every Nth ping) and the stride is recorded in the npz as
        ``png_row_stride`` so nobody mistakes it for the data.
        """
        chrono = self.chronological()
        if chrono is None:
            return False
        from pathlib import Path
        import cv2
        target = Path(target)
        target.mkdir(parents=True, exist_ok=True)
        stride = max(1, int(np.ceil(chrono.shape[0] / _PNG_MAX_ROWS)))
        # PNG is newest-on-top (SonarView convention, matching the live
        # view): flip the decimated rows before colouring. The npz keeps
        # the archival buffer oldest-first (row_order tag below).
        meta = self._meta_chrono()
        rows = chrono[::stride]
        if self._model_active():
            # The mission model with ITS gamma (the dataset transfer), not
            # the operator's slider: the quick-look matches the pictures.
            unit = self._model.render_unit(rows, self._pitch,
                                           meta[::stride, 5])[::-1]
            rgba = self._unit_renderer.to_rgba(unit, limits=(0.0, 1.0))
        else:
            rgba = self._renderer.to_rgba(rows[::-1], limits=self._global_limits())
        cv2.imwrite(str(target / "waterfall.png"),
                    cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
        # The display model (curves + window + TL parameters) and each
        # row's altitude, so the PNG inverts back to the raw dB kept in
        # intensity_db (see DisplayModelSnapshot.to_json()["mapping"]).
        snap = self._model.snapshot()
        np.savez_compressed(target / "waterfall_raw.npz",
                            intensity_db=chrono,
                            layout="slant_bins_centered",
                            row_order="oldest_first",
                            png_row_order="newest_first",
                            slant_pitch_m=self._pitch,
                            swath_half_range_m=self.half_extent_m,
                            columns=self.columns,
                            png_row_stride=stride,
                            row_altitude_m=meta[:, 5].astype(np.float32),
                            row_t_s=meta[:, 0].astype(np.float64),
                            value_domain="raw_db",
                            **snap.npz_items())
        return True

    def set_display(self, settings: DisplaySettings) -> None:
        """Debounced: a slider drag fires one re-render, not one per
        tick; the re-render itself is incremental (stale tiles)."""
        self._pending_settings = settings
        self._display_timer.start()

    def _apply_pending_display(self) -> None:
        if self._pending_settings is None:
            return
        self._renderer.settings = self._pending_settings
        # Model path: gamma is the model's transfer exponent (render_unit),
        # the renderer keeps colour map + brightness on the unit values.
        self._unit_renderer.settings = self._pending_settings.with_(
            gamma=1.0, auto_range=False, vmin_db=0.0, vmax_db=1.0)
        self._pending_settings = None
        self._last_limits = None
        self._stale_tiles.update(range(len(self._tiles)))
        self._dirty = self._layout_dirty = bool(self._tiles)
        if self._enabled:
            self._force_render()

    def request_rows(self, first_row: int, last_row: int) -> None:
        """The view scrolled onto rows whose pixmaps it dropped: render
        those tiles again on the next pass."""
        lo = max(first_row, self._row0)
        hi = min(last_row, self._total - 1)
        if hi < lo:
            return
        for tile_i in range((lo - self._row0) // _TILE_ROWS,
                            (hi - self._row0) // _TILE_ROWS + 1):
            if 0 <= tile_i < len(self._tiles):
                self._dirty_tiles.add(tile_i)
        self._dirty = True

    # ---- rendering -------------------------------------------------------------
    def _mark_all_dirty(self) -> None:
        self._dirty_tiles.update(range(len(self._tiles)))
        self._dirty = self._layout_dirty = bool(self._tiles)

    def _model_active(self) -> bool:
        """Render through the display model (default) — once it has seen
        enough seabed to say anything; the raw window covers the first
        rows of a live mission."""
        return self._nadir_contrast and self._model.ready and self.columns > 0

    def render_unit(self, rows: np.ndarray, h_rows: np.ndarray,
                    gamma: Optional[float] = None) -> np.ndarray:
        """Unit brightness of built rows through the model (NaN kept)."""
        return self._model.render_unit(rows, self._pitch, h_rows, gamma)

    def _tile_image(self, tile_i: int, limits) -> QImage:
        tile = self._tiles[tile_i]
        if self._model_active():
            unit = self._model.render_unit(tile, self._pitch,
                                           self._metas[tile_i][:, 5],
                                           self._renderer.settings.gamma)
            return self._unit_renderer.to_qimage(unit, flip=False,
                                                 limits=(0.0, 1.0))
        return self._renderer.to_qimage(tile, flip=False, limits=limits)

    def _global_limits(self) -> Optional[Tuple[float, float]]:
        """One contrast window for every tile (auto mode only — the manual
        window comes from DisplaySettings).

        Model path: the tiles are rendered in unit brightness, so the
        window is the constant (0, 1) and a change of the model itself
        (its ``version``) is what marks every tile stale. Legacy path
        (nadir_contrast off, or the model not ready yet): the plain global
        raw-value histogram at the configured percentiles.
        """
        if not self._renderer.settings.auto_range:
            return None
        if self._model_active():
            return 0.0, 1.0
        total = int(self._hist.sum())
        if total == 0:
            return None
        cum = np.cumsum(self._hist) / total * 100.0
        lo_i = int(np.searchsorted(cum, self._pcts[0]))
        hi_i = int(np.searchsorted(cum, self._pcts[1]))
        centers = (self._hist_edges[:-1] + self._hist_edges[1:]) / 2.0
        lo = float(centers[min(lo_i, centers.size - 1)])
        hi = float(centers[min(hi_i, centers.size - 1)])
        if hi - lo < 1e-6:
            hi = lo + 1e-6
        return lo, hi

    def _render_if_dirty(self) -> None:
        if self._throttled:
            return
        if self._enabled and (self._dirty or self._stale_tiles):
            self._force_render()

    def _force_render(self) -> None:
        self._dirty = False
        if self._total == self._row0:
            return
        limits = self._global_limits()
        # The model changed (froze / re-fit) or just became usable: every
        # tile was rendered with a different mapping and is stale.
        key = (self._model.version, self._model_active())
        if key != self._rendered_version:
            if self._rendered_version is not None:
                self._stale_tiles.update(range(len(self._tiles)))
            self._rendered_version = key
        if limits is not None:
            last = self._last_limits
            span = limits[1] - limits[0]
            frac = (_LIMITS_REFRESH_FRACTION
                    if self._total - self._row0 < self._LIMITS_FREEZE_ROWS
                    else self._LIMITS_REFRESH_FRACTION_LATE)
            if (last is None
                    or abs(limits[0] - last[0]) > frac * span
                    or abs(limits[1] - last[1]) > frac * span):
                # The global window moved: every tile is stale -- but they
                # are re-rendered a few per pass, newest first, never all
                # at once (the full-buffer pass froze the GUI for seconds).
                self._last_limits = limits
                self._stale_tiles.update(range(len(self._tiles)))
            else:
                limits = last          # hold the window steady
        if self._layout_dirty:
            self._layout_dirty = False
            self.layout_changed.emit(self._row0, self._total, self.columns,
                                     self.half_extent_m, self.row_pitch_m)
        todo = set(self._dirty_tiles)
        self._stale_tiles -= todo
        for tile_i in sorted(self._stale_tiles, reverse=True)[:self._STALE_PER_PASS]:
            todo.add(tile_i)
            self._stale_tiles.discard(tile_i)
        for tile_i in sorted(todo):
            if tile_i >= len(self._tiles):
                continue
            img = self._tile_image(tile_i, limits)
            self.tile_updated.emit(self._row0 + tile_i * _TILE_ROWS, img)
        self._dirty_tiles.clear()
        if self._stale_tiles:
            self._dirty = True          # keep the timer coming back
        self.detections_updated.emit(self._overlay())
