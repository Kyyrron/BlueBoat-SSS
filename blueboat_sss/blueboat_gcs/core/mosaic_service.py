"""Mosaic service: connects the ping stream to the map raster.

Runs entirely in the GUI thread (fed by queued signals):

* on every ``SonarPing`` (~28 Hz): project the pre-corrected lateral
  samples to world coordinates (reused ``project_to_world``) and
  scatter-add them into the reused ``MosaicGrid`` — a sub-millisecond
  numpy operation;
* on a QTimer at ``render_hz`` (default 4 Hz): if the grid changed,
  render mean intensities (optionally gap-filled) to a QImage and emit
  ``raster_updated`` for the map layer. Decoupling ingestion from
  rendering is what keeps the GUI smooth at any ping rate.

Interpolation is applied at render time only; the grid and the saved
``.npz`` contain unrendered data (see mapping/interpolation.py).

Intensity domain (2026-09-05): when the window injects its
``DisplayModel`` (``core/display_model``) every sample is normalised
**at ingestion** — two-way transmission loss removed, the mission's
seabed curve in ``r/h`` divided out — so overlapping passes at different
ranges and altitudes accumulate comparable values (a raw-dB mean across
passes was not comparable: the stream is pre-TVG) and the mosaic reads
like the waterfall and the AI pictures. The saved ``.npz`` is tagged
``value_domain`` and carries the model. While the model is still
warming up live, pings are held back and ingested in order at the
freeze, so the whole grid is on one mapping. Without a model (bench,
tests, ``nadir_contrast`` off) the grid holds raw dB as before.
"""

from __future__ import annotations

from dataclasses import replace as dc_replace
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import time
import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QImage

from ..config.settings import AppConfig
from ..mapping.interpolation import fill_small_gaps
from ..mapping.mosaic import MosaicGrid, project_to_world
from ..mapping.rasterizer import PingRasterizer
from ..mapping.renderer import DisplaySettings, MosaicRenderer
from ..models.sonar import SonarPing
from .display_model import DisplayModel


class MosaicService(QObject):
    """Owns the mosaic grid; produces display rasters and saved artifacts."""

    #: QImage, extent (xmin, xmax, ymin, ymax), cell size [m]
    raster_updated = Signal(QImage, tuple, float)
    #: All accumulated SSS data was discarded ("Clear SSS data").
    cleared = Signal()
    #: Emitted when the ground-sample distance changes [m/cell].
    resolution_changed = Signal(float)

    def __init__(self, config: AppConfig,
                 model: Optional[DisplayModel] = None) -> None:
        super().__init__()
        self._config = config
        # The window's display model (see module docstring); None = raw dB.
        self._model = model if config.mosaic.nadir_contrast else None
        self._warmup: List[SonarPing] = []
        self._warmup_cap = int(getattr(config.display, "warmup_rows", 300)) + 64
        self._unit_renderer = MosaicRenderer()
        self._unit_renderer.settings = DisplaySettings(gamma=1.0)
        # Resolution is adaptive: `_cell_size` starts at the configured
        # value and follows the data's actual across-track sample
        # spacing when mosaic.auto_cell_size is set (see
        # _maybe_autotune). A fixed 25 cm grid was throwing away
        # most of the sensor's resolution on short-range logs.
        self._cell_size = float(config.mosaic.cell_size_m)
        self._auto_mode = bool(config.mosaic.auto_cell_size)
        self._cell_tuned = not self._auto_mode
        self._budget_floor = 0.0     # cell size forced by max_grid_cells
        # Auto mode re-checks the data-supported GSD periodically (a
        # range change mid-mission changes the sample spacing); the
        # first ping is checked immediately.
        self._autotune_countdown = 0
        self._grid = self._new_grid()
        self._rasterizer = PingRasterizer(self._cell_size)
        self._renderer = MosaicRenderer(
            percentiles=tuple(config.mosaic.contrast_percentiles))
        self._interpolate = False
        # Overlap policy default from config (see MosaicGrid.PRIORITY_MODES);
        # "closest" (smallest slant range) is SonarView's default.
        self._priority = str(config.mosaic.priority_mode)
        self._depth_t: list[float] = []
        self._depth_z: list[float] = []
        self._traj: list[Tuple[float, float]] = []
        self._t_first: Optional[float] = None

        self._timer = QTimer(self)
        self._timer.setInterval(int(1000.0 / config.mosaic.mosaic_render_hz))
        self._timer.timeout.connect(self._render_if_dirty)
        self._timer.start()
        # Partial re-colormapping: the last rendered raster and its RGBA
        # are kept so a ping that touched a sliver only re-colormaps that
        # sliver (the full-raster colormap was ~40 ms, 4x/s).
        self._rgba_cache: Optional[np.ndarray] = None
        self._cache_key: Optional[tuple] = None
        self._throttled = False
        # Auto-range window of the whole raster, refreshed at most every
        # _LIMITS_REFRESH_S so the partial (sliver) re-colormap can use a
        # stable window between full passes.
        self._limits_cache: Optional[tuple[float, tuple[float, float]]] = None

    def throttle(self, on: bool) -> None:
        """Skip renders while the GUI thread is behind the ping stream
        (data still accumulates; rendering catches up afterwards)."""
        self._throttled = bool(on)

    def _new_grid(self) -> MosaicGrid:
        g = MosaicGrid(
            cell_size_m=self._cell_size,
            initial_half_extent_m=self._config.mosaic.initial_half_extent_m)
        g.max_cells = int(self._config.mosaic.max_grid_cells)
        g.max_extent_m = float(self._config.mosaic.max_extent_m)
        return g

    # ---- resolution -----------------------------------------------------------
    #: Re-check the data-supported GSD every this many pings in auto mode.
    AUTOTUNE_EVERY_PINGS = 20
    #: Seconds an auto-range window is held between full re-derivations.
    _LIMITS_REFRESH_S = 5.0

    def _derive_cell_size(self, ping: SonarPing) -> Optional[float]:
        """Derive the ground-sample distance from the data itself.

        The natural limit is the across-track sample spacing,
        range / num_results (33 mm for a 20 m / 600-sample log, 133 mm
        for our 80 m setting). Rendering finer than that invents detail;
        rendering much coarser — as the old fixed 0.25 m grid did —
        discards it. One sample per cell is the honest choice, clamped
        to keep memory sane on very long ranges.
        """
        # ONE side only. A two-sided ping concatenates port and starboard
        # and any transducer-offset asymmetry interleaves their bins, so
        # the median spacing of the merged array collapsed to the
        # min_cell_size clamp (3x the cells, the 2026-09-03 crash).
        yl = np.asarray(ping.y_local, dtype=np.float64)
        port = yl[yl > 0.0]
        stbd = -yl[yl < 0.0]
        y = port if port.size >= stbd.size else stbd
        y = np.sort(y[np.isfinite(y)])
        if y.size < 8:
            return None
        # Ground-range spacing is not uniform: near nadir it stretches
        # (dg/di = slant/ground * ds/di), and it tightens to the slant
        # sample spacing at long range, where most of the swath area
        # lies. Take the median spacing over the outer half of the
        # swath — that is the resolution actually worth rendering.
        d = np.diff(y[y.size // 2:])
        d = d[d > 0]
        if d.size == 0:
            return None
        spacing = float(np.median(d))
        if spacing <= 0.0:
            return None
        return min(max(spacing, self._config.mosaic.min_cell_size_m),
                   self._config.mosaic.max_cell_size_m)

    def _maybe_autotune(self, ping: SonarPing) -> None:
        """Keep the grid at the best cell size the data supports.

        First successful derivation sets the working resolution (either
        way); afterwards only *refinements* >15 % are applied — a
        coarser acquisition mid-mission never degrades what is already
        on screen, its data simply lands blockier in the finer grid.
        Any change preserves accumulated data (see set_cell_size).

        The cell-count budget (``mosaic.max_grid_cells``) takes
        precedence: as the surveyed area grows, the grid coarsens rather
        than growing without bound (GUI-thread cost and memory both
        scale with cell count — an unbounded grid is what made the app
        crawl and starve the ROS executor). ``_budget_floor`` stops the
        autotuner from re-refining below a budget-forced cell size.
        """
        self._autotune_countdown -= 1
        if self._autotune_countdown > 0:
            return
        self._autotune_countdown = self.AUTOTUNE_EVERY_PINGS
        self._enforce_cell_budget()
        cell = self._derive_cell_size(ping)
        if cell is None:
            return
        cell = max(cell, self._budget_floor)
        if not self._cell_tuned:
            self._cell_tuned = True
            if abs(cell - self._cell_size) / max(self._cell_size,
                                                 1e-6) >= 0.15:
                self.set_cell_size(cell)
            return
        if cell < self._cell_size * 0.85:
            self.set_cell_size(cell)

    def _follow_grid_cell(self) -> None:
        """The grid may have coarsened itself on a growth (budget honoured
        before allocation): the service's cell size, rasterizer and
        budget floor follow it."""
        if abs(self._grid.cell_size_m - self._cell_size) > 1e-9:
            self._cell_size = self._grid.cell_size_m
            self._budget_floor = max(self._budget_floor, self._cell_size)
            self._rasterizer = PingRasterizer(self._cell_size)
            self._rgba_cache = None
            self.resolution_changed.emit(self._cell_size)

    def _enforce_cell_budget(self) -> None:
        self._follow_grid_cell()
        h, w = self._grid.shape
        budget = int(self._config.mosaic.max_grid_cells)
        if budget <= 0 or h * w <= budget:
            return
        cell = self._cell_size * float(np.sqrt(h * w / budget)) * 1.05
        cell = min(cell, self._config.mosaic.max_cell_size_m)
        if cell > self._cell_size:
            self._budget_floor = cell
            self.set_cell_size(cell)

    def enable_auto_resolution(self) -> None:
        """Auto mode: re-derive the cell size from the next ping."""
        self._auto_mode = True
        self._cell_tuned = False
        self._autotune_countdown = 0

    def set_fixed_cell_size(self, cell_m: float) -> None:
        """Manual override: fix the GSD and stop auto-tuning."""
        self._auto_mode = False
        self.set_cell_size(cell_m)

    def set_cell_size(self, cell_m: float) -> None:
        """Change mosaic resolution, preserving accumulated data.

        The old grid is resampled into the new one (see
        MosaicGrid.resample_from): old data keeps its native resolution,
        new pings accumulate at the new cell size. Nothing is cleared.
        """
        cell_m = min(max(float(cell_m), self._config.mosaic.min_cell_size_m),
                     self._config.mosaic.max_cell_size_m)
        if abs(cell_m - self._cell_size) < 1e-9:
            return
        ratio = cell_m / self._cell_size
        if ratio >= 1.5 and abs(ratio - round(ratio)) < 0.35:
            # Coarsening by an integer factor: aggregate in place -- no
            # second grid, no multi-hundred-MB transient, no GUI freeze.
            self._grid.coarsen(int(round(ratio)))
            self._cell_size = self._grid.cell_size_m
        else:
            self._cell_size = cell_m
            old_grid = self._grid
            self._grid = self._new_grid()
            self._grid.resample_from(old_grid)
            del old_grid
        # A fresh rasterizer holds no previous pose: no along-track
        # interpolation is drawn across the resolution change.
        self._rasterizer = PingRasterizer(cell_m)
        self.resolution_changed.emit(cell_m)

    @property
    def cell_size_m(self) -> float:
        return self._cell_size

    # ---- ingestion ------------------------------------------------------------
    @property
    def model(self) -> Optional[DisplayModel]:
        return self._model

    def set_model(self, model: Optional[DisplayModel]) -> None:
        """Inject / replace the display model (new pings only)."""
        self._model = model if self._config.mosaic.nadir_contrast else None

    def _normalised(self, ping: SonarPing) -> SonarPing:
        """The ping with ``intensity_db`` replaced by the model's
        normalised level (per sample: r = hypot(|y|, h), its side)."""
        snap = self._model.snapshot()
        h = float(ping.bottom_slant_m)
        if h <= 0.0:
            h = max(float(ping.water_depth), 0.0)
        y = np.asarray(ping.y_local, dtype=np.float64)
        db = np.asarray(ping.intensity_db, dtype=np.float32)
        e = db.copy()
        hh = h if h > 0.0 else snap.fallback_h
        r = np.hypot(np.abs(y), hh)
        for side, mask in ((0, y > 0.0), (1, y < 0.0)):
            if mask.any():
                e[mask] = snap.normalise_samples(db[mask], r[mask], h, side)
        return dc_replace(ping, intensity_db=e)

    def flush_warmup(self) -> None:
        """Ingest the pings held back while the model warmed up (STOP,
        or the model froze)."""
        if not self._warmup:
            return
        held, self._warmup = self._warmup, []
        for p in held:
            self._ingest(p)

    def on_sonar_ping(self, ping: SonarPing) -> None:
        if self._auto_mode:
            self._maybe_autotune(ping)
        else:
            # Manual resolution still honours the cell budget: the grid
            # grows with the surveyed area and must stay bounded.
            self._autotune_countdown -= 1
            if self._autotune_countdown <= 0:
                self._autotune_countdown = self.AUTOTUNE_EVERY_PINGS
                self._enforce_cell_budget()
        if self._model is not None:
            if not self._model.frozen:
                # Hold back until the mapping is final, so the grid is
                # on ONE normalisation (bounded: the oldest are ingested
                # with the running estimate if the warm-up overruns).
                self._warmup.append(ping)
                if len(self._warmup) > self._warmup_cap:
                    self._ingest(self._warmup.pop(0))
                return
            self.flush_warmup()
        self._ingest(ping)

    def _ingest(self, ping: SonarPing) -> None:
        if self._model is not None and self._model.ready:
            ping = self._normalised(ping)
        if self._config.mosaic.densify:
            xw, yw, v, rng = self._rasterizer.rasterize(ping)
        else:                                   # legacy point-scatter path
            xw, yw = project_to_world(ping.robot_x, ping.robot_y, ping.yaw,
                                      ping.y_local)
            v, rng = ping.intensity_db, np.abs(ping.y_local)
        self._grid.add_samples(xw, yw, v, slant_range=rng,
                               bilinear=self._config.mosaic.bilinear_splat)
        if self._grid.cell_size_m != self._cell_size:
            self._follow_grid_cell()
        if self._t_first is None:
            self._t_first = ping.t
        self._depth_t.append(ping.t - self._t_first)
        self._depth_z.append(ping.water_depth)
        self._traj.append((ping.robot_x, ping.robot_y))

    def reset_tracking(self) -> None:
        """START / data resumption: never interpolate across the break."""
        self.flush_warmup()
        self._rasterizer.reset()

    def clear(self) -> None:
        """'Clear SSS data': discard the accumulated grid, keep everything
        else (trajectory, detections, view transform...). New pings keep
        accumulating immediately into the fresh grid."""
        self._grid = self._new_grid()
        self._warmup.clear()
        self._rasterizer.reset()
        self._depth_t.clear()
        self._depth_z.clear()
        self._traj.clear()
        self._t_first = None
        self.cleared.emit()

    # ---- display -----------------------------------------------------------------
    def set_interpolation(self, enabled: bool) -> None:
        if enabled != self._interpolate:
            self._interpolate = enabled
            self._force_render()

    def set_priority_mode(self, mode: str) -> None:
        """Cell-value policy: 'average' | 'closest' | 'oldest' | 'newest'."""
        if mode != self._priority:
            self._priority = mode
            self._force_render()

    def set_display(self, settings) -> None:
        """Apply operator display settings (visualization only)."""
        self._renderer.settings = settings
        self._unit_renderer.settings = settings.with_(
            gamma=1.0, auto_range=False, vmin_db=0.0, vmax_db=1.0)
        self._force_render()

    def _model_active(self) -> bool:
        return self._model is not None and self._model.ready

    def _colour(self, values: np.ndarray, limits) -> np.ndarray:
        """RGBA of a (normalised-dB or raw) raster: through the model's
        transfer with the operator's gamma when active, else the legacy
        percentile window."""
        if self._model_active():
            unit = self._model.snapshot().to_unit(values,
                                                  self._renderer.settings.gamma)
            return self._unit_renderer.to_rgba(unit, limits=(0.0, 1.0))
        return self._renderer.to_rgba(values, limits=limits)

    def _render_if_dirty(self) -> None:
        if self._throttled:
            return
        if self._grid.consume_dirty():
            self._force_render(partial=True)

    def _force_render(self, partial: bool = False) -> None:
        # Display decimation: colormap at most max_render_pixels cells.
        # The raster still spans the full extent (every Nth cell); the
        # grid itself keeps full resolution.
        h, w = self._grid.shape
        budget = max(int(self._config.mosaic.max_render_pixels), 1)
        stride = max(1, int(np.ceil(np.sqrt(h * w / budget))))
        bbox = self._grid.consume_dirty_bbox()
        settings = self._renderer.settings
        active = self._model_active()
        key = (h, w, stride, self._priority, self._interpolate, id(settings),
               active, self._model.version if active else None)
        limits = None
        now = time.monotonic()
        if active:
            limits = (0.0, 1.0)             # unit domain: fixed window
        elif settings.auto_range and self._limits_cache is not None \
                and now - self._limits_cache[0] < self._LIMITS_REFRESH_S:
            limits = self._limits_cache[1]
        cache_ok = (partial and bbox is not None and self._rgba_cache is not None
                    and self._cache_key == key
                    and (limits is not None or not settings.auto_range))
        cell = self._grid.cell_size_m * stride
        if cache_ok:
            y0, y1, x0, x1 = bbox
            ry0, ry1 = y0 // stride, -(-y1 // stride)
            rx0, rx1 = x0 // stride, -(-x1 // stride)
            sub = self._grid.render(self._priority, stride)[ry0:ry1, rx0:rx1]
            if self._interpolate:
                count = self._grid.count[::stride, ::stride][ry0:ry1, rx0:rx1]
                sub, _mask = fill_small_gaps(
                    sub, count, cell,
                    max_gap_m=self._config.interpolation.max_gap_m,
                    min_neighbors=self._config.interpolation.min_neighbors)
            self._rgba_cache[ry0:ry1, rx0:rx1] = self._colour(sub, limits)
            rgba = self._rgba_cache
        else:
            mean = self._grid.render(self._priority, stride)
            if not np.isfinite(mean).any():
                return
            if self._interpolate:
                count = self._grid.count[::stride, ::stride]
                mean, _mask = fill_small_gaps(
                    mean, count, cell,
                    max_gap_m=self._config.interpolation.max_gap_m,
                    min_neighbors=self._config.interpolation.min_neighbors)
            if settings.auto_range and not active:
                limits = self._renderer.auto_limits(mean)
                self._limits_cache = (now, limits)
            rgba = self._colour(mean, limits)
            self._rgba_cache, self._cache_key = rgba, key
        image = self._renderer.rgba_to_qimage(rgba)
        self.raster_updated.emit(image, self._grid.extent, cell)

    # ---- persistence (same artifacts as the legacy listener) ---------------------
    @property
    def has_data(self) -> bool:
        return self._t_first is not None or bool(self._warmup)

    def save_into(self, target: Path) -> Optional[Path]:
        """Write mosaic .npz/.png + trajectory/depth CSV into ``target``."""
        # Pings still held for the model warm-up are data too: a session
        # shorter than the warm-up must not export an empty mosaic.
        self.flush_warmup()
        if self._t_first is None:
            return None
        target.mkdir(parents=True, exist_ok=True)
        extra = {"value_domain": "raw_db"}
        if self._model is not None:
            extra = {"value_domain": ("normalised_db" if self._model.ready
                                      else "raw_db"),
                     **self._model.snapshot().npz_items()}
        self._grid.save(target, extra=extra)  # unrendered, never interpolated
        import csv
        with open(target / "boat_trajectory.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_since_first_s", "x_m", "y_m", "depth_m"])
            for t, (x, y), z in zip(self._depth_t, self._traj, self._depth_z):
                w.writerow([f"{t:.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}"])
        return target

    def save(self) -> Optional[Path]:
        """Legacy quick-save into data_root/<date> (used when no recording
        session is active — sessions call save_into on their own folder)."""
        stamp = datetime.today().strftime("%Y_%m_%d-%H_%M")
        return self.save_into(
            Path(self._config.data_root).expanduser() / stamp)
