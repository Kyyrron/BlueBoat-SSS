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
``.npz`` always contain raw data (see mapping/interpolation.py).
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QImage

from ..config.settings import AppConfig
from ..mapping.interpolation import fill_small_gaps
from ..mapping.mosaic import MosaicGrid, project_to_world
from ..mapping.rasterizer import PingRasterizer
from ..mapping.renderer import MosaicRenderer
from ..models.sonar import SonarPing


class MosaicService(QObject):
    """Owns the mosaic grid; produces display rasters and saved artifacts."""

    #: QImage, extent (xmin, xmax, ymin, ymax), cell size [m]
    raster_updated = Signal(QImage, tuple, float)
    #: All accumulated SSS data was discarded ("Clear SSS data").
    cleared = Signal()
    #: Emitted when the ground-sample distance changes [m/cell].
    resolution_changed = Signal(float)

    def __init__(self, config: AppConfig) -> None:
        super().__init__()
        self._config = config
        # Resolution is adaptive: `_cell_size` starts at the configured
        # value and follows the data's actual across-track sample
        # spacing when mosaic.auto_cell_size is set (see
        # _maybe_autotune). A fixed 25 cm grid was throwing away
        # most of the sensor's resolution on short-range logs.
        self._cell_size = float(config.mosaic.cell_size_m)
        self._auto_mode = bool(config.mosaic.auto_cell_size)
        self._cell_tuned = not self._auto_mode
        # Auto mode re-checks the data-supported GSD periodically (a
        # range change mid-mission changes the sample spacing); the
        # first ping is checked immediately.
        self._autotune_countdown = 0
        self._grid = self._new_grid()
        self._rasterizer = PingRasterizer(self._cell_size)
        self._renderer = MosaicRenderer(
            percentiles=tuple(config.mosaic.contrast_percentiles))
        self._interpolate = False
        self._priority = "average"          # see MosaicGrid.PRIORITY_MODES
        self._depth_t: list[float] = []
        self._depth_z: list[float] = []
        self._traj: list[Tuple[float, float]] = []
        self._t_first: Optional[float] = None

        self._timer = QTimer(self)
        self._timer.setInterval(int(1000.0 / config.mosaic.render_hz))
        self._timer.timeout.connect(self._render_if_dirty)
        self._timer.start()

    def _new_grid(self) -> MosaicGrid:
        return MosaicGrid(
            cell_size_m=self._cell_size,
            initial_half_extent_m=self._config.mosaic.initial_half_extent_m)

    # ---- resolution -----------------------------------------------------------
    #: Re-check the data-supported GSD every this many pings in auto mode.
    AUTOTUNE_EVERY_PINGS = 20

    def _derive_cell_size(self, ping: SonarPing) -> Optional[float]:
        """Derive the ground-sample distance from the data itself.

        The natural limit is the across-track sample spacing,
        range / num_results (33 mm for a 20 m / 600-sample log, 133 mm
        for our 80 m setting). Rendering finer than that invents detail;
        rendering much coarser — as the old fixed 0.25 m grid did —
        discards it. One sample per cell is the honest choice, clamped
        to keep memory sane on very long ranges.
        """
        y = np.abs(ping.y_local)
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
        """
        self._autotune_countdown -= 1
        if self._autotune_countdown > 0:
            return
        self._autotune_countdown = self.AUTOTUNE_EVERY_PINGS
        cell = self._derive_cell_size(ping)
        if cell is None:
            return
        if not self._cell_tuned:
            self._cell_tuned = True
            if abs(cell - self._cell_size) / max(self._cell_size,
                                                 1e-6) >= 0.15:
                self.set_cell_size(cell)
            return
        if cell < self._cell_size * 0.85:
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
        self._cell_size = cell_m
        old_grid = self._grid
        self._grid = self._new_grid()
        self._grid.resample_from(old_grid)
        # A fresh rasterizer holds no previous pose: no along-track
        # interpolation is drawn across the resolution change.
        self._rasterizer = PingRasterizer(cell_m)
        self.resolution_changed.emit(cell_m)

    @property
    def cell_size_m(self) -> float:
        return self._cell_size

    # ---- ingestion ------------------------------------------------------------
    def on_sonar_ping(self, ping: SonarPing) -> None:
        if self._auto_mode:
            self._maybe_autotune(ping)
        if self._config.mosaic.densify:
            xw, yw, v, rng = self._rasterizer.rasterize(ping)
        else:                                   # legacy point-scatter path
            xw, yw = project_to_world(ping.robot_x, ping.robot_y, ping.yaw,
                                      ping.y_local)
            v, rng = ping.intensity_db, np.abs(ping.y_local)
        self._grid.add_samples(xw, yw, v, slant_range=rng,
                               bilinear=self._config.mosaic.bilinear_splat)
        if self._t_first is None:
            self._t_first = ping.t
        self._depth_t.append(ping.t - self._t_first)
        self._depth_z.append(ping.water_depth)
        self._traj.append((ping.robot_x, ping.robot_y))

    def reset_tracking(self) -> None:
        """START / data resumption: never interpolate across the break."""
        self._rasterizer.reset()

    def clear(self) -> None:
        """'Clear SSS data': discard the accumulated grid, keep everything
        else (trajectory, detections, view transform...). New pings keep
        accumulating immediately into the fresh grid."""
        self._grid = self._new_grid()
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
        self._force_render()

    def _render_if_dirty(self) -> None:
        if self._grid.consume_dirty():
            self._force_render()

    def _force_render(self) -> None:
        mean = self._grid.render(self._priority)
        if not np.isfinite(mean).any():
            return
        if self._interpolate:
            mean, _mask = fill_small_gaps(
                mean, self._grid.count, self._grid.cell_size_m,
                max_gap_m=self._config.interpolation.max_gap_m,
                min_neighbors=self._config.interpolation.min_neighbors)
        image = self._renderer.to_qimage(mean)
        self.raster_updated.emit(image, self._grid.extent,
                                 self._grid.cell_size_m)

    # ---- persistence (same artifacts as the legacy listener) ---------------------
    @property
    def has_data(self) -> bool:
        return self._t_first is not None

    def save_into(self, target: Path) -> Optional[Path]:
        """Write mosaic .npz/.png + trajectory/depth CSV into ``target``."""
        if self._t_first is None:
            return None
        target.mkdir(parents=True, exist_ok=True)
        self._grid.save(target)  # raw data only, never interpolated
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
