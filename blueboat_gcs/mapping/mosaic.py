"""Auto-growing running-mean mosaic grid.

**Reused from the existing ``sss_helper.MosaicGrid``** — the algorithm
(scatter-add of (sum, count) per cell, chunked auto-growth anchored at the
origin) is unchanged and field-proven. Two deliberate modifications only:

* ``save()`` writes the quick-look PNG with OpenCV instead of matplotlib
  (the GUI must not depend on matplotlib); the ``.npz`` layout is
  byte-compatible with the old listener's output, so downstream analysis
  scripts keep working.
* ``project_to_world`` is carried over verbatim next to the class it
  feeds.

Rendering priorities (SonarView-like)
-------------------------------------
Where survey lines overlap, a cell has been hit by several pings and a
policy must pick the displayed value. Besides the original running
*mean*, the grid now maintains three additional per-cell planes:

* ``closest`` — sample acquired at the smallest slant range wins
  (SonarView default: near-range samples have the best resolution);
* ``oldest``  — first sample ever written to the cell wins;
* ``newest``  — most recent sample wins.

All planes are updated on every ping (a few extra vectorised scatter
ops, ~24 bytes/cell — negligible at enclosed-basin survey scale), so
switching the priority in the GUI is instant and lossless: no ping
history needs to be stored or replayed. New policies = one more plane
updated in ``add_samples`` and listed in ``PRIORITY_MODES``.

The grid is only ever accessed from the GUI thread (see core/signals.py),
so no locking is required.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

from .renderer import copper_lut


def project_to_world(robot_x: float, robot_y: float, yaw: float,
                     y_local: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rotate a ping's lateral samples into world coordinates.

    Verbatim from ``sss_helper.project_to_world``. The ping is purely
    lateral in the boat frame (x_body = 0), REP-103: +y_body = port.
    """
    x_world = robot_x - math.sin(yaw) * y_local
    y_world = robot_y + math.cos(yaw) * y_local
    return x_world, y_world


#: Selectable cell-value policies, in GUI order.
PRIORITY_MODES = ("average", "closest", "oldest", "newest")


class MosaicGrid:
    """2D raster of sonar intensity, growing on demand.

    Maintains the running mean plus the closest/oldest/newest priority
    planes (see module docstring)."""

    def __init__(self, cell_size_m: float = 0.25,
                 initial_half_extent_m: float = 50.0) -> None:
        self._cell = cell_size_m
        n = int(math.ceil(2 * initial_half_extent_m / cell_size_m))
        self._sum: np.ndarray = np.zeros((n, n), dtype=np.float64)
        self._wsum: np.ndarray = np.zeros((n, n), dtype=np.float32)
        self._count: np.ndarray = np.zeros((n, n), dtype=np.uint32)
        # Priority planes: displayed value per policy + the slant range at
        # which the 'closest' value was acquired (+inf = never written).
        self._closest: np.ndarray = np.zeros((n, n), dtype=np.float32)
        self._closest_rng: np.ndarray = np.full((n, n), np.inf,
                                                dtype=np.float32)
        self._oldest: np.ndarray = np.zeros((n, n), dtype=np.float32)
        self._newest: np.ndarray = np.zeros((n, n), dtype=np.float32)
        # World coordinates of the lower-left corner of cell [0, 0].
        self._x0: float = -initial_half_extent_m
        self._y0: float = -initial_half_extent_m
        self._chunk = int(math.ceil(50.0 / cell_size_m))  # grow by 50 m
        self._dirty = False

    # ---- geometry ----------------------------------------------------------
    @property
    def cell_size_m(self) -> float:
        return self._cell

    @property
    def shape(self) -> tuple[int, int]:
        return self._sum.shape

    @property
    def extent(self) -> tuple[float, float, float, float]:
        """(xmin, xmax, ymin, ymax) in world metres."""
        h, w = self._sum.shape
        return (self._x0, self._x0 + w * self._cell,
                self._y0, self._y0 + h * self._cell)

    @property
    def count(self) -> np.ndarray:
        return self._count

    def consume_dirty(self) -> bool:
        """True if samples were added since the last call (render gating)."""
        d, self._dirty = self._dirty, False
        return d

    # ---- accumulation (unchanged algorithm) --------------------------------
    def _world_to_cell(self, x: np.ndarray, y: np.ndarray
                       ) -> tuple[np.ndarray, np.ndarray]:
        cx = ((x - self._x0) / self._cell).astype(np.int32)
        cy = ((y - self._y0) / self._cell).astype(np.int32)
        return cx, cy

    def _ensure_contains(self, xmin: float, xmax: float,
                         ymin: float, ymax: float) -> None:
        h, w = self._sum.shape
        pad_left = pad_right = pad_bot = pad_top = 0
        if xmin < self._x0:
            pad_left = max(int(math.ceil((self._x0 - xmin) / self._cell)),
                           self._chunk)
        if xmax >= self._x0 + w * self._cell:
            pad_right = max(int(math.ceil(
                (xmax - (self._x0 + w * self._cell)) / self._cell)) + 1,
                self._chunk)
        if ymin < self._y0:
            pad_bot = max(int(math.ceil((self._y0 - ymin) / self._cell)),
                          self._chunk)
        if ymax >= self._y0 + h * self._cell:
            pad_top = max(int(math.ceil(
                (ymax - (self._y0 + h * self._cell)) / self._cell)) + 1,
                self._chunk)
        if pad_left or pad_right or pad_bot or pad_top:
            pads = ((pad_bot, pad_top), (pad_left, pad_right))
            self._sum = np.pad(self._sum, pads)
            self._wsum = np.pad(self._wsum, pads)
            self._count = np.pad(self._count, pads)
            self._closest = np.pad(self._closest, pads)
            self._closest_rng = np.pad(self._closest_rng, pads,
                                       constant_values=np.inf)
            self._oldest = np.pad(self._oldest, pads)
            self._newest = np.pad(self._newest, pads)
            self._x0 -= pad_left * self._cell
            self._y0 -= pad_bot * self._cell

    def add_samples(self, x: np.ndarray, y: np.ndarray,
                    intensity: np.ndarray,
                    slant_range: Optional[np.ndarray] = None,
                    bilinear: bool = True) -> None:
        """Accumulate one (densified) ping.

        The *mean* plane uses bilinear splatting (each sample deposits
        into its 4 surrounding cells with bilinear weights — the adjoint
        of bilinear interpolation, i.e. anti-aliased accumulation).
        The priority planes (closest/oldest/newest) and the hit count
        keep nearest-cell semantics: they answer "which measurement does
        this cell show", which must stay a single real sample.
        ``bilinear=False`` restores the legacy nearest-cell mean.
        """
        if x.size == 0:
            return
        self._ensure_contains(float(x.min()), float(x.max()),
                              float(y.min()), float(y.max()))
        h, w = self._sum.shape

        # ---- mean plane -----------------------------------------------------
        if bilinear:
            gx = (x - self._x0) / self._cell - 0.5
            gy = (y - self._y0) / self._cell - 0.5
            ix = np.floor(gx).astype(np.int32)
            iy = np.floor(gy).astype(np.int32)
            fx = (gx - ix).astype(np.float32)
            fy = (gy - iy).astype(np.float32)
            vf = intensity.astype(np.float32)
            for dx_, dy_, wgt in (
                    (0, 0, (1 - fx) * (1 - fy)), (1, 0, fx * (1 - fy)),
                    (0, 1, (1 - fx) * fy), (1, 1, fx * fy)):
                cx_, cy_ = ix + dx_, iy + dy_
                ok = (cx_ >= 0) & (cx_ < w) & (cy_ >= 0) & (cy_ < h) \
                    & (wgt > 1e-4)
                np.add.at(self._sum, (cy_[ok], cx_[ok]),
                          (vf[ok] * wgt[ok]).astype(np.float64))
                np.add.at(self._wsum, (cy_[ok], cx_[ok]), wgt[ok])

        # ---- nearest-cell planes (count + priorities) --------------------------
        cx, cy = self._world_to_cell(x, y)
        ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        idx = (cy[ok], cx[ok])
        v = intensity[ok].astype(np.float32)
        if not bilinear:
            np.add.at(self._sum, idx, v)
            np.add.at(self._wsum, idx, 1.0)

        empty_before = self._count[idx] == 0     # for 'oldest', pre-update
        np.add.at(self._count, idx, 1)

        # -- newest: plain assignment (last duplicate wins = latest sample) -
        self._newest[idx] = v

        # -- oldest: write only cells that had never been written -----------
        if empty_before.any():
            e = empty_before
            self._oldest[(idx[0][e], idx[1][e])] = v[e]

        # -- closest: keep the sample with the smallest slant range ---------
        rng = (np.abs(slant_range[ok]).astype(np.float32)
               if slant_range is not None
               else np.zeros(v.shape, dtype=np.float32))
        np.minimum.at(self._closest_rng, idx, rng)
        winners = rng <= self._closest_rng[idx]   # ties: any sample is fine
        if winners.any():
            wsel = winners
            self._closest[(idx[0][wsel], idx[1][wsel])] = v[wsel]

        self._dirty = True

    #: A cell is displayed once it has gathered at least this much
    #: bilinear weight — suppresses the faint half-cell halo a pure
    #: splat would produce at the outer swath edge.
    _MIN_WEIGHT = 0.25

    def render(self, mode: str = "average") -> np.ndarray:
        """Intensity raster for a priority mode (NaN = empty), row 0 = ymin."""
        if mode == "average":
            with np.errstate(invalid="ignore", divide="ignore"):
                return np.where(self._wsum >= self._MIN_WEIGHT,
                                self._sum / self._wsum, np.nan)
        plane = {"closest": self._closest, "oldest": self._oldest,
                 "newest": self._newest}.get(mode)
        if plane is None:
            raise ValueError(f"Unknown priority mode: {mode!r}")
        return np.where(self._count > 0, plane, np.float32(np.nan))

    # ---- persistence ---------------------------------------------------------
    def save(self, log_root: Path, prefix: str = "sonar_mosaic"
             ) -> Tuple[Path, Path]:
        """Save as compact .npz (same keys as the legacy listener) + PNG."""
        log_root.mkdir(parents=True, exist_ok=True)
        img = self.render()
        npz_path = log_root / f"{prefix}.npz"
        png_path = log_root / f"{prefix}.png"
        np.savez_compressed(
            npz_path,
            # Legacy keys — byte-compatible with the old listener output.
            mean_intensity=img.astype(np.float32),
            count=self._count,
            cell_size_m=self._cell,
            x0=self._x0,
            y0=self._y0,
            # New priority planes (extra keys; old scripts ignore them).
            closest_intensity=self.render("closest").astype(np.float32),
            oldest_intensity=self.render("oldest").astype(np.float32),
            newest_intensity=self.render("newest").astype(np.float32),
        )
        valid = img[np.isfinite(img)]
        vmin, vmax = (np.percentile(valid, [2, 98]) if valid.size
                      else (0.0, 1.0))
        norm = np.clip((np.nan_to_num(img, nan=vmin) - vmin)
                       / max(vmax - vmin, 1e-9), 0.0, 1.0)
        rgb = copper_lut()[(norm * 255).astype(np.uint8)]
        # origin='lower' equivalent: flip rows for image convention.
        cv2.imwrite(str(png_path), cv2.cvtColor(np.flipud(rgb), cv2.COLOR_RGB2BGR))
        return npz_path, png_path
