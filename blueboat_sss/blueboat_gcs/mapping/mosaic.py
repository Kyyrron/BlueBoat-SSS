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
        #: Cell budget honoured *before* any allocation (0 = unbounded).
        #: When a growth would exceed it the grid coarsens in place first
        #: (block aggregation, old + new planes never both resident) --
        #: the previous after-the-fact budget check let a 20 Hz two-sided
        #: stream allocate multi-GB planes and die (2026-09-03).
        self.max_cells = 0
        #: Largest extent [m] a single growth may request; a pose glitch
        #: that jumps the boat kilometres away is refused, not allocated.
        self.max_extent_m = 5000.0
        #: Cells touched since the last consume_dirty_bbox(), as
        #: (y0, y1, x0, x1) in grid indices, for partial re-colormapping.
        self._dirty_bbox: Optional[tuple[int, int, int, int]] = None
        self.refused_growths = 0
        self.coarsenings = 0

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

    def _grown_shape(self, xmin: float, xmax: float, ymin: float,
                     ymax: float) -> tuple[int, int, int, int, int, int]:
        """(pad_bot, pad_top, pad_left, pad_right, new_h, new_w) for an
        extent, without allocating anything."""
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
        return (pad_bot, pad_top, pad_left, pad_right,
                h + pad_bot + pad_top, w + pad_left + pad_right)

    def coarsen(self, factor: int) -> None:
        """Aggregate the grid in place by an integer ``factor`` per axis.

        The mean plane transfers its weighted sums exactly; the count
        sums; ``closest`` keeps the block's smallest-slant sample; ``oldest``
        / ``newest`` keep the block's first / last written cell (the
        per-cell acquisition order inside a block is not stored, so this
        is the best available proxy). Peak memory is one plane's worth
        of temporaries, never a second full grid.
        """
        f = int(factor)
        if f <= 1:
            return
        h, w = self._sum.shape
        hh, ww = -(-h // f) * f, -(-w // f) * f       # pad up to a multiple
        pads = ((0, hh - h), (0, ww - w))

        def blocks(plane: np.ndarray, fill=0) -> np.ndarray:
            p = np.pad(plane, pads, constant_values=fill) if pads != ((0, 0), (0, 0)) else plane
            return p.reshape(hh // f, f, ww // f, f)

        self._sum = blocks(self._sum).sum(axis=(1, 3))
        self._wsum = blocks(self._wsum).sum(axis=(1, 3), dtype=np.float32)
        count = blocks(self._count).sum(axis=(1, 3), dtype=np.uint32)
        rng_b = blocks(self._closest_rng, np.inf)
        flat = rng_b.transpose(0, 2, 1, 3).reshape(hh // f, ww // f, f * f)
        arg = flat.argmin(axis=2)
        pick = lambda plane: (blocks(plane).transpose(0, 2, 1, 3)  # noqa: E731
                              .reshape(hh // f, ww // f, f * f))
        self._closest = np.take_along_axis(pick(self._closest), arg[..., None], 2)[..., 0]
        self._closest_rng = np.take_along_axis(flat, arg[..., None], 2)[..., 0]
        written = pick(self._count) > 0
        first = written.argmax(axis=2)
        last = f * f - 1 - written[..., ::-1].argmax(axis=2)
        self._oldest = np.take_along_axis(pick(self._oldest), first[..., None], 2)[..., 0]
        self._newest = np.take_along_axis(pick(self._newest), last[..., None], 2)[..., 0]
        self._count = count
        self._cell *= f
        self._chunk = int(math.ceil(50.0 / self._cell))
        self.coarsenings += 1
        self._dirty = True
        self._dirty_bbox = None

    def _ensure_contains(self, xmin: float, xmax: float,
                         ymin: float, ymax: float) -> bool:
        """Grow (or coarsen, then grow) so the extent fits. Returns False
        when the request is refused as a pose glitch."""
        if (xmax - xmin > self.max_extent_m or ymax - ymin > self.max_extent_m):
            self.refused_growths += 1
            return False
        h, w = self._sum.shape
        if self.max_cells > 0:
            *_, nh, nw = self._grown_shape(xmin, xmax, ymin, ymax)
            if nh * nw > self.max_cells:
                span_x = max(xmax, self._x0 + w * self._cell) - min(xmin, self._x0)
                span_y = max(ymax, self._y0 + h * self._cell) - min(ymin, self._y0)
                need = (span_x + 100.0 * self._cell) * (span_y + 100.0 * self._cell)
                f = int(math.ceil(math.sqrt(need / (self.max_cells * self._cell ** 2))))
                if f > 1:
                    self.coarsen(f)
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
            self._dirty_bbox = None            # indices shifted: full render
        return True

    @staticmethod
    def _bbox_scatter_add(plane: np.ndarray, cy: np.ndarray, cx: np.ndarray,
                          weights: Optional[np.ndarray]) -> None:
        """Scatter-add restricted to the samples' bounding box.

        ``np.add.at`` costs ~1.5 µs per element (it dominated the
        per-ping ingest at survey scale); one ``np.bincount`` over the
        touched bounding box plus a vectorized block add is an order of
        magnitude faster while touching the same cells.
        """
        y0, y1 = int(cy.min()), int(cy.max()) + 1
        x0, x1 = int(cx.min()), int(cx.max()) + 1
        bw = x1 - x0
        flat = (cy - y0).astype(np.int64) * bw + (cx - x0)
        acc = np.bincount(flat, weights=weights,
                          minlength=(y1 - y0) * bw)
        plane[y0:y1, x0:x1] += acc.reshape(y1 - y0, bw).astype(plane.dtype,
                                                               copy=False)

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
        if not self._ensure_contains(float(x.min()), float(x.max()),
                                     float(y.min()), float(y.max())):
            return
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
                if ok.any():
                    self._bbox_scatter_add(
                        self._sum, cy_[ok], cx_[ok],
                        (vf[ok] * wgt[ok]).astype(np.float64))
                    self._bbox_scatter_add(self._wsum, cy_[ok], cx_[ok],
                                           wgt[ok].astype(np.float64))

        # ---- nearest-cell planes (count + priorities) --------------------------
        cx, cy = self._world_to_cell(x, y)
        ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        idx = (cy[ok], cx[ok])
        v = intensity[ok].astype(np.float32)
        if v.size == 0:
            self._dirty = True
            return
        if not bilinear:
            self._bbox_scatter_add(self._sum, idx[0], idx[1],
                                   v.astype(np.float64))
            self._bbox_scatter_add(self._wsum, idx[0], idx[1],
                                   np.ones(v.size, dtype=np.float64))

        empty_before = self._count[idx] == 0     # for 'oldest', pre-update
        self._bbox_scatter_add(self._count, idx[0], idx[1], None)

        # -- newest: plain assignment (last duplicate wins = latest sample) -
        self._newest[idx] = v

        # -- oldest: write only cells that had never been written -----------
        if empty_before.any():
            e = empty_before
            self._oldest[(idx[0][e], idx[1][e])] = v[e]

        # -- closest: keep the sample with the smallest slant range ---------
        # Sorting far -> near makes the LAST duplicate assignment the
        # nearest sample, replacing the (slow) np.minimum.at scatter.
        rng = (np.abs(slant_range[ok]).astype(np.float32)
               if slant_range is not None
               else np.zeros(v.shape, dtype=np.float32))
        order = np.argsort(-rng, kind="stable")
        oy, ox = idx[0][order], idx[1][order]
        orng, ov = rng[order], v[order]
        winners = orng <= self._closest_rng[oy, ox]
        if winners.any():
            self._closest_rng[oy[winners], ox[winners]] = orng[winners]
            self._closest[oy[winners], ox[winners]] = ov[winners]

        self._dirty = True
        y0, y1 = int(idx[0].min()) - 1, int(idx[0].max()) + 2
        x0, x1 = int(idx[1].min()) - 1, int(idx[1].max()) + 2
        if self._dirty_bbox is None:
            self._dirty_bbox = (max(y0, 0), min(y1, h), max(x0, 0), min(x1, w))
        else:
            b = self._dirty_bbox
            self._dirty_bbox = (min(b[0], max(y0, 0)), max(b[1], min(y1, h)),
                                min(b[2], max(x0, 0)), max(b[3], min(x1, w)))

    def consume_dirty_bbox(self) -> Optional[tuple[int, int, int, int]]:
        """Cells touched since the last call as (y0, y1, x0, x1), or
        None when the whole raster must be re-rendered (growth,
        coarsening, resolution change)."""
        b, self._dirty_bbox = self._dirty_bbox, None
        return b

    # ---- resolution change ---------------------------------------------------
    def resample_from(self, old: "MosaicGrid") -> None:
        """Carry another grid's accumulated data into this *empty* grid.

        Used on resolution changes so the mosaic is never wiped. Old data
        keeps its native resolution: a coarse cell paints the whole block
        of finer cells it covered (no finer detail ever existed to
        recover), and fine cells aggregate into a coarser one. The mean
        plane transfers its weighted sums, so data acquired after the
        change blends with the old exactly as if both had always been
        accumulated here.
        """
        mask = (old._count > 0) | (old._wsum > 0)
        if not mask.any():
            return
        self.max_cells = old.max_cells
        self.max_extent_m = old.max_extent_m
        iy, ix = np.nonzero(mask)
        xmin = old._x0 + float(ix.min()) * old._cell
        xmax = old._x0 + (float(ix.max()) + 1.0) * old._cell
        ymin = old._y0 + float(iy.min()) * old._cell
        ymax = old._y0 + (float(iy.max()) + 1.0) * old._cell
        self._ensure_contains(xmin, xmax, ymin, ymax)
        h, w = self._sum.shape

        # r×r sub-points per old cell guarantee every overlapped new cell
        # is hit when refining; r = 1 is the plain aggregation scatter
        # when coarsening. Sub-points carry the old cell's sums verbatim:
        # sum and wsum scale together, so the displayed mean is exact
        # either way, and no transferred cell can fall under the
        # _MIN_WEIGHT display gate.
        r = max(1, int(math.ceil(old._cell / self._cell)))
        offsets = (np.arange(r, dtype=np.float64) + 0.5) / r  # in old cells
        oxg, oyg = np.meshgrid(offsets, offsets)
        oxg, oyg = oxg.ravel(), oyg.ravel()

        # Chunked: refining a large survey can expand to tens of millions
        # of sub-points; bound the temporaries.
        max_points = 1_000_000
        cells_per_batch = max(1, max_points // (r * r))
        for lo in range(0, ix.size, cells_per_batch):
            bix = ix[lo:lo + cells_per_batch]
            biy = iy[lo:lo + cells_per_batch]
            src = (biy, bix)
            x = old._x0 + (bix[:, None] + oxg[None, :]) * old._cell
            y = old._y0 + (biy[:, None] + oyg[None, :]) * old._cell
            cx, cy = self._world_to_cell(x.ravel(), y.ravel())
            ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
            idx = (cy[ok], cx[ok])

            def spread(plane: np.ndarray) -> np.ndarray:
                """Old per-cell values repeated onto the r² sub-points."""
                return np.repeat(plane[src], r * r)[ok]

            np.add.at(self._sum, idx, spread(old._sum))
            np.add.at(self._wsum, idx, spread(old._wsum).astype(np.float32))
            np.add.at(self._count, idx,
                      spread(old._count).astype(np.uint32))
            np.minimum.at(self._closest_rng, idx, spread(old._closest_rng))
            self._closest[idx] = spread(old._closest)
            self._oldest[idx] = spread(old._oldest)
            self._newest[idx] = spread(old._newest)
        self._dirty = True

    #: A cell is displayed once it has gathered at least this much
    #: bilinear weight — suppresses the faint half-cell halo a pure
    #: splat would produce at the outer swath edge.
    _MIN_WEIGHT = 0.25

    def render(self, mode: str = "average", stride: int = 1) -> np.ndarray:
        """Intensity raster for a priority mode (NaN = empty), row 0 = ymin.

        ``stride`` > 1 renders every Nth cell (display decimation for
        very large grids: the raster still covers the full extent, at
        ``stride * cell_size_m`` per rendered pixel, and the cost drops
        quadratically). The grid data is untouched.
        """
        s = max(int(stride), 1)
        if mode == "average":
            wsum = self._wsum[::s, ::s]
            with np.errstate(invalid="ignore", divide="ignore"):
                return np.where(wsum >= self._MIN_WEIGHT,
                                self._sum[::s, ::s] / wsum, np.nan)
        plane = {"closest": self._closest, "oldest": self._oldest,
                 "newest": self._newest}.get(mode)
        if plane is None:
            raise ValueError(f"Unknown priority mode: {mode!r}")
        return np.where(self._count[::s, ::s] > 0, plane[::s, ::s],
                        np.float32(np.nan))

    # ---- persistence ---------------------------------------------------------
    def save(self, log_root: Path, prefix: str = "sonar_mosaic",
             extra: Optional[dict] = None) -> Tuple[Path, Path]:
        """Save as compact .npz (same keys as the legacy listener) + PNG.

        ``extra`` adds keys to the npz (the value domain of the planes and
        the display model that produced it — see core/mosaic_service)."""
        log_root.mkdir(parents=True, exist_ok=True)
        img = self.render()
        npz_path = log_root / f"{prefix}.npz"
        png_path = log_root / f"{prefix}.png"
        np.savez_compressed(
            npz_path,
            **(extra or {}),
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
