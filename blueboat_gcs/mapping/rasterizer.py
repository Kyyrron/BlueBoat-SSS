"""Ping densification for professional-quality mosaicking.

Why the old mosaic looked worse than SonarView (analysis)
---------------------------------------------------------
The previous pipeline scattered each raw sample into exactly one 25 cm
cell and averaged. Professional packages (SonarView, SonarWiz, HYPACK,
Chesapeake) instead treat the waterfall as a *continuous image* draped
onto the seabed: consecutive pings define thin quadrilaterals that are
**rasterized with bilinear interpolation** onto the grid (see P. Blondel,
*The Handbook of Sidescan Sonar*, Springer 2009, ch. 5 "Sonar data
processing"; P. Cervenka & C. de Moustier, "Sidescan sonar image
processing techniques", IEEE J. Oceanic Eng. 18(2), 1993). The visual
consequences of point-scatter vs. quad-rasterization are exactly the
artefacts observed:

* **nearest-cell quantization** → jagged, aliased line work;
* **holes and fans in turns** → the outer swath tip sweeps several cells
  between pings and nothing fills them (SonarView never shows this);
* **averaging dozens of raw samples per coarse cell** → over-smoothed
  texture yet blocky edges: resolution wasted across-track, missing
  along-track.

Chosen strategy (and why not textured quads directly)
-----------------------------------------------------
Full per-quad GPU/CPU texture mapping would couple the renderer to the
scene graph and break the clean "grid in, raster out" architecture. The
mathematically equivalent, architecture-preserving approach is
**supersampled densification + bilinear splatting**:

1. **across-track resampling**: each side's samples are linearly
   resampled to a spacing of ``cell/2`` (Nyquist w.r.t. the grid), so
   every crossed cell receives support — classic gridding practice;
2. **along-track ping interpolation**: between consecutive pings the
   pose is interpolated (position lerp, shortest-arc yaw) and the two
   intensity profiles are cross-faded, generating virtual pings until
   the *swath tip* (not just the hull) moves less than ``cell/2`` —
   this is precisely the interior of the SonarWiz-style quad, evaluated
   bilinearly, and it closes turn fans by construction;
3. **bilinear splatting** into the grid (in ``MosaicGrid``): each dense
   sample deposits into its 4 surrounding cells with bilinear weights —
   the adjoint of bilinear interpolation, i.e. anti-aliased rendering
   (standard resampling theory; cf. gridding in Cervenka & de Moustier
   1993).

Physical consistency: interpolation only ever happens *between two
adjacent real measurements* (one ping interval, one sample interval).
Nothing is invented beyond the sensor's own sampling footprint — unlike
the display-side gap fill, this is resampling, not inpainting. Guards:
if the boat teleports (> ``max_gap_m``) or the heading jumps
(> ``max_yaw_jump_rad``) — dropouts, restarts — no interpolation is
performed across the discontinuity.

Both stages can be disabled (``mosaic.densify: false``) to recover the
legacy point-scatter behaviour for A/B comparison.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from ..models.sonar import SonarPing

_MAX_VIRTUAL_PINGS = 64          # hard cap per ping pair (safety)


@dataclass
class _PrevPing:
    x: float
    y: float
    yaw: float
    y_port: np.ndarray
    v_port: np.ndarray
    y_stbd: np.ndarray
    v_stbd: np.ndarray


class PingRasterizer:
    """Densifies pings across- and along-track before grid accumulation."""

    def __init__(self, cell_size_m: float,
                 max_gap_m: float = 2.5,
                 max_yaw_jump_rad: float = 0.6) -> None:
        self._step = 0.5 * cell_size_m       # Nyquist w.r.t. the grid
        self._max_gap = max_gap_m
        self._max_yaw_jump = max_yaw_jump_rad
        self._prev: Optional[_PrevPing] = None

    def reset(self) -> None:
        """Forget the previous ping (START, Clear SSS data, dropouts)."""
        self._prev = None

    # ---- public ---------------------------------------------------------------
    def rasterize(self, ping: SonarPing
                  ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Ping -> dense world samples (x, y, intensity, |slant range|)."""
        cur = self._resample_sides(ping)
        prev, self._prev = self._prev, cur

        interp_ok = (
            prev is not None
            and math.hypot(cur.x - prev.x, cur.y - prev.y) <= self._max_gap
            and abs(self._ang_diff(cur.yaw, prev.yaw)) <= self._max_yaw_jump)

        if not interp_ok:
            xs, ys, vs, rs = self._project(cur, 1.0, cur)
            return xs, ys, vs, rs

        # Number of virtual pings: densify until the fastest-moving point
        # of the swath (hull translation + rotation lever arm at the tip)
        # advances at most one grid step.
        r_max = max(
            float(cur.y_port[-1]) if cur.y_port.size else 0.0,
            float(-cur.y_stbd[-1]) if cur.y_stbd.size else 0.0)
        hull = math.hypot(cur.x - prev.x, cur.y - prev.y)
        tip = hull + abs(self._ang_diff(cur.yaw, prev.yaw)) * r_max
        n = min(max(int(math.ceil(tip / self._step)), 1), _MAX_VIRTUAL_PINGS)

        xs, ys, vs, rs = [], [], [], []
        for k in range(1, n + 1):
            f = k / n                          # (0, 1], f = 1 -> current ping
            x, y, v, r = self._project(prev, f, cur)
            xs.append(x); ys.append(y); vs.append(v); rs.append(r)
        return (np.concatenate(xs), np.concatenate(ys),
                np.concatenate(vs), np.concatenate(rs))

    # ---- internals -------------------------------------------------------------
    def _resample_sides(self, ping: SonarPing) -> _PrevPing:
        """Split port/starboard and resample each to grid-step spacing.

        The two sides are handled independently: interpolating across the
        nadir gap would invent data where the sonar genuinely has none.
        """
        y, v = ping.y_local, ping.intensity_db

        def side(mask, flip: bool):
            ys, vs = y[mask], v[mask]
            if ys.size < 2:
                return ys.astype(np.float64), vs.astype(np.float32)
            order = np.argsort(ys)
            ys, vs = ys[order], vs[order]
            lo, hi = float(ys[0]), float(ys[-1])
            n = max(int(math.ceil((hi - lo) / self._step)) + 1, 2)
            yg = np.linspace(lo, hi, n)
            vg = np.interp(yg, ys, vs).astype(np.float32)
            return yg, vg

        y_p, v_p = side(y > 0, False)
        y_s, v_s = side(y < 0, True)
        return _PrevPing(ping.robot_x, ping.robot_y, ping.yaw,
                         y_p, v_p, y_s, v_s)

    def _project(self, prev: _PrevPing, f: float, cur: _PrevPing
                 ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Virtual ping at fraction ``f`` between prev (0) and cur (1)."""
        x = prev.x + f * (cur.x - prev.x)
        y = prev.y + f * (cur.y - prev.y)
        yaw = prev.yaw + f * self._ang_diff(cur.yaw, prev.yaw)
        sin_yaw, cos_yaw = math.sin(yaw), math.cos(yaw)

        xs, ys, vs, rs = [], [], [], []
        for yg_cur, vg_cur, yg_prev, vg_prev in (
                (cur.y_port, cur.v_port, prev.y_port, prev.v_port),
                (cur.y_stbd, cur.v_stbd, prev.y_stbd, prev.v_stbd)):
            if yg_cur.size == 0:
                continue
            if f >= 1.0 or yg_prev.size < 2:
                vg = vg_cur
            else:
                # Cross-fade the two real profiles on the current lateral
                # grid — the bilinear interior of the ping-pair quad.
                v_prev_on_cur = np.interp(yg_cur, yg_prev, vg_prev)
                vg = ((1.0 - f) * v_prev_on_cur + f * vg_cur).astype(
                    np.float32)
            xs.append(x - sin_yaw * yg_cur)
            ys.append(y + cos_yaw * yg_cur)
            vs.append(vg)
            rs.append(np.abs(yg_cur))
        return (np.concatenate(xs), np.concatenate(ys),
                np.concatenate(vs).astype(np.float32),
                np.concatenate(rs))

    @staticmethod
    def _ang_diff(a: float, b: float) -> float:
        """Shortest signed angle a - b."""
        return math.atan2(math.sin(a - b), math.cos(a - b))
