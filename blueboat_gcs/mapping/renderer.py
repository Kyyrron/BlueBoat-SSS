"""Mosaic raster -> displayable RGBA image.

Design decisions
----------------
* The renderer runs on a GUI-side QTimer at ``mosaic.render_hz`` (default
  4 Hz), fully decoupled from the ~28 Hz ping rate: ingestion is a cheap
  numpy scatter-add per ping, drawing is a bounded-rate raster refresh.
  This is what removes the flicker/slowdown of the matplotlib listener.
* Contrast stretching keeps the field-proven 2–98 percentile scheme of
  the old listener, but percentiles are computed on a subsample of valid
  cells so the cost stays flat as the survey grows.
* Empty cells are fully transparent (alpha 0) so the satellite layer
  shows through — the SSS mosaic always remains the top data layer.
* The colormap is a numpy implementation of matplotlib's "copper" so the
  application does not depend on matplotlib.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Dict, Optional, Tuple

import numpy as np
from PySide6.QtGui import QImage

_LUTS: Dict[str, np.ndarray] = {}


def _build_luts() -> Dict[str, np.ndarray]:
    """256x3 uint8 colormaps expected by SSS operators (no matplotlib)."""
    x = np.linspace(0.0, 1.0, 256)
    copper = np.stack([np.clip(1.25 * x, 0, 1), 0.7812 * x, 0.4975 * x],
                      axis=1)
    gray = np.stack([x, x, x], axis=1)
    # "Gold": warm high-contrast palette common in commercial SSS software.
    gold = np.stack([np.clip(1.6 * x, 0, 1),
                     np.clip(1.15 * x - 0.05, 0, 1),
                     np.clip(0.9 * x - 0.35, 0, 1)], axis=1)
    return {
        "Copper": (copper * 255).astype(np.uint8),
        "Grayscale": (gray * 255).astype(np.uint8),
        "Inverse gray": (gray[::-1] * 255).astype(np.uint8),
        "Gold": (gold * 255).astype(np.uint8),
    }


def lut(name: str) -> np.ndarray:
    global _LUTS
    if not _LUTS:
        _LUTS = _build_luts()
    return _LUTS.get(name, _LUTS["Copper"])


def lut_names() -> Tuple[str, ...]:
    lut("Copper")  # ensure the cache is populated
    return tuple(_LUTS.keys())


def copper_lut() -> np.ndarray:
    """Backward-compatible accessor (used by MosaicGrid.save)."""
    return lut("Copper")


@dataclass(frozen=True)
class DisplaySettings:
    """Operator-adjustable intensity mapping — **visualization only**.

    Mirrors the controls SSS operators expect from SonarView:

    * ``auto_range``      — dynamic range from percentiles of the data
      (the field-proven 2–98 % scheme) vs. the manual window below;
    * ``vmin_db/vmax_db`` — manual dynamic-range window [dB];
    * ``gamma``           — contrast curve (1 = linear, <1 brightens
      mid-tones, >1 darkens them);
    * ``brightness``      — post-gamma offset in [-0.5, +0.5];
    * ``colormap``        — one of :func:`lut_names`.

    Raw grids/buffers are never modified; this maps values to pixels.
    """

    auto_range: bool = True
    vmin_db: float = -60.0
    vmax_db: float = -10.0
    gamma: float = 1.0
    brightness: float = 0.0
    colormap: str = "Copper"

    def with_(self, **kw) -> "DisplaySettings":
        return replace(self, **kw)


class MosaicRenderer:
    """Intensity raster -> RGBA8888 QImage, honouring DisplaySettings.

    Shared by the mosaic path and the waterfall path so both views react
    identically to the display controls.
    """

    def __init__(self, percentiles: Tuple[float, float] = (2.0, 98.0),
                 max_percentile_samples: int = 200_000) -> None:
        self._p = percentiles
        self._max_samples = max_percentile_samples
        self.settings = DisplaySettings()

    def _auto_limits(self, img: np.ndarray) -> Tuple[float, float]:
        valid = img[np.isfinite(img)]
        if valid.size == 0:
            return 0.0, 1.0
        if valid.size > self._max_samples:  # keep percentile cost bounded
            step = valid.size // self._max_samples
            valid = valid[::step]
        vmin, vmax = np.percentile(valid, self._p)
        if vmax - vmin < 1e-6:
            vmax = vmin + 1e-6
        return float(vmin), float(vmax)

    def to_rgba(self, values: np.ndarray) -> np.ndarray:
        """(H, W) float raster (NaN = empty) -> (H, W, 4) uint8 RGBA."""
        s = self.settings
        if s.auto_range:
            vmin, vmax = self._auto_limits(values)
        else:
            vmin, vmax = s.vmin_db, max(s.vmax_db, s.vmin_db + 1e-6)
        finite = np.isfinite(values)
        norm = np.zeros_like(values, dtype=np.float32)
        norm[finite] = np.clip((values[finite] - vmin) / (vmax - vmin),
                               0.0, 1.0)
        if s.gamma != 1.0:
            norm[finite] = norm[finite] ** np.float32(s.gamma)
        if s.brightness != 0.0:
            norm[finite] = np.clip(norm[finite] + np.float32(s.brightness),
                                   0.0, 1.0)
        idx = (norm * 255).astype(np.uint8)
        rgba = np.zeros((*values.shape, 4), dtype=np.uint8)
        rgba[..., :3] = lut(s.colormap)[idx]
        rgba[..., 3] = np.where(finite, 255, 0)
        return rgba

    def to_qimage(self, values: np.ndarray, flip: bool = True) -> QImage:
        """Raster (row 0 = ymin, NaN = empty) -> QImage (row 0 = ymax).

        ``flip=False`` keeps row order (waterfall: row = ping index).
        """
        rgba = self.to_rgba(values)
        if flip:
            # World row 0 is ymin ('origin=lower'); QImage row 0 is drawn
            # at the top and the map view maps scene-y-down to world-y-up.
            rgba = np.flipud(rgba)
        rgba = np.ascontiguousarray(rgba)
        h, w = rgba.shape[:2]
        img = QImage(rgba.data, w, h, 4 * w, QImage.Format_RGBA8888)
        return img.copy()  # detach from the numpy buffer lifetime
