"""Optional small-gap interpolation between consecutive sonar lines.

Requirements from the specification, and how each is met:

* *Only interpolate between nearby consecutive sonar lines* — a cell is a
  candidate only if it lies within ``max_gap_m`` of measured cells
  (distance transform on the empty mask). Along-track gaps between
  consecutive pings at survey speed are a few cells wide; large unmapped
  regions are farther than the threshold from data and stay empty.
* *Visually realistic* — filled values are a normalized-convolution
  (distance-weighted local mean) of the surrounding measured cells, i.e.
  plausible local texture, no invented structure.
* *Never overwrite raw data / reversible* — this module reads
  (mean, count) and returns a **new** array; ``MosaicGrid`` is untouched
  and the saved ``.npz`` always contains raw data only. Toggling the
  checkbox simply re-renders.

Whether AI should consume interpolated images is discussed in
docs/HANDOVER.md (recommendation: raw only).
"""

from __future__ import annotations

import numpy as np
import cv2


def fill_small_gaps(mean: np.ndarray, count: np.ndarray,
                    cell_size_m: float, max_gap_m: float,
                    min_neighbors: int = 3) -> tuple[np.ndarray, np.ndarray]:
    """Fill empty cells close to data with a local mean of valid cells.

    Parameters
    ----------
    mean:
        Raster from ``MosaicGrid.render()`` (NaN where empty).
    count:
        Per-cell sample count (0 where empty).
    cell_size_m / max_gap_m:
        Fill only empty cells whose distance to the nearest measured cell
        is <= max_gap_m.
    min_neighbors:
        Minimum number of measured cells inside the averaging kernel for
        a fill to be accepted (rejects lonely speckles at swath edges).

    Returns
    -------
    (filled_mean, filled_mask):
        ``filled_mean`` is a copy of ``mean`` with accepted gap cells
        replaced by interpolated values; ``filled_mask`` is True exactly
        on those cells (used by the renderer / future QA overlays).
    """
    valid = count > 0
    if not valid.any():
        return mean.copy(), np.zeros_like(valid)

    max_gap_cells = max(1, int(round(max_gap_m / cell_size_m)))

    # Distance (in cells) from every empty cell to the nearest valid cell.
    # cv2.distanceTransform measures distance to the nearest zero pixel,
    # so feed it the inverted validity mask.
    dist = cv2.distanceTransform((~valid).astype(np.uint8), cv2.DIST_L2, 3)
    candidates = (~valid) & (dist <= max_gap_cells)
    if not candidates.any():
        return mean.copy(), np.zeros_like(valid)

    # Normalized box convolution: sum(values) / sum(weights) over a kernel
    # slightly larger than the gap radius, computed only from valid cells.
    k = 2 * max_gap_cells + 1
    values = np.where(valid, np.nan_to_num(mean, nan=0.0), 0.0).astype(np.float32)
    weights = valid.astype(np.float32)
    sum_v = cv2.boxFilter(values, ddepth=-1, ksize=(k, k), normalize=False)
    sum_w = cv2.boxFilter(weights, ddepth=-1, ksize=(k, k), normalize=False)

    with np.errstate(invalid="ignore", divide="ignore"):
        local_mean = sum_v / sum_w

    accept = candidates & (sum_w >= float(min_neighbors))
    filled = mean.copy()
    filled[accept] = local_mean[accept]
    return filled, accept
