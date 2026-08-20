#!/usr/bin/env python3
"""Pure-math helpers and trackers for side scan sonar post-processing.
"""

from __future__ import annotations

import os
import math
from pathlib import Path
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from typing import Deque, List, Optional, Sequence, Tuple

def project_to_world(
    robot_x: float, robot_y: float, yaw: float,
    y_local: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Rotate a ping's lateral samples into world coordinates.

    The ping is purely lateral in the boat frame: each sample sits at
    (x_body=0, y_body=y_local[i]). REP-103 conventions:
      * +y_body = port  -> processor publishes positive `port_y[i]`
      * -y_body = stbd  -> processor publishes negative `starboard_y[i]`

    The 2D rotation by yaw of (0, y_body) into world frame is:
        x_w = -sin(yaw) * y_body
        y_w =  cos(yaw) * y_body
    """
    x_world = robot_x - math.sin(yaw) * y_local
    y_world = robot_y + math.cos(yaw) * y_local
    return x_world, y_world


def scale_to_db(
    pwr_u16: Sequence[int], min_pwr_db: float, max_pwr_db: float
) -> List[float]:
    """Convert raw u16 power samples to dB.

    Formula (Cerulean Omniscan 450 template):
        db = min_pwr_db + (raw / 65535) * (max_pwr_db - min_pwr_db)

    See: github.com/bluerobotics/ping-python/blob/master/generate/templates/omniscan450.py.in
    """
    span = max_pwr_db - min_pwr_db
    return [min_pwr_db + (s / 65535.0) * span for s in pwr_u16]

def find_noise_window_start(
    pwr_db,
    search_max: int = 60,
    drop_db: float = 10.0,
    persistence: int = 5,
    fallback: int = 30,
) -> int:
    """Locate the first sample past the transmit-pulse ringing.

    The transmit pulse and its ringing tail occupy the first samples of
    every ping with intensity well above the post-ringing water-column
    noise. This function returns the index at which ringing has decayed
    enough to safely estimate the noise floor.

    Algorithm
    ---------
    1. peak = max(pwr_db[:search_max])     # ringing peak (near sample 0-1)
    2. target = peak - drop_db
    3. Return the first index i in [0, search_max - persistence] such that
       pwr_db[i : i + persistence] are all strictly below target.
    4. If no such index is found, return `fallback` (defensive default;
       see note below).

    Why this adapts to the environment
    ----------------------------------
    The algorithm only uses values relative to the ping's own ringing peak.
    A pool with long ringing tails, an open-water run with short ringing,
    or a configuration with different hardware gain all produce a different
    `target` and a different settle index — derived from the data, not
    from any tuned constant.

    Parameters
    ----------
    pwr_db : sequence of float
        Per-sample intensity in dB (already scaled from u16 via scale_to_db).
    search_max : int
        How many samples from the start to consider. Must be smaller than
        the expected sample index of the shallowest plausible FBR — for
        example, 60 samples × 25 mm/sample = 1.5 m of slant range. For
        deployments where altitude could be < 1.5 m the value can be lowered.
    drop_db : float
        How far below the ringing peak we consider "settled". 10 dB is the
        empirical value from the pool run and matches the literature
        (pulse-to-noise contrast is typically 15-25 dB).
    persistence : int
        How many consecutive samples must be below target to count as
        settled. Guards against single-sample dips inside the ringing tail.
    fallback : int
        Returned when no settle is found in [0, search_max). 30 ≈ 0.75 m
        for the Omniscan's typical 25 mm sample spacing — past any sensible
        ringing tail, before any sensible bottom return.

    Returns
    -------
    int : index where the noise window should start.
    """
    n = len(pwr_db)
    if n < search_max + persistence:
        return fallback
    peak = max(pwr_db[:search_max])
    target = peak - drop_db
    for i in range(search_max - persistence + 1):
        if all(pwr_db[i + k] < target for k in range(persistence)):
            return i
    return fallback

def detect_fbr_slant_m(
    pwr_db, start_mm, length_mm, num_results,
    *, noise_floor_window, threshold_delta_db, persistence,
    ringing_search_max: int = 60,
    ringing_drop_db: float = 10.0,
    ringing_persistence: int = 5,
):
    nw_start = find_noise_window_start(
        pwr_db,
        search_max=ringing_search_max,
        drop_db=ringing_drop_db,
        persistence=ringing_persistence,
    )
    nw_end = nw_start + noise_floor_window
    if len(pwr_db) < nw_end + persistence:
        return None
    noise = sum(pwr_db[nw_start:nw_end]) / noise_floor_window
    threshold = noise + threshold_delta_db
    for i in range(nw_end, len(pwr_db) - persistence + 1):
        if pwr_db[i] <= threshold:
            continue
        if all(pwr_db[i + k] > threshold for k in range(persistence)):
            # existing slant-range conversion — keep whatever you have here:
            slant_mm = start_mm + (i / max(num_results - 1, 1)) * length_mm
            return slant_mm / 1000.0
    return None


def project_side(
    pwr_db: Sequence[float],
    start_mm: int,
    length_mm: int,
    num_results: int,
    altitude_m: float,
    transducer_y_offset_m: float,
    side_sign: float,
) -> Tuple[List[float], List[float]]:
    """Slant-range-correct one side, drop water-column samples.

    Returns (y_in_base_link, intensities_db); sample i of one list pairs
    with sample i of the other. `side_sign` = +1 for port, -1 for starboard.

    Assumes the seabed is locally flat under each ping (standard SSS
    practice with single-beam data).
    """
    y_out: List[float] = []
    db_out: List[float] = []
    start_m = start_mm / 1000.0
    length_m = length_mm / 1000.0
    denom = max(num_results - 1, 1)
    for i in range(len(pwr_db)):
        slant = start_m + (i / denom) * length_m
        if slant <= altitude_m:
            continue  # water column
        ground = math.sqrt(slant * slant - altitude_m * altitude_m)
        y_out.append(float(side_sign * (transducer_y_offset_m + ground)))
        db_out.append(float(pwr_db[i]))
    return y_out, db_out


class _SideTracker:
    """Single-side altitude tracker bootstrapped from temporal self-consistency.

    A side is considered locked once `bootstrap_pings` of its most recent
    detections all fall within `agreement_tol_m` of each other (a stable,
    plausible seabed return) -- it does NOT need to agree with the other
    side. This makes the system robust to one transducer (e.g. a low-gain
    channel) returning noise: the good side carries the estimate alone.

    After lock, each new detection within `outlier_tol_m` of the current
    altitude updates it; detections further away are rejected as outliers
    (a fish, a rock edge, a noise spike) and the last value is held. A
    long run of rejects (longer than `relock_after`) forces a re-bootstrap,
    so the tracker recovers if the seabed genuinely steps.
    """

    def __init__(self, bootstrap_pings: int, agreement_tol_m: float,
                 outlier_tol_m: float, relock_after: int) -> None:
        self._bootstrap_pings = bootstrap_pings
        self._agreement_tol_m = agreement_tol_m
        self._outlier_tol_m = outlier_tol_m
        self._relock_after = relock_after
        self._window: Deque[float] = deque(maxlen=bootstrap_pings)
        self._altitude: Optional[float] = None
        self._reject_streak = 0
        self._miss_streak = 0

    @property
    def altitude(self) -> Optional[float]:
        return self._altitude

    @property
    def locked(self) -> bool:
        return self._altitude is not None

    def update(self, fbr: Optional[float]) -> Optional[float]:
        if self._altitude is None:
            # --- bootstrap phase ---
            # An occasional missed detection (None) shouldn't wipe progress;
            # we just don't add to the window. Because the window is a
            # fixed-size sliding deque, stale early values age out on their
            # own, so a slowly-drifting seabed still locks once the most
            # recent `bootstrap_pings` detections are mutually consistent.
            if fbr is None:
                self._miss_streak += 1
                # Only a long blackout (no bottom at all) clears progress.
                if self._miss_streak >= self._relock_after:
                    self._window.clear()
                    self._miss_streak = 0
                return None
            self._miss_streak = 0
            self._window.append(fbr)
            if len(self._window) == self._bootstrap_pings:
                if (max(self._window) - min(self._window)) <= self._agreement_tol_m:
                    self._altitude = sum(self._window) / len(self._window)
            return self._altitude

        # --- operational phase ---
        if fbr is None:
            self._reject_streak += 1
        elif abs(fbr - self._altitude) <= self._outlier_tol_m:
            self._altitude = fbr
            self._reject_streak = 0
        else:
            self._reject_streak += 1

        if self._reject_streak >= self._relock_after:
            # Lost the bottom -- drop the lock and re-bootstrap.
            self._altitude = None
            self._window.clear()
            self._reject_streak = 0
            self._miss_streak = 0
        return self._altitude


class FBRTracker:
    """Dual-side altitude estimator: each side bootstraps independently.

    Per-ping inputs are the FBR slant ranges from each side. Each side runs
    its own `_SideTracker`; the fused altitude is the max() of whichever
    sides are currently locked (max so the seabed is never under-estimated,
    which would push samples into the water column and create false holes).

    The critical property vs the old design: bootstrap no longer requires
    the two sides to agree with each other. If one transducer is low-gain
    and returns noise, the other side bootstraps and carries the estimate
    by itself. The system produces an altitude as soon as EITHER side is
    self-consistent for `bootstrap_pings` detections.
    """

    def __init__(self, bootstrap_pings: int, agreement_tol_m: float,
                 outlier_tol_m: float = 1.0, relock_after: int = 15) -> None:
        self._port = _SideTracker(bootstrap_pings, agreement_tol_m,
                                  outlier_tol_m, relock_after)
        self._stbd = _SideTracker(bootstrap_pings, agreement_tol_m,
                                  outlier_tol_m, relock_after)
        self._altitude: Optional[float] = None

    @property
    def is_bootstrapped(self) -> bool:
        return self._altitude is not None

    @property
    def altitude(self) -> Optional[float]:
        return self._altitude

    def update(
        self, port_alt: Optional[float], stbd_alt: Optional[float]
    ) -> Optional[float]:
        p = self._port.update(port_alt)
        s = self._stbd.update(stbd_alt)

        if p is not None and s is not None:
            self._altitude = max(p, s)
        elif p is not None:
            self._altitude = p
        elif s is not None:
            self._altitude = s
        # else: neither side locked -- keep last altitude (may be None during
        # initial bootstrap, or a held value if both sides transiently lost
        # the bottom before re-locking).
        return self._altitude


class MosaicGrid:
    """Auto-growing 2D running-mean raster of sonar intensity.

    World extent is anchored at (0, 0) and grows in `chunk` increments as
    samples land outside the current bounds. Each cell stores (sum, count)
    so the displayed image is the mean intensity per cell.
    """

    def __init__(self, cell_size_m: float = 0.25,
                 initial_half_extent_m: float = 50.0, log_root: Path = Path(os.path.expanduser("../../../data/SSS_data"))) -> None:
        self._cell = cell_size_m
        n = int(math.ceil(2 * initial_half_extent_m / cell_size_m))
        self._sum:   np.ndarray = np.zeros((n, n), dtype=np.float64)
        self._count: np.ndarray = np.zeros((n, n), dtype=np.uint32)
        # World coordinates of the lower-left corner of cell [0, 0].
        self._x0: float = -initial_half_extent_m
        self._y0: float = -initial_half_extent_m
        self._chunk = int(math.ceil(50.0 / cell_size_m))  # grow by 50 m

        self.log_root = log_root

    @property
    def shape(self) -> tuple[int, int]:
        return self._sum.shape

    @property
    def extent(self) -> tuple[float, float, float, float]:
        h, w = self._sum.shape
        return (self._x0, self._x0 + w * self._cell,
                self._y0, self._y0 + h * self._cell)

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
            pad_left = int(math.ceil((self._x0 - xmin) / self._cell))
            pad_left = max(pad_left, self._chunk)
        if xmax >= self._x0 + w * self._cell:
            pad_right = int(math.ceil(
                (xmax - (self._x0 + w * self._cell)) / self._cell)) + 1
            pad_right = max(pad_right, self._chunk)
        if ymin < self._y0:
            pad_bot = int(math.ceil((self._y0 - ymin) / self._cell))
            pad_bot = max(pad_bot, self._chunk)
        if ymax >= self._y0 + h * self._cell:
            pad_top = int(math.ceil(
                (ymax - (self._y0 + h * self._cell)) / self._cell)) + 1
            pad_top = max(pad_top, self._chunk)
        if pad_left or pad_right or pad_bot or pad_top:
            self._sum = np.pad(
                self._sum, ((pad_bot, pad_top), (pad_left, pad_right))
            )
            self._count = np.pad(
                self._count, ((pad_bot, pad_top), (pad_left, pad_right))
            )
            self._x0 -= pad_left * self._cell
            self._y0 -= pad_bot * self._cell

    def add_samples(self, x: np.ndarray, y: np.ndarray,
                    intensity: np.ndarray) -> None:
        if x.size == 0:
            return
        self._ensure_contains(float(x.min()), float(x.max()),
                              float(y.min()), float(y.max()))
        cx, cy = self._world_to_cell(x, y)
        h, w = self._sum.shape
        ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        # `np.add.at` does unbuffered scatter-add so duplicate (cx,cy)
        # indices accumulate properly.
        np.add.at(self._sum,   (cy[ok], cx[ok]), intensity[ok])
        np.add.at(self._count, (cy[ok], cx[ok]), 1)

    def render(self) -> np.ndarray:
        """Return the mean-intensity raster (NaN where no samples)."""
        with np.errstate(invalid="ignore", divide="ignore"):
            img = np.where(self._count > 0, self._sum / self._count, np.nan)
        return img

    def save(self, prefix: str | Path) -> tuple[Path, Path]:
        """Save raster as compact .npz and quick-look .png."""
        prefix = Path(prefix)
        img = self.render()
        npz_path = self.log_root / prefix.with_suffix(".npz")
        png_path = self.log_root / prefix.with_suffix(".png")
        np.savez_compressed(
            npz_path,
            mean_intensity=img.astype(np.float32),
            count=self._count,
            cell_size_m=self._cell,
            x0=self._x0,
            y0=self._y0,
        )
        # Quick-look PNG with sensible percentile contrast.
        valid = img[np.isfinite(img)]
        if valid.size > 0:
            vmin, vmax = np.percentile(valid, [2, 98])
        else:
            vmin, vmax = 0.0, 1.0
        plt.imsave(png_path, img,
                   origin="lower", cmap="copper", vmin=vmin, vmax=vmax)
        return npz_path, png_path