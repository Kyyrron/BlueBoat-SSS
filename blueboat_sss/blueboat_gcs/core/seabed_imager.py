"""Seabed imaging for AI: waterfall-domain pictures + georeferencing.

One implementation serves both consumers:

* **live** — fed ping-by-ping from the signal bus in the main window;
  every ``stride`` pings it emits a ``SeabedImage`` covering the last
  ``rows`` pings and (while a recording session is active) saves it under
  ``<session>/seabed_images/`` with metadata in an inner ``metadata/``
  folder; each image is passed to the analyzer (dummy for now) and the
  result is published (see main_window / ros_manager);
* **from a log** — ``generate_from_pings`` runs the identical code over a
  decoded mission (replay window's "Save pictures from the log").

Domain: waterfall only — matches the SSS detection literature
(Sethuraman et al. 2024 IJRR; CMRE MCM work), avoids renderer artifacts
(densification/blending are survey-geometry-correlated), and loses no
georeferencing because pixel→world is exact per ping row (see metadata).

The pictures are **raw waterfall** in the native slant-bin domain
(SonarView's convention): one column per device range bin, values
verbatim, no resampling and no interpolation — hole-free by
construction. The dark centre band is the physical water column (bins
below the tracked altitude, cut upstream), whose width follows from the
per-ping geometry. Image width adapts to the acquisition; a range
change ends the current window (a training image must be one
homogeneous grid) and the next window adopts the new layout.

**The AI feed is these pictures + their JSON metadata.** The companion
``_world.npz`` (per-pixel world grids + float dB) is an auxiliary
georeferencing/analysis record, not the training input.

Windowing: ``rows`` = 256, ``stride`` = 128 (50 % overlap) by default.
At 28 Hz / 0.8 m s⁻¹ (≈ 2.9 cm per row) 256 rows span ≈ 7.3 m
along-track against a 15–36 m swath — a near-square ground footprint —
and a 50 % overlap is the standard sliding-window tiling guarantee that
any object smaller than the stride appears *entirely* inside at least
one image (tiled-inference practice, e.g. SAHI, Akyön et al. 2022);
expected targets (0.5–2 m ≈ 17–70 rows) are far below the 128-row
stride. Both values are configurable (``config seabed`` block).

Artifacts per image ``seabed_{id:05d}``:

* ``seabed_XXXXX.png``          — 8-bit grayscale through the window's ONE
  display model (``core/display_model``): two-way transmission loss
  removed, the mission's seabed curve in normalised slant range divided
  out, then the power-law transfer — the same mapping the live waterfall
  and the mosaic use, one model for every tile of a mission (a detector
  needs the same seabed to read the same everywhere). Nothing is erased:
  the water column and the ringing core are kept and darken by physics.
  The model is recorded (JSON + ``_world.npz``) so the picture is
  losslessly invertible back to dB;
* ``metadata/seabed_XXXXX.json``      — the pixel→world contract:
  per-row pose/time/speed/altitude/source ping, the display model, the
  closed-form pixel→world formula, boat summary;
* ``metadata/seabed_XXXXX_world.npz`` — precomputed per-pixel
  ``world_x``/``world_y`` float32 grids (H×W) + the raw float dB image
  (``intensity_db``) + the model curves — a YOLO bbox center maps to the
  world with one array lookup, and the raw radiometry is preserved.

Row geometry (``seabed.row_geometry``, decision 2026-09-05, PROVISIONAL):

* ``"square"`` (default) — a picture row is one across-track bin pitch of
  along-track distance, so a pixel is the same size along- and across-
  track and an object keeps its shape at any boat speed (the "speed
  correction" of the sidescan literature; SonarView draws true scale).
  Each row copies exactly one ping's bins verbatim (nearest ping to the
  row's along-track position; ``ping_index`` per row in the JSON); when
  the boat outruns the pitch some pings are not represented in the
  *picture* (they stay in ``waterfall_raw.npz``), when it is slower a
  ping fills several rows. Row poses are interpolated along-track so the
  world grids stay continuous. ``rows``/``stride`` count picture rows.
* ``"ping"`` — one row per ping, the previous contract. **This is the
  revert switch** if detector results are worse with square pixels.

Pixel convention (identical to the live waterfall view): row 0 = newest,
last row = oldest; column 0 = far range **port**, last column = far
range starboard, transducer at the centre seam; every column is one
slant bin of ``bin_pitch_m`` metres. Pixel→world:

    k(j)         = (W/2 - 1 - j) if j < W/2 else (j - W/2)
    s(j)         = (k + 0.5) * bin_pitch_m
    y_local(i,j) = ±sqrt(max(s² - depth_i², 0))   (+ port, − starboard)
    world_x(i,j) = x_i - sin(yaw_i) * y_local
    world_y(i,j) = y_i + cos(yaw_i) * y_local
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import (Callable, Iterable, List, Optional, Sequence, Tuple)

import cv2
import numpy as np
from PySide6.QtCore import QObject, QRunnable, QThreadPool, Signal

from ..config.settings import AppConfig
from ..models.sonar import (SonarPing, build_slant_row, native_bins,
                            slant_col_to_y_local)
from .display_model import DisplayModel, DisplayModelSnapshot

SCHEMA_VERSION = 4


def waterfall_pixel_to_world(x, y, yaw, depth, pitch_m, col, width: int):
    """Waterfall pixel -> world position (the documented formula).

    Native slant-bin layout: the column's slant range is
    ``(k + 0.5) · pitch`` about the centre seam, the row's altitude
    turns it into ``y_local = ±sqrt(max(s² − depth², 0))`` (+ port,
    − starboard), then rotate/translate by the row pose:
    ``wx = x − sin(yaw)·y_local``, ``wy = y + cos(yaw)·y_local``.

    Accepts scalars (a waterfall click) or broadcasting numpy arrays
    (the per-pixel world grids of a whole seabed image). This is the
    single definition; the waterfall service's detection overlay applies
    its exact inverse.
    """
    y_local = slant_col_to_y_local(col, width, pitch_m, depth)
    return x - np.sin(yaw) * y_local, y + np.cos(yaw) * y_local


# ---------------------------------------------------------------------------
# Product
# ---------------------------------------------------------------------------
@dataclass
class SeabedImage:
    """One AI-ready seabed picture + its georeferencing."""

    image_id: int
    intensity_db: np.ndarray            # (H, W) float32, NaN = no sample
    world_x: np.ndarray                 # (H, W) float32
    world_y: np.ndarray                 # (H, W) float32
    row_t: np.ndarray                   # (H,) time of each ping [s]
    row_pose: np.ndarray                # (H, 3) x, y, yaw per row
    row_range: np.ndarray               # (H,) swath half-range [m]
    row_speed: np.ndarray               # (H,) boat speed estimate [m/s]
    row_altitude: np.ndarray            # (H,) correction altitude [m] (world grid)
    bin_pitch_m: float = 0.0            # slant metres per column
    detections: List[dict] = field(default_factory=list)
    #: Tracked bottom per row [m] — the ``h`` of the display model.
    row_bottom: Optional[np.ndarray] = None
    #: Source ping of each picture row (index within the imager's window,
    #: newest-first) and its along-track position [m] from the oldest
    #: ping of the window; identity / cumulative for "ping" geometry.
    row_ping_index: Optional[np.ndarray] = None
    row_along_m: Optional[np.ndarray] = None
    row_geometry: str = "ping"
    #: The mission display model that maps this tile to pixels (frozen
    #: snapshot: safe on the save thread). None -> legacy per-image
    #: 2-98 % stretch of the raw dB (nadir_contrast off).
    display: Optional[DisplayModelSnapshot] = None

    # ---- derived -------------------------------------------------------------
    def pixel_to_world(self, row: int, col: int) -> Tuple[float, float]:
        return float(self.world_x[row, col]), float(self.world_y[row, col])

    def _bottoms(self) -> np.ndarray:
        if self.row_bottom is not None:
            return np.asarray(self.row_bottom, dtype=np.float64)
        return np.asarray(self.row_altitude, dtype=np.float64)

    def normalised(self) -> np.ndarray:
        """The model's normalised level ``e`` (raw dB untouched); the raw
        dB itself when no model applies."""
        if self.display is None or not self.display.ready:
            return self.intensity_db
        return self.display.normalise(self.intensity_db, self.bin_pitch_m,
                                      self._bottoms())

    def to_png8(self) -> np.ndarray:
        if self.display is not None and self.display.ready:
            return self.display.to_png8(self.normalised())
        img = self.intensity_db
        finite = np.isfinite(img)
        if not finite.any():
            return np.zeros(img.shape, np.uint8)
        lo, hi = np.percentile(img[finite], (2.0, 98.0))   # legacy fallback
        hi = max(hi, lo + 1e-6)
        out = np.zeros(img.shape, np.float32)
        out[finite] = np.clip((img[finite] - lo) / (hi - lo), 0, 1)
        return (out * 255).astype(np.uint8)

    def metadata(self) -> dict:
        H, W = self.intensity_db.shape
        n = H
        ping_index = (self.row_ping_index if self.row_ping_index is not None
                      else np.arange(n))
        along = (self.row_along_m if self.row_along_m is not None
                 else np.zeros(n))
        bottoms = self._bottoms()
        return {
            "schema": SCHEMA_VERSION,
            "image_id": self.image_id,
            "rows": H, "cols": W,
            "t_start_s": float(self.row_t[0]),
            "t_end_s": float(self.row_t[-1]),
            "bin_pitch_m": float(self.bin_pitch_m),
            "row_geometry": self.row_geometry,
            "row_geometry_note": (
                "square: each row spans one bin_pitch_m along-track and copies "
                "the nearest ping's bins verbatim (ping_index); its pose is "
                "interpolated along-track. ping: one row per ping. "
                "Revert switch: seabed.row_geometry in config/default.yaml."),
            "display_model": (None if self.display is None or not self.display.ready
                              else self.display.to_json()),
            "display_note": (
                "pixel = round(255*clip(10^(gamma*(e-hi_db)/10),0,1)) with "
                "e = db + TL(r) - A_side(r/h) from display_model (curves also in "
                "the companion _world.npz); h = rows_data[i].bottom_m, r = the "
                "column's slant range. None -> legacy per-image 2-98 % stretch. "
                "NaN (no sample) -> 0. The raw float dB is in the _world.npz."),
            "pixel_convention": {
                "row0": "newest ping", "last_row": "oldest ping",
                "col0": "far range (PORT)",
                "last_col": "far range (STARBOARD)",
                "domain": "native slant bins (raw waterfall, SonarView "
                          "convention); centre seam = transducer; the "
                          "dark centre band is the physical water column, "
                          "kept in the data and darkened by the display "
                          "model (not erased)",
                "formula": ("k = (W/2-1-j) if j < W/2 else (j-W/2); "
                            "s = (k+0.5)*bin_pitch_m; "
                            "y_local(i,j) = sign * sqrt(max(s^2 - "
                            "altitude_m[i]^2, 0)) (+ port, - starboard); "
                            "world = (x_i - sin(yaw_i)*y_local, "
                            "y_i + cos(yaw_i)*y_local); or use the "
                            "precomputed world_x/world_y grids in the "
                            "companion _world.npz"),
            },
            "rows_data": [
                {"t_s": float(self.row_t[i]),
                 "x_m": float(self.row_pose[i, 0]),
                 "y_m": float(self.row_pose[i, 1]),
                 "yaw_rad": float(self.row_pose[i, 2]),
                 "range_m": float(self.row_range[i]),
                 "speed_mps": float(self.row_speed[i]),
                 "altitude_m": float(self.row_altitude[i]),
                 "bottom_m": float(bottoms[i]),
                 "ping_index": int(ping_index[i]),
                 "along_track_m": float(along[i])}
                for i in range(n)
            ],
            "boat": self.boat_summary(),
            "detections": self.detections,
        }

    def boat_summary(self) -> dict:
        return {
            "mean_speed_mps": float(np.mean(self.row_speed)),
            "mean_altitude_m": float(np.mean(self.row_altitude)),
            "start_pose": [float(v) for v in self.row_pose[0]],
            "end_pose": [float(v) for v in self.row_pose[-1]],
        }

    def analysis_json(self, png_path: Optional[str],
                      metadata_path: Optional[str]) -> str:
        """The /sss_ai/seabed_analysis payload: metadata + detections,
        never the pixels (schema documented in HANDOVER)."""
        return json.dumps({
            "schema": SCHEMA_VERSION,
            "image": {
                "image_id": self.image_id,
                "t_start_s": float(self.row_t[0]),
                "t_end_s": float(self.row_t[-1]),
                "rows": int(self.intensity_db.shape[0]),
                "cols": int(self.intensity_db.shape[1]),
                "png_path": png_path,
                "metadata_path": metadata_path,
                "boat": self.boat_summary(),
            },
            "detections": self.detections,
        })

    # ---- persistence ------------------------------------------------------------
    def save(self, out_dir: Path) -> Tuple[Path, Path]:
        """Write PNG + metadata (JSON + world grids npz). Returns paths."""
        out_dir = Path(out_dir)
        meta_dir = out_dir / "metadata"
        meta_dir.mkdir(parents=True, exist_ok=True)
        stem = f"seabed_{self.image_id:05d}"
        png = out_dir / f"{stem}.png"
        cv2.imwrite(str(png), self.to_png8())
        with open(meta_dir / f"{stem}.json", "w", encoding="utf-8") as f:
            json.dump(self.metadata(), f, indent=1)
        extra = ({} if self.display is None or not self.display.ready
                 else self.display.npz_items())
        np.savez_compressed(meta_dir / f"{stem}_world.npz",
                            world_x=self.world_x, world_y=self.world_y,
                            intensity_db=self.intensity_db,
                            row_altitude_m=np.asarray(self.row_altitude, np.float32),
                            row_bottom_m=self._bottoms().astype(np.float32),
                            row_ping_index=(np.asarray(self.row_ping_index, np.int32)
                                            if self.row_ping_index is not None
                                            else np.arange(self.intensity_db.shape[0],
                                                           dtype=np.int32)),
                            **extra)
        return png, meta_dir / f"{stem}.json"


class _SaveJob(QRunnable):
    """Write one seabed image off the GUI thread (the image is never
    mutated after emission, so no copy is needed)."""

    def __init__(self, image: "SeabedImage", out_dir: Path) -> None:
        super().__init__()
        self._image, self._out_dir = image, out_dir
        self.setAutoDelete(True)

    def run(self) -> None:  # noqa: D401
        try:
            self._image.save(self._out_dir)
        except Exception as exc:  # pragma: no cover - reported, never raised
            print(f"seabed image save failed: {exc}")


# ---------------------------------------------------------------------------
# Analyzer (dummy until the model exists)
# ---------------------------------------------------------------------------
def dummy_center_analyzer(image: SeabedImage) -> List[dict]:
    """Placeholder AI: 'detects' the center pixel of every image.

    Replace with the real model here — the contract is: take a
    SeabedImage, return a list of detection dicts with at least
    pixel [row, col], world [x, y], class_name, confidence. Everything
    downstream (topic payload, map markers, dataset alignment) already
    consumes this contract.
    """
    h, w = image.intensity_db.shape
    row, col = h // 2, w // 2
    wx, wy = image.pixel_to_world(row, col)
    return [{"pixel": [row, col], "world": [wx, wy],
             "class_name": "dummy_center", "confidence": 0.5}]


# ---------------------------------------------------------------------------
# Imager
# ---------------------------------------------------------------------------
class SeabedImager(QObject):
    """Sliding-window waterfall imager (live service + offline function)."""

    image_ready = Signal(object)        # SeabedImage (after analysis/save)

    def __init__(self, config: AppConfig,
                 analyzer: Callable[[SeabedImage], List[dict]]
                 = dummy_center_analyzer,
                 model: Optional[DisplayModel] = None) -> None:
        super().__init__()
        self._rows = int(config.seabed.rows)
        self._stride = int(config.seabed.stride)
        geometry = str(getattr(config.seabed, "row_geometry", "ping")).lower()
        if geometry not in ("square", "ping"):
            raise ValueError(f"seabed.row_geometry must be square|ping, got {geometry!r}")
        self._geometry = geometry
        # Native slant-bin layout, adopted from the data (seabed.columns
        # is deprecated: nothing is forced onto a fixed grid any more).
        self._pitch = 0.0
        self._half = 0
        # The window's display model (shared with the waterfall and the
        # mosaic). Owned only when nothing is injected — then this imager
        # feeds it itself; a shared model is fed by the waterfall service
        # (feeding it twice would double-count every row).
        self._nadir_contrast = bool(config.mosaic.nadir_contrast)
        self._model = model if model is not None else DisplayModel(config)
        self._owns_model = model is None
        self._analyzer = analyzer
        self._out_dir: Optional[Path] = None
        self._next_id = 0
        self._since_last = 0                 # pings since the last emission
        self._buf_rows: List[np.ndarray] = []
        #: per buffered ping: (t, x, y, yaw, range, depth, speed, h, along_m)
        self._buf_meta: List[tuple] = []
        self._last_pose: Optional[Tuple[float, float, float]] = None
        self._along = 0.0                    # cumulative along-track [m]
        self._s_emit: Optional[float] = None # along position of the last emission
        self._emitted_any = False

    # ---- live control -----------------------------------------------------------
    @property
    def model(self) -> DisplayModel:
        return self._model

    def set_output_dir(self, out_dir: Optional[Path]) -> None:
        """Images are written to disk only while a directory is set
        (i.e. while a recording session is active)."""
        self._out_dir = Path(out_dir) if out_dir is not None else None

    def reset(self) -> None:
        self._buf_rows.clear()
        self._buf_meta.clear()
        self._since_last = 0
        self._last_pose = None
        self._along = 0.0
        self._s_emit = None
        self._next_id = 0
        self._emitted_any = False
        self._pitch = 0.0
        self._half = 0
        if self._owns_model:
            self._model.reset()

    # ---- ingestion --------------------------------------------------------------
    def on_sonar_ping(self, ping: SonarPing) -> None:
        nb = native_bins(ping)
        if nb is None or nb.extent_bins == 0:
            return
        # A training image must be one homogeneous grid: a change of
        # acquisition (bin pitch or extent) ends the current window and
        # the next one adopts the new layout.
        mismatch = (self._pitch > 0.0
                    and (abs(nb.bin_size_m - self._pitch) > 1e-6 * self._pitch
                         or nb.extent_bins > self._half))
        if mismatch:
            self.break_window()
        if self._pitch == 0.0 or mismatch:
            self._pitch = nb.bin_size_m
            self._half = nb.extent_bins
        row = build_slant_row(nb, self._pitch, self._half)
        h = float(ping.bottom_slant_m)
        if h <= 0.0:
            h = max(float(ping.water_depth), 0.0)
        if self._owns_model:
            self._model.observe_row(row, self._pitch, h)
        speed = 0.0
        if self._last_pose is not None:
            dt = ping.t - self._last_pose[0]
            step = math.hypot(ping.robot_x - self._last_pose[1],
                              ping.robot_y - self._last_pose[2])
            if dt > 1e-3:
                speed = step / dt
            self._along += step
        self._last_pose = (ping.t, ping.robot_x, ping.robot_y)
        self._buf_rows.append(row)
        self._buf_meta.append((ping.t, ping.robot_x, ping.robot_y, ping.yaw,
                               self._half * self._pitch, ping.water_depth,
                               speed, h, self._along))
        self._since_last += 1
        if self._geometry == "ping":
            # Bound memory to one window.
            if len(self._buf_rows) > self._rows:
                self._buf_rows.pop(0)
                self._buf_meta.pop(0)
            if (self._since_last >= self._stride
                    and len(self._buf_rows) >= self._rows):
                self._since_last = 0
                self._emitted_any = True
                self._emit_window(self._rows)
            return
        # Square geometry: windows are measured in along-track metres.
        span = self._rows * self._pitch
        # Bound memory to one window: drop the oldest ping only while the
        # next one still covers a full window (+ a margin for the nearest
        # selection at the far end), so the buffer never falls short of
        # a window between two emissions.
        while (len(self._buf_meta) > 2
               and self._along - self._buf_meta[1][8] >= span + 2 * self._pitch):
            self._buf_rows.pop(0)
            self._buf_meta.pop(0)
        covered = self._along - self._buf_meta[0][8]
        since = (self._along if self._s_emit is None
                 else self._along - self._s_emit)
        if covered >= span and since >= self._stride * self._pitch:
            self._s_emit = self._along
            self._since_last = 0
            self._emitted_any = True
            self._emit_square(self._along, self._rows)

    def break_window(self) -> None:
        """End the current window at a session boundary and start clean.

        A ``.svlog`` may hold several recording sessions. A window that
        straddles the dead time between two of them produces a single image
        whose rows are minutes apart, with a fictitious speed spike where the
        boat "jumped" — a corrupt training sample, and one that looks perfectly
        ordinary in an annotation tool. Flushing the tail and clearing the
        buffer keeps every image inside one session. ``_next_id`` is deliberately
        *not* reset, so image ids stay unique and ordered across the whole log.
        """
        self.flush()
        self._buf_rows.clear()
        self._buf_meta.clear()
        self._since_last = 0
        self._last_pose = None
        self._along = 0.0
        self._s_emit = None

    def flush(self) -> None:
        """Emit the final, possibly truncated picture — no data wasted.

        Everything acquired after the last full window ended (or the
        whole buffer if no full window was ever emitted, e.g. a mission
        shorter than one window) becomes one last image with fewer rows.
        Called on Record OFF / STOP (live) and at the end of every offline
        generation / Run-AI pass.
        """
        if self._geometry == "ping":
            tail = (self._since_last if self._emitted_any
                    else len(self._buf_rows))
            tail = min(tail, len(self._buf_rows))
            if tail >= 2:                       # a 1-row "image" is noise
                self._emit_window(tail)
            self._since_last = 0
            return
        if not self._buf_meta or self._pitch <= 0.0:
            return
        since = (self._along - self._buf_meta[0][8] if self._s_emit is None
                 else self._along - self._s_emit)
        n = min(int(since / self._pitch) + (1 if self._s_emit is None else 0),
                self._rows)
        if n >= 2:
            self._emit_square(self._along, n)
        self._s_emit = self._along
        self._since_last = 0

    # ---- window assembly -----------------------------------------------------------
    def _emit_window(self, n_rows: int) -> None:
        """"ping" geometry: the last ``n_rows`` pings, newest first."""
        rows = self._buf_rows[-n_rows:][::-1]
        meta = self._buf_meta[-n_rows:][::-1]
        n = len(meta)
        t = np.array([m[0] for m in meta], np.float64)
        pose = np.array([[m[1], m[2], m[3]] for m in meta], np.float64)
        rng = np.array([m[4] for m in meta], np.float64)
        depth = np.array([m[5] for m in meta], np.float32)
        speed = np.array([m[6] for m in meta], np.float32)
        bottoms = np.array([m[7] for m in meta], np.float64)
        along = np.array([m[8] for m in meta], np.float64)
        along = along - along.min()
        self._finish(np.vstack(rows), t, pose, rng, depth, speed, bottoms,
                     np.arange(n), along)

    def _emit_square(self, s_top: float, n_rows: int) -> None:
        """"square" geometry: picture row k (0 = newest) sits at along-track
        position ``s_top - k * pitch`` and copies the nearest ping's bins;
        pose and time are interpolated along-track between the two pings
        bracketing it, so the world grids stay continuous."""
        s = np.array([m[8] for m in self._buf_meta], np.float64)
        targets = s_top - np.arange(n_rows) * self._pitch
        targets = np.maximum(targets, s[0])
        hi = np.clip(np.searchsorted(s, targets), 1, len(s) - 1)
        lo = hi - 1
        # nearest ping for the verbatim bins
        nearest = np.where(np.abs(s[hi] - targets) < np.abs(targets - s[lo]),
                           hi, lo)
        arr = np.vstack([self._buf_rows[i] for i in nearest])
        meta = np.array(self._buf_meta, np.float64)      # (n, 9)
        span = np.maximum(s[hi] - s[lo], 1e-9)
        f = np.clip((targets - s[lo]) / span, 0.0, 1.0)
        f = np.where(s[hi] > s[lo], f, 0.0)
        def lerp(col):
            return meta[lo, col] + f * (meta[hi, col] - meta[lo, col])
        t = lerp(0)
        x, y = lerp(1), lerp(2)
        yaw_lo, yaw_hi = meta[lo, 3], meta[hi, 3]
        dyaw = (yaw_hi - yaw_lo + math.pi) % (2 * math.pi) - math.pi
        yaw = yaw_lo + f * dyaw
        pose = np.stack([x, y, yaw], axis=1)
        rng = meta[nearest, 4]
        depth = meta[nearest, 5].astype(np.float32)
        speed = meta[nearest, 6].astype(np.float32)
        bottoms = meta[nearest, 7]
        self._finish(arr, t, pose, rng, depth, speed, bottoms,
                     nearest - int(nearest.min()) if nearest.size else nearest,
                     targets - s[0])

    def _finish(self, arr, t, pose, rng, depth, speed, bottoms, ping_index,
                along) -> None:
        image = self._build(arr, t, pose, rng, depth, speed, bottoms,
                            ping_index, along, self._next_id)
        self._next_id += 1
        image.detections = self._analyzer(image)
        for det in image.detections:
            # Timestamp each detection with ITS pixel row's ping time —
            # this is what lets the waterfall view place the marker on
            # the exact ping line the object was seen on.
            r = int(min(max(det["pixel"][0], 0),
                        image.intensity_db.shape[0] - 1))
            det["t_s"] = float(image.row_t[r])
        png_path = meta_path = None
        if self._out_dir is not None:
            # Paths are deterministic (image id), so publishers get them
            # now; the PNG + npz write (50 ms, 36 ms of it zlib) runs on a
            # pool thread instead of hitching the GUI every 128 pings.
            out_dir = Path(self._out_dir)
            stem = f"seabed_{image.image_id:05d}"
            png_path = str(out_dir / f"{stem}.png")
            meta_path = str(out_dir / "metadata" / f"{stem}.json")
            QThreadPool.globalInstance().start(_SaveJob(image, out_dir))
        image._png_path = png_path              # transported for publishers
        image._metadata_path = meta_path
        self.image_ready.emit(image)

    def _build(self, arr, t, pose, rng, depth, speed, bottoms, ping_index,
               along, image_id: int) -> SeabedImage:
        """Assemble the picture (arrays already newest-first)."""
        W = arr.shape[1]
        # Per-pixel world grids (vectorized over the whole window); each
        # row's altitude turns its slant columns into ground offsets.
        j = np.arange(W, dtype=np.float64)
        wx, wy = waterfall_pixel_to_world(
            pose[:, 0:1], pose[:, 1:2], pose[:, 2:3],
            np.maximum(depth[:, None].astype(np.float64), 0.0),
            self._pitch, j[None, :], W)
        # The model snapshot is immutable: safe on the save thread.
        snap = (self._model.snapshot()
                if self._nadir_contrast and self._model.ready else None)
        return SeabedImage(
            image_id=image_id, intensity_db=np.ascontiguousarray(arr, np.float32),
            world_x=wx.astype(np.float32), world_y=wy.astype(np.float32),
            row_t=np.asarray(t, np.float64), row_pose=np.asarray(pose, np.float64),
            row_range=np.asarray(rng, np.float64),
            row_speed=np.asarray(speed, np.float32),
            row_altitude=np.asarray(depth, np.float32),
            bin_pitch_m=self._pitch,
            row_bottom=np.asarray(bottoms, np.float64),
            row_ping_index=np.asarray(ping_index, np.int32),
            row_along_m=np.asarray(along, np.float64),
            row_geometry=self._geometry,
            display=snap)


# ---------------------------------------------------------------------------
# Offline generation (replay window "Save pictures from the log")
# ---------------------------------------------------------------------------
def feed_pings(imager: SeabedImager, pings: Sequence[SonarPing],
               breaks: Sequence[float] = (),
               progress: Optional[Callable[[float], None]] = None) -> None:
    """Feed a decoded log through an imager, breaking at each session boundary.

    ``breaks`` is mission times at which a session ended — normally
    ``SvlogMission.gap_times``. Every consumer that replays a whole log goes
    through here so the break rule has one implementation: the offline dataset
    export and the replay window's AI pass would otherwise drift apart.
    """
    pending = sorted(float(b) for b in breaks)
    for k, ping in enumerate(pings):
        while pending and ping.t > pending[0]:
            pending.pop(0)
            imager.break_window()
        if progress is not None and k % 200 == 0:
            progress(k / max(len(pings), 1))
        imager.on_sonar_ping(ping)
    imager.flush()                       # truncated tail: no data wasted
    if progress is not None:
        progress(1.0)


def generate_from_pings(pings: Iterable[SonarPing], out_dir: Path,
                        config: AppConfig,
                        progress: Optional[Callable[[float], None]] = None,
                        run_analyzer: bool = False,
                        breaks: Sequence[float] = (),
                        model: Optional[DisplayModel] = None) -> int:
    """Run the identical imaging code over a decoded log; returns the
    number of images written to ``out_dir`` (+ inner ``metadata/``).

    ``model``: the window's fitted display model, so the pictures use the
    very mapping the replay waterfall shows; fitted here in one pass over
    the log when not given (the offline export)."""
    pings = list(pings)
    if model is None and config.mosaic.nadir_contrast:
        model = DisplayModel.fit(config, pings)
    imager = SeabedImager(config, model=model)
    imager.set_output_dir(out_dir)
    if not run_analyzer:
        imager._analyzer = lambda img: []        # dataset mode: raw images
    written = []
    imager.image_ready.connect(lambda img: written.append(img.image_id))
    feed_pings(imager, pings, breaks, progress)
    # Saves run on the pool; an offline export must have its files on
    # disk when it returns (the caller lists / packages them next).
    QThreadPool.globalInstance().waitForDone()
    return len(written)
