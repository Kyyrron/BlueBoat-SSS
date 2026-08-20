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

Windowing: ``rows`` = 256, ``stride`` = 128 (50 % overlap) by default.
At 28 Hz / 0.8 m s⁻¹ (≈ 2.9 cm per row) 256 rows span ≈ 7.3 m
along-track against a 15–36 m swath — a near-square ground footprint —
and a 50 % overlap is the standard sliding-window tiling guarantee that
any object smaller than the stride appears *entirely* inside at least
one image (tiled-inference practice, e.g. SAHI, Akyön et al. 2022);
expected targets (0.5–2 m ≈ 17–70 rows) are far below the 128-row
stride. Both values are configurable (``config seabed`` block).

Artifacts per image ``seabed_{id:05d}``:

* ``seabed_XXXXX.png``          — 8-bit grayscale, per-image 2–98 %
  normalization (annotation-tool friendly);
* ``metadata/seabed_XXXXX.json``      — the pixel→world contract:
  per-row pose/time/speed/altitude, per-row swath half-range, the
  closed-form pixel→world formula, boat summary;
* ``metadata/seabed_XXXXX_world.npz`` — precomputed per-pixel
  ``world_x``/``world_y`` float32 grids (H×W) + the raw float dB image
  (``intensity_db``) — a YOLO bbox center maps to the world with one
  array lookup, and the raw radiometry is preserved for training.

Pixel convention (identical to the live waterfall view): row 0 = oldest
ping, last row = newest; column 0 = +max range **port**, last column =
−max range starboard; each row is scaled to its own swath. Pixel→world:

    y_local(i, j) = r_i * (1 - 2 j / (W-1))
    world_x(i, j) = x_i - sin(yaw_i) * y_local
    world_y(i, j) = y_i + cos(yaw_i) * y_local
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, List, Optional, Tuple

import cv2
import numpy as np
from PySide6.QtCore import QObject, Signal

from ..config.settings import AppConfig
from ..models.sonar import SonarPing

SCHEMA_VERSION = 1


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
    row_altitude: np.ndarray            # (H,) water depth / altitude [m]
    detections: List[dict] = field(default_factory=list)

    # ---- derived -------------------------------------------------------------
    def pixel_to_world(self, row: int, col: int) -> Tuple[float, float]:
        return float(self.world_x[row, col]), float(self.world_y[row, col])

    def to_png8(self) -> np.ndarray:
        img = self.intensity_db
        finite = np.isfinite(img)
        if not finite.any():
            return np.zeros(img.shape, np.uint8)
        lo, hi = np.percentile(img[finite], (2.0, 98.0))
        hi = max(hi, lo + 1e-6)
        out = np.zeros(img.shape, np.float32)
        out[finite] = np.clip((img[finite] - lo) / (hi - lo), 0, 1)
        return (out * 255).astype(np.uint8)

    def metadata(self) -> dict:
        H, W = self.intensity_db.shape
        return {
            "schema": SCHEMA_VERSION,
            "image_id": self.image_id,
            "rows": H, "cols": W,
            "t_start_s": float(self.row_t[0]),
            "t_end_s": float(self.row_t[-1]),
            "pixel_convention": {
                "row0": "oldest ping", "last_row": "newest ping",
                "col0": "+range (PORT)", "last_col": "-range (STARBOARD)",
                "formula": ("y_local(i,j) = row_range[i] * (1 - 2*j/(W-1)); "
                            "world = (x_i - sin(yaw_i)*y_local, "
                            "y_i + cos(yaw_i)*y_local); "
                            "or use the precomputed world_x/world_y grids "
                            "in the companion _world.npz"),
            },
            "rows_data": [
                {"t_s": float(self.row_t[i]),
                 "x_m": float(self.row_pose[i, 0]),
                 "y_m": float(self.row_pose[i, 1]),
                 "yaw_rad": float(self.row_pose[i, 2]),
                 "range_m": float(self.row_range[i]),
                 "speed_mps": float(self.row_speed[i]),
                 "altitude_m": float(self.row_altitude[i])}
                for i in range(H)
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
        np.savez_compressed(meta_dir / f"{stem}_world.npz",
                            world_x=self.world_x, world_y=self.world_y,
                            intensity_db=self.intensity_db)
        return png, meta_dir / f"{stem}.json"


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
                 = dummy_center_analyzer) -> None:
        super().__init__()
        self._rows = int(config.seabed.rows)
        self._stride = int(config.seabed.stride)
        self._cols = int(config.seabed.columns)
        self._analyzer = analyzer
        self._out_dir: Optional[Path] = None
        self._next_id = 0
        self._since_last = 0
        self._buf_rows: List[np.ndarray] = []
        self._buf_meta: List[tuple] = []    # (t, x, y, yaw, range, depth)
        self._last_pose: Optional[Tuple[float, float, float]] = None
        self._emitted_any = False

    # ---- live control -----------------------------------------------------------
    def set_output_dir(self, out_dir: Optional[Path]) -> None:
        """Images are written to disk only while a directory is set
        (i.e. while a recording session is active)."""
        self._out_dir = Path(out_dir) if out_dir is not None else None

    def reset(self) -> None:
        self._buf_rows.clear()
        self._buf_meta.clear()
        self._since_last = 0
        self._last_pose = None
        self._next_id = 0
        self._emitted_any = False

    # ---- ingestion --------------------------------------------------------------
    def on_sonar_ping(self, ping: SonarPing) -> None:
        y = ping.y_local
        if y.size < 2:
            return
        r = float(np.abs(y).max())
        if r <= 0:
            return
        col = np.clip(((r - y) / (2 * r) * (self._cols - 1)).astype(np.int32),
                      0, self._cols - 1)
        row = np.full(self._cols, np.nan, np.float32)
        row[col] = ping.intensity_db
        self._buf_rows.append(row)
        speed = 0.0
        if self._last_pose is not None:
            dt = ping.t - self._last_pose[0]
            if dt > 1e-3:
                speed = math.hypot(ping.robot_x - self._last_pose[1],
                                   ping.robot_y - self._last_pose[2]) / dt
        self._last_pose = (ping.t, ping.robot_x, ping.robot_y)
        self._buf_meta.append((ping.t, ping.robot_x, ping.robot_y, ping.yaw,
                               r, ping.water_depth, speed))
        # Bound memory to one window.
        if len(self._buf_rows) > self._rows:
            self._buf_rows.pop(0)
            self._buf_meta.pop(0)
        self._since_last += 1
        if (self._since_last >= self._stride
                and len(self._buf_rows) >= self._rows):
            self._since_last = 0
            self._emitted_any = True
            self._emit_window(self._rows)

    def flush(self) -> None:
        """Emit the final, possibly truncated picture — no data wasted.

        Everything acquired after the last full window ended (or the
        whole buffer if no full window was ever emitted, e.g. a mission
        shorter than ``rows`` pings) becomes one last image with fewer
        than ``rows`` rows. Called on Record OFF / STOP (live) and at
        the end of every offline generation / Run-AI pass.
        """
        tail = (self._since_last if self._emitted_any
                else len(self._buf_rows))
        tail = min(tail, len(self._buf_rows))
        if tail >= 2:                       # a 1-row "image" is noise
            self._emit_window(tail)
        self._since_last = 0

    # ---- window assembly -----------------------------------------------------------
    def _emit_window(self, n_rows: int) -> None:
        image = self._build(self._buf_rows[-n_rows:],
                            self._buf_meta[-n_rows:], self._next_id)
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
            p, m = image.save(self._out_dir)
            png_path, meta_path = str(p), str(m)
        image._png_path = png_path              # transported for publishers
        image._metadata_path = meta_path
        self.image_ready.emit(image)

    def _build(self, rows: List[np.ndarray], meta: List[tuple],
               image_id: int) -> SeabedImage:
        H, W = len(rows), self._cols
        arr = np.vstack(rows)
        t = np.array([m[0] for m in meta], np.float64)
        pose = np.array([[m[1], m[2], m[3]] for m in meta], np.float64)
        rng = np.array([m[4] for m in meta], np.float64)
        depth = np.array([m[5] for m in meta], np.float32)
        speed = np.array([m[6] for m in meta], np.float32)
        # Per-pixel world grids (vectorized over the whole window).
        j = np.arange(W, dtype=np.float64)
        y_local = rng[:, None] * (1.0 - 2.0 * j[None, :] / (W - 1))
        wx = pose[:, 0:1] - np.sin(pose[:, 2:3]) * y_local
        wy = pose[:, 1:2] + np.cos(pose[:, 2:3]) * y_local
        return SeabedImage(
            image_id=image_id, intensity_db=arr,
            world_x=wx.astype(np.float32), world_y=wy.astype(np.float32),
            row_t=t, row_pose=pose, row_range=rng,
            row_speed=speed, row_altitude=depth)


# ---------------------------------------------------------------------------
# Offline generation (replay window "Save pictures from the log")
# ---------------------------------------------------------------------------
def generate_from_pings(pings: Iterable[SonarPing], out_dir: Path,
                        config: AppConfig,
                        progress: Optional[Callable[[float], None]] = None,
                        run_analyzer: bool = False) -> int:
    """Run the identical imaging code over a decoded log; returns the
    number of images written to ``out_dir`` (+ inner ``metadata/``)."""
    imager = SeabedImager(config)
    imager.set_output_dir(out_dir)
    if not run_analyzer:
        imager._analyzer = lambda img: []        # dataset mode: raw images
    written = []
    imager.image_ready.connect(lambda img: written.append(img.image_id))
    pings = list(pings)
    for k, ping in enumerate(pings):
        if progress is not None and k % 200 == 0:
            progress(k / max(len(pings), 1))
        imager.on_sonar_ping(ping)
    imager.flush()                       # truncated tail: no data wasted
    if progress is not None:
        progress(1.0)
    return len(written)
