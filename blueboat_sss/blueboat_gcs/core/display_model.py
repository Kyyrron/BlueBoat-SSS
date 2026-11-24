"""One display model for every side-scan picture the GCS produces.

The live waterfall, the replay waterfall, the AI seabed pictures and the
mosaic all map raw device dB to pixels through **one** instance of
:class:`DisplayModel` per window, so the same seabed reads at the same
brightness everywhere and a picture inverts back to dB with the model
stored next to it. It replaces the three per-consumer ``SeabedContrast``
copies of ``core/contrast.py`` (2026-09-03), whose per-column *mean*
reference and 5th-percentile low handle were both contaminated by
acoustic shadows — the measured cause of the bright "nadir", the
vertical bands and the grey noise inside shadows
(``docs/SCIENTIFIC_BACKGROUND.md`` §8).

The model (``docs/SCIENTIFIC_BACKGROUND.md`` §9)
------------------------------------------------
Normalised level, per sample at slant range ``r`` in a ping whose
tracked bottom is ``h``::

    e = db + TL(r) - A_side(r / h)

* ``TL(r) = k·log10(r) + 2·α·r`` is the deterministic two-way
  transmission loss (``display.tl_k`` 40 dB/decade, ``display.
  tl_alpha_db_per_m`` 0.1 at 450 kHz). The stream is pre-TVG, so this is
  the gain a sonar receiver would have applied.
* ``A_side(x)`` is the empirical seabed level after TL as a function of
  the normalised slant range ``x = r/h`` (a proxy for the grazing angle,
  ``sin θ = 1/x``): per side, per log-spaced bin of ``x``, the **mode**
  of ``db + TL(r)`` over seabed samples (``r >= max(h, ring_m)``) — the
  peak of the bin's smoothed level histogram — constrained to physics:
  a robust straight line in ``log10 x`` (slope within
  ``[-40, 0]`` dB/decade: a TL-compensated seabed can only get dimmer
  with grazing angle, never brighter) is fitted through the bin modes —
  the line holding the **longest unbroken run** of bin modes, since the
  seabed is one continuous population across grazing angle and shadows,
  walls and a shore only interrupt it (2026-09-28) — and a bin refines
  the shape only while its mode stays within
  ``display.curve_tolerance_db`` of that line — a bin whose mode is
  another population takes the line exactly. The mode is immune to shadows, walls
  and targets as long as plain seabed holds the plurality of a bin (a
  median already shifts by half a texture deviation at 30 %
  contamination, which is what banded the image); the line is what
  survives the enclosed-basin case where the whole far range lies in
  the shadow of a wall on every ping — there the bin mode is the noise
  floor, 25 dB under the seabed, and without the constraint the far
  range rendered lifted and the wall tops saturated. The ``x`` axis
  makes the curve invariant to altitude and to the range setting, so a
  range change needs no rebuild. Sparse bins blend toward the line. For
  ``x < 1`` (the water column) ``A`` is held at ``A(1)``, so the
  extrapolated ``TL`` predicts a seabed far brighter than anything in the
  water column and ``e`` falls tens of dB below the seabed: the nadir and
  the transmit ringing go black by physics — nothing is masked, nothing
  erased, the raw dB is kept.
* The seabed median of ``e`` is 0 dB by construction.
* **Receive-gain compensation (2026-09-15).** The device's auto-gain
  walks a ladder whose ``analog_gain`` is an amplitude gain: the reported
  dB axis moves by ``20·log10`` of its ratio (measured on the harbour
  logs: every downward step matches to ~1 dB; ``project_review.md``
  §3.2). Per row and per side the term ``g = 20·log10(analog_gain /
  display.gain_reference_analog)`` (reference 74.55 = gain index 4, the
  calibration anchor) is removed from the level, ``e = db + TL(r) -
  A_side(r/h) - g``, so a mid-mission gain step no longer renders the
  same seabed at a different brightness. ``g = 0`` when the gain is
  unknown (a ping without its raw profile, legacy fixtures).

Transfer to a unit brightness (the renderer then applies the colour
map; the AI pictures take ``round(255·u)``). Two transfers exist
(``display.transfer``); the **power law** is the shipped default::

    p = 10 ** (gamma · (e - hi) / 10)
    u = p                                   for p <= knee
    u = knee + (1-knee)·(1 - exp(-(p-knee)/(1-knee)))   above the knee

and the **dB window** (``db_window``, the reading of SonarView's own
recorded view state — a window on the dB-linear per-ping counts — that
``project_review.md`` §3.2 found closest to its picture)::

    W = window_db / gamma
    u = clip((e - (hi - W)) / W, 0, 1)

``hi`` is a robust high percentile of ``e`` over seabed samples
(``display.hi_pct``) and ``gamma`` the operator's Contrast slider
(``display.gamma``; under the power law 1.0 is linear power and 0.5
amplitude; under the dB window it scales the window, 1.0 = ``window_db``
dB of dynamic range). There is deliberately **no low handle** in the
power law: a shadow 15–40 dB below the seabed lands at ``u < 0.03`` on
its own, and nothing the shadows do can drag a handle around any more.
Above the soft ``knee`` (``display.knee``, 0.7) highlights are
compressed instead of clipped, so a wall face or a debris field keeps
its texture for a few dB past ``hi`` before saturating. The dB window
has no knee: it clips at ``hi`` and at ``hi - W``.

Invertible: power law ``p = u`` below the knee, ``p = knee -
(1-knee)·ln(1 - (u-knee)/(1-knee))`` above it, ``e = hi +
(10/gamma)·log10(p)``; dB window ``e = hi - W + u·W``; then ``db = e -
TL(r) + A_side(r/h) + g`` — every picture stores ``hi``, ``gamma``,
``knee``, ``transfer``, ``window_db``, the gain reference, the TL
parameters, the two ``A`` curves and per row ``h`` and ``g`` (JSON +
npz, :meth:`snapshot`).

Life cycle
----------
Live, :meth:`observe_row` accumulates until ``display.warmup_rows`` rows
were seen, then the model **freezes** and ``version`` bumps once (the
services re-render every tile on that bump and never again). Replay and
offline exports call :meth:`fit` for a one-pass fixed model. The model
is layout-independent (physical units in, per-row ``h``), so a waterfall
remap never touches it, and :meth:`snapshot` returns an immutable copy
that a pool thread can render with safely.
"""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable, Optional, Tuple

import math

import numpy as np

from ..models.sonar import SonarPing, build_slant_row, native_bins

#: Histogram of the TL-compensated level ``d = db + TL(r)`` [dB].
_D_LO, _D_HI, _D_STEP = -60.0, 220.0, 0.5
_D_BINS = int(round((_D_HI - _D_LO) / _D_STEP))

#: Dense lookup of ``A`` over ``log10(x)``: resolution and lower bound
#: (x = 0.01 covers the ringing core at any altitude).
_LX_LO, _LX_STEP = -2.0, 0.005

#: Width of the median filter across ``x`` bins (odd).
_MEDFILT = 5

#: Gaussian kernel (in ``_D_STEP`` bins) that smooths each x bin's level
#: histogram before its **mode** is read: 3 dB, under the ~5.6 dB spread
#: of fully developed speckle, so texture stays one peak while a shadow
#: population 15-40 dB down or a wall 10-20 dB up forms its own — the
#: seabed peak wins as long as it holds the plurality, where a median
#: would already have shifted by half a texture deviation at 30 %
#: contamination (the vertical-band mechanism).
_MODE_SIGMA_BINS = 6

#: Physical bounds on the slope of the TL-compensated seabed level in
#: log10(x): Lambert alone gives -20 dB/decade, a beam pattern and a
#: rough seabed steepen it; it never rises with range.
_SLOPE_MIN, _SLOPE_MAX = -40.0, 0.0
_SLOPE_STEP = 5.0                         # consensus search step [dB/decade]
_PRIOR_SLOPE_DB_PER_DECADE = -20.0        # fallback when nothing fits

SCHEMA = "blueboat_display_model/1"

#: The device's ``analog_gain`` at gain index 4 — the calibration anchor
#: (measured on every field profile at that index; the simulator's
#: ANALOG_GAIN_TABLE carries the same value).
GAIN_REFERENCE_ANALOG = 74.55

#: Transfers the model can apply (``display.transfer``).
TRANSFERS = ("power", "db_window")


def gain_offset_db(analog_gain, gain_ref: float = GAIN_REFERENCE_ANALOG):
    """Level shift [dB] the receive chain added at ``analog_gain``
    relative to the reference: ``20·log10(analog_gain / gain_ref)``
    (an amplitude gain). 0 where the gain is unknown (``<= 0``)."""
    ref = float(gain_ref)
    if isinstance(analog_gain, (int, float)):
        # Scalar fast path: called three times per ping on the live path
        # (waterfall, mosaic, imager); the numpy version cost ~0.1 ms/ping.
        g = float(analog_gain)
        if not (g > 0.0 and ref > 0.0) or not math.isfinite(g):
            return 0.0
        return 20.0 * math.log10(g / ref)
    g = np.asarray(analog_gain, dtype=np.float64)
    with np.errstate(divide="ignore", invalid="ignore"):
        out = np.where((g > 0.0) & (ref > 0.0), 20.0 * np.log10(g / ref), 0.0)
    return float(out) if out.ndim == 0 else out


def _column_slant(width: int, pitch_m: float) -> np.ndarray:
    """Slant range [m] of each of ``width`` centred slant-bin columns
    (port mirrored left, starboard right — the ``build_slant_row``
    layout)."""
    half = width // 2
    col = np.arange(width)
    k = np.where(col < half, half - 1 - col, col - half)
    return (k + 0.5) * pitch_m


def _column_side(width: int) -> np.ndarray:
    """0 = port (left half), 1 = starboard (right half)."""
    return (np.arange(width) >= width // 2).astype(np.int8)


def transmission_loss_db(r: np.ndarray, k: float, alpha: float) -> np.ndarray:
    """Two-way ``TL(r) = k·log10(r) + 2·α·r`` [dB]; ``r`` clipped at 1 cm
    so the ringing core does not reach ``-inf``."""
    rr = np.maximum(np.asarray(r, dtype=np.float64), 0.01)
    return k * np.log10(rr) + 2.0 * alpha * rr


def _mode_index(hist_row: np.ndarray, sigma_bins: int = _MODE_SIGMA_BINS
                ) -> float:
    """Sub-bin position of the dominant peak of a 1-D histogram."""
    kx = np.arange(-3 * sigma_bins, 3 * sigma_bins + 1, dtype=np.float64)
    kern = np.exp(-0.5 * (kx / sigma_bins) ** 2)
    sm = np.convolve(hist_row.astype(np.float64), kern, mode="same")
    i = int(np.argmax(sm))
    if 0 < i < sm.size - 1:
        y0, y1, y2 = sm[i - 1], sm[i], sm[i + 1]
        den = y0 - 2.0 * y1 + y2
        if den < 0:                              # parabolic refinement
            return i + 0.5 * (y0 - y2) / den
    return float(i)


def _longest_run(mask: np.ndarray) -> Tuple[int, int]:
    """``(i, j)``: the longest stretch ``mask[i:j]`` of consecutive True
    (the first one on a tie); ``(0, 0)`` when there is none."""
    best, start = (0, 0), None
    for k, v in enumerate(np.append(np.asarray(mask, dtype=bool), False)):
        if v and start is None:
            start = k
        elif not v and start is not None:
            if k - start > best[1] - best[0]:
                best = (start, k)
            start = None
    return best


def _median_filter(v: np.ndarray, width: int) -> np.ndarray:
    if v.size < width or width <= 1:
        return v.copy()
    h = width // 2
    pad = np.concatenate([np.full(h, v[0]), v, np.full(h, v[-1])])
    win = np.lib.stride_tricks.sliding_window_view(pad, width)
    return np.median(win, axis=1)


# ---------------------------------------------------------------------------
# Immutable snapshot: what a picture stores and what a pool thread renders
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DisplayModelSnapshot:
    """Frozen copy of a :class:`DisplayModel` — the complete dB→pixel
    mapping of one mission, safe to share across threads and to serialise
    next to every picture."""

    tl_k: float
    tl_alpha: float
    x_max: float
    x_centers: np.ndarray            # coarse bin centres in x (1 .. x_max)
    a_port: np.ndarray               # A per coarse bin [dB]
    a_stbd: np.ndarray
    hi_db: float                     # window top over e
    gamma: float
    ring_m: float
    rows_observed: int
    frozen: bool
    version: int
    ready: bool
    knee: float = 0.7                # soft knee of the transfer (1 = hard clip)
    #: Receive-gain reference: per row and side ``g = 20·log10(analog_gain
    #: / gain_ref)`` is removed from the level (0 when the gain is unknown).
    gain_ref: float = GAIN_REFERENCE_ANALOG
    #: "power" (default) or "db_window"; see the module docstring.
    transfer: str = "power"
    #: Dynamic range [dB] of the dB window at gamma 1 (scaled by 1/gamma).
    window_db: float = 40.0

    @staticmethod
    def _gain_matrix(gain_rows, n: int) -> Optional[np.ndarray]:
        """Per-row, per-side gain offsets as an (n, 2) float32 array, or
        None when nothing was supplied / everything is zero."""
        if gain_rows is None:
            return None
        g = np.asarray(gain_rows, dtype=np.float32)
        if g.ndim == 1:
            g = np.broadcast_to(g.reshape(1, -1), (n, 2)) if g.size == 2 \
                else np.repeat(g.reshape(-1, 1), 2, axis=1)
        if g.shape != (n, 2):
            raise ValueError(f"gain_rows must be (n, 2), got {g.shape}")
        if not np.any(g):
            return None
        return g

    # ---- dense lookups (derived, cached lazily) --------------------------------
    def _dense(self, a_coarse: np.ndarray) -> np.ndarray:
        lx = np.arange(_LX_LO, np.log10(self.x_max) + _LX_STEP, _LX_STEP)
        cx = np.log10(self.x_centers)
        # Hold A(1) below x = 1 (water column), the last bin beyond x_max.
        dense = np.interp(lx, cx, a_coarse, left=a_coarse[0], right=a_coarse[-1])
        return dense.astype(np.float32)

    def _lookup(self, side: int) -> np.ndarray:
        key = "_dense_port" if side == 0 else "_dense_stbd"
        cached = self.__dict__.get(key)
        if cached is None:
            cached = self._dense(self.a_port if side == 0 else self.a_stbd)
            object.__setattr__(self, key, cached)
        return cached

    def a_at(self, x: np.ndarray, side: int) -> np.ndarray:
        """``A_side(x)`` for an array of normalised slant ranges."""
        lut = self._lookup(side)
        idx = np.rint((np.log10(np.maximum(x, 10 ** _LX_LO)) - _LX_LO)
                      / _LX_STEP).astype(np.int64)
        return lut[np.clip(idx, 0, lut.size - 1)]

    # ---- forward -------------------------------------------------------------
    def normalise(self, rows: np.ndarray, pitch_m: float,
                  h_rows: np.ndarray,
                  gain_rows: Optional[np.ndarray] = None) -> np.ndarray:
        """Normalised level ``e`` of built slant rows (2-D, port mirrored
        left) with one tracked bottom ``h`` per row (0 → the model's
        fallback altitude) and, optionally, one receive-gain offset [dB]
        per row and side (``gain_rows``, shape (n, 2): port, starboard —
        see :func:`gain_offset_db`). NaN passes through."""
        rows = np.asarray(rows, dtype=np.float32)
        if rows.ndim == 1:
            rows = rows[None, :]
        n, w = rows.shape
        if w == 0 or pitch_m <= 0.0 or not self.ready:
            return rows.copy()
        slant = _column_slant(w, pitch_m)
        side = _column_side(w)
        h = np.asarray(h_rows, dtype=np.float64).reshape(-1)
        if h.size != n:
            h = np.full(n, float(h[0]) if h.size else 0.0)
        h = np.where(h > 0.0, h, self.fallback_h)
        gains = self._gain_matrix(gain_rows, n)
        tl = transmission_loss_db(slant, self.tl_k, self.tl_alpha).astype(np.float32)
        lx = np.log10(slant)[None, :] - np.log10(h)[:, None]     # log10(x)
        idx = np.rint((lx - _LX_LO) / _LX_STEP).astype(np.int64)
        out = rows + tl[None, :]
        for s in (0, 1):
            cols = side == s
            if not cols.any():
                continue
            lut = self._lookup(s)
            ii = np.clip(idx[:, cols], 0, lut.size - 1)
            out[:, cols] -= lut[ii]
            if gains is not None:
                out[:, cols] -= gains[:, s][:, None]
        return out

    def normalise_samples(self, db: np.ndarray, r: np.ndarray, h: float,
                          side: int, gain_db: float = 0.0) -> np.ndarray:
        """Per-sample form for the mosaic (ground samples: ``r =
        hypot(|y|, h)``); ``gain_db`` is the side's receive-gain offset."""
        db = np.asarray(db, dtype=np.float32)
        if db.size == 0 or not self.ready:
            return db.copy()
        r = np.asarray(r, dtype=np.float64)
        hh = float(h) if h > 0.0 else self.fallback_h
        tl = transmission_loss_db(r, self.tl_k, self.tl_alpha).astype(np.float32)
        return db + tl - self.a_at(r / hh, side) - np.float32(gain_db)

    def window_width_db(self, gamma: Optional[float] = None) -> float:
        """Dynamic range [dB] of the dB-window transfer at ``gamma``."""
        g = float(self.gamma if gamma is None else gamma)
        return float(self.window_db) / max(g, 1e-6)

    def to_unit(self, e: np.ndarray, gamma: Optional[float] = None
                ) -> np.ndarray:
        """Unit brightness of a normalised level (NaN preserved).

        ``power``: ``p = 10^(gamma·(e − hi)/10)``, soft-kneed above
        ``knee``, clipped to [0, 1]. ``db_window``: linear in dB over
        ``window_db / gamma`` below ``hi``, hard-clipped.
        """
        g = float(self.gamma if gamma is None else gamma)
        e = np.asarray(e, dtype=np.float32)
        if self.transfer == "db_window":
            W = np.float32(self.window_width_db(g))
            p = (e - (np.float32(self.hi_db) - W)) / W
            return np.clip(p, 0.0, 1.0, out=p)
        with np.errstate(over="ignore", invalid="ignore"):
            p = np.power(np.float32(10.0),
                         np.float32(g / 10.0) * (e - np.float32(self.hi_db)))
            k = np.float32(self.knee)
            if 0.0 < self.knee < 1.0:
                hi = p > k
                p[hi] = k + (1.0 - k) * (1.0 - np.exp(-(p[hi] - k) / (1.0 - k)))
        return np.clip(p, 0.0, 1.0, out=p)

    def to_png8(self, e: np.ndarray, gamma: Optional[float] = None
                ) -> np.ndarray:
        u = self.to_unit(e, gamma)
        out = np.zeros(u.shape, np.uint8)
        fin = np.isfinite(u)
        out[fin] = np.rint(u[fin] * 255.0).astype(np.uint8)
        return out

    # ---- inverse ---------------------------------------------------------------
    def invert_unit(self, u: np.ndarray, gamma: Optional[float] = None
                    ) -> np.ndarray:
        """``e`` from unit brightness (``u <= 0`` → -inf, ``u >= 1`` → the
        knee's asymptote)."""
        g = float(self.gamma if gamma is None else gamma)
        u = np.asarray(u, dtype=np.float64)
        if self.transfer == "db_window":
            W = self.window_width_db(g)
            return (self.hi_db - W) + u * W
        p = u.copy()
        k = float(self.knee)
        if 0.0 < k < 1.0:
            hi = u > k
            frac = np.clip((u[hi] - k) / (1.0 - k), 0.0, 1.0 - 1e-6)
            p[hi] = k - (1.0 - k) * np.log(1.0 - frac)
        with np.errstate(divide="ignore"):
            return self.hi_db + (10.0 / g) * np.log10(p)

    def invert_rows(self, u: np.ndarray, pitch_m: float, h_rows: np.ndarray,
                    gamma: Optional[float] = None,
                    gain_rows: Optional[np.ndarray] = None) -> np.ndarray:
        """Raw dB from unit-brightness rows (the exact inverse of
        ``normalise`` + ``to_unit`` where ``u`` was not clipped);
        ``gain_rows`` as in :meth:`normalise`."""
        e = self.invert_unit(u, gamma)
        if e.ndim == 1:
            e = e[None, :]
        n, w = e.shape
        slant = _column_slant(w, pitch_m)
        side = _column_side(w)
        h = np.asarray(h_rows, dtype=np.float64).reshape(-1)
        if h.size != n:
            h = np.full(n, float(h[0]) if h.size else 0.0)
        h = np.where(h > 0.0, h, self.fallback_h)
        gains = self._gain_matrix(gain_rows, n)
        tl = transmission_loss_db(slant, self.tl_k, self.tl_alpha)
        out = e - tl[None, :]
        for s in (0, 1):
            cols = side == s
            if cols.any():
                x = slant[None, cols] / h[:, None]
                out[:, cols] += self.a_at(x, s)
                if gains is not None:
                    out[:, cols] += gains[:, s][:, None]
        return out

    @property
    def fallback_h(self) -> float:
        return float(self.__dict__.get("_fallback_h", 1.0))

    # ---- persistence -------------------------------------------------------------
    def to_json(self) -> dict:
        return {
            "schema": SCHEMA,
            "tl_k_db_per_decade": float(self.tl_k),
            "tl_alpha_db_per_m": float(self.tl_alpha),
            "x_max": float(self.x_max),
            "x_centers": [float(v) for v in self.x_centers],
            "a_port_db": [float(v) for v in self.a_port],
            "a_stbd_db": [float(v) for v in self.a_stbd],
            "hi_db": float(self.hi_db),
            "gamma": float(self.gamma),
            "knee": float(self.knee),
            "transfer": str(self.transfer),
            "window_db": float(self.window_db),
            "gain_reference_analog": float(self.gain_ref),
            "ring_m": float(self.ring_m),
            "fallback_altitude_m": self.fallback_h,
            "rows_observed": int(self.rows_observed),
            "frozen": bool(self.frozen),
            "mapping": (
                "e = db + TL(r) - A_side(r/h) - g; TL(r) = tl_k*log10(max(r,0.01)) "
                "+ 2*tl_alpha*r; A_side(x) = linear interpolation of a_*_db "
                "over log10(x_centers), held at a[0] for x < x_centers[0] and "
                "at a[-1] beyond; r = (k+0.5)*bin_pitch_m of the column, h = "
                "the row's altitude_m (fallback_altitude_m when 0); g = the "
                "row's gain_<side>_db = 20*log10(analog_gain / "
                "gain_reference_analog), 0 when unknown. "
                "transfer 'power': p = 10^(gamma*(e-hi_db)/10); u = p for p <= "
                "knee, else knee + (1-knee)*(1-exp(-(p-knee)/(1-knee))). "
                "transfer 'db_window': W = window_db/gamma; u = clip((e - "
                "(hi_db - W))/W, 0, 1). pixel = round(255*clip(u, 0, 1)). "
                "Invert: power: p = u for u <= knee, else knee - "
                "(1-knee)*ln(1-(u-knee)/(1-knee)); e = hi_db + "
                "(10/gamma)*log10(p); db_window: e = hi_db - W + u*W; then "
                "db = e - TL(r) + A_side(r/h) + g. NaN (no sample) -> 0."),
        }

    @classmethod
    def from_json(cls, d: dict) -> "DisplayModelSnapshot":
        snap = cls(tl_k=float(d["tl_k_db_per_decade"]),
                   tl_alpha=float(d["tl_alpha_db_per_m"]),
                   x_max=float(d["x_max"]),
                   x_centers=np.asarray(d["x_centers"], np.float64),
                   a_port=np.asarray(d["a_port_db"], np.float64),
                   a_stbd=np.asarray(d["a_stbd_db"], np.float64),
                   hi_db=float(d["hi_db"]), gamma=float(d["gamma"]),
                   ring_m=float(d.get("ring_m", 1.0)),
                   rows_observed=int(d.get("rows_observed", 0)),
                   frozen=bool(d.get("frozen", True)), version=0, ready=True,
                   knee=float(d.get("knee", 1.0)),
                   gain_ref=float(d.get("gain_reference_analog",
                                        GAIN_REFERENCE_ANALOG)),
                   transfer=str(d.get("transfer", "power")),
                   window_db=float(d.get("window_db", 40.0)))
        object.__setattr__(snap, "_fallback_h",
                           float(d.get("fallback_altitude_m", 1.0)))
        return snap

    def npz_items(self) -> dict:
        return {"display_x_centers": self.x_centers.astype(np.float32),
                "display_a_port": self.a_port.astype(np.float32),
                "display_a_stbd": self.a_stbd.astype(np.float32),
                "display_model_json": json.dumps(self.to_json())}


# ---------------------------------------------------------------------------
# The streaming estimator
# ---------------------------------------------------------------------------
class DisplayModel:
    """Streaming / one-pass estimator of the mission display model."""

    def __init__(self, config) -> None:
        d = config.display
        self._k = float(d.tl_k)
        self._alpha = float(d.tl_alpha_db_per_m)
        self._x_bins = int(d.x_bins)
        self._x_max = float(d.x_max)
        self._warmup = int(d.warmup_rows)
        self._hi_pct = float(d.hi_pct)
        self._min_count = int(d.min_bin_count)
        self._tol = float(getattr(d, "curve_tolerance_db", 4.0))
        self.gamma = float(d.gamma)
        self.knee = float(getattr(d, "knee", 0.7))
        self.transfer = str(getattr(d, "transfer", "power")).lower()
        if self.transfer not in TRANSFERS:
            raise ValueError(f"display.transfer must be one of {TRANSFERS}, "
                             f"got {self.transfer!r}")
        self.window_db = float(getattr(d, "window_db", 40.0))
        self._gain_ref = float(getattr(d, "gain_reference_analog",
                                       GAIN_REFERENCE_ANALOG))
        self._gain_comp = bool(getattr(d, "gain_compensation", True))
        self._ring_m = float(config.mosaic.water_column_min_m)
        self._lx_edges = np.linspace(0.0, np.log10(self._x_max), self._x_bins + 1)
        self._x_centers = 10 ** ((self._lx_edges[:-1] + self._lx_edges[1:]) / 2.0)
        self.reset()

    # ---- state -------------------------------------------------------------------
    def reset(self) -> None:
        """Forget everything and unfreeze (Clear SSS data / new mission)."""
        self._hist = np.zeros((2, self._x_bins, _D_BINS), dtype=np.int64)
        self._rows = 0
        self._next_publish = self._warmup
        self._frozen = False
        self._h_recent: Deque[float] = deque(maxlen=2000)   # fallback altitude
        self._snapshot: Optional[DisplayModelSnapshot] = None
        self._snap_key = None
        self.version = getattr(self, "version", 0) + 1

    @property
    def frozen(self) -> bool:
        return self._frozen

    @property
    def rows_observed(self) -> int:
        return self._rows

    @property
    def ready(self) -> bool:
        """At least one populated x bin on one side."""
        return bool((self._hist.sum(axis=2) >= self._min_count).any())

    def freeze(self) -> None:
        if not self._frozen:
            self._frozen = True
            self.version += 1
            self._snapshot = None

    def unfreeze(self) -> None:
        self._frozen = False

    # ---- ingestion -------------------------------------------------------------
    def gain_offsets(self, ping: SonarPing) -> Tuple[float, float]:
        """(port, starboard) receive-gain offsets [dB] of a ping from its
        ``*_analog_gain`` fields (0 when unknown or compensation is off)."""
        if not self._gain_comp:
            return 0.0, 0.0
        return (float(gain_offset_db(getattr(ping, "port_analog_gain", 0.0),
                                     self._gain_ref)),
                float(gain_offset_db(getattr(ping, "stbd_analog_gain", 0.0),
                                     self._gain_ref)))

    def observe_row(self, row: np.ndarray, pitch_m: float, h: float,
                    gain_db: Tuple[float, float] = (0.0, 0.0)) -> None:
        """Fold one built slant row (port mirrored left, starboard right)
        into the estimator. ``h`` = the row's tracked bottom [m]; rows
        without one (``h <= 0``) do not feed the curve (no grazing angle
        to file them under) but still count toward the warm-up.
        ``gain_db`` = the row's (port, starboard) receive-gain offsets,
        removed before the level is filed (:func:`gain_offset_db`)."""
        if self._frozen or pitch_m <= 0.0 or row.size < 2:
            return
        row = np.asarray(row, dtype=np.float32)
        finite = np.isfinite(row)
        if not finite.any():
            return
        self._rows += 1
        self._snapshot = None
        if h > 0.0:
            self._h_recent.append(float(h))
            w = row.size
            slant = _column_slant(w, pitch_m)
            side = _column_side(w)
            sb = finite & (slant >= max(float(h), self._ring_m))
            if sb.any():
                d = row[sb] + transmission_loss_db(slant[sb], self._k, self._alpha)
                lx = np.log10(slant[sb] / float(h))
                xi = np.clip((lx / self._lx_edges[-1] * self._x_bins).astype(np.int64),
                             0, self._x_bins - 1)
                for s in (0, 1):
                    m = side[sb] == s
                    if m.any():
                        ds = d[m] - np.float32(gain_db[s])
                        di = np.clip(((ds - _D_LO) / _D_STEP).astype(np.int64),
                                     0, _D_BINS - 1)
                        flat = xi[m] * _D_BINS + di
                        self._hist[s].ravel()[:] += np.bincount(
                            flat, minlength=self._x_bins * _D_BINS)
        if self._rows >= self._next_publish:
            # Live, the estimate is re-published, never frozen. Before
            # 2026-09-19 this line froze the model after `warmup_rows` and
            # nothing ever unfroze it, so every ping after the first ~15 s
            # was displayed through a curve fitted on the launch area -- one
            # altitude, one range setting, one bottom type -- while replay
            # fitted the whole file. That is the whole of "replay looks
            # better than live". Worse, `observe_row` returns immediately
            # when frozen, so a session whose bottom tracker had not locked
            # during those first 300 rows froze an EMPTY histogram and spent
            # the rest of its life on the raw 2/98 percentile fallback.
            #
            # Nothing extra happens per ping: the histogram was already
            # accumulated per row, and the solve behind `version` is cached
            # (`_snapshot`) and runs at most once per publish -- once every
            # `warmup_rows` rows, 15 s at 20 Hz.
            self._next_publish = self._rows + max(self._warmup, 1)
            self.version += 1
            self._snapshot = None

    def refit(self, pings: Iterable[SonarPing]) -> None:
        """Re-estimate in place from decoded pings and freeze (the
        services holding this instance re-render on the version bump)."""
        self.reset()
        warm = self._warmup
        self._warmup = 1 << 60                  # never freeze mid-pass
        try:
            for ping in pings:
                nb = native_bins(ping)
                if nb is None or nb.extent_bins == 0:
                    continue
                row = build_slant_row(nb, nb.bin_size_m, nb.extent_bins)
                h = float(getattr(ping, "bottom_slant_m", 0.0))
                if h <= 0.0:
                    h = max(float(getattr(ping, "water_depth", 0.0)), 0.0)
                self.observe_row(row, nb.bin_size_m, h, self.gain_offsets(ping))
        finally:
            self._warmup = warm
        self.freeze()

    @classmethod
    def fit(cls, config, pings: Iterable[SonarPing]) -> "DisplayModel":
        """One pass over decoded pings (replay / offline export), frozen."""
        model = cls(config)
        model.refit(pings)
        return model

    # ---- estimates -------------------------------------------------------------
    def _curve(self, side: int) -> np.ndarray:
        """``A_side`` per coarse bin: per-bin mode of ``d``, constrained to
        a robust physical line in ``log10 x`` (±``curve_tolerance_db``),
        prior-blended where sparse, median-filtered across bins."""
        hist = self._hist[side]
        counts = hist.sum(axis=1)
        med = np.full(self._x_bins, np.nan)
        for i in np.nonzero(counts > 0)[0]:
            med[i] = _D_LO + (_mode_index(hist[i]) + 0.5) * _D_STEP
        lx = np.log10(self._x_centers)
        valid = counts >= self._min_count
        line = self._fit_line(lx, med, counts, valid)
        # Bins whose mode sits within the tolerance of the line are seabed
        # and refine the shape (beam pattern, specular bulge near nadir);
        # a bin dominated by another population — a wall's shadow (25 dB
        # under) or its face (15 dB over) on every ping — is not seabed
        # at all, so it takes the physical line exactly.
        resid = np.nan_to_num(med - line, nan=0.0)
        dev = np.where(np.abs(resid) <= self._tol, resid, 0.0)
        n = counts.astype(np.float64)
        n0 = float(self._min_count)
        blended = line + np.where(counts > 0, n / (n + n0), 0.0) * dev
        return _median_filter(blended, _MEDFILT)

    def _fit_line(self, lx: np.ndarray, med: np.ndarray, counts: np.ndarray,
                  valid: np.ndarray) -> np.ndarray:
        """The physical line through the bin modes, found by consensus.

        Seabed bins lie on one line in ``log10 x`` whose slope the physics
        bounds to ``[_SLOPE_MIN, _SLOPE_MAX]`` but does not fix: Lambert
        alone gives -20 dB/decade, a receiver that compensates part of its
        range loss gives a flatter curve. Bins dominated by a wall's shadow
        lie on the noise floor, whose TL-compensated level *rises*
        40 dB/decade, and a shore or a one-sided shadow drops a far band
        15-20 dB under the seabed. So every valid bin proposes a line
        through itself at every slope of the physical range (5 dB/decade
        steps), and the proposal with the **longest contiguous run** of
        bins within the tolerance wins (ties: the most samples): the
        seabed is one unbroken population across grazing angle, the other
        populations only interrupt it. Slope and level are then refined on
        that run, clamped to the physical range.

        Until 2026-09-28 the consensus ran at the -20 prior slope alone and
        scored plain inlier counts. On a flat seabed with a low far band on
        one side, a -20 line through the middle bins caught the far band and
        won; the near-nadir bins then sat outside the tolerance and took
        the line, 7-17 dB above their modes, and that side rendered a dark
        band beside the nadir on every ping (``tests/test_display_model.py
        ::test_a_one_sided_far_band_does_not_darken_that_sides_nadir``).
        """
        ok = valid & np.isfinite(med)
        x_all = lx
        if ok.sum() < 2:
            if ok.sum() == 1:
                level = float(med[ok][0] - _PRIOR_SLOPE_DB_PER_DECADE * lx[ok][0])
            elif np.isfinite(med).any():
                level = float(np.nanmedian(med - _PRIOR_SLOPE_DB_PER_DECADE * lx))
            else:
                level = 0.0
            return level + _PRIOR_SLOPE_DB_PER_DECADE * x_all
        x, y, w = lx[ok], med[ok], counts[ok].astype(np.float64)
        tol = self._tol
        # 1. consensus: the line through one bin, at one physical slope,
        #    holding the longest unbroken run of bins within the tolerance
        best_key, b, c, run = (-1, 0.0), _PRIOR_SLOPE_DB_PER_DECADE, float(y[0]), (0, 1)
        for slope in np.arange(_SLOPE_MIN, _SLOPE_MAX + 1e-9, _SLOPE_STEP):
            for level in y - slope * x:
                i, j = _longest_run(np.abs(y - (level + slope * x)) <= tol)
                key = (j - i, float(w[i:j].sum()))
                if key > best_key:
                    best_key, b, c, run = key, float(slope), float(level), (i, j)
        # 2. refine slope + level on that run (twice, re-selecting the run)
        for _ in range(2):
            inl = np.zeros(x.size, dtype=bool)
            inl[run[0]:run[1]] = True
            if inl.sum() < 2:
                break
            ww = w[inl]
            xs, ys = x[inl], y[inl]
            sw = ww.sum()
            mx, my = (ww * xs).sum() / sw, (ww * ys).sum() / sw
            vx = (ww * (xs - mx) ** 2).sum()
            if vx > 1e-12:
                b = float(np.clip((ww * (xs - mx) * (ys - my)).sum() / vx,
                                  _SLOPE_MIN, _SLOPE_MAX))
            c = float(my - b * mx)
            i, j = _longest_run(np.abs(y - (c + b * x)) <= tol)
            if j - i >= 2:
                run = (i, j)
        return c + b * x_all

    def _window(self, a_port: np.ndarray, a_stbd: np.ndarray) -> float:
        """``hi``: percentile of the seabed ``e`` distribution, read off the
        2-D histograms shifted by each bin's own ``A`` (exact to the bin)."""
        e_hist = np.zeros(2 * _D_BINS, dtype=np.int64)     # e in [-2*span..]
        off = _D_BINS // 2
        for side, a in ((0, a_port), (1, a_stbd)):
            hist = self._hist[side]
            for i in range(self._x_bins):
                if hist[i].sum() == 0:
                    continue
                shift = int(np.rint((a[i] - _D_LO) / _D_STEP))
                # e index = d index - shift (+off to stay positive)
                lo = off - shift
                src = hist[i]
                dst_lo = max(lo, 0)
                dst_hi = min(lo + _D_BINS, e_hist.size)
                if dst_hi > dst_lo:
                    e_hist[dst_lo:dst_hi] += src[dst_lo - lo:dst_hi - lo]
        tot = int(e_hist.sum())
        if tot == 0:
            return 0.0
        cum = np.cumsum(e_hist) / tot * 100.0
        idx = int(np.searchsorted(cum, self._hi_pct))
        return float((min(idx, e_hist.size - 1) - off + 0.5) * _D_STEP)

    def _fallback_h(self) -> float:
        """Median tracked bottom of the rows seen (1 m before any)."""
        if not self._h_recent:
            return 1.0
        return float(np.median(np.fromiter(self._h_recent, dtype=np.float64)))

    def snapshot(self) -> DisplayModelSnapshot:
        """Immutable copy of the current estimate (cached until the next
        observed row / freeze)."""
        key = (self._rows, self.version, self.gamma, self.knee,
               self.transfer, self.window_db)
        if self._snapshot is not None and self._snap_key == key:
            return self._snapshot
        a_port = self._curve(0)
        a_stbd = self._curve(1)
        # A side with no data at all borrows the other's curve.
        if not (self._hist[0].sum(axis=1) > 0).any() and (self._hist[1].sum(axis=1) > 0).any():
            a_port = a_stbd.copy()
        elif not (self._hist[1].sum(axis=1) > 0).any() and (self._hist[0].sum(axis=1) > 0).any():
            a_stbd = a_port.copy()
        snap = DisplayModelSnapshot(
            tl_k=self._k, tl_alpha=self._alpha, x_max=self._x_max,
            x_centers=self._x_centers.copy(), a_port=a_port, a_stbd=a_stbd,
            hi_db=self._window(a_port, a_stbd), gamma=self.gamma,
            ring_m=self._ring_m, rows_observed=self._rows,
            frozen=self._frozen, version=self.version, ready=self.ready,
            knee=self.knee, gain_ref=self._gain_ref,
            transfer=self.transfer, window_db=self.window_db)
        object.__setattr__(snap, "_fallback_h", self._fallback_h())
        self._snapshot, self._snap_key = snap, key
        return snap

    # ---- convenience (delegates to the snapshot) ------------------------------------
    def normalise(self, rows: np.ndarray, pitch_m: float,
                  h_rows: np.ndarray,
                  gain_rows: Optional[np.ndarray] = None) -> np.ndarray:
        return self.snapshot().normalise(rows, pitch_m, h_rows, gain_rows)

    def normalise_samples(self, db, r, h: float, side: int,
                          gain_db: float = 0.0) -> np.ndarray:
        return self.snapshot().normalise_samples(db, r, h, side, gain_db)

    def to_unit(self, e: np.ndarray, gamma: Optional[float] = None) -> np.ndarray:
        return self.snapshot().to_unit(e, gamma)

    def render_unit(self, rows: np.ndarray, pitch_m: float, h_rows,
                    gamma: Optional[float] = None,
                    gain_rows: Optional[np.ndarray] = None) -> np.ndarray:
        """``normalise`` then ``to_unit`` in one call (NaN preserved)."""
        snap = self.snapshot()
        return snap.to_unit(snap.normalise(rows, pitch_m, h_rows, gain_rows),
                            gamma)

    @property
    def hi_db(self) -> float:
        return self.snapshot().hi_db
