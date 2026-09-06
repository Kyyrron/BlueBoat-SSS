"""ROS-free representation of one processed side-scan ping.

`ros/sonar_listener.py` converts `blueboat_interfaces/ProcessedSSSPing`
into this dataclass at the ROS/Qt boundary, so that everything past the
signal bus (mosaic, renderer, GUI, simulator) has *zero* dependency on
ROS message types. This is what makes the whole GUI testable on a laptop
with `--sim` and no ROS installation.

This module also owns the **native slant-bin row model** shared by the
waterfall service and the seabed imager: the raw acquisition domain is
one column per device range bin (exactly how SonarView draws its
waterfall), so a row is continuous by construction — no resampling, no
interpolated values, no empty-column holes. See :func:`native_bins` and
:func:`build_slant_row`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True, slots=True)
class SonarPing:
    """One merged port+starboard ping, already slant-range corrected.

    Attributes
    ----------
    t:
        Ping timestamp in seconds (ROS time of the port packet).
    robot_x, robot_y:
        Robot position in the local odom frame [m] snapped at ping time.
    yaw:
        Robot heading in the local frame [rad], REP-103 (CCW from +x).
    water_depth:
        Estimated water depth under the boat [m] (FBR altitude + draft).
    y_local:
        Lateral sample coordinates in base_link [m]; +y = port,
        -y = starboard (concatenation of the two sides).
    intensity_db:
        Per-sample intensity [dB], aligned with ``y_local``.
    slant_range_m:
        The sonar's *configured* range for this ping [m] (``length_mm``
        / 1000). Optional, 0.0 when unknown. This is the geometrically
        stable across-track extent: unlike ``max|y_local|`` it does not
        move when the altitude estimate wobbles, so the waterfall uses
        it to keep a fixed column scale (a wandering altitude estimate
        otherwise rescales every row and makes the image ripple).
    sides:
        Which sides this ping actually carries: "both", "port" or
        "starboard". Pings are never dropped just because one side is
        missing, so consumers may see one-sided rows.
    gap_before:
        Device pings lost immediately before this one (a jump in the
        device's own ping_number, normalised across the two units).
        Zero on a contiguous stream. The waterfall renders each lost
        ping as a blank line so genuine acquisition/QoS loss is visible
        instead of silently stitching non-adjacent pings together.
    bin_size_m / port_bin0 / port_db / stbd_bin0 / stbd_db:
        Optional **native slant-bin** payload — the device's own range
        bins, verbatim dB, before any slant-range projection.
        ``*_db[k]`` is the sample of device bin ``*_bin0 + k`` (bins are
        contiguous once the water column / nadir blank has been cut
        upstream, so one offset suffices); device bin ``b`` sits at
        slant range ``b * bin_size_m``. Producers that hold the raw
        profile (the svlog replay decoder, the live listener via the
        exact inverse ``s = sqrt(y² + depth²)``, the bench simulator)
        fill these; when they are absent :func:`native_bins` falls back
        to reconstructing them from ``y_local``/``water_depth``.
    bottom_slant_m:
        The FBR-tracked bottom slant range [m], **independent of the
        Depth comp. mode** (the tracker is advanced in every mode, so
        this carries a bottom estimate even in ``off``/``manual`` where
        ``water_depth`` is 0). 0.0 when no bottom has been detected. It
        does not affect geometry — it is used only to split each ping
        into its water-column band (slant < bottom) and its seabed return
        (slant >= bottom) for the display contrast window, so the nadir
        maps toward black without erasing any sample.
    """

    t: float
    robot_x: float
    robot_y: float
    yaw: float
    water_depth: float
    y_local: np.ndarray  # float64, shape (N,)
    intensity_db: np.ndarray  # float32, shape (N,)
    slant_range_m: float = 0.0
    sides: str = "both"
    gap_before: int = 0
    bin_size_m: float = 0.0
    port_bin0: int = 0
    port_db: Optional[np.ndarray] = None    # float32, native port bins
    stbd_bin0: int = 0
    stbd_db: Optional[np.ndarray] = None    # float32, native stbd bins
    bottom_slant_m: float = 0.0             # tracked bottom, any depth mode
    #: Starboard native bin pitch [m] when it differs from ``bin_size_m``
    #: (the two units are configured independently); 0 = same as port.
    stbd_bin_size_m: float = 0.0
    #: Device receiver gain index of the ping (-1 = unknown). Consumers of
    #: the display model treat a change as a break (the absolute dB scale
    #: moves with the gain ladder).
    gain_index: int = -1
    #: Monotonic arrival counter stamped by the live listener (0 on replay
    #: / synthetic pings): the GUI compares it with the listener's newest
    #: to measure how far behind the queue is and throttle rendering.
    seq: int = 0


@dataclass(frozen=True, slots=True)
class NativeBins:
    """One ping in the native slant-bin domain (both sides).

    ``bin_size_m`` is the **layout pitch**: the finest pitch among the
    sides present. A side whose own pitch is coarser (``port_size_m`` /
    ``stbd_size_m``) paints by pure pixel stretch in
    :func:`build_slant_row`, so two units configured at different ranges
    still land on one row instead of one of them being dropped.
    """

    bin_size_m: float
    port_bin0: int
    port_db: np.ndarray                     # float32, may be empty
    stbd_bin0: int
    stbd_db: np.ndarray                     # float32, may be empty
    port_size_m: float = 0.0                # 0 = bin_size_m
    stbd_size_m: float = 0.0                # 0 = bin_size_m

    @property
    def port_pitch(self) -> float:
        return self.port_size_m if self.port_size_m > 0 else self.bin_size_m

    @property
    def stbd_pitch(self) -> float:
        return self.stbd_size_m if self.stbd_size_m > 0 else self.bin_size_m

    @property
    def extent_bins(self) -> int:
        """Largest slant extent across both sides, in layout-pitch
        columns (bin index + 1 when both sides share the pitch)."""
        if self.bin_size_m <= 0:
            return 0
        p = ((self.port_bin0 + self.port_db.size) * self.port_pitch
             if self.port_db.size else 0.0)
        s = ((self.stbd_bin0 + self.stbd_db.size) * self.stbd_pitch
             if self.stbd_db.size else 0.0)
        return int(np.ceil(max(p, s) / self.bin_size_m - 1e-9))


def native_from_profile(start_mm: float, length_mm: float,
                        num_results: int) -> Tuple[int, float]:
    """(first bin index, bin pitch [m]) of a device profile on the uniform
    slant grid ``slant_i = start + i * pitch``.

    The ONE definition both paths use — the replay loader from the packet
    header and the live listener from ``OmniscanProfile`` — so a range
    start offset (``range_start_mm``) lands the first sample on the same
    column on both. The pitch is ``length / (num_results - 1)`` (sample 0
    at ``start``, the last at ``start + length``), matching
    ``project_side``.
    """
    pitch = length_mm / 1000.0 / max(int(num_results) - 1, 1)
    if pitch <= 0.0:
        return 0, 0.0
    return int(round(start_mm / 1000.0 / pitch)), float(pitch)


def side_bins_from_ground(y: np.ndarray, v: np.ndarray, depth: float
                          ) -> Tuple[int, np.ndarray, float]:
    """Reconstruct (bin0, values, bin_size) for one side.

    The processor's ``project_side`` maps device bin ``i`` (slant
    ``i*Δ``) to ``ground = sqrt(slant² − h²)`` and drops bins below the
    cut, keeping the tail contiguous. ``s = sqrt(ground² + h²)`` is its
    exact inverse, so the recovered slant values fall back onto the
    uniform device grid and ``Δ`` is simply their median spacing.
    """
    if y.size < 2:
        return 0, np.asarray([], dtype=np.float32), 0.0
    order = np.argsort(np.abs(y))
    s = np.hypot(np.abs(y[order]), depth)
    d = np.diff(s)
    d = d[d > 0]
    if d.size == 0:
        return 0, np.asarray([], dtype=np.float32), 0.0
    delta = float(np.median(d))
    idx = np.rint(s / delta).astype(np.int64)
    vals = np.asarray(v, dtype=np.float32)[order]
    # On the uniform device grid (the processor keeps a contiguous bin
    # tail) idx is exactly [i0 .. i0+n-1]; a foreign producer that is not
    # on such a grid still lands here as a dense pack from its first
    # recovered bin — hole-free either way.
    return int(idx[0]), vals, delta


def native_bins(ping: SonarPing) -> Optional[NativeBins]:
    """The ping's native slant-bin payload, reconstructing it if absent."""
    empty = np.asarray([], dtype=np.float32)
    if ping.port_db is not None or ping.stbd_db is not None:
        if ping.bin_size_m <= 0.0:
            return None
        port_size = float(ping.bin_size_m)
        stbd_size = (float(ping.stbd_bin_size_m)
                     if ping.stbd_bin_size_m > 0.0 else port_size)
        has_p = ping.port_db is not None and len(ping.port_db) > 0
        has_s = ping.stbd_db is not None and len(ping.stbd_db) > 0
        sizes = ([port_size] if has_p else []) + ([stbd_size] if has_s else [])
        return NativeBins(
            bin_size_m=min(sizes) if sizes else port_size,
            port_bin0=int(ping.port_bin0),
            port_db=(np.asarray(ping.port_db, dtype=np.float32)
                     if ping.port_db is not None else empty),
            stbd_bin0=int(ping.stbd_bin0),
            stbd_db=(np.asarray(ping.stbd_db, dtype=np.float32)
                     if ping.stbd_db is not None else empty),
            port_size_m=port_size, stbd_size_m=stbd_size,
        )
    y, v = ping.y_local, ping.intensity_db
    depth = max(float(ping.water_depth), 0.0)
    p0, pv, pd = side_bins_from_ground(y[y > 0], v[y > 0], depth)
    s0, sv, sd = side_bins_from_ground(-y[y < 0], v[y < 0], depth)
    deltas = [d for d in (pd, sd) if d > 0]
    if not deltas:
        return None
    return NativeBins(bin_size_m=float(np.mean(deltas)),
                      port_bin0=p0, port_db=pv, stbd_bin0=s0, stbd_db=sv)


def fill_side_segment(seg: np.ndarray, bin0: int, values: np.ndarray,
                      bin_size_m: float, pitch_m: float) -> None:
    """Paint one side's native bins into ``seg`` (length = half columns).

    Column ``k`` covers slant ``[k·pitch, (k+1)·pitch)``. Each device bin
    ``b`` (slant ``b·Δ``) paints every column its interval overlaps —
    verbatim value repetition (a pure pixel stretch), never invented
    samples, and gap-free by construction whenever ``pitch <= Δ``. Bins
    below ``bin0`` (the water column / nadir blank cut upstream) leave
    their columns NaN: the rendered centre gap *is* the physical water
    column.
    """
    if values.size == 0 or bin_size_m <= 0 or pitch_m <= 0:
        return
    half = seg.shape[0]
    b = bin0 + np.arange(values.size + 1, dtype=np.float64)
    if abs(bin_size_m - pitch_m) <= 1e-9 * pitch_m:
        edges = b.astype(np.int64)          # native 1:1, immune to fp noise
    else:
        edges = np.floor(b * bin_size_m / pitch_m).astype(np.int64)
    edges = np.clip(edges, 0, half)
    counts = np.diff(edges)
    total = int(edges[-1] - edges[0])
    if total <= 0:
        return
    seg[edges[0]:edges[-1]] = np.repeat(values, counts)


def build_slant_row(nb: NativeBins, pitch_m: float, half_cols: int
                    ) -> np.ndarray:
    """One waterfall row (width ``2*half_cols``) from native bins.

    Layout: column ``half_cols - 1 - k`` = port slant column ``k``,
    column ``half_cols + k`` = starboard slant column ``k`` — port far
    range on the left edge, starboard far range on the right, transducer
    at the centre seam. NaN = no data (water column, beyond the ping's
    extent, or an absent side).
    """
    row = np.full(2 * half_cols, np.nan, dtype=np.float32)
    port = np.full(half_cols, np.nan, dtype=np.float32)
    stbd = np.full(half_cols, np.nan, dtype=np.float32)
    fill_side_segment(port, nb.port_bin0, nb.port_db, nb.port_pitch, pitch_m)
    fill_side_segment(stbd, nb.stbd_bin0, nb.stbd_db, nb.stbd_pitch, pitch_m)
    row[:half_cols] = port[::-1]
    row[half_cols:] = stbd
    return row


def slant_col_to_y_local(col, width: int, pitch_m: float, depth):
    """Waterfall column -> lateral offset in base_link [m] (+ = port).

    Inverse of the :func:`build_slant_row` layout: the column's slant
    range is ``(k + 0.5) * pitch`` and the ground range follows from the
    row's altitude, ``ground = sqrt(max(s² − depth², 0))`` — a column
    inside the water column maps to the nadir point. Accepts scalars or
    broadcasting arrays (per-pixel world grids of a seabed image).
    """
    col = np.asarray(col, dtype=np.float64)
    half = width / 2.0
    port = col < half
    k = np.where(port, half - 1.0 - col, col - half)
    s = (k + 0.5) * pitch_m
    ground = np.sqrt(np.maximum(s * s - np.square(depth), 0.0))
    return np.where(port, ground, -ground)
