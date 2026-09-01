"""Online georeferencing between the ROS local-ENU world frame and GPS.

Ported verbatim from BlueBoat-MCS (``mcs/core/geo.py``), which solved the
GPS-anchored map for this project (see its ``GPS_MAP_ARCHITECTURE.md``).
The web-mercator tile helpers are omitted — the GCS already has them in
``mapping/tiles.py``.

``robot_interface.py`` publishes ``/blueboat/odom`` in a **local-ENU** frame:
position translated so the origin is the boat's launch point, axes East/North,
yaw absolute ENU (0 = East, CCW+). The simulator publishes the same frame kind
(Gazebo world, ENU). The only unknown between that frame and GPS is therefore a
**pure translation** — the EN position of the world origin — which the station
estimates online by pairing odom positions with GPS fixes.

There is deliberately **no rotation estimation** here. The previous MCS design
fitted a rotation with the Kabsch algorithm; against an ENU-axis odom frame the
fitted angle is ~0 by construction, the fit needed vehicle motion to converge
(deadlocking GPS-anchored deployments that hold position until deployed), and
its rolling-window refits made the whole scene wander. A translation estimate
converges from the first GPS fixes, with no motion required.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config.settings import GeoConfig
from ..utils.series import TimeSeries

EARTH_RADIUS_M = 6378137.0


# ----------------------------------------------------------------- lat/lon
def latlon_to_local_en(lat: float, lon: float,
                       lat0: float, lon0: float) -> tuple[float, float]:
    """Equirectangular projection of (lat, lon) to metres east/north of (lat0, lon0)."""
    east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * EARTH_RADIUS_M
    return east, north


def local_en_to_latlon(east: float, north: float,
                       lat0: float, lon0: float) -> tuple[float, float]:
    lat = lat0 + math.degrees(north / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


# ------------------------------------------------------------- georeferencer
@dataclass
class GeoFit:
    """Translation between the robot's local-ENU world frame and GPS.

    The one convention, defined here and nowhere else::

        EN = world + t          world = EN - t          t = (tx, ty)

    where ``EN`` are metres east/north of ``(lat0, lon0)`` (equirectangular)
    and ``t`` is the EN position of the world frame's origin — the boat's
    launch point. There is **no rotation**: the world frame's axes are already
    East/North (see the module docstring), so world→EN and EN→world are pure
    translations and a world yaw *is* a true ENU heading.

    ``rms_m`` is the residual of the window pairs against ``t`` — the health
    figure shown to the operator; ``n_pairs`` how many pairs support it.
    """

    tx: float             # EN east of the world origin
    ty: float             # EN north of the world origin
    lat0: float           # projection origin (first GPS fix)
    lon0: float
    rms_m: float          # residual of pairs against t
    n_pairs: int

    def world_to_enu(self, x: float, y: float) -> tuple[float, float]:
        """World point -> local east/north metres. ``EN = world + t``."""
        return x + self.tx, y + self.ty

    def enu_to_world(self, east: float, north: float) -> tuple[float, float]:
        """Local east/north metres -> world point. ``world = EN - t``."""
        return east - self.tx, north - self.ty

    # world <-> lat/lon
    def world_to_latlon(self, x: float, y: float) -> tuple[float, float]:
        return local_en_to_latlon(x + self.tx, y + self.ty, self.lat0, self.lon0)

    def latlon_to_world(self, lat: float, lon: float) -> tuple[float, float]:
        east, north = latlon_to_local_en(lat, lon, self.lat0, self.lon0)
        return east - self.tx, north - self.ty


class GeoReferencer:
    """Accumulates (world XY, GPS) pairs and maintains the translation estimate.

    ``add_pair`` must be fed at **GPS rate** with a fresh odom position (the
    caller guards staleness) — pairing every odom message with a possibly
    stale GPS fix biases the estimate. ``t`` is the per-axis **median** of
    ``EN - world`` over the rolling window, so a single GPS glitch cannot
    drag the map; the RMS residual against ``t`` is the health figure.
    """

    def __init__(self, cfg: GeoConfig) -> None:
        self._cfg = cfg
        self._pairs = TimeSeries(dim=4)  # x, y, east, north
        self._lat0: float | None = None
        self._lon0: float | None = None
        self._fit: GeoFit | None = None
        self._last_fit_t: float = -1e18

    @property
    def fit(self) -> GeoFit | None:
        return self._fit

    @property
    def is_valid(self) -> bool:
        """True once enough pairs agree on the translation.

        ``min_pairs`` is small (a couple of seconds of GPS): no vehicle
        motion is needed for a translation to be observable, so the map
        anchors almost immediately after the first fixes."""
        return (
            self._fit is not None
            and self._fit.n_pairs >= self._cfg.min_pairs
            and self._fit.rms_m <= self._cfg.max_residual_m
        )

    def add_pair(self, t: float, x: float, y: float,
                 lat: float, lon: float) -> None:
        """Feed a GPS fix with the concurrent odom position (called at GPS rate)."""
        if lat == 0.0 and lon == 0.0:  # NavSatFix with no fix
            return
        if self._lat0 is None:
            self._lat0, self._lon0 = lat, lon
        east, north = latlon_to_local_en(lat, lon, self._lat0, self._lon0)
        self._pairs.append(t, (x, y, east, north))
        # Refit on every pair until the estimate is supported, then throttle.
        supported = (self._fit is not None
                     and self._fit.n_pairs >= self._cfg.min_pairs)
        if not supported or t - self._last_fit_t >= self._cfg.refit_period_s:
            self._last_fit_t = t
            self._refit(t)

    # ------------------------------------------------------------- internal
    def _refit(self, now: float) -> None:
        ts, vs = self._pairs.window(now - self._cfg.fit_window_s, now + 1.0)
        if len(ts) == 0:
            return
        offsets = vs[:, 2:4] - vs[:, 0:2]          # EN - world, per pair
        t_est = np.median(offsets, axis=0)          # robust to GPS glitches
        residual = offsets - t_est
        err = np.hypot(residual[:, 0], residual[:, 1])
        # Robust spread (MAD-scaled): a single glitch in the window must not
        # inflate the health figure and hide the map; only a *sustained*
        # inconsistency between odom and GPS pushes it past max_residual_m.
        rms = float(np.median(err) * 1.4826)
        assert self._lat0 is not None and self._lon0 is not None
        self._fit = GeoFit(
            tx=float(t_est[0]), ty=float(t_est[1]),
            lat0=self._lat0, lon0=self._lon0, rms_m=rms, n_pairs=len(ts),
        )
