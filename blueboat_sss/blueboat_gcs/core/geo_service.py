"""GPS anchor state for the map (the GCS analogue of the MCS data store).

Owns the :class:`~blueboat_gcs.mapping.geo.GeoReferencer` and enforces
the pairing policy from ``GPS_MAP_ARCHITECTURE.md``:

* pair in the **GPS** callback, at GPS rate, with a freshness guard on
  the (faster) odom stream — pairing at odom rate re-uses one fix
  against several odom positions and biases the estimate;
* both sides of the freshness guard use the **wall clock**
  (``time.monotonic()``): message stamps may be sim time or replayed;
* the ``(0, 0)`` no-fix sentinel is rejected inside ``add_pair``.

Runs entirely in the GUI thread (fed by queued signals). Everything the
GUI needs of the anchor goes through here: readiness, the scene
translation, and the world <-> GPS conversions (all None-safe, like the
old ``CoordinateConverter`` they replace).
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

from PySide6.QtCore import QObject, Signal

from ..config.settings import GeoConfig
from ..mapping.geo import GeoFit, GeoReferencer

#: An odom position older than this is not paired with a GPS fix.
ODOM_FRESH_S = 0.5


class GeoService(QObject):
    """Single owner of the odom<->GPS anchor state."""

    #: A (valid) fit was created or updated — reposition the world root.
    fit_changed = Signal(object)               # GeoFit
    #: The anchor became valid for the first time: (lat0, lon0).
    anchored = Signal(float, float)
    #: GPS fixes are arriving but cannot pair (no fresh odom). Emitted
    #: once per blockage episode so the console explains a map stuck on
    #: "Waiting for GPS fix" while fixes are visibly flowing.
    pairing_blocked = Signal(str)

    def __init__(self, cfg: GeoConfig, require_anchor: bool = True) -> None:
        super().__init__()
        self._cfg = cfg
        self._require_anchor = require_anchor
        self._referencer = GeoReferencer(cfg)
        self._fixed_fit: Optional[GeoFit] = None
        self._last_odom: Optional[Tuple[float, float, float]] = None
        self._was_ready = False
        self._block_logged = False

    # ---- ingestion (GUI thread, queued signals) ---------------------------
    def on_robot_state(self, state) -> None:
        """Record the freshest odom/world position for pairing."""
        self._last_odom = (time.monotonic(), float(state.x), float(state.y))

    def on_gps_fix(self, t_wall: float, lat: float, lon: float) -> None:
        """One accepted GPS fix, at GPS rate (see module docstring)."""
        if self._fixed_fit is not None:
            return
        odom = self._last_odom
        if odom is None or t_wall - odom[0] > ODOM_FRESH_S:
            if not self._block_logged:
                self._block_logged = True
                self.pairing_blocked.emit(
                    "GPS fixes are arriving but cannot anchor the map: "
                    + ("no /blueboat/odom position received yet."
                       if odom is None else
                       f"the latest odom position is {t_wall - odom[0]:.1f} s "
                       f"old (> {ODOM_FRESH_S:.1f} s)."))
            return
        self._block_logged = False
        before = self._referencer.fit
        self._referencer.add_pair(t_wall, odom[1], odom[2], lat, lon)
        fit = self.fit
        if fit is not None and fit is not before:
            self.fit_changed.emit(fit)
        if fit is not None and not self._was_ready:
            self._was_ready = True
            self.anchored.emit(fit.lat0, fit.lon0)

    def set_fixed_fit(self, lat0: float, lon0: float,
                      world_x0: float = 0.0, world_y0: float = 0.0) -> None:
        """Anchor from a known origin instead of the online estimate.

        Replay: the log records the geographic position ``(lat0, lon0)``
        of the world point ``(world_x0, world_y0)``, so
        ``t = -world_origin`` exactly (the MCS designer-map trick).
        """
        self._fixed_fit = GeoFit(tx=-float(world_x0), ty=-float(world_y0),
                                 lat0=float(lat0), lon0=float(lon0),
                                 rms_m=0.0, n_pairs=0)
        self._was_ready = True
        self.fit_changed.emit(self._fixed_fit)
        self.anchored.emit(float(lat0), float(lon0))

    def reset(self) -> None:
        """Forget the anchor (new robot-side world origin expected)."""
        self._referencer = GeoReferencer(self._cfg)
        self._fixed_fit = None
        self._was_ready = False

    # ---- state ------------------------------------------------------------
    @property
    def fit(self) -> Optional[GeoFit]:
        if self._fixed_fit is not None:
            return self._fixed_fit
        return self._referencer.fit if self._referencer.is_valid else None

    @property
    def ready(self) -> bool:
        """Gate for the whole map (MCS ``map_frame_ready``)."""
        return self.fit is not None or not self._require_anchor

    @property
    def origin(self) -> Optional[Tuple[float, float]]:
        """(lat0, lon0) once anchored — the tile layer's georeference."""
        fit = self.fit
        return None if fit is None else (fit.lat0, fit.lon0)

    @property
    def translation(self) -> Tuple[float, float]:
        """(tx, ty): EN = world + t. Identity until anchored."""
        fit = self.fit
        return (0.0, 0.0) if fit is None else (fit.tx, fit.ty)

    @property
    def rms_m(self) -> Optional[float]:
        fit = self.fit
        return None if fit is None else fit.rms_m

    # ---- conversions (None-safe, exact inverses of one another) -----------
    def local_to_gps(self, x: float, y: float
                     ) -> Optional[Tuple[float, float]]:
        """World/odom metres -> (lat, lon); None until anchored."""
        fit = self.fit
        return None if fit is None else fit.world_to_latlon(x, y)

    def gps_to_local(self, lat: float, lon: float
                     ) -> Optional[Tuple[float, float]]:
        fit = self.fit
        return None if fit is None else fit.latlon_to_world(lat, lon)

    def world_to_en(self, x: float, y: float) -> Tuple[float, float]:
        fit = self.fit
        return (x, y) if fit is None else fit.world_to_enu(x, y)

    def en_to_world(self, east: float, north: float) -> Tuple[float, float]:
        fit = self.fit
        return (east, north) if fit is None else fit.enu_to_world(east, north)
