"""Local odom frame <-> WGS-84 conversion.

Strategy (see docs/ARCHITECTURE.md, "GPS strategy"):

* The SSS pipeline works entirely in the local odom frame (metres); GPS
  is *not* carried per pixel. The converter binds one origin — the first
  time both a local pose and a GPS fix are available it records
  ``(x0, y0) <-> (lat0, lon0)`` — and thereafter converts on demand
  (clicked points, tile placement, robot info panel). This avoids the
  per-pixel GPS computation the specification warns against.
* mavros publishes the local position in ENU, so the default local->ENU
  rotation is identity. If the odom frame is heading-aligned at boot
  instead, set ``map.frame_yaw_offset_deg`` in the config (this replaces
  the ad-hoc ``local_to_enu(yaw0)`` of the old ``math_helper``).
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from ..utils import geodesy


class CoordinateConverter:
    """Bidirectional local<->GPS converter. GUI-thread only."""

    def __init__(self, frame_yaw_offset_deg: float = 0.0) -> None:
        self._theta = math.radians(frame_yaw_offset_deg)
        self._lat0: Optional[float] = None
        self._lon0: Optional[float] = None
        self._x0: float = 0.0
        self._y0: float = 0.0

    # ---- origin binding -----------------------------------------------------
    @property
    def ready(self) -> bool:
        return self._lat0 is not None

    @property
    def origin(self) -> Optional[Tuple[float, float]]:
        if self._lat0 is None or self._lon0 is None:
            return None
        return self._lat0, self._lon0

    def bind_origin(self, lat: float, lon: float, x: float, y: float) -> None:
        """Anchor the frames: local (x, y) corresponds to GPS (lat, lon)."""
        self._lat0, self._lon0 = lat, lon
        self._x0, self._y0 = x, y

    # ---- conversions ---------------------------------------------------------
    def _rot(self, x: float, y: float, sign: float) -> Tuple[float, float]:
        c, s = math.cos(sign * self._theta), math.sin(sign * self._theta)
        return c * x - s * y, s * x + c * y

    def local_to_gps(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        if not self.ready:
            return None
        east, north = self._rot(x - self._x0, y - self._y0, +1.0)
        return geodesy.enu_to_gps(self._lat0, self._lon0, east, north)

    def gps_to_local(self, lat: float, lon: float) -> Optional[Tuple[float, float]]:
        if not self.ready:
            return None
        east, north = geodesy.gps_to_enu(self._lat0, self._lon0, lat, lon)
        x, y = self._rot(east, north, -1.0)
        return x + self._x0, y + self._y0
