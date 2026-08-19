"""Geodesy and orientation helpers.

The math is **reused verbatim from the existing ``math_helper.py``**
(``enu_to_gps``, yaw extraction) but re-expressed on plain floats so the
mapping layer carries no dependency on ROS message types. Equirectangular
approximation around the origin — sub-decimetre accurate over the few
hundred metres of a harbour survey, and consistent in both directions.
"""

from __future__ import annotations

import math
from typing import Tuple

EARTH_RADIUS_M: float = 6378137.0  # WGS-84 equatorial radius (as in math_helper)


def enu_to_gps(lat0_deg: float, lon0_deg: float,
               east: float, north: float) -> Tuple[float, float]:
    """Offset (east, north) metres from (lat0, lon0) -> (lat, lon) degrees."""
    lat0 = math.radians(lat0_deg)
    dlat = north / EARTH_RADIUS_M
    dlon = east / (EARTH_RADIUS_M * math.cos(lat0))
    return lat0_deg + math.degrees(dlat), lon0_deg + math.degrees(dlon)


def gps_to_enu(lat0_deg: float, lon0_deg: float,
               lat_deg: float, lon_deg: float) -> Tuple[float, float]:
    """Inverse of :func:`enu_to_gps` (same linearisation, so exact inverse)."""
    lat0 = math.radians(lat0_deg)
    north = math.radians(lat_deg - lat0_deg) * EARTH_RADIUS_M
    east = math.radians(lon_deg - lon0_deg) * EARTH_RADIUS_M * math.cos(lat0)
    return east, north


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw (Z rotation, rad) from a quaternion — same formula as
    ``math_helper.quaternion_to_yaw`` but on plain floats."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_compass_deg(yaw_rad: float) -> float:
    """ENU yaw (CCW from East) -> compass heading (CW from North, 0..360)."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


def format_latlon(lat: float, lon: float) -> str:
    """Uniform GPS display format used everywhere in the GUI."""
    return f"{lat:.7f}, {lon:.7f}"
