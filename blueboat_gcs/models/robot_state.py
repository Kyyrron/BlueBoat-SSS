"""Aggregated robot telemetry shown in the left panel and map."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True, slots=True)
class RobotState:
    """Snapshot of the robot state at time ``t`` (seconds, ROS time).

    ``x``/``y``/``yaw`` are in the local odom frame (metres / radians).
    ``lat``/``lon`` are WGS-84 degrees, ``None`` until a GPS fix exists.
    ``heading_deg`` is the compass heading (0..360, clockwise from North)
    when available from mavros, otherwise derived from local yaw once the
    coordinate converter knows the frame orientation.
    ``speed_mps`` is ground speed.
    """

    t: float
    x: float
    y: float
    yaw: float
    lat: Optional[float] = None
    lon: Optional[float] = None
    heading_deg: Optional[float] = None
    speed_mps: Optional[float] = None
