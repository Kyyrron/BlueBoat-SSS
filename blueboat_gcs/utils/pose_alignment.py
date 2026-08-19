"""Pose alignment helpers (pure, ROS-free, unit-testable).

Context (sea-trial bug): live ProcessedSSSPing poses can arrive frozen at
(0, 0) even with GPS on — the processor snaps its pose from
``/blueboat/odom``, and the live publisher of that topic either emits
zeros (EKF origin / robot_interface issue) or stamps on a different
clock than the sonar profiles, in which case the processor's
tolerance-free nearest-stamp lookup latches onto a boot-time zero
sample. Rosbag replays are immune because ``svlog_to_rosbag`` *synthesizes*
``/blueboat/odom`` on the same synthetic clock from real
LOCAL_POSITION_NED — which is exactly why "bag works, live doesn't".

Two GCS-side defenses (config block ``alignment``):

* :class:`FrozenPoseDetector` — recognizes the pathology: embedded ping
  poses pinned at the origin while the GCS's own telemetry shows the
  boat somewhere else. main_window then re-stamps pings with the
  time-nearest RobotState pose before they reach the mosaic/imager.
* :class:`GpsPoseSynthesizer` — dead-reckons a pose directly from
  NavSatFix + compass heading (first-fix ENU reference) when
  ``/blueboat/odom`` itself is silent or zero-frozen. GPS is on during
  the affected trials, so this always yields a usable world frame; the
  telemetry listener emits it as an ordinary RobotState, and origin
  binding / trajectory / re-stamped pings all align on the satellite map.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from .geodesy import gps_to_enu

#: |x| and |y| below this count as "at the origin".
FROZEN_EPS_M = 0.05
#: Consecutive frozen pings before the policy engages.
FROZEN_AFTER_PINGS = 20
#: The boat must be at least this far from the origin (per GCS telemetry)
#: for frozen ping poses to be considered pathological.
MOVED_MIN_M = 1.0


class FrozenPoseDetector:
    """Detects ping poses pathologically frozen at the origin."""

    def __init__(self, eps_m: float = FROZEN_EPS_M,
                 after: int = FROZEN_AFTER_PINGS,
                 moved_min_m: float = MOVED_MIN_M) -> None:
        self._eps = eps_m
        self._after = after
        self._moved_min = moved_min_m
        self._streak = 0
        self.engaged = False

    def update(self, ping_x: float, ping_y: float,
               robot_x: Optional[float], robot_y: Optional[float]) -> bool:
        """Feed one ping pose + the current GCS robot pose; returns True
        while re-stamping should be applied."""
        frozen = abs(ping_x) < self._eps and abs(ping_y) < self._eps
        moved = (robot_x is not None
                 and math.hypot(robot_x, robot_y) > self._moved_min)
        if frozen and moved:
            self._streak += 1
            if self._streak >= self._after:
                self.engaged = True
        elif not frozen:
            # Real poses are flowing again: disengage immediately.
            self._streak = 0
            self.engaged = False
        return self.engaged


class GpsPoseSynthesizer:
    """NavSatFix + compass heading -> local ENU pose (dead reckoning).

    The first fix becomes the local origin (0, 0); every later fix maps
    through the same equirectangular conversion the rest of the app uses,
    so a CoordinateConverter bound from these poses is self-consistent.
    Yaw comes from the compass (deg, clockwise from North) converted to
    ENU radians. Speed is estimated from consecutive fixes.
    """

    def __init__(self) -> None:
        self._ref: Optional[Tuple[float, float]] = None
        self._last: Optional[Tuple[float, float, float]] = None  # t, x, y

    @property
    def has_reference(self) -> bool:
        return self._ref is not None

    def update(self, t: float, lat: float, lon: float,
               compass_deg: Optional[float]
               ) -> Tuple[float, float, float, float]:
        """Returns (x, y, yaw, speed) in the local ENU frame."""
        if self._ref is None:
            self._ref = (lat, lon)
        x, y = gps_to_enu(self._ref[0], self._ref[1], lat, lon)
        yaw = (math.radians(90.0 - compass_deg)
               if compass_deg is not None else 0.0)
        speed = 0.0
        if self._last is not None:
            dt = t - self._last[0]
            if dt > 1e-3:
                speed = math.hypot(x - self._last[1], y - self._last[2]) / dt
        self._last = (t, x, y)
        return x, y, yaw, speed


def robot_to_world(px: float, py: float,
                   robot_x: float, robot_y: float, yaw: float
                   ) -> Tuple[float, float]:
    """Vehicle-frame point (FLU: x forward, y port/left) -> world frame.

    Used for USBL pinger fixes: a USBL natively reports positions
    relative to its transducer, so ``alignment.pinger_frame: robot``
    interprets [x, y] as body coordinates and rotates them through the
    robot pose nearest the fix.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return (robot_x + c * px - s * py,
            robot_y + s * px + c * py)
