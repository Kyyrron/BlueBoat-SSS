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

#: Ping poses within this of the frozen reference count as "pinned".
FROZEN_EPS_M = 0.05
#: Consecutive pinned pings before the policy engages.
FROZEN_AFTER_PINGS = 20
#: The boat must have moved at least this far (per GCS telemetry) while
#: the ping pose sat still for the freeze to count as pathological.
MOVED_MIN_M = 1.0
#: Consecutive genuinely-moving pings before an engaged detector lets
#: go. Hysteresis: the old instant-disengage flapped whenever the pose
#: froze anywhere but the exact origin.
RELEASE_AFTER_PINGS = 5


class FrozenPoseDetector:
    """Detects ping poses pathologically pinned at a constant.

    Field history: the first version only recognized a freeze at (0, 0).
    The processor's tolerance-free nearest-stamp lookup can just as well
    latch an arbitrary stale sample — the second-session-of-the-day
    variant of the same bug — which the origin test not only missed but
    actively *disengaged* on. The generalized rule is: the ping pose has
    not moved (within ``eps_m``) for ``after`` consecutive pings while
    the GCS's own telemetry says the boat travelled more than
    ``moved_min_m`` in the same span. State survives nothing: call
    :meth:`reset` on every acquisition START.
    """

    def __init__(self, eps_m: float = FROZEN_EPS_M,
                 after: int = FROZEN_AFTER_PINGS,
                 moved_min_m: float = MOVED_MIN_M) -> None:
        self._eps = eps_m
        self._after = after
        self._moved_min = moved_min_m
        self.reset()

    def reset(self) -> None:
        """Forget everything (new mission / acquisition START)."""
        self._streak = 0
        self._release = 0
        self._ref_ping: Optional[Tuple[float, float]] = None
        self._ref_robot: Optional[Tuple[float, float]] = None
        self.engaged = False

    def update(self, ping_x: float, ping_y: float,
               robot_x: Optional[float], robot_y: Optional[float]) -> bool:
        """Feed one ping pose + the current GCS robot pose; returns True
        while re-stamping should be applied."""
        if self._ref_ping is None:
            self._latch(ping_x, ping_y, robot_x, robot_y)
            return self.engaged
        pinned = (abs(ping_x - self._ref_ping[0]) < self._eps
                  and abs(ping_y - self._ref_ping[1]) < self._eps)
        if not pinned:
            # The source pose moved. Re-latch so a freeze at a *new*
            # constant is caught, and only release an engaged detector
            # after several consecutive moving pings (hysteresis).
            self._latch(ping_x, ping_y, robot_x, robot_y)
            self._streak = 0
            if self.engaged:
                self._release += 1
                if self._release >= RELEASE_AFTER_PINGS:
                    self.engaged = False
                    self._release = 0
            return self.engaged
        self._release = 0
        if self._ref_robot is None and robot_x is not None:
            # Telemetry appeared after the candidate started: measure
            # boat displacement from here on.
            self._ref_robot = (robot_x, robot_y)
        moved = (robot_x is not None and self._ref_robot is not None
                 and math.hypot(robot_x - self._ref_robot[0],
                                robot_y - self._ref_robot[1])
                 > self._moved_min)
        # The classic pathology — pinned at the origin with the boat far
        # from it — engages on absolute distance too, so it is caught
        # even when telemetry only appeared once the boat was already
        # out on the survey line.
        origin_case = (math.hypot(*self._ref_ping) < self._eps
                       and robot_x is not None
                       and math.hypot(robot_x, robot_y) > self._moved_min)
        if moved or origin_case:
            self._streak += 1
            if self._streak >= self._after:
                self.engaged = True
        return self.engaged

    def _latch(self, ping_x: float, ping_y: float,
               robot_x: Optional[float], robot_y: Optional[float]) -> None:
        self._ref_ping = (ping_x, ping_y)
        self._ref_robot = (None if robot_x is None
                           else (robot_x, robot_y))


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


class HeadingPolicy:
    """Live heading source: compass first, odom yaw as fallback.

    The MCS rule (``GPS_MAP_ARCHITECTURE.md`` §heading): the compass is
    the true-north reference and is converted **once**, at ingestion —
    ``θ = wrap(radians(90 − hdg_deg))`` — never corrected downstream;
    odom yaw (absolute ENU since the 2026-08-31 robot-side fix) covers a
    silent compass. This is what fixed the field's misaligned live SSS
    pings; replay derives yaw from mavlink ATTITUDE and never comes
    through here.

    Timestamps are wall-clock (``time.monotonic()``) on both update
    paths so staleness is immune to sim time and replayed stamps.
    """

    def __init__(self, compass_stale_s: float = 2.0) -> None:
        self._stale_s = compass_stale_s
        self._compass: Optional[Tuple[float, float]] = None   # t, θ ENU rad
        self._odom: Optional[Tuple[float, float]] = None      # t, yaw

    def update_compass(self, t_wall: float, heading_deg: float) -> None:
        a = math.radians(90.0 - heading_deg)
        self._compass = (t_wall, math.atan2(math.sin(a), math.cos(a)))

    def update_odom(self, t_wall: float, yaw: float) -> None:
        self._odom = (t_wall, yaw)

    def heading(self, now: float) -> Optional[float]:
        """ENU yaw [rad] to draw with, or None when nothing is fresh."""
        if (self._compass is not None
                and now - self._compass[0] <= self._stale_s):
            return self._compass[1]
        if self._odom is not None:
            return self._odom[1]
        return None


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
