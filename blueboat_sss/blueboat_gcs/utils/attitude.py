"""Latest-sample attitude for the seabed-image metadata (ROS-free).

The GCS records, per seabed-image row, the boat's roll, pitch and yaw
rate (owner's order Q8, 2026-10-15) -- **metadata only, never geometry**.
Live, ``ros/telemetry_listener.py`` turns each ``/mavros/imu/data``
sample into ``AppSignals.attitude``; :class:`AttitudeBuffer` keeps the
latest and attaches it to every ping at its reception, with its age --
the same pattern as the compass heading (``pose_alignment.HeadingPolicy``).

``/mavros/imu/data`` exists in both graphs (mavros on the boat, the
simulator's mavros shim in Gazebo). The ping's own quaternion is **not**
used: it is yaw-only on the real boat (``robot_interface``) and full in
simulation, so the field would mean different things in the two graphs.

Pairing is by arrival time, not by stamp: mavros header stamps may read 0
on the real launch (root ``CLAUDE.md`` §4.1). The age says how stale the
attached sample was; a sample older than ``max_age_s`` is not attached.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Optional, Tuple


def quat_to_roll_pitch(x: float, y: float, z: float, w: float) -> Tuple[float, float]:
    """(roll, pitch) [rad] of a quaternion (ZYX / aerospace sequence)."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


class AttitudeBuffer:
    def __init__(self, max_age_s: float = 2.0) -> None:
        self.max_age_s = float(max_age_s)
        self._last: Optional[Tuple[float, float, float, float]] = None

    def update(self, t_wall: float, roll: float, pitch: float, yaw_rate: float) -> None:
        self._last = (float(t_wall), float(roll), float(pitch), float(yaw_rate))

    def reset(self) -> None:
        self._last = None

    def sample(self, now: float) -> Optional[Tuple[float, float, float, float]]:
        """(roll, pitch, yaw_rate, age_s), or None when nothing fresh."""
        if self._last is None:
            return None
        t, roll, pitch, rate = self._last
        age = now - t
        if age > self.max_age_s or age < -1.0:
            return None
        return roll, pitch, rate, max(age, 0.0)

    def attach(self, ping, now: float):
        """The ping with the latest attitude (unchanged when it already
        carries one, e.g. a replayed ping, or when nothing is fresh)."""
        if math.isfinite(getattr(ping, "roll", math.nan)):
            return ping
        s = self.sample(now)
        if s is None:
            return ping
        return replace(ping, roll=s[0], pitch=s[1], yaw_rate=s[2], attitude_age_s=s[3])
