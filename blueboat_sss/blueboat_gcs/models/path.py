"""Planned mission path model (ROS-free)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class PlannedPath:
    """Planned mission path in the local odom frame.

    Produced by ros/path_listener.py from the path server's answer
    (``/path_request`` over the whole mission), or from ``nav_msgs/Path``
    on ``topics.planned_path`` as the fallback. A new path fully replaces
    the previous one; an empty one clears it.
    """

    t: float                                  # reception time [s]
    points: Tuple[Tuple[float, float], ...]   # ((x, y), ...) [m]
