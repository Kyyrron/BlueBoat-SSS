"""Planned mission path model (ROS-free)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class PlannedPath:
    """Planned mission path in the local odom frame.

    Produced by ros/path_listener.py from ``nav_msgs/Path`` on
    ``topics.planned_path`` (the same message ``path_publisher.py``
    sends to RViz). A new message fully replaces the previous path.
    """

    t: float                                  # reception time [s]
    points: Tuple[Tuple[float, float], ...]   # ((x, y), ...) [m]
