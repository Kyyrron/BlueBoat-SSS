"""Planned mission path listener.

Subscribes to ``nav_msgs/Path`` on ``topics.planned_path`` (default
``/set_path``) — the exact message ``path_publisher.py`` publishes for
RViz at 1 Hz (it re-publishes the same saved path; the GUI just replaces
its displayed path each time, so repeated messages are free).

Frame: the poses come from the path-generation service in the local odom
frame (the same frame RViz displays them in), so no conversion is
needed. Only ``pose.position.x/.y`` are used.
"""

from __future__ import annotations

import time

from ..core.signals import AppSignals
from ..models.path import PlannedPath

try:
    from nav_msgs.msg import Path  # noqa: F401
    from rclpy.node import Node
    _MSGS_AVAILABLE = True
except ImportError:                                # pragma: no cover
    _MSGS_AVAILABLE = False


class PathListener:
    """nav_msgs/Path -> models.path.PlannedPath on the signal bus."""

    def __init__(self, node: "Node", signals: AppSignals,
                 topic: str) -> None:
        self._signals = signals
        if not _MSGS_AVAILABLE:
            signals.status_message.emit(
                "nav_msgs not available — planned path display disabled.")
            return
        node.create_subscription(Path, topic, self._on_path, 10)
        node.get_logger().info(f"Planned path listener on {topic}")

    def _on_path(self, msg: "Path") -> None:
        points = tuple((p.pose.position.x, p.pose.position.y)
                       for p in msg.poses)
        self._signals.planned_path.emit(
            PlannedPath(t=time.time(), points=points))
