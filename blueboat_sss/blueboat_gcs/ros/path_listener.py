"""Planned mission path listener.

Subscribes to ``nav_msgs/Path`` on ``topics.planned_path`` (default
``/set_path``) — the exact message ``path_publisher.py`` publishes for
RViz at 1 Hz. The publisher re-sends the same saved path every period
(~4700 poses on a full mission), so an unchanged path is detected by a
cheap fingerprint and dropped before the ~0.5 MB tuple rebuild and the
map repaint it would trigger.

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


def path_fingerprint(poses) -> tuple:
    """Cheap identity for a pose sequence: length + first/last positions.

    Enough to tell the publisher's verbatim 1 Hz re-sends apart from a
    genuinely new path (a new mission changes at least one endpoint or
    the pose count) without touching all ~4700 poses.
    """
    if not poses:
        return (0,)
    first = poses[0].pose.position
    last = poses[-1].pose.position
    return (len(poses), first.x, first.y, last.x, last.y)


class PathListener:
    """nav_msgs/Path -> models.path.PlannedPath on the signal bus."""

    def __init__(self, node: "Node", signals: AppSignals,
                 topic: str) -> None:
        self._signals = signals
        self._last_fingerprint: tuple = ()
        if not _MSGS_AVAILABLE:
            signals.status_message.emit(
                "nav_msgs not available — planned path display disabled.")
            return
        node.create_subscription(Path, topic, self._on_path, 10)
        node.get_logger().info(f"Planned path listener on {topic}")

    def _on_path(self, msg: "Path") -> None:
        fingerprint = path_fingerprint(msg.poses)
        if fingerprint == self._last_fingerprint:
            return
        self._last_fingerprint = fingerprint
        points = tuple((p.pose.position.x, p.pose.position.y)
                       for p in msg.poses)
        self._signals.planned_path.emit(
            PlannedPath(t=time.time(), points=points))
