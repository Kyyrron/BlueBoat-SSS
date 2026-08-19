"""USBL pinger position listener (real interface, no longer a placeholder).

    Topic   : config ``topics.pinger``
              (default ``/blueboat/pinger_coordinates``)
    Type    : std_msgs/Float32MultiArray
    Payload : ``data = [x_world, y_world]`` — pinger position in the
              world/odom frame [m] (same frame as /blueboat/odom).

The GUI keeps only the latest fix (marker + info panel + live distance
to the robot), so any publish rate works. Extra array elements are
ignored; malformed messages (fewer than 2 values, NaN) are dropped.
"""

from __future__ import annotations

import math
import time

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from ..core.signals import AppSignals
from ..models.detection import PingerFix

# Float32MultiArray carries no covariance; conservative display ring.
DEFAULT_ACCURACY_M = 2.0


class PingerListener:
    """Forwards the last known USBL pinger fix to the GUI."""

    def __init__(self, node: Node, signals: AppSignals, topic: str) -> None:
        self._signals = signals
        node.create_subscription(Float32MultiArray, topic, self._on_msg, 10)
        node.get_logger().info(f"Pinger listener on {topic}")

    def _on_msg(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        x, y = float(msg.data[0]), float(msg.data[1])
        if math.isnan(x) or math.isnan(y):
            return
        self._signals.pinger_fix.emit(PingerFix(
            t=time.time(), x=x, y=y, accuracy_m=DEFAULT_ACCURACY_M))
