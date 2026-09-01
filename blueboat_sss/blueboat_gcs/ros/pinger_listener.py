"""USBL pinger position listener (real interface, no longer a placeholder).

    Topic   : config ``topics.pinger``
              (default ``/blueboat/pinger_coordinates``)
    Type    : std_msgs/Float32MultiArray
    Payload : two shapes from the same producer — ``[x, y, z]`` body
              frame (normal path) or ``[x, y]`` world frame
              (``fixed_pinger`` path); see ``utils/pinger.py``.

All gating (zero placeholder, NaN, malformed, frame disambiguation)
lives in the pure :func:`blueboat_gcs.utils.pinger.parse_pinger` so it
is testable without rclpy; this adapter only subscribes and forwards.
"""

from __future__ import annotations

import time

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from ..core.signals import AppSignals
from ..models.detection import PingerFix
from ..utils.pinger import parse_pinger

# Float32MultiArray carries no covariance; conservative display ring.
DEFAULT_ACCURACY_M = 2.0


class PingerListener:
    """Forwards genuine USBL pinger fixes to the GUI."""

    def __init__(self, node: Node, signals: AppSignals, topic: str) -> None:
        self._signals = signals
        node.create_subscription(Float32MultiArray, topic, self._on_msg, 10)
        node.get_logger().info(f"Pinger listener on {topic}")

    def _on_msg(self, msg: Float32MultiArray) -> None:
        parsed = parse_pinger(msg.data)
        if parsed is None:
            return
        x, y, frame = parsed
        self._signals.pinger_fix.emit(PingerFix(
            t=time.time(), x=x, y=y,
            accuracy_m=DEFAULT_ACCURACY_M, frame=frame))
