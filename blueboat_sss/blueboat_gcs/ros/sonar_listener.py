"""Subscriber for the processed SSS stream.

Topic     : config ``topics.processed_ping`` (default /sss_processor/processed)
Type      : blueboat_interfaces/ProcessedSSSPing
Rate      : ~28 Hz (one merged port+starboard ping)
Publisher : sss_processor_node.py (existing repository, unchanged)

This is the ROS/Qt boundary for sonar data: the message is converted to
the ROS-free ``SonarPing`` dataclass here (same fields the old
``processed_sss_listener._on_processed_ping`` consumed) and emitted on the
signal bus. Everything downstream — mosaic, renderer, GUI — is ROS-free.
"""

from __future__ import annotations

import numpy as np
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ..core.signals import AppSignals
from ..models.sonar import SonarPing
from ..utils.geodesy import quat_to_yaw

try:  # pragma: no cover - environment dependent
    from blueboat_interfaces.msg import ProcessedSSSPing
    INTERFACES_AVAILABLE = True
except ImportError:  # pragma: no cover
    INTERFACES_AVAILABLE = False


class SonarListener:
    """Converts ProcessedSSSPing messages into SonarPing signals."""

    def __init__(self, node: Node, signals: AppSignals, topic: str) -> None:
        self._signals = signals
        if not INTERFACES_AVAILABLE:
            signals.status_message.emit(
                "blueboat_interfaces not found — sonar stream disabled.")
            return
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        node.create_subscription(ProcessedSSSPing, topic, self._on_msg, qos)

    def _on_msg(self, msg: "ProcessedSSSPing") -> None:
        # Same merging convention as the legacy listener: the sign of
        # y_local encodes the side (port=+, stbd=-), so one array suffices.
        y_local = np.concatenate([
            np.asarray(msg.port_y, dtype=np.float64),
            np.asarray(msg.starboard_y, dtype=np.float64),
        ])
        intensity = np.concatenate([
            np.asarray(msg.port_intensity_db, dtype=np.float32),
            np.asarray(msg.starboard_intensity_db, dtype=np.float32),
        ])
        q = msg.robot_orientation
        ping = SonarPing(
            t=msg.port_stamp.sec + msg.port_stamp.nanosec * 1e-9,
            robot_x=float(msg.robot_x),
            robot_y=float(msg.robot_y),
            yaw=quat_to_yaw(q.x, q.y, q.z, q.w),
            water_depth=float(msg.water_depth),
            y_local=y_local,
            intensity_db=intensity,
        )
        self._signals.sonar_ping.emit(ping)
