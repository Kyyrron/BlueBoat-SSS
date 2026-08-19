"""ROS 2 lifecycle management.

Owns the single rclpy node of the application and spins it in a
background thread. Every listener registers its subscriptions on this
node; callbacks run in the ROS thread and must only emit ``AppSignals``
(Qt queues them to the GUI thread — see core/signals.py).

rclpy is imported lazily and defensively: on a machine without ROS the
application still starts (e.g. for ``--sim`` bench work or mosaic replay),
with a clear status message instead of an ImportError stack trace.
"""

from __future__ import annotations

import threading
from typing import Optional

from ..config.settings import AppConfig
from ..core.signals import AppSignals

try:  # pragma: no cover - environment dependent
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    ROS_AVAILABLE = True
except ImportError:  # pragma: no cover
    rclpy = None  # type: ignore[assignment]
    ROS_AVAILABLE = False


class RosManager:
    """Starts/stops rclpy, hosts the app node, exposes control publishers."""

    def __init__(self, config: AppConfig, signals: AppSignals) -> None:
        self._config = config
        self._signals = signals
        self._node: Optional["Node"] = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._ping_pub = None
        self._svlog_pub = None
        self._analysis_pub = None

    # ---- lifecycle -----------------------------------------------------------
    @property
    def available(self) -> bool:
        return ROS_AVAILABLE

    @property
    def node(self) -> Optional["Node"]:
        return self._node

    def start(self) -> bool:
        """Initialise rclpy and start spinning. Returns False without ROS."""
        if not ROS_AVAILABLE:
            self._signals.status_message.emit(
                "rclpy not available — running without ROS (use --sim).")
            self._signals.ros_connected.emit(False)
            return False
        rclpy.init()
        self._node = rclpy.create_node("blueboat_gcs")
        self._ping_pub = self._node.create_publisher(
            Bool, self._config.topics.ping_enable, 10)
        self._svlog_pub = self._node.create_publisher(
            Bool, self._config.topics.svlog_enable, 10)
        # AI seabed analysis output (JSON payload; schema in HANDOVER).
        self._analysis_pub = self._node.create_publisher(
            String, self._config.topics.seabed_analysis, 10)
        self._subscribe_rosout()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._spin, name="rclpy-spin", daemon=True)
        self._thread.start()
        self._signals.ros_connected.emit(True)
        return True

    def _subscribe_rosout(self) -> None:
        """ROS 2 log messages from ALL nodes -> embedded console.

        /rosout (rcl_interfaces/Log) carries every node's logger output,
        including sss_processor_node's — so the operator sees ROS logs
        without an external terminal. Our own node is filtered out to
        avoid duplicating lines already captured by the logging bus.
        """
        try:
            from rcl_interfaces.msg import Log
        except ImportError:                        # pragma: no cover
            return
        _LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN",
                   40: "ERROR", 50: "FATAL"}

        def on_log(msg: "Log") -> None:
            if msg.name == "blueboat_gcs":
                return
            level = _LEVELS.get(int(msg.level), str(msg.level))
            self._signals.log_line.emit(
                "rosout", f"[{level}] {msg.name}: {msg.msg}")

        self._node.create_subscription(Log, "/rosout", on_log, 50)

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception as exc:  # noqa: BLE001 - surface, never crash the GUI
            self._signals.status_message.emit(f"ROS executor stopped: {exc}")
            self._signals.ros_connected.emit(False)

    def shutdown(self) -> None:
        if not ROS_AVAILABLE or self._node is None:
            return
        self._executor.shutdown(timeout_sec=1.0)
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ---- control publishers --------------------------------------------------
    def publish_ping_enable(self, enable: bool) -> None:
        if self._ping_pub is not None:
            self._ping_pub.publish(Bool(data=enable))

    def publish_svlog_enable(self, enable: bool) -> None:
        if self._svlog_pub is not None:
            self._svlog_pub.publish(Bool(data=enable))

    def publish_seabed_analysis(self, payload_json: str) -> None:
        """Publish one seabed-image analysis (metadata + detections,
        never the pixels) on topics.seabed_analysis."""
        if self._analysis_pub is not None:
            self._analysis_pub.publish(String(data=payload_json))
