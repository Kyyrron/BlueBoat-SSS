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

from typing import Optional

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

    def __init__(self, node: Node, signals: AppSignals, topic: str,
                 queue_depth: int = 200,
                 warn_on_ping_gap: bool = True) -> None:
        self._signals = signals
        self._warn_gaps = warn_on_ping_gap
        # Stream health counters (also read by tests).
        self.received = 0
        self.device_gaps = 0        # pings the device numbered but we never saw
        self.crossed_pairs = 0      # port/starboard halves from different pings
        self._last_port_pn: Optional[int] = None
        self._warned_cross = False
        self._next_gap_warn = 50
        if not INTERFACES_AVAILABLE:
            signals.status_message.emit(
                "blueboat_interfaces not found — sonar stream disabled.")
            return
        # The processor publishes BEST_EFFORT; a RELIABLE subscriber would
        # be QoS-incompatible and receive nothing. BEST_EFFORT never
        # retransmits, so a deep queue is the only protection against
        # losing pings while the GUI thread renders (see
        # config.sonar_stream).
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=int(queue_depth))
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
        # --- stream health -------------------------------------------------
        self.received += 1
        pn = int(getattr(msg, "port_ping_number", 0))
        spn = int(getattr(msg, "starboard_ping_number", 0))
        if pn and spn and pn != spn:
            # The processor paired two different pings: on our sea-trial
            # data this happened on 27 % of rows and is what produces the
            # torn/mirrored mosaic. Report it once; the fix is robot-side
            # (see docs/SONARVIEW_SVLOG_ANALYSIS.md §9).
            self.crossed_pairs += 1
            if self._warn_gaps and not self._warned_cross:
                self._warned_cross = True
                self._signals.log_line.emit(
                    "app",
                    "SONAR: port/starboard halves come from different pings "
                    f"(#{pn} vs #{spn}). The processor is pairing by arrival "
                    "time; it should assemble by ping_number and route by "
                    "channel_number (see HANDOVER 'Sonar stream integrity').")
        if pn and self._last_port_pn is not None:
            missing = pn - self._last_port_pn - 1
            if 0 < missing < 1000:
                first = self.device_gaps == 0
                self.device_gaps += missing
                if self._warn_gaps and (first or
                                        self.device_gaps >= self._next_gap_warn):
                    while self.device_gaps >= self._next_gap_warn:
                        self._next_gap_warn *= 4
                    self._signals.log_line.emit(
                        "app",
                        f"SONAR: {self.device_gaps} ping(s) lost upstream of "
                        "the GCS (gap in the device's own ping_number). This "
                        "is acquisition/QoS loss, not a display problem.")
        if pn:
            self._last_port_pn = pn

        q = msg.robot_orientation
        # Recover the sonar's CONFIGURED slant range exactly, even when
        # the altitude estimate wobbles: ground_max = sqrt(R^2 - h^2), so
        # R = hypot(ground_max, h). This gives the live waterfall the
        # same stable column scale the replay path gets from length_mm.
        ground_max = float(np.abs(y_local).max()) if y_local.size else 0.0
        depth = float(msg.water_depth)
        slant_range = float(np.hypot(ground_max, depth))
        ping = SonarPing(
            t=msg.port_stamp.sec + msg.port_stamp.nanosec * 1e-9,
            robot_x=float(msg.robot_x),
            robot_y=float(msg.robot_y),
            yaw=quat_to_yaw(q.x, q.y, q.z, q.w),
            water_depth=float(msg.water_depth),
            y_local=y_local,
            intensity_db=intensity,
            slant_range_m=slant_range,
            sides=("both" if (len(msg.port_y) and len(msg.starboard_y))
                   else ("port" if len(msg.port_y) else "starboard")),
        )
        self._signals.sonar_ping.emit(ping)
