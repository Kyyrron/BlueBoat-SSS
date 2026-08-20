"""Robot telemetry aggregation.

Topics (all provided by the robot-control repositories; mavros + odom):

======================================  ==========================  ======
Topic (config key)                      Message type                Rate
======================================  ==========================  ======
topics.odom                             nav_msgs/Odometry           ~20 Hz
topics.navsat                           sensor_msgs/NavSatFix       ~5 Hz
topics.compass_hdg                      std_msgs/Float64 (degrees)  ~10 Hz
topics.vfr_hud (optional)               mavros_msgs/VfrHud          ~4 Hz
======================================  ==========================  ======

The listener merges the latest values into a ``RobotState`` and emits it
on every odom message, throttled to ``EMIT_HZ`` — the panels do not need
20 Hz updates, and this keeps the queued-signal traffic low.

The GUI-side ``CoordinateConverter`` binds its GPS origin from the first
RobotState that carries both a local pose and a fix (see main_window).
mavros_msgs is optional: without it, speed falls back to the odom twist.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from ..config.settings import RosTopics
from ..core.signals import AppSignals
from ..models.robot_state import RobotState
from ..utils.geodesy import quat_to_yaw, yaw_to_compass_deg
from ..utils.pose_alignment import GpsPoseSynthesizer

try:  # pragma: no cover - mavros_msgs may be absent on the basestation
    from mavros_msgs.msg import VfrHud
    MAVROS_MSGS_AVAILABLE = True
except ImportError:  # pragma: no cover
    MAVROS_MSGS_AVAILABLE = False

EMIT_HZ = 5.0


class TelemetryListener:
    """Merges odom + mavros telemetry into RobotState signals."""

    def __init__(self, node: Node, signals: AppSignals,
                 topics: RosTopics, gps_fallback: bool = True) -> None:
        self._signals = signals
        self._lock = threading.Lock()
        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._compass_deg: Optional[float] = None
        self._speed: Optional[float] = None
        # GPS dead-reckoning fallback (sea-trial fix): if /blueboat/odom
        # is silent — or frozen at the origin while the GPS fix moves —
        # RobotState is synthesized from NavSatFix + compass heading so
        # trajectory / origin binding / ping re-stamping keep working.
        self._gps_fallback = gps_fallback
        self._synth = GpsPoseSynthesizer()
        self._last_odom_wall: float = 0.0
        self._odom_frozen_hint = False
        self._fallback_announced = False
        if gps_fallback:
            node.create_timer(1.0 / EMIT_HZ, self._maybe_synthesize)
        self._last_emit_t = 0.0

        best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST, depth=10)
        node.create_subscription(Odometry, topics.odom,
                                 self._on_odom, best_effort)
        node.create_subscription(NavSatFix, topics.navsat,
                                 self._on_navsat, best_effort)
        node.create_subscription(Float64, topics.compass_hdg,
                                 self._on_compass, 10)
        if MAVROS_MSGS_AVAILABLE:
            node.create_subscription(VfrHud, topics.vfr_hud,
                                     self._on_vfr_hud, best_effort)
        else:
            signals.status_message.emit(
                "mavros_msgs not found — speed derived from odom twist.")

    # ---- callbacks (ROS thread) -----------------------------------------------
    def _on_navsat(self, msg: NavSatFix) -> None:
        if (msg.status.status < 0 or math.isnan(msg.latitude)
                or math.isnan(msg.longitude)):
            return
        with self._lock:
            self._lat, self._lon = float(msg.latitude), float(msg.longitude)
        if self._gps_fallback:
            # Passive tracking: keeps the dead-reckoning reference + last
            # position current so the zero-frozen-odom check in _on_odom
            # can compare "where GPS says we are" without emitting.
            self._synth.update(time.time(), float(msg.latitude),
                               float(msg.longitude), None)

    def _maybe_synthesize(self) -> None:
        """5 Hz node timer: emit a GPS-dead-reckoned RobotState when the
        odom stream is dead (> 2 s silent) or zero-frozen while the GPS
        fix moves. Regular odom resumes normal emission automatically."""
        now = time.monotonic()
        odom_dead = now - self._last_odom_wall > 2.0
        if not (odom_dead or self._odom_frozen_hint):
            return
        with self._lock:
            lat, lon = self._lat, self._lon
            heading = self._compass_deg
            speed_ext = self._speed
        if lat is None or lon is None:
            return                                  # nothing to reckon from
        t = time.time()
        x, y, yaw, speed = self._synth.update(t, lat, lon, heading)
        if not self._fallback_announced and math.hypot(x, y) > 1.0:
            self._fallback_announced = True
            self._signals.log_line.emit(
                "app",
                "ALIGNMENT: /blueboat/odom is "
                + ("silent" if odom_dead else "frozen at the origin")
                + " while the GPS fix moves — RobotState is now GPS+compass "
                  "dead-reckoned (fix the robot-side odom publisher; see "
                  "HANDOVER 'Sea-trial pose alignment').")
        self._signals.robot_state.emit(RobotState(
            t=t, x=x, y=y, yaw=yaw, lat=lat, lon=lon,
            heading_deg=(heading if heading is not None
                         else yaw_to_compass_deg(yaw)),
            speed_mps=(speed_ext if speed_ext is not None else speed)))

    def _on_compass(self, msg: Float64) -> None:
        if math.isnan(msg.data) or math.isinf(msg.data):
            return
        with self._lock:
            self._compass_deg = float(msg.data) % 360.0

    def _on_vfr_hud(self, msg: "VfrHud") -> None:
        with self._lock:
            self._speed = float(msg.groundspeed)

    def _on_odom(self, msg: Odometry) -> None:
        # Throttle on the WALL clock, never on message stamps: replayed
        # bags, sim time, or a restarted publisher can carry stamps that
        # jump backwards, and a stamp-based throttle would then silently
        # reject every message until the stamps caught up with the last
        # seen value (robot frozen until it "returns" to its previous
        # time). monotonic() is immune to all of that.
        now = time.monotonic()
        self._last_odom_wall = now
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        # Zero-frozen odom while a GPS fix exists and moves is the live
        # pathology: hand pose duty to the GPS synthesizer instead of
        # emitting a state pinned at the origin.
        if self._gps_fallback and abs(px) < 0.05 and abs(py) < 0.05:
            with self._lock:
                has_fix = self._lat is not None
            if has_fix and self._synth.has_reference:
                sx, sy = self._synth._last[1:] if self._synth._last else (0, 0)
                if math.hypot(sx, sy) > 1.0:      # GPS says we moved
                    self._odom_frozen_hint = True
                    return                        # synthesizer will emit
        self._odom_frozen_hint = False
        if now - self._last_emit_t < 1.0 / EMIT_HZ:
            return
        self._last_emit_t = now
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        with self._lock:
            lat, lon = self._lat, self._lon
            heading = self._compass_deg
            speed = self._speed
        if heading is None:
            heading = yaw_to_compass_deg(yaw)  # odom frame assumed ENU
        if speed is None:
            tw = msg.twist.twist.linear
            speed = math.hypot(tw.x, tw.y)
        self._signals.robot_state.emit(RobotState(
            t=t,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw, lat=lat, lon=lon,
            heading_deg=heading, speed_mps=speed,
        ))
