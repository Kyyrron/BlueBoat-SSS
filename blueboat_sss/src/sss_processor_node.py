#!/usr/bin/env python3
"""
Side Scan Sonar processor node for the BlueBoat.

Two responsibilities, both consumers of the side scan sonar streams:

1. **Processing.** Consume parsed `OmniscanProfile` packets from the port +
   starboard transducers and produce one merged `ProcessedSSSPing` per
   matched pair: dB-scaled samples, FBR-based altitude tracking, slant-range
   correction, water-column drop, robot pose snapped from /blueboat/odom.

2. **SonarView .svlog logging.** Consume the raw framed packets published
   on `~/raw` topics by `sss_node`, interleave them with mavlink wrapper
   packets built from mavros telemetry, and write a single SonarView-
   compatible .svlog file. Logging is OFF on startup; toggle via
   `~/log/enable`.

The two responsibilities are independent: processing runs whether or not
logging is enabled, and logging needs no processing-side bootstrap.

Note on non-flat seabed: this node performs per-ping altitude tracking
(altitude varies between pings) but assumes the seabed is flat within a
single ping's swath. Standard SSS practice with single-beam data.

Topics
------
Sub  /side_scan_sonar/port/profile         blueboat_interfaces/OmniscanProfile
Sub  /side_scan_sonar/starboard/profile    blueboat_interfaces/OmniscanProfile
Sub  /side_scan_sonar/port/raw             std_msgs/UInt8MultiArray
Sub  /side_scan_sonar/starboard/raw        std_msgs/UInt8MultiArray
Sub  /blueboat/odom                        nav_msgs/Odometry
Sub  /mavros/imu/data                      sensor_msgs/Imu
Sub  /mavros/global_position/global        sensor_msgs/NavSatFix
Sub  /mavros/global_position/rel_alt       std_msgs/Float64
Sub  /mavros/global_position/compass_hdg   std_msgs/Float64
Sub  ~/log/enable                          std_msgs/Bool
Pub  ~/processed                           blueboat_interfaces/ProcessedSSSPing
"""

from __future__ import annotations

import math
import os
import sys
from datetime import datetime
import threading
import time
from collections import deque
from pathlib import Path
from typing import Deque, Optional, Tuple

# Flat-file imports: pick up svlog.py and sss_processing.py from the same
# install directory as this script. ament_cmake_python installs all four
# .py files into install/<pkg>/lib/<pkg>/, and Python auto-prepends the
# script's directory to sys.path when invoked directly; the explicit
# insert below makes that behaviour robust to alternative invocations
# (e.g. importlib, IDE runners, packaged launch wrappers).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Time as TimeMsg
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import HomePosition, VfrHud
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64, UInt8MultiArray

from blueboat_interfaces.msg import OmniscanProfile, ProcessedSSSPing

from svlog_helper import (
    DEFAULT_MAVLINK_FILTER,
    DEVICE_ID_PORT,
    DEVICE_ID_STBD,
    SvlogWriter,
    build_mavlink_wrapper,
    build_session_metadata,
    retag_packet_src_device_id,
)
from sss_helper import (
    FBRTracker,
    detect_fbr_slant_m,
    project_side,
    scale_to_db,
)

from math_helper import (
    stamp_to_ns,
    quat_to_euler_rpy,
)


# ---------------------------------------------------------------------------
# Transducer geometry -- measure on the physical BlueBoat and fill in.
# All in meters, expressed in base_link (REP-103: +x forward, +y left = port,
# +z up). y offsets are positive magnitudes; port/starboard sign is in code.
# ---------------------------------------------------------------------------
TRANSDUCER_X_OFFSET_M:      float = 0.0  # TODO: forward offset (probably negative)
TRANSDUCER_Y_OFFSET_PORT_M: float = 0.0  # TODO: lateral offset of port transducer
TRANSDUCER_Y_OFFSET_STBD_M: float = 0.0  # TODO: lateral offset of starboard transducer
TRANSDUCER_SUBMERSION_M:    float = 0.0  # TODO: depth below the waterline


# ---------------------------------------------------------------------------
# FBR / altitude-tracking parameters (tunable on first field experiment).
# ---------------------------------------------------------------------------
NOISE_FLOOR_WINDOW:       int   = 20    # samples used to estimate noise floor
FBR_THRESHOLD_DELTA_DB:   float = 8.0   # dB above noise floor
WITHIN_PING_PERSISTENCE:  int   = 3     # consecutive samples above threshold

RINGING_SEARCH_MAX:       int   = 60    # search horizon in samples (1.5 m at 25 mm/sample)
RINGING_DROP_DB:          float = 10.0  # how far below the ringing peak counts as 'settled'
RINGING_PERSISTENCE:      int   = 5     # consecutive samples below target

BOOTSTRAP_PINGS:          int   = 10    # per-side self-consistency window
ALTITUDE_AGREEMENT_TOL_M: float = 0.30  # max spread within a side's bootstrap window
ALTITUDE_OUTLIER_TOL_M:   float = 1.0   # post-lock per-ping jump rejected as outlier
ALTITUDE_RELOCK_AFTER:    int   = 15    # consecutive rejects force a side to re-bootstrap

TIME_MATCH_TOLERANCE_NS:  int   = 50_000_000  # port-vs-starboard pairing window
ODOM_BUFFER_SECONDS:      float = 5.0

# ---------------------------------------------------------------------------
# Odom buffer
# ---------------------------------------------------------------------------
class _OdomBuffer:
    """Thread-safe sliding buffer of /blueboat/odom samples with nearest-stamp
    lookup. A linear scan is fine: at 20 Hz odom + 5 s window, ~100 entries.
    """

    def __init__(self, max_age_ns: int) -> None:
        self._max_age_ns = max_age_ns
        self._samples: Deque[Tuple[int, Odometry]] = deque()
        self._lock = threading.Lock()

    def push(self, msg: Odometry) -> None:
        ts = stamp_to_ns(msg.header.stamp)
        with self._lock:
            self._samples.append((ts, msg))
            cutoff = ts - self._max_age_ns
            while self._samples and self._samples[0][0] < cutoff:
                self._samples.popleft()

    def has_data(self) -> bool:
        with self._lock:
            return bool(self._samples)

    def nearest(self, target_ns: int) -> Optional[Odometry]:
        with self._lock:
            if not self._samples:
                return None
            best_ts, best_msg = self._samples[0]
            best_dt = abs(best_ts - target_ns)
            for ts, msg in self._samples:
                dt = abs(ts - target_ns)
                if dt < best_dt:
                    best_ts, best_msg, best_dt = ts, msg, dt
            return best_msg


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class SSSProcessorNode(Node):
    """Process OmniscanProfile streams; also write SonarView .svlog files."""

    def __init__(self) -> None:
        super().__init__("sss_processor")

        # ---- Processing state ---------------------------------------------
        self._port_buf: Deque[OmniscanProfile] = deque()
        self._stbd_buf: Deque[OmniscanProfile] = deque()
        self._buf_lock = threading.Lock()
        self._tol_ns = TIME_MATCH_TOLERANCE_NS

        self._odom_buf = _OdomBuffer(int(ODOM_BUFFER_SECONDS * 1e9))
        self._fbr = FBRTracker(
            bootstrap_pings=BOOTSTRAP_PINGS,
            agreement_tol_m=ALTITUDE_AGREEMENT_TOL_M,
            outlier_tol_m=ALTITUDE_OUTLIER_TOL_M,
            relock_after=ALTITUDE_RELOCK_AFTER,
        )

        self._dropped_no_odom = 0
        self._dropped_bootstrap = 0
        self._already_bootstrapped_logged = False

        # ---- Logging + mavlink envelope state ------------------------------

        #self.date = datetime.today().strftime('%Y_%m_%d-%H_%M')
        #self.declare_parameter("log_folder", self.date)
        #self.folder_name = self.get_parameter("log_folder").value
        self.log_root = Path(os.path.expanduser("../../../data/SSS_data")) #/ self.folder_name
        #self.log_root.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f"log directory: {self.log_root}")
        self._svlog = SvlogWriter(
            log_dir=self.log_root,
            metadata_provider=self._build_metadata,
        )

        # Aux mavros signals used to enrich GLOBAL_POSITION_INT.
        self._aux_lock = threading.Lock()
        self._latest_rel_alt_mm:        Optional[int] = None
        self._latest_compass_hdg_cdeg:  Optional[int] = None
        # Latest local-position velocity (paired with PoseStamped to build
        # LOCAL_POSITION_NED). Holding the most recent twist is fine -- on
        # the robot they're published at the same rate from the same source.
        self._latest_local_twist:       Optional[TwistStamped] = None

        # Monotonic per-message sequence counter for mavlink2rest envelopes
        # (matches mavlink's 0..255 wrap).
        self._mav_seq = 0
        self._mav_seq_lock = threading.Lock()
        self._node_boot_ns = time.monotonic_ns()

        # ---- QoS -----------------------------------------------------------
        sonar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- IO ------------------------------------------------------------
        # Parsed profiles -> processing pipeline.
        self.create_subscription(OmniscanProfile, "/side_scan_sonar/port/profile",      self._on_port,          sonar_qos)
        self.create_subscription(OmniscanProfile, "/side_scan_sonar/starboard/profile", self._on_starboard,     sonar_qos)
        # Raw bytes -> svlog (independent of processing).
        self.create_subscription(UInt8MultiArray, "/side_scan_sonar/port/raw",          self._on_port_raw,      sonar_qos)
        self.create_subscription(UInt8MultiArray, "/side_scan_sonar/starboard/raw",     self._on_starboard_raw, sonar_qos)
        # Pose for processing.
        self.create_subscription(Odometry,        "/blueboat/odom",                     self._on_odom,          odom_qos)
        # Mavros telemetry -> svlog mavlink wrappers.
        self.create_subscription(Imu,             "/mavros/imu/data",                       self._on_mavros_imu,           sonar_qos)
        self.create_subscription(NavSatFix,       "/mavros/global_position/global",         self._on_mavros_navsat,        sonar_qos)
        self.create_subscription(Float64,         "/mavros/global_position/rel_alt",        self._on_mavros_rel_alt,       10)
        self.create_subscription(Float64,         "/mavros/global_position/compass_hdg",    self._on_mavros_compass_hdg,   10)
        self.create_subscription(TwistStamped,    "/mavros/local_position/velocity_local",  self._on_mavros_local_vel,     sonar_qos)
        self.create_subscription(PoseStamped,     "/mavros/local_position/pose",            self._on_mavros_local_pose,    sonar_qos)
        self.create_subscription(HomePosition,    "/mavros/home_position/home",             self._on_mavros_home_position, 10)
        self.create_subscription(GeoPointStamped, "/mavros/global_position/gp_origin",      self._on_mavros_gp_origin,     10)
        self.create_subscription(VfrHud,          "/mavros/vfr_hud",                        self._on_mavros_vfr_hud,       sonar_qos)
        self.get_logger().info("mavros telemetry subscriptions active")
        # Logging toggle.
        self.create_subscription(Bool, "~/log/enable", self._on_log_enable, 10)

        self._pub = self.create_publisher(ProcessedSSSPing, "~/processed", sonar_qos)

        self.get_logger().info(
            "sss_processor ready (log OFF):\n"
            "  port  ← /side_scan_sonar/port/profile\n"
            "  stbd  ← /side_scan_sonar/starboard/profile\n"
            "  port raw  ← /side_scan_sonar/port/raw\n"
            "  stbd raw  ← /side_scan_sonar/starboard/raw\n"
            "  odom  ← /blueboat/odom\n"
            "  out   → ~/processed\n"
            f"  bootstrap: {BOOTSTRAP_PINGS} self-consistent pings per side within "
            f"{ALTITUDE_AGREEMENT_TOL_M*100:.0f} cm (either side suffices)\n"
            "  Toggle logging with:\n"
            "  ros2 topic pub --once /sss_processor/log/enable std_msgs/msg/Bool 'data: true'"
        )

    # ----- shutdown ---------------------------------------------------------
    def shutdown(self) -> None:
        self.get_logger().info("stopping sss_processor")
        self._svlog.stop()

    # ----- sonar subscribers ------------------------------------------------
    def _on_port(self, msg: OmniscanProfile) -> None:
        with self._buf_lock:
            self._port_buf.append(msg)
            self._drain_matches()

    def _on_starboard(self, msg: OmniscanProfile) -> None:
        with self._buf_lock:
            self._stbd_buf.append(msg)
            self._drain_matches()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_buf.push(msg)

    def _on_port_raw(self, msg: UInt8MultiArray) -> None:
        self._write_raw_with_src_tag(msg, DEVICE_ID_PORT)

    def _on_starboard_raw(self, msg: UInt8MultiArray) -> None:
        self._write_raw_with_src_tag(msg, DEVICE_ID_STBD)

    def _on_log_enable(self, msg: Bool) -> None:
        if msg.data:
            self._svlog.start()
            self.get_logger().info(f"logging -> {self.log_root}")
        else:
            self._svlog.current_path
            self._svlog.stop()
            self.get_logger().info(f"stopped logging ({self.log_root})")

    # ----- mavros subscribers -----------------------------------------------
    def _on_mavros_rel_alt(self, msg: Float64) -> None:
        if math.isnan(msg.data) or math.isinf(msg.data):
            return
        with self._aux_lock:
            self._latest_rel_alt_mm = int(round(msg.data * 1000.0))

    def _on_mavros_compass_hdg(self, msg: Float64) -> None:
        if math.isnan(msg.data) or math.isinf(msg.data):
            return
        # mavros publishes degrees; mavlink GLOBAL_POSITION_INT.hdg is cdeg.
        cdeg = int(round(msg.data * 100.0)) % 36000
        with self._aux_lock:
            self._latest_compass_hdg_cdeg = cdeg

    def _on_mavros_imu(self, msg: Imu) -> None:
        if not self._svlog.active:
            return
        envelope = self._build_attitude_envelope(msg)
        if envelope is None:
            return
        try:
            self._svlog.write(build_mavlink_wrapper(envelope))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"mavlink ATTITUDE write failed: {exc}")

    def _on_mavros_navsat(self, msg: NavSatFix) -> None:
        if not self._svlog.active:
            return
        envelope = self._build_global_position_envelope(msg)
        if envelope is None:
            return
        try:
            self._svlog.write(build_mavlink_wrapper(envelope))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"mavlink GLOBAL_POSITION_INT write failed: {exc}")

    def _on_mavros_local_vel(self, msg: TwistStamped) -> None:
        # Cache only -- LOCAL_POSITION_NED is emitted on the pose callback,
        # so we can pair this twist with the next pose.
        with self._aux_lock:
            self._latest_local_twist = msg

    def _on_mavros_local_pose(self, msg: PoseStamped) -> None:
        if not self._svlog.active:
            return
        with self._aux_lock:
            twist = self._latest_local_twist
        try:
            envelope = self._build_local_position_envelope(msg, twist)
            self._svlog.write(build_mavlink_wrapper(envelope))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"mavlink LOCAL_POSITION_NED write failed: {exc}")

    def _on_mavros_home_position(self, msg: HomePosition) -> None:
        if not self._svlog.active:
            return
        try:
            self._svlog.write(build_mavlink_wrapper(
                self._build_home_position_envelope(msg)))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"mavlink HOME_POSITION write failed: {exc}")

    def _on_mavros_gp_origin(self, msg: GeoPointStamped) -> None:
        if not self._svlog.active:
            return
        try:
            self._svlog.write(build_mavlink_wrapper(
                self._build_gp_origin_envelope(msg)))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"mavlink GPS_GLOBAL_ORIGIN write failed: {exc}")

    def _on_mavros_vfr_hud(self, msg: VfrHud) -> None:
        if not self._svlog.active:
            return
        try:
            self._svlog.write(build_mavlink_wrapper(
                self._build_vfr_hud_envelope(msg)))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"mavlink VFR_HUD write failed: {exc}")

    # ----- mavlink envelope builders ----------------------------------------
    def _next_seq(self) -> int:
        with self._mav_seq_lock:
            s = self._mav_seq
            self._mav_seq = (self._mav_seq + 1) & 0xFF
        return s

    def _time_boot_ms(self, stamp: TimeMsg) -> int:
        """Derive `time_boot_ms` from a ROS header.stamp.

        Critical: SonarView pairs ATTITUDE / GLOBAL_POSITION_INT /
        LOCAL_POSITION_NED by `time_boot_ms` to compute heading-corrected
        position. ROS messages from the same source mavlink burst carry
        identical `header.stamp` (set by mavros from the source timestamp;
        set by the svlog-to-rosbag converter from the source mavlink burst).
        Deriving `time_boot_ms` from the stamp preserves that coherence.

        The offset (relative to node start, modulo 2^32) is arbitrary --
        SonarView only looks at value equality, not absolute meaning.
        """
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        return ((stamp_ns - self._node_boot_ns) // 1_000_000) & 0xFFFFFFFF

    def _mavlink_header(self) -> dict:
        return {
            "system_id":    1,
            "component_id": 1,
            "sequence":     self._next_seq(),
        }

    def _build_attitude_envelope(self, imu: Imu) -> Optional[dict]:
        q = imu.orientation
        # Reject obviously-invalid quaternions (mavros publishes (0,0,0,0)
        # when it hasn't received AHRS yet).
        if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) < 1e-6:
            return None
        # mavros publishes orientation in ENU/FLU per REP-103. Mavlink
        # ATTITUDE expects NED/FRD, so we invert the same rotation the
        # converter applies (NED -> ENU): roll unchanged, pitch and yaw
        # signed by the ENU<->NED frame swap.
        roll_enu, pitch_enu, yaw_enu = quat_to_euler_rpy(q.x, q.y, q.z, q.w)
        roll_ned  =  roll_enu
        pitch_ned = -pitch_enu
        yaw_ned   = math.pi / 2.0 - yaw_enu
        return {
            "header": self._mavlink_header(),
            "message": {
                "type":         "ATTITUDE",
                "time_boot_ms": self._time_boot_ms(imu.header.stamp),
                "roll":         float(roll_ned),
                "pitch":        float(pitch_ned),
                "yaw":          float(yaw_ned),
                "rollspeed":    float(imu.angular_velocity.x),
                "pitchspeed":  -float(imu.angular_velocity.y),
                "yawspeed":    -float(imu.angular_velocity.z),
            },
        }

    def _build_global_position_envelope(self, fix: NavSatFix) -> Optional[dict]:
        # status.status == -1 (STATUS_NO_FIX) means lat/lon are meaningless.
        if fix.status.status < 0:
            return None
        if (math.isnan(fix.latitude) or math.isnan(fix.longitude)
                or math.isnan(fix.altitude)):
            return None
        with self._aux_lock:
            rel_alt_mm = self._latest_rel_alt_mm or 0
            hdg_cdeg = self._latest_compass_hdg_cdeg or 0
        return {
            "header": self._mavlink_header(),
            "message": {
                "type":         "GLOBAL_POSITION_INT",
                "time_boot_ms": self._time_boot_ms(fix.header.stamp),
                "lat":          int(round(fix.latitude  * 1e7)),
                "lon":          int(round(fix.longitude * 1e7)),
                "alt":          int(round(fix.altitude  * 1000.0)),  # mm AMSL
                "relative_alt": int(rel_alt_mm),
                "vx":           0,
                "vy":           0,
                "vz":           0,
                "hdg":          int(hdg_cdeg),
            },
        }

    def _build_local_position_envelope(self, pose: 'PoseStamped',
                                       twist: Optional['TwistStamped']) -> dict:
        """Build LOCAL_POSITION_NED from /mavros/local_position/pose (ENU)
        plus an optional matching velocity_local TwistStamped. The pose
        provides position only; we convert ENU -> NED. Velocity is
        optional -- zeros are acceptable per the mavlink schema."""
        px, py, pz = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        # ENU -> NED: x_n = y_e (ROS y), y_e = x_e (ROS x), z_d = -z_u
        x_n, y_e, z_d = py, px, -pz
        vx = vy = vz = 0.0
        if twist is not None:
            tx, ty, tz = (twist.twist.linear.x, twist.twist.linear.y,
                          twist.twist.linear.z)
            vx, vy, vz = ty, tx, -tz
        return {
            "header": self._mavlink_header(),
            "message": {
                "type":         "LOCAL_POSITION_NED",
                "time_boot_ms": self._time_boot_ms(pose.header.stamp),
                "x":  float(x_n), "y":  float(y_e), "z":  float(z_d),
                "vx": float(vx),  "vy": float(vy),  "vz": float(vz),
            },
        }

    def _build_home_position_envelope(self, h: 'HomePosition') -> dict:
        return {
            "header": self._mavlink_header(),
            "message": {
                "type":      "HOME_POSITION",
                "latitude":  int(round(h.geo.latitude  * 1e7)),
                "longitude": int(round(h.geo.longitude * 1e7)),
                "altitude":  int(round(h.geo.altitude  * 1000.0)),
                "x": 0.0, "y": 0.0, "z": 0.0,
                "q": [1.0, 0.0, 0.0, 0.0],
                "approach_x": 0.0, "approach_y": 0.0, "approach_z": 0.0,
            },
        }

    def _build_gp_origin_envelope(self, g: 'GeoPointStamped') -> dict:
        return {
            "header": self._mavlink_header(),
            "message": {
                "type":      "GPS_GLOBAL_ORIGIN",
                "latitude":  int(round(g.position.latitude  * 1e7)),
                "longitude": int(round(g.position.longitude * 1e7)),
                "altitude":  int(round(g.position.altitude  * 1000.0)),
            },
        }

    def _build_vfr_hud_envelope(self, v: 'VfrHud') -> dict:
        return {
            "header": self._mavlink_header(),
            "message": {
                "type":        "VFR_HUD",
                "airspeed":    float(v.airspeed),
                "groundspeed": float(v.groundspeed),
                "heading":     int(v.heading),
                "throttle":    int(round(v.throttle * 100.0)),
                "alt":         float(v.altitude),
                "climb":       float(v.climb),
            },
        }

    # ----- svlog helpers ----------------------------------------------------
    def _write_raw_with_src_tag(self, msg: UInt8MultiArray, src_device_id: int) -> None:
        if not self._svlog.active:
            return
        try:
            tagged = retag_packet_src_device_id(
                bytes(bytearray(msg.data)), src_device_id
            )
            self._svlog.write(tagged)
        except ValueError as exc:
            self.get_logger().warn(f"dropping malformed raw packet: {exc}")

    def _build_metadata(self) -> bytes:
        """Called by SvlogWriter on each new file."""
        return build_session_metadata(
            port_url="tcp://192.168.2.92:51200",
            starboard_url="tcp://192.168.2.93:51200",
            mavlink_url="ws://blueos.local:6040/v1/ws/mavlink",
            mavlink_filter=DEFAULT_MAVLINK_FILTER,
        )

    # ----- two-pointer time matcher -----------------------------------------
    def _drain_matches(self) -> None:
        """Emit every possible match from the heads of both buffers.

        Called under self._buf_lock. Standard two-pointer merge: if the
        oldest port and oldest starboard are within tolerance, match;
        otherwise drop whichever is older (no partner left to find).
        """
        while self._port_buf and self._stbd_buf:
            p = self._port_buf[0]
            s = self._stbd_buf[0]
            p_ns = stamp_to_ns(p.header.stamp)
            s_ns = stamp_to_ns(s.header.stamp)
            dt = p_ns - s_ns
            if abs(dt) <= self._tol_ns:
                self._port_buf.popleft()
                self._stbd_buf.popleft()
                self._emit_merged(p, s)
            elif dt > 0:
                self._stbd_buf.popleft()
            else:
                self._port_buf.popleft()

    # ----- processing -------------------------------------------------------
    def _emit_merged(self, port: OmniscanProfile, stbd: OmniscanProfile) -> None:
        log = self.get_logger()

        # 1. Odom must exist; if SSS started before robot_interface, drop early pings.
        if not self._odom_buf.has_data():
            self._dropped_no_odom += 1
            if self._dropped_no_odom == 1 or self._dropped_no_odom % 20 == 0:
                log.warn(
                    f"dropping ping pair: no /blueboat/odom yet "
                    f"(total dropped: {self._dropped_no_odom})"
                )
            return

        # 2. dB conversion.
        port_db = scale_to_db(port.pwr_results, port.min_pwr_db, port.max_pwr_db)
        stbd_db = scale_to_db(stbd.pwr_results, stbd.min_pwr_db, stbd.max_pwr_db)

        # 3. FBR detection per side.
        port_alt = detect_fbr_slant_m(
            port_db, port.start_mm, port.length_mm, port.num_results,
            noise_floor_window=NOISE_FLOOR_WINDOW,
            threshold_delta_db=FBR_THRESHOLD_DELTA_DB,
            persistence=WITHIN_PING_PERSISTENCE,
            ringing_search_max=RINGING_SEARCH_MAX,        
            ringing_drop_db=RINGING_DROP_DB,             
            ringing_persistence=RINGING_PERSISTENCE, 
        )
        stbd_alt = detect_fbr_slant_m(
            stbd_db, stbd.start_mm, stbd.length_mm, stbd.num_results,
            noise_floor_window=NOISE_FLOOR_WINDOW,
            threshold_delta_db=FBR_THRESHOLD_DELTA_DB,
            persistence=WITHIN_PING_PERSISTENCE,
            ringing_search_max=RINGING_SEARCH_MAX,        
            ringing_drop_db=RINGING_DROP_DB,             
            ringing_persistence=RINGING_PERSISTENCE, 
        )

        # 4. Update the cross-ping altitude tracker.
        altitude = self._fbr.update(port_alt, stbd_alt)
        if altitude is None:
            self._dropped_bootstrap += 1
            if self._dropped_bootstrap == 1 or self._dropped_bootstrap % BOOTSTRAP_PINGS == 0:
                log.info(
                    f"FBR bootstrap in progress "
                    f"({self._dropped_bootstrap} ping pairs dropped so far; "
                    f"port_fbr={port_alt}, stbd_fbr={stbd_alt})"
                )
            return
        if not self._already_bootstrapped_logged:
            log.info(f"FBR bootstrapped: altitude = {altitude:.2f} m above seabed")
            self._already_bootstrapped_logged = True

        # 5. Water depth = transducer altitude + submersion.
        water_depth = altitude + TRANSDUCER_SUBMERSION_M

        # 6. Slant-range correct each side; drop water-column samples.
        port_y, port_int = project_side(
            port_db, port.start_mm, port.length_mm, port.num_results,
            altitude_m=altitude,
            transducer_y_offset_m=TRANSDUCER_Y_OFFSET_PORT_M,
            side_sign=+1.0,
        )
        stbd_y, stbd_int = project_side(
            stbd_db, stbd.start_mm, stbd.length_mm, stbd.num_results,
            altitude_m=altitude,
            transducer_y_offset_m=TRANSDUCER_Y_OFFSET_STBD_M,
            side_sign=-1.0,
        )

        # 7. Snap robot pose using port stamp (pair is within tolerance).
        merged_ns = stamp_to_ns(port.header.stamp)
        odom = self._odom_buf.nearest(merged_ns)
        if odom is None:
            self._dropped_no_odom += 1
            return

        # 8. Assemble + publish.
        out = ProcessedSSSPing()
        out.port_stamp = port.header.stamp
        out.starboard_stamp = stbd.header.stamp
        out.port_ping_number = port.ping_number
        out.starboard_ping_number = stbd.ping_number
        out.robot_x = float(odom.pose.pose.position.x)
        out.robot_y = float(odom.pose.pose.position.y)
        out.robot_orientation = odom.pose.pose.orientation
        out.water_depth = float(water_depth)
        out.transducer_x_offset = float(TRANSDUCER_X_OFFSET_M)
        out.port_intensity_db = port_int
        out.port_y = port_y
        out.starboard_intensity_db = stbd_int
        out.starboard_y = stbd_y
        self._pub.publish(out)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = SSSProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
