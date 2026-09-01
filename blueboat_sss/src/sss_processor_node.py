#!/usr/bin/env python3
"""
Side Scan Sonar processor node for the BlueBoat.

Two responsibilities, both consumers of the side scan sonar streams:

1. **Processing.** Consume parsed `OmniscanProfile` packets from the port +
   starboard transducers and produce one `ProcessedSSSPing` per acquired
   ping: dB-scaled samples, FBR-based altitude tracking, slant-range
   correction, water-column drop, robot pose snapped from /blueboat/odom.

   Rows are assembled by `ping_number`, normalised across the two devices
   (they are independent units with independent counters). A row is emitted
   with whichever sides arrived; one-sided rows are published rather than
   discarded, and no ping is withheld while the altitude tracker bootstraps
   (NON-NEGOTIABLE #2). The only ping that does not reach the output is one
   with no `/blueboat/odom` pose to place it at.

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
import struct
import sys
from datetime import datetime
import threading
import time
from collections import OrderedDict, deque
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
    OS_MONO_PROFILE_ID,
    SvlogWriter,
    build_mavlink_wrapper,
    build_session_metadata,
    retag_packet_src_device_id,
)
from sss_helper import (
    FBRTracker,
    PingCounterOffset,
    detect_fbr_slant_m,
    project_side,
    scale_to_db,
)

from math_helper import (
    stamp_to_ns,
    quat_to_euler_rpy,
)


# ---------------------------------------------------------------------------
# OS_MONO_PROFILE frame offsets, used to read side identity straight out of the
# raw framed packet (8-byte Ping-Protocol header + payload offset).
# NON-NEGOTIABLE #1: side identity comes from the packet, never from the topic
# it arrived on nor from the src tag already in the frame.
# ---------------------------------------------------------------------------
CHANNEL_BYTE: int = 34   # payload offset 26, uint8: 0 = port, 1 = starboard
HEADING_BYTE: int = 52   # payload offset 44, float32 LE: transducer_heading_deg


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

RINGING_SEARCH_MAX:       int   = 60    # search horizon in SAMPLES, not metres: its physical
                                        # reach scales with range_length_mm / num_results.
                                        # 2.0 m at the 33.3 mm/sample default (20 m / 600).
RINGING_DROP_DB:          float = 10.0  # how far below the ringing peak counts as 'settled'
RINGING_PERSISTENCE:      int   = 5     # consecutive samples below target

BOOTSTRAP_PINGS:          int   = 10    # per-side self-consistency window
ALTITUDE_AGREEMENT_TOL_M: float = 0.30  # max spread within a side's bootstrap window
ALTITUDE_OUTLIER_TOL_M:   float = 1.0   # post-lock per-ping jump rejected as outlier
ALTITUDE_RELOCK_AFTER:    int   = 15    # consecutive rejects force a side to re-bootstrap

ODOM_BUFFER_SECONDS:      float = 5.0
# Maximum |odom stamp - profile stamp| for a pose to be usable. Without
# it, a dead or clock-mismatched /blueboat/odom left the buffer's last
# sample latched and EVERY ping was stamped with that one stale pose --
# the "pings pile on one point" field bug. Past the tolerance the ping
# takes the sanctioned missing-pose drop (NON-NEGOTIABLE #2) with a
# warning naming the measured skew, instead of a silently wrong pose.
# 1 s = ~20 odom periods: generous against jitter, tiny against the
# minutes-scale skews of a genuine clock mismatch.
ODOM_NEAREST_TOLERANCE_S: float = 1.0

# ---------------------------------------------------------------------------
# Row assembly (NON-NEGOTIABLE #2: never drop a ping).
#
# Rows are keyed on `ping_number`, normalised across the two devices by
# `PingCounterOffset`. The two Omniscan units are independent devices with
# independent counters whose relative offset is arbitrary but constant per
# power-up (0, -1 and +60 all measured in the field corpus -- see
# blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md), so the raw counter is not a
# usable cross-side key on its own.
#
# A group is emitted as soon as both sides are in it. An incomplete group is
# emitted ONE-SIDED once it falls too far behind, by ping number or by wall
# clock, whichever comes first -- never discarded, and never held forever.
# ---------------------------------------------------------------------------
ASSEMBLY_MAX_LAG_PINGS:   int   = 128          # 2x the worst measured in-flight lag (63)
ASSEMBLY_MAX_LAG_NS:      int   = 1_000_000_000
ASSEMBLY_MAX_GROUPS:      int   = 256          # hard cap; oldest is force-flushed
ASSEMBLY_FLUSH_PERIOD_S:  float = 0.2          # tail flush when the stream stops
OFFSET_VOTE_WINDOW:       int   = 64
OFFSET_VOTE_DEFER:        int   = 4            # below this the estimator goes bimodal
# The estimator needs a few pings before it has voted at all. Pings that
# arrive first are held rather than keyed at a provisional offset, because
# keying them wrongly splits their rows -- and the start of the mission is
# exactly what NON-NEGOTIABLE #2 exists to protect. The cap bounds the hold
# for a single-transducer run, where no vote is ever cast.
OFFSET_MIN_VOTES:         int   = 8
OFFSET_PREROLL_MAX:       int   = 24

# ---------------------------------------------------------------------------
# Odom buffer
# ---------------------------------------------------------------------------
class _OdomBuffer:
    """Thread-safe sliding buffer of /blueboat/odom samples with nearest-stamp
    lookup. A linear scan is fine: at 20 Hz odom + 5 s window, ~100 entries.
    """

    def __init__(self, max_age_ns: int,
                 tolerance_ns: int = int(ODOM_NEAREST_TOLERANCE_S * 1e9)
                 ) -> None:
        self._max_age_ns = max_age_ns
        self._tolerance_ns = tolerance_ns
        self._samples: Deque[Tuple[int, Odometry]] = deque()
        self._lock = threading.Lock()
        #: |odom - profile| of the last refused lookup [ns]; the caller
        #: reads it to name the measured skew in its warning.
        self.last_refused_dt_ns: Optional[int] = None

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
        """The sample nearest ``target_ns``, or None when nothing lies
        within the tolerance.

        Pruning also happens here, against the *lookup* stamp: ``push``
        prunes against arrival, so a topic that went silent kept its
        last samples forever and served the same stale pose to every
        later ping. A ping whose nearest pose is farther than the
        tolerance is unplaceable and takes the sanctioned drop instead
        (NON-NEGOTIABLE #2 — a missing pose is the one legitimate drop).
        """
        with self._lock:
            cutoff = target_ns - self._max_age_ns
            while self._samples and self._samples[0][0] < cutoff:
                self._samples.popleft()
            if not self._samples:
                return None
            best_ts, best_msg = self._samples[0]
            best_dt = abs(best_ts - target_ns)
            for ts, msg in self._samples:
                dt = abs(ts - target_ns)
                if dt < best_dt:
                    best_ts, best_msg, best_dt = ts, msg, dt
            if best_dt > self._tolerance_ns:
                self.last_refused_dt_ns = best_dt
                return None
            return best_msg


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class SSSProcessorNode(Node):
    """Process OmniscanProfile streams; also write SonarView .svlog files."""

    def __init__(self) -> None:
        super().__init__("sss_processor")

        # ---- Processing state ---------------------------------------------
        # ping_number (normalised onto the port counter) -> group.
        # Insertion-ordered, and the keys are monotone in acquisition order,
        # so the oldest group is always the first one.
        self._groups: "OrderedDict[int, dict]" = OrderedDict()
        self._offset = PingCounterOffset(window=OFFSET_VOTE_WINDOW,
                                         defer=OFFSET_VOTE_DEFER)
        # Arrivals held until the counter offset is known; None once drained.
        self._preroll: Optional[list] = []
        self._max_key_seen: Optional[int] = None
        self._buf_lock = threading.Lock()

        self._odom_buf = _OdomBuffer(int(ODOM_BUFFER_SECONDS * 1e9))
        self._fbr = FBRTracker(
            bootstrap_pings=BOOTSTRAP_PINGS,
            agreement_tol_m=ALTITUDE_AGREEMENT_TOL_M,
            outlier_tol_m=ALTITUDE_OUTLIER_TOL_M,
            relock_after=ALTITUDE_RELOCK_AFTER,
        )

        self._dropped_no_odom = 0
        # Pings emitted while the FBR tracker was not locked. These are
        # EMITTED, not dropped (NON-NEGOTIABLE #2); the counter reports
        # altitude quality, never data loss.
        self._unlocked_pings = 0
        self._emitted = 0
        self._one_sided = 0
        self._already_bootstrapped_logged = False

        # ---- Logging + mavlink envelope state ------------------------------

        #self.date = datetime.today().strftime('%Y_%m_%d-%H_%M')
        #self.declare_parameter("log_folder", self.date)
        #self.folder_name = self.get_parameter("log_folder").value
        # Relative to the launch working directory, and resolved once here so
        # that every path this node reports afterwards is absolute -- "where
        # did my recording go?" must not need the reader to know the cwd.
        self.log_root = Path(
            os.path.expanduser("../../../../data/SSS_data")  #/ self.folder_name
        ).resolve()
        # Created at startup so a bad log path is known before the mission
        # rather than at Record ON. Failing is not fatal: processing and
        # logging are independent, and a processor that refused to start
        # would drop pings over a recording problem. SvlogWriter.start()
        # retries it.
        try:
            self.log_root.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(
                f"cannot create the log directory {self.log_root}: {exc}; "
                "processing continues, recording will fail until this is fixed"
            )
        self.get_logger().info(f"log directory: {self.log_root}")
        self._svlog = SvlogWriter(
            log_dir=self.log_root,
            metadata_provider=self._build_metadata,
            # svlog_helper is ROS-free by contract, so it cannot reach a ROS
            # logger itself; this is how a write failure reaches /rosout and
            # the GCS console.
            error_reporter=lambda message: self.get_logger().error(message),
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

        # Emits groups the arrival path can no longer reach, so the tail of a
        # run is not stranded when the stream stops (NON-NEGOTIABLE #2).
        self._flush_timer = self.create_timer(ASSEMBLY_FLUSH_PERIOD_S,
                                              self._flush_pending)

        self.get_logger().info(
            "sss_processor ready (log OFF):\n"
            "  port  ← /side_scan_sonar/port/profile\n"
            "  stbd  ← /side_scan_sonar/starboard/profile\n"
            "  port raw  ← /side_scan_sonar/port/raw\n"
            "  stbd raw  ← /side_scan_sonar/starboard/raw\n"
            "  odom  ← /blueboat/odom\n"
            "  out   → ~/processed\n"
            "  rows assembled by ping_number (offset-normalised across the two\n"
            "  devices); one-sided rows are emitted, never dropped\n"
            f"  bootstrap: {BOOTSTRAP_PINGS} self-consistent pings per side within "
            f"{ALTITUDE_AGREEMENT_TOL_M*100:.0f} cm (either side suffices); pings are\n"
            "  emitted with a provisional altitude until then\n"
            "  Toggle logging with:\n"
            "  ros2 topic pub --once /sss_processor/log/enable std_msgs/msg/Bool 'data: true'"
        )

    # ----- shutdown ---------------------------------------------------------
    def shutdown(self) -> None:
        self.get_logger().info("stopping sss_processor")
        # Anything still buffered is emitted rather than discarded.
        self._flush_pending(drain_all=True)
        self.get_logger().info(
            f"emitted {self._emitted} row(s), {self._one_sided} one-sided, "
            f"{self._unlocked_pings} with a provisional altitude, "
            f"{self._dropped_no_odom} dropped for missing odom; "
            f"ping-counter offset {self._offset.offset:+d} "
            f"({self._offset.confidence * 100:.0f}% of {self._offset.votes} votes)"
        )
        self._svlog.stop()

    # ----- sonar subscribers ------------------------------------------------
    def _on_port(self, msg: OmniscanProfile) -> None:
        self._accept(msg)

    def _on_starboard(self, msg: OmniscanProfile) -> None:
        self._accept(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_buf.push(msg)

    # The device id passed here is only the fallback: _write_raw_with_src_tag
    # tags each frame from the packet's own channel_number (NON-NEGOTIABLE #1).
    def _on_port_raw(self, msg: UInt8MultiArray) -> None:
        self._write_raw_with_src_tag(msg, DEVICE_ID_PORT)

    def _on_starboard_raw(self, msg: UInt8MultiArray) -> None:
        self._write_raw_with_src_tag(msg, DEVICE_ID_STBD)

    def _on_log_enable(self, msg: Bool) -> None:
        if msg.data:
            # start() reports its own reason and returns None on failure. The
            # GCS has no feedback topic, so this error line on /rosout is what
            # tells the operator the Record button is lying.
            path = self._svlog.start()
            if path is None:
                self.get_logger().error(
                    f"RECORDING FAILED: nothing is being written to "
                    f"{self.log_root}"
                )
            else:
                self.get_logger().info(f"logging -> {path}")
        else:
            # Read the path before stop() clears it, so the operator is told
            # which file was closed rather than only which directory.
            path = self._svlog.current_path
            self._svlog.stop()
            self.get_logger().info(
                f"stopped logging ({path if path is not None else self.log_root})"
            )

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
    @staticmethod
    def _src_from_packet(raw: bytes, fallback: int) -> int:
        """Device id read out of the packet; the topic is only a fallback.

        NON-NEGOTIABLE #1. `channel_number` is authoritative; where the device
        leaves it outside (0, 1) the transducer bearing (-90 = port,
        +90 = starboard) decides. The two agree on 100 % of packets in every
        field recording measured, so the fallback only ever fires where
        `channel_number` carries nothing usable.
        """
        if (len(raw) > CHANNEL_BYTE
                and int.from_bytes(raw[4:6], "little") == OS_MONO_PROFILE_ID):
            ch = raw[CHANNEL_BYTE]
            if ch in (0, 1):
                return DEVICE_ID_PORT if ch == 0 else DEVICE_ID_STBD
            if len(raw) >= HEADING_BYTE + 4:
                heading, = struct.unpack_from("<f", raw, HEADING_BYTE)
                if heading:
                    return DEVICE_ID_STBD if heading > 0 else DEVICE_ID_PORT
        return fallback

    def _write_raw_with_src_tag(self, msg: UInt8MultiArray, fallback_src: int) -> None:
        if not self._svlog.active:
            return
        raw = bytes(bytearray(msg.data))
        try:
            tagged = retag_packet_src_device_id(
                raw, self._src_from_packet(raw, fallback_src)
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

    # ----- row assembly -----------------------------------------------------
    def _accept(self, msg: OmniscanProfile) -> None:
        """Route one profile into its ping-number group and emit what is ready.

        NON-NEGOTIABLE #2: nothing is discarded here. A group leaves either
        complete (both sides) or one-sided via `_flush_stale`, and every
        arriving ping belongs to exactly one group.

        NON-NEGOTIABLE #1: the side comes from the packet's own
        `channel_number`, not from the subscription that delivered it, so a
        profile published on the wrong topic still lands on the right side.
        """
        channel = self._channel_of(msg)
        stamp_ns = stamp_to_ns(msg.header.stamp)
        ready = []
        with self._buf_lock:
            self._offset.observe(channel, int(msg.ping_number), stamp_ns)

            if self._preroll is not None:
                # Hold until the offset is established, then key the whole
                # pre-roll at once. Keying a ping at a provisional offset
                # would split its row when the real offset arrives.
                self._preroll.append((channel, msg, stamp_ns))
                if (self._offset.votes < OFFSET_MIN_VOTES
                        and len(self._preroll) < OFFSET_PREROLL_MAX):
                    return
                held, self._preroll = self._preroll, None
                for ch, held_msg, held_ns in held:
                    ready.extend(self._insert(ch, held_msg, held_ns))
            else:
                ready.extend(self._insert(channel, msg, stamp_ns))
            ready.extend(self._collect_stale(stamp_ns))
        # Publish outside the lock: _emit_group does the heavy numeric work
        # and must not block the other side's callback.
        for group in ready:
            self._emit_group(group)

    def _insert(self, channel: int, msg: OmniscanProfile, stamp_ns: int) -> list:
        """Place one profile in its group; return the group if it is complete.

        Called under self._buf_lock.
        """
        key = self._offset.key(channel, int(msg.ping_number))
        self._max_key_seen = (key if self._max_key_seen is None
                              else max(self._max_key_seen, key))
        group = self._groups.get(key)
        if group is None:
            group = self._groups[key] = {"first_ns": stamp_ns}
        group[channel] = msg
        if 0 in group and 1 in group:
            del self._groups[key]
            return [group]
        return []

    def _collect_stale(self, now_ns: int) -> list:
        """Remove groups that have waited long enough to be emitted one-sided.

        Called under self._buf_lock. Bounded three ways, so no group is ever
        held indefinitely: by how far its ping number has fallen behind the
        newest one seen, by wall clock, and by a hard cap on live groups.
        """
        out = []
        while self._groups:
            key, group = next(iter(self._groups.items()))
            behind = (self._max_key_seen is not None
                      and self._max_key_seen - key > ASSEMBLY_MAX_LAG_PINGS)
            stale = now_ns - group["first_ns"] > ASSEMBLY_MAX_LAG_NS
            over_cap = len(self._groups) > ASSEMBLY_MAX_GROUPS
            if not (behind or stale or over_cap):
                break
            del self._groups[key]
            out.append(group)
        return out

    def _flush_pending(self, drain_all: bool = False) -> None:
        """Timer/shutdown path: emit groups the arrival path can no longer reach.

        Without this the last group of a run would sit in the buffer until
        the next ping, which may never come -- the stream stopping is exactly
        when it must not be lost.
        """
        now_ns = self.get_clock().now().nanoseconds
        with self._buf_lock:
            ready = []
            # A run shorter than the pre-roll would otherwise strand every
            # ping it produced. Only force it once it has gone stale, though:
            # draining a pre-roll that is still filling would key it at a
            # half-learned offset, which is what it exists to avoid.
            if self._preroll and (drain_all or
                                  now_ns - self._preroll[0][2] > ASSEMBLY_MAX_LAG_NS):
                held, self._preroll = self._preroll, None
                for ch, msg, ns in held:
                    ready.extend(self._insert(ch, msg, ns))
            if drain_all:
                ready.extend(self._groups.values())
                self._groups.clear()
            else:
                ready.extend(self._collect_stale(now_ns))
        for group in ready:
            self._emit_group(group)

    # ----- processing -------------------------------------------------------
    @staticmethod
    def _channel_of(msg: OmniscanProfile) -> int:
        """Side identity (0 = port, 1 = starboard) from the message itself.

        NON-NEGOTIABLE #1, and the same rule the svlog writer applies to raw
        frames: `channel_number` decides, with the transducer bearing as the
        fallback for devices that leave it outside (0, 1). `msg.side` is a
        label attached by the acquisition worker that published it and is
        deliberately not read, nor is the topic it arrived on.
        """
        ch = msg.channel_number
        if ch not in (0, 1):
            ch = 1 if msg.transducer_heading_deg > 0 else 0
        return ch

    @classmethod
    def _side_geometry(cls, msg: OmniscanProfile) -> Tuple[float, float]:
        """(side_sign, transducer y offset) from the message itself.

        Sign convention on the wire is +y = port, -y = starboard.
        """
        if cls._channel_of(msg) == 0:
            return +1.0, TRANSDUCER_Y_OFFSET_PORT_M
        return -1.0, TRANSDUCER_Y_OFFSET_STBD_M

    def _fbr_of(self, msg: Optional[OmniscanProfile]):
        """(dB samples, FBR slant range) for one side, or (None, None)."""
        if msg is None:
            return None, None
        db = scale_to_db(msg.pwr_results, msg.min_pwr_db, msg.max_pwr_db)
        alt = detect_fbr_slant_m(
            db, msg.start_mm, msg.length_mm, msg.num_results,
            noise_floor_window=NOISE_FLOOR_WINDOW,
            threshold_delta_db=FBR_THRESHOLD_DELTA_DB,
            persistence=WITHIN_PING_PERSISTENCE,
            ringing_search_max=RINGING_SEARCH_MAX,
            ringing_drop_db=RINGING_DROP_DB,
            ringing_persistence=RINGING_PERSISTENCE,
        )
        return db, alt

    def _emit_group(self, group: dict) -> None:
        """Publish one assembled row. May carry one side only.

        ONE-SIDED ROW CONVENTION (the message type is fixed -- CM-1, so this
        is expressed in the existing fields): the absent side has
        `*_ping_number = 0`, a zeroed `*_stamp`, and empty `*_intensity_db` /
        `*_y`. **A consumer tests presence with `*_ping_number != 0`**, not
        with array length: `project_side` can legitimately return an empty
        array for a side that IS present, when the altitude estimate exceeds
        the whole swath.
        """
        log = self.get_logger()
        port: Optional[OmniscanProfile] = group.get(0)
        stbd: Optional[OmniscanProfile] = group.get(1)
        ref = port if port is not None else stbd
        if ref is None:                      # defensive; groups always hold one
            return

        # 1. Odom is a hard gate: a ping with no pose is unplaceable. This is
        #    the one legitimate drop, and it is owned by BlueBoat-Control
        #    (/blueboat/odom reads zero on the real boat) rather than here.
        if not self._odom_buf.has_data():
            self._dropped_no_odom += 1
            if self._dropped_no_odom == 1 or self._dropped_no_odom % 20 == 0:
                log.warn(
                    f"dropping ping: no /blueboat/odom yet "
                    f"(total dropped: {self._dropped_no_odom})"
                )
            return

        # 2. dB conversion + FBR detection, per side that is present.
        port_db, port_alt = self._fbr_of(port)
        stbd_db, stbd_alt = self._fbr_of(stbd)

        # 3. Altitude. The tracker never withholds: locked, else provisional,
        #    else last known. `None` means nothing has ever been detected, and
        #    resolves to 0.0 -- no slant correction, ground range = slant
        #    range. That is an identity transform, not a fabricated altitude,
        #    and it matches the GCS "Depth comp. = off" value exactly.
        #    NON-NEGOTIABLE #2: no ping is withheld while the tracker
        #    bootstraps.
        altitude = self._fbr.update(port_alt, stbd_alt)
        if altitude is None:
            altitude = 0.0
        if not self._fbr.locked:
            self._unlocked_pings += 1
            if self._unlocked_pings == 1 or self._unlocked_pings % 50 == 0:
                log.info(
                    f"FBR not locked: {self._unlocked_pings} ping(s) emitted with a "
                    f"provisional altitude so far (port_fbr={port_alt}, "
                    f"stbd_fbr={stbd_alt}); none dropped"
                )
        elif not self._already_bootstrapped_logged:
            log.info(f"FBR locked: altitude = {altitude:.2f} m above seabed")
            self._already_bootstrapped_logged = True

        # 4. Water depth = transducer altitude + submersion.
        water_depth = altitude + TRANSDUCER_SUBMERSION_M

        # 5. Slant-range correct each present side; drop water-column samples.
        #    The geometry comes from each message's own channel_number, not
        #    from the subscription it arrived on (NON-NEGOTIABLE #1), so a
        #    packet delivered to the wrong topic still lands on the right side.
        port_y, port_int = [], []
        stbd_y, stbd_int = [], []
        if port is not None:
            sign, offset = self._side_geometry(port)
            port_y, port_int = project_side(
                port_db, port.start_mm, port.length_mm, port.num_results,
                altitude_m=altitude,
                transducer_y_offset_m=offset,
                side_sign=sign,
            )
        if stbd is not None:
            sign, offset = self._side_geometry(stbd)
            stbd_y, stbd_int = project_side(
                stbd_db, stbd.start_mm, stbd.length_mm, stbd.num_results,
                altitude_m=altitude,
                transducer_y_offset_m=offset,
                side_sign=sign,
            )

        # 6. Snap robot pose from whichever side is present. Both halves of a
        #    complete row are the same acquisition instant, so either stamp
        #    resolves to the same odom sample.
        odom = self._odom_buf.nearest(stamp_to_ns(ref.header.stamp))
        if odom is None:
            self._dropped_no_odom += 1
            if self._dropped_no_odom == 1 or self._dropped_no_odom % 20 == 0:
                dt = self._odom_buf.last_refused_dt_ns
                detail = ("" if dt is None else
                          f" (nearest odom is {dt / 1e9:.1f} s away — "
                          "stale topic or a sonar/odom clock mismatch)")
                log.warn(
                    f"dropping ping: no usable /blueboat/odom pose{detail}; "
                    f"total dropped: {self._dropped_no_odom}")
            return

        # 7. Assemble + publish.
        zero_stamp = TimeMsg()
        out = ProcessedSSSPing()
        out.port_stamp = port.header.stamp if port is not None else zero_stamp
        out.starboard_stamp = stbd.header.stamp if stbd is not None else zero_stamp
        out.port_ping_number = port.ping_number if port is not None else 0
        out.starboard_ping_number = stbd.ping_number if stbd is not None else 0
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

        self._emitted += 1
        if port is None or stbd is None:
            self._one_sided += 1


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
