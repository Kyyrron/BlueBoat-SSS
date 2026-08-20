#!/usr/bin/env python3
"""
Convert a SonarView .svlog file into a ROS 2 rosbag2 directory replayable
into sss_processor_node, bypassing sss_node entirely.

The output bag carries every topic sss_processor consumes -- nothing more,
nothing less -- so feeding it back through the processor with logging
enabled produces a fresh .svlog whose content matches the original.

Topics written
--------------
    /side_scan_sonar/port/profile         blueboat_interfaces/OmniscanProfile
    /side_scan_sonar/port/raw             std_msgs/UInt8MultiArray
    /side_scan_sonar/starboard/profile    blueboat_interfaces/OmniscanProfile
    /side_scan_sonar/starboard/raw        std_msgs/UInt8MultiArray
    /mavros/imu/data                      sensor_msgs/Imu
    /mavros/global_position/global        sensor_msgs/NavSatFix
    /mavros/global_position/rel_alt       std_msgs/Float64
    /mavros/global_position/compass_hdg   std_msgs/Float64
    /mavros/local_position/pose           geometry_msgs/PoseStamped
    /mavros/local_position/velocity_local geometry_msgs/TwistStamped
    /mavros/home_position/home            mavros_msgs/HomePosition
    /mavros/global_position/gp_origin     geographic_msgs/GeoPointStamped
    /mavros/vfr_hud                       mavros_msgs/VfrHud
    /blueboat/odom                        nav_msgs/Odometry  (synthesized)

Mavlink types that don't affect SonarView geometry (HEARTBEAT,
AUTOPILOT_VERSION, PARAM_VALUE, STATUSTEXT) are skipped.

Timestamping
------------
Messages from the same mavlink burst (sharing the same `time_boot_ms` in
the source file) are assigned the SAME ROS header.stamp. SonarView pairs
ATTITUDE+GLOBAL_POSITION_INT+LOCAL_POSITION_NED by `time_boot_ms` to
compute heading-corrected position; the processor downstream derives
`time_boot_ms` back from `header.stamp` -- preserving the grouping is
critical for the "no waterfall" view to show the boat's actual trajectory.

Usage
-----
    Source environements & just run the script; it writes to ./output_bag, which must not exist.
"""

from __future__ import annotations

import json
import math
import os
import struct
import sys
from pathlib import Path
from typing import Iterator, Optional, Tuple

import rclpy
from rclpy.serialization import serialize_message

from rosbag2_py import (
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    TopicMetadata,
)

from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Float64, UInt8MultiArray
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import HomePosition, VfrHud

from blueboat_interfaces.msg import OmniscanProfile

script_dir = os.path.dirname(os.path.abspath(__file__))
custom_libs = os.path.join(script_dir, '..', 'blueboat_sss', 'src', '_custom_libraries')
print(custom_libs)
sys.path.insert(0, os.path.abspath(custom_libs))

from svlog_helper import (
    DEVICE_ID_PORT,
    DEVICE_ID_STBD,
    MAVLINK_WRAPPER_ID,
    OS_MONO_PROFILE_ID,
    retag_packet_src_device_id,
)

# Start this file in /ros2_ws, or from anywhere, but care about these paths
# ----------------------------------------------------------------------------- 
INPUT_FILE = "data/SSS_data/2026_07_08-14_49/2026-07-08-14-56-01.svlog"
OUTPUT_BAG = "src/BlueBoat-SideScanSonar/sim" # If exists, gonna be renamed by adding _2, _3, etc.
# -----------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Topic names (match what sss_processor_node subscribes to)
# ---------------------------------------------------------------------------
TOPIC_PORT_PROFILE = "/side_scan_sonar/port/profile"
TOPIC_STBD_PROFILE = "/side_scan_sonar/starboard/profile"
TOPIC_PORT_RAW     = "/side_scan_sonar/port/raw"
TOPIC_STBD_RAW     = "/side_scan_sonar/starboard/raw"
TOPIC_IMU          = "/mavros/imu/data"
TOPIC_NAVSAT       = "/mavros/global_position/global"
TOPIC_REL_ALT      = "/mavros/global_position/rel_alt"
TOPIC_COMPASS_HDG  = "/mavros/global_position/compass_hdg"
TOPIC_LOCAL_POSE   = "/mavros/local_position/pose"
TOPIC_LOCAL_VEL    = "/mavros/local_position/velocity_local"
TOPIC_HOME_POS     = "/mavros/home_position/home"
TOPIC_GP_ORIGIN    = "/mavros/global_position/gp_origin"
TOPIC_VFR_HUD      = "/mavros/vfr_hud"
TOPIC_ODOM         = "/blueboat/odom"

# Synthetic ROS-clock increment per source-time "tick". Mavlink bursts that
# share a `time_boot_ms` in the source file all get the same stamp; the
# clock only advances when a new tick is observed.
NS_PER_TICK: int = 20_000_000   # 20 ms, well under the 50 ms processor tol

# Fixed epoch for synthesized stamps (Jan 1 2024 00:00:00 UTC).
EPOCH_NS: int = 1_704_067_200 * 1_000_000_000

# OS_MONO_PROFILE payload format (per brping Omniscan450 template).
_OS_MONO_PROFILE_HEAD_FMT: str = "<IIIIIHHHBBffffff"
_OS_MONO_PROFILE_HEAD_SIZE: int = struct.calcsize(_OS_MONO_PROFILE_HEAD_FMT)


# ---------------------------------------------------------------------------
# Pure-data helpers (no ROS deps; unit-testable in isolation)
# ---------------------------------------------------------------------------
def walk_packets(data: bytes) -> Iterator[bytes]:
    """Yield framed packet bytes from a svlog stream, skipping junk."""
    pos = 0
    n = len(data)
    while pos < n - 8:
        if data[pos:pos + 2] != b"BR":
            pos += 1
            continue
        plen = struct.unpack_from("<H", data, pos + 2)[0]
        total = 8 + plen + 2
        if pos + total > n:
            return  # truncated tail
        yield data[pos:pos + total]
        pos += total


def decode_os_mono_profile(payload: bytes) -> dict:
    """Decode an OS_MONO_PROFILE payload (packet_id=2198) to named fields."""
    if len(payload) < _OS_MONO_PROFILE_HEAD_SIZE:
        raise ValueError(
            f"OS_MONO_PROFILE payload too short: {len(payload)} bytes"
        )
    (ping_number, start_mm, length_mm, timestamp_ms, ping_hz,
     gain_index, num_results, sos_dmps, channel_number, _reserved,
     pulse_duration_sec, analog_gain, max_pwr_db, min_pwr_db,
     transducer_heading_deg, vehicle_heading_deg) = struct.unpack(
        _OS_MONO_PROFILE_HEAD_FMT,
        payload[:_OS_MONO_PROFILE_HEAD_SIZE],
    )
    expected = _OS_MONO_PROFILE_HEAD_SIZE + 2 * num_results
    if len(payload) < expected:
        raise ValueError(
            f"OS_MONO_PROFILE payload truncated: "
            f"expect {expected} bytes for {num_results} samples, got {len(payload)}"
        )
    pwr_results = list(struct.unpack(
        f"<{num_results}H",
        payload[_OS_MONO_PROFILE_HEAD_SIZE:_OS_MONO_PROFILE_HEAD_SIZE + 2 * num_results],
    ))
    return {
        "ping_number": ping_number, "start_mm": start_mm,
        "length_mm": length_mm, "timestamp_ms": timestamp_ms,
        "ping_hz": ping_hz, "gain_index": gain_index,
        "num_results": num_results, "sos_dmps": sos_dmps,
        "channel_number": channel_number,
        "pulse_duration_sec": pulse_duration_sec,
        "analog_gain": analog_gain,
        "max_pwr_db": max_pwr_db, "min_pwr_db": min_pwr_db,
        "transducer_heading_deg": transducer_heading_deg,
        "vehicle_heading_deg": vehicle_heading_deg,
        "pwr_results": pwr_results,
    }


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """ZYX intrinsic Euler -> (x, y, z, w) quaternion (ROS convention)."""
    cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def ned_to_enu_xyz(x_n: float, y_e: float, z_d: float) -> Tuple[float, float, float]:
    """Mavlink NED -> ROS REP-103 ENU."""
    return (y_e, x_n, -z_d)


def ned_to_enu_rpy(roll_ned: float, pitch_ned: float, yaw_ned: float
                   ) -> Tuple[float, float, float]:
    """Convert NED-frame body roll/pitch/yaw (mavlink ATTITUDE) to ENU.

    Mavlink ATTITUDE is in NED world / FRD body conventions:
      yaw_ned = 0 -> nose points North,  yaw increases clockwise (toward East)
    ROS REP-103 conventions are ENU world / FLU body:
      yaw_enu = 0 -> nose points East,   yaw increases counter-clockwise (toward N)

    The transformation between the two body/world conventions reduces to:
        roll_enu  =  roll_ned
        pitch_enu = -pitch_ned
        yaw_enu   =  π/2 - yaw_ned
    (See REP-103 + REP-105; e.g. mavros uses this same rotation when
    republishing autopilot attitude on /mavros/imu/data.)
    """
    return (roll_ned, -pitch_ned, math.pi / 2.0 - yaw_ned)


def ns_to_time_msg(ns: int) -> TimeMsg:
    t = TimeMsg()
    t.sec = ns // 1_000_000_000
    t.nanosec = ns % 1_000_000_000
    return t


# ---------------------------------------------------------------------------
# Converter
# ---------------------------------------------------------------------------
class Converter:
    """Stateful svlog -> rosbag2 converter."""

    def __init__(self, writer: SequentialWriter) -> None:
        self._writer = writer

        # Source-time-aware clock. `_clock_ns` is the stamp we assign to
        # the next message; it advances only when we observe a new "tick"
        # (different time_boot_ms, or a non-mavlink packet). Mavlink
        # messages in the same burst (same time_boot_ms) share a stamp.
        self._clock_ns = EPOCH_NS
        self._last_time_boot_ms: Optional[int] = None

        # Latest values used to synthesize /blueboat/odom (ATTITUDE for
        # orientation, LOCAL_POSITION_NED for translation/velocity).
        self._latest_quat: Optional[Tuple[float, float, float, float]] = None
        self._latest_lpn:  Optional[dict] = None

        self.counts = {
            "sonar_port":            0,
            "sonar_stbd":            0,
            "imu":                   0,
            "navsat":                0,
            "rel_alt":               0,
            "compass_hdg":           0,
            "local_pose":            0,
            "local_velocity":        0,
            "home_position":         0,
            "gp_origin":             0,
            "vfr_hud":               0,
            "odom_synthesized":      0,
            "mavlink_skipped_type":  0,
            "skipped_non_target":    0,
            "decode_errors":         0,
        }

    # ----- clock --------------------------------------------------------
    def _tick(self, time_boot_ms: Optional[int]) -> int:
        """Return a ROS stamp for the current packet.

        Mavlink messages from the same burst share a `time_boot_ms` in the
        source file; we give them all the same stamp so the processor can
        later recover `time_boot_ms` coherence by stamping the rebuilt
        envelope from `header.stamp`. Any new value of `time_boot_ms`
        advances the clock; `None` (non-mavlink, or mavlink without a
        timestamp) always advances.
        """
        if time_boot_ms is not None and time_boot_ms == self._last_time_boot_ms:
            return self._clock_ns
        self._clock_ns += NS_PER_TICK
        self._last_time_boot_ms = time_boot_ms
        return self._clock_ns

    # ----- topic setup --------------------------------------------------
    def setup_topics(self) -> None:
        topics = [
            (TOPIC_PORT_PROFILE, "blueboat_interfaces/msg/OmniscanProfile"),
            (TOPIC_STBD_PROFILE, "blueboat_interfaces/msg/OmniscanProfile"),
            (TOPIC_PORT_RAW,     "std_msgs/msg/UInt8MultiArray"),
            (TOPIC_STBD_RAW,     "std_msgs/msg/UInt8MultiArray"),
            (TOPIC_IMU,          "sensor_msgs/msg/Imu"),
            (TOPIC_NAVSAT,       "sensor_msgs/msg/NavSatFix"),
            (TOPIC_REL_ALT,      "std_msgs/msg/Float64"),
            (TOPIC_COMPASS_HDG,  "std_msgs/msg/Float64"),
            (TOPIC_LOCAL_POSE,   "geometry_msgs/msg/PoseStamped"),
            (TOPIC_LOCAL_VEL,    "geometry_msgs/msg/TwistStamped"),
            (TOPIC_HOME_POS,     "mavros_msgs/msg/HomePosition"),
            (TOPIC_GP_ORIGIN,    "geographic_msgs/msg/GeoPointStamped"),
            (TOPIC_VFR_HUD,      "mavros_msgs/msg/VfrHud"),
            (TOPIC_ODOM,         "nav_msgs/msg/Odometry"),
        ]
        for name, type_name in topics:
            self._writer.create_topic(
                TopicMetadata(
                    id=0,
                    name=name,
                    type=type_name,
                    serialization_format="cdr",
                    offered_qos_profiles=[]
                )
            )

    # ----- packet entry point -------------------------------------------
    def handle_packet(self, packet: bytes) -> None:
        pid = struct.unpack_from("<H", packet, 4)[0]
        src = packet[6]
        payload = packet[8:-2]  # strip 8-byte header and 2-byte checksum

        if pid == OS_MONO_PROFILE_ID:
            self._handle_sonar(packet, src, payload)
        elif pid == MAVLINK_WRAPPER_ID:
            try:
                envelope = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.counts["decode_errors"] += 1
                return
            self._handle_mavlink(envelope)
        else:
            # JSON_META, VIEW_CONFIG, Omniscan settings (2194), EOS (0).
            self.counts["skipped_non_target"] += 1

    # ----- sonar --------------------------------------------------------
    def _handle_sonar(self, packet: bytes, src: int, payload: bytes) -> None:
        # `side` is the OmniscanProfile.side string -- must match what
        # sss_node publishes ("port" / "stbd"), not the topic-path name.
        if src == DEVICE_ID_PORT:
            side, frame_id = "port", "sss_port_link"
            topic_prof, topic_raw = TOPIC_PORT_PROFILE, TOPIC_PORT_RAW
            count_key = "sonar_port"
        elif src == DEVICE_ID_STBD:
            side, frame_id = "stbd", "sss_starboard_link"
            topic_prof, topic_raw = TOPIC_STBD_PROFILE, TOPIC_STBD_RAW
            count_key = "sonar_stbd"
        else:
            self.counts["skipped_non_target"] += 1
            return

        try:
            d = decode_os_mono_profile(payload)
        except ValueError:
            self.counts["decode_errors"] += 1
            return

        # Sonar packets always advance the clock (not part of a mavlink burst).
        stamp_ns = self._tick(None)
        stamp = ns_to_time_msg(stamp_ns)

        prof = OmniscanProfile()
        prof.header.stamp = stamp
        prof.header.frame_id = frame_id
        prof.side                  = side
        prof.ping_number           = d["ping_number"]
        prof.start_mm              = d["start_mm"]
        prof.length_mm             = d["length_mm"]
        prof.timestamp_ms          = d["timestamp_ms"]
        prof.ping_hz               = d["ping_hz"]
        prof.gain_index            = d["gain_index"]
        prof.num_results           = d["num_results"]
        prof.sos_dmps              = d["sos_dmps"]
        prof.channel_number        = d["channel_number"]
        prof.pulse_duration_sec    = float(d["pulse_duration_sec"])
        prof.analog_gain           = float(d["analog_gain"])
        prof.max_pwr_db            = float(d["max_pwr_db"])
        prof.min_pwr_db            = float(d["min_pwr_db"])
        prof.transducer_heading_deg = float(d["transducer_heading_deg"])
        prof.vehicle_heading_deg   = float(d["vehicle_heading_deg"])
        prof.pwr_results           = d["pwr_results"]

        # Raw bytes: retag src back to 0 to mirror what sss_node publishes
        # (the Omniscan emits src=0; the file has it retagged to 1/2 by
        # the writer at recording time).
        raw = UInt8MultiArray()
        raw.data = list(retag_packet_src_device_id(packet, 0))

        self._writer.write(topic_prof, serialize_message(prof), stamp_ns)
        self._writer.write(topic_raw,  serialize_message(raw),  stamp_ns)
        self.counts[count_key] += 1

    # ----- mavlink dispatch ---------------------------------------------
    def _handle_mavlink(self, envelope: dict) -> None:
        m = envelope.get("message", {})
        msg_type = m.get("type")
        time_boot_ms = m.get("time_boot_ms")  # may be None
        stamp_ns = self._tick(time_boot_ms)

        if msg_type == "ATTITUDE":
            self._emit_attitude(m, stamp_ns)
        elif msg_type == "GLOBAL_POSITION_INT":
            self._emit_global_position(m, stamp_ns)
        elif msg_type == "LOCAL_POSITION_NED":
            self._emit_local_position(m, stamp_ns)
        elif msg_type == "HOME_POSITION":
            self._emit_home_position(m, stamp_ns)
        elif msg_type == "GPS_GLOBAL_ORIGIN":
            self._emit_gp_origin(m, stamp_ns)
        elif msg_type == "VFR_HUD":
            self._emit_vfr_hud(m, stamp_ns)
        else:
            # HEARTBEAT, AUTOPILOT_VERSION, PARAM_VALUE, STATUSTEXT --
            # don't affect SonarView geometry; safe to skip.
            self.counts["mavlink_skipped_type"] += 1

    # ----- mavlink emitters ---------------------------------------------
    def _emit_attitude(self, m: dict, stamp_ns: int) -> None:
        # mavlink ATTITUDE is in NED world / FRD body frame; mavros (on the
        # real robot) republishes /mavros/imu/data in ENU/FLU per REP-103,
        # so we replicate that rotation here for replay consistency.
        roll_ned  = float(m.get("roll",  0.0))
        pitch_ned = float(m.get("pitch", 0.0))
        yaw_ned   = float(m.get("yaw",   0.0))
        roll, pitch, yaw = ned_to_enu_rpy(roll_ned, pitch_ned, yaw_ned)
        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)
        self._latest_quat = (qx, qy, qz, qw)

        imu = Imu()
        imu.header.stamp = ns_to_time_msg(stamp_ns)
        imu.header.frame_id = "base_link"
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        # Body-frame angular velocity: FRD -> FLU is also (x, -y, -z).
        imu.angular_velocity.x =  float(m.get("rollspeed",  0.0))
        imu.angular_velocity.y = -float(m.get("pitchspeed", 0.0))
        imu.angular_velocity.z = -float(m.get("yawspeed",   0.0))
        self._writer.write(TOPIC_IMU, serialize_message(imu), stamp_ns)
        self.counts["imu"] += 1
        self._maybe_publish_odom(stamp_ns)

    def _emit_global_position(self, m: dict, stamp_ns: int) -> None:
        stamp = ns_to_time_msg(stamp_ns)

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "base_link"
        fix.status.status  = 0   # STATUS_FIX
        fix.status.service = 1   # SERVICE_GPS
        fix.latitude  = float(m.get("lat", 0)) / 1e7
        fix.longitude = float(m.get("lon", 0)) / 1e7
        fix.altitude  = float(m.get("alt", 0)) / 1000.0      # mm AMSL -> m
        self._writer.write(TOPIC_NAVSAT, serialize_message(fix), stamp_ns)
        self.counts["navsat"] += 1

        rel = Float64(); rel.data = float(m.get("relative_alt", 0)) / 1000.0
        self._writer.write(TOPIC_REL_ALT, serialize_message(rel), stamp_ns)
        self.counts["rel_alt"] += 1

        hdg = Float64(); hdg.data = float(m.get("hdg", 0)) / 100.0  # cdeg -> deg
        self._writer.write(TOPIC_COMPASS_HDG, serialize_message(hdg), stamp_ns)
        self.counts["compass_hdg"] += 1

    def _emit_local_position(self, m: dict, stamp_ns: int) -> None:
        # Cache for /blueboat/odom synthesis.
        self._latest_lpn = {
            "x":  float(m.get("x",  0.0)),
            "y":  float(m.get("y",  0.0)),
            "z":  float(m.get("z",  0.0)),
            "vx": float(m.get("vx", 0.0)),
            "vy": float(m.get("vy", 0.0)),
            "vz": float(m.get("vz", 0.0)),
        }
        stamp = ns_to_time_msg(stamp_ns)
        ex, ey, ez   = ned_to_enu_xyz(self._latest_lpn["x"],
                                      self._latest_lpn["y"],
                                      self._latest_lpn["z"])
        vex, vey, vez = ned_to_enu_xyz(self._latest_lpn["vx"],
                                       self._latest_lpn["vy"],
                                       self._latest_lpn["vz"])

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = ex
        pose.pose.position.y = ey
        pose.pose.position.z = ez
        if self._latest_quat is not None:
            qx, qy, qz, qw = self._latest_quat
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
        else:
            pose.pose.orientation.w = 1.0
        self._writer.write(TOPIC_LOCAL_POSE, serialize_message(pose), stamp_ns)
        self.counts["local_pose"] += 1

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = vex
        twist.twist.linear.y = vey
        twist.twist.linear.z = vez
        self._writer.write(TOPIC_LOCAL_VEL, serialize_message(twist), stamp_ns)
        self.counts["local_velocity"] += 1

        self._maybe_publish_odom(stamp_ns)

    def _emit_home_position(self, m: dict, stamp_ns: int) -> None:
        h = HomePosition()
        h.header.stamp = ns_to_time_msg(stamp_ns)
        h.header.frame_id = "map"
        h.geo.latitude  = float(m.get("latitude",  0)) / 1e7
        h.geo.longitude = float(m.get("longitude", 0)) / 1e7
        h.geo.altitude  = float(m.get("altitude",  0)) / 1000.0  # mm -> m
        # The mavros HomePosition has additional local-frame fields not
        # populated from the source mavlink message; leave them at default.
        self._writer.write(TOPIC_HOME_POS, serialize_message(h), stamp_ns)
        self.counts["home_position"] += 1

    def _emit_gp_origin(self, m: dict, stamp_ns: int) -> None:
        g = GeoPointStamped()
        g.header.stamp = ns_to_time_msg(stamp_ns)
        g.header.frame_id = "map"
        g.position.latitude  = float(m.get("latitude",  0)) / 1e7
        g.position.longitude = float(m.get("longitude", 0)) / 1e7
        g.position.altitude  = float(m.get("altitude",  0)) / 1000.0  # mm -> m
        self._writer.write(TOPIC_GP_ORIGIN, serialize_message(g), stamp_ns)
        self.counts["gp_origin"] += 1

    def _emit_vfr_hud(self, m: dict, stamp_ns: int) -> None:
        v = VfrHud()
        v.header.stamp = ns_to_time_msg(stamp_ns)
        v.header.frame_id = "base_link"
        v.airspeed    = float(m.get("airspeed",    0.0))
        v.groundspeed = float(m.get("groundspeed", 0.0))
        v.heading     = int(m.get("heading", 0))
        v.throttle    = float(m.get("throttle", 0)) / 100.0
        v.altitude    = float(m.get("alt",   0.0))
        v.climb       = float(m.get("climb", 0.0))
        self._writer.write(TOPIC_VFR_HUD, serialize_message(v), stamp_ns)
        self.counts["vfr_hud"] += 1

    # ----- odom synthesis -----------------------------------------------
    def _maybe_publish_odom(self, stamp_ns: int) -> None:
        if self._latest_quat is None or self._latest_lpn is None:
            return
        qx, qy, qz, qw = self._latest_quat
        px, py, pz = ned_to_enu_xyz(self._latest_lpn["x"],
                                    self._latest_lpn["y"],
                                    self._latest_lpn["z"])
        vx, vy, vz = ned_to_enu_xyz(self._latest_lpn["vx"],
                                    self._latest_lpn["vy"],
                                    self._latest_lpn["vz"])

        odom = Odometry()
        odom.header.stamp = ns_to_time_msg(stamp_ns)
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        self._writer.write(TOPIC_ODOM, serialize_message(odom), stamp_ns)
        self.counts["odom_synthesized"] += 1


def main() -> None:

    input_path = Path(INPUT_FILE)
    output_path = Path(OUTPUT_BAG)

    if not input_path.is_file():
        sys.exit("input file not found")

    if output_path.exists():
        print(f"output path {output_path} already exists; renaming with _2, _3, etc.")
        i = 2
        new_path = output_path.with_name(f"{output_path.stem}_{i}{output_path.suffix}")
        while new_path.exists():
            i += 1
            new_path = output_path.with_name(f"{output_path.stem}_{i}{output_path.suffix}")

        output_path = new_path
    rclpy.init()

    try:
        writer = SequentialWriter()

        writer.open(
            StorageOptions(
                uri=str(output_path),
                storage_id="mcap",
            ),
            ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )

        conv = Converter(writer)
        conv.setup_topics()

        data = input_path.read_bytes()
        n = 0

        for packet in walk_packets(data):
            conv.handle_packet(packet)
            n += 1
        # Force the writer to close cleanly before rclpy shutdown.
        del writer

        print(f"Converted {n} packets from {input_path.name} -> {output_path.name}")
        for k, v in sorted(conv.counts.items()):
            print(f"  {k:>22s}: {v}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()