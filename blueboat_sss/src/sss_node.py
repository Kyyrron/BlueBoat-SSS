#!/usr/bin/env python3
"""
Side Scan Sonar (SSS) ROS 2 node for the BlueBoat.

Drives the pair of Cerulean Omniscan 450 SS devices (port + starboard) and
republishes what they send on two ROS topics per side:

  * parsed `OmniscanProfile` -- for downstream processing,
  * raw framed Ping-Protocol bytes (`UInt8MultiArray`) -- for the processor
    node to interleave into a SonarView .svlog file. Every device frame goes
    here, not only the profiles: the Omniscan status frames (id 2194) that
    SonarView records reach the .svlog the same way.

Each device is one `OmniscanLink` (`_custom_libraries/omniscan_link.py`): its
own socket, receive buffer and framer, in its own thread. Nothing is shared
between the two sides -- `brping`, which this node used until 2026-10-27,
shared one receive buffer between both devices (see omniscan_link.py).

Pinging is OFF when the node starts. Connections are opened immediately so
the devices are ready to fire on demand. On-disk logging is not this node's
responsibility -- the processor node owns the .svlog file.

Topics
------
Pub  ~/port/profile         blueboat_interfaces/OmniscanProfile
Pub  ~/port/raw             std_msgs/UInt8MultiArray
Pub  ~/starboard/profile    blueboat_interfaces/OmniscanProfile
Pub  ~/starboard/raw        std_msgs/UInt8MultiArray
Sub  ~/ping/enable          std_msgs/Bool   true=start, false=stop

Run-dependent parameters
------------------------
Acquisition    range_start_mm, range_length_mm, msec_per_ping, gain_index,
               num_results, pulse_len_percent

Acquisition parameters are re-read each time pinging is enabled, so
`ros2 param set ...` between runs takes effect on the next "start".
"""

from __future__ import annotations

import array
import os
import sys
import threading
import time
from collections import Counter
from dataclasses import dataclass
from typing import Optional

# Flat-file imports, as in sss_processor_node.py: CMakeLists installs the
# helpers next to this script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, UInt8MultiArray

from blueboat_interfaces.msg import OmniscanProfile

from omniscan_link import (
    COMMON_NACK_ID,
    OMNISCAN_STATUS_ID,
    PROTOCOL_REPLY_IDS,
    OmniscanLink,
    decode_profile,
)
from svlog_helper import OS_MONO_PROFILE_ID


# ---------------------------------------------------------------------------
# Pinned (non run-dependent) parameters
# ---------------------------------------------------------------------------
FILTER_DURATION_PERCENT: float = 0.0015     # Cerulean docs: 0.0015 typical
WORKER_JOIN_TIMEOUT_S:   float = 5.0
RECONNECT_DELAY_S:       float = 2.0        # wait between reconnect attempts
HEALTH_PERIOD_S:         float = 10.0       # one health line per side while pinging
TCP_PORT:                int   = 51200


@dataclass(frozen=True)
class PingParams:
    """Run-dependent ping configuration shared by both transducers."""

    start_mm: int
    length_mm: int
    msec_per_ping: int
    gain_index: int
    num_results: int
    pulse_len_percent: float


# ---------------------------------------------------------------------------
# Per-side acquisition health
# ---------------------------------------------------------------------------
class _Health:
    """What one side received, per HEALTH_PERIOD_S window.

    Missing ping numbers counted *here* were lost before ROS (device, WiFi
    link or TCP); the processor's and the GCS's own counts can then only add
    DDS losses on top. That split is what this line exists to make visible.
    """

    def __init__(self) -> None:
        self.reset_window()
        self._last_pn: Optional[int] = None
        self._last_wall_ns: Optional[int] = None

    def reset_window(self) -> None:
        self.profiles = 0
        self.missing = 0
        self.restarts = 0
        self.longest_gap_ns = 0
        self.other_ids: Counter = Counter()

    def forget_sequence(self) -> None:
        """Pinging stopped or the link dropped: the next ping starts afresh."""
        self._last_pn = None
        self._last_wall_ns = None

    def profile(self, ping_number: int, wall_ns: int) -> None:
        self.profiles += 1
        if self._last_pn is not None:
            step = ping_number - self._last_pn
            if 1 < step < 100_000:
                self.missing += step - 1
            elif step <= 0:
                self.restarts += 1          # device power cycle / counter wrap
        if self._last_wall_ns is not None:
            self.longest_gap_ns = max(self.longest_gap_ns,
                                      wall_ns - self._last_wall_ns)
        self._last_pn = ping_number
        self._last_wall_ns = wall_ns


# ---------------------------------------------------------------------------
# Per-device worker
# ---------------------------------------------------------------------------
class OmniscanWorker:
    """Owns one Omniscan 450. Publishes parsed + raw packets for one side.

    Lifecycle is fully driven by the worker thread:
      1. Try to connect. On failure, sleep RECONNECT_DELAY_S and retry.
      2. Once connected, (re-)apply the desired pinging state.
      3. Run the receive loop. If the link dies, tear down and loop back to 1.

    `set_pinging()` updates the *desired* state -- it pushes the command
    to the device immediately if connected, and the desired state is
    automatically re-applied after every reconnection.
    """

    def __init__(
        self,
        node: Node,
        side: str,
        ip: str,
        tcp_port: int,
        profile_publisher,
        raw_publisher,
        frame_id: str,
    ) -> None:
        self._node = node
        self._side = side
        self._ip = ip
        self._tcp_port = tcp_port
        self._profile_pub = profile_publisher
        self._raw_pub = raw_publisher
        self._frame_id = frame_id

        # `_link` is reassigned by the worker thread on connect/disconnect;
        # other threads only sample the reference.
        self._link: Optional[OmniscanLink] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Desired vs applied pinging state. Updated by set_pinging(),
        # consumed by _apply_desired_state_locked(). The lock serializes the
        # check-then-send logic so two callbacks racing don't both
        # trigger a duplicate enable command.
        self._control_lock = threading.Lock()
        self._desired_ping_enable = False
        self._desired_ping_params: Optional[PingParams] = None
        self._currently_pinging = False

        self._health = _Health()
        self._health_t0 = time.monotonic()
        self._reported_ids: set = set()
        self._parser_errors_reported = 0
        self._parser_skipped_reported = 0

    # ----- lifecycle -------------------------------------------------------
    def start(self) -> None:
        """Kick the worker thread. Connection happens asynchronously."""
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._main_loop, name=f"omniscan-{self._side}", daemon=True
        )
        self._thread.start()
        self._node.get_logger().info(
            f"[{self._side}] worker started (will connect to {self._ip}:{self._tcp_port})"
        )

    def stop(self) -> None:
        """Signal stop, tell the device to stop pinging, close, join."""
        self._stop_event.set()
        link = self._link
        if link is not None:
            # Best-effort: leave the device silent. If it fails (already
            # disconnected), the next start resets it on connect.
            try:
                self._send_params(link, enable=False)
            except Exception:  # noqa: BLE001
                pass
            # Closing the socket ends a pending receive immediately.
            link.close()
        if self._thread is not None:
            self._thread.join(timeout=WORKER_JOIN_TIMEOUT_S)
            if self._thread.is_alive():
                self._node.get_logger().warn(
                    f"[{self._side}] worker did not join in {WORKER_JOIN_TIMEOUT_S}s"
                )
        self._link = None

    # ----- control ---------------------------------------------------------
    def set_pinging(self, enable: bool, params: Optional[PingParams] = None) -> None:
        """Update the desired pinging state.

        Pushed to the device immediately if connected; re-applied
        automatically by the worker thread on every reconnect. Calling
        this before the worker has connected for the first time is
        safe -- the state is queued and applied on first connection.
        """
        with self._control_lock:
            self._desired_ping_enable = enable
            if enable and params is not None:
                self._desired_ping_params = params
            self._apply_desired_state_locked()

    def _send_params(self, link: OmniscanLink, enable: bool) -> None:
        # A stop repeats the last configuration with enable=0 rather than
        # brping's library defaults, so the device never holds a stray
        # 5 m / 600-bin configuration between runs.
        p = self._desired_ping_params or PingParams(0, 20000, 50, -1, 1200, 0.00125)
        link.set_ping_params(
            start_mm=p.start_mm,
            length_mm=p.length_mm,
            msec_per_ping=p.msec_per_ping,
            pulse_len_percent=p.pulse_len_percent,
            filter_duration_percent=FILTER_DURATION_PERCENT,
            gain_index=p.gain_index,
            num_results=p.num_results,
            enable=enable,
        )

    def _apply_desired_state_locked(self) -> None:
        """Push the desired state to the device. Call under _control_lock."""
        log = self._node.get_logger()
        link = self._link
        if link is None:
            return  # Not connected yet; will be applied on connect.

        enable = self._desired_ping_enable
        params = self._desired_ping_params
        if enable == self._currently_pinging:
            return  # idempotent

        try:
            if enable:
                if params is None:
                    log.error(f"[{self._side}] cannot start ping without params")
                    return
                self._send_params(link, enable=True)
                log.info(
                    f"[{self._side}] pinging started "
                    f"(range {params.start_mm}-{params.start_mm + params.length_mm} mm, "
                    f"{params.msec_per_ping} ms/ping, gain {params.gain_index}, "
                    f"n={params.num_results})"
                )
                self._health.reset_window()
                self._health_t0 = time.monotonic()
            else:
                self._send_params(link, enable=False)
                log.info(f"[{self._side}] pinging stopped")
            self._health.forget_sequence()
            self._currently_pinging = enable
        except Exception as exc:  # noqa: BLE001
            # Connection probably died mid-command. Don't update
            # _currently_pinging; the next reconnect will resync.
            log.warn(f"[{self._side}] apply ping state failed: {exc}")

    # ----- main loop -------------------------------------------------------
    def _main_loop(self) -> None:
        log = self._node.get_logger()
        while not self._stop_event.is_set() and rclpy.ok():
            # ---- Phase 1: connect (with retries) ----
            link = self._connect()
            if link is None:
                # Cancellable sleep -- returns True if stop was signalled.
                if self._stop_event.wait(RECONNECT_DELAY_S):
                    return
                continue

            # ---- Phase 2: (re-)apply desired ping state ----
            # After connect, the device is in enable=0 (we just sent it).
            with self._control_lock:
                self._link = link
                self._currently_pinging = False
                self._apply_desired_state_locked()

            # ---- Phase 3: receive loop ----
            while not self._stop_event.is_set() and rclpy.ok():
                try:
                    frames = link.receive()
                except ConnectionError as exc:
                    if not self._stop_event.is_set():
                        log.warn(f"[{self._side}] connection lost: {exc}")
                    break  # drop out, reconnect
                for mid, frame in frames:
                    self._dispatch(mid, frame)
                self._maybe_report_health(link)

            # ---- Phase 4: tear down for reconnect ----
            with self._control_lock:
                self._link = None
            link.close()
            self._health.forget_sequence()

    def _connect(self) -> Optional[OmniscanLink]:
        """One connection attempt. Returns the link iff the device answered."""
        log = self._node.get_logger()
        link = OmniscanLink(self._ip, self._tcp_port)
        try:
            if not link.connect():
                log.warn(
                    f"[{self._side}] no device_information from "
                    f"{self._ip}:{self._tcp_port}, will retry"
                )
                return None
            # Start from a known clean state on the device.
            try:
                self._send_params(link, enable=False)
            except Exception:  # noqa: BLE001
                pass
            log.info(
                f"[{self._side}] connected at {self._ip}:{self._tcp_port} "
                f"(device_information {link.device_information})"
            )
            return link
        except Exception as exc:  # noqa: BLE001
            link.close()
            log.warn(
                f"[{self._side}] connect failed at {self._ip}:{self._tcp_port}: "
                f"{exc} (retry in {RECONNECT_DELAY_S}s)"
            )
            return None

    # ----- dispatch ---------------------------------------------------------
    def _dispatch(self, mid: int, frame: bytes) -> None:
        if mid == OS_MONO_PROFILE_ID:
            try:
                fields = decode_profile(frame)
            except ValueError as exc:
                self._node.get_logger().warn(f"[{self._side}] bad profile: {exc}")
                return
            now = self._node.get_clock().now()
            self._health.profile(int(fields["ping_number"]), now.nanoseconds)
            self._publish_profile(fields, now.to_msg())
            self._publish_raw(frame)
            return
        if mid == COMMON_NACK_ID:
            self._node.get_logger().warn(
                f"[{self._side}] device refused a command (NACK {frame[8:-2].hex()})")
            return
        if mid in PROTOCOL_REPLY_IDS:
            return
        # Everything else the device sends is acquisition record: status
        # frames (2194) and anything newer firmware adds. Into the .svlog.
        self._health.other_ids[mid] += 1
        if mid not in self._reported_ids:
            self._reported_ids.add(mid)
            what = "Omniscan status" if mid == OMNISCAN_STATUS_ID else "unknown"
            self._node.get_logger().info(
                f"[{self._side}] device sends message id {mid} ({what}); "
                "forwarded on ~/raw into the .svlog")
        self._publish_raw(frame)

    def _maybe_report_health(self, link: OmniscanLink) -> None:
        if not self._currently_pinging:
            return
        elapsed = time.monotonic() - self._health_t0
        if elapsed < HEALTH_PERIOD_S:
            return
        h = self._health
        errors = link.parser.checksum_errors - self._parser_errors_reported
        skipped = link.parser.skipped_bytes - self._parser_skipped_reported
        self._parser_errors_reported = link.parser.checksum_errors
        self._parser_skipped_reported = link.parser.skipped_bytes
        total = h.profiles + h.missing
        loss = 100.0 * h.missing / total if total else 0.0
        other = ", ".join(f"id {k} x{v}" for k, v in sorted(h.other_ids.items()))
        line = (
            f"[{self._side}] health {elapsed:.0f} s: {h.profiles} profiles "
            f"({h.profiles / elapsed:.1f} Hz), {h.missing} missing ({loss:.1f} %), "
            f"longest gap {h.longest_gap_ns / 1e6:.0f} ms, "
            f"{errors} checksum errors, {skipped} resync bytes"
            + (f", {h.restarts} counter restart(s)" if h.restarts else "")
            + (f"; also {other}" if other else "")
        )
        if h.missing or errors or h.profiles == 0:
            self._node.get_logger().warn(line)
        else:
            self._node.get_logger().info(line)
        h.reset_window()
        self._health_t0 = time.monotonic()
        # The window restarts, the ping sequence does not.

    # ----- publishing ------------------------------------------------------
    def _publish_profile(self, f: dict, stamp) -> None:
        msg = OmniscanProfile()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.side = self._side

        msg.ping_number = int(f["ping_number"])
        msg.start_mm = int(f["start_mm"])
        msg.length_mm = int(f["length_mm"])
        msg.timestamp_ms = int(f["timestamp_ms"])
        msg.ping_hz = int(f["ping_hz"])
        msg.gain_index = int(f["gain_index"])
        msg.num_results = int(f["num_results"])
        msg.sos_dmps = int(f["sos_dmps"])
        msg.channel_number = int(f["channel_number"])
        msg.pulse_duration_sec = float(f["pulse_duration_sec"])
        msg.analog_gain = float(f["analog_gain"])
        msg.max_pwr_db = float(f["max_pwr_db"])
        msg.min_pwr_db = float(f["min_pwr_db"])
        msg.transducer_heading_deg = float(f["transducer_heading_deg"])
        msg.vehicle_heading_deg = float(f["vehicle_heading_deg"])
        msg.pwr_results = f["pwr_results"]      # array('H'), no per-sample copy

        self._profile_pub.publish(msg)

    def _publish_raw(self, frame: bytes) -> None:
        msg = UInt8MultiArray()
        msg.data = array.array("B", frame)
        self._raw_pub.publish(msg)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class SideScanSonarNode(Node):
    """ROS 2 node managing the port + starboard Omniscan 450 SS devices."""

    def __init__(self) -> None:
        super().__init__("side_scan_sonar")

        # ---- Run-dependent parameters --------------------------------------

        self.declare_parameter("range_start_mm", 0)
        self.declare_parameter("range_length_mm", 20000)
        # SonarView's command (2026-10-27): 50 ms, i.e. 20 pings/s up to
        # ~35 m and range-limited (2R/c + ~3 ms) beyond. 0 free-runs the
        # device at 2R/c + ~2.3 ms -- 29 ms (34.5 Hz) at 20 m, measured --
        # which SonarView never does.
        self.declare_parameter("msec_per_ping", 50)
        self.declare_parameter("gain_index", -1)
        # The device's best range resolution (2026-09-28): 1200 bins is its
        # maximum, and pulse = max(44.3 us, pulse_len_percent x 2R/c) (measured
        # on every corpus recording) sits on its 44.3 us floor up to ~27 m at
        # 0.00125 -- SonarView's own 1.5 / num_results. 33 mm true range
        # resolution at 20 m, against 40 mm at the former 600 / 0.002.
        self.declare_parameter("num_results", 1200)
        self.declare_parameter("pulse_len_percent", 0.00125)

        # ---- Publishers (best-effort: sonar is a high-rate lossy stream) ---
        sonar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._port_profile_pub = self.create_publisher(
            OmniscanProfile, "~/port/profile", sonar_qos
        )
        self._port_raw_pub = self.create_publisher(
            UInt8MultiArray, "~/port/raw", sonar_qos
        )
        self._stbd_profile_pub = self.create_publisher(
            OmniscanProfile, "~/starboard/profile", sonar_qos
        )
        self._stbd_raw_pub = self.create_publisher(
            UInt8MultiArray, "~/starboard/raw", sonar_qos
        )

        # ---- Workers --------------------------------------------------------
        self._port_worker = OmniscanWorker(
            self,
            side="port",
            ip="192.168.2.92",
            tcp_port=TCP_PORT,
            profile_publisher=self._port_profile_pub,
            raw_publisher=self._port_raw_pub,
            frame_id="sss_port_link",
        )
        self._starboard_worker = OmniscanWorker(
            self,
            side="starboard",
            ip="192.168.2.93",
            tcp_port=TCP_PORT,
            profile_publisher=self._stbd_profile_pub,
            raw_publisher=self._stbd_raw_pub,
            frame_id="sss_starboard_link",
        )

        # start() kicks the worker threads; connection happens asynchronously
        # with retries, so there's nothing to check for failure here.
        self._port_worker.start()
        self._starboard_worker.start()

        # ---- Control subscriber --------------------------------------------
        self.create_subscription(Bool, "~/ping/enable", self._on_ping_enable, 10)

        self.get_logger().info(
            "side_scan_sonar ready, ping OFF. Toggle with:\n"
            "  ros2 topic pub --once /side_scan_sonar/ping/enable std_msgs/msg/Bool 'data: true'"
        )

    # ----- callbacks --------------------------------------------------------
    def _on_ping_enable(self, msg: Bool) -> None:
        if msg.data:
            params = self._collect_ping_params()
            self._port_worker.set_pinging(True, params)
            self._starboard_worker.set_pinging(True, params)
        else:
            self._port_worker.set_pinging(False)
            self._starboard_worker.set_pinging(False)

    # ----- shutdown ---------------------------------------------------------
    def shutdown(self) -> None:
        self.get_logger().info("stopping side scan sonar node")
        self._port_worker.stop()
        self._starboard_worker.stop()

    # ----- helpers ----------------------------------------------------------
    def _collect_ping_params(self) -> PingParams:
        return PingParams(
            start_mm=self._int_param("range_start_mm"),
            length_mm=self._int_param("range_length_mm"),
            msec_per_ping=self._int_param("msec_per_ping"),
            gain_index=self._int_param("gain_index"),
            num_results=self._int_param("num_results"),
            pulse_len_percent=self._float_param("pulse_len_percent"),
        )

    def _int_param(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    def _float_param(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = SideScanSonarNode()
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
