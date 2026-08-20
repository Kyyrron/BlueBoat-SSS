#!/usr/bin/env python3
"""
Side Scan Sonar (SSS) ROS 2 node for the BlueBoat.

Drives the pair of Cerulean Omniscan 450 SS devices (port + starboard) and
republishes their `os_mono_profile` packets on two ROS topics, in two forms:

  * parsed `OmniscanProfile` -- for downstream processing,
  * raw framed Ping-Protocol bytes (`UInt8MultiArray`) -- for the processor
    node to interleave into a SonarView .svlog file.

Pinging is OFF when the node starts. Connections are opened immediately so
the devices are ready to fire on demand. On-disk logging is no longer this
node's responsibility -- the processor node owns the .svlog file.

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

import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, UInt8MultiArray

# bluerobotics-ping -- pip install --user bluerobotics-ping --upgrade
from brping import Omniscan450, definitions

from blueboat_interfaces.msg import OmniscanProfile


# ---------------------------------------------------------------------------
# Pinned (non run-dependent) parameters
# ---------------------------------------------------------------------------
FILTER_DURATION_PERCENT: float = 0.0015     # Cerulean docs: 0.0015 typical
WORKER_JOIN_TIMEOUT_S:   float = 5.0
RECONNECT_DELAY_S:       float = 2.0        # wait between reconnect attempts


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
# Per-device worker
# ---------------------------------------------------------------------------
class OmniscanWorker:
    """Owns one Omniscan450. Publishes parsed + raw packets for one side.

    Lifecycle is fully driven by the worker thread:
      1. Try to connect. On failure, sleep RECONNECT_DELAY_S and retry.
      2. Once connected, (re-)apply the desired pinging state.
      3. Run the receive loop. If wait_message errors out (socket dead),
         tear down and loop back to step 1.

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

        # Device + thread state. `_device` is reassigned by the worker
        # thread on connect/disconnect; CPython makes pointer
        # reads/writes atomic, so other threads can sample it directly.
        self._device: Optional[Omniscan450] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Desired vs applied pinging state. Updated by set_pinging(),
        # consumed by _apply_desired_state(). The lock serializes the
        # check-then-send logic so two callbacks racing don't both
        # trigger a duplicate enable command.
        self._control_lock = threading.Lock()
        self._desired_ping_enable = False
        self._desired_ping_params: Optional[PingParams] = None
        self._currently_pinging = False

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
        """Signal stop, force-close socket to unblock wait_message, join."""
        self._stop_event.set()

        # Best-effort: tell the device to stop pinging before we leave.
        # If it fails (already disconnected), no big deal -- next start
        # of the node will reset it via initialize() / enable=0.
        dev = self._device
        if dev is not None:
            try:
                dev.control_os_ping_params(enable=0)
            except Exception:  # noqa: BLE001
                pass
            # Closing iodev breaks any blocking wait_message immediately.
            # Without this, stop() would wait up to the library's internal
            # wait_message timeout (~4s) for the receive thread to exit.
            try:
                if dev.iodev:
                    dev.iodev.close()
            except Exception:  # noqa: BLE001
                pass

        if self._thread is not None:
            self._thread.join(timeout=WORKER_JOIN_TIMEOUT_S)
            if self._thread.is_alive():
                self._node.get_logger().warn(
                    f"[{self._side}] worker did not join in {WORKER_JOIN_TIMEOUT_S}s"
                )
        self._device = None

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

    def _apply_desired_state_locked(self) -> None:
        """Push the desired state to the device. Call under _control_lock."""
        log = self._node.get_logger()
        dev = self._device
        if dev is None:
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
                dev.control_os_ping_params(
                    start_mm=params.start_mm,
                    length_mm=params.length_mm,
                    msec_per_ping=params.msec_per_ping,
                    pulse_len_percent=params.pulse_len_percent,
                    filter_duration_percent=FILTER_DURATION_PERCENT,
                    gain_index=params.gain_index,
                    num_results=params.num_results,
                    enable=1,
                )
                log.info(
                    f"[{self._side}] pinging started "
                    f"(range {params.start_mm}-{params.start_mm + params.length_mm} mm, "
                    f"gain {params.gain_index}, n={params.num_results})"
                )
            else:
                dev.control_os_ping_params(enable=0)
                log.info(f"[{self._side}] pinging stopped")
            self._currently_pinging = enable
        except Exception as exc:  # noqa: BLE001
            # Connection probably died mid-command. Don't update
            # _currently_pinging; the next reconnect will resync.
            log.warn(f"[{self._side}] apply ping state failed: {exc}")

    # ----- main loop -------------------------------------------------------
    def _main_loop(self) -> None:
        log = self._node.get_logger()
        target = [definitions.OMNISCAN450_OS_MONO_PROFILE]

        while not self._stop_event.is_set() and rclpy.ok():
            # ---- Phase 1: connect (with retries) ----
            if not self._connect():
                # Cancellable sleep -- returns True if stop was signalled.
                if self._stop_event.wait(RECONNECT_DELAY_S):
                    return
                continue

            # ---- Phase 2: (re-)apply desired ping state ----
            # After connect(), the device is in enable=0 (we just sent it).
            with self._control_lock:
                self._currently_pinging = False
                self._apply_desired_state_locked()

            # ---- Phase 3: receive loop ----
            while not self._stop_event.is_set() and rclpy.ok():
                try:
                    data = self._device.wait_message(target)
                except Exception as exc:  # noqa: BLE001
                    log.warn(f"[{self._side}] connection lost: {exc}")
                    break  # drop out, reconnect
                if data is None:
                    continue
                self._publish_profile(data)
                # brping populates msg_data inside wait_message with the
                # already-framed Ping-Protocol bytes -- republish verbatim.
                if data.msg_data:
                    self._publish_raw(bytes(data.msg_data))

            # ---- Phase 4: tear down for reconnect ----
            self._teardown_device()

    def _connect(self) -> bool:
        """One connection attempt. Returns True iff the device is usable."""
        log = self._node.get_logger()
        try:
            dev = Omniscan450(logging=False)
            dev.connect_tcp(self._ip, self._tcp_port)
            if dev.initialize() is False:
                log.warn(
                    f"[{self._side}] initialize() returned False at "
                    f"{self._ip}:{self._tcp_port}, will retry"
                )
                self._safe_close_iodev(dev)
                return False
            # Start from a known clean state on the device.
            try:
                dev.control_os_ping_params(enable=0)
            except Exception:  # noqa: BLE001
                pass
            self._device = dev
            log.info(f"[{self._side}] connected at {self._ip}:{self._tcp_port}")
            return True
        except Exception as exc:  # noqa: BLE001
            log.warn(
                f"[{self._side}] connect failed at {self._ip}:{self._tcp_port}: "
                f"{exc} (retry in {RECONNECT_DELAY_S}s)"
            )
            return False

    def _teardown_device(self) -> None:
        if self._device is not None:
            self._safe_close_iodev(self._device)
            self._device = None

    @staticmethod
    def _safe_close_iodev(dev) -> None:
        try:
            if dev.iodev:
                dev.iodev.close()
        except Exception:  # noqa: BLE001
            pass

    # ----- publishing ------------------------------------------------------
    def _publish_profile(self, data) -> None:
        msg = OmniscanProfile()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.side = self._side

        msg.ping_number = int(data.ping_number)
        msg.start_mm = int(data.start_mm)
        msg.length_mm = int(data.length_mm)
        msg.timestamp_ms = int(data.timestamp_ms)
        msg.ping_hz = int(data.ping_hz)
        msg.gain_index = int(data.gain_index)
        msg.num_results = int(data.num_results)
        msg.sos_dmps = int(data.sos_dmps)
        msg.channel_number = int(data.channel_number)
        msg.pulse_duration_sec = float(data.pulse_duration_sec)
        msg.analog_gain = float(data.analog_gain)
        msg.max_pwr_db = float(data.max_pwr_db)
        msg.min_pwr_db = float(data.min_pwr_db)
        msg.transducer_heading_deg = float(data.transducer_heading_deg)
        msg.vehicle_heading_deg = float(data.vehicle_heading_deg)
        msg.pwr_results = list(data.pwr_results)

        self._profile_pub.publish(msg)

    def _publish_raw(self, raw: bytes) -> None:
        msg = UInt8MultiArray()
        # rclpy expects a sequence of ints for uint8[] fields; list(bytes)
        # is unambiguous and avoids any binding-version pitfalls.
        msg.data = list(raw)
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
        self.declare_parameter("range_length_mm", 30000)
        self.declare_parameter("msec_per_ping", 0)
        self.declare_parameter("gain_index", -1)
        self.declare_parameter("num_results", 600)
        self.declare_parameter("pulse_len_percent", 0.002)

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
            tcp_port=51200,
            profile_publisher=self._port_profile_pub,
            raw_publisher=self._port_raw_pub,
            frame_id="sss_port_link",
        )
        self._starboard_worker = OmniscanWorker(
            self,
            side="starboard",
            ip="192.168.2.93",
            tcp_port=51200,
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
