#!/usr/bin/env python3
"""SonarView .svlog file format support.
Used by the processor node to write .svlog files that can be opened directly in SonarView for
easy visualization and comparision. 

The .svlog format is a stream of Cerulean Ping Protocol packets:
https://docs.ceruleansonar.com/c/cerulean-ping-protocol/universal-packet-format

Packet layout (little-endian throughout):

    byte 0      'B'  (0x42)
    byte 1      'R'  (0x52)
    byte 2..3   u16  payload length N
    byte 4..5   u16  packet_id
    byte 6      u8   src_device_id    ("reserved" in the public spec; brping
                                       uses it for the originating device)
    byte 7      u8   dst_device_id
    byte 8..    u8[] payload          (N bytes)
    byte 8+N..  u16  checksum = sum(byte[0..7+N]) & 0xFFFF

The byte 6/7 ordering follows the canonical `brping.pingmessage` parser,
which declares `header_format = "BBHHBB"` with field order
(start_1, start_2, payload_length, message_id, src_device_id, dst_device_id).

Observed conventions in reference SonarView .svlog files:
    * Session header packets (packet_id 10, 12) carry src=0 (server),
      dst=0xFF (broadcast).
    * Sonar profile packets (packet_id 2198) carry src = Omniscan device_id
      (1=port, 2=starboard), dst=0 (host).
    * Mavlink wrapper packets (packet_id 150) carry src = platform device_id
      (3=BlueBoat), dst=0 (host).

SonarView links a packet to a device using byte 6 (src_device_id) matched
against the device_id field declared in the session_devices / session_platform
JSON in the file's first packet.
"""

from __future__ import annotations

import json
import platform
import sys
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional


# ---------------------------------------------------------------------------
# Packet IDs
# ---------------------------------------------------------------------------
JSON_WRAPPER_ID:    int = 10     # session metadata (first packet in file)
VIEW_CONFIG_ID:     int = 12     # SonarView view config (gamma, range, ...)
MAVLINK_WRAPPER_ID: int = 150    # mavlink2rest JSON envelope
OS_MONO_PROFILE_ID: int = 2198   # Omniscan 450 sonar profile

# Device IDs used as byte-6 routing tag AND as session_devices[*].device_id /
# session_platform.device_id in the metadata JSON. They must agree.
DEVICE_ID_PORT:     int = 1
DEVICE_ID_STBD:     int = 2
DEVICE_ID_PLATFORM: int = 3

# Broadcast destination tag (byte 7) used on session-metadata packets.
DST_BROADCAST: int = 0xFF

# Default file roll threshold (matches brping's MAX_LOG_SIZE_MB).
MAX_LOG_SIZE_BYTES: int = 500 * 1000 * 1000

# Filter string SonarView declares in its real .svlog session_platform.url.
# Used purely as a session-metadata description -- it's NOT consumed by this
# module. Mavlink data flows in via mavros subscriptions in the consumer node.
DEFAULT_MAVLINK_FILTER: str = (
    "ATTITUDE|GLOBAL_POSITION_INT|LOCAL_POSITION_NED|HOME_POSITION|"
    "HEARTBEAT|SCALED_PRESSURE2|DISTANCE_SENSOR|GPS_GLOBAL_ORIGIN|"
    "PARAM_VALUE|VFR_HUD|STATUSTEXT|AUTOPILOT_VERSION"
)

# Mavlink message subscription list reported in session_platform.options.
# These are the mavlink message_id + rate pairs the SonarView/SonarLink
# server requests from BlueOS. Purely descriptive in our case -- mavros
# does the real subscribing on the robot. Values match what SonarView
# itself writes in reference .svlog files.
DEFAULT_PERIODIC_MESSAGES: list = [
    {"message_id": 0,   "frequency_hz": 1.0},          # HEARTBEAT
    {"message_id": 30,  "frequency_hz": 30.0},         # ATTITUDE
    {"message_id": 33,  "frequency_hz": 30.0},         # GLOBAL_POSITION_INT
    {"message_id": 32,  "frequency_hz": 30.0},         # LOCAL_POSITION_NED
    {"message_id": 137, "frequency_hz": 3.0},          # SCALED_PRESSURE2
    {"message_id": 74,  "frequency_hz": 3.0},          # VFR_HUD
    {"message_id": 49,  "frequency_hz": 1.0},          # GPS_GLOBAL_ORIGIN
    {"message_id": 148, "frequency_hz": 1.0 / 60.0},   # AUTOPILOT_VERSION
]


# ---------------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------------
def frame_packet(packet_id: int, payload: bytes, src: int = 0, dst: int = 0) -> bytes:
    """Frame `payload` as a complete Cerulean Ping Protocol packet.

    `src` lands in byte 6, `dst` in byte 7 (brping convention).
    """
    buf = bytearray()
    buf += b"BR"
    buf += len(payload).to_bytes(2, "little")
    buf += packet_id.to_bytes(2, "little")
    buf += src.to_bytes(1, "little")  # byte 6
    buf += dst.to_bytes(1, "little")  # byte 7
    buf += payload
    buf += (sum(buf) & 0xFFFF).to_bytes(2, "little")
    return bytes(buf)


def retag_packet_src_device_id(raw_packet: bytes, new_src: int) -> bytes:
    """Rewrite byte 6 (src_device_id) and refresh the checksum.

    Used by the svlog writer to tag Omniscan packets with the correct
    originating device_id (port=1, starboard=2) before interleaving them
    into the file. The packet's original byte 6 (whatever the device
    emitted, typically 0) is replaced with `new_src` and the trailing u16
    checksum is recomputed.
    """
    if len(raw_packet) < 10 or raw_packet[0:2] != b"BR":
        raise ValueError("not a Ping-Protocol packet")
    payload_len = int.from_bytes(raw_packet[2:4], "little")
    if len(raw_packet) != 8 + payload_len + 2:
        raise ValueError(
            f"packet length mismatch: header says {payload_len}, "
            f"buffer is {len(raw_packet) - 10}"
        )
    buf = bytearray(raw_packet)
    buf[6] = new_src & 0xFF
    checksum = sum(buf[: 8 + payload_len]) & 0xFFFF
    buf[8 + payload_len : 8 + payload_len + 2] = checksum.to_bytes(2, "little")
    return bytes(buf)


# ---------------------------------------------------------------------------
# Session metadata + mavlink-wrapper builders
# ---------------------------------------------------------------------------
def build_session_metadata(
    port_url: str,
    starboard_url: str,
    mavlink_url: Optional[str] = None,
    mavlink_filter: str = DEFAULT_MAVLINK_FILTER,
    session_plan_name: str = "Dual omniscan",
) -> bytes:
    """Build the JSON_WRAPPER session metadata packet (packet_id=10).

    `session_devices` lists both Omniscan units with the device_ids that
    `retag_packet_src_device_id` will use at write time. `session_platform`
    declares the BlueBoat as a mavlink2rest source (device_id=3); pass
    `mavlink_url=None` to leave session_platform null when not relaying
    mavlink. The `mavlink_url` here is purely informational -- this module
    never opens it.
    """
    content = {
        "session_id": 1,
        "session_uptime": 0.0,
        "session_devices": [
            {
                "url": port_url,
                "options": {
                    "transducer_deg": -90,
                    "imu_deg": 0,
                    "doppler_enable": False,
                },
                "device_id": DEVICE_ID_PORT,
                "product_id": "os450",
                "nickname": "port omniscan",
                "status": "CONNECTED",
            },
            {
                "url": starboard_url,
                "options": {
                    "transducer_deg": 90,
                    "imu_deg": 0,
                    "doppler_enable": False,
                },
                "device_id": DEVICE_ID_STBD,
                "product_id": "os450",
                "nickname": "starboard omniscan",
                "status": "CONNECTED",
            },
        ],
        "session_platform": (
            None
            if mavlink_url is None
            else {
                "url": f"{mavlink_url}?filter={mavlink_filter}",
                "protocol": "mavlink2rest",
                "options": {
                    "system_id": 1,
                    "periodic_messages": list(DEFAULT_PERIODIC_MESSAGES),
                },
                "nickname": "BlueBoat",
                "model": "blue_boat",
                "device_id": DEVICE_ID_PLATFORM,
                "status": "CONNECTED",
            }
        ),
        "session_clients": [],
        "session_plan_name": session_plan_name,
        "is_recording": True,
        "sonarlink_version": "",
        "os_hostname": platform.node(),
        "os_uptime": None,
        "os_version": platform.version(),
        "os_platform": platform.system().lower(),
        "os_release": platform.release(),
        "process_path": sys.executable,
        "process_version": f"v{platform.python_version()}",
        "process_uptime": 0.0,
        "process_arch": platform.machine(),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "timestamp_timezone_offset": int(
            datetime.now().astimezone().utcoffset().total_seconds() // 60
        ),
    }
    payload = json.dumps(content, indent=2).encode("utf-8")
    return frame_packet(JSON_WRAPPER_ID, payload, src=0, dst=DST_BROADCAST)


def build_mavlink_wrapper(mavlink_message: dict) -> bytes:
    """Wrap a mavlink2rest-style JSON dict as a MAVLINK_WRAPPER packet (id=150).

    The dict must already be in mavlink2rest's envelope shape:
        {"header":  {"system_id": ..., "component_id": ..., "sequence": ...},
         "message": {"type": "ATTITUDE" | "GLOBAL_POSITION_INT" | ..., ...}}
    """
    payload = json.dumps(mavlink_message, indent=2).encode("utf-8")
    return frame_packet(
        MAVLINK_WRAPPER_ID, payload, src=DEVICE_ID_PLATFORM, dst=0
    )


# ---------------------------------------------------------------------------
# Writer
# ---------------------------------------------------------------------------
class SvlogWriter:
    """Thread-safe append-only .svlog writer.

    Opaque about packet semantics: callers frame their own bytes (using the
    `build_*` helpers or `frame_packet` above) and call `write()`. The only
    knowledge it has of packet shape is the metadata header it writes at
    file-open time, which is produced by an injected `metadata_provider`
    callable so the caller controls every field.

    All public methods are thread-safe; the port-side sonar thread, the
    starboard-side sonar thread and the ROS executor thread may all call
    `write()` concurrently.
    """

    def __init__(
        self,
        log_dir: Path,
        metadata_provider: Callable[[], bytes],
        max_size_bytes: int = MAX_LOG_SIZE_BYTES,
    ) -> None:
        self._log_dir = Path(log_dir)
        self._metadata_provider = metadata_provider
        self._max_size_bytes = max_size_bytes

        self._lock = threading.Lock()
        self._active = False
        self._path: Optional[Path] = None
        self._bytes_written = 0

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    @property
    def current_path(self) -> Optional[Path]:
        with self._lock:
            return self._path

    def start(self) -> Optional[Path]:
        """Open a new .svlog and write the session header."""
        with self._lock:
            if self._active:
                return self._path
            #self._log_dir.mkdir(parents=True, exist_ok=True)
            self._roll_unlocked()
            self._active = True
            return self._path

    def stop(self) -> None:
        with self._lock:
            self._active = False
            self._path = None
            self._bytes_written = 0

    def write(self, raw_bytes: bytes) -> None:
        """Append a single framed packet. No-op if not active."""
        with self._lock:
            if not self._active or self._path is None:
                return
            if self._bytes_written > self._max_size_bytes:
                self._roll_unlocked()
            try:
                with open(self._path, "ab") as f:
                    f.write(raw_bytes)
                self._bytes_written += len(raw_bytes)
            except OSError:
                # Disk full / unmounted / permissions -- stop quietly.
                self._active = False
                self._path = None

    def _roll_unlocked(self) -> None:
        name = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        path = self._log_dir / f"{name}.svlog"
        if path.exists():
            path.unlink()
        self._path = path
        self._bytes_written = 0
        meta = self._metadata_provider()
        with open(path, "ab") as f:
            f.write(meta)
        self._bytes_written += len(meta)
