#!/usr/bin/env python3
"""Cerulean Omniscan 450 TCP link: framing, commands, profile decoding.

ROS-free, so the suite tests it without a workspace. `sss_node.py` owns one
`OmniscanLink` per transducer and nothing else talks to the devices.

Why not `brping` (2026-10-27). Its `PingDevice._input_buffer = deque()` is a
*class* attribute that `Omniscan450.__init__` never replaces, so the port and
starboard instances of one process read both TCP streams into the same deque
from two threads. The field log of the 2026-10-27 15:07 run holds the proof —
``[starboard] connection lost: pop from an empty deque``, which only two
threads popping one deque can raise — and it forced a reconnect mid-survey.
On top of that, its TCP ``read_io`` busy-spins on ``BlockingIOError`` (a full
core per idle thread, and ``wait_message``'s timeout never fires), it parses
byte by byte in Python, and it silently drops every message id it has no
template for (the Omniscan status frames, id 2194, that SonarView records).
`brping` stays in the test suite as the oracle for the command bytes.

Here every link owns its socket, its receive buffer and its parser; nothing
is shared between instances. Every complete, checksum-valid frame is returned
whatever its id, so unknown device messages reach `~/raw` and the `.svlog`.

Packet layout (Cerulean Ping Protocol, little-endian; see svlog_helper):
``'B' 'R' | u16 payload_len | u16 msg_id | u8 src | u8 dst | payload | u16 sum``
with ``sum = sum(bytes[0 .. 7+N]) & 0xFFFF``.
"""

from __future__ import annotations

import array
import socket
import struct
import sys
import threading
import time
from collections import Counter
from typing import Dict, List, Optional, Tuple

from svlog_helper import OS_MONO_PROFILE_ID, frame_packet


# ---------------------------------------------------------------------------
# Message ids and layouts (brping.definitions, bluerobotics-ping 0.2.4)
# ---------------------------------------------------------------------------
COMMON_ACK_ID:           int = 1
COMMON_NACK_ID:          int = 2
COMMON_ASCII_TEXT_ID:    int = 3
DEVICE_INFORMATION_ID:   int = 4
PROTOCOL_VERSION_ID:     int = 5
GENERAL_REQUEST_ID:      int = 6
SET_SPEED_OF_SOUND_ID:   int = 1002
OMNISCAN_STATUS_ID:      int = 2194   # ~0.9 Hz per device in SonarView logs
OS_PING_PARAMS_ID:       int = 2197

# Replies to our own commands. They describe the conversation, not the
# acquisition, so they are not recorded; everything else the device sends is.
PROTOCOL_REPLY_IDS = frozenset({COMMON_ACK_ID, COMMON_NACK_ID,
                                COMMON_ASCII_TEXT_ID, DEVICE_INFORMATION_ID,
                                PROTOCOL_VERSION_ID})

HEADER_SIZE:   int = 8
CHECKSUM_SIZE: int = 2
SYNC:          bytes = b"BR"
# A false 'BR' inside a profile can carry any length. The largest legitimate
# frame is a 1200-bin profile (52 + 2 * 1200 B payload); anything longer than
# this is treated as a false sync and skipped one byte at a time, so a
# corrupted length can never stall the stream waiting for 64 kB.
MAX_PAYLOAD_BYTES: int = 16384

# os_ping_params: start_mm, length_mm, msec_per_ping, reserved_1, reserved_2,
# pulse_len_percent, filter_duration_percent, gain_index, num_results,
# enable, reserved_3, reserved_4, reserved_5  (36 bytes)
_OS_PING_PARAMS = struct.Struct("<IIIffffhHBBBB")
# os_mono_profile header (52 bytes), followed by num_results x u16 pwr_results.
_PROFILE_HEAD = struct.Struct("<IIIIIHHHBBffffff")
PROFILE_FIELDS: Tuple[str, ...] = (
    "ping_number", "start_mm", "length_mm", "timestamp_ms", "ping_hz",
    "gain_index", "num_results", "sos_dmps", "channel_number", "reserved",
    "pulse_duration_sec", "analog_gain", "max_pwr_db", "min_pwr_db",
    "transducer_heading_deg", "vehicle_heading_deg",
)


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def pack_os_ping_params(start_mm: int, length_mm: int, msec_per_ping: int,
                        pulse_len_percent: float,
                        filter_duration_percent: float, gain_index: int,
                        num_results: int, enable: bool) -> bytes:
    """Framed os_ping_params command, byte-identical to brping's."""
    payload = _OS_PING_PARAMS.pack(
        int(start_mm), int(length_mm), int(msec_per_ping), 0.0, 0.0,
        float(pulse_len_percent), float(filter_duration_percent),
        int(gain_index), int(num_results), 1 if enable else 0, 0, 0, 0)
    return frame_packet(OS_PING_PARAMS_ID, payload, src=0, dst=0)


def pack_general_request(requested_id: int) -> bytes:
    """Framed general_request asking the device to send ``requested_id``."""
    return frame_packet(GENERAL_REQUEST_ID, struct.pack("<H", requested_id),
                        src=0, dst=0)


# ---------------------------------------------------------------------------
# Decoding
# ---------------------------------------------------------------------------
def message_id(frame: bytes) -> int:
    return frame[4] | (frame[5] << 8)


def decode_profile(frame: bytes) -> Dict[str, object]:
    """os_mono_profile frame -> header fields plus ``pwr_results`` (array 'H').

    The samples stay one C array from the socket to the ROS message: no
    per-sample Python objects (the old driver built ~2400 per ping pair).
    Raises ValueError on a short or non-profile frame.
    """
    if message_id(frame) != OS_MONO_PROFILE_ID:
        raise ValueError(f"not an os_mono_profile frame (id {message_id(frame)})")
    payload = memoryview(frame)[HEADER_SIZE:len(frame) - CHECKSUM_SIZE]
    if len(payload) < _PROFILE_HEAD.size:
        raise ValueError("profile payload too short")
    out: Dict[str, object] = dict(zip(PROFILE_FIELDS,
                                      _PROFILE_HEAD.unpack_from(payload, 0)))
    n = int(out["num_results"])
    end = _PROFILE_HEAD.size + 2 * n
    if len(payload) < end:
        raise ValueError(f"profile truncated: {n} bins announced")
    pwr = array.array("H")
    pwr.frombytes(payload[_PROFILE_HEAD.size:end])
    if sys.byteorder != "little":
        pwr.byteswap()
    out["pwr_results"] = pwr
    return out


# ---------------------------------------------------------------------------
# Framer
# ---------------------------------------------------------------------------
class FrameParser:
    """Incremental Ping-Protocol framer for one byte stream.

    ``feed()`` takes whatever the socket returned — any chunk size, frames
    split anywhere — and returns every complete frame whose checksum holds.
    A bad checksum or an impossible length is a false sync: skip one byte and
    search again, so one corrupted frame costs that frame only.
    """

    def __init__(self, max_payload: int = MAX_PAYLOAD_BYTES) -> None:
        self._buf = bytearray()
        self._max_payload = max_payload
        self.frames = 0
        self.checksum_errors = 0
        self.skipped_bytes = 0

    def feed(self, data) -> List[Tuple[int, bytes]]:
        buf = self._buf
        buf += data
        out: List[Tuple[int, bytes]] = []
        pos = 0
        n = len(buf)
        while True:
            i = buf.find(SYNC, pos)
            if i < 0:
                # Keep a trailing 'B': it may be the first half of a sync.
                keep = n - 1 if n > pos and buf[n - 1] == SYNC[0] else n
                self.skipped_bytes += keep - pos
                pos = keep
                break
            self.skipped_bytes += i - pos
            pos = i
            if n - pos < HEADER_SIZE:
                break
            payload_len = buf[pos + 2] | (buf[pos + 3] << 8)
            if payload_len > self._max_payload:
                self.skipped_bytes += 1
                pos += 1
                continue
            end = pos + HEADER_SIZE + payload_len + CHECKSUM_SIZE
            if end > n:
                break
            frame = bytes(buf[pos:end])
            if (sum(frame[:-CHECKSUM_SIZE]) & 0xFFFF) != (frame[-2] | (frame[-1] << 8)):
                self.checksum_errors += 1
                self.skipped_bytes += 1
                pos += 1
                continue
            out.append((frame[4] | (frame[5] << 8), frame))
            self.frames += 1
            pos = end
        del buf[:pos]
        return out


# ---------------------------------------------------------------------------
# Link
# ---------------------------------------------------------------------------
class OmniscanLink:
    """One TCP connection to one Omniscan 450. Owns all of its state.

    ``receive()`` blocks in the kernel for at most ``recv_timeout_s`` (no
    polling, no busy-spin) and returns the frames completed by what arrived.
    ``send()`` may be called from another thread while a receive is pending.
    """

    def __init__(self, host: str, port: int, *,
                 connect_timeout_s: float = 5.0,
                 recv_timeout_s: float = 0.5,
                 rcvbuf_bytes: int = 4 * 1024 * 1024) -> None:
        self.host = host
        self.port = port
        self._connect_timeout_s = connect_timeout_s
        self._recv_timeout_s = recv_timeout_s
        self._rcvbuf_bytes = rcvbuf_bytes
        self._sock: Optional[socket.socket] = None
        self._send_lock = threading.Lock()
        self._rx = bytearray(256 * 1024)
        self._rx_view = memoryview(self._rx)
        self.parser = FrameParser()
        # Frames that arrived while connect() waited for device_information;
        # handed out first by the next receive(), never dropped.
        self._pending: List[Tuple[int, bytes]] = []
        self.device_information: Optional[Tuple[int, ...]] = None
        self.ids_seen: Counter = Counter()

    # ----- connection --------------------------------------------------------
    def connect(self, info_timeout_s: float = 2.0) -> bool:
        """Open the socket and confirm a device answers. False if none does."""
        sock = socket.create_connection((self.host, self.port),
                                        timeout=self._connect_timeout_s)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF,
                            self._rcvbuf_bytes)
        except OSError:
            pass                    # tuning only; the link works without it
        sock.settimeout(self._recv_timeout_s)
        self._sock = sock
        self.parser = FrameParser()
        self._pending = []
        self.device_information = None
        self.send(pack_general_request(DEVICE_INFORMATION_ID))
        # A device left pinging by an earlier session streams profiles at
        # once: the deadline is wall time, not a count of quiet timeouts.
        deadline = time.monotonic() + info_timeout_s
        while time.monotonic() < deadline:
            for mid, frame in self._read_once():
                if mid == DEVICE_INFORMATION_ID and self.device_information is None:
                    body = frame[HEADER_SIZE:-CHECKSUM_SIZE]
                    self.device_information = tuple(body[:6])
                else:
                    self._pending.append((mid, frame))
            if self.device_information is not None:
                return True
        self.close()
        return False

    def close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    @property
    def connected(self) -> bool:
        return self._sock is not None

    # ----- io -----------------------------------------------------------------
    def send(self, frame: bytes) -> None:
        sock = self._sock
        if sock is None:
            raise ConnectionError("link closed")
        with self._send_lock:
            sock.sendall(frame)

    def set_ping_params(self, *, start_mm: int, length_mm: int,
                        msec_per_ping: int, pulse_len_percent: float,
                        filter_duration_percent: float, gain_index: int,
                        num_results: int, enable: bool) -> None:
        self.send(pack_os_ping_params(
            start_mm, length_mm, msec_per_ping, pulse_len_percent,
            filter_duration_percent, gain_index, num_results, enable))

    def receive(self) -> List[Tuple[int, bytes]]:
        """Frames completed since the last call; [] after a quiet timeout.

        Raises ConnectionError when the peer closed or the socket died.
        """
        if self._pending:
            out, self._pending = self._pending, []
            return out
        return self._read_once()

    def _read_once(self) -> List[Tuple[int, bytes]]:
        sock = self._sock
        if sock is None:
            raise ConnectionError("link closed")
        try:
            n = sock.recv_into(self._rx_view)
        except socket.timeout:
            return []
        except OSError as exc:
            raise ConnectionError(str(exc)) from exc
        if n == 0:
            raise ConnectionError("connection closed by the device")
        frames = self.parser.feed(self._rx_view[:n])
        for mid, _ in frames:
            self.ids_seen[mid] += 1
        return frames
