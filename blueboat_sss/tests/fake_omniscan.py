"""A fake Omniscan 450 on a local TCP port, for the driver tests.

It answers ``general_request(device_information)``, records every command it
receives, and on ``os_ping_params(enable=1)`` streams ``count`` profile frames
(plus a status frame every ``status_every`` profiles) in random-sized TCP
writes -- the shape of a real stream, frames split anywhere. Nothing here
imports ROS.
"""

from __future__ import annotations

import random
import socket
import struct
import threading
import time
from typing import List, Optional

from svlog_helper import OS_MONO_PROFILE_ID, frame_packet

PROFILE_HEAD = struct.Struct("<IIIIIHHHBBffffff")


def profile_frame(ping_number: int, channel: int, num_results: int = 1200,
                  timestamp_ms: Optional[int] = None) -> bytes:
    """A valid os_mono_profile frame whose samples encode the ping number."""
    head = PROFILE_HEAD.pack(
        ping_number, 0, 20000,
        ping_number * 50 if timestamp_ms is None else timestamp_ms,
        451127, 4, num_results, 15000, channel, 0,
        44.333e-6, 74.55, 65.0, -15.0,
        -90.0 if channel == 0 else 90.0, 12.5)
    pwr = struct.pack(f"<{num_results}H",
                      *(((ping_number * 7 + i) & 0xFFFF) for i in range(num_results)))
    return frame_packet(OS_MONO_PROFILE_ID, head + pwr, src=0, dst=0)


def status_frame(channel: int, t_ms: int) -> bytes:
    return frame_packet(2194, struct.pack("<ffI3xB", 55.0, 37.0, t_ms, channel),
                        src=0, dst=0)


def device_information_frame() -> bytes:
    return frame_packet(4, bytes([20, 1, 1, 2, 3, 0]), src=0, dst=0)


class FakeOmniscan:
    """One fake device. ``start()`` returns the port it listens on."""

    def __init__(self, channel: int, count: int = 300, period_s: float = 0.0,
                 status_every: int = 20, answer_info: bool = True,
                 stream_before_info: int = 0, seed: int = 0) -> None:
        self.channel = channel
        self.count = count
        self.period_s = period_s
        self.status_every = status_every
        self.answer_info = answer_info
        self.stream_before_info = stream_before_info
        self.commands: List[bytes] = []
        self._rng = random.Random(seed)
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.streamed = threading.Event()

    def start(self) -> int:
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        return self.port

    def stop(self) -> None:
        self._stop.set()
        try:
            self._srv.close()
        except OSError:
            pass
        if self._thread is not None:
            self._thread.join(timeout=5)

    # -- internals -------------------------------------------------------------
    def _send_chunked(self, conn: socket.socket, data: bytes) -> None:
        pos = 0
        while pos < len(data):
            k = self._rng.randint(1, 3000)
            conn.sendall(data[pos:pos + k])
            pos += k

    def _stream(self, conn: socket.socket, start_pn: int, n: int) -> None:
        for i in range(n):
            pn = start_pn + i
            data = profile_frame(pn, self.channel)
            if self.status_every and i % self.status_every == 0:
                data += status_frame(self.channel, pn * 50)
            self._send_chunked(conn, data)
            if self.period_s:
                time.sleep(self.period_s)

    def _serve(self) -> None:
        try:
            conn, _ = self._srv.accept()
        except OSError:
            return
        conn.settimeout(0.2)
        buf = bytearray()
        try:
            if self.stream_before_info:
                self._stream(conn, 10_000, self.stream_before_info)
            while not self._stop.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    return
                buf += data
                while len(buf) >= 10 and buf[:2] == b"BR":
                    n = buf[2] | (buf[3] << 8)
                    if len(buf) < 10 + n:
                        break
                    frame, buf = bytes(buf[:10 + n]), buf[10 + n:]
                    self.commands.append(frame)
                    mid = frame[4] | (frame[5] << 8)
                    if mid == 6 and self.answer_info:
                        conn.sendall(device_information_frame())
                    elif mid == 2197 and frame[8 + 32] == 1:      # enable
                        self._stream(conn, 1, self.count)
                        self.streamed.set()
        except OSError:
            return
        finally:
            conn.close()
