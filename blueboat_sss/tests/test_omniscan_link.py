"""The Omniscan TCP link that replaced brping in sss_node.py (2026-10-27).

ROS-free. What is pinned:

* the framer recovers every frame whatever the TCP chunking, resyncs past
  corruption at the cost of the corrupted frame only, and refuses a false
  sync with an impossible length;
* the command bytes are exactly brping's (pinned hex, and brping itself as
  the oracle when it is importable);
* the profile decode agrees field for field with the GCS's own decoder;
* two links share no state -- brping's class-level receive buffer, shared by
  the port and starboard devices, is what this module exists to replace;
* over real sockets, two links in two threads each receive exactly their own
  device's frames, with none lost.

The local SonarView recordings, when present, are opened read-only
(NC #6 / root CM-7).
"""

from __future__ import annotations

import importlib.util
import random
import threading
from pathlib import Path

import pytest

import omniscan_link as ol
from blueboat_gcs.core.svlog import decode_os_mono_profile, walk_packets
from fake_omniscan import (FakeOmniscan, device_information_frame,
                           profile_frame, status_frame)

# The brping bytes for os_ping_params(0, 20000, 50, 0.00125, 0.0015, -1, 1200,
# enable) -- captured from bluerobotics-ping 0.2.4 on 2026-10-27.
BRPING_OS_PING_PARAMS_HEX = (
    "425224009508000000000000204e00003200000000000000000000000ad7a33a"
    "a69bc43affffb00401000000a508")
SONARVIEW_POOL_LOGS = sorted(
    (Path.home() / "ros2_ws/data/SSS_data/sessions").glob("pool_*.svlog"))


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def test_os_ping_params_bytes_are_brpings():
    frame = ol.pack_os_ping_params(0, 20000, 50, 0.00125, 0.0015, -1, 1200, True)
    assert frame.hex() == BRPING_OS_PING_PARAMS_HEX
    assert len(frame) == 8 + 36 + 2


@pytest.mark.skipif(importlib.util.find_spec("brping") is None,
                    reason="brping (the oracle) is not importable here")
def test_command_bytes_match_brping_itself():
    from brping import definitions, pingmessage
    m = pingmessage.PingMessage(definitions.OMNISCAN450_OS_PING_PARAMS)
    for k, v in dict(start_mm=1000, length_mm=35000, msec_per_ping=50,
                     reserved_1=0, reserved_2=0, pulse_len_percent=0.00125,
                     filter_duration_percent=0.0015, gain_index=4,
                     num_results=600, enable=0, reserved_3=0, reserved_4=0,
                     reserved_5=0).items():
        setattr(m, k, v)
    m.pack_msg_data()
    assert bytes(m.msg_data) == ol.pack_os_ping_params(
        1000, 35000, 50, 0.00125, 0.0015, 4, 600, False)
    g = pingmessage.PingMessage(definitions.COMMON_GENERAL_REQUEST)
    g.requested_id = ol.DEVICE_INFORMATION_ID
    g.pack_msg_data()
    assert bytes(g.msg_data) == ol.pack_general_request(ol.DEVICE_INFORMATION_ID)


# ---------------------------------------------------------------------------
# Framer
# ---------------------------------------------------------------------------
def _stream(n=200):
    frames = []
    for pn in range(1, n + 1):
        frames.append(profile_frame(pn, pn % 2, num_results=600))
        if pn % 25 == 0:
            frames.append(status_frame(pn % 2, pn * 50))
    return frames


def _feed_chunked(parser, data, seed=0):
    rng = random.Random(seed)
    out, pos = [], 0
    while pos < len(data):
        k = rng.randint(1, 5000)
        out += parser.feed(data[pos:pos + k])
        pos += k
    return out


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_framer_recovers_every_frame_from_any_chunking(seed):
    frames = _stream()
    parser = ol.FrameParser()
    got = _feed_chunked(parser, b"".join(frames), seed)
    assert [f for _, f in got] == frames
    assert parser.checksum_errors == 0 and parser.skipped_bytes == 0


def test_framer_skips_leading_garbage_and_resyncs_after_corruption():
    frames = _stream(50)
    bad = bytearray(frames[10])
    bad[300] ^= 0xFF                                  # one flipped payload byte
    data = b"\x00\x13garbage" + b"".join(frames[:10]) + bytes(bad) + b"".join(frames[11:])
    parser = ol.FrameParser()
    got = [f for _, f in _feed_chunked(parser, data, 3)]
    assert got == frames[:10] + frames[11:]           # only the corrupted frame is lost
    assert parser.checksum_errors >= 1


def test_framer_refuses_a_false_sync_with_an_impossible_length():
    frames = _stream(5)
    false_sync = b"BR" + (60000).to_bytes(2, "little") + b"\x00" * 6
    parser = ol.FrameParser()
    got = [f for _, f in parser.feed(false_sync + b"".join(frames))]
    assert got == frames


def test_framer_keeps_a_split_sync_byte():
    frame = profile_frame(7, 0, num_results=16)
    parser = ol.FrameParser()
    assert parser.feed(frame[:1]) == []               # a lone 'B'
    assert [f for _, f in parser.feed(frame[1:])] == [frame]


# ---------------------------------------------------------------------------
# Decoding
# ---------------------------------------------------------------------------
def test_profile_decode_agrees_with_the_gcs_decoder():
    frame = profile_frame(4242, 1, num_results=1200)
    mine = ol.decode_profile(frame)
    gcs = decode_os_mono_profile(frame[8:-2])
    for key in ("ping_number", "start_mm", "length_mm", "timestamp_ms",
                "num_results", "channel_number", "gain_index",
                "transducer_heading_deg", "max_pwr_db", "min_pwr_db",
                "analog_gain"):
        assert mine[key] == pytest.approx(gcs[key]), key
    assert mine["pwr_results"].typecode == "H"
    assert list(mine["pwr_results"]) == [int(v) for v in gcs["pwr"]]


def test_decode_refuses_short_and_foreign_frames():
    frame = profile_frame(1, 0, num_results=100)
    with pytest.raises(ValueError):
        ol.decode_profile(status_frame(0, 1))
    truncated = frame[:8] + frame[8:60] + frame[-2:]   # header only, bins missing
    with pytest.raises(ValueError):
        ol.decode_profile(truncated)


# ---------------------------------------------------------------------------
# Links: no shared state, real sockets, two threads
# ---------------------------------------------------------------------------
def test_two_links_share_no_state():
    """brping's PingDevice._input_buffer is a class attribute: both devices
    of one process read into the same deque. Nothing may be shared here."""
    a, b = ol.OmniscanLink("127.0.0.1", 1), ol.OmniscanLink("127.0.0.1", 2)
    assert a.parser is not b.parser
    assert a._rx is not b._rx and a._pending is not b._pending
    assert a.ids_seen is not b.ids_seen


def _drain(link, want, out):
    while len(out) < want:
        try:
            frames = link.receive()
        except ConnectionError:
            return
        out += frames


def test_two_devices_in_two_threads_lose_nothing_and_never_mix():
    n = 400
    devices = [FakeOmniscan(channel=ch, count=n, seed=ch) for ch in (0, 1)]
    links = [ol.OmniscanLink("127.0.0.1", d.start()) for d in devices]
    try:
        assert all(link.connect(info_timeout_s=2.0) for link in links)
        got = [[], []]
        threads = [threading.Thread(target=_drain, args=(links[i], n + n // 20, got[i]),
                                    daemon=True)
                   for i in (0, 1)]
        for t in threads:
            t.start()
        for link in links:
            link.set_ping_params(start_mm=0, length_mm=20000, msec_per_ping=50,
                                 pulse_len_percent=0.00125,
                                 filter_duration_percent=0.0015, gain_index=-1,
                                 num_results=1200, enable=True)
        for t in threads:
            t.join(timeout=20)
        for ch in (0, 1):
            profiles = [ol.decode_profile(f) for mid, f in got[ch]
                        if mid == ol.OS_MONO_PROFILE_ID]
            assert [p["ping_number"] for p in profiles] == list(range(1, n + 1))
            assert {p["channel_number"] for p in profiles} == {ch}
            assert any(mid == ol.OMNISCAN_STATUS_ID for mid, _ in got[ch])
            assert links[ch].parser.checksum_errors == 0
    finally:
        for link in links:
            link.close()
        for d in devices:
            d.stop()


def test_connect_returns_false_when_the_device_never_answers():
    dev = FakeOmniscan(channel=0, answer_info=False)
    link = ol.OmniscanLink("127.0.0.1", dev.start(), recv_timeout_s=0.1)
    try:
        assert link.connect(info_timeout_s=0.3) is False
        assert not link.connected
    finally:
        dev.stop()


def test_frames_streaming_before_device_information_are_kept():
    """A device left pinging by an earlier session streams at once; those
    frames arrive before device_information and must not be dropped."""
    dev = FakeOmniscan(channel=1, stream_before_info=30, status_every=0)
    link = ol.OmniscanLink("127.0.0.1", dev.start())
    try:
        assert link.connect(info_timeout_s=2.0)
        first = link.receive()
        assert first and all(mid == ol.OS_MONO_PROFILE_ID for mid, _ in first)
        assert ol.decode_profile(first[0][1])["ping_number"] == 10_000
    finally:
        link.close()
        dev.stop()


def test_send_after_close_raises_connection_error():
    link = ol.OmniscanLink("127.0.0.1", 1)
    with pytest.raises(ConnectionError):
        link.send(device_information_frame())
    with pytest.raises(ConnectionError):
        link.receive()


# ---------------------------------------------------------------------------
# The local SonarView recordings, when present (read-only)
# ---------------------------------------------------------------------------
@pytest.mark.skipif(not SONARVIEW_POOL_LOGS,
                    reason="local SonarView pool recordings not present")
def test_framer_reproduces_a_sonarview_recording_frame_for_frame():
    data = SONARVIEW_POOL_LOGS[0].read_bytes()
    parser = ol.FrameParser()
    got = [f for _, f in _feed_chunked(parser, data, 5)]
    assert got == [bytes(p) for p in walk_packets(data)]
    assert parser.checksum_errors == 0
