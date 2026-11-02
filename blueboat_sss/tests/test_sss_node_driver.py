"""sss_node.py's workers end to end against fake Omniscan devices.

Needs rclpy + blueboat_interfaces (skips cleanly without them). Two real
`OmniscanWorker` threads talk TCP to two `FakeOmniscan` servers on localhost:
the acquisition path of a real-robot run with the WiFi replaced by loopback.

Pinned:
* every profile of each device is published on its own side, none lost,
  none crossing sides (the brping shared-buffer defect);
* status frames (id 2194) reach ``~/raw`` and never ``~/profile``;
* the device is commanded with SonarView's acquisition: 50 ms per ping;
* a stop leaves the device commanded silent;
* the declared defaults are SonarView's (read from the source, like the
  simulator's CM-2 parity check).
"""

from __future__ import annotations

import importlib.util
import re
import struct
import sys
import time

import pytest

from conftest import PKG_PARENT

REQUIRED = ("rclpy", "blueboat_interfaces")
MISSING = [m for m in REQUIRED if importlib.util.find_spec(m) is None]
pytestmark = pytest.mark.skipif(
    bool(MISSING), reason=f"driver tests need a sourced ROS 2 env (missing: {MISSING})")

HELPERS = PKG_PARENT / "src" / "_custom_libraries"
if str(HELPERS) not in sys.path:
    sys.path.insert(0, str(HELPERS))
NODE_SRC = PKG_PARENT / "src" / "sss_node.py"

if not MISSING:
    import rclpy
    from rclpy.node import Node

    _spec = importlib.util.spec_from_file_location("sss_node", NODE_SRC)
    drv = importlib.util.module_from_spec(_spec)
    sys.modules["sss_node"] = drv           # dataclasses resolve annotations through it
    _spec.loader.exec_module(drv)

from fake_omniscan import FakeOmniscan  # noqa: E402

PARAMS_FMT = struct.Struct("<IIIffffhHBBBB")


class _Capture:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture
def host():
    rclpy.init()
    n = Node("sss_node_driver_test")
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def _wait(pred, timeout=20.0):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if pred():
            return True
        time.sleep(0.02)
    return False


def _params_sent(device):
    return [PARAMS_FMT.unpack(c[8:-2]) for c in device.commands
            if (c[4] | (c[5] << 8)) == 2197]


def test_two_workers_publish_every_ping_on_their_own_side(host):
    n = 300
    devices = [FakeOmniscan(channel=ch, count=n, status_every=20, seed=ch)
               for ch in (0, 1)]
    caps = [(_Capture(), _Capture()) for _ in (0, 1)]
    workers = [drv.OmniscanWorker(host, side, "127.0.0.1", devices[i].start(),
                                  profile_publisher=caps[i][0],
                                  raw_publisher=caps[i][1],
                                  frame_id=f"sss_{side}_link")
               for i, side in enumerate(("port", "starboard"))]
    try:
        for w in workers:
            w.start()
        params = drv.PingParams(0, 20000, 50, -1, 1200, 0.00125)
        for w in workers:
            w.set_pinging(True, params)
        assert _wait(lambda: all(len(c[0].msgs) >= n for c in caps)), \
            [len(c[0].msgs) for c in caps]
        for ch in (0, 1):
            profiles, raws = caps[ch][0].msgs, caps[ch][1].msgs
            assert [m.ping_number for m in profiles] == list(range(1, n + 1))
            assert {m.channel_number for m in profiles} == {ch}
            assert all(len(m.pwr_results) == 1200 for m in profiles)
            ids = [m.data[4] | (m.data[5] << 8) for m in raws]
            assert ids.count(2198) == n
            assert ids.count(2194) == n // 20          # status frames: ~/raw only
            # The device was commanded SonarView's interval and enabled.
            sent = _params_sent(devices[ch])
            assert sent[-1][2] == 50 and sent[-1][9] == 1
    finally:
        for w in workers:
            w.stop()
        for d in devices:
            d.stop()
    for d in devices:                                  # stop() leaves it silent
        assert _params_sent(d)[-1][9] == 0


def test_health_counts_missing_pings_and_restarts():
    h = drv._Health()
    for pn, t in ((10, 0), (11, 50), (14, 200), (2, 250)):
        h.profile(pn, t * 1_000_000)
    assert h.profiles == 4 and h.missing == 2 and h.restarts == 1
    assert h.longest_gap_ns == 150_000_000
    h.forget_sequence()
    h.profile(100, 1_000_000_000)
    assert h.missing == 2                               # a fresh start is no gap


def test_declared_defaults_are_sonarviews_acquisition():
    decl = {m.group(1): float(m.group(2)) for m in re.finditer(
        r'declare_parameter\("(\w+)",\s*(-?[\d.]+)\)', NODE_SRC.read_text())}
    assert decl == {"range_start_mm": 0, "range_length_mm": 20000,
                    "msec_per_ping": 50, "gain_index": -1,
                    "num_results": 1200, "pulse_len_percent": 0.00125}
    for launch in ("SSS_processing_launch.py", "SSS_simple_launch.py"):
        text = (PKG_PARENT / "launch" / launch).read_text()
        assert re.search(r"declare_arg\('msec_per_ping',\s*default_value=50\)", text), launch


def test_driver_no_longer_imports_brping():
    assert not re.search(r"^\s*(from|import)\s+brping", NODE_SRC.read_text(), re.M)
