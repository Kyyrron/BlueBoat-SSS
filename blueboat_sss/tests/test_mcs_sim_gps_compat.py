"""End-to-end: the GCS anchors from the MCS bridge's simulated GPS feed.

When MCS launches a GPS-anchored mission in Gazebo, nothing in the graph
publishes GPS except MCS itself: its bridge node synthesises
``sensor_msgs/NavSatFix`` on ``/mavros/global_position/global``
(BEST_EFFORT, depth 10, ~5 Hz) from the sim odom, and — crucially —
**never sets** ``status``, so every fix goes out with the message
default ``-2`` (STATUS_UNKNOWN, the ROS 2 Iron+ default). This test
reproduces that publisher on the wire, byte-for-byte in the fields that
matter, and asserts the GCS side (``TelemetryListener`` + ``GeoService``
wired exactly as ``main.py`` / ``main_window.py`` wire them) accepts the
fixes and anchors the map.

Needs a sourced ROS 2 environment; skips cleanly without one, so the
suite stays laptop-runnable. Availability is probed with ``find_spec``,
never ``importorskip`` (see CLAUDE.md, Headless GUI testing).
"""

from __future__ import annotations

import importlib.util
import threading
import time

import pytest

REQUIRED = ("rclpy", "sensor_msgs", "nav_msgs")
MISSING = [m for m in REQUIRED if importlib.util.find_spec(m) is None]

pytestmark = pytest.mark.skipif(
    bool(MISSING),
    reason=f"needs a sourced ROS 2 env (missing: {MISSING})")

if not MISSING:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import NavSatFix

# The MCS smoke-test regression values: a non-trivial translation
# |t| ≈ 44 m — with the boat at the world origin every frame bug is
# invisible.
LAT0, LON0 = 43.6961, 7.3080
WX, WY = 37.0, -23.0            # boat world position while GPS appears


def test_gcs_anchors_from_mcs_style_sim_gps(qapp, tmp_config):
    from blueboat_gcs.core.geo_service import GeoService
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.ros.telemetry_listener import TelemetryListener

    rclpy.init()
    executor = None
    spin = None
    try:
        # ---- the MCS bridge mimic (publisher side) ----------------------
        pub_node = rclpy.create_node("mcs_bridge_mimic")
        best_effort = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        gps_pub = pub_node.create_publisher(
            NavSatFix, tmp_config.topics.navsat, best_effort)
        odom_pub = pub_node.create_publisher(
            Odometry, tmp_config.topics.odom, 10)

        # ---- the GCS side, wired as in main.py / main_window.py ---------
        gcs_node = rclpy.create_node("blueboat_gcs_under_test")
        signals = AppSignals()
        TelemetryListener(gcs_node, signals, tmp_config.topics,
                          gps_fallback=tmp_config.alignment.gps_fallback)
        geo = GeoService(tmp_config.geo, require_anchor=True)
        signals.robot_state.connect(geo.on_robot_state)
        signals.gps_fix.connect(geo.on_gps_fix)
        states = []
        signals.robot_state.connect(states.append)

        executor = SingleThreadedExecutor()
        executor.add_node(pub_node)
        executor.add_node(gcs_node)
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()

        # The regression premise: an untouched NavSatFix carries the
        # STATUS_UNKNOWN default, which the GCS must accept.
        fix_proto = NavSatFix()
        assert fix_proto.status.status < 0, (
            "premise changed: NavSatFix() no longer defaults to a "
            "negative status — this regression test guards nothing")

        deadline = time.monotonic() + 15.0
        next_gps = 0.0
        while time.monotonic() < deadline and not geo.ready:
            odom = Odometry()
            odom.pose.pose.position.x = WX
            odom.pose.pose.position.y = WY
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            now = time.monotonic()
            if now >= next_gps:                     # ~5 Hz, as the bridge
                next_gps = now + 0.2
                fix = NavSatFix()                   # status left at default
                fix.latitude, fix.longitude = LAT0, LON0
                gps_pub.publish(fix)
            qapp.processEvents()                    # deliver queued signals
            time.sleep(0.05)

        assert geo.ready, \
            "GCS never anchored from the MCS-style simulated GPS feed"
        # The anchor must place the boat's world position on the fix.
        lat, lon = geo.local_to_gps(WX, WY)
        assert (lat, lon) == pytest.approx((LAT0, LON0), abs=1e-6)
        # And the left-panel path must carry the fix (the "no fix" symptom).
        assert states and states[-1].lat == pytest.approx(LAT0)
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        rclpy.shutdown()
        if spin is not None:
            spin.join(timeout=2.0)
        qapp.processEvents()
