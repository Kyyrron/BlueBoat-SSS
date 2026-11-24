"""The processor publishes the transducers' fore/aft arm (CR-SSS-1, 2026-11-15).

``ProcessedSSSPing.transducer_x_offset`` was a constant 0.0 ("assumed
amidships"). It is now the node parameter ``transducer_x_offset_m``, whose
default is the adopted installation value (``TRANSDUCER_X_OFFSET_M``, -0.345 m:
the simulator's ``model.mount_x_m``), so a recording made before that date
re-processes as it was with ``transducer_x_offset_m:=0.0`` -- a parameter, not
a code fork.

Drives the real node from the source tree, like ``test_processor_assembly.py``
(whose harness it reuses); skips cleanly without a sourced ROS 2 workspace.
The GCS half is ``test_fore_aft_arm.py``.
"""

from __future__ import annotations

import pytest

import test_processor_assembly as tpa

pytestmark = pytest.mark.skipif(
    bool(tpa.MISSING),
    reason=f"node-side tests need a sourced ROS 2 env (missing: {tpa.MISSING})")

if not tpa.MISSING:
    import rclpy
    proc = tpa.proc


def _run(args=None, n_pings: int = 12):
    """A real node started with ``args`` (ROS command-line arguments), fed
    ``n_pings`` two-sided pings; returns (node arm, published rows)."""
    rclpy.init(args=args)
    node = proc.SSSProcessorNode()
    published = []
    node._pub.publish = published.append
    assert not node._svlog.active, "a test must never write .svlog"
    try:
        tpa.feed_odom(node, count=40, period_ns=tpa.PING_PERIOD_NS)
        for i in range(n_pings):
            t = i * tpa.PING_PERIOD_NS
            node._on_port(tpa.make_profile(0, 1000 + i, t))
            node._on_starboard(tpa.make_profile(1, 1000 + i, t + 1_000_000))
        node._flush_pending(drain_all=True)
        return node._transducer_x_offset_m, published
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_the_default_is_the_adopted_installation_value():
    assert proc.TRANSDUCER_X_OFFSET_M == -0.345
    arm, rows = _run()
    assert arm == proc.TRANSDUCER_X_OFFSET_M
    assert len(rows) == 12
    for m in rows:
        # the field is float32 on the wire
        assert m.transducer_x_offset == pytest.approx(-0.345, abs=1e-7)


@pytest.mark.parametrize("text, value", [("0.0", 0.0), ("0", 0.0),
                                         ("-0.6", -0.6), ("-0.345", -0.345)])
def test_the_parameter_sets_what_every_row_carries(text, value):
    """``:=0.0`` is how a session recorded before 2026-11-15 is re-processed
    as it was; ``:=0`` (an integer on the launch line) must not be refused."""
    arm, rows = _run(["--ros-args", "-p", f"transducer_x_offset_m:={text}"])
    assert arm == value
    assert rows and all(m.transducer_x_offset == pytest.approx(value, abs=1e-7)
                        for m in rows)


@pytest.mark.parametrize("text", [".nan", ".inf", "aft", "true"])
def test_a_value_that_is_not_a_length_falls_back_to_the_default(text):
    arm, rows = _run(["--ros-args", "-p", f"transducer_x_offset_m:={text}"])
    assert arm == proc.TRANSDUCER_X_OFFSET_M
    assert rows and all(
        m.transducer_x_offset == pytest.approx(proc.TRANSDUCER_X_OFFSET_M, abs=1e-7)
        for m in rows)


def test_the_arm_changes_nothing_else_in_the_row():
    """Only the one field differs between the old value and the new: the
    pose stays the navigation pose, y and intensities are untouched."""
    _, new = _run()
    _, old = _run(["--ros-args", "-p", "transducer_x_offset_m:=0.0"])
    assert len(new) == len(old) > 0
    for a, b in zip(new, old):
        assert (a.robot_x, a.robot_y) == (b.robot_x, b.robot_y)
        assert a.robot_orientation == b.robot_orientation
        assert a.water_depth == b.water_depth
        assert list(a.port_y) == list(b.port_y)
        assert list(a.starboard_y) == list(b.starboard_y)
        assert list(a.port_intensity_db) == list(b.port_intensity_db)
        assert list(a.starboard_intensity_db) == list(b.starboard_intensity_db)
        assert a.port_ping_number == b.port_ping_number
