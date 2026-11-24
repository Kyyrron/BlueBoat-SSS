"""Pinger gating: the marker appears only on a genuine detection.

Field bug: robot_interface streams zeros(3) at ~20 Hz before the USBL
has ever seen the pinger; the old listener accepted it and the marker
rode the boat. The pure gate lives in utils/pinger.py; frame handling
and staleness live in the main window.
"""

from __future__ import annotations

import math
import time

import pytest

from blueboat_gcs.models.detection import PingerFix
from blueboat_gcs.utils.pinger import parse_pinger


# ----------------------------------------------------------------- pure gate
@pytest.mark.parametrize("data, expected", [
    ([], None),
    ([1.0], None),                              # malformed: too short
    ([0.0, 0.0], None),                         # world-shape placeholder
    ([0.0, 0.0, 0.0], None),                    # pre-detection zeros(3)
    ([float("nan"), 1.0, 2.0], None),           # NaN
    ([3.0, 4.0, 0.0], (3.0, 4.0, "body")),      # native USBL 3-vector
    ([3.0, 4.0, -1.5], (3.0, 4.0, "body")),
    ([12.0, 8.0], (12.0, 8.0, "world")),        # fixed_pinger 2-vector
    ([0.0, 0.0, 2.0], (0.0, 0.0, "body")),      # z alone proves detection
])
def test_parse_pinger_table(data, expected):
    assert parse_pinger(data) == expected


# ------------------------------------------------------- window integration
@pytest.fixture
def win(qapp, tmp_config, no_modal_dialogs):
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.gui.main_window import MainWindow
    from blueboat_gcs.sim.simulator import Simulator

    signals = AppSignals()
    window = MainWindow(tmp_config, signals, MosaicService(tmp_config),
                        Simulator(tmp_config, signals))
    yield window, signals
    window.close()
    qapp.processEvents()


def _robot_state(x=10.0, y=5.0, yaw=math.pi / 2):
    from blueboat_gcs.models.robot_state import RobotState
    return RobotState(t=0.0, x=x, y=y, yaw=yaw)


def test_body_fix_rotates_through_robot_pose(win, qapp):
    window, signals = win
    signals.robot_state.emit(_robot_state())
    qapp.processEvents()
    signals.pinger_fix.emit(PingerFix(t=0.0, x=3.0, y=0.0,
                                      accuracy_m=1.0, frame="body"))
    qapp.processEvents()
    assert window.pinger_layer._has_fix
    # x forward at yaw=pi/2 (facing north): world = (10, 5) + (0, 3).
    line = window.pinger_layer._cross_a.line()
    cx = (line.x1() + line.x2()) / 2.0
    cy = (line.y1() + line.y2()) / 2.0
    assert cx == pytest.approx(10.0, abs=1e-6)
    assert cy == pytest.approx(-8.0, abs=1e-6)   # scene y = -world y


def test_world_fix_passes_through(win, qapp):
    window, signals = win
    signals.pinger_fix.emit(PingerFix(t=0.0, x=45.0, y=25.0,
                                      accuracy_m=1.0, frame="world"))
    qapp.processEvents()
    line = window.pinger_layer._cross_a.line()
    assert (line.x1() + line.x2()) / 2.0 == pytest.approx(45.0, abs=1e-6)


def test_staleness_hides_marker_until_next_fix(win, qapp, monkeypatch):
    window, signals = win
    window._config.alignment.pinger_stale_after_s = 5.0
    signals.pinger_fix.emit(PingerFix(t=0.0, x=45.0, y=25.0, frame="world"))
    qapp.processEvents()
    group = window.pinger_layer._group

    window._check_telemetry_staleness()          # fresh: stays
    assert window.pinger_layer._has_fix

    t0 = time.monotonic()
    monkeypatch.setattr(time, "monotonic", lambda: t0 + 60.0)
    window._check_telemetry_staleness()
    assert not window.pinger_layer._has_fix
    assert not group.isVisible()

    signals.pinger_fix.emit(PingerFix(t=1.0, x=45.0, y=25.0, frame="world"))
    qapp.processEvents()
    assert window.pinger_layer._has_fix          # re-shows on the next fix
