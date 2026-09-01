"""Live heading policy: compass first, odom yaw fallback.

The MCS rule: the compass (degrees clockwise from north) is converted
exactly once at ingestion — θ = wrap(radians(90 − hdg)) — and is the
heading everything live draws with; odom yaw covers a silent compass.
This is the fix for the field's misaligned live SSS pings.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from blueboat_gcs.models.sonar import SonarPing
from blueboat_gcs.utils.pose_alignment import HeadingPolicy


def test_compass_conversion_is_the_mcs_formula():
    p = HeadingPolicy(compass_stale_s=2.0)
    p.update_compass(0.0, 0.0)                       # due north
    assert p.heading(0.0) == pytest.approx(math.pi / 2)
    p.update_compass(0.0, 90.0)                      # due east
    assert p.heading(0.0) == pytest.approx(0.0)
    p.update_compass(0.0, 180.0)                     # due south
    assert p.heading(0.0) == pytest.approx(-math.pi / 2)
    p.update_compass(0.0, 270.0)                     # due west: wrapped
    assert abs(p.heading(0.0)) == pytest.approx(math.pi)


def test_stale_compass_falls_back_to_odom_yaw():
    p = HeadingPolicy(compass_stale_s=2.0)
    assert p.heading(0.0) is None
    p.update_odom(0.0, 0.7)
    assert p.heading(0.0) == pytest.approx(0.7)
    p.update_compass(1.0, 0.0)
    assert p.heading(1.5) == pytest.approx(math.pi / 2)   # compass fresh
    assert p.heading(10.0) == pytest.approx(0.7)          # compass stale


def _ping(yaw: float) -> SonarPing:
    y = np.linspace(1.0, 10.0, 50)
    return SonarPing(t=0.0, robot_x=0.0, robot_y=0.0, yaw=yaw,
                     water_depth=2.0,
                     y_local=np.concatenate([y, -y]),
                     intensity_db=np.zeros(100, dtype=np.float32))


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


def test_live_ping_yaw_restamped_from_compass(win, qapp):
    window, signals = win
    assert window._config.alignment.heading_source == "compass"
    import time
    signals.compass_heading.emit(time.monotonic(), 0.0)   # boat faces north
    qapp.processEvents()
    aligned = window._align_ping_pose(_ping(yaw=0.3))     # odom quat wrong
    assert aligned.yaw == pytest.approx(math.pi / 2)


def test_embedded_mode_keeps_ping_yaw(win, qapp):
    window, signals = win
    window._config.alignment.heading_source = "embedded"
    import time
    signals.compass_heading.emit(time.monotonic(), 0.0)
    qapp.processEvents()
    aligned = window._align_ping_pose(_ping(yaw=0.3))
    assert aligned.yaw == pytest.approx(0.3)
