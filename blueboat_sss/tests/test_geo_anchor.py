"""GPS-anchored map: the MCS regression pattern, ported with the code.

Key property under guard (from MCS smoke_test): everything must hold
under a NON-TRIVIAL translation |t| > 1 m — with t = 0 every frame bug
is invisible. Round trips must be exact inverses, the anchor must
converge from a stationary vehicle, and the map must stay gated until
it does.
"""

from __future__ import annotations

import math
import time

import pytest

from blueboat_gcs.config.settings import GeoConfig
from blueboat_gcs.core.geo_service import GeoService
from blueboat_gcs.mapping.geo import (GeoFit, GeoReferencer,
                                      latlon_to_local_en, local_en_to_latlon)
from blueboat_gcs.models.robot_state import RobotState

LAT0, LON0 = 43.6961, 7.3080
# The referencer latches (lat0, lon0) from the FIRST accepted fix, so the
# translation it estimates is t = -(world position at the first fix).
# Put the boat 44 m from its world origin when GPS appears -> a
# non-trivial |t| ≈ 44 m, the MCS regression value.
W0 = (37.0, -23.0)              # boat world position at the first fix
T_TRUE = (-W0[0], -W0[1])       # = (-37, 23)


def _fit() -> GeoFit:
    return GeoFit(tx=T_TRUE[0], ty=T_TRUE[1], lat0=LAT0, lon0=LON0,
                  rms_m=0.5, n_pairs=10)


# ------------------------------------------------------------- round trips
def test_world_en_round_trip_exact():
    fit = _fit()
    x, y = 12.34, -56.78
    ex, ny = fit.world_to_enu(x, y)
    assert (ex, ny) != (x, y)                     # non-vacuous: t ≠ 0
    assert fit.enu_to_world(ex, ny) == pytest.approx((x, y), abs=1e-12)


def test_latlon_world_round_trip():
    fit = _fit()
    lat, lon = fit.world_to_latlon(12.34, -56.78)
    assert fit.latlon_to_world(lat, lon) == pytest.approx((12.34, -56.78),
                                                          abs=1e-6)


def test_projection_pair_is_exact_inverse():
    e, n = latlon_to_local_en(43.7000, 7.3100, LAT0, LON0)
    lat, lon = local_en_to_latlon(e, n, LAT0, LON0)
    assert (lat, lon) == pytest.approx((43.7000, 7.3100), abs=1e-12)


# ----------------------------------------------------------- geo referencer
def _gps_of_world(x: float, y: float):
    """GPS a receiver would report at world (x, y), given the first fix
    (LAT0, LON0) was taken at world W0 — the projection origin."""
    return local_en_to_latlon(x - W0[0], y - W0[1], LAT0, LON0)


def test_stationary_convergence_and_glitch_robustness():
    ref = GeoReferencer(GeoConfig())
    lat, lon = _gps_of_world(*W0)
    for k in range(5):                       # boat holding position at W0
        ref.add_pair(float(k), W0[0], W0[1], lat, lon)
    assert ref.is_valid
    assert ref.fit.tx == pytest.approx(T_TRUE[0], abs=0.05)
    assert ref.fit.ty == pytest.approx(T_TRUE[1], abs=0.05)

    # One GPS glitch must not drag the median or kill validity.
    ref.add_pair(5.0, W0[0], W0[1], lat + 0.001, lon)  # ~111 m outlier
    ref.add_pair(11.0, W0[0], W0[1], lat, lon)         # forces a refit
    assert ref.is_valid
    assert ref.fit.tx == pytest.approx(T_TRUE[0], abs=0.05)


def test_no_fix_sentinel_rejected():
    ref = GeoReferencer(GeoConfig())
    for k in range(10):
        ref.add_pair(float(k), 1.0, 2.0, 0.0, 0.0)
    assert ref.fit is None


# -------------------------------------------------------------- geo service
def test_service_pairs_only_with_fresh_odom(qapp):
    svc = GeoService(GeoConfig(), require_anchor=True)
    lat, lon = _gps_of_world(*W0)
    now = time.monotonic()
    svc.on_robot_state(RobotState(t=0.0, x=W0[0], y=W0[1], yaw=0.0))
    for k in range(6):                        # stale odom: > 0.5 s later
        svc.on_gps_fix(now + 1.0 + k, lat, lon)
    assert not svc.ready

    anchored = []
    svc.anchored.connect(lambda la, lo: anchored.append((la, lo)))
    for k in range(6):                        # fresh pairing
        svc.on_robot_state(RobotState(t=0.0, x=W0[0], y=W0[1], yaw=0.0))
        svc.on_gps_fix(time.monotonic(), lat, lon)
    assert svc.ready
    assert anchored and anchored[0] == pytest.approx((lat, lon))
    assert svc.translation == pytest.approx(T_TRUE, abs=0.05)
    # Exact inverse through the service boundary.
    assert svc.en_to_world(*svc.world_to_en(3.0, 4.0)) == pytest.approx(
        (3.0, 4.0), abs=1e-9)


def test_fixed_fit_is_the_replay_anchor(qapp):
    svc = GeoService(GeoConfig(), require_anchor=True)
    svc.set_fixed_fit(LAT0, LON0, 10.0, 20.0)
    assert svc.ready
    assert svc.translation == (-10.0, -20.0)
    # The world origin of the log maps back to the recorded lat/lon.
    assert svc.local_to_gps(10.0, 20.0) == pytest.approx((LAT0, LON0),
                                                         abs=1e-9)


def test_bypass_mode_is_identity(qapp):
    svc = GeoService(GeoConfig(), require_anchor=False)
    assert svc.ready
    assert svc.translation == (0.0, 0.0)
    assert svc.local_to_gps(1.0, 2.0) is None        # nothing to georeference
    assert svc.en_to_world(1.0, 2.0) == (1.0, 2.0)


# --------------------------------------------------------- window-level gate
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


def test_map_gated_until_anchor_then_opens(win, qapp):
    window, signals = win
    assert window._config.map.require_gps_anchor
    assert not window.world_root.item.isVisible()
    assert not window.map_view._interactive
    # isHidden(), not isVisible(): the window itself is never shown in
    # the offscreen suite, so children can never report visible.
    assert not window.map_view._waiting.isHidden()

    lat, lon = _gps_of_world(*W0)
    for _ in range(6):
        signals.robot_state.emit(RobotState(t=0.0, x=W0[0], y=W0[1],
                                            yaw=0.0))
        signals.gps_fix.emit(time.monotonic(), lat, lon)
        qapp.processEvents()

    assert window.geo.ready
    assert window.world_root.item.isVisible()
    assert window.map_view._interactive
    assert window.map_view._waiting.isHidden()
    # The world root carries the fit translation (scene y is flipped).
    pos = window.world_root.item.pos()
    assert pos.x() == pytest.approx(T_TRUE[0], abs=0.1)
    assert pos.y() == pytest.approx(-T_TRUE[1], abs=0.1)
    # Click round trip: EN scene coordinates -> world -> GPS and back.
    ex, ny = window.geo.world_to_en(7.0, -3.0)
    assert window.geo.en_to_world(ex, ny) == pytest.approx((7.0, -3.0),
                                                           abs=1e-9)
