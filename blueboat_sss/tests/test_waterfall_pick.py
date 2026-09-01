"""Waterfall click -> world point -> map selection.

The pixel->world formula is the documented seabed-image convention
(``core/seabed_imager.waterfall_pixel_to_world``); the service's
detection overlay applies its exact inverse; the window handler resolves
a (row, col) click through the row's stored pose to a world position,
marks it on the map's SelectionLayer and fills the coordinate card.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from blueboat_gcs.core.seabed_imager import waterfall_pixel_to_world
from blueboat_gcs.core.waterfall_service import WaterfallService
from blueboat_gcs.models.sonar import SonarPing

W = 800
RANGE_M = 20.0


def make_ping(i: int, yaw: float = 0.0) -> SonarPing:
    y = np.linspace(RANGE_M, -RANGE_M, 64)
    return SonarPing(
        t=float(i), robot_x=10.0 + i, robot_y=-4.0, yaw=yaw,
        water_depth=2.0, y_local=y,
        intensity_db=np.full(64, -30.0, np.float32),
        slant_range_m=RANGE_M)


# ---- the formula -----------------------------------------------------------

def test_column_extremes_map_to_port_and_starboard():
    # yaw 0 = facing +x; port (+y_local) is +y world.
    wx, wy = waterfall_pixel_to_world(5.0, 7.0, 0.0, RANGE_M, 0, W)
    assert (wx, wy) == pytest.approx((5.0, 7.0 + RANGE_M))
    wx, wy = waterfall_pixel_to_world(5.0, 7.0, 0.0, RANGE_M, W - 1, W)
    assert (wx, wy) == pytest.approx((5.0, 7.0 - RANGE_M))


def test_centre_column_is_the_nadir():
    wx, wy = waterfall_pixel_to_world(5.0, 7.0, 1.234, RANGE_M,
                                      (W - 1) / 2.0, W)
    assert (wx, wy) == pytest.approx((5.0, 7.0))


def test_yaw_rotates_the_swath():
    # Facing +y (yaw 90°): port points to -x.
    wx, wy = waterfall_pixel_to_world(0.0, 0.0, math.pi / 2, RANGE_M, 0, W)
    assert (wx, wy) == pytest.approx((-RANGE_M, 0.0), abs=1e-9)


def test_matches_the_seabed_image_world_grid():
    """The click formula and the per-pixel grids of a seabed image are
    the same function — literally, since _build calls it — so a click on
    a waterfall pixel names the same world point the dataset metadata
    records for that pixel."""
    x, y, yaw, r = 3.0, -2.0, 0.7, 15.0
    j = np.arange(W, dtype=np.float64)
    wx, wy = waterfall_pixel_to_world(x, y, yaw, r, j, W)
    for col in (0, 123, W // 2, W - 1):
        sx, sy = waterfall_pixel_to_world(x, y, yaw, r, float(col), W)
        assert (sx, sy) == pytest.approx((wx[col], wy[col]))


# ---- service round trip ----------------------------------------------------

def test_overlay_is_the_exact_inverse(qapp, tmp_config):
    """A world point built by the forward formula lands back on its own
    (row, col) through the service's detection overlay."""
    svc = WaterfallService(tmp_config)
    for i in range(30):
        svc.on_sonar_ping(make_ping(i, yaw=0.3))
    row, col = 17, 211
    meta = svc.row_meta(row)
    assert meta is not None
    _t, rx, ry, yaw, r = meta
    wx, wy = waterfall_pixel_to_world(rx, ry, yaw, r, float(col), svc._cols)
    received = []
    svc.detections_updated.connect(received.extend)
    svc.add_detection(float(row), float(wx), float(wy), "target")
    svc.set_enabled(True)
    hit = [d for d in received if d["label"] == "target"]
    assert hit, "the detection never reached the overlay"
    assert hit[-1]["row"] == row
    assert abs(hit[-1]["col"] - col) <= 1        # one px of quantisation


def test_row_meta_returns_the_ingested_pose(qapp, tmp_config):
    svc = WaterfallService(tmp_config)
    for i in range(5):
        svc.on_sonar_ping(make_ping(i, yaw=0.25))
    meta = svc.row_meta(3)
    assert meta == pytest.approx((3.0, 13.0, -4.0, 0.25, RANGE_M))
    assert svc.row_meta(99) is None


# ---- window handler --------------------------------------------------------

def test_click_marks_the_map_and_the_card(qapp, tmp_config, no_modal_dialogs):
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.gui.main_window import MainWindow

    from test_recording_guards import FakeAcquisition

    window = MainWindow(tmp_config, AppSignals(), MosaicService(tmp_config),
                        FakeAcquisition())
    try:
        # Anchor the map the way replay does — the SelectionLayer lives
        # under the GPS-gated WorldRoot, so an unanchored map (correctly)
        # shows no marker.
        window.geo.set_fixed_fit(43.6961, 7.3080, 0.0, 0.0)
        window.world_root.set_ready(True)
        for i in range(10):
            window.waterfall_service.on_sonar_ping(make_ping(i))
        row, col = 6, 100
        window._on_waterfall_point(row, col)

        meta = window.waterfall_service.row_meta(row)
        wx, wy = waterfall_pixel_to_world(
            meta[1], meta[2], meta[3], meta[4], float(col),
            tmp_config.mosaic.waterfall_columns)
        group = window.selection_layer._group
        assert group.isVisible(), "the map marker did not appear"
        assert group.pos().x() == pytest.approx(float(wx))
        assert group.pos().y() == pytest.approx(-float(wy))   # scene y-flip
        assert window.waterfall_view._selected == (row, col)

        # A gap row refuses politely.
        window.waterfall_service.break_row()
        window._on_waterfall_point(10, 100)      # the gap row
        assert window.waterfall_view._selected == (row, col), (
            "a gap row must not move the selection")
    finally:
        window.close()
        qapp.processEvents()
