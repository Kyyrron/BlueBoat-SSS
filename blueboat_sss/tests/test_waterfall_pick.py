"""Waterfall click -> world point -> map selection.

The pixel->world formula is the documented seabed-image convention
(``core/seabed_imager.waterfall_pixel_to_world``), now in the native
slant-bin domain: a column's slant range is ``(k + 0.5) * pitch`` about
the centre seam and the row's altitude turns it into a lateral ground
offset. The service's detection overlay applies its exact inverse; the
window handler resolves a (row, col) click through the row's stored pose
to a world position, marks it on the map's SelectionLayer and fills the
coordinate card.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from blueboat_gcs.core.seabed_imager import waterfall_pixel_to_world
from blueboat_gcs.core.waterfall_service import WaterfallService
from blueboat_gcs.models.sonar import SonarPing

N_BINS = 400                # device bins per side
PITCH = 0.05                # slant metres per bin
DEPTH = 2.0                 # altitude under the boat
RANGE_M = N_BINS * PITCH    # 20 m half extent
W = 2 * N_BINS


def make_ping(i: int, yaw: float = 0.0) -> SonarPing:
    """A ping on the device's uniform slant grid, water column cut —
    exactly what the processor emits after slant-range correction."""
    slant = np.arange(N_BINS) * PITCH
    keep = slant > DEPTH
    ground = np.sqrt(slant[keep] ** 2 - DEPTH ** 2)
    y = np.concatenate([ground, -ground])
    v = np.full(y.size, -30.0, np.float32)
    return SonarPing(
        t=float(i), robot_x=10.0 + i, robot_y=-4.0, yaw=yaw,
        water_depth=DEPTH, y_local=y, intensity_db=v,
        slant_range_m=RANGE_M)


def col_y_local(col: float) -> float:
    """Reference implementation of the documented formula."""
    half = W / 2.0
    k = (half - 1.0 - col) if col < half else (col - half)
    s = (k + 0.5) * PITCH
    g = math.sqrt(max(s * s - DEPTH * DEPTH, 0.0))
    return g if col < half else -g


# ---- the formula -----------------------------------------------------------

def test_column_extremes_map_to_port_and_starboard():
    # yaw 0 = facing +x; port (+y_local) is +y world.
    y0 = col_y_local(0)
    wx, wy = waterfall_pixel_to_world(5.0, 7.0, 0.0, DEPTH, PITCH, 0, W)
    assert y0 > 0.9 * RANGE_M            # far port range, ground-corrected
    assert (float(wx), float(wy)) == pytest.approx((5.0, 7.0 + y0))
    wx, wy = waterfall_pixel_to_world(5.0, 7.0, 0.0, DEPTH, PITCH, W - 1, W)
    assert (float(wx), float(wy)) == pytest.approx((5.0, 7.0 - y0))


def test_centre_columns_are_the_nadir():
    """The two seam columns are bin 0 — inside the water column, so they
    map to the point under the boat (the physical nadir)."""
    for col in (W // 2 - 1, W // 2):
        wx, wy = waterfall_pixel_to_world(5.0, 7.0, 1.234, DEPTH, PITCH,
                                          col, W)
        assert (float(wx), float(wy)) == pytest.approx((5.0, 7.0))


def test_yaw_rotates_the_swath():
    # Facing +y (yaw 90°): port points to -x.
    y0 = col_y_local(0)
    wx, wy = waterfall_pixel_to_world(0.0, 0.0, math.pi / 2, DEPTH, PITCH,
                                      0, W)
    assert (float(wx), float(wy)) == pytest.approx((-y0, 0.0), abs=1e-9)


def test_matches_the_seabed_image_world_grid():
    """The click formula and the per-pixel grids of a seabed image are
    the same function — literally, since _build calls it — so a click on
    a waterfall pixel names the same world point the dataset metadata
    records for that pixel."""
    x, y, yaw = 3.0, -2.0, 0.7
    j = np.arange(W, dtype=np.float64)
    wx, wy = waterfall_pixel_to_world(x, y, yaw, DEPTH, PITCH, j, W)
    for col in (0, 123, W // 2, W - 1):
        sx, sy = waterfall_pixel_to_world(x, y, yaw, DEPTH, PITCH,
                                          float(col), W)
        assert (float(sx), float(sy)) == pytest.approx((wx[col], wy[col]))


# ---- service round trip ----------------------------------------------------

def test_service_adopts_the_native_layout(qapp, tmp_config):
    svc = WaterfallService(tmp_config)
    svc.on_sonar_ping(make_ping(0))
    assert svc.columns == W
    assert svc.pitch_m == pytest.approx(PITCH)
    assert svc.half_extent_m == pytest.approx(RANGE_M)


def test_rows_have_no_holes_outside_the_water_column(qapp, tmp_config):
    """The defect that motivated the native-bin rewrite: the old fixed
    800-column scatter left NaN holes all over each row. Now every
    column beyond the water column carries a sample."""
    svc = WaterfallService(tmp_config)
    svc.on_sonar_ping(make_ping(0))
    row = svc.chronological()[0]
    half = svc.columns // 2
    wc_bins = int(np.floor(DEPTH / PITCH)) + 1   # bins with slant <= depth
    port = row[:half][::-1]
    stbd = row[half:]
    for side in (port, stbd):
        assert np.isnan(side[:wc_bins - 1]).all(), "water column must be empty"
        assert np.isfinite(side[wc_bins:]).all(), "no holes past the FBR"


def test_overlay_is_the_exact_inverse(qapp, tmp_config):
    """A world point built by the forward formula lands back on its own
    (row, col) through the service's detection overlay."""
    svc = WaterfallService(tmp_config)
    for i in range(30):
        svc.on_sonar_ping(make_ping(i, yaw=0.3))
    row, col = 17, 211
    meta = svc.row_meta(row)
    assert meta is not None
    _t, rx, ry, yaw, depth = meta
    wx, wy = waterfall_pixel_to_world(rx, ry, yaw, depth, svc.pitch_m,
                                      float(col), svc.columns)
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
    assert meta == pytest.approx((3.0, 13.0, -4.0, 0.25, DEPTH))
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
            meta[1], meta[2], meta[3], meta[4],
            window.waterfall_service.pitch_m, float(col),
            window.waterfall_service.columns)
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


def test_view_shows_newest_ping_on_top(qapp, tmp_config):
    """SonarView orientation: the newest ping is at the top of the view.

    The service still stores rows oldest-first; only the view maps them
    with ``scene y = -(row)`` so the newest (largest index) sits at the
    smallest y. A click near the top must therefore resolve to the newest
    rows, near the bottom to the oldest.
    """
    from blueboat_gcs.gui.waterfall_view import WaterfallView

    svc = WaterfallService(tmp_config)
    view = WaterfallView()
    svc.layout_changed.connect(view.on_layout)
    svc.tile_updated.connect(view.on_tile)
    svc.set_enabled(True)
    for i in range(20):
        svc.on_sonar_ping(make_ping(i))
    svc._force_render()
    qapp.processEvents()

    rect = view._scene.sceneRect()
    # Scene spans [-total, -row0): newest at the top, oldest at the bottom.
    assert rect.top() == pytest.approx(-svc.total_rows)
    assert rect.bottom() == pytest.approx(-svc.first_row)
    # The click mapping (row = floor(-y)) puts the top at the newest row.
    top_row = int(math.floor(-(rect.top() + 0.5)))
    bot_row = int(math.floor(-(rect.bottom() - 0.5)))
    assert top_row == svc.total_rows - 1        # newest ping, at the top
    assert bot_row == svc.first_row             # oldest ping, at the bottom


def test_true_scale_stretches_rows_and_keeps_clicks_exact(qapp, tmp_config):
    """True scale (default on) draws each row ``row_pitch / col_pitch``
    taller than a column; the click mapping goes through the same view
    transform, so a viewport point resolves to the same (row, col)
    whichever mode is on."""
    from PySide6.QtCore import QPoint
    from blueboat_gcs.gui.waterfall_view import WaterfallView

    svc = WaterfallService(tmp_config)
    view = WaterfallView()
    view.resize(900, 700)
    svc.layout_changed.connect(view.on_layout)
    svc.tile_updated.connect(view.on_tile)
    svc.set_enabled(True)
    from dataclasses import replace as dc_replace
    for i in range(40):
        # 0.5 m between pings against a 0.05 m bin pitch: aspect 10.
        svc.on_sonar_ping(dc_replace(make_ping(i), robot_x=0.5 * i))
    svc._force_render()
    qapp.processEvents()
    assert svc.row_pitch_m == pytest.approx(0.5)
    assert view.aspect == pytest.approx(10.0)
    t = view.transform()
    assert t.m22() / t.m11() == pytest.approx(10.0)

    # A scene point maps out and back to the same (row, col) in both modes.
    scene_pt = (123.5, -(17 + 0.5))                # column 123, row 17
    for on in (True, False):
        view.set_true_scale(on)
        t = view.transform()
        assert t.m22() / t.m11() == pytest.approx(10.0 if on else 1.0)
        vp = view.mapFromScene(*scene_pt)
        back = view.mapToScene(QPoint(vp.x(), vp.y()))
        # Exact to within one device pixel in scene units, in both axes.
        assert abs(back.x() - scene_pt[0]) <= 1.0 / t.m11() + 1e-9
        assert abs(back.y() - scene_pt[1]) <= 1.0 / t.m22() + 1e-9
