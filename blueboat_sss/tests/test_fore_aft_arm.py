"""The transducers' fore/aft lever arm: one value, applied wherever a ping is placed.

2026-11-15 (CR-SSS-1 / CR-SSS-2 of the simulator completion round). The
simulator renders every ping from a point 0.345 m aft of ``base_link``; until
this date the processor published ``transducer_x_offset = 0.0`` and the GCS
placed every ping from the navigation point. The error lies along the heading,
so it flips with it: one object seen on two opposite passes landed
``2 x 0.345 = 0.69 m`` apart.

What is pinned here (ROS-free; the processor's parameter is in
``test_processor_fore_aft_arm.py``):

* **one set of values**: the processor's constants, the GCS mirror, the GCS
  config default, the launch default, and the simulator's
  ``config/default_sonar.yaml`` when the superproject checkout is there;
* **the rule**: a ping is placed from ``pose + R(yaw)·(arm, 0)``;
* **the effect**, on every placement path (mosaic in both modes, waterfall
  click, seabed-picture grids): the same point seen heading east and heading
  west coincides with the arm and is ``2·|arm|`` apart without it;
* **nothing moves without an arm**: arm 0 is bit for bit what it was
  (the Perception goldens, untouched, are the wider check), and a picture built
  from such pings still says georef convention 2;
* **the pictures say what they did**: convention 3 and ``lever_arm_m.x``
  with an arm, the row pose still the navigation pose, a change of arm closes
  the window;
* **where the value comes from**: live, the ping's own field; replay and
  rebuild, ``alignment.transducer_x_offset_m``; recorded in ``metadata.json``.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import replace as dc_replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from conftest import PKG_PARENT, REPO_ROOT

from blueboat_gcs.config.settings import AlignmentConfig, AppConfig, load_config
from blueboat_gcs.core import svlog as gcs_svlog
from blueboat_gcs.core.live_native import transducer_x_offset_of
from blueboat_gcs.core.recording_session import session_x_offset
from blueboat_gcs.core.seabed_imager import (FORMULA_CONVENTION_2,
                                             FORMULA_CONVENTION_3, GEOREF,
                                             LEVER_ARM_M, georef_for,
                                             waterfall_pixel_to_world)
from blueboat_gcs.core.waterfall_service import WaterfallService
from blueboat_gcs.mapping.rasterizer import PingRasterizer
from blueboat_gcs.models.sonar import SonarPing, sensor_point

from test_seabed_pictures import run as _images

ARM = gcs_svlog.TRANSDUCER_X_OFFSET_M          # -0.345: 0.345 m aft

# ---- one synthetic scene -------------------------------------------------------
# A point target at TARGET. East pass along y = 0 (target 8 m to port),
# west pass along y = 16 (target 8 m to port again). 20 pings/s at 1 m/s.
TARGET = (20.0, 8.0)
ACROSS = 8.0                      # the target's |y_local| on both passes
STEP = 0.05                       # metres between pings
N_PINGS = 81                      # the abeam ping is the middle one
MID = N_PINGS // 2
H = 2.0                           # altitude (tracked bottom)
PITCH = 0.05                      # slant metres per device bin
N_BINS = 400                      # 20 m


def _target_bin() -> int:
    """The device bin holding the target: slant range of a ground offset
    ``ACROSS - lever`` from the transducer at altitude H."""
    ground = ACROSS - LEVER_ARM_M[0]
    return int(math.hypot(ground, H) / PITCH)


def pass_pings(heading: str, arm: float, placed_with: float | None = None):
    """The pings of one pass over TARGET.

    The boat sails so that its TRANSDUCERS (``arm`` behind the navigation
    point) are abeam of the target on ping MID -- what the water does. Each
    ping then carries ``placed_with`` as its ``x_offset_m`` (default: the
    true arm; 0.0 = a consumer that ignores it).
    """
    yaw = 0.0 if heading == "east" else math.pi
    y_line = TARGET[1] - ACROSS if heading == "east" else TARGET[1] + ACROSS
    ux = math.cos(yaw)                              # +1 east, -1 west
    nav_x_abeam = TARGET[0] - ux * arm              # sensor = nav + ux*arm
    slant = (np.arange(N_BINS) + 0.5) * PITCH
    keep = slant > H
    ground = np.sqrt(slant[keep] ** 2 - H ** 2)
    y_local = np.concatenate([LEVER_ARM_M[0] + ground,
                              -(LEVER_ARM_M[1] + ground)])
    out = []
    for i in range(N_PINGS):
        port = np.full(N_BINS, -60.0, np.float32)
        stbd = np.full(N_BINS, -60.0, np.float32)
        v = np.full(y_local.size, -60.0, np.float32)
        if i == MID:
            port[_target_bin()] = 0.0
            v[int(np.argmin(np.abs(y_local - ACROSS)))] = 0.0
        out.append(SonarPing(
            t=0.05 * i, robot_x=nav_x_abeam + ux * STEP * (i - MID),
            robot_y=y_line, yaw=yaw, water_depth=H + 0.22,
            y_local=y_local, intensity_db=v, slant_range_m=N_BINS * PITCH,
            bin_size_m=PITCH, port_bin0=0, port_db=port, stbd_bin0=0,
            stbd_db=stbd, bottom_slant_m=H, gain_index=4,
            port_ping_number=1000 + i, stbd_ping_number=1000 + i,
            x_offset_m=arm if placed_with is None else placed_with))
    return out


# ---- one set of values ---------------------------------------------------------

def _processor_constants() -> dict:
    """The processor's geometry constants, read from its source text (so
    this runs without ROS)."""
    text = (PKG_PARENT / "src" / "sss_processor_node.py").read_text()
    out = {}
    for name in ("TRANSDUCER_X_OFFSET_M", "TRANSDUCER_Y_OFFSET_PORT_M",
                 "TRANSDUCER_Y_OFFSET_STBD_M", "TRANSDUCER_SUBMERSION_M"):
        m = re.search(rf"^{name}\s*:\s*float\s*=\s*([-+0-9.eE]+)", text, re.M)
        assert m, f"{name} not found in sss_processor_node.py"
        out[name] = float(m.group(1))
    return out


def test_processor_and_gcs_hold_the_same_installation():
    proc = _processor_constants()
    assert proc["TRANSDUCER_X_OFFSET_M"] == gcs_svlog.TRANSDUCER_X_OFFSET_M == -0.345
    assert proc["TRANSDUCER_Y_OFFSET_PORT_M"] == gcs_svlog.TRANSDUCER_Y_OFFSET_PORT_M
    assert proc["TRANSDUCER_Y_OFFSET_STBD_M"] == gcs_svlog.TRANSDUCER_Y_OFFSET_STBD_M
    assert proc["TRANSDUCER_SUBMERSION_M"] == gcs_svlog.TRANSDUCER_SUBMERSION_M


def test_every_default_of_the_arm_is_the_same_number():
    """The dataclass default, the shipped YAML and the launch argument."""
    assert AlignmentConfig().transducer_x_offset_m == ARM
    assert load_config().alignment.transducer_x_offset_m == ARM
    launch = (PKG_PARENT / "launch" / "SSS_processing_launch.py").read_text()
    m = re.search(r"declare_arg\('transducer_x_offset_m',\s*default_value=([-+0-9.eE]+)\)",
                  launch)
    assert m and float(m.group(1)) == ARM
    assert "'transducer_x_offset_m': sl_transducer_x_offset_m" in launch


def test_the_simulator_renders_from_the_same_point():
    """CM-2: the simulator's single source of every profile agrees. Read
    only; skipped outside the superproject checkout."""
    sim = (REPO_ROOT.parent / "BlueBoat-SSS-Sim" / "blueboat_sss_sim" / "config"
           / "default_sonar.yaml")
    if not sim.is_file():
        pytest.skip(f"no simulator checkout at {sim}")
    yaml = pytest.importorskip("yaml")
    model = yaml.safe_load(sim.read_text())["model"]
    proc = _processor_constants()
    assert float(model["mount_x_m"]) == proc["TRANSDUCER_X_OFFSET_M"]
    assert float(model["mount_y_abs_m"]) == proc["TRANSDUCER_Y_OFFSET_PORT_M"] \
        == proc["TRANSDUCER_Y_OFFSET_STBD_M"]
    assert float(model["sensor_depth_m"]) == proc["TRANSDUCER_SUBMERSION_M"]


# ---- the rule --------------------------------------------------------------------

def test_sensor_point_follows_the_heading():
    for yaw, expected in ((0.0, (10.0 + ARM, 5.0)),             # east: behind = west
                          (math.pi, (10.0 - ARM, 5.0)),         # west: behind = east
                          (math.pi / 2, (10.0, 5.0 + ARM))):    # north: behind = south
        x, y = sensor_point(10.0, 5.0, yaw, ARM)
        assert (x, y) == pytest.approx(expected, abs=1e-12)
    p = pass_pings("west", ARM)[0]
    assert (p.sensor_x, p.sensor_y) == pytest.approx(
        (p.robot_x - ARM, p.robot_y), abs=1e-12)


def test_no_arm_returns_the_pose_itself():
    """Arm 0 is not 'plus zero': the very same objects come back, so a
    negative zero stays one and nothing computed before the arm moves."""
    x = np.array([-0.0, 3.0])
    y = np.array([1.0, -0.0])
    sx, sy = sensor_point(x, y, np.array([0.3, 2.0]), 0.0)
    assert sx is x and sy is y
    p = pass_pings("east", ARM, placed_with=0.0)[3]
    assert (p.sensor_x, p.sensor_y) == (p.robot_x, p.robot_y)
    # A non-finite arm is no arm, whoever built the ping: SonarPing is
    # the one place the value is made safe.
    for bad in (math.nan, math.inf):
        q = dc_replace(p, x_offset_m=bad)
        assert q.x_offset_m == 0.0
        assert (q.sensor_x, q.sensor_y) == (q.robot_x, q.robot_y)


def test_pixel_to_world_without_an_arm_is_bit_identical():
    rng = np.random.default_rng(3)
    x = rng.normal(size=(40, 1)); x[0, 0] = -0.0
    y = rng.normal(size=(40, 1)); y[1, 0] = -0.0
    yaw = rng.uniform(-math.pi, math.pi, size=(40, 1)); yaw[0, 0] = 0.0
    bottom = np.full((40, 1), H)
    j = np.arange(2 * N_BINS, dtype=np.float64)[None, :]
    old = waterfall_pixel_to_world(x, y, yaw, bottom, PITCH, j, 2 * N_BINS,
                                   *LEVER_ARM_M)
    new = waterfall_pixel_to_world(x, y, yaw, bottom, PITCH, j, 2 * N_BINS,
                                   *LEVER_ARM_M, lever_x=0.0)
    for a, b in zip(old, new):
        assert a.tobytes() == b.tobytes()


def test_pixel_to_world_with_the_arm_moves_along_the_heading_only():
    yaw = np.array([[0.0], [0.7], [math.pi], [-2.0]])
    x = np.array([[1.0], [2.0], [3.0], [4.0]])
    y = np.array([[5.0], [6.0], [7.0], [8.0]])
    bottom = np.full((4, 1), H)
    j = np.arange(2 * N_BINS, dtype=np.float64)[None, :]
    wx0, wy0 = waterfall_pixel_to_world(x, y, yaw, bottom, PITCH, j,
                                        2 * N_BINS, *LEVER_ARM_M)
    wx, wy = waterfall_pixel_to_world(x, y, yaw, bottom, PITCH, j,
                                      2 * N_BINS, *LEVER_ARM_M, lever_x=ARM)
    np.testing.assert_allclose(wx - wx0, np.cos(yaw) * ARM + 0 * j, atol=1e-12)
    np.testing.assert_allclose(wy - wy0, np.sin(yaw) * ARM + 0 * j, atol=1e-12)
    # along the heading: no across-track component
    across = -(wx - wx0) * np.sin(yaw) + (wy - wy0) * np.cos(yaw)
    np.testing.assert_allclose(across, 0.0, atol=1e-12)


# ---- the effect: two opposite passes over one point ---------------------------

def _rasterised_target(pings) -> tuple:
    """World position of the bright sample through the mosaic rasteriser."""
    ras = PingRasterizer(cell_size_m=0.05)
    best, where = -1e9, None
    for p in pings:
        xs, ys, vs, _ = ras.rasterize(p)
        k = int(np.argmax(vs))
        if vs[k] > best:
            best, where = float(vs[k]), (float(xs[k]), float(ys[k]))
    return where


def _mosaic_target(cfg, pings) -> tuple:
    """Centre of the brightest mosaic cell after the whole pass."""
    from blueboat_gcs.core.mosaic_service import MosaicService
    svc = MosaicService(cfg)
    for p in pings:
        svc.on_sonar_ping(p)
    svc.flush_warmup()
    grid = svc._grid
    img = grid.render("average")
    cy, cx = np.unravel_index(int(np.nanargmax(img)), img.shape)
    return (grid._x0 + (cx + 0.5) * grid.cell_size_m,
            grid._y0 + (cy + 0.5) * grid.cell_size_m)


def _clicked_target(cfg, pings) -> tuple:
    """A waterfall click on the bright pixel, resolved as both windows do."""
    svc = WaterfallService(cfg)
    for p in pings:
        svc.on_sonar_ping(p)
    row = MID
    col = svc.columns // 2 - 1 - _target_bin()
    assert svc.chronological()[row, col] == 0.0, "not the target's pixel"
    return svc.pixel_to_world(row, col)


def _pictured_target(cfg, pings) -> tuple:
    """The bright pixel's world position in the seabed pictures' own grids."""
    for img in _images(cfg, pings):
        r, c = np.unravel_index(int(np.nanargmax(img.intensity_db)),
                                img.intensity_db.shape)
        if img.intensity_db[r, c] == 0.0:
            return img.pixel_to_world(int(r), int(c))
    raise AssertionError("the target is in no picture")


def _config(tmp_config, *, densify=True) -> AppConfig:
    cfg = tmp_config
    cfg.mosaic.auto_cell_size = False
    cfg.mosaic.cell_size_m = 0.05
    cfg.mosaic.densify = densify
    cfg.mosaic.nadir_contrast = False       # raw dB: the bright cell is the target
    cfg.seabed.rows, cfg.seabed.stride = 64, 32
    cfg.seabed.row_geometry = "ping"
    return cfg


PATHS = {
    "rasteriser": lambda cfg, pings: _rasterised_target(pings),
    "mosaic": _mosaic_target,
    "mosaic_scatter": _mosaic_target,
    "waterfall_click": _clicked_target,
    "seabed_picture": _pictured_target,
}
#: placement tolerance of each path [m]: a grid cell or a slant bin.
TOL = {"rasteriser": 0.03, "mosaic": 0.08, "mosaic_scatter": 0.08,
       "waterfall_click": 0.03, "seabed_picture": 0.03}


@pytest.mark.parametrize("path", sorted(PATHS))
def test_opposite_passes_meet_with_the_arm_and_part_without_it(
        qapp, tmp_config, path):
    cfg = _config(tmp_config, densify=(path != "mosaic_scatter"))
    place = PATHS[path]
    tol = TOL[path]

    # The arm applied: both passes put the target on itself.
    east = place(cfg, pass_pings("east", ARM))
    west = place(cfg, pass_pings("west", ARM))
    assert east == pytest.approx(TARGET, abs=tol), path
    assert west == pytest.approx(TARGET, abs=tol), path
    assert abs(east[0] - west[0]) <= 2 * tol

    # The same water, placed without the arm (before 2026-11-15): each pass
    # carries the target |arm| ahead of itself, so the two are 0.69 m apart.
    east0 = place(cfg, pass_pings("east", ARM, placed_with=0.0))
    west0 = place(cfg, pass_pings("west", ARM, placed_with=0.0))
    assert east0[0] - TARGET[0] == pytest.approx(-ARM, abs=tol)     # ahead = east
    assert west0[0] - TARGET[0] == pytest.approx(+ARM, abs=tol)     # ahead = west
    assert east0[0] - west0[0] == pytest.approx(2 * abs(ARM), abs=2 * tol)
    assert east0[1] == pytest.approx(TARGET[1], abs=tol)            # across-track
    assert west0[1] == pytest.approx(TARGET[1], abs=tol)            # is unchanged


def test_the_swath_line_passes_through_the_transducers(qapp, tmp_config,
                                                       no_modal_dialogs):
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.gui.main_window import MainWindow

    from test_recording_guards import FakeAcquisition

    tmp_config.alignment.pose_source = "embedded"
    tmp_config.alignment.heading_source = "embedded"
    window = MainWindow(tmp_config, AppSignals(), MosaicService(tmp_config),
                        FakeAcquisition())
    try:
        window._viz_enabled = True
        ping = pass_pings("west", ARM)[5]
        window._on_sonar_ping_data(ping)
        sx, sy, yaw, _range = window._pending_swath
        assert (sx, sy) == pytest.approx((ping.robot_x - ARM, ping.robot_y))
        assert yaw == ping.yaw
        # ...and a waterfall click on that row is placed from the same point.
        svc = window.waterfall_service
        col = svc.columns // 4
        _rx, _ry, _yaw, bottom = svc.row_georef(0)
        assert svc.pixel_to_world(0, col) == pytest.approx(
            waterfall_pixel_to_world(sx, sy, yaw, bottom, svc.pitch_m,
                                     float(col), svc.columns, *LEVER_ARM_M),
            abs=1e-9)
    finally:
        window.close()
        qapp.processEvents()


def test_the_detection_overlay_needs_no_arm(qapp, tmp_config):
    """A world point placed WITH the arm comes back on its own pixel: the
    overlay's row is the detection's time and its column the across-track
    component, which a fore/aft arm does not touch."""
    svc = WaterfallService(tmp_config)
    pings = pass_pings("west", ARM)
    for p in pings:
        svc.on_sonar_ping(p)
    row, col = MID, svc.columns // 2 - 1 - _target_bin()
    wx, wy = svc.pixel_to_world(row, col)
    received = []
    svc.detections_updated.connect(received.extend)
    svc.add_detection(pings[row].t, wx, wy, "target")
    svc.set_enabled(True)
    hit = [d for d in received if d["label"] == "target"]
    assert hit and hit[-1]["row"] == row
    assert abs(hit[-1]["col"] - col) <= 1


# ---- the pictures say what they did ---------------------------------------------

@pytest.mark.parametrize("geometry", ["ping", "square"])
def test_pictures_without_an_arm_are_convention_2_as_before(
        qapp, tmp_config, geometry):
    cfg = _config(tmp_config)
    cfg.seabed.row_geometry = geometry
    images = _images(cfg, pass_pings("east", ARM, placed_with=0.0))
    assert images
    for img in images:
        meta = img.metadata()
        assert meta["georef"] == GEOREF == georef_for(0.0)
        assert meta["schema"] == 5
        assert meta["pixel_convention"]["formula"] == FORMULA_CONVENTION_2
        assert FORMULA_CONVENTION_2.startswith("georef convention 2: ")


@pytest.mark.parametrize("geometry", ["ping", "square"])
def test_pictures_with_the_arm_are_convention_3(qapp, tmp_config, geometry):
    cfg = _config(tmp_config)
    cfg.seabed.row_geometry = geometry
    plain = _images(cfg, pass_pings("west", ARM, placed_with=0.0))
    armed = _images(cfg, pass_pings("west", ARM))
    assert len(plain) == len(armed) > 0
    for a, b in zip(plain, armed):
        meta = b.metadata()
        assert meta["georef"] == {
            "convention": 3, "altitude": "bottom_m",
            "lever_arm_m": {"x": ARM, "y_port": LEVER_ARM_M[0],
                            "y_stbd": LEVER_ARM_M[1]}}
        assert meta["schema"] == 5                    # no key added
        assert set(meta) == set(a.metadata())
        assert meta["pixel_convention"]["formula"] == FORMULA_CONVENTION_3
        assert "georef.lever_arm_m.x" in FORMULA_CONVENTION_3
        # The picture itself and its row pose (the navigation pose) are
        # the same; only the world grids moved, by the arm along the heading.
        assert a.intensity_db.tobytes() == b.intensity_db.tobytes()
        assert a.row_pose.tobytes() == b.row_pose.tobytes()
        yaw = b.row_pose[:, 2:3]
        np.testing.assert_allclose(b.world_x - a.world_x,
                                   np.cos(yaw) * ARM + 0 * a.world_x, atol=2e-5)
        np.testing.assert_allclose(b.world_y - a.world_y,
                                   np.sin(yaw) * ARM + 0 * a.world_y, atol=2e-5)
        # ...and the grids are the documented formula.
        j = np.arange(b.intensity_db.shape[1], dtype=np.float64)[None, :]
        wx, wy = waterfall_pixel_to_world(
            b.row_pose[:, 0:1], b.row_pose[:, 1:2], yaw,
            np.maximum(b.row_bottom[:, None], 0.0), b.bin_pitch_m, j,
            b.intensity_db.shape[1], *LEVER_ARM_M,
            lever_x=meta["georef"]["lever_arm_m"]["x"])
        assert wx.astype(np.float32).tobytes() == b.world_x.tobytes()
        assert wy.astype(np.float32).tobytes() == b.world_y.tobytes()


def test_a_change_of_arm_closes_the_picture(qapp, tmp_config):
    """One arm per picture: no window mixes pings placed two ways."""
    cfg = _config(tmp_config)
    first = pass_pings("east", ARM, placed_with=0.0)
    second = [dc_replace(p, t=p.t + 10.0, port_ping_number=p.port_ping_number + 500,
                         stbd_ping_number=p.stbd_ping_number + 500)
              for p in pass_pings("east", ARM)]
    images = _images(cfg, first + second)
    conventions = [img.metadata()["georef"]["convention"] for img in images]
    assert set(conventions) == {2, 3}
    assert conventions == sorted(conventions), "pictures out of order"
    for img, conv in zip(images, conventions):
        pns = img.row_ping_numbers[:, 0]
        if conv == 2:
            assert pns.max() < 1500, "an armed ping in an arm-less picture"
        else:
            assert pns.min() >= 1500, "an arm-less ping in an armed picture"


# ---- where the value comes from ----------------------------------------------------

def test_live_pings_take_the_processors_own_value():
    f32 = float(np.float32(ARM))                          # the field is float32
    assert transducer_x_offset_of(SimpleNamespace(transducer_x_offset=f32)) \
        == pytest.approx(ARM, abs=1e-7)
    # a processor built before 2026-11-15, a missing field, a bad value
    assert transducer_x_offset_of(SimpleNamespace(transducer_x_offset=0.0)) == 0.0
    assert transducer_x_offset_of(SimpleNamespace()) == 0.0
    assert transducer_x_offset_of(SimpleNamespace(transducer_x_offset=math.nan)) == 0.0
    assert transducer_x_offset_of(SimpleNamespace(transducer_x_offset="x")) == 0.0


def test_the_listener_reads_the_field_and_nothing_else_places_a_ping():
    """Source-level guard: the live listener stamps the field, and no
    placement site reads the navigation point where it means the sensor."""
    gcs = PKG_PARENT / "blueboat_gcs"
    assert "x_offset_m=transducer_x_offset_of(msg)" in \
        (gcs / "ros" / "sonar_listener.py").read_text()
    assert "_PrevPing(ping.sensor_x, ping.sensor_y" in \
        (gcs / "mapping" / "rasterizer.py").read_text()
    assert "project_to_world(ping.sensor_x, ping.sensor_y" in \
        (gcs / "core" / "mosaic_service.py").read_text()
    assert "(ping.sensor_x, ping.sensor_y, ping.yaw" in \
        (gcs / "gui" / "main_window.py").read_text()
    for window in ("main_window.py", "replay_window.py"):
        assert "self.waterfall_service.pixel_to_world(row, col)" in \
            (gcs / "gui" / window).read_text()


def test_replay_stamps_the_configured_arm(tmp_path):
    from blueboat_gcs.core.svlog import load_svlog
    from test_svlog_replay import build_log

    log = build_log(tmp_path / "a.svlog", pings=12)
    default = load_svlog(log)
    assert default.ping_count == 12
    assert {p.x_offset_m for p in default.pings} == {ARM}
    old = load_svlog(log, transducer_x_offset_m=0.0)
    assert {p.x_offset_m for p in old.pings} == {0.0}
    # the pose itself is untouched: the arm is carried, never folded in
    assert [(p.robot_x, p.robot_y, p.yaw) for p in old.pings] == \
        [(p.robot_x, p.robot_y, p.yaw) for p in default.pings]
    assert {p.x_offset_m for p in
            load_svlog(log, transducer_x_offset_m=math.nan).pings} == {0.0}


@pytest.mark.parametrize("arm", [ARM, 0.0])
def test_a_rebuilt_session_records_the_arm_it_used(qapp, tmp_config, tmp_path, arm):
    """``alignment.transducer_x_offset_m`` reaches a session rebuild, and
    the session says which arm placed it (0.0 = as before 2026-11-15)."""
    from blueboat_gcs.core.session_rebuild import rebuild_session
    from test_svlog_replay import build_log

    cfg = _config(tmp_config)
    cfg.seabed.rows, cfg.seabed.stride = 16, 8
    cfg.alignment.transducer_x_offset_m = arm
    session = tmp_path / f"session_{arm}"
    session.mkdir()
    log = build_log(session / "a.svlog", pings=40)
    mission = rebuild_session(log, session, cfg)
    assert {p.x_offset_m for p in mission.pings} == {arm}
    meta = json.loads((session / "metadata.json").read_text())
    assert meta["transducer_x_offset_m"] == arm
    pictures = sorted((session / "seabed_images" / "metadata").glob("seabed_*.json"))
    assert pictures
    for js in pictures:
        georef = json.loads(js.read_text())["georef"]
        assert georef["lever_arm_m"]["x"] == arm
        assert georef["convention"] == (3 if arm else 2)


def test_session_metadata_value():
    assert session_x_offset([]) is None
    assert session_x_offset([ARM]) == ARM
    assert session_x_offset([0.0, ARM]) == [0.0, ARM]


def test_a_live_session_records_the_arm_of_its_pings(qapp, tmp_config):
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.recording_session import RecordingManager
    from blueboat_gcs.core.signals import AppSignals

    signals = AppSignals()
    manager = RecordingManager(tmp_config, signals, MosaicService(tmp_config),
                               WaterfallService(tmp_config))
    assert manager.begin()
    for p in pass_pings("east", ARM)[:5]:
        signals.sonar_ping.emit(p)
    session = manager.session_dir
    manager._write_metadata(session, manager._start_wall, [])
    meta = json.loads((Path(session) / "metadata.json").read_text())
    assert meta["ping_count"] == 5
    assert meta["transducer_x_offset_m"] == ARM
