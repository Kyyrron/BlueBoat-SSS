"""AI seabed pictures on the shared display model (core/seabed_imager.py).

* both row geometries ("square" — speed-corrected, the provisional
  default — and "ping", the revert switch) produce valid, invertible
  tiles with the documented JSON keys;
* square rows copy exactly one ping's bins verbatim and record it;
* the picture uses the SAME mapping as the waterfall (parity);
* a live-shaped ping (raw profile attached) and a replay-shaped ping
  give byte-identical pictures.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from blueboat_gcs.config.settings import AppConfig
from blueboat_gcs.core.display_model import DisplayModel
from blueboat_gcs.core.seabed_imager import SeabedImager, feed_pings
from blueboat_gcs.core.waterfall_service import WaterfallService
from blueboat_gcs.models.sonar import SonarPing

from test_display_model import PITCH, N, make_row

H = 3.0


def make_pings(n: int, step_m: float, seed: int = 0):
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        row = make_row(H, rng)
        out.append(SonarPing(
            t=0.05 * i, robot_x=step_m * i, robot_y=0.0, yaw=0.0,
            water_depth=H, y_local=np.zeros(0), intensity_db=np.zeros(0, np.float32),
            bin_size_m=PITCH, port_bin0=0, port_db=row[:N][::-1].copy(),
            stbd_bin0=0, stbd_db=row[N:].copy(), bottom_slant_m=H, gain_index=4))
    return out


def run(cfg, pings, model=None):
    imager = SeabedImager(cfg, model=model)
    out = []
    imager.image_ready.connect(out.append)
    feed_pings(imager, pings)
    return out


@pytest.mark.parametrize("geometry", ["square", "ping"])
def test_both_geometries_produce_invertible_tiles(qapp, tmp_path, geometry):
    cfg = AppConfig()
    cfg.seabed.rows, cfg.seabed.stride = 32, 16
    cfg.seabed.row_geometry = geometry
    pings = make_pings(120, step_m=PITCH * 1.5)       # boat outruns the pitch
    model = DisplayModel.fit(cfg, pings)
    images = run(cfg, pings, model)
    assert len(images) >= 3
    img = images[1]
    assert img.intensity_db.shape == (32, 2 * N)
    assert img.row_geometry == geometry
    assert np.isfinite(img.intensity_db).all()          # nothing erased
    meta = img.metadata()
    assert meta["schema"] == 4 and meta["row_geometry"] == geometry
    assert meta["display_model"]["schema"].startswith("blueboat_display_model")
    for key in ("ping_index", "along_track_m", "bottom_m", "altitude_m"):
        assert key in meta["rows_data"][0]
    png = img.to_png8()
    assert png.dtype == np.uint8 and png.max() > 150
    # Invert the PNG through the stored model: unclipped pixels return to
    # the raw dB within the 8-bit quantisation.
    snap = img.display
    back = snap.invert_rows(png / 255.0, img.bin_pitch_m, img.row_bottom)
    u = snap.to_unit(snap.normalise(img.intensity_db, img.bin_pitch_m, img.row_bottom))
    good = (u > 0.1) & (u < 0.98)
    assert good.sum() > 1000
    step_db = (10.0 / snap.gamma) * np.log10(1.0 + 1.0 / 25.0)
    assert np.abs(back[good] - img.intensity_db[good]).max() < 2.0 * step_db
    # The saved artifacts carry the model and the row maps.
    png_path, json_path = img.save(tmp_path)
    with np.load(json_path.with_name(json_path.stem + "_world.npz")) as z:
        assert z["intensity_db"].shape == img.intensity_db.shape
        assert z["row_ping_index"].shape == (32,)
        assert "display_a_port" in z and "display_model_json" in z
    d = json.loads(json_path.read_text())
    assert d["display_model"]["gamma"] == pytest.approx(cfg.display.gamma)


def test_square_rows_are_verbatim_pings_with_a_row_map(qapp):
    cfg = AppConfig()
    cfg.seabed.rows, cfg.seabed.stride = 40, 20
    cfg.seabed.row_geometry = "square"
    step = PITCH * 1.5           # fast boat: a ping spans 1.5 rows -> repeats
    pings = make_pings(200, step_m=step)
    model = DisplayModel.fit(cfg, pings)
    images = run(cfg, pings, model)
    img = images[2]
    idx = img.row_ping_index
    along = img.row_along_m
    # Rows are one pitch apart along-track, newest first.
    assert np.allclose(np.diff(along), -PITCH, atol=1e-9)
    # Every row is a verbatim copy of one buffered ping's row (the rows
    # map to the pings that are nearest along-track); consecutive rows
    # repeat a ping when the boat moved more than a pitch between pings,
    # and skip pings when it moved less (the "square" trade-off).
    assert (np.diff(idx) <= 0).all()
    assert len(np.unique(idx)) < idx.size
    slow = run(cfg, make_pings(200, step_m=PITCH * 0.4), model)[2]
    assert len(np.unique(slow.row_ping_index)) == slow.row_ping_index.size
    for r in range(img.intensity_db.shape[0]):
        row = img.intensity_db[r]
        matches = [p for p in pings
                   if np.array_equal(np.concatenate([p.port_db[::-1], p.stbd_db]), row)]
        assert len(matches) == 1, "a picture row must be one ping's bins verbatim"
    # Interpolated poses advance monotonically along the track.
    x = img.row_pose[:, 0]
    assert (np.diff(x) <= 1e-9).all() and x[0] > x[-1]


def test_ping_geometry_is_the_old_contract(qapp):
    cfg = AppConfig()
    cfg.seabed.rows, cfg.seabed.stride = 16, 8
    cfg.seabed.row_geometry = "ping"
    pings = make_pings(64, step_m=0.1)
    images = run(cfg, pings, DisplayModel.fit(cfg, pings))
    # 64 pings, first window at 16, then every 8: 7 full + no tail
    assert len(images) == 7
    img = images[0]
    assert list(img.row_ping_index) == list(range(16))
    assert img.row_t[0] > img.row_t[-1]                  # newest on top


def test_picture_and_waterfall_share_the_mapping(qapp, tmp_config):
    """The same ping renders to the same grey in the tile and in the
    waterfall tile (the model's own gamma)."""
    cfg = tmp_config
    cfg.seabed.rows, cfg.seabed.stride = 16, 8
    cfg.seabed.row_geometry = "ping"
    pings = make_pings(64, step_m=0.1)
    model = DisplayModel.fit(cfg, pings)
    images = run(cfg, pings, model)
    img = images[-1]
    svc = WaterfallService(cfg, model)
    for p in pings:
        svc.on_sonar_ping(p)
    chrono = svc.chronological()
    unit = svc.render_unit(chrono, svc._meta_chrono()[:, 5])
    wf_png = np.rint(np.nan_to_num(unit, nan=0.0) * 255).astype(np.uint8)
    # the last image = the last 16 pings, newest first
    np.testing.assert_array_equal(img.to_png8(), wf_png[-16:][::-1])


def test_live_and_replay_pings_give_identical_pictures(qapp):
    """A live row (raw profile attached, bottom from the processor's
    altitude) and the replay row from the same bytes yield the same PNG."""
    from blueboat_gcs.core.live_native import (ProcessedFields, ProfileCache,
                                               attach_native, profile_from_msg)
    from test_live_native import fake_profile, ground_from
    cfg = AppConfig()
    cfg.seabed.rows, cfg.seabed.stride = 8, 4
    cfg.seabed.row_geometry = "ping"
    replay, live = [], []
    for n in range(1, 30):
        pm, sm = fake_profile(0, n, seed=1), fake_profile(1, n, seed=2)
        _, _, pp = profile_from_msg(pm)
        _, _, sp = profile_from_msg(sm)
        base = dict(t=0.05 * n, robot_x=0.1 * n, robot_y=0.0, yaw=0.0,
                    water_depth=H, y_local=np.zeros(0),
                    intensity_db=np.zeros(0, np.float32))
        replay.append(SonarPing(**base, bin_size_m=pp.length_mm / 1000 / (pp.num_results - 1),
                                port_bin0=0, port_db=pp.db, stbd_bin0=0, stbd_db=sp.db,
                                bottom_slant_m=H, gain_index=4))
        cache = ProfileCache()
        cache.put(0, n, pp); cache.put(1, n, sp)
        py, pd = ground_from(pp, H); sy, sd = ground_from(sp, H)
        out, hits = attach_native(ProcessedFields(n, n, H, py, pd, sy, sd), cache)
        assert hits == (True, True)
        live.append(SonarPing(**base, **out))
    model = DisplayModel.fit(cfg, replay)
    a = run(cfg, live, model)
    b = run(cfg, replay, model)
    assert len(a) == len(b) > 0
    for ia, ib in zip(a, b):
        np.testing.assert_array_equal(ia.to_png8(), ib.to_png8())
        np.testing.assert_array_equal(ia.intensity_db, ib.intensity_db)
