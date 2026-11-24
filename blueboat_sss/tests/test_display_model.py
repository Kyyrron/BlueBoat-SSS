"""The one dB->pixel display model (core/display_model.py).

Synthetic pings only — no ROS, no corpus, no display. The invariants
pinned here are the ones the field screenshots violated
(docs/SCIENTIFIC_BACKGROUND.md §8):

* near and far seabed render alike after the range/angle normalisation;
* a homogeneous seabed renders the same at any altitude (the curve is
  in normalised slant range x = r/h);
* acoustic shadows stay black — no low handle for them to contaminate;
* the water column / ringing core darken by physics, not by masking;
* a wall at a fixed range across many pings cannot band the image
  (median estimator);
* a low far band on one side (a shore) cannot darken that side's nadir
  (the seabed line is the longest unbroken run of bin modes);
* everything inverts back to dB from the stored model (JSON/npz);
* rows without a tracked bottom, warm-up / freeze / reset semantics.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from blueboat_gcs.config.settings import AppConfig
from blueboat_gcs.core.display_model import (DisplayModel, DisplayModelSnapshot,
                                             transmission_loss_db)
from blueboat_gcs.models.sonar import SonarPing

N = 400                     # bins per side
RANGE_M = 20.0
PITCH = RANGE_M / N
SEABED_ONSET_DB = 60.0      # first bottom return level (like the sim/field logs)
FALL_DB_PER_DECADE = -65.0  # measured on the field corpus


def _slant():
    return (np.arange(N) + 0.5) * PITCH


#: Backscatter part of the falloff (the rest is the two-way transmission
#: loss, 40 dB/decade + absorption): the seabed level after TL depends on
#: the grazing angle only, i.e. on x = r/h, at any altitude — which is
#: exactly the invariance the model relies on (SCIENTIFIC_BACKGROUND §2).
BS_DB_PER_DECADE = -25.0
SEABED_LEVEL_DB = SEABED_ONSET_DB + transmission_loss_db(np.array([3.0]), 40.0, 0.1)[0]


def seabed_db(slant: np.ndarray, h: float) -> np.ndarray:
    """Received seabed level: BS(x) - TL(r). Falls ~65 dB/decade of slant
    range at constant altitude; the first bottom return is weaker at a
    larger altitude, as physics says (60 dB at h = 3 m)."""
    x = np.maximum(slant / h, 1.0)
    return (SEABED_LEVEL_DB + BS_DB_PER_DECADE * np.log10(x)
            - transmission_loss_db(slant, 40.0, 0.1))


def make_row(h: float, rng: np.random.Generator, *, texture_db: float = 4.0,
             shadow: tuple = None, wall: tuple = None) -> np.ndarray:
    """One two-sided native row (port mirrored left): ringing core, dark
    water column, then seabed; optional shadow / wall bands (slant m)."""
    s = _slant()
    side = np.full(N, np.nan, np.float32)
    ring = 53.0 - 22.0 * s                       # 53 dB at 0 -> 31 dB at 1 m
    wc = 26.0 + rng.normal(0.0, 3.0, N)          # reverberation
    side[:] = np.maximum(ring, wc)
    sb = s >= h
    side[sb] = seabed_db(s[sb], h) + rng.normal(0.0, texture_db, int(sb.sum()))
    if shadow is not None:
        a, b = shadow
        m = (s >= a) & (s < b)
        side[m] = -5.0 + rng.normal(0.0, 3.0, int(m.sum()))   # receiver floor
    if wall is not None:
        a, b = wall
        m = (s >= a) & (s < b)
        side[m] = seabed_db(s[m], h) + 15.0
    row = np.empty(2 * N, np.float32)
    row[:N] = side[::-1]
    row[N:] = side
    return row


def fit_model(h: float, n_rows: int = 200, seed: int = 0, **kw) -> DisplayModel:
    cfg = AppConfig()
    model = DisplayModel(cfg)
    rng = np.random.default_rng(seed)
    for _ in range(n_rows):
        model.observe_row(make_row(h, rng, **kw), PITCH, h)
    model.freeze()
    return model


def _side_cols(width: int):
    half = width // 2
    k = np.where(np.arange(width) < half, half - 1 - np.arange(width),
                 np.arange(width) - half)
    return (k + 0.5) * PITCH


# ---------------------------------------------------------------------------
def test_near_and_far_seabed_render_alike():
    h = 3.0
    model = fit_model(h)
    rng = np.random.default_rng(7)
    rows = np.vstack([make_row(h, rng) for _ in range(50)])
    u = model.render_unit(rows, PITCH, np.full(50, h))
    s = _side_cols(rows.shape[1])
    near = u[:, (s >= h * 1.2) & (s < h * 1.8)]
    far = u[:, s >= RANGE_M * 0.75]
    assert np.isfinite(near).all() and np.isfinite(far).all()
    # The raw dB differ by ~45 dB between these bands; rendered, they are
    # the same grey to within 15 %.
    assert abs(near.mean() - far.mean()) < 0.15 * max(near.mean(), far.mean())
    assert far.mean() > 0.2, "far seabed crushed to black"
    assert near.mean() < 0.95, "near seabed saturated"


def test_altitude_invariance():
    """The same seabed at 3 m and at 9 m altitude renders to the same
    brightness: the curve lives in x = r/h, not in r."""
    rng = np.random.default_rng(3)
    m3, m9 = fit_model(3.0), fit_model(9.0)
    rows3 = np.vstack([make_row(3.0, rng) for _ in range(40)])
    rows9 = np.vstack([make_row(9.0, rng) for _ in range(40)])
    u3 = m3.render_unit(rows3, PITCH, np.full(40, 3.0))
    u9 = m9.render_unit(rows9, PITCH, np.full(40, 9.0))
    s = _side_cols(rows3.shape[1])
    band3 = u3[:, (s >= 3.0 * 1.5) & (s < 3.0 * 4.0)]
    band9 = u9[:, (s >= 9.0 * 1.5) & (s < 20.0)]
    assert abs(band3.mean() - band9.mean()) < 0.05, (band3.mean(), band9.mean())
    # And a model fitted at one altitude applied to the other still
    # renders the seabed uniformly (no re-fit needed when h changes).
    u_cross = m3.render_unit(rows9, PITCH, np.full(40, 9.0))
    assert abs(u_cross[:, s >= 13.5].mean() - band9.mean()) < 0.08


def test_shadow_stays_black():
    """A shadow 25+ dB below the seabed renders black regardless of how
    many shadows the mission holds: there is no low handle to drag."""
    h = 3.0
    # The same shadow band on 45 % of the rows: plain seabed keeps the
    # plurality in every bin, so the curve ignores it (a permanent shadow
    # on every row IS that range's "seabed" — the documented limit).
    cfg = AppConfig()
    model = DisplayModel(cfg)
    rng = np.random.default_rng(1)
    for i in range(300):
        model.observe_row(make_row(h, rng, shadow=(8.0, 11.0) if i % 20 < 9
                                   else None), PITCH, h)
    model.freeze()
    row = make_row(h, rng, shadow=(8.0, 11.0))
    u = model.render_unit(row, PITCH, np.array([h]))[0]
    s = _side_cols(row.size)
    sh = u[(s >= 8.3) & (s < 10.7)]
    sb = u[(s >= 4.0) & (s < 7.5)]
    assert np.mean(sh) < 0.05 and np.percentile(sh, 95) < 0.1, sh.mean()
    assert np.mean(sb) > 0.25                              # seabed still lit


def test_water_column_and_ringing_go_dark_by_physics():
    h = 3.0
    model = fit_model(h)
    rng = np.random.default_rng(2)
    row = make_row(h, rng)
    u = model.render_unit(row, PITCH, np.array([h]))[0]
    s = _side_cols(row.size)
    wc = u[(s >= 0.3) & (s < h * 0.9)]
    ring = u[s < 0.3]
    assert wc.max() < 0.05, "water column not dark"
    assert ring.max() < 0.05, "ringing core not dark"
    # ...and nothing was erased: the normalised value is finite everywhere.
    e = model.normalise(row, PITCH, np.array([h]))
    assert np.isfinite(e).all()


def test_a_wall_at_fixed_range_does_not_band_the_image():
    """A bright wall + its shadow at one slant range on 30 % of the rows
    must leave the seabed curve at that range untouched (< 1 dB)."""
    h = 3.0
    cfg = AppConfig()
    rng = np.random.default_rng(5)
    clean, walled = DisplayModel(cfg), DisplayModel(cfg)
    for i in range(300):
        r = make_row(h, rng)
        clean.observe_row(r, PITCH, h)
        if i % 10 < 3:
            r = make_row(h, rng, wall=(9.0, 9.6), shadow=(9.6, 12.0))
        walled.observe_row(r, PITCH, h)
    a0 = clean.snapshot().a_port
    a1 = walled.snapshot().a_port
    x = clean.snapshot().x_centers
    band = (x >= 9.0 / h) & (x <= 12.0 / h)
    assert band.any()
    assert np.abs(a1[band] - a0[band]).max() < 1.0, np.abs(a1 - a0).max()


def test_invertible_from_the_stored_model(tmp_path):
    h = 3.0
    model = fit_model(h)
    rng = np.random.default_rng(9)
    rows = np.vstack([make_row(h, rng) for _ in range(8)])
    hs = np.full(8, h)
    snap = model.snapshot()
    e = snap.normalise(rows, PITCH, hs)
    u = snap.to_unit(e)
    back = snap.invert_rows(u, PITCH, hs)
    keep = (u > 0.02) & (u < 0.98)                 # unclipped pixels
    assert keep.sum() > 0.3 * keep.size
    assert np.abs(back[keep] - rows[keep]).max() < 1e-3
    # 8-bit round trip stays within the quantisation step in dB.
    png = snap.to_png8(e)
    back8 = snap.invert_rows(png / 255.0, PITCH, hs)
    step_db = (10.0 / snap.gamma) * np.log10(1.0 + 1.0 / 25.0)   # at u >= 0.1
    good = (u > 0.1) & (u < 0.98)
    assert np.abs(back8[good] - rows[good]).max() < 2.0 * step_db
    # JSON / npz round trip reproduces the mapping exactly.
    d = json.loads(json.dumps(snap.to_json()))
    snap2 = DisplayModelSnapshot.from_json(d)
    assert np.allclose(snap2.normalise(rows, PITCH, hs), e, atol=1e-4)
    items = snap.npz_items()
    np.savez(tmp_path / "m.npz", **items)
    with np.load(tmp_path / "m.npz") as z:
        assert json.loads(str(z["display_model_json"]))["hi_db"] == pytest.approx(snap.hi_db)
        assert z["display_a_port"].shape == snap.a_port.shape


def test_rows_without_a_bottom_use_the_fallback_altitude():
    h = 3.0
    model = fit_model(h)
    rng = np.random.default_rng(4)
    row = make_row(h, rng)
    u_h = model.render_unit(row, PITCH, np.array([h]))
    u_0 = model.render_unit(row, PITCH, np.array([0.0]))
    assert np.isfinite(u_0[np.isfinite(row)[None, :]]).all()
    # The fallback is the mission's median bottom, so the render matches.
    assert abs(model.snapshot().fallback_h - h) < 1e-6
    assert np.allclose(u_h, u_0, equal_nan=True)
    # An h = 0 row still counts toward the warm-up but feeds no bin.
    fresh = DisplayModel(AppConfig())
    fresh.observe_row(row, PITCH, 0.0)
    assert fresh.rows_observed == 1 and not fresh.ready


def test_warmup_publishes_and_keeps_observing():
    """Live, the model re-publishes its estimate; it is never frozen.

    Before 2026-09-19 it froze after ``warmup_rows`` and nothing unfroze it,
    so everything after the first ~15 s was displayed through a curve fitted
    on the launch area while replay fitted the whole file -- which is what
    made replay look better than live. The cost is unchanged per ping: the
    histogram was always accumulated per row, and the solve behind
    ``version`` is cached and runs once per publish.
    """
    cfg = AppConfig()
    cfg.display.warmup_rows = 20
    model = DisplayModel(cfg)
    v0 = model.version
    rng = np.random.default_rng(0)
    for _ in range(19):
        model.observe_row(make_row(3.0, rng), PITCH, 3.0)
    assert not model.frozen and model.version == v0
    model.observe_row(make_row(3.0, rng), PITCH, 3.0)
    assert not model.frozen and model.version == v0 + 1   # published, not frozen
    for _ in range(20):
        model.observe_row(make_row(3.0, rng), PITCH, 3.0)
    assert model.rows_observed == 40 and model.version == v0 + 2
    model.reset()
    assert not model.frozen and model.rows_observed == 0


def test_a_late_regime_change_is_learned_live():
    """A mission that changes altitude half way is displayed on a curve that
    has seen both halves -- the thing freezing at 300 rows made impossible."""
    cfg = AppConfig()
    cfg.display.warmup_rows = 30
    rng = np.random.default_rng(5)
    live = DisplayModel(cfg)
    for _ in range(60):
        live.observe_row(make_row(2.0, rng), PITCH, 2.0)
    for _ in range(60):
        live.observe_row(make_row(6.0, rng), PITCH, 6.0)
    assert live.rows_observed == 120 and live.ready
    # The deep half is rendered sanely: its seabed lands inside the window
    # rather than crushed against an end of it.
    u = live.render_unit(make_row(6.0, rng), PITCH, np.array([6.0]))[0]
    seabed = u[np.isfinite(u)]
    assert 0.05 < float(np.median(seabed)) < 0.95


def test_streaming_matches_a_whole_pass_fit():
    """The live path (observe_row per row) and the replay path (fit over the
    same pings) must agree on the seabed level once both have seen the same
    data. Every other parity test in this repo holds the model FIXED and so
    cannot see a divergence here (tests/test_seabed_pictures.py)."""
    h = 3.0
    rng = np.random.default_rng(17)
    rows = [make_row(h, rng) for _ in range(200)]
    cfg = AppConfig()
    cfg.display.warmup_rows = 50
    live = DisplayModel(cfg)
    for row in rows:
        live.observe_row(row, PITCH, h)
    offline = DisplayModel(cfg)
    offline._warmup = 1 << 60          # noqa: SLF001 - what refit() does
    for row in rows:
        offline.observe_row(row, PITCH, h)
    offline.freeze()
    probe = make_row(h, rng)
    a = live.render_unit(probe, PITCH, np.array([h]))[0]
    b = offline.render_unit(probe, PITCH, np.array([h]))[0]
    ok = np.isfinite(a) & np.isfinite(b)
    assert ok.any()
    assert float(np.max(np.abs(a[ok] - b[ok]))) < 0.02


def test_fit_over_pings_uses_the_native_payload():
    h = 3.0
    rng = np.random.default_rng(11)
    pings = []
    for i in range(60):
        row = make_row(h, rng)
        pings.append(SonarPing(t=float(i), robot_x=float(i), robot_y=0.0, yaw=0.0,
                               water_depth=0.0, y_local=np.zeros(0),
                               intensity_db=np.zeros(0, np.float32),
                               bin_size_m=PITCH, port_bin0=0, port_db=row[:N][::-1],
                               stbd_bin0=0, stbd_db=row[N:], bottom_slant_m=h))
    model = DisplayModel.fit(AppConfig(), pings)
    assert model.frozen and model.ready and model.rows_observed == 60
    u = model.render_unit(make_row(h, rng), PITCH, np.array([h]))[0]
    s = _side_cols(u.size)
    assert u[s >= h * 1.2].mean() > 0.25


def test_transmission_loss_is_the_documented_law():
    r = np.array([1.0, 10.0, 100.0])
    tl = transmission_loss_db(r, 40.0, 0.1)
    assert tl == pytest.approx([0.2, 42.0, 100.0])
    assert transmission_loss_db(np.array([0.0]), 40.0, 0.1)[0] == pytest.approx(-80.0 + 0.002)


def test_far_range_in_permanent_shadow_stays_on_the_physical_line():
    """Enclosed basin: beyond the wall the whole far range is shadow on
    EVERY ping. The bin modes there are the noise floor, 25 dB under the
    seabed; the curve must follow the physical line through the near
    bins instead, so that (a) the shadow renders black and (b) seabed
    that does appear at those ranges renders like the near seabed."""
    h = 3.0
    cfg = AppConfig()
    model = DisplayModel(cfg)
    rng = np.random.default_rng(6)
    for _ in range(300):
        model.observe_row(make_row(h, rng, shadow=(11.0, 20.0)), PITCH, h)
    model.freeze()
    snap = model.snapshot()
    x = snap.x_centers
    inner = (x > 1.3) & (x < 3.0)
    outer = x > 11.0 / h * 1.15
    # The outer curve continues the inner trend (a straight line in
    # log10 x within the tolerance), instead of dropping to the floor.
    line = np.polyfit(np.log10(x[inner]), snap.a_port[inner], 1)
    pred = np.polyval(line, np.log10(x[outer]))
    assert np.abs(snap.a_port[outer] - pred).max() < 2.0 * cfg.display.curve_tolerance_db
    # The shadow renders black; an unshadowed row renders uniform seabed.
    shadowed = model.render_unit(make_row(h, rng, shadow=(11.0, 20.0)), PITCH,
                                 np.array([h]))[0]
    clean = model.render_unit(make_row(h, rng), PITCH, np.array([h]))[0]
    s = _side_cols(shadowed.size)
    sh = shadowed[(s > 12.0) & (s < 19.0)]
    # The floor is ~17 dB under the seabed at these ranges (the fixture's
    # -5 dB floor against a seabed near 12 dB), so the transfer decides how
    # dark that reads: u = 10^(gamma * -17 / 10), 0.07 at gamma 0.7 and 0.14
    # at the 0.5 shipped since 2026-09-18. Assert the physics rather than a
    # number tied to one default -- the speckle mean sits above the transfer
    # of the mean (Jensen), hence the 1.5x margin, and the tail is allowed
    # dark grey.
    u_floor = 10.0 ** (-1.7 * cfg.display.gamma)
    assert sh.mean() < 1.5 * u_floor and np.percentile(sh, 95) < 4.0 * u_floor, (
        sh.mean(), np.percentile(sh, 95), u_floor)
    near = clean[(s > 4.0) & (s < 8.0)].mean()
    far = clean[(s > 12.0) & (s < 19.0)].mean()
    assert abs(far - near) < 0.2 * near


def _flat_row(h: float, rng: np.random.Generator,
              port_far_low_from_x: float = 0.0) -> np.ndarray:
    """A two-sided row over a seabed whose TL-compensated level is FLAT in
    x (a receiver that compensates part of its range loss, as the device
    and the simulator do), optionally with a band 18 dB lower beyond
    ``x = port_far_low_from_x`` on the PORT side only (a shore, or a
    shadow that only one side looks into)."""
    s = _slant()
    sides = []
    for side in (0, 1):
        v = np.maximum(53.0 - 22.0 * s, 26.0 + rng.normal(0.0, 3.0, N))
        sb = s >= h
        v[sb] = (SEABED_LEVEL_DB - transmission_loss_db(s[sb], 40.0, 0.1)
                 + rng.normal(0.0, 4.0, int(sb.sum())))
        if side == 0 and port_far_low_from_x > 0.0:
            v[s >= port_far_low_from_x * h] -= 18.0
        sides.append(v.astype(np.float32))
    row = np.empty(2 * N, np.float32)
    row[:N] = sides[0][::-1]
    row[N:] = sides[1]
    return row


def test_a_one_sided_far_band_does_not_darken_that_sides_nadir():
    """The 2026-09-28 simulated sessions: a flat seabed on both sides, and
    on the shallow rows a low far band (the shore) on port only. The
    consensus used to search the -20 dB/decade prior slope alone; a -20
    line through the middle bins caught that far band and won, the port
    near-nadir bins fell outside the tolerance and took the line, and port
    rendered a dark band beside the nadir on every ping, U-turns included
    (8.7-10.3 dB here, 7-17 dB in the session). Both sides' curves must
    agree near the nadir, and a clean row must render symmetrically."""
    model = DisplayModel(AppConfig())
    rng = np.random.default_rng(3)
    for i in range(300):
        shallow = i % 2 == 1
        h = 0.6 if shallow else 2.0
        model.observe_row(_flat_row(h, rng, 12.0 if shallow else 0.0), PITCH, h)
    model.freeze()
    snap = model.snapshot()
    near = snap.x_centers < 3.0
    gap = np.abs(snap.a_port[near] - snap.a_stbd[near]).max()
    assert gap < 1.0, f"port/starboard curves differ by {gap:.1f} dB near the nadir"
    h = 2.0
    rows = np.vstack([_flat_row(h, rng) for _ in range(40)])
    u = model.render_unit(rows, PITCH, np.full(40, h))
    s = _side_cols(u.shape[1])
    band = (s >= 1.2 * h) & (s < 2.5 * h)
    port = u[:, :N][:, band[:N]].mean()
    stbd = u[:, N:][:, band[N:]].mean()
    assert abs(port - stbd) < 0.05 * max(port, stbd), (port, stbd)


def test_soft_knee_compresses_highlights_and_inverts():
    model = fit_model(3.0)
    snap = model.snapshot()
    e = np.linspace(snap.hi_db - 20.0, snap.hi_db + 20.0, 400)
    u = snap.to_unit(e)
    assert np.all(np.diff(u) >= 0), "the transfer must stay monotonic"
    assert np.all(np.diff(u[u < 0.995]) > 0)           # strictly, until saturation
    assert u[e <= snap.hi_db].max() <= 1.0 and u[-1] > 0.98
    # Below the knee it is the plain power law; above it highlights keep
    # gradation for several dB instead of clipping at hi.
    p = 10 ** (snap.gamma * (e - snap.hi_db) / 10)
    below = p <= snap.knee
    assert np.allclose(u[below], p[below], atol=1e-6)
    just_above = (e > snap.hi_db) & (e < snap.hi_db + 4.0)
    assert np.all(np.diff(u[just_above]) > 1e-4)
    back = snap.invert_unit(u)
    ok = u < 0.995
    assert np.abs(back[ok] - e[ok]).max() < 0.05


# ---------------------------------------------------------------------------
# 2026-09-15: dB-window transfer and receive-gain compensation
# (project_review.md §3.2 / §3.5, gaps G3 and G5)
# ---------------------------------------------------------------------------
def _fit(cfg: AppConfig, h: float = 3.0, seed: int = 0) -> DisplayModel:
    model = DisplayModel(cfg)
    rng = np.random.default_rng(seed)
    for _ in range(200):
        model.observe_row(make_row(h, rng), PITCH, h)
    model.freeze()
    return model


def test_db_window_transfer_is_linear_in_db_and_inverts():
    cfg = AppConfig()
    cfg.display.transfer = "db_window"
    cfg.display.window_db = 40.0
    snap = _fit(cfg).snapshot()
    assert snap.transfer == "db_window" and snap.window_db == 40.0
    W = snap.window_width_db()                  # window_db / gamma
    assert W == pytest.approx(40.0 / snap.gamma)
    assert snap.window_width_db(1.0) == pytest.approx(40.0)
    e = np.linspace(snap.hi_db - 2 * W, snap.hi_db + 10.0, 400)
    u = snap.to_unit(e)
    inside = (e > snap.hi_db - W) & (e < snap.hi_db)
    np.testing.assert_allclose(u[inside], (e[inside] - (snap.hi_db - W)) / W,
                               atol=1e-6)
    assert (u[e <= snap.hi_db - W] == 0.0).all()
    assert (u[e >= snap.hi_db] == 1.0).all()          # no knee: hard clip
    back = snap.invert_unit(u[inside])
    assert np.abs(back - e[inside]).max() < 1e-3
    d = json.loads(json.dumps(snap.to_json()))
    assert d["transfer"] == "db_window" and d["window_db"] == 40.0
    s2 = DisplayModelSnapshot.from_json(d)
    assert s2.transfer == "db_window" and s2.window_db == 40.0
    np.testing.assert_allclose(s2.to_unit(e), u, atol=1e-6)


def test_power_law_stays_the_default_and_unknown_transfers_are_refused():
    cfg = AppConfig()
    assert cfg.display.transfer == "power"
    assert DisplayModel(cfg).snapshot().transfer == "power"
    cfg.display.transfer = "sigmoid"
    with pytest.raises(ValueError):
        DisplayModel(cfg)


def test_gain_offset_is_20log10_of_the_analog_gain_ratio():
    from blueboat_gcs.core.display_model import (GAIN_REFERENCE_ANALOG,
                                                 gain_offset_db)
    assert GAIN_REFERENCE_ANALOG == pytest.approx(74.55)
    assert gain_offset_db(74.55) == pytest.approx(0.0)
    assert gain_offset_db(142.8) == pytest.approx(20 * np.log10(142.8 / 74.55))
    assert gain_offset_db(464.625) == pytest.approx(15.89, abs=0.01)
    assert gain_offset_db(0.0) == 0.0 and gain_offset_db(-1.0) == 0.0  # unknown


def test_gain_step_is_compensated_and_inverts():
    """The same seabed recorded one gain index up (analog 142.8 vs 74.55)
    arrives +5.65 dB louder; after normalisation the two rows agree and
    the inversion returns the louder raw dB."""
    h = 3.0
    g = 20 * np.log10(142.8 / 74.55)
    snap = _fit(AppConfig(), h).snapshot()
    rng = np.random.default_rng(3)
    row = make_row(h, rng)
    louder = row + g
    e0 = snap.normalise(row[None, :], PITCH, np.array([h]))
    e1 = snap.normalise(louder[None, :], PITCH, np.array([h]),
                        np.array([[g, g]], np.float32))
    assert np.nanmax(np.abs(e1 - e0)) < 1e-3
    # Without the offset the louder row would read +g everywhere.
    e_raw = snap.normalise(louder[None, :], PITCH, np.array([h]))
    assert np.nanmedian(e_raw - e0) == pytest.approx(g, abs=1e-3)
    u = snap.to_unit(e1)
    back = snap.invert_rows(u, PITCH, np.array([h]),
                            gain_rows=np.array([[g, g]], np.float32))
    good = (u > 0.05) & (u < 0.65)                  # below the knee, unclipped
    assert good.sum() > 100
    assert np.abs(back[good] - louder[None, :][good]).max() < 1e-2


def test_per_side_gain_offsets_come_from_the_ping():
    cfg = AppConfig()
    model = DisplayModel(cfg)
    ping = SonarPing(t=0.0, robot_x=0.0, robot_y=0.0, yaw=0.0, water_depth=3.0,
                     y_local=np.zeros(0), intensity_db=np.zeros(0, np.float32),
                     port_analog_gain=142.8, stbd_analog_gain=74.55)
    gp, gs = model.gain_offsets(ping)
    assert gp == pytest.approx(20 * np.log10(142.8 / 74.55)) and gs == 0.0
    unknown = SonarPing(t=0.0, robot_x=0.0, robot_y=0.0, yaw=0.0, water_depth=3.0,
                        y_local=np.zeros(0), intensity_db=np.zeros(0, np.float32))
    assert model.gain_offsets(unknown) == (0.0, 0.0)
    cfg.display.gain_compensation = False
    assert DisplayModel(cfg).gain_offsets(ping) == (0.0, 0.0)


def test_a_gain_step_mid_mission_does_not_move_the_fitted_curve():
    """Half the warm-up rows recorded one gain index up, with the offset
    passed to observe_row: the curve and hi fit the no-step mission."""
    h = 3.0
    g = 20 * np.log10(142.8 / 74.55)
    ref = _fit(AppConfig(), h, seed=5).snapshot()
    cfg = AppConfig()
    model = DisplayModel(cfg)
    rng = np.random.default_rng(5)
    for k in range(200):
        row = make_row(h, rng)
        if k % 2:
            model.observe_row(row + g, PITCH, h, (g, g))
        else:
            model.observe_row(row, PITCH, h)
    model.freeze()
    snap = model.snapshot()
    assert snap.hi_db == pytest.approx(ref.hi_db, abs=0.5)
    np.testing.assert_allclose(snap.a_port, ref.a_port, atol=1.0)
