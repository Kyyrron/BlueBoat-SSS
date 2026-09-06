"""The transmit ringing is blanked; the water column is not.

Right under the transducer the profile leaves at **55 dB** — brighter than any
seabed return in the file — and decays through 43 dB at 0.27 m to 33 dB at 1 m.
That bright core is the "very very noise" beneath the boat: a transmit-ringing
artefact that used to sit in the middle of every ``off`` image and splat onto
the track line in the mosaic. It is what this blank removes.

Everything past it is **not** an artefact. The water column settles to 16–28 dB,
which is *darker* than the seabed (25.6 dB at 20 m on the same log) and is
honest data. An earlier attempt blanked out to the tracked altitude — 9.4 m on
``diffDepthCompensation.svlog`` — and punched a hole straight through the
waterfall, the mosaic and the AI tiles. **The pictures have to be continuous**,
so the blank is narrow by construction and that is the invariant this file
guards hardest.

What this file pins:

* the blank removes the ringing and nothing wider — the image stays continuous;
* the blank moves no surviving sample;
* ``auto`` is bit-for-bit unchanged — its correction altitude already cut
  everything inside the water column, ringing included;
* the two ``project_side`` implementations (robot, GCS) stay interchangeable;
* the FBR tracker advances in *every* mode, which it did not before;
* the blank can never empty a ping, because an all-NaN row means "session gap"
  downstream;
* the blank applies to the **ground/mosaic projection only** — the raw-slant
  waterfall and the AI seabed pictures now carry every sample (the water
  column included) and darken the nadir with the display colour window
  instead of erasing it, so the pictures stay fully continuous and
  losslessly invertible back to dB.

No ROS, no corpus, no display: everything here is synthetic and runs anywhere.
"""

from __future__ import annotations

import sys

import numpy as np
import pytest

from conftest import PKG_PARENT

from blueboat_gcs.config.settings import AppConfig
from blueboat_gcs.core.seabed_imager import SeabedImager, feed_pings
from blueboat_gcs.core.svlog import (FBRTracker, clamp_nadir_mask, load_svlog,
                                     project_side, resolve_altitude)
from blueboat_gcs.core.waterfall_service import WaterfallService
from blueboat_gcs.models.sonar import SonarPing

from test_svlog_forensics import LENGTH_MM, NUM_RESULTS
from test_svlog_replay import build_log

HELPERS = PKG_PARENT / "src" / "_custom_libraries"
if str(HELPERS) not in sys.path:
    sys.path.insert(0, str(HELPERS))

import sss_helper                                            # noqa: E402

#: ``profile_packet``'s synthetic bottom return sits at sample 200 of 600 over
#: 20 m, i.e. a 6.67 m altitude. The FBR tracker finds it a sample or two late,
#: so tests compare against what the loader reports rather than this constant.
NOMINAL_ALT_M = 200 / (NUM_RESULTS - 1) * (LENGTH_MM / 1000.0)

RANGE_M = LENGTH_MM / 1000.0

#: The shipped default blank, and what ``load_svlog`` uses unless told otherwise.
BLANK_M = 0.75


def profile_db(bottom_sample: int = 200) -> np.ndarray:
    """A quiet water column, a hard bottom return, then seabed."""
    db = np.full(NUM_RESULTS, -90.0, np.float32)
    db[bottom_sample:bottom_sample + 30] = -20.0
    db[bottom_sample + 30:] = -50.0
    return db


def slant_of(index: int) -> float:
    return index / (NUM_RESULTS - 1) * RANGE_M


# ---------------------------------------------------------------------------
# 1. The mask itself
# ---------------------------------------------------------------------------
def test_mask_removes_the_water_column_and_moves_nothing_else():
    """With no correction, the mask cuts |y| < altitude and leaves the rest."""
    db = profile_db()
    alt = 6.0
    y, v = project_side(db, 0, LENGTH_MM, NUM_RESULTS, 0.0, 0.0, +1.0, alt)

    assert y.size < NUM_RESULTS
    assert y.min() > alt                      # nothing inside the band survives
    # Correction is off, so a surviving sample keeps its slant range exactly.
    slant = np.arange(NUM_RESULTS) / (NUM_RESULTS - 1) * RANGE_M
    keep = slant > alt
    assert y == pytest.approx(slant[keep])
    assert v == pytest.approx(db[keep])


def test_mask_is_a_pure_addition_to_the_old_cut():
    """Masking only ever removes samples; it never relocates a survivor."""
    db = profile_db()
    y_none, v_none = project_side(db, 0, LENGTH_MM, NUM_RESULTS, 0.0,
                                  0.0, +1.0, None)
    y_mask, v_mask = project_side(db, 0, LENGTH_MM, NUM_RESULTS, 0.0,
                                  0.0, +1.0, 6.0)
    assert y_mask.size < y_none.size
    tail = y_none.size - y_mask.size
    assert y_mask == pytest.approx(y_none[tail:])
    assert v_mask == pytest.approx(v_none[tail:])


def test_auto_mode_is_bit_for_bit_unchanged():
    """The mask equals the correction altitude in ``auto``, so it is inert.

    ``auto`` already cut the water column at exactly the altitude. This fix
    must not perturb it — only the modes whose correction altitude is 0.
    """
    db = profile_db()
    alt = 6.0
    fused = project_side(db, 0, LENGTH_MM, NUM_RESULTS, alt, 0.0, +1.0, None)
    split = project_side(db, 0, LENGTH_MM, NUM_RESULTS, alt, 0.0, +1.0, alt)
    assert np.array_equal(fused[0], split[0])
    assert np.array_equal(fused[1], split[1])


@pytest.mark.parametrize("sign", (+1.0, -1.0))
@pytest.mark.parametrize("alt,mask", [(0.0, 6.0), (6.0, 6.0), (2.0, 6.0),
                                      (6.0, 2.0), (0.0, None)])
def test_robot_and_gcs_project_side_agree(alt, mask, sign):
    """The two implementations are treated as one function; keep them so."""
    db = profile_db()
    y_np, v_np = project_side(db, 0, LENGTH_MM, NUM_RESULTS, alt, 0.0,
                              sign, mask)
    y_py, v_py = sss_helper.project_side(
        [float(x) for x in db], 0, LENGTH_MM, NUM_RESULTS, alt, 0.0, sign,
        mask)
    assert y_np == pytest.approx(np.asarray(y_py))
    assert v_np == pytest.approx(np.asarray(v_py, dtype=np.float32))


def test_a_mask_below_the_correction_altitude_cannot_widen_the_swath():
    """``max(correction, mask)`` keeps sqrt() out of the negative domain."""
    db = profile_db()
    y, _ = project_side(db, 0, LENGTH_MM, NUM_RESULTS, 6.0, 0.0, +1.0, 2.0)
    y_ref, _ = project_side(db, 0, LENGTH_MM, NUM_RESULTS, 6.0, 0.0, +1.0,
                            None)
    assert np.array_equal(y, y_ref)
    assert np.isfinite(y).all()


# ---------------------------------------------------------------------------
# 2. The policy
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("mode", ("auto", "manual", "off"))
def test_the_tracker_advances_in_every_mode(mode):
    """It used to early-return for ``off``/``manual``, so it never bootstrapped.

    Without this the ``off`` mode has no altitude to mask with, which is the
    whole point of the fix.
    """
    tracker = FBRTracker()
    for _ in range(20):
        resolve_altitude(tracker, 6.0, 6.0, mode, 3.0)
    assert tracker.locked
    assert tracker._altitude == pytest.approx(6.0)


def test_the_policy_table():
    """``mode`` governs the correction altitude, and only that."""
    def run(mode, manual=3.0):
        tracker = FBRTracker()
        for _ in range(20):
            out = resolve_altitude(tracker, 6.0, 6.0, mode, manual)
        return out

    assert run("auto") == (6.0, True)
    assert run("manual") == (3.0, True)
    assert run("off") == (0.0, True)
    # The nadir blank is NOT part of this: it is a fixed distance applied in
    # every mode, so nothing here varies with it.


def test_the_correction_is_never_a_guess():
    """Nothing detected means no correction — an identity transform.

    (``off`` reports locked because its altitude is a deliberate 0, not a
    failed estimate; that predates this change.)
    """
    assert resolve_altitude(FBRTracker(), None, None, "off") == (0.0, True)
    assert resolve_altitude(FBRTracker(), None, None, "auto") == (0.0, False)


# ---------------------------------------------------------------------------
# 3. The clamp — a blank can never empty a ping
# ---------------------------------------------------------------------------
def test_clamp_caps_the_blank_at_a_share_of_the_slant_extent():
    assert clamp_nadir_mask(45.0, 0, LENGTH_MM, 0.5) == pytest.approx(10.0)
    assert clamp_nadir_mask(BLANK_M, 0, LENGTH_MM, 0.5) == pytest.approx(BLANK_M)
    assert clamp_nadir_mask(None, 0, LENGTH_MM, 0.5) is None


def test_a_mis_set_blank_still_leaves_half_the_side():
    db = profile_db()
    mask = clamp_nadir_mask(45.0, 0, LENGTH_MM, 0.5)
    y, _ = project_side(db, 0, LENGTH_MM, NUM_RESULTS, 0.0, 0.0, +1.0, mask)
    assert y.size >= NUM_RESULTS // 2 > 0


def test_the_clamp_never_shortens_the_correction_cut():
    """In ``auto`` the correction is in charge and the clamp is inert."""
    db = profile_db()
    y_clamped, _ = project_side(db, 0, LENGTH_MM, NUM_RESULTS, 15.0, 0.0, +1.0,
                                clamp_nadir_mask(15.0, 0, LENGTH_MM, 0.5))
    y_plain, _ = project_side(db, 0, LENGTH_MM, NUM_RESULTS, 15.0, 0.0, +1.0,
                              None)
    assert np.array_equal(y_clamped, y_plain)


# ---------------------------------------------------------------------------
# 4. End to end — the blank is narrow and the pictures stay continuous
# ---------------------------------------------------------------------------
@pytest.fixture
def log(tmp_path):
    return build_log(tmp_path / "nadir.svlog", 30)


def _pings(mission):
    return [e[2] for e in mission.events if e[0] == "ping"]


def test_off_mode_blanks_the_ringing_and_keeps_the_water_column(log):
    """The regression this whole file exists for, in both directions."""
    plain = _pings(load_svlog(log, depth_mode="off", blank_nadir=False))
    blank = _pings(load_svlog(log, depth_mode="off"))
    assert len(plain) == len(blank) > 0            # NC #2: no ping is dropped

    inner_plain = min(float(np.abs(p.y_local).min()) for p in plain)
    inner_blank = min(float(np.abs(p.y_local).min()) for p in blank)
    assert inner_plain < 0.1                       # ringing, right under the boat
    assert inner_blank >= BLANK_M                  # ...removed

    # ...and NOT one metre more than that. This is the guard against
    # re-widening the blank to the altitude and holing every image.
    assert inner_blank < 2 * BLANK_M
    kept = np.mean([b.y_local.size / p.y_local.size
                    for p, b in zip(plain, blank)])
    assert kept > 0.9, f"blank swallowed {100 * (1 - kept):.0f} % of the swath"

    # Correction is still off: the outer swath is untouched slant range.
    assert (max(float(np.abs(p.y_local).max()) for p in blank)
            == pytest.approx(RANGE_M))


def test_the_water_column_is_still_there(log):
    """It is darker than the seabed, it is real, and it keeps the image whole."""
    blank = _pings(load_svlog(log, depth_mode="off"))
    # `water_depth` reports the CORRECTION altitude, which is 0 in `off`;
    # the tracked bottom is what `auto` resolves on the same file.
    alt = max(p.water_depth for p in _pings(load_svlog(log, depth_mode="auto")))
    assert alt > 2 * BLANK_M, "fixture too shallow to make this meaningful"
    # Samples strictly inside the water column, past the blanked ringing.
    inside = sum(int(((np.abs(p.y_local) > BLANK_M)
                      & (np.abs(p.y_local) < alt)).sum()) for p in blank)
    assert inside > 0


def test_auto_mode_output_is_unchanged_by_the_blank(log):
    """``auto`` already cut everything inside the water column, ringing too."""
    with_blank = _pings(load_svlog(log, depth_mode="auto"))
    without = _pings(load_svlog(log, depth_mode="auto", blank_nadir=False))
    assert len(with_blank) == len(without) > 0
    for a, b in zip(with_blank, without):
        assert np.array_equal(a.y_local, b.y_local)
        assert np.array_equal(a.intensity_db, b.intensity_db)


def test_no_ping_is_emptied_by_the_blank(log):
    """An all-NaN waterfall row means "session gap" downstream."""
    for mode in ("auto", "manual", "off"):
        for p in _pings(load_svlog(log, depth_mode=mode, manual_depth_m=9.0)):
            assert p.y_local.size > 0


def _waterfall(pings):
    service = WaterfallService(AppConfig())
    for p in pings:
        service.on_sonar_ping(p)
    return service.chronological()


def test_the_waterfall_keeps_every_sample(log, tmp_config):
    """No erasing: the raw-slant waterfall carries the full water column.

    The ringing blank now applies to the ground/mosaic projection only,
    so the native waterfall is identical with and without it, and the
    nadir is darkened by the display colour window instead of removed.
    """
    a = _waterfall(_pings(load_svlog(log, depth_mode="off",
                                     blank_nadir=False)))
    b = _waterfall(_pings(load_svlog(log, depth_mode="off")))
    assert a is not None and b is not None
    assert a.shape == b.shape
    # The native waterfall no longer depends on the blank: identical.
    np.testing.assert_array_equal(np.isfinite(a), np.isfinite(b))
    # Continuity: the two-sided synthetic profile leaves NO hole anywhere,
    # water column included — nothing is erased.
    assert np.isfinite(b).all()

    cols = b.shape[1]
    mid = (cols - 1) // 2
    half = int(BLANK_M / RANGE_M * (cols - 1) / 2)
    assert half >= 5
    assert np.isfinite(b[:, mid - half + 1:mid + half]).all()

    # The display model darkens the water column toward black without
    # removing it, and keeps the bottom return bright (SonarView parity).
    from blueboat_gcs.core.display_model import DisplayModel
    pings = _pings(load_svlog(log, depth_mode="off"))
    model = DisplayModel.fit(AppConfig(), pings)
    assert model.ready
    hs = np.array([p.bottom_slant_m for p in pings])
    u = model.render_unit(b, pings[0].bin_size_m, hs)
    assert np.isfinite(u).all()             # nothing erased, nothing NaN
    near_nadir = u[:, mid - half + 1:mid + half]
    assert near_nadir.max() < 0.2           # ringing / water column -> dark
    assert u.max() > 0.9                    # the bottom return -> bright
    # The water column is darker than the bottom return everywhere.
    assert near_nadir.mean() < 0.1 * u.max()


def _falloff_ping(i: int, rng: np.random.Generator) -> SonarPing:
    """A ping with a dark water column, then seabed carrying a strong
    pre-TVG range falloff (bright near, dim far) plus texture noise —
    native bins with the water column present (bin0 = 0), the replay-path
    shape. This is the input that produced the far-range-crushed render."""
    n, pitch, alt = 300, 0.05, 3.0
    slant = (np.arange(n) + 0.5) * pitch          # 0 .. 15 m
    db = np.full(n, -90.0, np.float32)            # water column: dark
    sb = slant >= alt
    # ~30 dB near the nadir falling ~1.6 dB/m to the far range: the far
    # seabed is ~18 dB below the near seabed in the RAW dB.
    base = 30.0 - 1.6 * slant[sb]
    db[sb] = (base + rng.normal(0.0, 2.5, int(sb.sum()))).astype(np.float32)
    return SonarPing(t=float(i), robot_x=float(i), robot_y=0.0, yaw=0.0,
                     water_depth=0.0, y_local=np.zeros(0),
                     intensity_db=np.zeros(0, np.float32),
                     bin_size_m=pitch, port_bin0=0, port_db=db,
                     stbd_bin0=0, stbd_db=db, bottom_slant_m=alt)


def test_far_range_seabed_is_not_crushed_to_black(qapp, tmp_config):
    """Regression for the over-contrast render (our_waterfall_toomuch_
    contrast.png): the ~18 dB range falloff must NOT black out the far
    seabed. Seabed-referenced EGN flattens near and far to one level, so
    BOTH halves of the swath render well above black while the nadir maps
    toward black — the SonarView-parity contract.
    """
    rng = np.random.default_rng(0)
    svc = WaterfallService(tmp_config)            # nadir_contrast on
    for i in range(60):
        svc.on_sonar_ping(_falloff_ping(i, rng))
    svc.model.freeze()
    chrono = svc.chronological()
    unit = svc.render_unit(chrono, svc._meta_chrono()[:, 5])
    gray = np.nan_to_num(unit, nan=0.0) * 255.0
    finite = np.isfinite(unit)                     # real samples only

    cols = gray.shape[1]
    half = cols // 2
    q = half // 3
    # Near-range seabed = the columns just OUTSIDE the nadir band on each
    # side; far-range seabed = the outer third on each side.
    near = np.concatenate([gray[:, half - q:half][finite[:, half - q:half]],
                           gray[:, half:half + q][finite[:, half:half + q]]])
    far = np.concatenate([gray[:, :q][finite[:, :q]],
                          gray[:, -q:][finite[:, -q:]]])
    assert near.size and far.size
    # The regression: far seabed must be nearly as bright as near seabed,
    # and clearly not blacked out.
    assert far.mean() > 60.0, f"far seabed crushed: mean {far.mean():.1f}"
    assert np.mean(far < 8) < 0.10, f"far seabed too black: {np.mean(far < 8):.2f}"
    assert far.mean() > 0.6 * near.mean(), (far.mean(), near.mean())

    # The nadir water-column band (centre columns, slant < alt) maps dark.
    nadir = gray[:, half - 3:half + 3]
    nfin = finite[:, half - 3:half + 3]
    if nfin.any():
        assert gray[:, half - 3:half + 3][nfin].mean() < 40.0


def test_the_seabed_tile_keeps_its_coverage(log, tmp_config):
    """CM-10 tiles must be continuous: nothing erased, nadir darkened."""
    def tiles(**kw):
        config = AppConfig()
        config.seabed.rows = 16
        config.seabed.stride = 8
        imager = SeabedImager(config)
        out = []
        imager.image_ready.connect(out.append)
        feed_pings(imager, _pings(load_svlog(log, depth_mode="off", **kw)))
        return out

    plain, blank = tiles(blank_nadir=False), tiles()
    assert plain and blank and len(plain) == len(blank)

    a, b = plain[0].intensity_db, blank[0].intensity_db
    assert a.shape == b.shape
    # Native tile ignores the blank now, and carries every sample.
    np.testing.assert_array_equal(np.isfinite(a), np.isfinite(b))
    assert np.isfinite(b).all()

    # The tile carries the invertible display model and renders the nadir
    # darker than the seabed — the darkening is colour, not erasure.
    assert blank[0].display is not None and blank[0].display.ready
    assert blank[0].metadata()["display_model"]["hi_db"] == pytest.approx(
        blank[0].display.hi_db)
    png = blank[0].to_png8()
    cols = b.shape[1]
    mid = (cols - 1) // 2
    half = int(BLANK_M / RANGE_M * (cols - 1) / 2)
    centre = png[:, mid - half + 1:mid + half]
    assert centre.mean() < 10              # nadir maps to near-black
    assert png.max() > 200                 # the seabed/bottom return stays bright
