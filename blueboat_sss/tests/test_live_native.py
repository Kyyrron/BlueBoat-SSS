"""Live-path parity: the raw OmniscanProfile cache and the attach step
(core/live_native.py). Fake messages only — no ROS."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from blueboat_gcs.core.live_native import (CachedProfile, PendingRows,
                                           ProcessedFields, ProfileCache,
                                           attach_native, profile_from_msg)
from blueboat_gcs.core.svlog import (TRANSDUCER_SUBMERSION_M,
                                     TRANSDUCER_Y_OFFSET_PORT_M, scale_to_db)
from blueboat_gcs.models.sonar import (build_slant_row, native_bins,
                                       native_from_profile,
                                       side_bins_from_ground)
from blueboat_gcs.models.sonar import SonarPing

N = 600
LENGTH_MM = 20000
DEPTH = 3.0


def fake_profile(channel: int, pn: int, *, start_mm: int = 0,
                 seed: int = 0, heading: float = None):
    rng = np.random.default_rng(seed + pn)
    pwr = rng.integers(0, 65535, N, dtype=np.uint16)
    pwr[0] = 0
    pwr[-1] = 65535
    return SimpleNamespace(
        ping_number=pn, start_mm=start_mm, length_mm=LENGTH_MM,
        num_results=N, gain_index=4, channel_number=channel,
        transducer_heading_deg=(-90.0 if channel == 0 else 90.0)
        if heading is None else heading,
        min_pwr_db=-20.0, max_pwr_db=65.0, pwr_results=pwr)


def ground_from(prof: CachedProfile, depth: float):
    """What the processor publishes for one side: slant-corrected ground
    samples with the water column deleted (project_side), from the
    transducer's lever arm and with the altitude = water_depth − submersion
    (994a74b; this helper used lever 0 and altitude = water_depth before
    the 2026-10-16 fallback fix, test_fallback_lever_arm.py). |y| only:
    callers apply the side's sign."""
    pitch = prof.length_mm / 1000.0 / (prof.num_results - 1)
    slant = prof.start_mm / 1000.0 + np.arange(prof.num_results) * pitch
    alt = max(depth - TRANSDUCER_SUBMERSION_M, 0.0)
    keep = slant > alt
    return (TRANSDUCER_Y_OFFSET_PORT_M + np.sqrt(slant[keep] ** 2 - alt ** 2),
            prof.db[keep].astype(np.float32))


def fields(port=None, stbd=None, depth=DEPTH):
    """ProcessedFields for the given cached profiles (None = absent side)."""
    py, pd = ground_from(port.prof, depth) if port else (np.zeros(0), np.zeros(0, np.float32))
    sy, sd = ground_from(stbd.prof, depth) if stbd else (np.zeros(0), np.zeros(0, np.float32))
    return ProcessedFields(port_pn=port.pn if port else 0,
                           stbd_pn=stbd.pn if stbd else 0,
                           water_depth=depth, port_y=py, port_db=pd,
                           stbd_y=sy, stbd_db=sd)


class P:  # a decoded profile + its ping number
    def __init__(self, channel, pn, **kw):
        side, self.pn, self.prof = profile_from_msg(fake_profile(channel, pn, **kw))
        self.side = side


def test_profile_from_msg_scales_and_sides():
    side, pn, prof = profile_from_msg(fake_profile(1, 42))
    assert (side, pn) == (1, 42)
    assert prof.db.dtype == np.float32 and prof.db.size == N
    assert prof.db[0] == pytest.approx(-20.0) and prof.db[-1] == pytest.approx(65.0)
    assert prof.gain_index == 4
    # CM-5: channel 255 falls back to the transducer bearing sign.
    side, _, _ = profile_from_msg(fake_profile(255, 1, heading=-90.0))
    assert side == 0
    side, _, _ = profile_from_msg(fake_profile(255, 1, heading=90.0))
    assert side == 1


def test_cache_bounds_evicts_and_restarts():
    c = ProfileCache(per_side=4, restart_pings=128)
    for pn in range(1, 7):
        c.put(0, pn, P(0, pn).prof)
    assert len(c) == 4 and c.evicted == 2
    assert c.take(0, 1) is None and c.take(0, 6) is not None
    # A counter restart (a key far behind the newest) empties that side.
    c.put(0, 1000, P(0, 1000).prof)
    c.put(0, 3, P(0, 3).prof)
    assert c.restarts == 1 and c.has(0, 3) and not c.has(0, 1000)


def test_attach_hit_is_the_verbatim_profile_bins():
    c = ProfileCache()
    port, stbd = P(0, 10), P(1, 12)
    c.put(0, 10, port.prof)
    c.put(1, 12, stbd.prof)
    out, hits = attach_native(fields(port, stbd), c)
    assert hits == (True, True)
    bin0, pitch = native_from_profile(0, LENGTH_MM, N)
    assert out["port_bin0"] == 0 and out["stbd_bin0"] == 0
    assert out["bin_size_m"] == pytest.approx(pitch)
    assert out["stbd_bin_size_m"] == 0.0                # same pitch both sides
    np.testing.assert_array_equal(out["port_db"], port.prof.db)
    np.testing.assert_array_equal(out["stbd_db"], stbd.prof.db)
    # water_depth = altitude + submersion (994a74b); baseline repair
    # 2026-10-16 -- this expected the submersion to be 0.
    assert out["bottom_slant_m"] == pytest.approx(DEPTH - TRANSDUCER_SUBMERSION_M)
    assert out["gain_index"] == 4
    assert len(c) == 0                                  # consumed


def test_attach_honours_start_mm_and_per_side_pitch():
    c = ProfileCache()
    port = P(0, 5, start_mm=1000)
    stbd = P(1, 5)
    c.put(0, 5, port.prof); c.put(1, 5, stbd.prof)
    out, _ = attach_native(fields(port, stbd), c)
    _, pitch = native_from_profile(0, LENGTH_MM, N)
    assert out["port_bin0"] == round(1.0 / pitch) and out["stbd_bin0"] == 0


def test_attach_miss_falls_back_to_the_reprojection():
    c = ProfileCache()
    port, stbd = P(0, 10), P(1, 12)
    c.put(1, 12, stbd.prof)                             # port profile lost
    f = fields(port, stbd)
    out, hits = attach_native(f, c)
    assert hits == (False, True)
    # The fallback inverts the processor's geometry (lever arm, altitude =
    # water_depth - submersion; fix of 2026-10-16), so the re-projected
    # side lands on the device's own bins: the profile's tail, verbatim.
    p0, pv, pd = side_bins_from_ground(np.abs(f.port_y), f.port_db,
                                       DEPTH - TRANSDUCER_SUBMERSION_M,
                                       TRANSDUCER_Y_OFFSET_PORT_M)
    assert out["port_bin0"] == p0 and out["port_bin0"] > 0
    np.testing.assert_array_equal(out["port_db"], pv)
    np.testing.assert_array_equal(out["port_db"], port.prof.db[p0:])
    np.testing.assert_array_equal(out["stbd_db"], stbd.prof.db)
    assert out["bin_size_m"] == pytest.approx(pd, rel=1e-6)


def test_one_sided_row_attaches_the_present_side_only():
    c = ProfileCache()
    stbd = P(1, 7)
    c.put(1, 7, stbd.prof)
    out, hits = attach_native(fields(None, stbd), c)
    assert hits == (False, True) and out["port_db"] is None
    np.testing.assert_array_equal(out["stbd_db"], stbd.prof.db)
    _, pitch = native_from_profile(0, LENGTH_MM, N)
    assert out["bin_size_m"] == pytest.approx(pitch)


def test_pending_rows_release_in_arrival_order():
    c = ProfileCache()
    q = PendingRows(c, wait_s=0.05)
    a = fields(P(0, 1), P(1, 1))                        # profiles NOT cached
    b_port, b_stbd = P(0, 2), P(1, 2)
    c.put(0, 2, b_port.prof); c.put(1, 2, b_stbd.prof)
    b = fields(b_port, b_stbd)                          # ready at once
    assert q.push(0.000, "A", a) == []                  # waits for profiles
    assert q.push(0.010, "B", b) == []                  # must not overtake A
    assert len(q) == 2
    rel = q.drain(0.030)
    assert rel == []
    rel = q.drain(0.060)                                # A timed out
    assert [(r[0], r[2]) for r in rel] == [("A", True), ("B", False)]
    assert len(q) == 0


def test_live_and_replay_native_rows_are_identical():
    """The parity contract: the live attach step and the replay decoder
    build the same native row from the same profile bytes."""
    c = ProfileCache()
    port, stbd = P(0, 3, seed=5), P(1, 3, seed=6)
    c.put(0, 3, port.prof); c.put(1, 3, stbd.prof)
    out, _ = attach_native(fields(port, stbd), c)
    live = SonarPing(t=0.0, robot_x=0.0, robot_y=0.0, yaw=0.0, water_depth=DEPTH,
                     y_local=np.zeros(0), intensity_db=np.zeros(0, np.float32),
                     **out)
    # Replay-side construction from the same decoded header + pwr.
    bin0, pitch = native_from_profile(0, LENGTH_MM, N)
    replay = SonarPing(t=0.0, robot_x=0.0, robot_y=0.0, yaw=0.0, water_depth=DEPTH,
                       y_local=np.zeros(0), intensity_db=np.zeros(0, np.float32),
                       bin_size_m=pitch, port_bin0=bin0,
                       port_db=scale_to_db(fake_profile(0, 3, seed=5).pwr_results, -20.0, 65.0),
                       stbd_bin0=bin0,
                       stbd_db=scale_to_db(fake_profile(1, 3, seed=6).pwr_results, -20.0, 65.0),
                       bottom_slant_m=DEPTH)
    nl, nr = native_bins(live), native_bins(replay)
    assert nl.extent_bins == nr.extent_bins == N
    np.testing.assert_array_equal(build_slant_row(nl, pitch, N),
                                  build_slant_row(nr, pitch, N))


# ---------------------------------------------------------------------------
# 2026-09-15: analog gain through the live path; the wrapped side rule
# ---------------------------------------------------------------------------
def test_analog_gain_is_cached_and_attached_per_side():
    pm, sm = fake_profile(0, 20), fake_profile(1, 20)
    pm.analog_gain, sm.analog_gain = 142.8, 74.55
    _, _, pp = profile_from_msg(pm)
    _, _, sp = profile_from_msg(sm)
    assert pp.analog_gain == pytest.approx(142.8) and sp.analog_gain == pytest.approx(74.55)
    c = ProfileCache()
    c.put(0, 20, pp)
    c.put(1, 20, sp)
    out, hits = attach_native(fields(P(0, 20), P(1, 20)), c)
    assert hits == (True, True)
    assert out["port_analog_gain"] == pytest.approx(142.8)
    assert out["stbd_analog_gain"] == pytest.approx(74.55)
    # A message without the field (older interface) reads as unknown = 0.
    _, _, bare = profile_from_msg(fake_profile(0, 21))
    assert bare.analog_gain == 0.0


def test_side_rule_wraps_the_bearing_and_reports_unresolved():
    from blueboat_gcs.core.live_native import resolve_side_of_profile as r
    assert r(0, 123.0) == (0, True) and r(1, -123.0) == (1, True)
    assert r(255, -90.0) == (0, True) and r(255, 90.0) == (1, True)
    # An absolute-bearing writer (the simulator until 2026-09-15): 270 is
    # the port transducer of a boat heading north, not starboard.
    assert r(255, 270.0) == (0, True) and r(255, 450.0) == (1, True)
    assert r(255, -270.0) == (1, True)
    # No sign to read: port, flagged.
    assert r(255, 0.0) == (0, False)
    assert r(255, 360.0) == (0, False)
    assert r(255, float("nan")) == (0, False)
    side, _, _ = profile_from_msg(fake_profile(255, 1, heading=270.0))
    assert side == 0


def test_live_nadir_blank_matches_replay():
    """Live applies the same nadir blank the replay decoder does.

    Before 2026-09-19 ``depth.blank_nadir`` / ``nadir_blank_m`` were read
    only by ``core/svlog.py``. The robot's processor passes no blank
    (``sss_helper.py``), so live kept the transmit ringing -- measured at
    55 dB, brighter than any seabed return in the file -- and painted it
    down the track at ground range ~0.
    """
    port, stbd = P(0, 21), P(1, 21)
    c0 = ProfileCache()
    c0.put(0, 21, port.prof)
    c0.put(1, 21, stbd.prof)
    plain, _ = attach_native(fields(port, stbd), c0)
    c1 = ProfileCache()
    c1.put(0, 21, P(0, 21).prof)
    c1.put(1, 21, P(1, 21).prof)
    blanked, _ = attach_native(fields(port, stbd), c1, 0.75)

    pitch = blanked["bin_size_m"]
    n_blank = int(0.75 / pitch)
    assert n_blank > 1
    assert np.all(np.isnan(blanked["port_db"][:n_blank]))
    assert np.all(np.isnan(blanked["stbd_db"][:n_blank]))
    # everything past the blank is the verbatim profile, untouched
    np.testing.assert_array_equal(blanked["port_db"][n_blank:],
                                  plain["port_db"][n_blank:])
    # and 0 (the default) keeps every bin, exactly as before
    assert np.isfinite(plain["port_db"][:n_blank]).all()
