"""Live-path parity: the raw OmniscanProfile cache and the attach step
(core/live_native.py). Fake messages only — no ROS."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from blueboat_gcs.core.live_native import (CachedProfile, PendingRows,
                                           ProcessedFields, ProfileCache,
                                           attach_native, profile_from_msg)
from blueboat_gcs.core.svlog import scale_to_db
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
    samples with the water column deleted (project_side)."""
    pitch = prof.length_mm / 1000.0 / (prof.num_results - 1)
    slant = prof.start_mm / 1000.0 + np.arange(prof.num_results) * pitch
    keep = slant > depth
    return (np.sqrt(slant[keep] ** 2 - depth ** 2),
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
    assert out["bottom_slant_m"] == pytest.approx(DEPTH)
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
    p0, pv, pd = side_bins_from_ground(np.abs(f.port_y), f.port_db, DEPTH)
    assert out["port_bin0"] == p0 and out["port_bin0"] > 0
    np.testing.assert_array_equal(out["port_db"], pv)
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
