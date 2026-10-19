"""The one along-track estimator (core/along_track.py).

The waterfall's true-scale row height and the seabed pictures' square
rows both need metres-per-ping. Until 2026-09-15 each derived it from
the raw displacement between consecutive ping poses; measured on the
field logs (project_review.md §3.1) the running median of those steps
was biased -24 % at 20 Hz — consecutive pings often share a pose sample
— and read zero in 26 % of its windows. The estimator pins here:

* repeated poses and centimetre jitter do not bias ground speed × ping
  period;
* a stationary boat reads 0 and the view draws square pixels;
* non-finite poses are ignored;
* the waterfall and the imager, fed the same pings, agree.
"""

from __future__ import annotations

import math
from dataclasses import replace as dc_replace

import numpy as np
import pytest

from blueboat_gcs.core.along_track import AlongTrackEstimator


def feed(est: AlongTrackEstimator, n: int, dt: float, v: float, *,
         pose_hz: float | None = None, jitter: float = 0.0,
         seed: int = 0) -> AlongTrackEstimator:
    """``n`` pings ``dt`` apart at ground speed ``v`` along +x; the pose
    the ping carries is updated at ``pose_hz`` (None = every ping) and
    jittered by ``jitter`` m (1-sigma, both axes)."""
    rng = np.random.default_rng(seed)
    for i in range(n):
        t = i * dt
        tp = t if pose_hz is None else math.floor(t * pose_hz + 1e-9) / pose_hz
        jx, jy = (rng.normal(0.0, jitter, 2) if jitter > 0 else (0.0, 0.0))
        est.observe(t, v * tp + jx, jy)
        est.advance(t)
    return est


def test_constant_speed_reads_exactly_from_the_second_ping():
    est = AlongTrackEstimator()
    est.observe(0.0, 0.0, 0.0)
    assert est.row_pitch_m == 0.0
    est.observe(1.0, 0.5, 0.0)
    assert est.speed_mps == pytest.approx(0.5)
    assert est.ping_period_s == pytest.approx(1.0)
    assert est.row_pitch_m == pytest.approx(0.5)


def test_repeated_poses_at_20hz_do_not_bias_the_pitch():
    """20 Hz pings, pose updated at 10 Hz: every second ping repeats the
    previous pose. The displacement median read half the steps as 0 and
    was biased low; speed × period must stay on the truth."""
    est = feed(AlongTrackEstimator(), 400, 0.05, 0.9, pose_hz=10.0)
    assert est.ping_period_s == pytest.approx(0.05)
    assert est.row_pitch_m == pytest.approx(0.9 * 0.05, rel=0.03)
    assert est.speed_mps == pytest.approx(0.9, rel=0.03)


def test_pose_jitter_of_3cm_stays_within_5pct():
    est = feed(AlongTrackEstimator(), 400, 0.05, 0.9, pose_hz=10.0, jitter=0.03)
    assert est.row_pitch_m == pytest.approx(0.9 * 0.05, rel=0.05)


def test_cumulative_along_track_is_monotone_and_not_inflated_by_jitter():
    est = feed(AlongTrackEstimator(), 400, 0.05, 0.9, pose_hz=10.0, jitter=0.03)
    # 399 intervals of 50 ms at 0.9 m/s (the first ~1 s warms up).
    assert est.along_m == pytest.approx(0.9 * 399 * 0.05, rel=0.06)
    # A summed-displacement integral would carry the rectified jitter:
    # sqrt(2)*0.03 m per ping ≈ 17 m over 400 pings on top of 18 m.
    assert est.along_m < 0.9 * 399 * 0.05 * 1.1


def test_stationary_boat_gives_zero_pitch_and_square_pixels(qapp):
    from blueboat_gcs.gui.waterfall_view import WaterfallView
    est = feed(AlongTrackEstimator(), 100, 0.05, 0.0)
    assert est.row_pitch_m == 0.0 and est.speed_mps == 0.0
    view = WaterfallView()
    view.on_layout(0, 100, 1200, 20.0, est.row_pitch_m)
    assert view.aspect == pytest.approx(1.0)
    view.on_layout(0, 100, 1200, 20.0, float("nan"))    # never saturates
    assert view.aspect == pytest.approx(1.0)
    view.on_layout(0, 100, 1200, 20.0, 0.5)             # 0.5 m rows / 33 mm bins
    assert view.aspect == pytest.approx(0.5 / (20.0 / 600.0))


def test_non_finite_poses_are_ignored():
    a = feed(AlongTrackEstimator(), 100, 0.05, 0.7)
    b = AlongTrackEstimator()
    for i in range(100):
        t = i * 0.05
        if i % 7 == 3:
            b.observe(t, float("nan"), 0.0)     # dropped, not a zero step
        b.observe(t, 0.7 * t, 0.0)
        b.advance(t)
    assert b.row_pitch_m == pytest.approx(a.row_pitch_m, rel=1e-6)
    assert b.along_m == pytest.approx(a.along_m, rel=1e-6)


def test_a_break_keeps_the_history_but_spans_no_chord():
    est = feed(AlongTrackEstimator(), 60, 0.05, 0.7)
    pitch = est.row_pitch_m
    est.mark_break()
    assert est.row_pitch_m == pytest.approx(pitch)     # history kept
    est.observe(100.0, 1000.0, 0.0)                      # far away, later
    est.observe(100.05, 1000.035, 0.0)
    assert est.speed_mps == pytest.approx(0.7, rel=0.05)   # no 1000 m chord
    est.reset()
    assert est.row_pitch_m == 0.0 and est.along_m == 0.0


def test_waterfall_and_imager_agree_on_the_same_pings(qapp, tmp_config):
    from blueboat_gcs.core.seabed_imager import SeabedImager
    from blueboat_gcs.core.waterfall_service import WaterfallService
    from test_waterfall_pick import make_ping
    svc = WaterfallService(tmp_config)
    imager = SeabedImager(tmp_config)
    for i in range(80):
        p = dc_replace(make_ping(i), t=0.05 * i, robot_x=0.045 * i)
        svc.on_sonar_ping(p)
        imager.on_sonar_ping(p)
    assert svc.row_pitch_m == pytest.approx(0.045, rel=1e-6)
    assert imager.along_track.row_pitch_m == pytest.approx(svc.row_pitch_m)
    assert imager.along_track.speed_mps == pytest.approx(0.9, rel=1e-6)


# ---- the window setting (2026-10-16, decision 6.3) --------------------------

def test_the_shipped_window_is_2_s_and_read_from_one_place(qapp, tmp_config):
    """5 s -> 2 s (BlueBoat-Perception PREPROCESSING_STUDY §6.3). The yaml,
    the dataclass default and the estimator default agree, and both
    consumers take the value from the config through ``from_config``."""
    from blueboat_gcs.config.settings import AlongTrackConfig, load_config
    from blueboat_gcs.core import along_track as at
    from blueboat_gcs.core.seabed_imager import SeabedImager
    from blueboat_gcs.core.waterfall_service import WaterfallService
    assert load_config().along_track.window_s == 2.0
    assert AlongTrackConfig().window_s == 2.0 == at.DEFAULT_WINDOW_S
    assert AlongTrackEstimator().window_s == 2.0
    tmp_config.along_track.window_s = 3.5
    assert SeabedImager(tmp_config).along_track.window_s == 3.5
    assert WaterfallService(tmp_config)._along.window_s == 3.5
    tmp_config.along_track.window_s = 0.5        # < pair_s: pair clamps to it
    est = AlongTrackEstimator.from_config(tmp_config)
    assert est.window_s == 0.5 and est._pair_s == 0.5


def test_a_shorter_window_follows_a_speed_change_sooner():
    """What the change buys: after a step from 0.5 to 1.0 m/s the 2 s window
    reads the new speed after 1.9 s (the 1 s chord span plus half the
    window), the 5 s window only after 3.35 s."""
    def settle(window):
        est = AlongTrackEstimator(window_s=window)
        x, t = 0.0, 0.0
        first = None
        for i in range(400):
            t = i * 0.05
            x += (0.5 if t < 10.0 else 1.0) * 0.05
            est.observe(t, x, 0.0)
            if t >= 10.0 and first is None and abs(est.speed_mps - 1.0) < 0.05:
                first = t - 10.0
        return first
    assert settle(2.0) == pytest.approx(1.9, abs=0.06)
    assert settle(5.0) == pytest.approx(3.35, abs=0.06)
