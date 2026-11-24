"""core/stream_clock.py: the GCS's world-timing clock (2026-11-07).

Wall monotonic in a real-robot graph (never fed), sim-paced under /clock,
wall-paced again when /clock stalls, never backwards. ROS-free, Qt-free.
"""

from __future__ import annotations

import time

import pytest

from blueboat_gcs.core.stream_clock import StreamClock


def test_unfed_clock_is_the_monotonic_clock():
    c = StreamClock()
    before = time.monotonic()
    v = c.now()
    assert before <= v <= time.monotonic()
    assert not c.sim_paced and c.ratio is None
    assert c.now(mono_s=123.0) == 123.0


def test_fed_clock_advances_in_sim_seconds_at_a_slowed_rate():
    """x0.1: 1 s of sim time per 10 s of wall time, /clock every 1 s wall."""
    c = StreamClock(stall_s=3.0)
    c.feed(100.0, mono_s=1000.0)
    assert c.now(mono_s=1000.0) == pytest.approx(1000.0)   # continuous at the switch
    for k in range(1, 21):
        c.feed(100.0 + 0.1 * k, mono_s=1000.0 + k)
    # 20 wall seconds later: 2 sim seconds elapsed on the clock.
    assert c.now(mono_s=1020.0) == pytest.approx(1002.0, abs=1e-6)
    assert c.ratio == pytest.approx(0.1, rel=0.05)
    # Between samples it interpolates at the measured ratio (never a step).
    assert c.now(mono_s=1020.5) == pytest.approx(1002.05, abs=0.01)


def test_real_speed_sim_tracks_wall_time():
    c = StreamClock()
    for k in range(200):
        c.feed(10.0 + 0.01 * k, mono_s=50.0 + 0.01 * k)
    assert c.now(mono_s=51.99) == pytest.approx(51.99, abs=1e-6)


def test_stalled_clock_falls_back_to_wall_pace_so_staleness_shows():
    c = StreamClock(stall_s=3.0)
    c.feed(0.0, mono_s=0.0)
    c.feed(1.0, mono_s=1.0)              # ratio ~ 1
    v_stall = c.now(mono_s=4.0)          # 3 s of interpolation
    later = c.now(mono_s=14.0)           # then 10 s at wall pace
    assert later - v_stall == pytest.approx(10.0, abs=1e-6)


def test_never_goes_backwards_on_overshoot_or_sim_restart():
    c = StreamClock()
    c.feed(0.0, mono_s=0.0)
    for k in range(1, 11):
        c.feed(0.1 * k, mono_s=float(k))         # ratio 0.1
    v1 = c.now(mono_s=12.5)                       # interpolated ahead
    c.feed(1.0, mono_s=12.6)                      # sim did not move: hold
    v2 = c.now(mono_s=12.6)
    assert v2 >= v1
    c.feed(0.0, mono_s=13.0)                      # Gazebo restarted
    v3 = c.now(mono_s=13.0)
    c.feed(0.5, mono_s=14.0)
    v4 = c.now(mono_s=14.0)
    assert v3 >= v2 and v4 == pytest.approx(v3 + 0.5, abs=1e-6)


def test_reset_returns_to_wall_time():
    c = StreamClock()
    c.feed(5.0, mono_s=1.0)
    c.reset()
    assert not c.sim_paced and c.now(mono_s=7.0) == 7.0
