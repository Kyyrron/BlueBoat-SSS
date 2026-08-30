"""`.svlog` replay fidelity: the log's own clock, its sessions, and id 2194.

Three properties, one loader. ``core/svlog.py`` used to advance a flat 20 ms
per packet and dispatch only two packet ids, so:

* a 611.5 s mission replayed as 251.6 s and x1 was not real time;
* a file holding two recording sessions was concatenated into one, stacking
  waterfall rows 397.8 s apart as neighbours and tiling seabed images across
  the join;
* unrecognised packet ids vanished silently.

Split the same way as ``test_svlog_forensics.py``: **synthetic** files built in
``tmp_path`` cover the properties that must hold on any input and run
everywhere; **field** cases pin the numbers against the real corpus and skip
cleanly when it is not mounted. Field files are opened read-only — they are
primary field data (CLAUDE.md NC #6 / root CM-7).

Where a field number is asserted here it is cross-checked against
``analysis/svlog_forensics.analyse``, which derives it from the same bytes by a
wholly separate path. Two implementations agreeing is the evidence; one
implementation agreeing with a number typed into a test is not.
"""

from __future__ import annotations

import struct
import sys

import numpy as np
import pytest

from conftest import PKG_PARENT

from blueboat_gcs.analysis.svlog_forensics import analyse
from blueboat_gcs.core.svlog import (NS_PER_TICK, STAMP_BACKSTEP_TOLERANCE_MS,
                                     decode_omniscan_status,
                                     decode_session_header, estimate_boot_skew,
                                     load_svlog, usable_stamps, walk_packets)

from test_svlog_forensics import (LENGTH_MM, profile_packet, session_packet,
                                  write_log)
from test_processor_assembly import FIELD_ROOT, field_log

HELPERS = PKG_PARENT / "src" / "_custom_libraries"
if str(HELPERS) not in sys.path:
    sys.path.insert(0, str(HELPERS))

import svlog_helper                                          # noqa: E402

CERULEAN_DEMO = "harbor_scan_combined.svlog"
SONARVIEW_20M = "ShiraishiJima/diffDepthCompensation.svlog"
TIRE_25M = "ShiraishiJima/TireExamples/Tire4-25m.svlog"
#: Every mavlink packet in this log carries one frozen ``time_boot_ms``.
FROZEN_CLOCK = ("ShiraishiJima/No_sonarVNotOK_usOK_SimpleCurve/"
                "2026-07-21-15-50-26.svlog")

SINGLE_SESSION = [TIRE_25M, SONARVIEW_20M,
                  "ShiraishiJima/toOptimize-harbourScan.svlog", FROZEN_CLOCK]


# ---------------------------------------------------------------------------
# Synthetic construction
# ---------------------------------------------------------------------------
def mavlink_packet(msg: dict) -> bytes:
    import json
    payload = json.dumps({"message": msg}).encode("utf-8")
    return svlog_helper.frame_packet(svlog_helper.MAVLINK_WRAPPER_ID, payload,
                                     src=3, dst=0)


def pose_burst(boot_ms: int, x: float, y: float) -> list:
    """The ATTITUDE + LOCAL_POSITION_NED pair the loader synthesizes odom from."""
    return [mavlink_packet({"type": "ATTITUDE", "time_boot_ms": boot_ms,
                            "yaw": 0.0}),
            mavlink_packet({"type": "LOCAL_POSITION_NED",
                            "time_boot_ms": boot_ms, "x": x, "y": y, "z": 0.0,
                            "vx": 1.0, "vy": 0.0, "vz": 0.0})]


def status_packet(channel: int, timestamp_ms: int,
                  v0: float = 69.5, v1: float = 52.0) -> bytes:
    """One framed id-2194 Omniscan status packet."""
    payload = struct.pack("<ffI3xB", v0, v1, timestamp_ms, channel)
    src = (svlog_helper.DEVICE_ID_PORT if channel == 0
           else svlog_helper.DEVICE_ID_STBD)
    return svlog_helper.frame_packet(2194, payload, src=src, dst=0)


def build_log(path, pings, *, skew_ms=1000, sessions=(0,), channels=(0, 1),
              stamp=lambda ch, n: n * 50):
    """A loadable synthetic log: poses, then ``pings`` ping-pairs per session.

    ``sessions`` gives the first ping index of each session. The sonar clock is
    ``stamp(channel, n)``; mavlink counts from its own boot, ``skew_ms`` behind,
    which is what ``estimate_boot_skew`` has to recover.
    """
    packets = []
    for n in range(pings):
        if n in sessions:
            packets.append(session_packet())
            # A pose must precede the first ping of the file or the loader
            # drops it as unplaceable, which is correct and not what these
            # tests are about.
            if n == 0:
                packets.extend(pose_burst(stamp(0, 0) - skew_ms, 0.0, 0.0))
        for ch in channels:
            packets.append(profile_packet(ch, n, stamp(ch, n)))
        # After the profiles, so each skew vote anchors to its own ping: the
        # estimator votes against the most recent profile, and interleaving the
        # other way biases it by the ping interval.
        packets.extend(pose_burst(stamp(0, n) - skew_ms, float(n), 0.0))
    return write_log(path, packets)


def ping_span(mission) -> float:
    """First to last ping. ``duration_s`` also spans poses, which bracket them."""
    t = [e[1] for e in mission.events if e[0] == "ping"]
    return max(t) - min(t)


# ---------------------------------------------------------------------------
# Step 1 — the clock
# ---------------------------------------------------------------------------
def test_timeline_uses_the_logs_own_timestamps(tmp_path):
    """20 pings at 500 ms is a 9.5 s mission, not 20 tick steps."""
    path = build_log(tmp_path / "real.svlog", 20,
                     stamp=lambda ch, n: 10_000 + n * 500)
    m = load_svlog(path)
    assert m.ping_count == 20
    assert not m.synthetic_clock
    assert ping_span(m) == pytest.approx(9.5, abs=0.05)
    # The old flat tick advanced once per packet, so 20 ping pairs plus their
    # pose bursts spanned 100 steps of 20 ms; the real clock is 9.5 s.
    assert ping_span(m) > 100 * NS_PER_TICK / 1e9


def test_boot_skew_is_recovered_so_poses_land_on_the_sonar_clock(tmp_path):
    """Sonar and autopilot count from their own boots; the offset is measured."""
    path = build_log(tmp_path / "skew.svlog", 30, skew_ms=1_234_567,
                     stamp=lambda ch, n: 900_000 + n * 100)
    m = load_svlog(path)
    assert m.boot_skew_ms == pytest.approx(1_234_567, abs=200)
    pings = [e[1] for e in m.events if e[0] == "ping"]
    states = [e[1] for e in m.events if e[0] == "state"]
    assert states, "poses were dropped"
    # Without the skew the two streams would sit ~1234 s apart.
    assert min(states) >= min(pings) - 1.0
    assert max(states) <= max(pings) + 1.0


def test_estimate_boot_skew_is_the_median_and_reports_nothing_to_measure():
    assert estimate_boot_skew([100, 102, 98, 5000]) == 101
    assert estimate_boot_skew([]) is None


def test_channel_batched_writer_does_not_trip_the_fallback(tmp_path):
    """File order is non-monotonic on real logs; per-channel order is not.

    The writer batches by channel — 145, 125 and 1076 file-order inversions
    measured across the corpus — so a file-order monotonicity test would reject
    the real clock almost everywhere. This is that shape, and it must load on
    real time.
    """
    packets = [session_packet()]
    packets.extend(pose_burst(0, 0.0, 0.0))
    for block in range(4):                     # 10 port pings, then 10 stbd
        for ch in (0, 1):
            for n in range(block * 10, block * 10 + 10):
                packets.append(profile_packet(ch, n, 1000 + n * 50))
    m = load_svlog(write_log(tmp_path / "batched.svlog", packets))
    assert not m.synthetic_clock
    assert m.ping_count == 40
    assert ping_span(m) == pytest.approx(39 * 0.05, abs=0.01)


def test_a_single_ping_swap_does_not_trip_the_fallback(tmp_path):
    """One corpus log swaps 9 adjacent pings out of ~17 000, each by one PRI.

    That is the writer emitting a pair out of order, fully absorbed by the
    final event sort. Rejecting a whole log's real timeline over it would be a
    strictly worse result than keeping it.
    """
    def stamp(ch, n):
        base = 1000 + n * 50
        return base - 60 if n == 17 else base       # one PRI backwards
    m = load_svlog(build_log(tmp_path / "swap.svlog", 40, stamp=stamp))
    assert not m.synthetic_clock
    assert m.ping_count == 40
    t = [e[1] for e in m.events]
    assert t == sorted(t), "events must still be emitted in time order"


def test_zeroed_timestamps_fall_back_to_the_tick(tmp_path):
    # Single-channel: with every stamp identical there is no "temporally
    # nearest opposite ping", so estimate_counter_offset has nothing to vote
    # on and a two-sided fixture would be testing that instead of the clock.
    m = load_svlog(build_log(tmp_path / "zero.svlog", 20, channels=(0,),
                             stamp=lambda ch, n: 0))
    assert m.synthetic_clock
    assert m.clock_notes and "timestamp_ms" in m.clock_notes[0]
    assert m.ping_count == 20
    t = [e[1] for e in m.events]
    assert t == sorted(t) and t[0] >= 0.0


def test_a_clock_reset_falls_back_to_the_tick(tmp_path):
    """A backwards jump far beyond one ping interval is a reset, not a swap."""
    jump = STAMP_BACKSTEP_TOLERANCE_MS * 5

    def stamp(ch, n):
        return 100_000 + n * 50 - (jump if n >= 25 else 0)
    m = load_svlog(build_log(tmp_path / "reset.svlog", 40, stamp=stamp))
    assert m.synthetic_clock
    assert m.ping_count == 40
    t = [e[1] for e in m.events]
    assert t == sorted(t)


def test_usable_stamps_boundary_is_the_documented_tolerance():
    assert usable_stamps([1, 2, 3])
    assert not usable_stamps([1, 0, 3])            # a zero is never a clock
    assert not usable_stamps([])
    base = [10_000, 20_000, 30_000]
    assert usable_stamps(base + [30_000 - STAMP_BACKSTEP_TOLERANCE_MS])
    assert not usable_stamps(base + [30_000 - STAMP_BACKSTEP_TOLERANCE_MS - 1])


@field_log
@pytest.mark.parametrize("rel", [CERULEAN_DEMO, SONARVIEW_20M, TIRE_25M,
                                 FROZEN_CLOCK])
def test_field_timeline_matches_the_forensics_span(rel):
    """The loader's duration and the forensics span are two paths, one answer.

    Forensics derives its span from ``timestamp_ms`` sorted per channel per
    segment and never touches ``load_svlog``; the loader builds a timeline and
    re-bases it. They agree only if the loader is on the log's real clock.
    """
    m = load_svlog(FIELD_ROOT / rel)
    f = analyse(FIELD_ROOT / rel, render=False)
    assert not m.synthetic_clock
    # The loader spans emitted pings, forensics spans all profiles, so a log
    # with pose-gated drops is legitimately a little shorter.
    assert m.duration_s <= f.span_s + 0.5
    assert m.duration_s >= f.span_s - 3.0
    assert m.acquisition_s == pytest.approx(f.acquisition_s, abs=3.0)

    t = np.array([e[1] for e in m.events if e[0] == "ping"])
    dt = np.diff(t)
    assert np.median(dt[dt > 0]) * 1000.0 == pytest.approx(
        f.pri_median_ms, abs=1.0), "ping cadence must be the log's own PRI"


@field_log
def test_the_cerulean_demo_is_no_longer_reported_as_half_its_length():
    """The headline symptom: 611.5 s of mission read as 251.6 s."""
    m = load_svlog(FIELD_ROOT / CERULEAN_DEMO)
    assert m.duration_s == pytest.approx(611.5, abs=5.0)
    assert m.acquisition_s == pytest.approx(213.8, abs=1.0)


@field_log
def test_frozen_autopilot_clock_keeps_the_sonar_timeline():
    """One log's mavlink carries a single frozen ``time_boot_ms`` throughout.

    Anchoring poses to it would collapse every RobotState onto one instant, and
    discarding the whole timeline over it would throw away a good sonar clock.
    Poses ride the sonar clock instead, and the loader says so.
    """
    m = load_svlog(FIELD_ROOT / FROZEN_CLOCK)
    assert not m.synthetic_clock
    assert m.boot_skew_ms is None
    assert any("frozen" in n for n in m.clock_notes)
    states = [e[1] for e in m.events if e[0] == "state"]
    assert len(set(np.round(states, 3))) > 100, "poses collapsed onto one stamp"


# ---------------------------------------------------------------------------
# Step 2 — sessions
# ---------------------------------------------------------------------------
def test_a_second_session_header_opens_a_segment(tmp_path):
    path = build_log(tmp_path / "two.svlog", 40, sessions=(0, 20),
                     stamp=lambda ch, n: (1000 + n * 50
                                          + (60_000 if n >= 20 else 0)))
    m = load_svlog(path)
    assert [s.index for s in m.segments] == [0, 1]
    assert [s.ping_count for s in m.segments] == [20, 20]
    assert len(m.gaps) == 1
    assert m.gaps[0].seconds == pytest.approx(60.0, abs=0.2)
    assert m.gaps[0].from_segment == 0 and m.gaps[0].to_segment == 1
    assert m.gap_times == [m.gaps[0].t]
    # The gap event must sit after the last ping of the segment it closes.
    kinds = [k for k, _t, _o in m.events if k in ("ping", "gap")]
    assert kinds.count("gap") == 1
    assert m.events[[k for k, _t, _o in m.events].index("gap") - 1][0] != "gap"


def test_single_session_files_report_one_segment_and_no_gap(tmp_path):
    m = load_svlog(build_log(tmp_path / "one.svlog", 30))
    assert len(m.segments) == 1
    assert m.gaps == [] and m.gap_times == []
    assert m.segments[0].ping_count == m.ping_count
    assert m.acquisition_s == pytest.approx(ping_span(m), abs=0.01)


def test_profiles_before_any_header_still_form_a_segment(tmp_path):
    packets = list(pose_burst(0, 0.0, 0.0))
    for n in range(10):
        for ch in (0, 1):
            packets.append(profile_packet(ch, n, 1000 + n * 50))
    m = load_svlog(write_log(tmp_path / "headless.svlog", packets))
    assert len(m.segments) == 1 and m.segments[0].ping_count == 10


def test_decode_session_header_survives_junk():
    assert decode_session_header(b"\x00\xffnot json") == {
        "wall_clock": None, "session_id": None, "devices": None}
    head = decode_session_header(
        b'{"session_id": "s", "timestamp": "T", '
        b'"session_devices": {"nickname": "port", "device_id": 1}}')
    assert head["session_id"] == "s" and head["wall_clock"] == "T"
    assert "port" in head["devices"]


@field_log
def test_cerulean_demo_segments_match_forensics():
    """Two sessions and a 397.8 s gap, agreed by both readers of the bytes."""
    m = load_svlog(FIELD_ROOT / CERULEAN_DEMO)
    f = analyse(FIELD_ROOT / CERULEAN_DEMO, render=False)
    assert len(m.segments) == len(f.segments) == 2
    assert [g.seconds for g in m.gaps] == pytest.approx(f.gaps_s, abs=1.0)
    assert [g.seconds for g in m.gaps] == pytest.approx([397.8], abs=1.0)
    assert [round(s.duration_s, 1) for s in m.segments] == pytest.approx(
        [165.1, 48.7], abs=1.0)
    assert [s.ping_count for s in m.segments] == [3303, 975]


@field_log
@pytest.mark.parametrize("rel", SINGLE_SESSION)
def test_single_session_field_logs_are_unaffected(rel):
    m = load_svlog(FIELD_ROOT / rel)
    assert len(m.segments) == 1
    assert m.gaps == []
    assert m.segments[0].ping_count == m.ping_count


# ---------------------------------------------------------------------------
# Step 2 — the breaks downstream
# ---------------------------------------------------------------------------
def test_waterfall_break_row_inserts_a_blank_row(qapp, tmp_config):
    from blueboat_gcs.core.waterfall_service import WaterfallService
    from blueboat_gcs.models.sonar import SonarPing

    wf = WaterfallService(tmp_config)
    y = np.linspace(-10.0, 10.0, 64)
    ping = SonarPing(t=0.0, robot_x=0.0, robot_y=0.0, yaw=0.0, water_depth=1.0,
                     y_local=y, intensity_db=np.full(64, -20.0, np.float32),
                     slant_range_m=10.0)
    wf.on_sonar_ping(ping)
    wf.break_row()
    wf.on_sonar_ping(ping)
    buf = wf.chronological()
    assert buf.shape[0] == 3
    assert np.isfinite(buf[0]).any() and np.isfinite(buf[2]).any()
    assert not np.isfinite(buf[1]).any(), "the separator row must be blank"


@field_log
def test_the_gap_breaks_every_accumulator(qapp, tmp_config):
    """Waterfall seam, rasterizer reset, trajectory break — on the real demo."""
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.waterfall_service import WaterfallService

    m = load_svlog(FIELD_ROOT / CERULEAN_DEMO)
    wf, mo = WaterfallService(tmp_config), MosaicService(tmp_config)
    saw_gap = False
    for kind, _t, obj in m.events:
        if kind == "ping":
            wf.on_sonar_ping(obj)
            mo.on_sonar_ping(obj)
        elif kind == "gap":
            wf.break_row()
            mo.reset_tracking()
            saw_gap = True
            assert mo._rasterizer._prev is None, (
                "the rasterizer would densify a swath across 397.8 s")
    assert saw_gap
    buf = wf.chronological()
    blank = np.where(~np.isfinite(buf).any(axis=1))[0]
    assert len(blank) == 1, "exactly one seam, at the session boundary"
    # The buffer holds the newest rows, so the seam sits one row above session 2.
    assert blank[0] == buf.shape[0] - m.segments[1].ping_count - 1


def test_feed_pings_breaks_the_window_at_a_session_boundary(tmp_path, qapp, tmp_config):
    from blueboat_gcs.core.seabed_imager import SeabedImager, feed_pings

    m = load_svlog(build_log(tmp_path / "tiles.svlog", 800, sessions=(0, 400),
                             stamp=lambda ch, n: (1000 + n * 50
                                                  + (300_000 if n >= 400 else 0))))
    assert len(m.gaps) == 1
    gap = m.gaps[0].seconds

    def run(breaks):
        imager = SeabedImager(tmp_config)
        imager._analyzer = lambda img: []
        out = []
        imager.image_ready.connect(out.append)
        feed_pings(imager, m.pings, breaks)
        return out

    without = run(())
    with_breaks = run(m.gap_times)
    assert any(np.diff(i.row_t).max() > gap / 2 for i in without), (
        "the fixture must actually produce a straddling window")
    assert all(np.diff(i.row_t).max() < gap / 2 for i in with_breaks)
    # Image ids stay unique and ordered across the boundary.
    ids = [i.image_id for i in with_breaks]
    assert ids == sorted(set(ids))


@field_log
def test_no_seabed_tile_straddles_the_cerulean_gap(qapp, tmp_config):
    """CM-10: a tile whose rows are 397.8 s apart is a corrupt training sample."""
    from blueboat_gcs.core.seabed_imager import SeabedImager, feed_pings

    m = load_svlog(FIELD_ROOT / CERULEAN_DEMO)
    imager = SeabedImager(tmp_config)
    imager._analyzer = lambda img: []
    images = []
    imager.image_ready.connect(images.append)
    feed_pings(imager, m.pings, m.gap_times)
    assert images
    worst = max(float(np.diff(i.row_t).max()) for i in images)
    assert worst < 1.0, f"an image spans {worst:.1f} s of the session gap"


# ---------------------------------------------------------------------------
# Step 3 — packet id 2194
# ---------------------------------------------------------------------------
def test_decode_omniscan_status_layout():
    pkt = status_packet(1, 4_275_186, v0=69.468, v1=52.052)
    d = decode_omniscan_status(pkt[8:-2])
    assert d["channel_number"] == 1
    assert d["timestamp_ms"] == 4_275_186
    assert d["value_0"] == pytest.approx(69.468, abs=1e-3)
    assert d["value_1"] == pytest.approx(52.052, abs=1e-3)
    with pytest.raises(ValueError):
        decode_omniscan_status(b"\x00" * 4)


def test_unrecognised_ids_are_counted_not_silently_skipped(tmp_path):
    packets = [session_packet(), status_packet(0, 1000),
               status_packet(1, 1000)]
    packets.extend(pose_burst(0, 0.0, 0.0))
    for n in range(10):
        for ch in (0, 1):
            packets.append(profile_packet(ch, n, 1000 + n * 50))
    m = load_svlog(write_log(tmp_path / "status.svlog", packets))
    assert m.unparsed_packets[2194] == 2
    assert m.ping_count == 10, "counting must not change what is emitted"


@field_log
@pytest.mark.parametrize("rel,expected", [(CERULEAN_DEMO, 192),
                                          (SONARVIEW_20M, 218),
                                          (TIRE_25M, 418)])
def test_field_status_packets_are_counted(rel, expected):
    m = load_svlog(FIELD_ROOT / rel)
    f = analyse(FIELD_ROOT / rel, render=False)
    assert m.unparsed_packets[2194] == expected
    assert f.census.counts[2194] == expected


@field_log
@pytest.mark.parametrize("rel", [CERULEAN_DEMO, SONARVIEW_20M, TIRE_25M])
def test_status_packets_carry_a_real_profile_timestamp_at_about_1_hz(rel):
    """What is established about id 2194, asserted against the bytes.

    Its ``uint32`` is a profile ``timestamp_ms`` from the same channel, and it
    arrives at ~0.9 Hz per device — not the 0.31 Hz once published, which
    divided the Cerulean demo's count by a span containing its 397.8 s gap.
    """
    from blueboat_gcs.core.svlog import (OS_MONO_PROFILE_ID,
                                         decode_os_mono_profile, side_of)

    data = (FIELD_ROOT / rel).read_bytes()
    status, profiles = [], {}
    for pkt in walk_packets(data):
        pid = struct.unpack_from("<H", pkt, 4)[0]
        if pid == 2194:
            status.append(decode_omniscan_status(pkt[8:-2]))
        elif pid == OS_MONO_PROFILE_ID:
            try:
                d = decode_os_mono_profile(pkt[8:-2])
            except ValueError:
                continue
            profiles.setdefault(side_of(d), set()).add(d["timestamp_ms"])
    assert status

    for ch in {s["channel_number"] for s in status}:
        rows = [s for s in status if s["channel_number"] == ch]
        assert ch in profiles, "status channel is not a channel the log carries"
        stamps = np.sort(np.array([s["timestamp_ms"] for s in rows], float))
        prof = np.sort(np.array(sorted(profiles[ch]), float))

        # Same device clock: inside the channel's profile time range,
        # monotonic, and never more than half a ping interval from the nearest
        # profile. It is independently sampled on that clock rather than a copy
        # of a profile stamp — only ~3 % of them land exactly on one.
        assert stamps.min() >= prof.min() and stamps.max() <= prof.max()
        i = np.clip(np.searchsorted(prof, stamps), 0, len(prof) - 1)
        delta = np.minimum(np.abs(prof[i] - stamps),
                           np.abs(prof[np.maximum(i - 1, 0)] - stamps))
        pri = float(np.median(np.diff(prof)))
        assert delta.max() <= pri / 2 + 1, (
            f"status stamps stray {delta.max():.0f} ms from a {pri:.0f} ms grid")

        dt = np.diff(stamps)
        rate = 1000.0 / np.median(dt[dt > 0])
        assert 0.7 < rate < 1.2, f"expected ~0.9 Hz per device, got {rate:.2f}"


@field_log
@pytest.mark.parametrize("rel", [CERULEAN_DEMO, TIRE_25M])
def test_status_floats_are_not_acoustic(rel):
    """The floats are unnamed here because they correlate with nothing.

    They are stable to under 1 over a whole log and uncorrelated with the same
    channel's power statistics, so they are housekeeping telemetry rather than
    signal statistics. This pins the evidence for that claim, so the docs are
    not left resting on an unverified guess.
    """
    from blueboat_gcs.core.svlog import (OS_MONO_PROFILE_ID,
                                         decode_os_mono_profile, side_of)

    data = (FIELD_ROOT / rel).read_bytes()
    status, prof = [], []
    for pkt in walk_packets(data):
        pid = struct.unpack_from("<H", pkt, 4)[0]
        if pid == 2194:
            s = decode_omniscan_status(pkt[8:-2])
            status.append((s["channel_number"], s["timestamp_ms"],
                           s["value_0"], s["value_1"]))
        elif pid == OS_MONO_PROFILE_ID:
            try:
                d = decode_os_mono_profile(pkt[8:-2])
            except ValueError:
                continue
            prof.append((side_of(d), d["timestamp_ms"], d["max_pwr_db"],
                         float(np.mean(d["pwr"]))))
    S, P = np.array(status, float), np.array(prof, float)
    for ch in np.unique(S[:, 0]):
        s, p = S[S[:, 0] == ch], P[P[:, 0] == ch]
        assert s[:, 2].std() < 1.0 and s[:, 3].std() < 1.0, (
            "the floats drift by under 1 over a whole log")
        idx = np.clip(np.searchsorted(p[:, 1], s[:, 1]), 0, len(p) - 1)
        for col in (2, 3):              # max_pwr_db, mean raw power
            for value in (s[:, 2], s[:, 3]):
                r = abs(np.corrcoef(value, p[idx, col])[0, 1])
                assert r < 0.3, f"unexpectedly correlated with column {col}"


# ---------------------------------------------------------------------------
def test_length_mm_fixture_is_the_documented_default():
    """Guards the import from test_svlog_forensics against silent drift."""
    assert LENGTH_MM == 20000
