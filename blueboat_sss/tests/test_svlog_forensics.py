"""`.svlog` forensics: the synthetic properties, and the field reference numbers.

Two halves, deliberately:

* **Synthetic** — files built in ``tmp_path`` from ``svlog_helper.frame_packet``,
  covering the properties that must hold on *any* input: the census enumerates
  unknown ids, a mid-file range change is flagged, a second session header
  starts a segment, ``channel_number = 255`` falls back to the transducer
  heading, and a corrupt file is refused rather than half-reported. These need
  no ROS, no display and no corpus, so they run everywhere.

* **Field reference** — the numbers already published in ``CLAUDE.md``'s
  *Measured acquisition settings* and in ``docs/SONARVIEW_SVLOG_ANALYSIS.md``.
  Reproducing that table automatically is this tool's acceptance test, so the
  table is pinned here rather than re-derived by hand every time. Gated on the
  corpus being mounted, exactly like ``test_processor_assembly.py``.

The field files are opened **read-only** — they are primary field data
(CLAUDE.md NC #6 / root CM-7). Nothing here writes outside ``tmp_path``.
"""

from __future__ import annotations

import struct
import sys

import pytest

from conftest import PKG_PARENT

from blueboat_gcs.analysis.svlog_forensics import (SvlogForensicsError, analyse,
                                                   main, render_comparison,
                                                   render_report)

# ``svlog_helper`` is the robot-side frame writer and is ROS-free; using it
# rather than hand-rolling frames means the synthetic files carry real
# checksums and a real header layout.
HELPERS = PKG_PARENT / "src" / "_custom_libraries"
if str(HELPERS) not in sys.path:
    sys.path.insert(0, str(HELPERS))

import svlog_helper                                          # noqa: E402

HEAD_FMT = "<IIIIIHHHBBffffff"
NUM_RESULTS = 600
LENGTH_MM = 20000
MIN_DB, MAX_DB = -100.0, 0.0


# ---------------------------------------------------------------------------
# Synthetic .svlog construction
# ---------------------------------------------------------------------------
def profile_packet(channel: int, ping_number: int, timestamp_ms: int, *,
                   src: int | None = None, length_mm: int = LENGTH_MM,
                   gain_index: int = 4, bottom_sample: int = 200,
                   heading: float | None = None) -> bytes:
    """One framed OS_MONO_PROFILE with a synthetic bottom return.

    ``src`` defaults to the value the packet's own side implies, so a test that
    wants a mis-tagged packet has to say so explicitly.
    """
    if heading is None:
        heading = -90.0 if channel == 0 else 90.0
    if src is None:
        side = channel if channel in (0, 1) else (1 if heading > 0 else 0)
        src = svlog_helper.DEVICE_ID_PORT if side == 0 else svlog_helper.DEVICE_ID_STBD

    pwr = [1200] * NUM_RESULTS
    for i in range(bottom_sample, min(bottom_sample + 12, NUM_RESULTS)):
        pwr[i] = 61000
    head = struct.pack(HEAD_FMT, ping_number, 0, length_mm, timestamp_ms, 0,
                       gain_index, NUM_RESULTS, 15000, channel, 0,
                       66.5e-6, 1.0, MAX_DB, MIN_DB, heading, 0.0)
    payload = head + struct.pack(f"<{NUM_RESULTS}H", *pwr)
    return svlog_helper.frame_packet(svlog_helper.OS_MONO_PROFILE_ID, payload,
                                     src=src, dst=0)


def session_packet(timestamp: str = "2026-08-30T00:00:00.000Z") -> bytes:
    payload = ('{"session_id": "s", "timestamp": "%s", '
               '"session_devices": [{"nickname": "port", "device_id": 1}]}'
               % timestamp).encode("utf-8")
    return svlog_helper.frame_packet(svlog_helper.JSON_WRAPPER_ID, payload,
                                     src=0, dst=svlog_helper.DST_BROADCAST)


def write_log(path, packets) -> object:
    path.write_bytes(b"".join(packets))
    return path


@pytest.fixture
def simple_log(tmp_path):
    """One session, two channels, 40 clean ping pairs at 50 ms."""
    packets = [session_packet()]
    for n in range(40):
        for ch in (0, 1):
            packets.append(profile_packet(ch, n, n * 50))
    return write_log(tmp_path / "simple.svlog", packets)


# ---------------------------------------------------------------------------
# Synthetic properties
# ---------------------------------------------------------------------------
def test_census_counts_every_packet_including_unknown_ids(tmp_path, simple_log):
    """The census enumerates ids; a whitelist would hide an unknown one."""
    extra = svlog_helper.frame_packet(4242, b"\x01\x02", src=3, dst=0)
    path = write_log(tmp_path / "unknown.svlog",
                     [simple_log.read_bytes(), extra, extra])

    r = analyse(path, render=False)
    assert r.census.counts[svlog_helper.OS_MONO_PROFILE_ID] == 80
    assert r.census.counts[svlog_helper.JSON_WRAPPER_ID] == 1
    assert r.census.counts[4242] == 2, "an unrecognised id must still be counted"
    assert r.census.frames == sum(r.census.counts.values())
    assert r.census.unframed_bytes == 0
    assert "unrecognised packet id(s): [4242]" in render_report(r)


def test_segments_split_on_session_header_and_gaps_are_reported(tmp_path):
    """A second id-10 starts a segment; the inter-segment gap is surfaced."""
    packets = [session_packet("2026-08-30T00:00:00.000Z")]
    for n in range(20):
        packets.append(profile_packet(0, n, n * 50))
    packets.append(session_packet("2026-08-30T00:10:00.000Z"))
    for n in range(20):                       # counter restarts, 400 s later
        packets.append(profile_packet(0, n, 400_000 + n * 50))
    path = write_log(tmp_path / "two_sessions.svlog", packets)

    r = analyse(path, render=False)
    assert [s.index for s in r.segments] == [0, 1]
    assert [s.profiles for s in r.segments] == [20, 20]
    assert r.gaps_s == pytest.approx([399.05], abs=0.1)
    # The counter restart must not read as loss: per segment both are clean,
    # while a whole-file count would have called the jump missing pings.
    for seg in r.segments:
        assert seg.channels[0].missing == 0
    assert r.missing_pct == 0.0


def test_mid_file_range_change_is_flagged(tmp_path):
    packets = [session_packet()]
    for n in range(30):
        packets.append(profile_packet(0, n, n * 50, length_mm=20000))
    for n in range(30, 40):
        packets.append(profile_packet(0, n, n * 50, length_mm=35000))
    path = write_log(tmp_path / "range_change.svlog", packets)

    r = analyse(path, render=False)
    stat = r.params[0]["length_mm"]
    assert stat.changed and stat.mode == 20000, "the mode is the majority value"
    assert stat.transitions == 1, "one operator change, not one per ping"
    assert r.param("length_mm") == 20000
    assert "Acquisition settings changed mid-file" in render_report(r)


def test_gain_drift_is_reported_as_a_transition_rate(tmp_path):
    """Auto-gain takes several values while changing on very few pings."""
    packets = [session_packet()]
    for n in range(100):
        packets.append(profile_packet(0, n, n * 50,
                                      gain_index=4 if n < 98 else 5))
    r = analyse(write_log(tmp_path / "gain.svlog", packets), render=False)
    stat = r.params[0]["gain_index"]
    assert stat.changed and stat.transitions == 1
    assert stat.transition_pct == pytest.approx(1.0)


def test_side_identity_falls_back_to_transducer_heading(tmp_path):
    """channel_number 255 on every packet: only the heading identifies the side.

    Two logs in the field corpus are exactly this (NC #1 / CM-5).
    """
    packets = [session_packet()]
    for n in range(20):
        packets.append(profile_packet(255, n, n * 50, heading=-90.0,
                                      src=svlog_helper.DEVICE_ID_PORT))
        packets.append(profile_packet(255, n, n * 50, heading=+90.0,
                                      src=svlog_helper.DEVICE_ID_STBD))
    r = analyse(write_log(tmp_path / "ch255.svlog", packets), render=False)

    assert set(r.params) == {0, 1}, "both sides recovered from the heading"
    assert r.src.channel_values == {255: 40}
    assert r.src.heading_fallback_pct == 100.0
    assert r.src.mismatches == 0


def test_wrong_src_is_measured_not_consumed(tmp_path):
    """A mis-tagged src is counted, and never used to route the ping."""
    packets = [session_packet()]
    for n in range(20):
        # Every port packet carries the starboard device tag.
        packets.append(profile_packet(0, n, n * 50,
                                      src=svlog_helper.DEVICE_ID_STBD))
        packets.append(profile_packet(1, n, n * 50))
    r = analyse(write_log(tmp_path / "mistag.svlog", packets), render=False)

    assert r.src.mismatches == 20
    assert r.src.pct == pytest.approx(50.0)
    assert r.src.longest_run == 1
    # Routing is unaffected: 20 pings on each side, keyed by channel_number.
    assert r.segments[0].channels[0].n == 20
    assert r.segments[0].channels[1].n == 20


def test_pri_uses_stamps_sorted_per_channel(tmp_path):
    """File order is not time order; sorting per channel is what makes PRI real.

    The writer batches by channel, so on real logs the stamps run backwards
    between batches. Interleaving two channels whose batches alternate
    reproduces that.
    """
    packets = [session_packet()]
    for block in range(10):
        for n in range(5):                    # a port batch...
            packets.append(profile_packet(0, block * 5 + n,
                                          (block * 5 + n) * 50))
        for n in range(5):                    # ...then the starboard batch,
            packets.append(profile_packet(1, block * 5 + n,   # stamps rewind
                                          (block * 5 + n) * 50))
    r = analyse(write_log(tmp_path / "batched.svlog", packets), render=False)

    assert r.non_monotonic > 0, "the fixture must actually rewind the clock"
    for ch in (0, 1):
        assert r.segments[0].channels[ch].pri_median_ms == pytest.approx(50.0)


def test_bottom_detection_and_waterfalls(tmp_path, simple_log):
    out = tmp_path / "out"
    r = analyse(simple_log, render=True, out_dir=out)

    assert r.fbr.detected == r.fbr.n == 80
    assert r.fbr.bottom_sample == 200, "the synthetic bottom sits at sample 200"
    assert r.fbr.groups == 40
    for name in ("waterfall_slant", "waterfall_ground"):
        assert r.images[name].is_file()
    import cv2
    slant = cv2.imread(str(r.images["waterfall_slant"]), cv2.IMREAD_GRAYSCALE)
    ground = cv2.imread(str(r.images["waterfall_ground"]), cv2.IMREAD_GRAYSCALE)
    assert slant.shape == ground.shape == (40, 800)
    for img in (slant, ground):
        assert len(set(img.ravel().tolist())) > 1, "image must not be flat"

    # The property the pair exists to show: the corrected image drops the water
    # column, so it retains fewer samples. Checked on the geometry rather than
    # on the PNGs — each is stretched 2-98 % independently, so pixel brightness
    # is not comparable between them.
    from blueboat_gcs.core.svlog import project_side
    import numpy as np
    db = np.zeros(NUM_RESULTS, dtype=np.float32)
    altitude = r.fbr.p50_m
    assert altitude == pytest.approx(6.68, abs=0.02), "bottom at sample 200"
    kept_slant = project_side(db, 0, LENGTH_MM, NUM_RESULTS, 0.0, 0.0, 1.0)[0]
    kept_ground = project_side(db, 0, LENGTH_MM, NUM_RESULTS, altitude,
                               0.0, 1.0)[0]
    assert kept_ground.size < kept_slant.size
    assert kept_ground.size == NUM_RESULTS - 201


@pytest.mark.parametrize("payload,message", [
    (b"", "file is empty"),
    (b"\x00" * 40_000, "no framed packets found"),
])
def test_unreadable_files_are_refused_not_half_reported(tmp_path, payload,
                                                        message):
    path = tmp_path / "bad.svlog"
    path.write_bytes(payload)
    with pytest.raises(SvlogForensicsError, match=message):
        analyse(path, render=False)


def test_header_only_file_is_refused(tmp_path, simple_log):
    """Framed packets but no decodable profile is still nothing to analyse."""
    path = tmp_path / "headers.svlog"
    path.write_bytes(session_packet())
    with pytest.raises(SvlogForensicsError, match="no decodable OS_MONO_PROFILE"):
        analyse(path, render=False)


def test_truncated_tail_is_warned_about_but_still_analysed(tmp_path, simple_log):
    data = simple_log.read_bytes()
    path = write_log(tmp_path / "cut.svlog", [data[:len(data) - 400]])

    r = analyse(path, render=False)
    assert r.census.truncated_tail and r.census.trailing_bytes > 0
    assert "the file is truncated" in render_report(r)


def test_report_is_deterministic_apart_from_the_timestamp(simple_log):
    a = render_report(analyse(simple_log, render=False), generated="T")
    b = render_report(analyse(simple_log, render=False), generated="T")
    assert a == b


def test_output_is_refused_inside_a_tree_holding_svlogs(tmp_path, simple_log):
    """NC #6 / CM-7: reports never land in the tree carrying field data."""
    for candidate in (simple_log.parent, simple_log.parent / "reports",
                      simple_log.parent.parent):
        rc = main(["--out", str(candidate), "--no-images", str(simple_log)])
        assert rc == 2, f"{candidate} should have been refused"
        assert not (candidate / simple_log.stem).exists()

    ok = tmp_path.parent / f"{tmp_path.name}-out"
    assert main(["--out", str(ok), "--no-images", str(simple_log)]) == 0
    assert (ok / simple_log.stem / "report.md").is_file()


def test_inputs_are_not_modified(tmp_path, simple_log):
    import hashlib
    before = hashlib.sha256(simple_log.read_bytes()).hexdigest()
    out = tmp_path.parent / f"{tmp_path.name}-untouched"
    assert main(["--out", str(out), str(simple_log)]) == 0
    assert hashlib.sha256(simple_log.read_bytes()).hexdigest() == before


# ---------------------------------------------------------------------------
# Field reference numbers — the acceptance test
# ---------------------------------------------------------------------------
from test_processor_assembly import FIELD_ROOT, field_log        # noqa: E402

#: The three files behind CLAUDE.md's *Measured acquisition settings*. The
#: 80 m sea trial is called ``reflection_evidence.svlog`` in
#: docs/SONARVIEW_SVLOG_ANALYSIS.md; this is the file that name refers to.
SEA_TRIAL_80M = ("ShiraishiJima/MainReflectionEvidence+misspingsWaterfall/"
                 "2026-07-23-11-45-00.svlog")
SONARVIEW_20M = "ShiraishiJima/diffDepthCompensation.svlog"
CERULEAN_DEMO = "harbor_scan_combined.svlog"


@field_log
def test_80m_sea_trial_reference():
    """Every published number for this file, reproduced from the bytes."""
    r = analyse(FIELD_ROOT / SEA_TRIAL_80M, render=False)

    assert r.profiles == 5250
    assert r.param("length_mm") == 80000 and r.param("num_results") == 600
    assert r.mm_per_sample == pytest.approx(133.3, abs=0.1)
    assert r.param("pulse_duration_us") == pytest.approx(213, abs=1)
    assert r.pri_median_ms == pytest.approx(110.0, abs=0.5)

    # SONARVIEW_SVLOG_ANALYSIS.md §2: 1040 of 5250, in runs of up to 24.
    assert r.src.mismatches == 1040
    assert r.src.pct == pytest.approx(19.8, abs=0.05)
    assert r.src.longest_run == 24

    # §3, channel-routed. The 356/192 the document used to quote are the
    # *src*-routed counts, i.e. the routing §9.1 replaced.
    ch = r.segments[0].channels
    assert (ch[0].missing, ch[1].missing) == (211, 49)
    assert ch[0].missing_pct == pytest.approx(7.7, abs=0.05)
    assert ch[1].missing_pct == pytest.approx(1.8, abs=0.05)
    assert r.missing_total == 260

    # §5: the altitude wandered 4.7 m to 45 m, p10 4.67 / p90 15.23.
    assert r.fbr.p10_m == pytest.approx(4.67, abs=0.01)
    assert r.fbr.p50_m == pytest.approx(6.54, abs=0.01)
    assert r.fbr.p90_m == pytest.approx(15.23, abs=0.01)
    assert r.fbr.max_m == pytest.approx(45.01, abs=0.01)
    assert r.fbr.bottom_sample == 49

    # §9.3: gain moved but changed on well under 1 % of pings, so it is not a
    # meaningful contributor to the banding.
    for stat in (r.params[0]["gain_index"], r.params[1]["gain_index"]):
        assert stat.transition_pct < 1.0


@field_log
def test_sonarview_reference():
    r = analyse(FIELD_ROOT / SONARVIEW_20M, render=False)

    assert r.profiles == 4900
    assert r.param("length_mm") == 20000 and r.param("num_results") == 600
    assert r.mm_per_sample == pytest.approx(33.3, abs=0.1)
    assert r.param("pulse_duration_us") == pytest.approx(66, abs=1)
    assert r.pri_median_ms == pytest.approx(50.0, abs=0.5)
    assert r.src.mismatches == 0
    assert r.missing_total == 0
    assert r.fbr.bottom_sample == 269

    # This file changes range mid-survey, which is why every single-value
    # summary of it quotes the mode.
    assert r.params[0]["length_mm"].changed
    assert r.fbr.bottom_sample_modal == 280, (
        "the sample index is not comparable across a range change")

    # §9.3: the two devices number their pings 60 apart on this log.
    assert (r.counter_offset, round(r.counter_confidence, 2)) == (60, 1.0)


@field_log
def test_cerulean_demo_reference_and_segmentation():
    r = analyse(FIELD_ROOT / CERULEAN_DEMO, render=False)

    assert r.profiles == 4278
    assert r.param("length_mm") == 25416 and r.param("num_results") == 1200
    assert r.mm_per_sample == pytest.approx(21.2, abs=0.1)
    assert r.param("pulse_duration_us") == pytest.approx(44, abs=1)
    assert r.pri_median_ms == pytest.approx(50.0, abs=0.5)
    assert r.fbr.bottom_sample == 137
    assert r.src.mismatches == 0

    # Two session headers and a 398 s gap.
    assert len(r.segments) == 2
    assert r.gaps_s == pytest.approx([397.8], abs=0.1)
    assert [round(s.duration_s, 1) for s in r.segments] == [165.1, 48.7]
    assert r.acquisition_s == pytest.approx(213.8, abs=0.1)
    assert r.span_s == pytest.approx(611.5, abs=0.1)

    # Counted per segment the file is clean; counted whole-file the counter
    # jump across the boundary would read as ~65 % loss.
    assert r.missing_total == 0
    assert r.census.counts[2194] == 192


@field_log
def test_comparison_table_reproduces_measured_acquisition_settings():
    """SPEC step 2: the CLAUDE.md table falls out of the tool automatically."""
    results = [analyse(FIELD_ROOT / rel, render=False)
               for rel in (SEA_TRIAL_80M, SONARVIEW_20M, CERULEAN_DEMO)]
    table = render_comparison(results, generated="T")

    for line in ("| Range | 80.0 m | 20.0 m | 25.4 m |",
                 "| Samples/ping | 600 | 600 | 1200 |",
                 "| Range sampling | 133 mm | 33 mm | 21 mm |",
                 "| Transmit pulse | 213 µs | 66 µs | 44 µs |",
                 "| Bottom at sample | 49/600 | 269/600 | 137/1200 |",
                 "| Wrong-`src` packets | 19.8 % | 0.0 % | 0.0 % |"):
        assert line in table, f"missing row: {line}"
    assert "| Ping interval | 110 ms (9.1 Hz) | 50 ms (20.0 Hz)" in table


#: docs/SONARVIEW_SVLOG_ANALYSIS.md §9.3 — the whole-corpus offset census.
#: 0 on ten two-sided logs, -1 on four, +60 on two, and two single-sided logs
#: with nothing to align.
OFFSET_CASES = [
    ("ShiraishiJima/DiffSV-US_whenDepthCompensation/2026-07-22-14-18-55.svlog", -1),
    ("ShiraishiJima/HarbourCleanExample.svlog", -1),
    ("ShiraishiJima/example-disturbances.svlog", -1),
    ("ShiraishiJima/toOptimize-harbourScan.svlog", -1),
    ("ShiraishiJima/diffDepthCompensation.svlog", 60),
    ("ShiraishiJima/No_sonarVNotOK_usOK_SimpleCurve/2026-07-21-15-50-26.svlog", 60),
    ("ShiraishiJima/TireExamples/Tire4-25m.svlog", 0),
    (SEA_TRIAL_80M, 0),
]


@field_log
@pytest.mark.parametrize("rel,expected", OFFSET_CASES)
def test_counter_offsets_match_the_corpus_census(rel, expected):
    r = analyse(FIELD_ROOT / rel, render=False)
    assert r.counter_offset == expected
    assert r.counter_confidence >= 0.94, (
        "the deferred vote holds >= 0.94 on every two-sided log in the corpus")


@field_log
def test_single_sided_log_reports_no_offset_rather_than_total_loss():
    """NC #2 / CM-6: a one-transducer log is clean, not 100 % lost."""
    r = analyse(FIELD_ROOT / CERULEAN_DEMO, render=False)
    assert set(r.params) == {0}
    assert (r.counter_offset, r.counter_confidence) == (0, 0.0)
    assert "nothing to align" in render_report(r, generated="T")


@field_log
def test_src_tag_is_uncorrelated_on_the_channel_255_logs():
    """The heading fallback is load-bearing here, not a formality.

    Both ``channel_number = 255`` logs carry a ``src`` that is close to a coin
    flip against the true side, so routing by the tag would scramble roughly
    half the file (NC #1 / CM-5).
    """
    for rel in ("ShiraishiJima/No_sonarVNotOK_usOK_SimpleCurve/"
                "2026-07-21-15-50-26.svlog",
                "ShiraishiJima/reflectionEvidenceFullharbour/"
                "2026-07-22-13-43-08.svlog"):
        r = analyse(FIELD_ROOT / rel, render=False)
        assert r.src.heading_fallback_pct == 100.0
        assert 35.0 < r.src.pct < 60.0, (rel, r.src.pct)
        assert set(r.params) == {0, 1}, "both sides recovered from the heading"
