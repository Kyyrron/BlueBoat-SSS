"""Robot-side row assembly: never drop a ping (CLAUDE.md NC #2 / root CM-6).

Unlike the rest of this suite these tests need a sourced ROS 2 environment,
because they drive the real ``sss_processor_node`` — the node under test *is*
the thing whose behaviour NC #2 constrains, and a reimplementation would test
nothing. They skip cleanly on a laptop without ROS, so the suite stays
laptop-runnable as documented.

The node is driven directly rather than spun: ``_on_port`` / ``_on_starboard``
are the real callbacks, and ``_pub.publish`` is captured. That keeps the tests
deterministic and off the DDS graph.

Three properties are under test:

* every ping that arrives with a pose leaves as a row, one-sided if that is
  all that arrived, and nothing is held forever;
* the first ping of a mission is published, rather than withheld until the
  FBR tracker locks;
* the two devices' independent ping counters are aligned before grouping,
  so the halves of a merged row are the same acquisition instant.

The field ``.svlog`` files are opened **read-only** — they are primary field
data (NC #6 / CM-7).
"""

from __future__ import annotations

import importlib.util
import struct
import sys
from functools import lru_cache
from pathlib import Path

import pytest

from conftest import PKG_PARENT

# Availability is probed with find_spec rather than pytest.importorskip:
# a module-level importorskip raises Skipped during collection, which on
# pytest 7.4.4 here aborts the WHOLE session — the GCS tests silently stop
# running. `pytestmark` skips this module without touching the rest.
REQUIRED = ("rclpy", "blueboat_interfaces", "mavros_msgs", "geographic_msgs")
MISSING = [m for m in REQUIRED if importlib.util.find_spec(m) is None]

pytestmark = pytest.mark.skipif(
    bool(MISSING),
    reason=f"robot-side tests need a sourced ROS 2 env (missing: {MISSING})")


# ---------------------------------------------------------------------------
# Loading the node under test
# ---------------------------------------------------------------------------
# CMakeLists.txt installs the five src/ scripts flat into lib/blueboat_sss/,
# which is what makes the node's own bare `from sss_helper import ...` work.
# From the source tree the helpers sit one directory deeper, so put that on
# sys.path and load the node by path — this tests the source, not whatever
# happens to be installed.
HELPERS = PKG_PARENT / "src" / "_custom_libraries"
if str(HELPERS) not in sys.path:
    sys.path.insert(0, str(HELPERS))

if not MISSING:
    import rclpy
    from blueboat_interfaces.msg import OmniscanProfile
    from nav_msgs.msg import Odometry

    _spec = importlib.util.spec_from_file_location(
        "sss_processor_node", PKG_PARENT / "src" / "sss_processor_node.py")
    proc = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(proc)

NUM_RESULTS = 600
LENGTH_MM = 20000
MIN_DB, MAX_DB = -100.0, 0.0
SAMPLE_MM = LENGTH_MM / (NUM_RESULTS - 1)


def make_profile(channel: int, ping_number: int, stamp_ns: int,
                 bottom_sample: int | None = 200) -> OmniscanProfile:
    """One synthetic profile. ``bottom_sample=None`` gives a flat, bottomless
    ping, which is how the FBR tracker is held unlocked."""
    msg = OmniscanProfile()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.side = "port" if channel == 0 else "starboard"
    msg.ping_number = ping_number
    msg.start_mm = 0
    msg.length_mm = LENGTH_MM
    msg.num_results = NUM_RESULTS
    msg.channel_number = channel
    msg.transducer_heading_deg = -90.0 if channel == 0 else 90.0
    msg.min_pwr_db = MIN_DB
    msg.max_pwr_db = MAX_DB
    # Quiet water column (-90 dB), with a hard return at bottom_sample.
    pwr = [6553] * NUM_RESULTS
    if bottom_sample is not None:
        for i in range(bottom_sample, min(bottom_sample + 40, NUM_RESULTS)):
            pwr[i] = 60000
    msg.pwr_results = pwr
    return msg


def make_odom(stamp_ns: int, x: float = 0.0, y: float = 0.0) -> Odometry:
    odom = Odometry()
    odom.header.stamp.sec = stamp_ns // 1_000_000_000
    odom.header.stamp.nanosec = stamp_ns % 1_000_000_000
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation.w = 1.0
    return odom


@pytest.fixture
def node():
    """A real SSSProcessorNode with its publisher captured.

    ``.svlog`` logging is OFF at construction and never enabled here, so
    nothing is written to the data root.
    """
    rclpy.init()
    n = proc.SSSProcessorNode()
    published = []
    n._pub.publish = published.append          # capture instead of transmit
    n.published = published
    assert not n._svlog.active, "logging must be off; a test must never write .svlog"
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def feed_odom(node, count=50, period_ns=50_000_000):
    for i in range(count):
        node._odom_buf.push(make_odom(i * period_ns, x=float(i)))


# ---------------------------------------------------------------------------
# Stage 1 — assembly
# ---------------------------------------------------------------------------
PING_PERIOD_NS = 50_000_000


def test_every_ping_is_emitted_despite_gaps_and_late_arrivals(node):
    """NC #2: emitted rows == distinct pings received. No silent losses.

    One side is missing on every 10th ping and one side arrives >50 ms late
    on every 7th — the two cases the old arrival-time matcher discarded.
    """
    feed_odom(node, count=200, period_ns=PING_PERIOD_NS)
    n_pings = 60
    late = []
    for i in range(n_pings):
        pn = 1000 + i
        t = i * PING_PERIOD_NS
        node._on_port(make_profile(0, pn, t))
        if i % 10 == 9:
            continue                      # starboard half never arrives
        stbd = make_profile(1, pn, t + 1_000_000)
        if i % 7 == 6:
            # 4 ping periods late — far outside the old 50 ms window.
            stbd.header.stamp.sec = (t + 4 * PING_PERIOD_NS) // 1_000_000_000
            stbd.header.stamp.nanosec = (t + 4 * PING_PERIOD_NS) % 1_000_000_000
            late.append(pn)
        node._on_starboard(stbd)
    node._flush_pending(drain_all=True)

    assert len(node.published) == n_pings, (
        f"{len(node.published)} rows for {n_pings} pings — "
        "pings are being dropped (NC #2)"
    )
    assert node._dropped_no_odom == 0

    # Every ping number appears exactly once, on the port side.
    seen = [m.port_ping_number for m in node.published]
    assert sorted(seen) == [1000 + i for i in range(n_pings)]

    # The six one-sided rows are exactly the ones whose starboard half never came.
    one_sided = [m for m in node.published if not m.starboard_ping_number]
    assert len(one_sided) == 6
    assert sorted(m.port_ping_number for m in one_sided) == \
        [1000 + i for i in range(n_pings) if i % 10 == 9]

    # Late arrivals still merged into a two-sided row rather than being split.
    for m in node.published:
        if m.port_ping_number in late:
            assert m.starboard_ping_number == m.port_ping_number


def test_one_sided_rows_carry_the_documented_convention(node):
    """The absent side is unambiguous: ping_number 0, zero stamp, empty arrays."""
    feed_odom(node)
    node._on_port(make_profile(0, 500, 0))
    node._flush_pending(drain_all=True)

    assert len(node.published) == 1
    m = node.published[0]
    assert m.port_ping_number == 500
    assert len(m.port_y) > 0
    # Presence is the ping number; the absent side is fully blanked and never
    # borrows the present side's values.
    assert m.starboard_ping_number == 0
    assert m.starboard_stamp.sec == 0 and m.starboard_stamp.nanosec == 0
    assert len(m.starboard_y) == 0
    assert len(m.starboard_intensity_db) == 0
    # +y = port is preserved on a one-sided row (NC #1).
    assert all(v > 0 for v in m.port_y)


def test_incomplete_group_is_flushed_not_held(node):
    """A group with one side must not be held forever when the stream stops."""
    feed_odom(node, count=200, period_ns=PING_PERIOD_NS)
    # Get past the pre-roll so a real group exists to age out.
    for i in range(proc.OFFSET_PREROLL_MAX):
        t = i * PING_PERIOD_NS
        node._on_port(make_profile(0, 900 + i, t))
        node._on_starboard(make_profile(1, 900 + i, t + 1_000_000))
    node.published.clear()

    node._on_port(make_profile(0, 5000, 100 * PING_PERIOD_NS))
    assert node.published == [], "a fresh group should wait briefly for its partner"
    assert 5000 in node._groups

    # The wall-clock bound elapses; the timer path emits it one-sided.
    node._groups[5000]["first_ns"] -= 2 * proc.ASSEMBLY_MAX_LAG_NS
    node._flush_pending()
    assert len(node.published) == 1
    assert node.published[0].port_ping_number == 5000
    assert 5000 not in node._groups


def test_preroll_is_bounded_and_never_strands_a_ping(node):
    """A run shorter than the pre-roll still delivers every ping it produced."""
    feed_odom(node)
    node._on_port(make_profile(0, 42, 0))
    assert node.published == [], "held while the counter offset is unknown"

    # Stream stops. The timer path must still deliver it once it goes stale.
    node._preroll[0] = (node._preroll[0][0], node._preroll[0][1],
                        -2 * proc.ASSEMBLY_MAX_LAG_NS)
    node._flush_pending()
    assert len(node.published) == 1
    assert node.published[0].port_ping_number == 42


def test_group_buffer_is_bounded(node):
    """The live group buffer never grows without limit."""
    feed_odom(node, count=400, period_ns=PING_PERIOD_NS)
    for i in range(4 * proc.ASSEMBLY_MAX_GROUPS):
        node._on_port(make_profile(0, 2000 + i, i * PING_PERIOD_NS))
    assert len(node._groups) <= proc.ASSEMBLY_MAX_GROUPS + 1
    # Everything evicted was published, not discarded.
    assert len(node.published) + len(node._groups) == 4 * proc.ASSEMBLY_MAX_GROUPS


def test_side_comes_from_the_packet_not_the_topic(node):
    """NC #1: a profile delivered to the wrong topic still lands on its own side."""
    feed_odom(node)
    # channel_number says starboard, but it arrives on the port callback.
    node._on_port(make_profile(1, 700, 0))
    node._flush_pending(drain_all=True)

    m = node.published[0]
    assert m.starboard_ping_number == 700 and m.port_ping_number == 0
    assert all(v < 0 for v in m.starboard_y), "-y = starboard"


# ---------------------------------------------------------------------------
# Stage 2 — the bootstrap gate is gone
# ---------------------------------------------------------------------------
def test_first_ping_is_published_before_the_tracker_locks(node):
    """NC #2: no ping is withheld while the FBR tracker bootstraps."""
    feed_odom(node)
    node._on_port(make_profile(0, 1, 0, bottom_sample=None))
    node._on_starboard(make_profile(1, 1, 1_000_000, bottom_sample=None))
    node._flush_pending(drain_all=True)

    assert len(node.published) == 1, "the first ping of a mission must be emitted"
    assert node.published[0].port_ping_number == 1, "ping #1, not the tenth"
    assert not node._fbr.locked
    assert node._unlocked_pings == 1


def test_unlocked_altitude_is_an_identity_transform_not_a_guess(node):
    """With nothing ever detected, water_depth is 0 and no correction is applied."""
    feed_odom(node)
    node._on_port(make_profile(0, 1, 0, bottom_sample=None))
    node._flush_pending(drain_all=True)

    m = node.published[0]
    assert m.water_depth == 0.0, "an unlocked altitude must be distinguishable"
    # altitude 0 => ground range == slant range. project_side keeps
    # `slant > altitude`, so only the degenerate zero-range sample goes; the
    # water column is not cut, and every y is the raw slant range.
    assert len(m.port_y) == NUM_RESULTS - 1
    assert m.port_y[0] == pytest.approx(SAMPLE_MM / 1000.0, abs=1e-6)
    assert m.port_y[-1] == pytest.approx(LENGTH_MM / 1000.0, abs=1e-3)


def test_bootstrap_counter_counts_emissions_never_drops(node):
    """The old _dropped_bootstrap counted discarded pings; nothing is discarded now."""
    feed_odom(node, count=200, period_ns=PING_PERIOD_NS)
    for i in range(30):
        t = i * PING_PERIOD_NS
        node._on_port(make_profile(0, 1 + i, t, bottom_sample=None))
        node._on_starboard(make_profile(1, 1 + i, t + 1_000_000, bottom_sample=None))

    assert len(node.published) == 30
    assert node._unlocked_pings == 30
    assert node._dropped_no_odom == 0
    assert not hasattr(node, "_dropped_bootstrap")


def test_pose_remains_a_hard_gate(node):
    """The odom gate is deliberately untouched: a ping with no pose is unplaceable."""
    node._on_port(make_profile(0, 1, 0))
    node._on_starboard(make_profile(1, 1, 1_000_000))
    node._flush_pending(drain_all=True)
    assert node.published == []
    assert node._dropped_no_odom == 1


# ---------------------------------------------------------------------------
# Stage 0 + 3 — the ping counters, and real field logs
# ---------------------------------------------------------------------------
FIELD_ROOT = Path("/media/kyyrron/OS/Users/killi/Desktop/Research Kyutech/"
                  "BlueBoat/allSvlogData")

# Offsets measured over the whole corpus; see
# blueboat_gcs/docs/SONARVIEW_SVLOG_ANALYSIS.md.
FIELD_CASES = [
    ("ShiraishiJima/TireExamples/Tire4-25m.svlog", 0),
    ("ShiraishiJima/toOptimize-harbourScan.svlog", -1),
    ("ShiraishiJima/diffDepthCompensation.svlog", 60),
    ("ShiraishiJima/No_sonarVNotOK_usOK_SimpleCurve/2026-07-21-15-50-26.svlog", 60),
    ("harbor_scan_combined.svlog", 0),          # single-sided
]

field_log = pytest.mark.skipif(not FIELD_ROOT.is_dir(),
                               reason="field .svlog corpus not mounted")

# Profiles fed through the full processor pipeline per log.
PROCESSOR_FEED_LIMIT = 3000

# load_svlog decodes and processes an entire file, which is far and away the
# slowest thing here. The offset and time-adjacency properties are checked on
# every log by the cheap header-only tests above; this end-to-end pass over
# the replay implementation runs on two representatives — the +60 case it
# exists to catch, and a single-transducer log.
REPLAY_CASES = [c for c in FIELD_CASES
                if c[0].endswith(("diffDepthCompensation.svlog",
                                  "harbor_scan_combined.svlog"))]


# The largest log is ~17 000 profiles; decoding one is the expensive part of
# these tests and several of them want the same file. This suite is wired
# into the pre-commit hook, so the cache is what keeps it usable.
@lru_cache(maxsize=None)
def read_profiles(path: Path):
    """(channel, ping_number, timestamp_ms) per profile packet, in file order.

    Read-only: these are primary field data (NC #6).
    """
    from blueboat_gcs.core.svlog import (OS_MONO_PROFILE_ID,
                                         decode_os_mono_profile, side_of,
                                         walk_packets)
    out = []
    for pkt in walk_packets(path.read_bytes()):
        if struct.unpack_from("<H", pkt, 4)[0] != OS_MONO_PROFILE_ID:
            continue
        try:
            d = decode_os_mono_profile(pkt[8:-2])
        except ValueError:
            continue
        out.append((side_of(d), d["ping_number"], d["timestamp_ms"]))
    return tuple(out)


@field_log
@pytest.mark.parametrize("rel,expected", FIELD_CASES)
def test_counter_offset_is_recovered_from_every_field_log(rel, expected):
    """Stage 0: the two devices' counters differ by an arbitrary constant.

    0, -1 and +60 all occur in the corpus, so the raw counter is not a shared
    key. The estimator must recover the offset decisively — a bimodal
    estimator would sit near 50 % and inject a one-ping mis-pairing.
    """
    est = proc.PingCounterOffset()
    profiles = read_profiles(FIELD_ROOT / rel)
    for channel, pn, ts in profiles:
        est.observe(channel, pn, ts)

    if len({c for c, _, _ in profiles}) < 2:      # single-sided log
        assert est.votes == 0
        return
    assert est.offset == expected
    assert est.confidence >= 0.90, "estimator is not decisive on this log"


@field_log
@pytest.mark.parametrize("rel,expected", FIELD_CASES)
def test_field_log_replays_without_losing_a_ping(rel, expected, node):
    """Stage 3: nothing is lost replaying a real log through the processor.

    Arrival stamps come from each ping's own device ``timestamp_ms``, which
    is its acquisition instant. File order is not a usable substitute: the
    ``.svlog`` writer batches the two channels in blocks, so on the +60 logs
    the two halves of one ping sit ~124 packets apart in the file while
    having been acquired ~1 ms apart.
    """
    # A prefix exercises every path the whole file does — the offset lock,
    # both-sided merges, one-sided flushes — at a fraction of the runtime,
    # and this suite gates every commit.
    profiles = read_profiles(FIELD_ROOT / rel)[:PROCESSOR_FEED_LIMIT]
    span_ms = profiles[-1][2] - profiles[0][2]
    feed_odom(node, count=int(span_ms / 1000) + 20, period_ns=1_000_000_000)

    t0 = profiles[0][2]
    for channel, pn, ts in profiles:
        node._on_port(make_profile(channel, pn, (ts - t0) * 1_000_000))
    node._flush_pending(drain_all=True)

    # The invariant NC #2 actually states: every profile that went in comes
    # out on some row. Counting sides rather than rows is what makes this
    # airtight — a lost half would otherwise hide inside a one-sided row.
    sides_out = sum(bool(m.port_ping_number) + bool(m.starboard_ping_number)
                    for m in node.published)
    assert sides_out == len(profiles), "a ping was dropped (NC #2)"
    assert node._dropped_no_odom == 0

    # And every distinct ping is represented.
    keys = {pn if c == 0 else pn + node._offset.offset for c, pn, _ in profiles}
    emitted = set()
    for m in node.published:
        if m.port_ping_number:
            emitted.add(m.port_ping_number)
        if m.starboard_ping_number:
            emitted.add(m.starboard_ping_number + node._offset.offset)
    assert emitted == keys
    assert len(node.published) == len(keys), (
        "a ping number produced more than one row: its two halves were "
        "split across the flush window"
    )


@field_log
@pytest.mark.parametrize("rel,expected",
                         [c for c in FIELD_CASES if c[1] not in (0,)])
def test_merged_halves_are_the_same_acquisition_instant(rel, expected):
    """The point of the offset: a merged row must not span seconds of travel.

    On the +60 logs, grouping on the raw counter merges halves ~1.7-3.0 s
    apart — the boat has moved metres between them.
    """
    profiles = read_profiles(FIELD_ROOT / rel)
    offset, confidence = _replay_offset(FIELD_ROOT / rel)
    assert offset == expected and confidence >= 0.90

    groups: dict = {}
    for channel, pn, ts in profiles:
        key = pn if channel == 0 else pn + offset
        groups.setdefault(key, {})[channel] = ts
    deltas = [abs(v[0] - v[1]) for v in groups.values() if 0 in v and 1 in v]
    assert deltas, "expected two-sided rows in this log"
    assert max(deltas) < 200, (
        f"merged halves are up to {max(deltas)} ms apart — the counters are "
        "not aligned"
    )


def _replay_offset(path: Path):
    from blueboat_gcs.core.svlog import estimate_counter_offset
    return estimate_counter_offset(read_profiles(path))


@field_log
@pytest.mark.parametrize("rel,expected", REPLAY_CASES)
def test_replay_path_agrees_with_the_processor(rel, expected):
    """The GCS replay path assembles the same rows as the robot side.

    They are two implementations of one rule; a divergence here means one of
    them is losing or inventing rows.
    """
    from blueboat_gcs.core.svlog import load_svlog

    path = FIELD_ROOT / rel
    mission = load_svlog(path)
    profiles = read_profiles(path)
    keys = {pn if c == 0 else pn + mission.counter_offset
            for c, pn, _ in profiles}

    assert mission.counter_offset == expected
    assert mission.ping_count + mission.dropped_no_pose == len(keys)
