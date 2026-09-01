"""svlog merge: two recorded logs -> one normal multi-session log.

Contract pinned here (core/svlog_merge.py + core/session_rebuild.py):

* the merged file loads through the UNCHANGED ``load_svlog`` as a
  two-session mission on the real clock, with every ping of both logs;
* the newer log lands right after the older one (gap == one median
  ping interval, so the ``MissionGap`` still breaks every accumulator);
* boot skew stays a single coherent file-wide median;
* poses auto-align via the two GPS origins when both exist, and pass
  through untouched (with a warning) when they don't;
* the sources are byte-identical afterwards (NC #6);
* the rebuilt session folder matches the live session template.

All synthetic, all in tmp_path, no ROS.
"""

from __future__ import annotations

import hashlib
import json
import struct

import numpy as np
import pytest

from blueboat_gcs.core.svlog import (OMNISCAN_STATUS_ID, OS_MONO_PROFILE_ID,
                                     decode_omniscan_status, load_svlog,
                                     walk_packets)
from blueboat_gcs.core.svlog_merge import default_merge_name, merge_svlogs
from blueboat_gcs.utils.geodesy import gps_to_enu

from test_svlog_forensics import profile_packet, session_packet, write_log
from test_svlog_replay import mavlink_packet, pose_burst, status_packet

PRI_MS = 50


def build_source_log(path, *, pings=30, stamp0=1_000, skew=1_000,
                     wall="2026-08-30T10:00:00.000Z", origin=None,
                     x0=0.0, with_status=True):
    """One single-session synthetic log with controllable wall clock,
    sonar-clock start, boot skew and (optional) GPS origin."""
    packets = [session_packet(timestamp=wall)]
    packets.extend(pose_burst(stamp0 - skew, x0, 0.0))
    if origin is not None:
        packets.append(mavlink_packet({
            "type": "GLOBAL_POSITION_INT", "time_boot_ms": stamp0 - skew,
            "lat": int(origin[0] * 1e7), "lon": int(origin[1] * 1e7)}))
    for n in range(pings):
        ts = stamp0 + n * PRI_MS
        for ch in (0, 1):
            packets.append(profile_packet(ch, n, ts))
        packets.extend(pose_burst(ts - skew, x0 + 0.5 * n, 0.0))
    if with_status:
        packets.append(status_packet(0, stamp0 + pings * PRI_MS // 2))
    return write_log(path, packets)


def sha256(path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def two_logs(tmp_path):
    older = build_source_log(tmp_path / "older.svlog", pings=30,
                             stamp0=1_000, skew=1_000,
                             wall="2026-08-30T10:00:00.000Z")
    newer = build_source_log(tmp_path / "newer.svlog", pings=20,
                             stamp0=500_000, skew=77_000,
                             wall="2026-08-30T11:30:45.000Z")
    return older, newer


def test_merged_log_is_a_normal_two_session_mission(two_logs, tmp_path):
    older, newer = two_logs
    out = tmp_path / "merged.svlog"
    report = merge_svlogs(older, newer, out)

    m = load_svlog(out)
    ma, mb = load_svlog(older), load_svlog(newer)
    assert len(m.segments) == 2
    assert m.ping_count == ma.ping_count + mb.ping_count, (
        "the merge lost or invented rows (NC #2)")
    assert not m.synthetic_clock, "the merged timeline fell off the real clock"
    # Exactly one gap, of one median ping interval: "right after".
    assert len(m.gaps) == 1
    assert m.gaps[0].seconds == pytest.approx(PRI_MS / 1000.0, abs=0.005)
    # Events stay time-sorted (the loader sorts; the shift must not fight it).
    times = [t for _k, t, _o in m.events]
    assert times == sorted(times)
    # One coherent boot skew: the older log's.
    assert m.boot_skew_ms == pytest.approx(1_000, abs=PRI_MS + 1)
    assert report.delta_ms == ((1_000 + 29 * PRI_MS + PRI_MS) - 500_000)


def test_overlapping_ping_numbers_do_not_collide(two_logs, tmp_path):
    """Both sources number their pings 0..N; the per-segment grouping key
    must keep all of them."""
    older, newer = two_logs
    out = tmp_path / "merged.svlog"
    merge_svlogs(older, newer, out)
    m = load_svlog(out)
    assert m.ping_count == 30 + 20


def test_sources_are_never_modified(two_logs, tmp_path):
    older, newer = two_logs
    before = (sha256(older), sha256(newer))
    merge_svlogs(older, newer, tmp_path / "merged.svlog")
    assert (sha256(older), sha256(newer)) == before, (
        "a source .svlog changed — NC #6")


def test_order_is_auto_detected(two_logs, tmp_path):
    """Passing (newer, older) writes the same file as (older, newer)."""
    older, newer = two_logs
    merge_svlogs(older, newer, tmp_path / "m1.svlog")
    report = merge_svlogs(newer, older, tmp_path / "m2.svlog")
    assert report.swapped
    assert report.older == older
    assert (tmp_path / "m1.svlog").read_bytes() == \
        (tmp_path / "m2.svlog").read_bytes()


def test_status_packets_are_shifted_too(two_logs, tmp_path):
    older, newer = two_logs
    out = tmp_path / "merged.svlog"
    report = merge_svlogs(older, newer, out)
    stamps = []
    for pkt in walk_packets(out.read_bytes()):
        if struct.unpack_from("<H", pkt, 4)[0] == OMNISCAN_STATUS_ID:
            stamps.append(decode_omniscan_status(pkt[8:-2])["timestamp_ms"])
    assert len(stamps) == 2
    # older's status untouched; newer's shifted by delta.
    assert stamps[0] == 1_000 + 30 * PRI_MS // 2
    assert stamps[1] == 500_000 + 20 * PRI_MS // 2 + report.delta_ms


def test_every_merged_packet_reframes_cleanly(two_logs, tmp_path):
    """walk_packets must re-parse the whole merged stream — checksummed,
    correctly framed, nothing swallowed."""
    older, newer = two_logs
    out = tmp_path / "merged.svlog"
    merge_svlogs(older, newer, out)
    n_src = (len(list(walk_packets(older.read_bytes())))
             + len(list(walk_packets(newer.read_bytes()))))
    merged = list(walk_packets(out.read_bytes()))
    assert len(merged) == n_src
    n_profiles = sum(
        1 for p in merged
        if struct.unpack_from("<H", p, 4)[0] == OS_MONO_PROFILE_ID)
    assert n_profiles == 2 * (30 + 20)


def test_gps_origins_align_the_newer_logs_poses(tmp_path):
    lat_a, lon_a = 43.6961, 7.3080
    lat_b, lon_b = 43.6964, 7.3085          # tens of metres away
    older = build_source_log(tmp_path / "a.svlog", pings=20, stamp0=1_000,
                             origin=(lat_a, lon_a),
                             wall="2026-08-30T10:00:00.000Z")
    newer = build_source_log(tmp_path / "b.svlog", pings=20, stamp0=900_000,
                             origin=(lat_b, lon_b),
                             wall="2026-08-30T11:00:00.000Z")
    out = tmp_path / "merged.svlog"
    report = merge_svlogs(older, newer, out)
    d_e, d_n = gps_to_enu(lat_a, lon_a, lat_b, lon_b)
    assert report.pose_offset_en == pytest.approx((d_e, d_n), abs=1e-6)

    mb = load_svlog(newer)
    m = load_svlog(out)
    # The newer half's last ping pose = its standalone pose + the ENU
    # offset (state x is ENU east; the builder walks east at 0.5 m/ping).
    last_merged = m.pings[-1]
    last_b = mb.pings[-1]
    assert last_merged.robot_x == pytest.approx(last_b.robot_x + d_e,
                                                abs=1e-6)
    assert last_merged.robot_y == pytest.approx(last_b.robot_y + d_n,
                                                abs=1e-6)
    # And the mission origin is the OLDER log's fix.
    assert m.origin == pytest.approx((lat_a, lon_a))


def test_without_gps_poses_pass_through_with_a_warning(two_logs, tmp_path):
    older, newer = two_logs                  # built with no GPS
    out = tmp_path / "merged.svlog"
    report = merge_svlogs(older, newer, out)
    assert report.pose_offset_en is None
    assert any("GPS" in w for w in report.warnings)
    m, mb = load_svlog(out), load_svlog(newer)
    assert m.pings[-1].robot_x == pytest.approx(mb.pings[-1].robot_x)


def test_self_merge_is_refused(two_logs, tmp_path):
    older, _ = two_logs
    with pytest.raises(ValueError):
        merge_svlogs(older, older, tmp_path / "nope.svlog")


def test_default_name_carries_both_times(two_logs):
    older, newer = two_logs
    # Wall clocks 10:00:00 and 11:30:45 -> MMSS 0000 and 3045.
    assert default_merge_name(older, newer) == "merged_2026_08_30_0000_3045"
    # Order-independent: the older log always names first.
    assert default_merge_name(newer, older) == "merged_2026_08_30_0000_3045"


def test_headerless_newer_log_gets_a_synthesized_session(tmp_path):
    """A log with no id-10 before its profiles would bleed into the older
    log's segment; the merge synthesizes a header for it."""
    older = build_source_log(tmp_path / "a.svlog", pings=10, stamp0=1_000,
                             wall="2026-08-30T10:00:00.000Z")
    # Hand-build a headerless log: poses + profiles only.
    packets = list(pose_burst(400_000 - 900, 0.0, 0.0))
    for n in range(10):
        for ch in (0, 1):
            packets.append(profile_packet(ch, n, 400_000 + n * PRI_MS))
        packets.extend(pose_burst(400_000 + n * PRI_MS - 900, float(n), 0.0))
    newer = write_log(tmp_path / "b.svlog", packets)
    out = tmp_path / "merged.svlog"
    merge_svlogs(older, newer, out)
    m = load_svlog(out)
    assert len(m.segments) == 2, "the headerless half merged into segment 1"
    assert m.ping_count == 20


# ---------------------------------------------------------------------------
# Session rebuild + GUI
# ---------------------------------------------------------------------------

def test_rebuilt_session_matches_the_live_template(qapp, tmp_config,
                                                   two_logs, tmp_path):
    """Same tree test_sim_session pins for a live session, produced
    offline from the merged log."""
    from blueboat_gcs.core.session_rebuild import rebuild_session

    older, newer = two_logs
    session = tmp_path / "merged_sessions" / "merged_x"
    session.mkdir(parents=True)
    out = session / "merged_x.svlog"
    merge_svlogs(older, newer, out)
    mission = rebuild_session(out, session, tmp_config,
                              extra_metadata={"merged_from": ["a", "b"]})

    assert mission.ping_count == 50
    meta = json.loads((session / "metadata.json").read_text())
    for key in ("ping_count", "detection_count", "adopted_svlogs", "mosaic"):
        assert key in meta, f"metadata.json lost {key!r}"
    assert meta["adopted_svlogs"] == ["merged_x.svlog"]
    assert meta["merged_from"] == ["a", "b"]
    for name in ("sonar_mosaic.npz", "sonar_mosaic.png",
                 "boat_trajectory.csv"):
        assert (session / "mosaic" / name).is_file(), f"missing mosaic/{name}"
    for name in ("waterfall.png", "waterfall_raw.npz"):
        assert (session / "waterfall" / name).is_file(), (
            f"missing waterfall/{name}")
    with np.load(session / "waterfall" / "waterfall_raw.npz") as npz:
        assert "intensity_db" in npz
        # 50 pings + 1 gap seam row.
        assert npz["intensity_db"].shape[0] == 51
    seabed = session / "seabed_images"
    assert list(seabed.glob("seabed_*.png")), "no seabed images rebuilt"
    assert (seabed / "metadata").is_dir()
    # The .svlog sits at the session ROOT, like an adopted one.
    assert (session / "merged_x.svlog").is_file()
    assert not (session / "svlog").exists()


def test_merge_button_end_to_end(qapp, tmp_config, no_modal_dialogs,
                                 monkeypatch, two_logs, tmp_path):
    """Drive ReplayWindow._merge_svlog through stubbed dialogs."""
    from PySide6.QtWidgets import QFileDialog, QInputDialog
    from blueboat_gcs.gui.replay_window import ReplayWindow

    older, newer = two_logs
    monkeypatch.setattr(QFileDialog, "getOpenFileName",
                        staticmethod(lambda *a, **k: (str(newer), "")))
    monkeypatch.setattr(QInputDialog, "getText",
                        staticmethod(lambda *a, **k: ("merged_gui", True)))

    win = ReplayWindow(load_svlog(older), tmp_config)
    try:
        win._merge_svlog()
        session = (tmp_path / "data_root" / "merged_sessions" / "merged_gui")
        assert (session / "merged_gui.svlog").is_file()
        assert (session / "metadata.json").is_file()
        assert (session / "mosaic" / "sonar_mosaic.png").is_file()
        m = load_svlog(session / "merged_gui.svlog")
        assert len(m.segments) == 2 and m.ping_count == 50

        # Refused on name collision — merged sessions are never overwritten.
        win._merge_svlog()
        assert len(list((tmp_path / "data_root"
                         / "merged_sessions").iterdir())) == 1
    finally:
        win.close()
        qapp.processEvents()
