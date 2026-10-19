"""The GCS side of BlueBoat-Perception (2026-10-16).

* the bridge: what goes out for every image is bit-identical to what the
  session saves (raw dB, PNG, metadata), under one key;
* ``stream_id``: one per imaging stream, new at every reset (START);
* georef convention 2: the picture grids, the click formula and the
  documented metadata agree;
* Q8: attitude (live buffer + replay ATTITUDE) and device ping numbers
  reach every picture row, from the row's own source ping;
* the publish job and ``RosManager.publish_seabed_image`` (no DDS: the
  publishers are captured);
* ``DetectionsListener``: a detection's own stamp wins when non-zero.
"""

from __future__ import annotations

import importlib.util
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from blueboat_gcs.config.settings import AppConfig
from blueboat_gcs.core import perception_bridge as bridge
from blueboat_gcs.core.display_model import DisplayModel
from blueboat_gcs.core.seabed_imager import (GEOREF, LEVER_ARM_M, SeabedImager,
                                             feed_pings, waterfall_pixel_to_world)
from blueboat_gcs.models.sonar import SonarPing
from blueboat_gcs.utils.attitude import AttitudeBuffer, quat_to_roll_pitch

from test_seabed_pictures import H, N, PITCH, make_row

HAVE_ROS_MSGS = all(importlib.util.find_spec(m) is not None
                    for m in ("rclpy", "sensor_msgs", "std_msgs"))
HAVE_VISION = HAVE_ROS_MSGS and importlib.util.find_spec("vision_msgs") is not None


def pings_with_attitude(n=80, step_m=PITCH * 0.7, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        row = make_row(H, rng)
        out.append(SonarPing(
            t=0.05 * i, robot_x=step_m * i, robot_y=0.0, yaw=0.2,
            water_depth=H + 0.15, y_local=np.zeros(0),
            intensity_db=np.zeros(0, np.float32), bin_size_m=PITCH, port_bin0=0,
            port_db=row[:N][::-1].copy(), stbd_bin0=0, stbd_db=row[N:].copy(),
            bottom_slant_m=H, gain_index=4,
            roll=0.001 * i, pitch=-0.002 * i, yaw_rate=0.01 * i,
            attitude_age_s=0.02, port_ping_number=1000 + i,
            stbd_ping_number=(1060 + i) if i % 10 else 0))
    return out


def images(geometry="square", pings=None, rows=16, stride=8):
    cfg = AppConfig()
    cfg.seabed.rows, cfg.seabed.stride = rows, stride
    cfg.seabed.row_geometry = geometry
    pings = pings if pings is not None else pings_with_attitude()
    imager = SeabedImager(cfg, model=DisplayModel.fit(cfg, pings))
    out = []
    imager.image_ready.connect(out.append)
    feed_pings(imager, pings)
    return imager, out, pings


# ---- the bridge ---------------------------------------------------------------
def test_the_envelope_and_arrays_are_what_the_session_saves(qapp, tmp_path):
    _, imgs, _ = images()
    img = imgs[1]
    env = bridge.envelope(img, "/p.png", "/m.json")
    assert env["schema"] == bridge.ENVELOPE_SCHEMA
    assert env["key"] == f"{img.stream_id}/{img.image_id}"
    assert env["metadata"] == img.metadata()
    assert env["stamp"] == dict(zip(("sec", "nanosec"),
                                    bridge.stamp(float(img.row_t[0]))))
    db, png = bridge.image_arrays(img)
    assert db.dtype == np.float32 and db.flags["C_CONTIGUOUS"]
    np.testing.assert_array_equal(db, img.intensity_db)
    np.testing.assert_array_equal(png, img.to_png8())
    # ...and the saved files carry the very same bits
    png_path, meta_path = img.save(tmp_path)
    import cv2
    np.testing.assert_array_equal(cv2.imread(str(png_path), cv2.IMREAD_GRAYSCALE), png)
    with np.load(tmp_path / "metadata" / f"seabed_{img.image_id:05d}_world.npz") as z:
        np.testing.assert_array_equal(z["intensity_db"], db)
    on_disk = json.loads(meta_path.read_text())
    assert on_disk == json.loads(json.dumps(env["metadata"]))
    key, stamp, db2, png2, js = bridge.publish_parts(img, None, None)
    assert key == env["key"] and json.loads(js)["key"] == key
    assert stamp == (env["stamp"]["sec"], env["stamp"]["nanosec"])


def test_stamps():
    assert bridge.stamp(12.25) == (12, 250_000_000)
    assert bridge.stamp(0.0) == (0, 0) and bridge.stamp(float("nan")) == (0, 0)
    assert bridge.stamp(1.9999999999) == (2, 0)


def test_stream_id_is_stable_within_a_stream_and_new_after_reset(qapp):
    imager, imgs, pings = images()
    assert len({i.stream_id for i in imgs}) == 1
    assert imgs[0].stream_id == imager.stream_id
    assert imager.stream_id.startswith("gcs-") and len(imager.stream_id) == 25
    first = imager.stream_id
    imager.reset()
    assert imager.stream_id != first
    later = []
    imager.image_ready.connect(later.append)
    feed_pings(imager, pings)
    assert later and later[0].image_id == 0 and later[0].stream_id == imager.stream_id


# ---- georef convention 2 ---------------------------------------------------------
@pytest.mark.parametrize("geometry", ["square", "ping"])
def test_grids_are_convention_2(qapp, geometry):
    _, imgs, _ = images(geometry)
    img = imgs[1]
    meta = img.metadata()
    assert meta["georef"] == GEOREF and meta["schema"] == 5
    W = img.intensity_db.shape[1]
    for i in (0, 7):
        r = meta["rows_data"][i]
        for j in (0, W // 2 - 1, W // 2, W - 1):
            wx, wy = waterfall_pixel_to_world(r["x_m"], r["y_m"], r["yaw_rad"],
                                              r["bottom_m"], img.bin_pitch_m, j, W,
                                              *LEVER_ARM_M)
            assert img.world_x[i, j] == np.float32(wx)
            assert img.world_y[i, j] == np.float32(wy)
    # the transducer's own point is the lever arm off the track
    i, j = 0, W // 2 - 1                          # first port bin (water column)
    r = meta["rows_data"][i]
    lateral = ((img.world_y[i, j] - r["y_m"]) * math.cos(r["yaw_rad"])
               - (img.world_x[i, j] - r["x_m"]) * math.sin(r["yaw_rad"]))
    assert lateral == pytest.approx(LEVER_ARM_M[0], abs=1e-5)


# ---- Q8: attitude and ping numbers ---------------------------------------------
@pytest.mark.parametrize("geometry", ["square", "ping"])
def test_rows_carry_their_source_pings_attitude_and_ping_numbers(qapp, geometry):
    _, imgs, pings = images(geometry)
    img = imgs[2]
    meta = img.metadata()
    by_t = {p.t: p for p in pings}
    for i, r in enumerate(meta["rows_data"]):
        if geometry == "ping":
            src = by_t[r["t_s"]]
        else:
            # the row copies the bins of one ping; its attitude is that ping's
            idx = int(np.argmin([abs(p.port_ping_number - r["port_ping_number"])
                                 for p in pings]))
            src = pings[idx]
        assert r["port_ping_number"] == src.port_ping_number
        assert r["stbd_ping_number"] == src.stbd_ping_number
        assert r["roll_rad"] == pytest.approx(src.roll)
        assert r["pitch_rad"] == pytest.approx(src.pitch)
        assert r["yaw_rate_rad_s"] == pytest.approx(src.yaw_rate)
        assert r["attitude_age_s"] == pytest.approx(0.02)
        # ...and it is the very ping whose bins the row copies
        np.testing.assert_array_equal(
            img.intensity_db[i], np.concatenate([src.port_db[::-1], src.stbd_db]))


def test_unknown_attitude_is_null_never_nan(qapp):
    from dataclasses import replace
    pings = [replace(p, roll=math.nan, pitch=math.nan, yaw_rate=math.nan,
                     attitude_age_s=math.nan) for p in pings_with_attitude()]
    _, imgs, _ = images(pings=pings)
    json.dumps(imgs[0].metadata()["rows_data"], allow_nan=False)   # raises on NaN
    r = imgs[0].metadata()["rows_data"][0]
    assert r["roll_rad"] is None and r["attitude_age_s"] is None


def test_attitude_buffer_attaches_the_latest_fresh_sample():
    buf = AttitudeBuffer(max_age_s=2.0)
    p = pings_with_attitude(1)[0]
    from dataclasses import replace
    live = replace(p, roll=math.nan, pitch=math.nan, yaw_rate=math.nan,
                   attitude_age_s=math.nan)
    assert math.isnan(buf.attach(live, 10.0).roll)            # nothing yet
    buf.update(9.9, 0.1, -0.2, 0.3)
    got = buf.attach(live, 10.0)
    assert (got.roll, got.pitch, got.yaw_rate) == (0.1, -0.2, 0.3)
    assert got.attitude_age_s == pytest.approx(0.1)
    assert math.isnan(buf.attach(live, 12.5).roll)            # stale: not attached
    assert buf.attach(p, 10.0).roll == p.roll                 # a replayed ping keeps its own


def test_quat_to_roll_pitch():
    r, pch = 0.3, -0.2
    cr, sr, cp, sp = math.cos(r / 2), math.sin(r / 2), math.cos(pch / 2), math.sin(pch / 2)
    q = (sr * cp, cr * sp, -sr * sp, cr * cp)        # yaw 0
    got = quat_to_roll_pitch(*q)
    assert got[0] == pytest.approx(r) and got[1] == pytest.approx(pch)


def test_replay_pings_carry_the_logs_attitude_and_ping_numbers(tmp_path):
    from blueboat_gcs.core.svlog import load_svlog
    from test_svlog_replay import (mavlink_packet, profile_packet, session_packet,
                                   write_log)
    packets = [session_packet()]
    for n in range(12):
        boot = n * 50 - 1000
        packets += [mavlink_packet({"type": "ATTITUDE", "time_boot_ms": boot,
                                    "roll": 0.05, "pitch": 0.02, "yaw": 0.0,
                                    "yawspeed": 0.1}),
                    mavlink_packet({"type": "LOCAL_POSITION_NED", "time_boot_ms": boot,
                                    "x": float(n), "y": 0.0, "z": 0.0,
                                    "vx": 1.0, "vy": 0.0, "vz": 0.0})]
        packets += [profile_packet(0, n, n * 50), profile_packet(1, n, n * 50)]
    mission = load_svlog(write_log(tmp_path / "att.svlog", packets))
    pings = mission.pings
    assert len(pings) >= 10
    p = pings[5]
    # NED/FRD on the wire (the processor's encoding) -> ENU/FLU here
    assert p.roll == pytest.approx(0.05)
    assert p.pitch == pytest.approx(-0.02)
    assert p.yaw_rate == pytest.approx(-0.1)
    assert p.attitude_age_s >= 0.0
    assert p.port_ping_number == 5 and p.stbd_ping_number == 5


# ---- publishing ------------------------------------------------------------------
def test_the_publish_job_sends_the_bridge_parts(qapp):
    from blueboat_gcs.gui.main_window import _PerceptionPublishJob
    _, imgs, _ = images()
    sent = []
    ros = SimpleNamespace(publish_seabed_image=lambda *a: sent.append(a) or True)
    _PerceptionPublishJob(ros, imgs[0], "/x.png", None).run()
    (key, stamp, db, png, js), = sent
    assert key == f"{imgs[0].stream_id}/0"
    np.testing.assert_array_equal(db, imgs[0].intensity_db)
    assert json.loads(js)["png_path"] == "/x.png"


@pytest.mark.skipif(not HAVE_ROS_MSGS, reason="needs rclpy + sensor_msgs")
def test_ros_manager_builds_bit_exact_image_messages(qapp):
    from sensor_msgs.msg import Image
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.ros.ros_manager import RosManager
    _, imgs, _ = images()
    img = imgs[0]
    rm = RosManager(AppConfig(), AppSignals())
    got = {"img": [], "png": [], "meta": []}
    rm._image_cls = Image
    rm._image_pubs = tuple(SimpleNamespace(publish=got[k].append)
                           for k in ("img", "png", "meta"))
    assert rm.publish_seabed_image(*bridge.publish_parts(img, None, None))
    m = got["img"][0]
    assert (m.encoding, m.height, m.width, m.step) == (
        "32FC1", img.intensity_db.shape[0], img.intensity_db.shape[1],
        4 * img.intensity_db.shape[1])
    back = np.frombuffer(bytes(m.data), "<f4").reshape(img.intensity_db.shape)
    np.testing.assert_array_equal(back, img.intensity_db)
    assert m.header.frame_id == f"{img.stream_id}/0"
    p = got["png"][0]
    np.testing.assert_array_equal(
        np.frombuffer(bytes(p.data), np.uint8).reshape(p.height, p.width), img.to_png8())
    assert json.loads(got["meta"][0].data)["metadata"]["image_id"] == 0


# ---- detections listener ------------------------------------------------------------
@pytest.mark.skipif(not HAVE_VISION, reason="needs vision_msgs")
def test_a_detections_own_stamp_wins_when_set():
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    from blueboat_gcs.ros.detections_listener import DetectionsListener
    arr = Detection2DArray()
    arr.header.stamp.sec, arr.header.stamp.nanosec = 100, 0
    for sec, nsec in ((95, 500_000_000), (0, 0)):
        d = Detection2D()
        d.header.stamp.sec, d.header.stamp.nanosec = sec, nsec
        d.id = "2000001"
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id, h.hypothesis.score = "dummy_center", 0.5
        d.results.append(h)
        arr.detections.append(d)
    a, b = DetectionsListener._msg_to_detections(arr)
    assert a.t == pytest.approx(95.5) and b.t == pytest.approx(100.0)
    assert a.uid == 2000001 and a.class_name == "dummy_center"


# ---- replay "Run AI" through Perception's core ------------------------------------
def test_replay_run_ai_draws_perceptions_detections(qapp, tmp_config, no_modal_dialogs,
                                                     tmp_path):
    from blueboat_gcs.core import perception_core
    from blueboat_gcs.core.svlog import load_svlog
    from blueboat_gcs.gui.replay_window import ReplayWindow
    from test_svlog_replay import build_log
    core, why = perception_core.load()
    if core is None:
        pytest.skip(why)
    tmp_config.seabed.rows, tmp_config.seabed.stride = 16, 8
    win = ReplayWindow(load_svlog(build_log(tmp_path / "ai.svlog", 60)), tmp_config)
    try:
        win._run_ai()
        uids = sorted(win.detection_layer._items)
        assert uids and all(2_000_000 <= u < 3_000_000 for u in uids)
        assert len(win.waterfall_service._detections) == len(uids)
    finally:
        win.close()
        qapp.processEvents()


def test_replay_run_ai_says_so_without_perception(qapp, tmp_config, no_modal_dialogs,
                                                  tmp_path, monkeypatch):
    from blueboat_gcs.core import perception_core
    from blueboat_gcs.core.svlog import load_svlog
    from blueboat_gcs.gui.replay_window import ReplayWindow
    from test_svlog_replay import build_log
    monkeypatch.setattr(perception_core, "load", lambda: (None, "not here"))
    win = ReplayWindow(load_svlog(build_log(tmp_path / "ai.svlog", 30)), tmp_config)
    try:
        win._run_ai()
        assert win.detection_layer._items == {}
        assert "not here" in win.statusBar().currentMessage()
    finally:
        win.close()
        qapp.processEvents()
