"""Regression: the station's seabed output before vs after the Perception work.

``fixtures/perception_golden.json`` was captured on 2026-10-16 from the
code as it stood **before** the BlueBoat-Perception changes
(``capture_perception_golden.py``, over the pings of ``golden_pings.py``).
Replaying the same pings through today's code must give:

* **bit-identical** pictures: raw dB, PNG, row times and poses, and the
  bytes of every saved PNG;
* identical metadata apart from exactly the ordered changes -- ``schema``
  4 -> 5, the pixel->world ``formula`` text (Q6), ``detections`` now ``[]``
  (one detection source: Perception) -- and exactly the additive keys
  (``stream_id``, ``georef``; per row the Q8 attitude and ping numbers);
* the identical ``/sss_ai/seabed_analysis`` payload apart from ``schema``
  and ``detections``;
* world grids that changed only because the convention changed: today's
  ``waterfall_pixel_to_world`` with its default arguments, over the same
  rows, gives the old grids bit for bit;
* no in-process ``Detection`` on the bus any more -- and BlueBoat-
  Perception's dummy, run on the same pictures under convention 1,
  reproduces every old dummy detection (pixel, world, row time, class,
  confidence; uid moved from the 1_000_000 to the 2_000_000 range).
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import numpy as np
import pytest

import capture_perception_golden as cap
import golden_pings as gp

GOLDEN = json.loads(cap.GOLDEN.read_text(encoding="utf-8"))
CHANGED_KEYS = {"schema", "pixel_convention", "detections"}
ADDED_KEYS = {"stream_id", "georef"}
ADDED_ROW_KEYS = {"roll_rad", "pitch_rad", "yaw_rate_rad_s", "attitude_age_s",
                  "port_ping_number", "stbd_ping_number"}
#: The convention-1 formula text of schema <= 4, verbatim.
OLD_FORMULA = ("k = (W/2-1-j) if j < W/2 else (j-W/2); "
               "s = (k+0.5)*bin_pitch_m; "
               "y_local(i,j) = sign * sqrt(max(s^2 - "
               "altitude_m[i]^2, 0)) (+ port, - starboard); "
               "world = (x_i - sin(yaw_i)*y_local, "
               "y_i + cos(yaw_i)*y_local); or use the "
               "precomputed world_x/world_y grids in the "
               "companion _world.npz")
SAME_NUMPY = np.__version__ == GOLDEN["numpy"]


@pytest.fixture(scope="module")
def pings():
    p = gp.make_golden_pings()
    assert gp.pings_sha256(p) == GOLDEN["pings"]["sha"], \
        "golden_pings.py drifted: the generator, not the station, changed"
    return p


@pytest.fixture(scope="module")
def runs(qapp_module, pings):
    return {f"{g}/{k}": cap.run_variant(g, k, pings) for g, k in cap.VARIANTS}


@pytest.fixture(scope="module")
def qapp_module():
    from PySide6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def _variants():
    return [f"{g}/{k}" for g, k in cap.VARIANTS]


@pytest.mark.parametrize("variant", _variants())
def test_pictures_are_bit_identical(runs, variant):
    images, golden = runs[variant], GOLDEN["variants"][variant]
    assert len(images) == len(golden)
    for img, g in zip(images, golden):
        assert img.image_id == g["image_id"]
        assert list(img.intensity_db.shape) == g["shape"]
        assert img.row_geometry == g["row_geometry"]
        assert (img.display is not None) == g["display_ready"]
        for k in ("intensity_db", "row_t", "row_pose"):
            assert gp.sha256_array(getattr(img, k)) == g["sha"][k], (img.image_id, k)
        assert gp.sha256_array(img.to_png8()) == g["sha"]["png8"], img.image_id


@pytest.mark.parametrize("variant", _variants())
def test_metadata_changed_exactly_as_ordered(runs, variant):
    for img, g in zip(runs[variant], GOLDEN["variants"][variant]):
        meta = img.metadata()
        top = {k for k in meta if k != "rows_data"}
        assert top - set(g["metadata"]["keys"]) == ADDED_KEYS
        assert set(g["metadata"]["keys"]) <= top
        for k, sha in g["metadata"]["keys"].items():
            if k not in CHANGED_KEYS:
                assert cap.sha_json(meta[k]) == sha, (img.image_id, k)
        assert meta["schema"] == 5 and g["metadata"]["schema"] == 4
        assert meta["detections"] == []
        pc = dict(meta["pixel_convention"], formula=OLD_FORMULA)
        assert cap.sha_json(pc) == g["metadata"]["keys"]["pixel_convention"]
        assert meta["georef"] == {"convention": 2, "altitude": "bottom_m",
                                  "lever_arm_m": {"x": 0.0, "y_port": 0.285,
                                                  "y_stbd": 0.285}}
        assert meta["stream_id"].startswith("gcs-")
        rows = cap.rows_data_digest(meta["rows_data"])
        assert set(rows) - set(g["metadata"]["rows_data"]) == ADDED_ROW_KEYS
        for k, sha in g["metadata"]["rows_data"].items():
            assert rows[k] == sha, (img.image_id, k)
        # the golden pings carry no attitude and no ping numbers
        r0 = meta["rows_data"][0]
        assert r0["roll_rad"] is None and r0["port_ping_number"] == 0


@pytest.mark.parametrize("variant", _variants())
def test_analysis_payload_changed_only_in_schema_and_detections(runs, variant):
    for img, g in zip(runs[variant], GOLDEN["variants"][variant]):
        payload = json.loads(img.analysis_json(None, None))
        assert payload["detections"] == [] and payload["schema"] == 5
        rest = {k: v for k, v in payload.items() if k != "detections"}
        assert cap.sha_json(dict(rest, schema=4)) == g["analysis"]["sha_without_detections"]
        with_paths = json.loads(img.analysis_json(cap.FAKE_PNG, cap.FAKE_META))
        old = dict(with_paths, schema=4, detections=g["dummy_detections"])
        assert cap.sha_json(old) == g["analysis"]["sha_paths"], img.image_id


@pytest.mark.skipif(not SAME_NUMPY, reason="world hashes are numpy-version specific")
@pytest.mark.parametrize("variant", _variants())
def test_default_arguments_reproduce_the_old_world_grids(runs, variant):
    from blueboat_gcs.core.seabed_imager import waterfall_pixel_to_world
    for img, g in zip(runs[variant], GOLDEN["variants"][variant]):
        W = img.intensity_db.shape[1]
        j = np.arange(W, dtype=np.float64)
        wx, wy = waterfall_pixel_to_world(
            img.row_pose[:, 0:1], img.row_pose[:, 1:2], img.row_pose[:, 2:3],
            np.maximum(img.row_altitude[:, None].astype(np.float64), 0.0),
            img.bin_pitch_m, j[None, :], W)
        assert gp.sha256_array(wx.astype(np.float32)) == g["sha"]["world_x"]
        assert gp.sha256_array(wy.astype(np.float32)) == g["sha"]["world_y"]
        # ...and the grids the picture carries are convention 2 (different;
        # at yaw 0 the lever arm moves world_y only)
        assert (gp.sha256_array(img.world_x) != g["sha"]["world_x"]
                or gp.sha256_array(img.world_y) != g["sha"]["world_y"])


@pytest.mark.parametrize("variant", _variants())
def test_no_in_process_detection_is_emitted_any_more(runs, variant):
    for img in runs[variant]:
        jobs = []
        emitted, published = cap.emitted_detections(img, jobs)
        assert emitted == []
        assert len(published) == 1                       # seabed_analysis still out
        assert json.loads(published[0])["detections"] == []
        assert len(jobs) == 1                            # the image, for Perception


@pytest.mark.parametrize("variant", _variants())
def test_perceptions_dummy_reproduces_the_old_dummy(runs, variant):
    from blueboat_gcs.core import perception_bridge as bridge
    from blueboat_gcs.core import perception_core
    core, why = perception_core.load()
    if core is None:
        pytest.skip(why)
    pipeline = core.Pipeline()
    for img, g in zip(runs[variant], GOLDEN["variants"][variant]):
        env = bridge.envelope(img, None, None)
        del env["metadata"]["georef"]                    # convention 1, as then
        inp = core.SeabedImageInput.from_wire(env, img.intensity_db)
        _, dets = pipeline.run(inp)
        (old,) = g["dummy_detections"]
        (new,) = dets
        assert new["pixel"] == old["pixel"]
        assert new["world_xy"] == old["world"]
        assert new["t_row_s"] == old["t_s"]
        assert (new["class_name"], new["confidence"]) == (old["class_name"],
                                                         old["confidence"])
        (em,) = g["emitted"]
        assert new["uid"] - 2_000_000 == em["uid"] - 1_000_000
        assert (new["t_row_s"], new["world_xy"][0], new["world_xy"][1]) == \
            (em["t"], em["x"], em["y"])
        assert new["extent_m"][0] == em["extent_m"]


@pytest.mark.parametrize("variant", _variants())
def test_live_perception_path_georeferences_with_convention_2(runs, variant):
    """Through the live envelope (georef kept), Perception's world point is
    the station's own convention-2 grid at that pixel."""
    from blueboat_gcs.core import perception_core
    core, why = perception_core.load()
    if core is None:
        pytest.skip(why)
    for img, _record, dets in perception_core.detect(runs[variant], core):
        (d,) = dets
        r, c = d["pixel"]
        assert d["world_xy"] == [float(img.world_x[r, c]), float(img.world_y[r, c])]


def test_saved_files(qapp_module, pings):
    with tempfile.TemporaryDirectory() as d:
        cap.run_variant("square", "fitted", pings, out_dir=Path(d))
        saved = cap.saved_digest(Path(d))
    golden = GOLDEN["saved_square_fitted"]
    assert set(saved) == set(golden)
    for stem, g in golden.items():
        s = saved[stem]
        assert s["png_file"] == g["png_file"], stem
        changed = {k for k in g["npz"] if s["npz"][k] != g["npz"][k]}
        assert set(s["npz"]) == set(g["npz"])
        assert changed <= {"world_x", "world_y"}, (stem, changed)
        if SAME_NUMPY:
            assert changed, stem              # the georef convention moved them
