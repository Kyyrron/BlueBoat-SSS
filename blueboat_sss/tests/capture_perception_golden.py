"""Capture ``tests/fixtures/perception_golden.json`` -- the station's seabed
output as it stood BEFORE the BlueBoat-Perception changes (2026-10-16).

Run once, from ``blueboat_sss/``, on the unmodified code:

    QT_QPA_PLATFORM=offscreen /usr/bin/python3 tests/capture_perception_golden.py

It refuses to overwrite an existing golden: the file is a record of what
the code did on that date, not something to regenerate after a change
(``--force`` exists for a deliberate re-capture, which must be reported).

What is captured, for each of four variants (square / ping row geometry ×
a fitted display model / the live main-window wiring, where the waterfall
service feeds a shared model and the imager reads it), over the
deterministic pings of ``golden_pings.py``:

* every image's raw dB, PNG, world grids (SHA-256 of the arrays);
* its ``metadata()`` (SHA-256 of the canonical JSON, whole and per key;
  ``rows_data`` per row field);
* its ``/sss_ai/seabed_analysis`` payload (whole, and without
  ``detections``), with no paths and with fixed fake paths;
* the in-process dummy's detection dicts, and the ``Detection`` stream
  ``MainWindow._on_seabed_image`` emitted for the map;
* for one variant, the bytes ``SeabedImage.save`` wrote (PNG, JSON) and
  the ``_world.npz`` arrays by key.

``test_perception_golden.py`` replays the same pings through the current
code and compares (see its docstring for what may differ and why).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, List

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

HERE = Path(__file__).resolve().parent
PKG_PARENT = HERE.parent
for p in (str(PKG_PARENT), str(HERE)):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np                                         # noqa: E402

import golden_pings as gp                                  # noqa: E402

GOLDEN = HERE / "fixtures" / "perception_golden.json"
ROWS, STRIDE = 64, 32
VARIANTS = [("square", "fitted"), ("square", "live"),
            ("ping", "fitted"), ("ping", "live")]
FAKE_PNG, FAKE_META = "/golden/seabed.png", "/golden/metadata/seabed.json"


def canonical(obj) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"),
                      allow_nan=True)


def sha_text(s: str) -> str:
    return hashlib.sha256(s.encode("utf-8")).hexdigest()


def sha_json(obj) -> str:
    return sha_text(canonical(obj))


def config(geometry: str):
    from blueboat_gcs.config.settings import load_config
    cfg = load_config()
    cfg.seabed.rows, cfg.seabed.stride = ROWS, STRIDE
    cfg.seabed.row_geometry = geometry
    # The value the golden was captured with (the shipped default until
    # 2026-10-16, when it became 2.0 s -- PREPROCESSING_STUDY decision 6.3).
    # Pinned so the golden keeps proving the imaging code unchanged.
    cfg.along_track.window_s = 5.0
    return cfg


def run_variant(geometry: str, model_kind: str, pings, out_dir=None):
    """Feed the pings the way the station does; returns the images.

    ``fitted``: one model fitted over the whole log, injected (replay /
    Run AI / Save pictures). ``live``: the main window's wiring -- a
    shared ``DisplayModel`` fed by the ``WaterfallService``, the imager
    reading it (``main_window.py`` construction and
    ``_on_sonar_ping_data``)."""
    from blueboat_gcs.core.display_model import DisplayModel
    from blueboat_gcs.core.seabed_imager import SeabedImager
    cfg = config(geometry)
    images: List = []
    if model_kind == "fitted":
        model = DisplayModel.fit(cfg, pings)
        imager = SeabedImager(cfg, model=model)
        feeders = [imager.on_sonar_ping]
    elif model_kind == "live":
        from blueboat_gcs.core.waterfall_service import WaterfallService
        model = DisplayModel(cfg)
        wf = WaterfallService(cfg, model)
        imager = SeabedImager(cfg, model=model)
        feeders = [wf.on_sonar_ping, imager.on_sonar_ping]
    else:
        raise ValueError(model_kind)
    imager.image_ready.connect(images.append)
    if out_dir is not None:
        imager.image_ready.connect(lambda img: img.save(out_dir))
    pending = sorted(gp.golden_breaks())
    for ping in pings:
        while pending and ping.t > pending[0]:
            pending.pop(0)
            imager.break_window()
            if model_kind == "live":
                wf.break_row()
        for feed in feeders:
            feed(ping)
    imager.flush()
    return images


def emitted_detections(image, jobs: List = None) -> List[dict]:
    """What ``MainWindow._on_seabed_image`` puts on the bus for the map,
    called on a stub ``self`` (no window, no ROS). ``jobs`` collects the
    Perception publish jobs queued since 2026-10-16 (the stub's pool only
    records them; the code the golden was captured from never used it)."""
    from blueboat_gcs.gui.main_window import MainWindow
    published: List[str] = []
    emitted: List = []
    jobs = [] if jobs is None else jobs
    stub = SimpleNamespace(
        _acquisition=SimpleNamespace(_ros=SimpleNamespace(
            publish_seabed_analysis=published.append)),
        console=SimpleNamespace(append_line=lambda *a, **k: None),
        _signals=SimpleNamespace(detection=SimpleNamespace(emit=emitted.append)),
        _perception_pool=SimpleNamespace(start=jobs.append))
    MainWindow._on_seabed_image(stub, image)
    out = [{"uid": d.uid, "t": d.t, "x": d.x, "y": d.y,
            "class_name": d.class_name, "confidence": d.confidence,
            "extent_m": d.extent_m} for d in emitted]
    return out, published


def rows_data_digest(rows_data: List[dict]) -> Dict[str, str]:
    keys = sorted(rows_data[0]) if rows_data else []
    return {k: sha_json([r.get(k) for r in rows_data]) for k in keys}


def image_digest(image) -> dict:
    meta = image.metadata()
    emitted, published = emitted_detections(image)
    payload = json.loads(image.analysis_json(None, None))
    payload_paths = json.loads(image.analysis_json(FAKE_PNG, FAKE_META))
    return {
        "image_id": int(image.image_id),
        "shape": list(image.intensity_db.shape),
        "row_geometry": image.row_geometry,
        "display_ready": image.display is not None,
        "sha": {"intensity_db": gp.sha256_array(image.intensity_db),
                "png8": gp.sha256_array(image.to_png8()),
                "world_x": gp.sha256_array(image.world_x),
                "world_y": gp.sha256_array(image.world_y),
                "row_t": gp.sha256_array(image.row_t),
                "row_pose": gp.sha256_array(image.row_pose)},
        "metadata": {"sha": sha_json(meta),
                     "keys": {k: sha_json(v) for k, v in meta.items()
                              if k != "rows_data"},
                     "rows_data": rows_data_digest(meta["rows_data"]),
                     "schema": meta["schema"]},
        "analysis": {"sha": sha_json(payload),
                     "sha_paths": sha_json(payload_paths),
                     "sha_without_detections": sha_json(
                         {k: v for k, v in payload.items() if k != "detections"}),
                     "published_sha": [sha_text(s) for s in published]},
        "dummy_detections": meta["detections"],
        "emitted": emitted,
    }


def saved_digest(out_dir: Path) -> dict:
    out = {}
    for png in sorted(out_dir.glob("seabed_*.png")):
        stem = png.stem
        meta = out_dir / "metadata" / f"{stem}.json"
        npz = np.load(out_dir / "metadata" / f"{stem}_world.npz")
        out[stem] = {
            "png_file": hashlib.sha256(png.read_bytes()).hexdigest(),
            "json_file": hashlib.sha256(meta.read_bytes()).hexdigest(),
            "npz": {k: gp.sha256_array(npz[k]) for k in sorted(npz.files)},
        }
    return out


def capture() -> dict:
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])  # noqa: F841
    import PySide6
    pings = gp.make_golden_pings()
    golden = {
        "captured": "2026-10-16, before the BlueBoat-Perception changes",
        "code": "BlueBoat-SSS dev at 994a74b + the baseline test repair only",
        "numpy": np.__version__, "pyside6": PySide6.__version__,
        "rows": ROWS, "stride": STRIDE,
        "pings": {"count": len(pings), "sha": gp.pings_sha256(pings)},
        "variants": {},
    }
    for geometry, kind in VARIANTS:
        images = run_variant(geometry, kind, pings)
        golden["variants"][f"{geometry}/{kind}"] = [image_digest(i) for i in images]
    with tempfile.TemporaryDirectory() as d:
        run_variant("square", "fitted", pings, out_dir=Path(d))
        golden["saved_square_fitted"] = saved_digest(Path(d))
    return golden


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--force", action="store_true",
                    help="overwrite an existing golden (report it)")
    ap.add_argument("--out", type=Path, default=GOLDEN)
    args = ap.parse_args()
    if args.out.exists() and not args.force:
        print(f"refusing to overwrite {args.out} (use --force deliberately)")
        return 1
    golden = capture()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(golden, indent=1, sort_keys=True) + "\n",
                        encoding="utf-8")
    n = {k: len(v) for k, v in golden["variants"].items()}
    print(f"wrote {args.out}: {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
