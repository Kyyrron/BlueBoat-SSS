"""BlueBoat-Perception's core, in-process -- the replay window's "Run AI".

Since 2026-10-16 the station has one detection source, Perception. Live,
its node returns detections on ``/sss_ai/detections``. A replay needs no
ROS graph, so "Run AI" runs **the same core** in-process: each picture
goes through the very envelope the live path publishes
(``core/perception_bridge.py``) into ``SeabedImageInput.from_wire`` and
``Pipeline.run`` -- one detector implementation, no second dummy here.

The import is guarded (ROS-free either way): the installed package first
(a sourced ``~/ros2_ws`` after ``colcon build --packages-select
blueboat_perception``), else the superproject's source checkout
``BlueBoat-Perception/blueboat_perception``. Without either, Run AI says so
and draws nothing. The GCS only *uses* Perception; Perception never
imports the GCS (CM-3).
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

from . import perception_bridge as bridge

#: The superproject checkout's package root (the colcon package dir).
SIBLING = Path(__file__).resolve().parents[4] / "BlueBoat-Perception" / "blueboat_perception"


def load() -> Tuple[Optional[object], str]:
    """(the ``blueboat_perception.core`` module, where it came from) or
    (None, why not)."""
    try:
        return importlib.import_module("blueboat_perception.core"), "installed"
    except ImportError:
        pass
    if (SIBLING / "blueboat_perception" / "core" / "__init__.py").is_file():
        if str(SIBLING) not in sys.path:
            sys.path.insert(0, str(SIBLING))
        try:
            return importlib.import_module("blueboat_perception.core"), str(SIBLING)
        except ImportError as exc:                 # pragma: no cover
            return None, f"BlueBoat-Perception import failed: {exc}"
    return None, ("BlueBoat-Perception is not available: build it "
                  "(colcon build --packages-select blueboat_perception) and "
                  "source ~/ros2_ws, or keep the superproject checkout")


def detect(images: Sequence, core=None) -> List[Tuple[object, dict, Optional[list]]]:
    """[(image, record, detections)] for GCS ``SeabedImage``s, through the
    live wire format. ``detections`` is None for an image without pixels."""
    if core is None:
        core, why = load()
        if core is None:
            raise ImportError(why)
    pipeline = core.Pipeline()
    out = []
    for image in images:
        env = bridge.envelope(image, getattr(image, "_png_path", None),
                              getattr(image, "_metadata_path", None))
        db, _png = bridge.image_arrays(image)
        img = core.SeabedImageInput.from_wire(env, db)
        record, dets = pipeline.run(img)
        out.append((image, record, dets))
    return out
