"""What the GCS sends BlueBoat-Perception for every seabed image (ROS-free).

Since 2026-10-16 the station has **one detection source**: Perception. Every
seabed image -- recording or not -- goes out as three samples sharing the
key ``"<stream_id>/<image_id>"`` (contract:
``BlueBoat-Perception/docs/topics.md`` §1-§2):

* ``topics.seabed_image`` -- ``sensor_msgs/Image`` 32FC1: the raw float dB,
  exactly the array saved as ``_world.npz:intensity_db``;
* ``topics.seabed_image_png`` -- ``sensor_msgs/Image`` mono8: exactly
  ``SeabedImage.to_png8()`` (= the session PNG);
* ``topics.seabed_image_meta`` -- ``std_msgs/String``: the JSON envelope
  below, whose ``metadata`` is exactly ``SeabedImage.metadata()``.

The key rides in ``Image.header.frame_id`` (an image key, not a TF frame:
no module reads TF); ``header.stamp`` is the newest row time
(``t_start_s``). Building the parts is pure (this module); publishing is
``ros/ros_manager.publish_seabed_image``, from a single-thread pool so the
GUI thread never encodes and the order is kept.
"""

from __future__ import annotations

import json
import math
from typing import Optional, Tuple

import numpy as np

ENVELOPE_SCHEMA = "blueboat_sss/seabed_image/1"


def image_key(stream_id: str, image_id: int) -> str:
    return f"{stream_id}/{int(image_id)}"


def stamp(t_s: float) -> Tuple[int, int]:
    """(sec, nanosec) of a time in seconds; (0, 0) when not positive."""
    if not math.isfinite(t_s) or t_s <= 0.0:
        return 0, 0
    sec = int(math.floor(t_s))
    nsec = int(round((t_s - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec, nsec = sec + 1, nsec - 1_000_000_000
    return sec, nsec


def envelope(image, png_path: Optional[str], metadata_path: Optional[str]) -> dict:
    meta = image.metadata()
    sec, nsec = stamp(float(meta["t_start_s"]))
    return {"schema": ENVELOPE_SCHEMA,
            "key": image_key(image.stream_id, image.image_id),
            "stream_id": image.stream_id, "image_id": int(image.image_id),
            "stamp": {"sec": sec, "nanosec": nsec},
            "png_path": png_path, "metadata_path": metadata_path,
            "metadata": meta}


def envelope_json(image, png_path: Optional[str], metadata_path: Optional[str]) -> str:
    return json.dumps(envelope(image, png_path, metadata_path))


def image_arrays(image) -> Tuple[np.ndarray, np.ndarray]:
    """(raw dB float32, PNG uint8), both C-contiguous [H, W]."""
    db = np.ascontiguousarray(image.intensity_db, dtype=np.float32)
    png = np.ascontiguousarray(image.to_png8(), dtype=np.uint8)
    return db, png


def publish_parts(image, png_path: Optional[str], metadata_path: Optional[str]):
    """Everything ``publish_seabed_image`` needs: (key, stamp, db, png, json)."""
    db, png = image_arrays(image)
    env = envelope(image, png_path, metadata_path)
    return (env["key"], (env["stamp"]["sec"], env["stamp"]["nanosec"]),
            db, png, json.dumps(env))
