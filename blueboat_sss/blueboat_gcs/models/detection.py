"""AI detection and USBL pinger models (GUI-side representations).

Both features are *placeholders* on the ROS side (see
``ros/detections_listener.py`` and ``ros/pinger_listener.py``) but the
GUI side is fully implemented against these models: once the real topics
exist, only the listener adapters need editing.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True, slots=True)
class Detection:
    """One AI detection projected into the local odom frame.

    Attributes
    ----------
    uid:
        Unique id (used to update/replace a detection after revisit).
    t:
        Detection timestamp [s].
    x, y:
        Estimated object position in the local odom frame [m].
    class_name:
        Detector class label (e.g. "tire", "block", "chain").
    confidence:
        Detector score in [0, 1].
    extent_m:
        Optional characteristic size for drawing the marker circle.
    """

    uid: int
    t: float
    x: float
    y: float
    class_name: str
    confidence: float
    extent_m: float = 1.0


@dataclass(frozen=True, slots=True)
class PingerFix:
    """Last known USBL pinger position in the local odom frame."""

    t: float
    x: float
    y: float
    accuracy_m: Optional[float] = None
