"""AI detections listener — **PLACEHOLDER / INTEGRATION POINT**.

The detection node (YOLO + classical proposer, see the project synthesis)
does not exist yet. The GUI side (DetectionLayer markers, per-class
summary in the left panel) is fully implemented and exercised by the
simulator; this listener defines the contract the future node should
publish, so that connecting it is editing one adapter function.

Expected interface:

    Topic   : config ``topics.detections`` (default /sss_ai/detections)
    Type    : vision_msgs/Detection2DArray
              * one Detection2D per object
              * results[0].hypothesis.class_id  -> class name (string)
              * results[0].hypothesis.score     -> confidence [0, 1]
              * bbox.center.position.x / .y     -> object position in the
                *local odom frame* [m] (the detector must project image
                detections through the mosaic geometry before publishing)
              * bbox.size_x                     -> object extent [m]
              * detections[i].id                -> stable uid (string int);
                republishing the same id after a revisit *updates* the
                marker instead of duplicating it.
    Rate    : event-driven (per detector inference, ~0.5–2 Hz bursts)

vision_msgs is a standard ROS package but may be absent on a bare
basestation; the import is guarded so the app degrades gracefully.
If the detection repo settles on a custom message instead, replace the
import and ``_msg_to_detections`` only.
"""

from __future__ import annotations

from typing import List

from rclpy.node import Node

from ..core.signals import AppSignals
from ..models.detection import Detection

try:  # pragma: no cover - environment dependent
    from vision_msgs.msg import Detection2DArray
    VISION_MSGS_AVAILABLE = True
except ImportError:  # pragma: no cover
    VISION_MSGS_AVAILABLE = False


class DetectionsListener:
    """Forwards AI detections to the GUI."""

    def __init__(self, node: Node, signals: AppSignals, topic: str) -> None:
        self._signals = signals
        if not VISION_MSGS_AVAILABLE:
            signals.status_message.emit(
                "vision_msgs not found — AI detections stream disabled "
                "(placeholder, see ros/detections_listener.py).")
            return
        node.create_subscription(Detection2DArray, topic, self._on_msg, 10)

    def _on_msg(self, msg: "Detection2DArray") -> None:
        for det in self._msg_to_detections(msg):
            self._signals.detection.emit(det)

    @staticmethod
    def _msg_to_detections(msg: "Detection2DArray") -> List[Detection]:
        """<-- EDIT HERE when connecting the real detection node."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out: List[Detection] = []
        for i, d in enumerate(msg.detections):
            if not d.results:
                continue
            hyp = d.results[0].hypothesis
            try:
                uid = int(d.id)
            except (ValueError, TypeError):
                uid = hash(d.id or f"det-{t}-{i}") & 0x7FFFFFFF
            out.append(Detection(
                uid=uid,
                t=t,
                x=float(d.bbox.center.position.x),
                y=float(d.bbox.center.position.y),
                class_name=str(hyp.class_id) or "object",
                confidence=float(hyp.score),
                extent_m=max(float(d.bbox.size_x), 0.5),
            ))
        return out
