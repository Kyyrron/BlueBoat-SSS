"""AI detections listener -- the station's ONE detection source.

Since 2026-10-16 every detection the station draws (map markers, the
waterfall overlay, the left panel's per-class summary, the session's
``detections/detections.csv``) arrives here, from BlueBoat-Perception
(today its dummy detector; a YOLO model later). The GCS no longer runs a
detector of its own, so a detection can never be drawn twice. (The no-ROS
``--sim`` bench simulator still draws its own synthetic markers: it has no
ROS graph and no Perception.)

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
              * detections[i].header.stamp      -> the detection's own row
                time when non-zero (places the waterfall marker on the
                ping line the object was seen on); else the array stamp.
    Rate    : event-driven (per detector inference, ~0.5–2 Hz bursts)

vision_msgs is a standard ROS package but may be absent from a bare
ROS install; the import is guarded so the app degrades gracefully.
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
        """The producer is BlueBoat-Perception (``/sss_ai/detections``,
        its ``docs/topics.md`` §3) -- the station's one detection source."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out: List[Detection] = []
        for i, d in enumerate(msg.detections):
            if not d.results:
                continue
            hyp = d.results[0].hypothesis
            # Per-detection row time (additive, 2026-10-16): a zero stamp
            # -- every producer before it -- keeps the array's stamp.
            ts = d.header.stamp
            t_det = (ts.sec + ts.nanosec * 1e-9) if (ts.sec or ts.nanosec) else t
            try:
                uid = int(d.id)
            except (ValueError, TypeError):
                uid = hash(d.id or f"det-{t}-{i}") & 0x7FFFFFFF
            out.append(Detection(
                uid=uid,
                t=t_det,
                x=float(d.bbox.center.position.x),
                y=float(d.bbox.center.position.y),
                class_name=str(hyp.class_id) or "object",
                confidence=float(hyp.score),
                extent_m=max(float(d.bbox.size_x), 0.5),
            ))
        return out
