"""The simulator's run record, kept for a session's ``metadata.json``.

Why (BlueBoat-SSS-Sim review, 2026-11-11). A recorded simulation session did
not say what had produced it: which sonar renderer, which profile, which
world, which seeds, which build of the simulator. The simulated sonar node
now publishes that as JSON on a latched topic (``TOPIC``; schema
``blueboat_sss_sim_run/1`` in BlueBoat-SSS-Sim ``docs/topics.md``), at its
start-up, at every ping enable and at stop. ``ros/ros_manager`` feeds it here;
``core/recording_session`` writes :func:`latest` under ``"simulation"``.

A real-robot graph has no such publisher: nothing is ever fed and the key is
``null``. A record left over from a simulator that has since gone (the GCS
outlives its launches) is not reported either: :func:`latest` asks the probe
``ros_manager`` installs whether the topic still has a publisher.

Qt-free and ROS-free (``tests/test_sim_run_info.py``).
"""

from __future__ import annotations

import json
import threading
from typing import Callable, Optional

#: the simulated sonar's latched run record (RELIABLE / TRANSIENT_LOCAL / 1)
TOPIC = "/side_scan_sonar/ground_truth/run_info"

_lock = threading.Lock()
_doc: Optional[dict] = None
_alive: Optional[Callable[[], bool]] = None


def feed(text: str) -> bool:
    """One message of ``TOPIC``. A payload that is not a JSON object is
    ignored (False); the last good record stays."""
    global _doc
    try:
        doc = json.loads(text)
    except (TypeError, ValueError):
        return False
    if not isinstance(doc, dict):
        return False
    with _lock:
        _doc = doc
    return True


def set_alive_probe(probe: Optional[Callable[[], bool]]) -> None:
    """``probe()`` -> does ``TOPIC`` have a publisher now? (None: no check.)"""
    global _alive
    with _lock:
        _alive = probe


def latest() -> Optional[dict]:
    """The simulator's current run record, or None: none received (a real
    run), or its publisher is gone."""
    with _lock:
        doc, probe = _doc, _alive
    if doc is None:
        return None
    if probe is not None:
        try:
            if not probe():
                return None
        except Exception:                    # pragma: no cover - metadata only
            return None
    return dict(doc)


def clear() -> None:
    global _doc
    with _lock:
        _doc = None
