"""Planned-path dedupe: verbatim 1 Hz re-sends must cost nothing.

path_publisher.py re-publishes the same ~4700-pose Path every second;
the listener fingerprint and the layer early-out together turn that
into a no-op instead of a tuple rebuild + full map repaint per second.
"""

from __future__ import annotations

from types import SimpleNamespace

from PySide6.QtWidgets import QGraphicsScene

from blueboat_gcs.gui.map_layers import PlannedPathLayer
from blueboat_gcs.ros.path_listener import path_fingerprint


def _pose(x: float, y: float):
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y)))


def test_fingerprint_distinguishes_paths():
    a = [_pose(0.0, 0.0), _pose(1.0, 1.0), _pose(2.0, 0.0)]
    assert path_fingerprint(a) == path_fingerprint(list(a))
    assert path_fingerprint(a) != path_fingerprint(a[:-1])       # length
    assert path_fingerprint(a) != path_fingerprint(
        [_pose(0.0, 0.0), _pose(1.0, 1.0), _pose(2.0, 5.0)])     # endpoint
    assert path_fingerprint([]) == (0,)


def test_layer_ignores_identical_points(qapp):
    scene = QGraphicsScene()
    layer = PlannedPathLayer(scene)
    calls = []
    original = layer._item.setPath
    layer._item.setPath = lambda p: (calls.append(1), original(p))

    pts = ((0.0, 0.0), (10.0, 0.0), (10.0, 5.0))
    layer.set_path(pts)
    layer.set_path(tuple(pts))            # identical re-send
    assert len(calls) == 1
    layer.set_path(pts + ((0.0, 5.0),))   # genuinely new
    assert len(calls) == 2
    layer.clear()                         # clear() itself repaints empty
    assert len(calls) == 3
    layer.set_path(pts)                   # after clear it must redraw
    assert len(calls) == 4
