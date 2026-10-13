"""Planned-path dedupe: verbatim re-sends must cost nothing — and a
changed path must never be mistaken for one.

path_publisher.py re-publishes the same ~4700-pose Path every
refresh_period (5 s); the listener compare and the layer early-out
together turn that into a no-op instead of a full map repaint. Until
2026-09-15 the listener compared (len, first, last) only, so a
re-deployed mission with the same endpoints and pose count never
reached the map (project_review.md §3.3); it now compares the full
point tuple, and ``reset()`` lets the operator's "clear overlays" get
the path back on the next re-send.
"""

from __future__ import annotations

from types import SimpleNamespace

from PySide6.QtWidgets import QGraphicsScene

from blueboat_gcs.gui.map_layers import PlannedPathLayer
from blueboat_gcs.ros.path_listener import PathListener, path_fingerprint


def _pose(x: float, y: float):
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y)))


def test_fingerprint_distinguishes_paths():
    a = [_pose(0.0, 0.0), _pose(1.0, 1.0), _pose(2.0, 0.0)]
    assert path_fingerprint(a) == path_fingerprint(list(a))
    assert path_fingerprint(a) != path_fingerprint(a[:-1])       # length
    assert path_fingerprint(a) != path_fingerprint(
        [_pose(0.0, 0.0), _pose(1.0, 1.0), _pose(2.0, 5.0)])     # endpoint
    # Same length, same endpoints, different interior: a different path.
    assert path_fingerprint(a) != path_fingerprint(
        [_pose(0.0, 0.0), _pose(1.0, -1.0), _pose(2.0, 0.0)])
    assert path_fingerprint([]) == ()


def _listener(qapp):
    from blueboat_gcs.core.signals import AppSignals
    signals = AppSignals()
    got = []
    signals.planned_path.connect(got.append)
    node = SimpleNamespace(create_subscription=lambda *a, **k: None,
                           get_logger=lambda: SimpleNamespace(info=lambda *a: None))
    return PathListener(node, signals, "/set_path"), got


def test_listener_forwards_a_changed_interior_and_drops_verbatim_resends(qapp):
    listener, got = _listener(qapp)
    a = [_pose(0.0, 0.0), _pose(1.0, 1.0), _pose(2.0, 0.0)]
    listener._on_path(SimpleNamespace(poses=a))
    listener._on_path(SimpleNamespace(poses=list(a)))          # verbatim re-send
    assert len(got) == 1
    listener._on_path(SimpleNamespace(
        poses=[_pose(0.0, 0.0), _pose(1.0, -1.0), _pose(2.0, 0.0)]))
    assert len(got) == 2 and got[-1].points[1] == (1.0, -1.0)


def test_reset_lets_the_next_resend_through(qapp):
    listener, got = _listener(qapp)
    a = [_pose(0.0, 0.0), _pose(3.0, 4.0)]
    listener._on_path(SimpleNamespace(poses=a))
    listener._on_path(SimpleNamespace(poses=a))
    assert len(got) == 1
    listener.reset()                                          # overlay cleared
    listener._on_path(SimpleNamespace(poses=a))
    assert len(got) == 2


def test_full_compare_is_cheap_at_mission_size(qapp):
    import time
    listener, got = _listener(qapp)
    poses = [_pose(0.01 * i, 0.5 * (i % 7)) for i in range(4700)]
    listener._on_path(SimpleNamespace(poses=poses))
    t0 = time.perf_counter()
    for _ in range(10):
        listener._on_path(SimpleNamespace(poses=poses))       # 10 verbatim re-sends
    per_call = (time.perf_counter() - t0) / 10
    assert len(got) == 1
    assert per_call < 0.05, f"{per_call * 1e3:.1f} ms per re-send"


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
