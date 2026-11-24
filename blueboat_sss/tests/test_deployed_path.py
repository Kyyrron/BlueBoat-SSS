"""The planned path is asked of the running path server, like MCS does.

Until 2026-09-28 the map drew the *newest* ``.deployed/*.yaml``, once, at
the first GPS anchor, re-projected through the designer file's
``geo_anchor``. In a World Builder simulation that drew another mission
entirely (MCS deletes the current mission's deployed file at launch and
rewrites it only after its GPS fit converges, so "newest" was an older
mission), and even the right file landed 24.5 m off (the deployed points
are already in the run's world frame). The GCS now does what the station
does for its preview: read ``path_generation``'s ``trajectory`` parameter,
size ONE ``/path_request`` by that file's duration, draw the answer.
"""

from __future__ import annotations

import os
from types import SimpleNamespace

import pytest
import yaml

from blueboat_gcs.config.settings import PlannedPathConfig, load_config
from blueboat_gcs.core import deployed_path as dp
from blueboat_gcs.core.deployed_path import (PlannedPathSource, is_holding,
                                             read_trajectory_info,
                                             request_times, trajectory_file)

SURVEY_S = 1317.569                       # plage_bathymetry2, 2026-09-07


def _write(path, points, *, loop=False, duration=None, name=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {"format": "blueboat_trajectory/1", "name": name or path.stem,
           "frame": "world", "loop": loop,
           "duration_s": duration if duration is not None
           else float(points[-1][0]),
           "points": [list(p) for p in points]}
    path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    return path


def _survey(path, **kw):
    n = 1200
    pts = [[SURVEY_S * i / (n - 1), float(i), float(2 * i), 0.0]
           for i in range(n)]
    return _write(path, pts, **kw)


# ---- pure helpers -----------------------------------------------------------
def test_trajectory_file_mirrors_path_generation():
    assert trajectory_file("from_yaml:/a/b.yaml") == dp.Path("/a/b.yaml")
    # yaml_path wins, exactly as path_generation.py reads it.
    assert trajectory_file("from_yaml", "/c/d.yaml") == dp.Path("/c/d.yaml")
    assert trajectory_file("from_yaml:/a/b.yaml", "/c/d.yaml") == \
        dp.Path("/c/d.yaml")
    assert trajectory_file("lawnmower") is None
    assert trajectory_file("station_keeping") is None
    assert trajectory_file("from_yaml") is None           # nothing named
    assert trajectory_file("") is None


def test_trajectory_info_reads_the_last_sample_time(tmp_path):
    f = _survey(tmp_path / "survey.yaml", loop=True, duration=999.0)
    info = read_trajectory_info(f)
    # The last sample's t is what yaml_trajectory.py loops on; duration_s
    # is only the fallback.
    assert info.duration_s == pytest.approx(SURVEY_S)
    assert info.loop is True and info.name == "survey"
    assert read_trajectory_info(tmp_path / "absent.yaml") is None
    (tmp_path / "junk.yaml").write_text(": : :\n- [", encoding="utf-8")
    assert read_trajectory_info(tmp_path / "junk.yaml") is None


def test_request_covers_the_whole_mission():
    times = request_times(SURVEY_S, 0.5, 20000)
    assert times[0] == 0.0 and times[-1] == pytest.approx(SURVEY_S)
    assert len(times) == 2636                   # int(1317.569/0.5) + 1
    assert len(request_times(1e6, 0.5, 20000)) == 20000       # capped
    assert request_times(0.0, 0.5, 20000) == (0.0, 0.0)


def test_holding_answer_is_not_a_path():
    assert is_holding(())
    assert is_holding(((0.0, 0.0),) * 50)
    assert not is_holding(((0.0, 0.0), (0.0, 0.5)))


# ---- the source -------------------------------------------------------------
def _engaged(trajectory, yaml_path=""):
    src = PlannedPathSource(dt_s=0.5, default_horizon_s=1000.0,
                            max_points=20000)
    src.service_ready(True)
    assert src.needs_parameters()
    src.on_parameters(trajectory, yaml_path)
    return src


def test_set_path_draws_only_without_a_server():
    src = PlannedPathSource()
    assert src.accept_set_path()                 # nothing to ask
    src.service_ready(True)
    assert src.accept_set_path()                 # parameters not known yet
    src.on_parameters("lawnmower")
    assert not src.accept_set_path()             # we ask it ourselves
    src.service_ready(False)                     # mission ended
    assert src.accept_set_path() and src.poll(0.0) is None


def test_one_request_per_deployment(tmp_path):
    f = tmp_path / ".deployed" / "plage_bathymetry2.yaml"
    src = _engaged(f"from_yaml:{f}")
    # Not deployed yet (MCS deleted it at launch): clear once, ask nothing.
    step = src.poll(0.0)
    assert step.waiting and step.times == ()
    assert src.poll(1.0) is None
    # Deployed: one request over the whole mission.
    _survey(f, loop=True)
    step = src.poll(2.0)
    assert not step.waiting and len(step.times) == 2636
    assert step.times[-1] == pytest.approx(SURVEY_S)
    assert "plage_bathymetry2" in step.label
    pts = tuple((float(i), 0.0) for i in range(2636))
    assert src.on_response(step.key, pts) == pts
    assert src.poll(3.0) is None                 # same deployment: no re-ask
    # Re-deployed (a new mtime): asked again.
    st = os.stat(f)
    os.utime(f, ns=(st.st_atime_ns, st.st_mtime_ns + 10_000_000))
    assert src.poll(4.0) is not None
    # The operator cleared the overlay: asked again.
    src.on_response(src.poll(4.0).key, pts)
    src.reset()
    assert src.poll(5.0) is not None


def test_a_newer_unrelated_deployed_file_is_never_drawn(tmp_path):
    """Cause A: the mission is the one the server names, not the newest."""
    running = _survey(tmp_path / ".deployed" / "plage_bathymetry2.yaml")
    other = _write(tmp_path / ".deployed" / "plage_lawnmower.yaml",
                   [[0.0, 0.0, 0.0, 0.0], [40.0, 20.0, 0.0, 0.0]])
    st = os.stat(running)
    os.utime(other, ns=(st.st_atime_ns, st.st_mtime_ns + 10**9))  # newer
    step = _engaged(f"from_yaml:{running}").poll(0.0)
    assert step.times[-1] == pytest.approx(SURVEY_S)
    assert "plage_bathymetry2" in step.label


def test_built_in_shape_uses_the_default_window():
    src = _engaged("lawnmower")
    step = src.poll(0.0)
    assert step.times[-1] == pytest.approx(1000.0)
    src.on_response(step.key, ((0.0, 0.0), (1.0, 1.0)))
    assert src.poll(1.0) is None


def test_holding_response_clears_and_failure_backs_off(tmp_path):
    f = _survey(tmp_path / "survey.yaml")
    src = _engaged(f"from_yaml:{f}")
    step = src.poll(0.0)
    assert src.on_response(step.key, ((0.0, 0.0),) * 10) == ()
    src.reset()
    src.on_failure(10.0)
    assert src.poll(11.0) is None                # backing off
    assert src.poll(10.0 + dp.RETRY_AFTER_S) is not None


# ---- the ROS glue, against fakes ---------------------------------------------
class _Future:
    def __init__(self, result=None):
        self._result, self._done = result, result is not None

    def done(self):
        return self._done

    def result(self):
        return self._result

    def cancel(self):
        self._done = False


class _Client:
    def __init__(self, ready=True):
        self.ready, self.calls = ready, []

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        fut = _Future()
        self.calls.append((req, fut))
        return fut


def _pose(x, y):
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y)))


@pytest.fixture
def glue(monkeypatch, qapp):
    """A PathListener wired to fake clients; returns (listener, clients,
    emitted paths, status lines)."""
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.ros import path_listener as pl

    class _Req:
        class Request:
            def __init__(self):
                self.names = []
                self.path_request = SimpleNamespace(data=[])

    monkeypatch.setattr(pl, "_MSGS_AVAILABLE", True)
    monkeypatch.setattr(pl, "_SERVICE_AVAILABLE", True)
    monkeypatch.setattr(pl, "Path", object, raising=False)
    monkeypatch.setattr(pl, "RequestPath", _Req, raising=False)
    monkeypatch.setattr(pl, "GetParameters", _Req, raising=False)
    clients = {}

    def create_client(_type, name):
        clients[name] = _Client()
        return clients[name]

    log = SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    node = SimpleNamespace(create_subscription=lambda *a, **k: None,
                           create_client=create_client,
                           create_timer=lambda *a: None,
                           get_logger=lambda: log)
    signals = AppSignals()
    got, status = [], []
    signals.planned_path.connect(got.append)
    signals.status_message.connect(status.append)
    listener = pl.PathListener(node, signals, "/set_path",
                               PlannedPathConfig())
    return listener, clients, got, status


def _answer(client, result):
    req, fut = client.calls[-1]
    fut._result, fut._done = result, True
    return req


def test_listener_asks_the_server_and_ignores_set_path(glue, tmp_path):
    listener, clients, got, status = glue
    f = _survey(tmp_path / ".deployed" / "plage_bathymetry2.yaml", loop=True)
    params = clients["/path_generation/get_parameters"]
    server = clients["/path_request"]
    # A truncated /set_path draws while the server's mission is unknown...
    listener._on_path(SimpleNamespace(poses=[_pose(0, 0), _pose(1, 1)]))
    assert len(got) == 1
    listener._tick()                                   # asks the parameters
    req = _answer(params, SimpleNamespace(values=[
        SimpleNamespace(string_value=f"from_yaml:{f}"),
        SimpleNamespace(string_value="")]))
    assert req.names == ["trajectory", "yaml_path"]
    listener._tick()                                   # harvests them
    listener._tick()                                   # asks the path
    req = _answer(server, SimpleNamespace(path=SimpleNamespace(
        poses=[_pose(float(i), 2.0 * i) for i in range(2636)])))
    assert len(req.path_request.data) == 2636
    assert req.path_request.data[-1] == pytest.approx(SURVEY_S)
    listener._tick()                                   # harvests the path
    assert len(got[-1].points) == 2636 and got[-1].points[5] == (5.0, 10.0)
    assert any("plage_bathymetry2" in s for s in status)
    # ...and is ignored from then on: the whole mission stays drawn.
    listener._on_path(SimpleNamespace(poses=[_pose(0, 0), _pose(9, 9)]))
    assert len(got[-1].points) == 2636
    n_calls = len(server.calls)
    listener._tick()
    assert len(server.calls) == n_calls                # asked once, not polled
    # The server goes away: /set_path draws again.
    server.ready = False
    listener._tick()
    listener._on_path(SimpleNamespace(poses=[_pose(0, 0), _pose(9, 9)]))
    assert got[-1].points == ((0, 0), (9, 9))


def test_listener_clears_while_waiting_for_deployment(glue, tmp_path):
    listener, clients, got, status = glue
    listener._on_path(SimpleNamespace(poses=[_pose(0, 0), _pose(1, 1)]))
    listener._tick()
    _answer(clients["/path_generation/get_parameters"],
            SimpleNamespace(values=[SimpleNamespace(
                string_value=f"from_yaml:{tmp_path / 'absent.yaml'}")]))
    listener._tick()
    listener._tick()
    assert got[-1].points == ()                        # old path cleared
    assert any("waiting" in s for s in status)
    assert clients["/path_request"].calls == []        # nothing to ask yet


def test_old_config_keys_still_load(tmp_path):
    cfg = tmp_path / "override.yaml"
    cfg.write_text("planned_path: {prefer_deployed: false, name: x}\n",
                   encoding="utf-8")
    assert load_config(cfg).planned_path.from_service is True
