"""The toolbar's Sonar selector: Simulation / Real sonars (2026-09-28).

Everything runs on the operator laptop. **Real** makes the pipeline launch
also start ``sss_node.py``; **Simulation** leaves ``/side_scan_sonar`` to the
simulator. Two ``/side_scan_sonar`` nodes must never run at once, a driver an
operator started by hand is never swept, and the volatile ping enable is only
sent once the sonar node can hear it. No ROS is needed: the process table is
faked with a ``/proc``-shaped directory and the ROS manager with a stub.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from blueboat_gcs.config.settings import AcquisitionConfig, PipelineConfig
from blueboat_gcs.core.signals import AppSignals
from blueboat_gcs.ros import sonar_source as ss
from blueboat_gcs.ros.pipeline_launcher import PipelineLauncher, PipelineState

BASE = ["ros2", "launch", "blueboat_sss", "SSS_processing_launch.py"]
PY = "/usr/bin/python3"
DRIVER = "/home/op/ros2_ws/install/blueboat_sss/lib/blueboat_sss/sss_node.py"
SIM = "/home/op/ros2_ws/install/blueboat_sss_sim/lib/blueboat_sss_sim/sss_sim_node"


def _proc(root: Path, pid: int, argv: list, env: dict) -> None:
    d = root / str(pid)
    d.mkdir(parents=True)
    (d / "cmdline").write_bytes(b"\0".join(a.encode() for a in argv) + b"\0")
    (d / "environ").write_bytes(
        b"\0".join(f"{k}={v}".encode() for k, v in env.items()) + b"\0")


# ---- pure functions ----------------------------------------------------------
def test_unknown_source_is_simulation():
    assert ss.normalise("REAL") == ss.REAL
    assert ss.normalise("gazebo") == ss.SIMULATION
    assert ss.normalise(None) == ss.SIMULATION


def test_launch_command_adds_the_driver_for_real_only():
    args = ["with_acquisition:=True"]
    assert ss.launch_command(BASE, ss.SIMULATION, args) == BASE
    assert ss.launch_command(BASE, ss.REAL, args) == BASE + args
    assert ss.launch_command(BASE + args, ss.REAL, args) == BASE + args


def test_classify_nodes_not_editors():
    env = {"HOME": "/home/op"}
    assert ss.classify(1, [PY, DRIVER, "--ros-args"], env.keys()).kind == "driver"
    assert ss.classify(2, [PY, "/opt/ros/jazzy/bin/ros2", "run", "blueboat_sss",
                           "sss_node.py"], ()).kind == "driver"
    assert ss.classify(3, [PY, SIM, "--ros-args"], ()).kind == "simulator"
    assert ss.classify(4, ["vim", DRIVER], ()) is None
    assert ss.classify(5, [PY, "/opt/ros/jazzy/bin/ros2", "launch",
                           "blueboat_sss_sim", "sss_sim_launch.py"], ()) is None


def test_conflict_rules():
    sim = ss.SonarProcess(pid=10, kind="simulator", owned=False)
    ours = ss.SonarProcess(pid=11, kind="driver", owned=True)
    hand = ss.SonarProcess(pid=12, kind="driver", owned=False)
    assert ss.driver_conflict(ss.SIMULATION, [sim, hand]) is None
    assert ss.driver_conflict(ss.REAL, []) is None
    assert ss.driver_conflict(ss.REAL, [ours]) is None     # a leftover, swept
    assert "10" in ss.driver_conflict(ss.REAL, [sim])
    assert "12" in ss.driver_conflict(ss.REAL, [hand])


def test_scan_and_ownership_from_the_process_table(tmp_path):
    _proc(tmp_path, 100, [PY, DRIVER, "--ros-args"], {ss.OWNER_ENV: "1"})
    _proc(tmp_path, 101, [PY, DRIVER, "--ros-args"], {"HOME": "/home/op"})
    _proc(tmp_path, 102, [PY, SIM, "--ros-args"], {})
    _proc(tmp_path, 103, ["bash"], {ss.OWNER_ENV: "1"})
    found = {p.pid: p for p in ss.scan_processes(tmp_path)}
    assert set(found) == {100, 101, 102}
    assert found[100].owned and not found[101].owned
    assert found[102].kind == "simulator"
    assert ss.owned_pids([100, 101, 999], tmp_path) == [100]


# ---- the launcher ------------------------------------------------------------
class StubRos:
    def __init__(self, subscribers=None) -> None:
        self.enables: list = []
        self.subscribers = subscribers

    def publish_ping_enable(self, enable: bool) -> None:
        self.enables.append(enable)

    def publish_svlog_enable(self, enable: bool) -> None:
        pass

    def ping_enable_subscribers(self):
        return self.subscribers


def _launcher(ros, source, **cfg):
    signals = AppSignals()
    lch = PipelineLauncher(
        PipelineConfig(leftover_process_patterns=[], owned_process_patterns=[],
                       **cfg),
        ros, signals, acquisition=AcquisitionConfig(), sonar_source=source)
    return lch, signals


def test_real_launch_starts_the_driver_unless_another_sonar_runs(
        qapp, monkeypatch):
    lch, signals = _launcher(StubRos(), ss.REAL)
    monkeypatch.setattr(ss, "scan_processes", lambda: [])
    assert lch._command()[-1] == "with_acquisition:=True"

    messages: list = []
    signals.status_message.connect(messages.append)
    monkeypatch.setattr(ss, "scan_processes", lambda: [
        ss.SonarProcess(pid=7, kind="simulator", owned=False)])
    assert lch._command() == list(PipelineConfig().launch_command)
    assert any("sss_sim_node (pid 7)" in m for m in messages)

    sim, _ = _launcher(StubRos(), ss.SIMULATION)
    assert sim._command() == list(PipelineConfig().launch_command)


def test_enable_waits_for_the_sonar_node_to_subscribe(qapp):
    ros = StubRos(subscribers=0)
    lch, signals = _launcher(ros, ss.SIMULATION, ping_enable_wait_s=60.0)
    lch._set_state(PipelineState.RUNNING)
    lch.enable_pinging()
    assert ros.enables == [], "sent before anything could hear it"
    ros.subscribers = 1
    lch._poll_ping_enable()
    assert ros.enables == [True]
    lch._poll_ping_enable()
    assert ros.enables == [True], "sent twice"


def test_enable_is_sent_anyway_after_the_wait_with_advice(qapp):
    ros = StubRos(subscribers=0)
    lch, signals = _launcher(ros, ss.SIMULATION, ping_enable_wait_s=0.0)
    messages: list = []
    signals.status_message.connect(messages.append)
    lch._set_state(PipelineState.RUNNING)
    lch.enable_pinging()
    assert ros.enables == [True]
    assert any("Gazebo world" in m for m in messages)


def test_enable_waits_for_bring_up_and_stop_cancels_it(qapp):
    ros = StubRos(subscribers=1)
    lch, _ = _launcher(ros, ss.REAL)
    lch._set_state(PipelineState.STARTING)
    lch.enable_pinging()
    assert ros.enables == []
    lch._set_state(PipelineState.RUNNING)
    lch._poll_ping_enable()
    assert ros.enables == [True]

    ros2 = StubRos(subscribers=0)
    lch2, _ = _launcher(ros2, ss.REAL, ping_enable_wait_s=60.0)
    lch2._set_state(PipelineState.RUNNING)
    lch2.enable_pinging()
    lch2.disable_pinging()
    ros2.subscribers = 1
    lch2._poll_ping_enable()
    assert ros2.enables == [False], "a cancelled START still enabled pinging"


def test_switching_source_while_down_only_selects(qapp, monkeypatch):
    lch, _ = _launcher(StubRos(), ss.SIMULATION)
    started: list = []
    monkeypatch.setattr(lch, "start", lambda: started.append(True) or True)
    lch.set_source(ss.REAL)
    assert lch.source == ss.REAL and started == []


@pytest.mark.parametrize("pattern", ["sss_node.py"])
def test_owned_patterns_match_the_launch_file(pattern):
    cfg = PipelineConfig()
    assert pattern in cfg.owned_process_patterns
    launch = (Path(__file__).resolve().parents[1] / "launch"
              / cfg.launch_command[-1]).read_text(encoding="utf-8")
    assert f"'{pattern}'" in launch, (
        f"owned_process_patterns has {pattern!r}, absent from the launch file")


# ---- bag replay: START starts the simulator over the replay's world (2026-11-02)
GZ = ["ruby", "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz", "sim"]
REPLAY = "/home/op/ros2_ws/install/blueboat_control/lib/blueboat_control/bag_replay.py"


def _world(root: Path, name: str, acoustic: bool = True) -> Path:
    d = root / name
    d.mkdir(parents=True)
    (d / "world.sdf").write_text("<sdf/>")
    if acoustic:
        (d / "scene.npz").write_bytes(b"")
        (d / "scene_manifest.yaml").write_text("version: 1\n")
    return d


def test_world_dir_from_a_gazebo_command_line(tmp_path):
    w = _world(tmp_path, "pool_1w")
    stock = _world(tmp_path, "ocean", acoustic=False)
    assert ss.world_dir_from_argv(GZ + [str(w / "world.sdf"), "-r"]) == str(w)
    assert ss.world_dir_from_argv(GZ + [str(stock / "world.sdf")]) is None
    # the two shapes the process table really shows (measured 2026-11-02)
    assert ss.world_dir_from_argv(["/bin/sh", "-c", " ".join(
        GZ + [str(w / "world.sdf"), "-r", "-s", "--force-version", "8"])]) == str(w)
    assert ss.world_dir_from_argv([f"gz sim {w / 'world.sdf'} -r -s"]) == str(w)
    assert ss.world_dir_from_argv([PY, DRIVER]) is None


def test_find_replay_reads_the_process_table(tmp_path):
    w = _world(tmp_path / "worlds", "pool_2w")
    proc = tmp_path / "proc"
    _proc(proc, 300, [PY, REPLAY, "--ros-args"], {})
    _proc(proc, 301, GZ + [str(w / "world.sdf"), "-r"], {})
    _proc(proc, 302, ["vim", REPLAY], {})
    assert ss.find_replay(proc) == (300, str(w))
    empty = tmp_path / "proc2"
    _proc(empty, 400, GZ + [str(w / "world.sdf")], {})       # a world, no replay
    assert ss.find_replay(empty) == (None, str(w))


def test_replay_simulator_plan_rules():
    sim = ss.SonarProcess(pid=5, kind="simulator", owned=False)
    drv = ss.SonarProcess(pid=6, kind="driver", owned=True)
    w = "/home/op/worlds/p/pool_1w"
    world, msg = ss.replay_simulator_plan(ss.SIMULATION, [], 300, w)
    assert world == w and "starting sss_sim_node" in msg
    assert ss.replay_simulator_plan(ss.REAL, [], 300, w) == (None, "")
    assert ss.replay_simulator_plan(ss.SIMULATION, [], None, w) == (None, "")
    assert ss.replay_simulator_plan(ss.SIMULATION, [sim], 300, w) == (None, "")
    assert ss.replay_simulator_plan(ss.SIMULATION, [], 300, w, enabled=False) == (None, "")
    world, msg = ss.replay_simulator_plan(ss.SIMULATION, [drv], 300, w)
    assert world is None and "pid 6" in msg
    world, msg = ss.replay_simulator_plan(ss.SIMULATION, [], 300, None)
    assert world is None and "world folder" in msg
    assert ss.simulator_command(PipelineConfig().replay_simulator_launch, w)[-1] \
        == f"world_dir:={w}"


class FakeProc:
    def __init__(self, cmd, **kw) -> None:
        self.cmd, self.env, self.pid = cmd, kw.get("env", {}), 4242
        self.stdout = iter(())
        self.code = None

    def poll(self):
        return self.code


def test_start_starts_the_replay_simulator_and_stop_signals_it(qapp, monkeypatch):
    from blueboat_gcs.ros import pipeline_launcher as pl
    w = "/home/op/worlds/p/pool_1w"
    monkeypatch.setattr(ss, "find_replay", lambda: (300, w))
    monkeypatch.setattr(ss, "scan_processes", lambda: [])
    spawned: list = []
    monkeypatch.setattr(pl.subprocess, "Popen",
                        lambda cmd, **kw: spawned.append(FakeProc(cmd, **kw)) or spawned[-1])
    sent: list = []
    monkeypatch.setattr(pl.os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(pl.os, "killpg", lambda pg, sig: sent.append((pg, sig)))
    ros = StubRos(subscribers=0)
    lch, signals = _launcher(ros, ss.SIMULATION, ping_enable_wait_s=0.0,
                             replay_simulator_wait_s=60.0)
    messages: list = []
    signals.status_message.connect(messages.append)
    lch._set_state(PipelineState.RUNNING)
    lch.enable_pinging()
    assert len(spawned) == 1 and spawned[0].cmd[-1] == f"world_dir:={w}"
    assert "sss_sim_launch.py" in spawned[0].cmd
    assert spawned[0].env.get(ss.OWNER_ENV) == "1"
    assert ros.enables == [], "did not wait for the simulator to load"
    ros.subscribers = 1
    lch._poll_ping_enable()
    assert ros.enables == [True]
    lch.enable_pinging()                       # a second START: no second simulator
    assert len(spawned) == 1
    lch.stop()
    assert sent and sent[0][0] == 4242


def test_real_or_no_replay_starts_no_simulator(qapp, monkeypatch):
    from blueboat_gcs.ros import pipeline_launcher as pl
    monkeypatch.setattr(ss, "find_replay", lambda: (None, "/w"))
    monkeypatch.setattr(ss, "scan_processes", lambda: [])
    spawned: list = []
    monkeypatch.setattr(pl.subprocess, "Popen",
                        lambda cmd, **kw: spawned.append(cmd) or FakeProc(cmd, **kw))
    for source in (ss.SIMULATION, ss.REAL):
        lch, _ = _launcher(StubRos(subscribers=1), source)
        lch._set_state(PipelineState.RUNNING)
        lch.enable_pinging()
    assert spawned == []


def test_replay_open_dialog_tells_no_profiles_from_no_poses():
    from types import SimpleNamespace
    from blueboat_gcs.gui.replay_window import no_pings_message
    assert "no sonar profile" in no_pings_message(SimpleNamespace(dropped_no_pose=0))
    assert "lacked a pose" in no_pings_message(SimpleNamespace(dropped_no_pose=12))
