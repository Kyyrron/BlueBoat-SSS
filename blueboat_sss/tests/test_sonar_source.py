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
