"""Runtime sonar-range control (right-panel slider).

sss_node reads its parameters only on the ping/enable rising edge, so a
live range change is a three-step dance owned by the launcher:
enable=false → settle → set_parameters → enable=true on the result —
and pinging is ALWAYS resumed, success or failure. The GUI control is
enabled only while the pipeline runs; the simulator refuses gracefully.
"""

from __future__ import annotations

import pytest

from blueboat_gcs.config.settings import AcquisitionConfig, PipelineConfig
from blueboat_gcs.core.signals import AppSignals
from blueboat_gcs.ros.pipeline_launcher import PipelineLauncher, PipelineState


class StubRos:
    """RosManager stand-in: records the dance, answers on the bus."""

    def __init__(self, signals: AppSignals, ok: bool = True,
                 sendable: bool = True) -> None:
        self._signals = signals
        self._ok = ok
        self._sendable = sendable
        self.enables: list = []
        self.range_calls: list = []

    def publish_ping_enable(self, enable: bool) -> None:
        self.enables.append(enable)

    def publish_svlog_enable(self, enable: bool) -> None:
        pass

    def reset_stream_health(self) -> None:
        pass

    def set_sonar_range(self, range_m: float) -> bool:
        self.range_calls.append(range_m)
        self._signals.sonar_params_result.emit(
            self._ok, "ok" if self._ok else "rejected")
        return self._sendable


@pytest.fixture
def launcher(qapp):
    signals = AppSignals()
    ros = StubRos(signals)
    lch = PipelineLauncher(
        PipelineConfig(leftover_process_patterns=[]), ros, signals,
        acquisition=AcquisitionConfig(settle_delay_s=0.0))
    return lch, ros, signals


def _spin(qapp, ms: int = 50) -> None:
    """Let zero-delay QTimer.singleShot callbacks run."""
    from PySide6.QtCore import QCoreApplication, QEventLoop, QTimer
    loop = QEventLoop()
    QTimer.singleShot(ms, loop.quit)
    loop.exec()
    QCoreApplication.processEvents()


def test_refused_when_pipeline_not_running(launcher, qapp):
    lch, ros, _signals = launcher
    assert lch.set_range(30.0) is False
    assert ros.enables == [], "pinging was touched despite the refusal"


def test_success_path_runs_the_full_dance(launcher, qapp):
    lch, ros, _signals = launcher
    lch._set_state(PipelineState.RUNNING)
    assert lch.set_range(30.0) is True
    _spin(qapp)
    assert ros.range_calls == [30.0]
    # enable=false (pause), then enable=true (resume on the result).
    assert ros.enables == [False, True]
    assert not lch._range_pending


def test_failure_still_resumes_pinging(qapp):
    """A rejected parameter change must never leave acquisition off."""
    signals = AppSignals()
    ros = StubRos(signals, ok=False)
    lch = PipelineLauncher(
        PipelineConfig(leftover_process_patterns=[]), ros, signals,
        acquisition=AcquisitionConfig(settle_delay_s=0.0))
    messages: list = []
    signals.status_message.connect(messages.append)
    lch._set_state(PipelineState.RUNNING)
    lch.set_range(30.0)
    _spin(qapp)
    assert ros.enables == [False, True], "pinging was left off after a failure"
    assert any("FAILED" in m for m in messages)


def test_reentrancy_guard(launcher, qapp):
    lch, ros, _signals = launcher
    lch._set_state(PipelineState.RUNNING)
    assert lch.set_range(30.0) is True
    assert lch.set_range(40.0) is False       # one dance at a time
    _spin(qapp)
    assert ros.range_calls == [30.0]


def test_simulator_refuses_gracefully(qapp, tmp_config):
    from blueboat_gcs.sim.simulator import Simulator
    signals = AppSignals()
    messages: list = []
    signals.status_message.connect(messages.append)
    sim = Simulator(tmp_config, signals)
    assert sim.set_range(30.0) is False
    assert any("fixed" in m for m in messages)


def test_gui_gates_the_control_and_breaks_the_waterfall(qapp, tmp_config,
                                                        no_modal_dialogs):
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.gui.main_window import MainWindow
    from test_recording_guards import FakeAcquisition

    signals = AppSignals()
    window = MainWindow(tmp_config, signals, MosaicService(tmp_config),
                        FakeAcquisition())
    try:
        assert window.right_panel._acq_box is not None
        assert not window.right_panel._acq_box.isEnabled()
        signals.pipeline_state.emit("running")
        assert window.right_panel._acq_box.isEnabled()
        signals.pipeline_state.emit("stopped")
        assert not window.right_panel._acq_box.isEnabled()

        # A successful range change marks the waterfall with a seam: the
        # column scale changes at that row.
        before = window.waterfall_service.total_rows
        signals.sonar_params_result.emit(True, "ok")
        assert window.waterfall_service.total_rows == before + 1
        signals.sonar_params_result.emit(False, "nope")
        assert window.waterfall_service.total_rows == before + 1
    finally:
        window.close()
        qapp.processEvents()


def test_replay_panel_has_no_acquisition_group(qapp):
    from blueboat_gcs.gui.right_panel import RightPanel
    panel = RightPanel()                      # replay-style construction
    assert panel._acq_box is None
