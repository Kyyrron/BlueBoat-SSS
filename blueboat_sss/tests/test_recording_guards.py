"""Recording/session lifecycle guards (the "only the first session gets
a .svlog" field bug).

Root causes fixed and pinned here:

* ``PipelineLauncher.set_recording(True)`` silently no-oped when the
  pipeline was not RUNNING while the GUI opened a session anyway — a
  session folder with ``adopted_svlogs: []`` forever;
* ``start()`` refused while STOPPING but the GUI set the viz gate
  regardless, greying START out with nothing running;
* a pipeline drop while recording only unchecked the toolbar button
  (``blockSignals``), leaving ``RecordingManager`` active so the next
  Record ON silently reused the first session's folder;
* adoption ran the instant the session ended, racing the asynchronous
  ``log_enable=False`` (the processor's next append recreated a stub);
* ``_adopt_svlogs`` was silent when it adopted nothing, and would sweep
  a merged session's .svlog out of ``merged_sessions/``.

No ROS: the acquisition controller is a fake with the launcher's
interface, exactly the way ``--sim`` substitutes the Simulator.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

import svlog_helper


def _valid_svlog_bytes() -> bytes:
    """One real framed packet — the adoption sweep must accept it."""
    return svlog_helper.frame_packet(
        svlog_helper.JSON_WRAPPER_ID, b"{}", src=0,
        dst=svlog_helper.DST_BROADCAST)


class FakeAcquisition:
    """PipelineLauncher stand-in with scriptable refusals."""

    def __init__(self) -> None:
        self.running = False
        self.refuse_start = False
        self.refuse_recording = False
        self.recording_calls: list = []

    def start(self) -> bool:
        if self.refuse_start:
            return False
        self.running = True
        return True

    def stop(self) -> None:
        self.running = False

    def enable_pinging(self) -> None:
        pass

    def disable_pinging(self) -> None:
        pass

    def set_recording(self, on: bool) -> bool:
        if on and (self.refuse_recording or not self.running):
            return False
        self.recording_calls.append(on)
        return True

    def reset_stream_health(self) -> None:
        pass


@pytest.fixture
def gui(qapp, tmp_config, no_modal_dialogs):
    """A MainWindow driven by the fake acquisition controller."""
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.gui.main_window import MainWindow

    signals = AppSignals()
    mosaic = MosaicService(tmp_config)
    acquisition = FakeAcquisition()
    window = MainWindow(tmp_config, signals, mosaic, acquisition)
    try:
        yield window, acquisition, signals, tmp_config
    finally:
        window.recording._pending_adoption = None   # no cross-test carry
        window.close()
        qapp.processEvents()


@pytest.fixture
def manager(qapp, tmp_config):
    """A bare RecordingManager (no GUI) plus the messages it emitted."""
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.recording_session import RecordingManager
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.core.waterfall_service import WaterfallService

    signals = AppSignals()
    messages: list = []
    signals.status_message.connect(messages.append)
    rec = RecordingManager(tmp_config, signals,
                           MosaicService(tmp_config),
                           WaterfallService(tmp_config))
    return rec, messages, Path(tmp_config.data_root)


# ---- GUI-level guards ------------------------------------------------------------

def test_refused_record_on_creates_no_session(gui):
    window, acquisition, _signals, config = gui
    acquisition.running = False                 # pipeline not running
    window.toolbar._record_btn.setEnabled(True)
    window.toolbar._record_btn.setChecked(True)  # the real click path
    assert not window.recording.active, (
        "a session opened although the processor never saw log_enable")
    assert not (Path(config.data_root) / "sessions").exists()
    assert not window.toolbar._record_btn.isChecked(), (
        "the Record button stayed checked after the refusal")


def test_refused_start_leaves_viz_gate_closed(gui):
    window, acquisition, _signals, _config = gui
    acquisition.refuse_start = True              # e.g. launcher STOPPING
    window._on_start()
    assert not window._viz_enabled
    assert not window.toolbar._start_btn.isEnabled() is False  # still enabled
    # And a later successful START works normally.
    acquisition.refuse_start = False
    window._on_start()
    assert window._viz_enabled


def test_pipeline_drop_closes_the_session_once(gui):
    window, acquisition, signals, config = gui
    acquisition.running = True
    window._on_record_toggled(True)
    assert window.recording.active
    session = window.recording.session_dir

    signals.pipeline_state.emit("stopped")       # pipeline died
    assert not window.recording.active, (
        "RecordingManager stayed active after the pipeline dropped — the "
        "next Record ON would silently reuse this session's folder")
    assert (session / "metadata.json").is_file()

    # A later Record ON opens a fresh session rather than being the
    # silent early-return that used to keep appending to session 1.
    # (Two sessions in the same wall-clock second share a stamp — dir
    # names are second-resolution — so assert on the lifecycle, not the
    # path.)
    acquisition.running = True
    window._on_record_toggled(True)
    assert window.recording.active, "the second Record ON was a no-op"
    window._on_stop()


def test_stop_adopts_synchronously(gui):
    window, acquisition, _signals, config = gui
    acquisition.running = True
    window._on_record_toggled(True)
    session = window.recording.session_dir
    root = Path(config.data_root)
    root.mkdir(parents=True, exist_ok=True)
    (root / "planted.svlog").write_bytes(_valid_svlog_bytes())

    window._on_stop()
    meta = json.loads((session / "metadata.json").read_text())
    assert meta["adopted_svlogs"] == ["planted.svlog"]
    assert (session / "planted.svlog").is_file()


# ---- RecordingManager guards ----------------------------------------------------

def test_begin_while_active_is_refused_and_visible(manager):
    rec, messages, _root = manager
    assert rec.begin() is True
    first = rec.session_dir
    messages.clear()
    assert rec.begin() is False
    assert rec.session_dir == first
    assert any("still active" in m for m in messages)
    rec.end()
    rec.adopt_now()


def test_deferred_adoption_catches_a_late_file(manager):
    """The file appears AFTER end() (the log_enable=False race) and the
    deferred adopt_now() still moves it into the session."""
    rec, _messages, root = manager
    rec.begin()
    session = rec.session_dir
    rec.end()
    meta = json.loads((session / "metadata.json").read_text())
    assert meta["adopted_svlogs"] == []          # not adopted yet

    root.mkdir(parents=True, exist_ok=True)
    (root / "late.svlog").write_bytes(_valid_svlog_bytes())
    adopted = rec.adopt_now()
    assert adopted == ["late.svlog"]
    assert (session / "late.svlog").is_file()
    meta = json.loads((session / "metadata.json").read_text())
    assert meta["adopted_svlogs"] == ["late.svlog"]
    assert rec.adopt_now() == []                 # idempotent


def test_empty_adoption_with_pings_warns(manager):
    rec, messages, root = manager
    root.mkdir(parents=True, exist_ok=True)
    rec.begin()
    rec._ping_count = 123                        # pings flowed…
    rec.end()
    messages.clear()
    assert rec.adopt_now() == []                 # …but no .svlog appeared
    assert any("WARNING" in m and "no .svlog" in m for m in messages), messages


def test_stub_files_are_skipped_not_moved(manager):
    rec, messages, root = manager
    root.mkdir(parents=True, exist_ok=True)
    rec.begin()
    rec.end()
    stub = root / "stub.svlog"
    stub.write_bytes(b"BR\x00")                  # < smallest framed packet
    assert rec.adopt_now() == []
    assert stub.is_file(), "the stub was moved or deleted (NC #6)"
    assert any("stub" in m.lower() for m in messages)


def test_merged_sessions_are_never_swept(manager):
    """A merged session's .svlog is derived-but-settled data: the next
    live session must not steal it out of merged_sessions/."""
    rec, _messages, root = manager
    merged = root / "merged_sessions" / "merged_2026_01_01_0101_0202"
    merged.mkdir(parents=True)
    protected = merged / "merged.svlog"
    protected.write_bytes(_valid_svlog_bytes())

    rec.begin()
    rec.end()
    assert rec.adopt_now() == []
    assert protected.is_file(), (
        "the adoption sweep moved a .svlog out of merged_sessions/")
