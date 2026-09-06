"""End-to-end ``--sim`` runs: START/STOP, and a recording session.

The whole stack runs with no ROS and no boat, exactly as
``python -m blueboat_gcs.main --sim`` does. Driven with
``QTimer.singleShot`` phases ending in ``app.quit()``, which is the
pattern that has repeatedly caught real regressions here.

Trap worth knowing before editing these tests: pings only reach the
mosaic, waterfall and seabed imager while ``_viz_enabled`` is set —
``gui/main_window.py``, ``_on_sonar_ping_data``. The pipeline runs from
application startup but the views stay dark until START. A harness that
calls ``enable_pinging()`` directly instead of pressing START sees pings
on the signal bus and an empty mosaic. Both tests below press START.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List

import numpy as np
import pytest

# The robot-side frame writer: ROS-free, and the module that produces every
# real .svlog. conftest.py puts src/_custom_libraries on sys.path.
import svlog_helper

# Wall-clock length of the acquisition window. Long enough that the
# 15 Hz simulator delivers plenty of pings and the seabed imager has a
# flushable tail; short enough for a pre-commit hook.
ACQUIRE_MS = 3000
SETTLE_MS = 400

# Nominal is ACQUIRE_MS/1000 * sim.ping_hz = 45. The bound is deliberately
# slack: QTimer jitter under offscreen Qt is the only thing being tolerated,
# and a real acquisition break would fall far below this.
MIN_PINGS = 20

SESSION_STAMP_FORMAT = "%Y_%m_%d-%H_%M_%S"

# Name of the .svlog the layout test plants in data_root mid-session; the
# processor's own naming (svlog_helper.SvlogWriter), so the adoption sweep
# sees exactly what it sees in the field.
_PLANTED_SVLOG = "2026-07-08-14-02-35.svlog"


@dataclass
class SimHarness:
    """The assembled --sim application plus what the run observed."""

    app: object
    config: object
    signals: object
    window: object
    mosaic: object
    pings: List[object] = field(default_factory=list)

    @property
    def data_root(self) -> Path:
        return Path(self.config.data_root).expanduser()

    @property
    def sessions_dir(self) -> Path:
        return self.data_root / "sessions"


@pytest.fixture
def sim_app(qapp, tmp_config, no_modal_dialogs):
    """Build the --sim stack the way main.py does, minus the ROS branch."""
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.signals import AppSignals
    from blueboat_gcs.gui.main_window import MainWindow
    from blueboat_gcs.sim.simulator import Simulator

    signals = AppSignals()
    mosaic = MosaicService(tmp_config)
    acquisition = Simulator(tmp_config, signals)
    window = MainWindow(tmp_config, signals, mosaic, acquisition)

    harness = SimHarness(app=qapp, config=tmp_config, signals=signals,
                         window=window, mosaic=mosaic)
    # Tap the bus. Connected after MainWindow so the window's own slot
    # runs first and the ping has already been through the viz gate.
    signals.sonar_ping.connect(harness.pings.append)

    window.show()
    acquisition.start()          # pipeline up: telemetry only, no pings
    try:
        yield harness
    finally:
        window.close()           # disable_pinging + stop, per closeEvent
        qapp.processEvents()


def _run_phases(app, phases):
    """Run (delay_ms, callable) phases on the Qt event loop, then quit."""
    from PySide6.QtCore import QTimer

    for delay, fn in phases:
        QTimer.singleShot(delay, fn)
    QTimer.singleShot(phases[-1][0] + SETTLE_MS, app.quit)
    app.exec()


def test_sim_start_stop_cycle(sim_app):
    """A START -> acquire -> STOP cycle, and NC #9: no session, no export."""
    h = sim_app
    observed = {}

    def on_stop():
        # Capture before STOP tears the acquisition down.
        observed["cell_size_m"] = h.mosaic.cell_size_m
        chrono = h.window.waterfall_service.chronological()
        observed["waterfall_shape"] = None if chrono is None else chrono.shape
        observed["anchored"] = h.window.geo.ready
        observed["world_root_visible"] = h.window.world_root.item.isVisible()
        h.window.toolbar.stop_clicked.emit()

    _run_phases(h.app, [
        (0, h.window.toolbar.start_clicked.emit),
        (ACQUIRE_MS, on_stop),
    ])

    # --- pings actually flowed -------------------------------------------------
    assert len(h.pings) >= MIN_PINGS, (
        f"only {len(h.pings)} pings in {ACQUIRE_MS} ms at "
        f"{h.config.sim.ping_hz} Hz (nominal "
        f"{ACQUIRE_MS / 1000 * h.config.sim.ping_hz:.0f})"
    )

    # --- side split is even ----------------------------------------------------
    # +y = port, -y = starboard (CLAUDE.md, ProcessedSSSPing). Structural in
    # the simulator (_SAMPLES_PER_SIDE), so this asserts the models and the
    # signal path have not started dropping or mangling one side.
    for ping in h.pings:
        port = int(np.count_nonzero(ping.y_local > 0))
        stbd = int(np.count_nonzero(ping.y_local < 0))
        assert port == stbd and port > 0, (
            f"uneven side split: {port} +y vs {stbd} -y")

    # --- the mosaic auto-tuned off the configured default ----------------------
    default_cell = h.config.mosaic.cell_size_m
    assert h.config.mosaic.auto_cell_size, "fixture lost auto_cell_size"
    assert observed["cell_size_m"] != pytest.approx(default_cell), (
        f"mosaic cell size never moved off the {default_cell} m default; "
        "auto-tuning is not running")
    assert (h.config.mosaic.min_cell_size_m
            <= observed["cell_size_m"]
            <= h.config.mosaic.max_cell_size_m), observed["cell_size_m"]

    # --- the waterfall filled --------------------------------------------------
    shape = observed["waterfall_shape"]
    assert shape is not None, "waterfall buffer never received a row"
    rows, cols = shape
    # Native slant-bin layout: one column per device bin per side —
    # nothing is forced onto a fixed 800-column grid any more.
    from blueboat_gcs.sim import simulator as sim_mod
    assert cols == 2 * sim_mod._SAMPLES_PER_SIDE, (
        f"waterfall column count {cols}, expected "
        f"{2 * sim_mod._SAMPLES_PER_SIDE} (native bins)")
    assert rows == len(h.pings), (
        f"{rows} waterfall rows for {len(h.pings)} pings — the viz gate "
        "dropped rows that reached the bus")

    # --- the GPS anchor opened during the run ----------------------------------
    # The simulator emits gps_fix + robot_state at 5 Hz; with min_pairs=5
    # the translation-only anchor must be valid well inside the run, and
    # the gated world root must be showing (GPS-anchored map port).
    assert observed["anchored"], (
        "the odom<->GPS anchor never became valid during a --sim run")
    assert observed["world_root_visible"], (
        "anchor valid but the world root stayed hidden — the gate wiring "
        "is broken")

    # --- NC #9: data leaves the GCS only through a recording session -----------
    assert not h.sessions_dir.exists(), (
        f"STOP with no active session created {h.sessions_dir}; "
        "nothing may be exported without a session (NC #9)")


def test_sim_recording_session_layout(sim_app):
    """Record ON -> pings -> STOP writes the documented session tree."""
    h = sim_app

    def start_and_record():
        h.window.toolbar.start_clicked.emit()
        h.window.toolbar.record_toggled.emit(True)

    def plant_svlog():
        """Stand in for the processor, which --sim does not run.

        A real framed packet rather than junk bytes, so the file the
        adoption sweep moves is one SonarView would accept.
        """
        h.data_root.mkdir(parents=True, exist_ok=True)
        (h.data_root / _PLANTED_SVLOG).write_bytes(
            svlog_helper.frame_packet(svlog_helper.JSON_WRAPPER_ID,
                                      b"{}", src=0,
                                      dst=svlog_helper.DST_BROADCAST))

    _run_phases(h.app, [
        (0, start_and_record),
        (ACQUIRE_MS // 2, plant_svlog),
        (ACQUIRE_MS, h.window.toolbar.stop_clicked.emit),
    ])

    assert len(h.pings) >= MIN_PINGS, f"only {len(h.pings)} pings recorded"

    sessions = sorted(h.sessions_dir.iterdir())
    assert len(sessions) == 1, f"expected one session folder, got {sessions}"
    session = sessions[0]

    # Stamp format is what makes sessions sort chronologically on disk.
    datetime.strptime(session.name, SESSION_STAMP_FORMAT)

    # --- metadata --------------------------------------------------------------
    meta_path = session / "metadata.json"
    assert meta_path.is_file(), "session has no metadata.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    for key in ("ping_count", "detection_count", "adopted_svlogs", "mosaic"):
        assert key in meta, f"metadata.json lost {key!r}: {sorted(meta)}"
    assert meta["ping_count"] >= MIN_PINGS, meta["ping_count"]

    # --- mosaic ----------------------------------------------------------------
    for name in ("sonar_mosaic.npz", "sonar_mosaic.png", "boat_trajectory.csv"):
        assert (session / "mosaic" / name).is_file(), f"missing mosaic/{name}"

    # --- waterfall (waterfall_raw.npz is the dataset source) -------------------
    for name in ("waterfall.png", "waterfall_raw.npz"):
        assert (session / "waterfall" / name).is_file(), f"missing waterfall/{name}"
    with np.load(session / "waterfall" / "waterfall_raw.npz") as npz:
        assert "intensity_db" in npz, (
            "waterfall_raw.npz lost intensity_db — this is the array AI "
            "datasets train on, not the PNG")

    # --- seabed images ---------------------------------------------------------
    # A run shorter than seabed.rows still yields one truncated image:
    # flush() emits the tail so no data is wasted.
    seabed = session / "seabed_images"
    assert seabed.is_dir(), "no seabed_images/ in the session"
    assert list(seabed.glob("seabed_*.png")), "flush() emitted no seabed image"

    # --- detections are written only when there are any ------------------------
    det_dir = session / "detections"
    if det_dir.exists():
        assert (det_dir / "detections.csv").is_file()

    # --- adopted .svlog: session root, not a svlog/ subfolder ------------------
    # Planted by the test: --sim has no processor, so nothing writes a real
    # .svlog. What is under test is _adopt_svlogs' destination, which is
    # independent of who produced the file.
    adopted = meta["adopted_svlogs"]
    assert adopted == [_PLANTED_SVLOG], (
        f"planted .svlog was not adopted: {adopted}")
    assert (session / _PLANTED_SVLOG).is_file(), (
        "adopted .svlog is not at the session root")
    assert not (session / "svlog").exists(), (
        "a svlog/ subfolder appeared; the destination is the session root "
        "and core/recording_session.py, docs/HANDOVER.md and CLAUDE.md all "
        "say so")
    assert not (h.data_root / _PLANTED_SVLOG).exists(), (
        "the .svlog was copied rather than moved out of data_root")
