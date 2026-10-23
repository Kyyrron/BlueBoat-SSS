"""Bottom toolbar: acquisition control, recording sessions, console.

Lifecycle (see ros/pipeline_launcher.py + gui/main_window.py):

* the processing pipeline is launched automatically at application
  startup — no button starts nodes;
* **START** enables sonar pinging + live visualization (no node
  restart); the button flips to reflect the running state;
* **Record ON/OFF** is a toggle, independent from visualization:
  ON publishes log_enable and opens a recording session, OFF closes the
  session and saves every artifact into its folder;
* **STOP** disables pinging, closes an active recording session, and
  gracefully terminates the ROS 2 nodes (same as closing the app);
* **Sonar: Simulation / Real sonars** chooses what the pipeline serves
  (``ros/sonar_source.py``): the simulator's ``/side_scan_sonar``, or the
  real driver ``sss_node.py``, which the pipeline then starts itself. The
  choice is remembered; changing it while the pipeline runs restarts it.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (QComboBox, QLabel, QPushButton, QSizePolicy,
                               QToolBar, QWidget)

from ..ros import sonar_source as ss


class AcquisitionToolbar(QToolBar):
    """Sonar source / START / STOP / Record ON-OFF / Console toggle."""

    sonar_source_changed = Signal(str)  # sonar_source.SIMULATION | REAL
    start_clicked = Signal()
    stop_clicked = Signal()
    record_toggled = Signal(bool)      # True = Record ON, False = OFF
    console_toggled = Signal(bool)
    open_svlog_clicked = Signal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("Acquisition", parent)
        self.setMovable(False)
        self.setFloatable(False)

        self.addWidget(QLabel(" Sonar: "))
        self._source_box = QComboBox()
        for key in ss.SOURCES:
            self._source_box.addItem(ss.LABELS[key], key)
        self._source_box.setToolTip(
            "Which sonar /side_scan_sonar is.\n"
            "Simulation: the simulator's node, started with the Gazebo\n"
            "world; the pipeline starts the processor only.\n"
            "Real sonars: the pipeline also starts sss_node.py, which\n"
            "drives the two Omniscan units over the BlueBoat Base Station\n"
            "WiFi. Refused (processor only) while another\n"
            "/side_scan_sonar is running. Remembered between sessions;\n"
            "changing it while the pipeline runs restarts the pipeline.")
        self._source_box.currentIndexChanged.connect(self._on_source_index)
        self.addWidget(self._source_box)

        self._start_btn = QPushButton("▶  START")
        self._start_btn.setObjectName("startButton")
        self._start_btn.setToolTip(
            "Enable sonar pinging and live visualization.\n"
            "The processing node is already running (launched at startup);\n"
            "no ROS 2 node is restarted.")
        self._start_btn.clicked.connect(self.start_clicked)
        self.addWidget(self._start_btn)

        self._stop_btn = QPushButton("■  STOP")
        self._stop_btn.setObjectName("stopButton")
        self._stop_btn.setEnabled(False)
        self._stop_btn.setToolTip(
            "Disable pinging, close an active recording session, and\n"
            "gracefully terminate the ROS 2 nodes.")
        self._stop_btn.clicked.connect(self.stop_clicked)
        self.addWidget(self._stop_btn)

        self._record_btn = QPushButton("⏺  Record: OFF")
        self._record_btn.setCheckable(True)
        self._record_btn.setEnabled(False)
        self._record_btn.setToolTip(
            "Recording is independent from visualization.\n"
            "ON: publishes log_enable and opens a recording session.\n"
            "OFF: closes the session and saves every artifact\n"
            "(svlog, mosaic, waterfall, trajectory, detections, metadata)\n"
            "into one folder — one experiment = one folder.")
        self._record_btn.toggled.connect(self._on_record_toggled)
        self.addWidget(self._record_btn)

        self._console_btn = QPushButton("Console")
        self._console_btn.setCheckable(True)
        self._console_btn.setToolTip(
            "Embedded SSS console: sonar driver and sss_processor output,\n"
            "connection state, acquisition parameters and GCS warnings.\n"
            "Control / navigation nodes are in the Mission Control Station.")
        self._console_btn.toggled.connect(self.console_toggled)
        self.addWidget(self._console_btn)

        self._svlog_open_btn = QPushButton("Open SVLOG")
        self._svlog_open_btn.setToolTip(
            "Open a recorded .svlog in the replay window:\n"
            "render any time range, or replay the mission at x1–x8,\n"
            "with the same map/waterfall options as the live view.")
        self._svlog_open_btn.clicked.connect(self.open_svlog_clicked)
        self.addWidget(self._svlog_open_btn)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.addWidget(spacer)

        self._state_label = QLabel("pipeline: stopped")
        self._state_label.setObjectName("valueLabel")
        self.addWidget(self._state_label)

        self._pipeline_running = False
        self._viz_running = False

    # ---- sonar source --------------------------------------------------------------
    def set_sonar_source(self, source: str) -> None:
        """Show ``source`` without emitting (startup, or a declined switch)."""
        i = self._source_box.findData(ss.normalise(source))
        self._source_box.blockSignals(True)
        self._source_box.setCurrentIndex(max(i, 0))
        self._source_box.blockSignals(False)

    def set_sonar_selector_available(self, available: bool) -> None:
        """The built-in simulator (``--sim``) has no pipeline to choose for."""
        self._source_box.setEnabled(available)

    def _on_source_index(self, _index: int) -> None:
        self.sonar_source_changed.emit(str(self._source_box.currentData()))

    # ---- slots -----------------------------------------------------------------
    def _on_record_toggled(self, on: bool) -> None:
        self._record_btn.setText("⏺  Record: ON" if on else "⏺  Record: OFF")
        self.record_toggled.emit(on)

    def on_recording_state(self, active: bool) -> None:
        """Keep the toggle honest if the session state changes elsewhere
        (e.g. STOP closes it)."""
        if self._record_btn.isChecked() != active:
            self._record_btn.blockSignals(True)
            self._record_btn.setChecked(active)
            self._record_btn.setText("⏺  Record: ON" if active
                                     else "⏺  Record: OFF")
            self._record_btn.blockSignals(False)

    def set_console_checked(self, on: bool) -> None:
        self._console_btn.setChecked(on)

    def on_viz_state(self, running: bool) -> None:
        self._viz_running = running
        self._refresh()

    def on_pipeline_state(self, state: str) -> None:
        self._state_label.setText(f"pipeline: {state}")
        self._pipeline_running = (state == "running")
        if state in ("stopped", "error"):
            self.on_recording_state(False)
        self._refresh()

    def _refresh(self) -> None:
        # START is available whenever visualization is not live (it also
        # relaunches the pipeline after a STOP); STOP whenever something
        # is running; Record only with a live pipeline.
        self._start_btn.setEnabled(not self._viz_running)
        self._stop_btn.setEnabled(self._viz_running or self._pipeline_running)
        self._record_btn.setEnabled(self._pipeline_running)
