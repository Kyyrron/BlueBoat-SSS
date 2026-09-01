"""Right panel: view mode, map tools, display controls, distance tool."""

from __future__ import annotations

from typing import Optional, Tuple

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QComboBox, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QPushButton, QSlider,
                               QVBoxLayout, QWidget)

from ..mapping.mosaic import PRIORITY_MODES
from ..mapping.renderer import DisplaySettings, lut_names
from .live_plot import LivePlot
from .widgets import CoordinateCard

#: View-mode identifiers (QStackedWidget pages in MainWindow).
VIEW_MOSAIC = "mosaic"
VIEW_WATERFALL = "waterfall"


class RightPanel(QWidget):
    """View mode, zoom/center, SonarView-like display controls, rendering
    priority, overlay clearing, and the two-click distance tool."""

    zoom_in_clicked = Signal()
    zoom_out_clicked = Signal()
    center_robot_clicked = Signal()
    measure_toggled = Signal(bool)
    view_mode_changed = Signal(str)        # VIEW_MOSAIC | VIEW_WATERFALL
    priority_changed = Signal(str)         # mosaic.PRIORITY_MODES entry
    display_changed = Signal(object)       # renderer.DisplaySettings
    clear_overlays_clicked = Signal()
    clear_sss_clicked = Signal()
    sss_opacity_changed = Signal(float)    # 0.0 .. 1.0
    resolution_changed = Signal(float)     # mosaic cell size [m], 0 = auto
    depth_mode_changed = Signal(str, float)  # (auto|manual|off, manual_m)
    range_apply_requested = Signal(float)  # sonar range [m] (live only)

    def __init__(self, parent: Optional[QWidget] = None, *,
                 acquisition_range: Optional[Tuple[float, float]] = None
                 ) -> None:
        """``acquisition_range=(min_m, max_m)`` adds the live sonar-range
        group (main window only; the replay window has no sonar)."""
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        # ---- live plots (top of the panel) ----------------------------------------
        # Modular: any future live plot is one more LivePlot instance fed
        # from a bus signal in main_window (speed, SNR, ping rate, ...).
        alt_box = QGroupBox("Robot Altitude")
        al = QVBoxLayout(alt_box)
        al.setContentsMargins(4, 4, 4, 4)
        self.altitude_plot = LivePlot(title="water depth under boat",
                                      unit="m", window_s=60.0)
        al.addWidget(self.altitude_plot)
        root.addWidget(alt_box)

        # ---- view mode ---------------------------------------------------------
        view_box = QGroupBox("View")
        vl = QVBoxLayout(view_box)
        self._view_combo = QComboBox()
        self._view_combo.addItem("Mosaic view", VIEW_MOSAIC)
        self._view_combo.addItem("Waterfall view (raw pings)", VIEW_WATERFALL)
        self._view_combo.currentIndexChanged.connect(
            lambda i: self.view_mode_changed.emit(self._view_combo.itemData(i)))
        vl.addWidget(self._view_combo)
        root.addWidget(view_box)

        # ---- map tools -----------------------------------------------------
        tools = QGroupBox("Map tools")
        tl = QVBoxLayout(tools)
        zoom_in = QPushButton("Zoom  +")
        zoom_out = QPushButton("Zoom  −")
        center = QPushButton("Center robot")
        center.setToolTip("Centers the view on the robot once; "
                          "the camera stays free afterwards.")
        zoom_in.clicked.connect(self.zoom_in_clicked)
        zoom_out.clicked.connect(self.zoom_out_clicked)
        center.clicked.connect(self.center_robot_clicked)
        clear_btn = QPushButton("Clear currently displayed data")
        clear_btn.setToolTip(
            "Clears every overlay currently shown on the map (trajectory,\n"
            "detections, measurements, pinger, planned path…).\n"
            "Hidden overlays and the SSS mosaic are preserved.\n"
            "New incoming data continues to be displayed immediately.")
        clear_btn.clicked.connect(self.clear_overlays_clicked)
        clear_sss = QPushButton("Clear SSS data")
        clear_sss.setToolTip(
            "Discards the accumulated sonar data (mosaic + waterfall)\n"
            "while preserving every other layer, the map position and the\n"
            "zoom level. New pings keep accumulating immediately.")
        clear_sss.clicked.connect(self.clear_sss_clicked)
        for b in (zoom_in, zoom_out, center, clear_btn, clear_sss):
            tl.addWidget(b)
        root.addWidget(tools)

        # ---- display controls (SonarView-like; visualization only) ------------
        root.addWidget(self._build_display_box())

        # ---- live sonar acquisition (range slider) ----------------------------
        self._acq_box = None
        if acquisition_range is not None:
            root.addWidget(self._build_acquisition_box(*acquisition_range))

        # ---- distance tool ---------------------------------------------------
        dist = QGroupBox("Distance tool")
        dl = QVBoxLayout(dist)
        self._measure_btn = QPushButton("Measure distance")
        self._measure_btn.setCheckable(True)
        self._measure_btn.toggled.connect(self._on_measure_toggled)
        dl.addWidget(self._measure_btn)

        self._hint = QLabel("Enable, then click two points on the map.")
        self._hint.setWordWrap(True)
        self._hint.setStyleSheet("color:#8b95a1;")
        dl.addWidget(self._hint)

        self._distance_label = QLabel("—")
        self._distance_label.setObjectName("sectionValue")
        dl.addWidget(self._distance_label)

        self.point_a = CoordinateCard("Point A")
        self.point_b = CoordinateCard("Point B")
        dl.addWidget(self.point_a)
        dl.addWidget(self.point_b)
        root.addWidget(dist)

        root.addStretch(1)

    # ---- live sonar acquisition -------------------------------------------------
    def _build_acquisition_box(self, min_m: float, max_m: float) -> QGroupBox:
        """Sonar range slider + Apply. Enabled only while the pipeline is
        RUNNING (main_window drives ``set_acquisition_enabled``); Apply
        triggers the enable-off → set-param → enable-on dance in the
        launcher, so pinging pauses for ~a second at the change."""
        box = QGroupBox("Acquisition")
        form = QFormLayout(box)
        form.setLabelAlignment(Qt.AlignRight)
        self._range_slider = QSlider(Qt.Horizontal)
        self._range_slider.setRange(int(min_m), int(max_m))
        self._range_slider.setValue(20)                 # the field default
        self._range_lbl = QLabel("20 m")
        self._range_slider.valueChanged.connect(
            lambda v: self._range_lbl.setText(f"{v} m"))
        form.addRow("Sonar range",
                    self._wrap(self._range_slider, self._range_lbl))
        self._range_apply = QPushButton("Apply range")
        self._range_apply.setToolTip(
            "Set range_length_mm on the sonar node while it runs.\n"
            "Pinging pauses briefly (the node reads parameters on the\n"
            "enable rising edge) and resumes automatically.\n"
            "Rule of thumb (NC #5): ~4x the deepest expected water,\n"
            "not the area to cover — an over-long range breaks bottom\n"
            "detection.")
        self._range_apply.clicked.connect(
            lambda: self.range_apply_requested.emit(
                float(self._range_slider.value())))
        form.addRow(self._range_apply)
        box.setEnabled(False)          # until the pipeline reports running
        self._acq_box = box
        return box

    def set_acquisition_enabled(self, on: bool) -> None:
        if self._acq_box is not None:
            self._acq_box.setEnabled(on)

    # ---- display controls -------------------------------------------------------
    def _build_display_box(self) -> QGroupBox:
        box = QGroupBox("Display")
        form = QFormLayout(box)
        form.setLabelAlignment(Qt.AlignRight)

        self._cmap = QComboBox()
        self._cmap.addItems(lut_names())
        form.addRow("Color", self._cmap)

        self._priority = QComboBox()
        for mode in PRIORITY_MODES:      # average / closest / oldest / newest
            self._priority.addItem(mode.capitalize(), mode)
        self._priority.setToolTip(
            "Which sample a mosaic cell shows where survey lines overlap\n"
            "(SonarView-like): Average, Closest (smallest slant range),\n"
            "Oldest (first pass), Newest (last pass).")
        form.addRow("Priority", self._priority)

        # ---- resolution --------------------------------------------------------
        # SonarView does not render at a fixed ground-sample distance and
        # neither do we any more: "Auto" follows the sonar's own sample
        # spacing (range / num_results), which is where most of the
        # apparent sharpness difference came from.
        self._resolution = QComboBox()
        self._resolution.addItem("Auto (from data)", 0.0)
        for cm in (2, 5, 10, 15, 25, 50):
            self._resolution.addItem(f"{cm} cm", cm / 100.0)
        self._resolution.setToolTip(
            "Mosaic ground-sample distance.\n"
            "Auto keeps the best cell size the sonar's across-track\n"
            "sample spacing supports, refining as data allows.\n"
            "Changing it resamples the existing mosaic — nothing is\n"
            "cleared; older data keeps its native resolution.")
        self._resolution.currentIndexChanged.connect(
            lambda i: self.resolution_changed.emit(
                float(self._resolution.itemData(i))))
        form.addRow("Resolution", self._resolution)

        # ---- depth compensation -------------------------------------------------
        # Same control SonarView exposes: the altitude used for
        # slant-range correction. "Off" == their Manual/0 m mode, which
        # is the robust choice when bottom detection is unreliable.
        self._depth_mode = QComboBox()
        for label, key in (("Auto (bottom detect)", "auto"),
                           ("Manual", "manual"),
                           ("Off (no correction)", "off")):
            self._depth_mode.addItem(label, key)
        self._depth_mode.setToolTip(
            "Altitude used for slant-range correction.\n"
            "Auto: track the first bottom return.\n"
            "Manual: fixed value below.\n"
            "Off: plot slant range directly (SonarView 'Manual 0 m').")
        self._depth_manual = QSlider(Qt.Horizontal)
        self._depth_manual.setRange(0, 300)          # 0.0 .. 30.0 m
        self._depth_manual.setValue(0)
        self._depth_manual_lbl = QLabel("0.0 m")
        self._depth_mode.currentIndexChanged.connect(self._emit_depth)
        self._depth_manual.valueChanged.connect(self._emit_depth)
        form.addRow("Depth comp.", self._depth_mode)
        form.addRow("Manual depth",
                    self._wrap(self._depth_manual, self._depth_manual_lbl, ""))

        # Dynamic range is ALWAYS derived from the data (percentiles).
        # The former manual min/max dB sliders were removed: real
        # Omniscan data (uint16 pwr_results, per-gain min/max_pwr_db
        # spanning e.g. +7..+64 dB) makes any fixed dB window meaningless
        # across gain settings; the percentile scheme adapts by design,
        # and Contrast/Brightness below give the operator the same
        # practical control without unit assumptions.
        self._gamma = self._slider(30, 300, 100)     # /100 -> 0.3 … 3.0
        self._gamma_lbl = QLabel()
        form.addRow("Contrast", self._wrap(self._gamma, self._gamma_lbl))

        self._bright = self._slider(-50, 50, 0)      # /100 -> -0.5 … +0.5
        self._bright_lbl = QLabel()
        form.addRow("Brightness", self._wrap(self._bright, self._bright_lbl))

        # SSS layer transparency over the map background (compositing
        # only — the underlying data is never modified).
        self._opacity = self._slider(0, 100, 100)
        self._opacity_lbl = QLabel()
        form.addRow("SSS opacity", self._wrap(self._opacity,
                                              self._opacity_lbl))
        self._opacity.valueChanged.connect(self._emit_opacity)

        reset = QPushButton("Reset display")
        form.addRow(reset)

        # Wiring: any control change emits one immutable DisplaySettings.
        self._cmap.currentTextChanged.connect(self._emit_display)
        self._priority.currentIndexChanged.connect(
            lambda i: self.priority_changed.emit(self._priority.itemData(i)))
        for s in (self._gamma, self._bright):
            s.valueChanged.connect(self._emit_display)
        reset.clicked.connect(self._reset_display)
        self._emit_display()
        self._emit_opacity(self._opacity.value())
        return box

    @staticmethod
    def _slider(lo: int, hi: int, val: int) -> QSlider:
        s = QSlider(Qt.Horizontal)
        s.setRange(lo, hi)
        s.setValue(val)
        return s

    @staticmethod
    def _wrap(slider: QSlider, label: QLabel, unit: str = "") -> QWidget:
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        label.setMinimumWidth(44)
        label.setObjectName("valueLabel")
        lay.addWidget(slider, 1)
        lay.addWidget(label)
        label.setProperty("_unit", unit)
        return w

    def _reset_display(self) -> None:
        d = DisplaySettings()
        self._cmap.setCurrentText(d.colormap)
        self._gamma.setValue(int(d.gamma * 100))
        self._bright.setValue(int(d.brightness * 100))
        self._opacity.setValue(100)
        self._emit_display()

    def _emit_depth(self, *_a) -> None:
        mode = self._depth_mode.currentData()
        manual = self._depth_manual.value() / 10.0
        self._depth_manual_lbl.setText(f"{manual:.1f} m")
        self._depth_manual.setEnabled(mode == "manual")
        self.depth_mode_changed.emit(mode, manual)

    def _emit_opacity(self, value: int) -> None:
        self._opacity_lbl.setText(f"{value} %")
        self.sss_opacity_changed.emit(value / 100.0)

    def _emit_display(self, *_args) -> None:
        gamma = self._gamma.value() / 100.0
        bright = self._bright.value() / 100.0
        self._gamma_lbl.setText(f"{gamma:.2f}")
        self._bright_lbl.setText(f"{bright:+.2f}")
        # auto_range is always true: the dynamic range is derived from
        # the data percentiles (see comment in _build_display_box).
        self.display_changed.emit(DisplaySettings(
            auto_range=True, gamma=gamma, brightness=bright,
            colormap=self._cmap.currentText()))

    # ---- slots -----------------------------------------------------------------
    def _on_measure_toggled(self, on: bool) -> None:
        self._hint.setText("Click the first point…" if on
                           else "Enable, then click two points on the map.")
        if on:
            self._distance_label.setText("—")
            self.point_a.clear()
            self.point_b.clear()
        self.measure_toggled.emit(on)

    def set_measure_active(self, on: bool) -> None:
        self._measure_btn.setChecked(on)

    def on_first_point(self) -> None:
        self._hint.setText("Click the second point…")

    def show_distance(self, distance_m: float) -> None:
        self._distance_label.setText(f"Distance: {distance_m:.2f} m")
        self._hint.setText("Click again to start a new measurement.")
