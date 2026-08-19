"""SVLOG replay window — a full app-in-the-app.

Opened by the main window's "Open SVLOG" button. Architecture payoff:
because every live component consumes plain models from a signal bus,
this window simply instantiates a SECOND copy of the existing stack —
its own ``AppSignals``, ``MosaicService``, ``WaterfallService``,
``MapView`` + layers (satellite tiles, mosaic, trajectory), the
interactive ``WaterfallView`` and the standard ``RightPanel`` (same view
selector, colormap/priority/contrast/opacity controls, clear buttons and
distance tool as the main window) — and feeds it from the decoded log
instead of ROS. Zero changes to any reused component.

Two consumption modes (spec):

* **Render range** — the dual-handle time slider selects
  [start, end] within the mission (e.g. begin+5 s → end−10 s) and the
  whole selection is rasterized at once;
* **Replay** — the same selection is replayed against the wall clock at
  ×1/×2/×4/×8, driving the map, waterfall, trajectory and altitude
  exactly "as if live".

Plus **Save pictures from the log** (dataset generation): runs the
identical ``SeabedImager`` over every ping of the log and writes
``seabed_images_<logname>/`` (+ inner ``metadata/``) next to the log
file.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import numpy as np
from PySide6.QtCore import QRectF, Qt, QTimer
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (QDockWidget, QHBoxLayout, QLabel, QMainWindow,
                               QMessageBox, QProgressDialog, QComboBox,
                               QPushButton, QScrollArea, QStackedWidget,
                               QToolBar, QWidget)

from ..config.settings import AppConfig
from ..core.mosaic_service import MosaicService
from ..core.seabed_imager import generate_from_pings
from ..core.signals import AppSignals
from ..core.svlog import SvlogMission, load_svlog
from ..core.waterfall_service import WaterfallService
from ..mapping.coordinate_converter import CoordinateConverter
from ..mapping.tiles import TileFetcher
from ..models.robot_state import RobotState
from ..models.sonar import SonarPing
from . import right_panel as rp
from .main_window import PANEL_MIN_WIDTH
from .map_layers import (DetectionLayer, MeasureLayer, MosaicLayer,
                         TileLayer, TrajectoryLayer)
from .map_view import MapMode, MapView
from .range_slider import RangeSlider
from .right_panel import RightPanel
from .waterfall_view import WaterfallView

_SLIDER_STEPS = 1000
_REPLAY_TICK_MS = 33


def _fmt_t(seconds: float) -> str:
    m, s = divmod(max(0.0, seconds), 60.0)
    return f"{int(m):02d}:{s:04.1f}"


class ReplayWindow(QMainWindow):
    """Standalone viewer/replayer for one .svlog mission."""

    def __init__(self, mission: SvlogMission, config: AppConfig,
                 parent=None) -> None:
        super().__init__(parent)
        self._mission = mission
        self._config = config
        self.setWindowTitle(
            f"SVLOG replay — {mission.path.name}   "
            f"({mission.ping_count} pings, {_fmt_t(mission.duration_s)})")
        self.resize(1400, 900)

        # ---- second instance of the live stack -------------------------------
        self.signals = AppSignals()
        self.mosaic_service = MosaicService(config)
        self.waterfall_service = WaterfallService(config)
        self.map_view = MapView()
        self.waterfall_view = WaterfallView()
        self._stack = QStackedWidget()
        self._stack.addWidget(self.map_view)
        self._stack.addWidget(self.waterfall_view)
        self.setCentralWidget(self._stack)
        scene = self.map_view.scene()

        self.converter = CoordinateConverter(config.map.frame_yaw_offset_deg)
        tile_url = (config.map.satellite_url if config.map.use_satellite
                    else config.map.osm_url)
        self._tile_fetcher = TileFetcher(
            tile_url, Path(config.map.tile_cache_dir).expanduser(),
            config.map.max_concurrent_tile_requests, parent=self)
        self.tile_layer = TileLayer(scene, self._tile_fetcher, self.converter)
        self.mosaic_layer = MosaicLayer(scene)
        self.trajectory_layer = TrajectoryLayer(scene)
        self.detection_layer = DetectionLayer(scene)
        self.measure_layer = MeasureLayer(scene)

        # GPS origin from the log, if present -> tiles + GPS readouts work.
        if mission.origin is not None:
            self.converter.bind_origin(mission.origin[0], mission.origin[1],
                                       mission.origin_xy[0],
                                       mission.origin_xy[1])

        # ---- reused right panel (identical map options to the main window) ---
        self.right_panel = RightPanel()
        dock = QDockWidget("Tools")
        dock.setFeatures(QDockWidget.DockWidgetMovable)
        scroll = QScrollArea()
        scroll.setWidget(self.right_panel)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(PANEL_MIN_WIDTH)
        dock.setWidget(scroll)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

        # ---- replay control bar ------------------------------------------------
        self._build_replay_bar()

        # ---- replay engine state ----------------------------------------------
        self._cursor = 0                    # next event index during replay
        self._replay_t = 0.0                # mission time [s]
        self._last_wall: Optional[float] = None
        self._replay_timer = QTimer(self)
        self._replay_timer.setInterval(_REPLAY_TICK_MS)
        self._replay_timer.timeout.connect(self._replay_tick)

        self._connect()
        self._on_range_changed(*self._range.values())
        self.statusBar().showMessage(
            "Render range for an instant picture, or Replay to watch the "
            "mission live." + ("" if mission.origin else
                               "   (no GPS in this log: satellite layer "
                               "unavailable, local frame only)"))

    # ------------------------------------------------------------------ UI --
    def _build_replay_bar(self) -> None:
        bar = QToolBar("Replay")
        bar.setMovable(False)
        wrap = QWidget()
        lay = QHBoxLayout(wrap)
        lay.setContentsMargins(6, 2, 6, 2)

        self._t_lo_lbl = QLabel(_fmt_t(0))
        self._t_lo_lbl.setObjectName("valueLabel")
        self._range = RangeSlider(0, _SLIDER_STEPS)
        self._range.setMinimumWidth(320)
        self._t_hi_lbl = QLabel(_fmt_t(self._mission.duration_s))
        self._t_hi_lbl.setObjectName("valueLabel")

        self._render_btn = QPushButton("Render range")
        self._render_btn.setToolTip(
            "Rasterize everything between the two handles at once.")
        self._replay_btn = QPushButton("▶  Replay")
        self._replay_btn.setCheckable(True)
        self._speed = QComboBox()
        for s in (1, 2, 4, 8):
            self._speed.addItem(f"x{s}", s)
        self._pos_lbl = QLabel("")
        self._pos_lbl.setObjectName("valueLabel")
        self._ai_btn = QPushButton("Run AI")
        self._ai_btn.setToolTip(
            "Recreate every seabed picture from this log (256-row windows,\n"
            "50 % overlap, plus the truncated tail) and run the detection\n"
            "function on each; detections appear on the map and in the\n"
            "waterfall view, exactly as in the live window.")
        self._save_btn = QPushButton("Save pictures from the log")
        self._save_btn.setToolTip(
            "Generate every AI seabed image (+ metadata/) from this log\n"
            "into seabed_images_<logname>/ next to the file.")
        self._rosbag_btn = QPushButton("Save as rosbag")
        self._rosbag_btn.setToolTip(
            "Convert this .svlog to a rosbag2 (mcap) folder next to the\n"
            "log, using the team's svlog_to_rosbag converter.\n"
            "Requires a sourced ROS 2 environment.")

        lay.addWidget(QLabel("Window"))
        lay.addWidget(self._t_lo_lbl)
        lay.addWidget(self._range, 1)
        lay.addWidget(self._t_hi_lbl)
        lay.addWidget(self._render_btn)
        lay.addWidget(self._replay_btn)
        lay.addWidget(self._speed)
        lay.addWidget(self._pos_lbl)
        lay.addStretch(0)
        lay.addWidget(self._ai_btn)
        lay.addWidget(self._save_btn)
        lay.addWidget(self._rosbag_btn)
        bar.addWidget(wrap)
        self.addToolBar(Qt.BottomToolBarArea, bar)

    # ------------------------------------------------------------ wiring --
    def _connect(self) -> None:
        s = self.signals
        s.sonar_ping.connect(self._on_ping)
        s.robot_state.connect(self._on_state)
        self.mosaic_service.raster_updated.connect(
            lambda img, ext, cell: self.mosaic_layer.update(img, ext, cell))
        self.mosaic_service.cleared.connect(self.mosaic_layer.clear)
        self.waterfall_service.image_updated.connect(
            self.waterfall_view.on_image)
        self.waterfall_service.detections_updated.connect(
            self.waterfall_view.on_detections)

        p = self.right_panel
        p.zoom_in_clicked.connect(self.map_view.zoom_in)
        p.zoom_out_clicked.connect(self.map_view.zoom_out)
        p.center_robot_clicked.connect(self._center_robot)
        p.view_mode_changed.connect(self._on_view_mode)
        p.priority_changed.connect(self.mosaic_service.set_priority_mode)
        p.display_changed.connect(self._on_display)
        p.sss_opacity_changed.connect(self.mosaic_layer.set_opacity)
        p.clear_sss_clicked.connect(self._clear_outputs)
        p.clear_overlays_clicked.connect(self._clear_overlays)
        p.measure_toggled.connect(self._on_measure_toggled)

        self.map_view.point_clicked.connect(self._on_point_clicked)
        self.map_view.measure_started.connect(self._on_measure_started)
        self.map_view.measure_done.connect(self._on_measure_done)
        self.map_view.viewport_changed.connect(self.tile_layer.update_viewport)

        self._range.range_changed.connect(self._on_range_changed)
        self._render_btn.clicked.connect(self._render_range)
        self._replay_btn.toggled.connect(self._on_replay_toggled)
        self._ai_btn.clicked.connect(self._run_ai)
        self._save_btn.clicked.connect(self._save_pictures)
        self._rosbag_btn.clicked.connect(self._save_rosbag)

    # ------------------------------------------------------- event feeding --
    def _on_ping(self, ping: SonarPing) -> None:
        self.mosaic_service.on_sonar_ping(ping)
        self.waterfall_service.on_sonar_ping(ping)
        self.right_panel.altitude_plot.append(ping.water_depth)

    def _on_state(self, state: RobotState) -> None:
        self.trajectory_layer.add_pose(state.x, state.y, state.yaw)
        for card in (self.right_panel.point_a, self.right_panel.point_b):
            card.update_robot_position(state.x, state.y)

    # ------------------------------------------------------- range / labels --
    def _t_of(self, step: int) -> float:
        return step / _SLIDER_STEPS * self._mission.duration_s

    def _on_range_changed(self, lo: int, hi: int) -> None:
        self._t_lo_lbl.setText(_fmt_t(self._t_of(lo)))
        self._t_hi_lbl.setText(_fmt_t(self._t_of(hi)))

    # --------------------------------------------------------- render mode --
    def _clear_outputs(self) -> None:
        self.mosaic_service.clear()
        self.waterfall_service.clear()
        self.right_panel.altitude_plot.clear()

    def _clear_overlays(self) -> None:
        self.trajectory_layer.clear()
        self.detection_layer.clear()
        self.waterfall_service.clear_detections()
        self.measure_layer.clear()
        self.right_panel.set_measure_active(False)

    def _render_range(self) -> None:
        """Batch mode: rasterize the whole [start, end] selection at once."""
        self._replay_btn.setChecked(False)
        t0, t1 = (self._t_of(v) for v in self._range.values())
        self._clear_outputs()
        self.trajectory_layer.clear()
        self.waterfall_service.set_enabled(True)   # render even if hidden now
        events = [e for e in self._mission.events if t0 <= e[1] <= t1]
        progress = QProgressDialog("Rendering selection…", None, 0,
                                   len(events), self)
        progress.setWindowModality(Qt.WindowModal)
        for k, (kind, _t, obj) in enumerate(events):
            (self._on_ping if kind == "ping" else self._on_state)(obj)
            if k % 500 == 0:
                progress.setValue(k)
        progress.setValue(len(events))
        self.waterfall_service.set_enabled(
            self._stack.currentWidget() is self.waterfall_view)
        self._pos_lbl.setText(f"rendered {_fmt_t(t0)} → {_fmt_t(t1)}")
        if events and self.trajectory_layer.current_pos() is not None:
            self.map_view.center_on_world(*self.trajectory_layer.current_pos())

    # --------------------------------------------------------- replay mode --
    def _on_replay_toggled(self, on: bool) -> None:
        if on:
            t0, _t1 = (self._t_of(v) for v in self._range.values())
            self._clear_outputs()
            self.trajectory_layer.clear()
            self._replay_t = t0
            self._cursor = next(
                (i for i, e in enumerate(self._mission.events)
                 if e[1] >= t0), len(self._mission.events))
            self._last_wall = time.monotonic()
            self._replay_btn.setText("⏸  Pause")
            self._replay_timer.start()
        else:
            self._replay_timer.stop()
            self._replay_btn.setText("▶  Replay")

    def _replay_tick(self) -> None:
        now = time.monotonic()
        dt = (now - self._last_wall) * float(self._speed.currentData())
        self._last_wall = now
        self._replay_t += dt
        _t0, t1 = (self._t_of(v) for v in self._range.values())
        events = self._mission.events
        while (self._cursor < len(events)
               and events[self._cursor][1] <= min(self._replay_t, t1)):
            kind, _t, obj = events[self._cursor]
            (self._on_ping if kind == "ping" else self._on_state)(obj)
            self._cursor += 1
        self._pos_lbl.setText(
            f"{_fmt_t(min(self._replay_t, t1))} / {_fmt_t(t1)}")
        if self._replay_t >= t1 or self._cursor >= len(events):
            self._replay_btn.setChecked(False)
            self._pos_lbl.setText(f"finished at {_fmt_t(t1)}")

    # -------------------------------------------------- dataset generation --
    def _save_pictures(self) -> None:
        out = (self._mission.path.parent
               / f"seabed_images_{self._mission.path.stem}")
        pings = self._mission.pings
        progress = QProgressDialog(
            f"Generating seabed images into {out.name}…", None, 0, 100, self)
        progress.setWindowModality(Qt.WindowModal)
        n = generate_from_pings(
            pings, out, self._config,
            progress=lambda f: progress.setValue(int(f * 100)))
        progress.setValue(100)
        QMessageBox.information(
            self, "Seabed images",
            f"{n} images written to\n{out}\n(+ metadata/ inside).")

    # ------------------------------------------------------------- Run AI --
    def _run_ai(self) -> None:
        """Recreate every seabed picture from the log, run the detection
        function on each, and display the results exactly as live: markers
        on the map's DetectionLayer and on the waterfall overlay. The
        window is blocked by a modal progress bar while it runs."""
        from ..core.seabed_imager import SeabedImager
        from ..models.detection import Detection

        imager = SeabedImager(self._config)      # dummy analyzer for now
        results: list = []
        imager.image_ready.connect(results.append)

        pings = self._mission.pings
        progress = QProgressDialog("Running AI on the mission…", None,
                                   0, len(pings), self)
        progress.setWindowModality(Qt.WindowModal)   # freezes the app
        progress.setMinimumDuration(0)
        for k, ping in enumerate(pings):
            imager.on_sonar_ping(ping)
            if k % 100 == 0:
                progress.setValue(k)
        imager.flush()                               # truncated tail image
        progress.setValue(len(pings))

        # Make sure there is imagery under the markers: if nothing has
        # been rendered yet, render the current selection first (renders
        # clear overlays, so this must happen BEFORE adding detections).
        if not np.isfinite(self.mosaic_service._grid.render()).any():
            self._render_range()

        self.detection_layer.clear()
        self.waterfall_service.clear_detections()
        n_det = 0
        for image in results:
            for k, det in enumerate(image.detections):
                t_row = float(det.get("t_s", image.row_t[-1]))
                self.detection_layer.upsert(Detection(
                    uid=1_000_000 + image.image_id * 16 + k, t=t_row,
                    x=float(det["world"][0]), y=float(det["world"][1]),
                    class_name=det["class_name"],
                    confidence=float(det["confidence"]), extent_m=1.0))
                self.waterfall_service.add_detection(
                    t_row, float(det["world"][0]), float(det["world"][1]),
                    det["class_name"])
                n_det += 1
        self.detection_layer.set_visible(True)
        # Refresh the waterfall overlay against the freshly rendered buffer.
        was = self.waterfall_service._enabled
        self.waterfall_service.set_enabled(True)
        self.waterfall_service.set_enabled(was or
                                           self._stack.currentWidget()
                                           is self.waterfall_view)
        self.statusBar().showMessage(
            f"AI pass: {len(results)} images "
            f"(incl. truncated tail), {n_det} detections — shown on the "
            f"map and in the waterfall view.", 15000)

    # ------------------------------------------------------ rosbag export --
    def _save_rosbag(self) -> None:
        """Convert the loaded .svlog to a rosbag2 folder next to it,
        using the verbatim team converter in blueboat_gcs/tools/."""
        from PySide6.QtWidgets import QInputDialog
        default = f"bag_{self._mission.path.stem}"
        name, ok = QInputDialog.getText(
            self, "Save as rosbag",
            "Output folder name (created next to the .svlog):",
            text=default)
        if not ok or not name.strip():
            return
        out = self._mission.path.parent / name.strip()

        # Import the verbatim tools (they use flat imports for
        # svlog_helper, so their directory goes on sys.path).
        import sys as _sys
        tools_dir = str(Path(__file__).resolve().parent.parent / "tools")
        if tools_dir not in _sys.path:
            _sys.path.insert(0, tools_dir)
        try:
            import svlog_to_rosbag as s2r
        except ImportError as exc:
            QMessageBox.critical(
                self, "Save as rosbag",
                "The rosbag converter needs a sourced ROS 2 environment\n"
                "(rclpy, rosbag2_py, blueboat_interfaces, mavros_msgs,\n"
                f"geographic_msgs).\n\nImport error: {exc}")
            return

        # Same rename-if-exists behaviour as the converter's main().
        if out.exists():
            i = 2
            candidate = out.with_name(f"{out.stem}_{i}{out.suffix}")
            while candidate.exists():
                i += 1
                candidate = out.with_name(f"{out.stem}_{i}{out.suffix}")
            out = candidate

        import rclpy
        from rosbag2_py import (ConverterOptions, SequentialWriter,
                                StorageOptions)
        we_inited = False
        try:
            rclpy.init()
            we_inited = True
        except RuntimeError:
            pass                    # app's RosManager already initialized it
        try:
            writer = SequentialWriter()
            writer.open(
                StorageOptions(uri=str(out), storage_id="mcap"),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
            conv = s2r.Converter(writer)
            conv.setup_topics()
            data = self._mission.path.read_bytes()
            packets = list(s2r.walk_packets(data))
            progress = QProgressDialog(f"Converting to {out.name}…", None,
                                       0, len(packets), self)
            progress.setWindowModality(Qt.WindowModal)
            progress.setMinimumDuration(0)
            for k, packet in enumerate(packets):
                conv.handle_packet(packet)
                if k % 500 == 0:
                    progress.setValue(k)
            del writer              # close cleanly (converter main() does this)
            progress.setValue(len(packets))
            counts = "\n".join(f"  {k}: {v}"
                               for k, v in sorted(conv.counts.items()) if v)
            QMessageBox.information(
                self, "Save as rosbag",
                f"Converted {len(packets)} packets to\n{out}\n\n{counts}")
        except Exception as exc:               # noqa: BLE001 — surfaced to user
            QMessageBox.critical(self, "Save as rosbag",
                                 f"Conversion failed:\n{exc}")
        finally:
            if we_inited:
                rclpy.shutdown()

    # ------------------------------------------------------------- helpers --
    def _on_view_mode(self, mode: str) -> None:
        waterfall = (mode == rp.VIEW_WATERFALL)
        self._stack.setCurrentWidget(self.waterfall_view if waterfall
                                     else self.map_view)
        self.waterfall_service.set_enabled(waterfall)

    def _on_display(self, settings) -> None:
        self.mosaic_service.set_display(settings)
        self.waterfall_service.set_display(settings)

    def _center_robot(self) -> None:
        pos = self.trajectory_layer.current_pos()
        if pos is not None:
            self.map_view.center_on_world(*pos)

    def _on_measure_toggled(self, on: bool) -> None:
        self.map_view.set_mode(MapMode.MEASURE if on else MapMode.NAVIGATE)
        if not on:
            self.measure_layer.clear()

    def _on_point_clicked(self, x: float, y: float) -> None:
        gps = self.converter.local_to_gps(x, y)
        gps_txt = f"   |   {gps[0]:.7f}, {gps[1]:.7f}" if gps else ""
        self.statusBar().showMessage(
            f"Point:  x {x:+.2f} m,  y {y:+.2f} m{gps_txt}", 15000)

    def _on_measure_started(self, x: float, y: float) -> None:
        self.measure_layer.show_first(x, y)
        self.right_panel.on_first_point()
        self.right_panel.point_a.set_point(x, y,
                                           self.converter.local_to_gps(x, y))
        self.right_panel.point_b.clear()

    def _on_measure_done(self, x1, y1, x2, y2, dist) -> None:
        self.measure_layer.show_measurement((x1, y1), (x2, y2), dist)
        self.right_panel.point_b.set_point(
            x2, y2, self.converter.local_to_gps(x2, y2))
        self.right_panel.show_distance(dist)

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802
        self._replay_timer.stop()
        super().closeEvent(event)


# ---------------------------------------------------------------------------
def open_svlog_dialog(parent, config: AppConfig) -> Optional[ReplayWindow]:
    """File dialog + progress-loaded ReplayWindow (used by main_window)."""
    from PySide6.QtWidgets import QFileDialog
    path, _f = QFileDialog.getOpenFileName(
        parent, "Open SVLOG", str(Path(config.data_root).expanduser()),
        "SonarView logs (*.svlog);;All files (*)")
    if not path:
        return None
    progress = QProgressDialog(f"Reading {Path(path).name}…", None,
                               0, 100, parent)
    progress.setWindowModality(Qt.WindowModal)
    progress.setMinimumDuration(0)
    try:
        mission = load_svlog(
            Path(path), progress=lambda f: progress.setValue(int(f * 100)))
    except (OSError, ValueError) as exc:
        progress.close()
        QMessageBox.critical(parent, "Open SVLOG",
                             f"Could not read the log:\n{exc}")
        return None
    progress.close()
    if mission.ping_count == 0:
        QMessageBox.warning(
            parent, "Open SVLOG",
            "No processable ping pairs found in this log\n"
            "(missing pose data before pings, or FBR never bootstrapped).")
        return None
    win = ReplayWindow(mission, config, parent=parent)
    win.show()
    return win
