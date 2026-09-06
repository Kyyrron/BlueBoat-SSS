"""Main window — composition root of the GUI.

All wiring between data sources (signal bus), services (mosaic,
converter, launcher) and widgets (map, panels, toolbar) lives here and
only here; the individual modules know nothing about each other. To plug
a new ROS topic later: add a listener + model + signal, then connect it
in ``_connect_signals`` — nothing else changes.
"""

from __future__ import annotations

import time
from dataclasses import replace as dc_replace
from pathlib import Path
from typing import Optional

import numpy as np

from PySide6.QtCore import QRectF, Qt, QTimer
from PySide6.QtGui import QCloseEvent, QImage
from PySide6.QtWidgets import (QDockWidget, QMainWindow, QScrollArea,
                               QStackedWidget)

from ..config.settings import AppConfig
from ..core.geo_service import GeoService
from ..core.mosaic_service import MosaicService
from ..core.recording_session import RecordingManager
from ..core.signals import AppSignals
from ..core.waterfall_service import WaterfallService
from ..mapping.tiles import TileFetcher
from ..models.detection import Detection, PingerFix
from ..models.path import PlannedPath
from ..models.robot_state import RobotState
from ..models.sonar import SonarPing
from ..utils.pose_alignment import (FrozenPoseDetector, HeadingPolicy,
                                    robot_to_world)
from . import left_panel as lp
from . import right_panel as rp
from .left_panel import LeftPanel
from .log_console import LogConsole
from ..core.seabed_imager import waterfall_pixel_to_world
from ..utils.geodesy import format_latlon
from .map_layers import (DetectionLayer, MeasureLayer, MosaicLayer,
                         PingerLayer, PlannedPathLayer, SelectionLayer,
                         SwathLayer, TileLayer, TrajectoryLayer, WorldRoot)
from .map_view import MapMode, MapView
from .right_panel import RightPanel
from .toolbar import AcquisitionToolbar
from .waterfall_view import WaterfallView

#: Base minimum width of the left/right dock panels [px] — bump here if
#: labels ever get cramped.
PANEL_MIN_WIDTH = 270


class MainWindow(QMainWindow):
    """BlueBoat GCS main window."""

    def __init__(self, config: AppConfig, signals: AppSignals,
                 mosaic_service: MosaicService,
                 acquisition_controller,  # PipelineLauncher or Simulator
                 ) -> None:
        super().__init__()
        self._config = config
        self._signals = signals
        self._mosaic_service = mosaic_service
        self._acquisition = acquisition_controller
        self._mission_start: Optional[float] = None
        self._last_robot_state: Optional[RobotState] = None

        self.setWindowTitle("BlueBoat GCS — Side-Scan Sonar Survey")
        self.resize(1500, 950)

        # ---- central: stacked Mosaic view / Waterfall view --------------------
        self.map_view = MapView()
        self.waterfall_view = WaterfallView()
        self._stack = QStackedWidget()
        self._stack.addWidget(self.map_view)        # index 0 = VIEW_MOSAIC
        self._stack.addWidget(self.waterfall_view)  # index 1 = VIEW_WATERFALL
        self.setCentralWidget(self._stack)
        scene = self.map_view.scene()

        # ONE display model for the waterfall, the AI pictures and the
        # mosaic of this window (core/display_model.py): fed by the
        # waterfall service, frozen after the warm-up, shared by all three
        # so every picture maps dB to grey identically.
        from ..core.display_model import DisplayModel
        self.display_model = DisplayModel(config)
        self.waterfall_service = WaterfallService(config, self.display_model)
        mosaic_service.set_model(self.display_model)
        self.recording = RecordingManager(config, signals,
                                          mosaic_service,
                                          self.waterfall_service)
        # Live AI seabed imaging (waterfall domain, core/seabed_imager.py):
        # every stride rows -> image + metadata + dummy analysis; written
        # to <session>/seabed_images while a recording session is active.
        from ..core.seabed_imager import SeabedImager
        self.seabed_imager = SeabedImager(config, model=self.display_model)
        self._frozen_detector = FrozenPoseDetector(
            eps_m=config.alignment.frozen_epsilon_m,
            after=config.alignment.frozen_after_pings)
        self._replay_windows: list = []      # keep references alive

        # Telemetry staleness watchdog (robot synchronization): if no
        # RobotState arrives for a while — pipeline stopped, mavros down,
        # radio dropout — the marker dims and a trajectory break is armed
        # so that WHENEVER messages resume (START pressed or not) the
        # pose re-syncs instantly with no phantom segment.
        self._telemetry_stale = False
        self._last_state_walltime: Optional[float] = None
        self._last_pinger_walltime: Optional[float] = None
        self._stale_after_s = 3.0
        self._watchdog = QTimer(self)
        self._watchdog.setInterval(1000)
        self._watchdog.timeout.connect(self._check_telemetry_staleness)
        self._watchdog.start()

        # GPS anchor (ported from BlueBoat-MCS): the scene is EN metres
        # about the first accepted fix; every world-anchored layer lives
        # under one WorldRoot whose position is the odom->EN translation
        # and whose visibility is the readiness gate. The compass is the
        # live heading reference (alignment.heading_source).
        self.geo = GeoService(config.geo, config.map.require_gps_anchor)
        self._heading = HeadingPolicy(config.alignment.compass_stale_s)
        tile_url = (config.map.satellite_url if config.map.use_satellite
                    else config.map.osm_url)
        self._tile_fetcher = TileFetcher(
            tile_url, Path(config.map.tile_cache_dir).expanduser(),
            config.map.max_concurrent_tile_requests, parent=self)
        # Tiles are pinned to (lat0, lon0), never to the translation —
        # imagery must not slide when the anchor refits.
        self.tile_layer = TileLayer(scene, self._tile_fetcher,
                                    lambda: self.geo.origin)
        self.world_root = WorldRoot(scene)
        root = self.world_root.item
        self.mosaic_layer = MosaicLayer(scene, root)
        self.trajectory_layer = TrajectoryLayer(scene, root)
        self.planned_path_layer = PlannedPathLayer(scene, root)
        self.swath_layer = SwathLayer(scene, root)
        self.detection_layer = DetectionLayer(scene, root)
        self.pinger_layer = PingerLayer(scene, root)
        self.selection_layer = SelectionLayer(scene, root)
        self.measure_layer = MeasureLayer(scene)   # EN space: distances only
        self.world_root.set_ready(self.geo.ready)
        self.map_view.set_waiting(not self.geo.ready)
        self.map_view.set_interactive(self.geo.ready)

        # ---- side panels ------------------------------------------------------
        self.left_panel = LeftPanel()
        self.addDockWidget(Qt.LeftDockWidgetArea,
                           self._dock("Mission", self.left_panel))
        self.right_panel = RightPanel(acquisition_range=(
            config.acquisition.range_min_m, config.acquisition.range_max_m),
            config=config)
        self.addDockWidget(Qt.RightDockWidgetArea,
                           self._dock("Tools", self.right_panel))

        # ---- bottom toolbar + console + status bar ----------------------------------
        self.toolbar = AcquisitionToolbar(self)
        self.addToolBar(Qt.BottomToolBarArea, self.toolbar)
        self.statusBar().showMessage("Ready.")

        # Embedded application console (collapsed by default; the
        # "Console" toolbar button or dragging the dock expands it).
        self.console = LogConsole()
        self._console_dock = QDockWidget("Console")
        self._console_dock.setFeatures(QDockWidget.DockWidgetMovable
                                       | QDockWidget.DockWidgetClosable)
        self._console_dock.setWidget(self.console)
        self.addDockWidget(Qt.BottomDockWidgetArea, self._console_dock)
        self._console_dock.hide()
        self._console_dock.visibilityChanged.connect(
            self.toolbar.set_console_checked)

        # Live visualization gate: data may flow at any time (the
        # pipeline starts with the application), but the map/waterfall
        # only update after START.
        self._viz_enabled = False

        self._mission_timer = QTimer(self)
        self._mission_timer.setInterval(1000)
        self._mission_timer.timeout.connect(self._update_mission_time)
        self._mission_timer.start()

        # Swath-line coalescing: the range line is a scene mutation, and
        # mutating it per ping forces a full-viewport repaint at ping
        # rate. Latest-wins, flushed at the mosaic render cadence.
        self._pending_swath = None
        self._swath_timer = QTimer(self)
        self._swath_timer.setInterval(
            max(1, int(1000 / config.mosaic.render_hz)))
        self._swath_timer.timeout.connect(self._flush_swath)
        self._swath_timer.start()

        self._connect_signals()

    @staticmethod
    def _dock(title: str, widget) -> QDockWidget:
        dock = QDockWidget(title)
        dock.setFeatures(QDockWidget.DockWidgetMovable)
        scroll = QScrollArea()
        scroll.setWidget(widget)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(PANEL_MIN_WIDTH)
        dock.setWidget(scroll)
        return dock

    # ---- wiring ----------------------------------------------------------------
    def _connect_signals(self) -> None:
        s = self._signals

        # Data streams (queued from the ROS thread) -> services & layers.
        # SSS ingestion is gated on the START button (viz gate).
        s.sonar_ping.connect(self._on_sonar_ping_data)
        s.robot_state.connect(self._on_robot_state)
        s.detection.connect(self._on_detection)
        s.pinger_fix.connect(self._on_pinger)
        s.planned_path.connect(self._on_planned_path)

        # GPS anchor: pair fixes with fresh odom, follow the fit.
        s.gps_fix.connect(self.geo.on_gps_fix)
        s.compass_heading.connect(self._heading.update_compass)
        self.geo.fit_changed.connect(
            lambda fit: self.world_root.set_translation(fit.tx, fit.ty))
        self.geo.anchored.connect(self._on_anchored)
        self.geo.pairing_blocked.connect(
            lambda msg: self._signals.status_message.emit(msg))
        s.pipeline_state.connect(self.toolbar.on_pipeline_state)
        s.pipeline_state.connect(self._on_pipeline_state)
        s.status_message.connect(
            lambda msg: self.statusBar().showMessage(msg, 8000))
        s.status_message.connect(
            lambda msg: self.console.append_line("app", msg))
        s.log_line.connect(self.console.append_line)
        self._mosaic_service.raster_updated.connect(self._on_raster)
        self.waterfall_service.layout_changed.connect(
            self.waterfall_view.on_layout)
        self.waterfall_service.tile_updated.connect(
            self.waterfall_view.on_tile)
        self.waterfall_view.tiles_requested.connect(
            self.waterfall_service.request_rows)
        self.waterfall_service.detections_updated.connect(
            self.waterfall_view.on_detections)
        self.waterfall_view.point_selected.connect(self._on_waterfall_point)

        # Map interactions.
        self.map_view.point_clicked.connect(self._on_point_clicked)
        self.map_view.measure_started.connect(self._on_measure_started)
        self.map_view.measure_done.connect(self._on_measure_done)
        self.map_view.viewport_changed.connect(self._on_viewport_changed)

        # Left panel toggles.
        self.left_panel.layer_toggled.connect(self._on_layer_toggled)

        # Right panel tools.
        self.right_panel.zoom_in_clicked.connect(self.map_view.zoom_in)
        self.right_panel.zoom_out_clicked.connect(self.map_view.zoom_out)
        self.right_panel.center_robot_clicked.connect(self._center_robot)
        self.right_panel.measure_toggled.connect(self._on_measure_toggled)
        self.right_panel.view_mode_changed.connect(self._on_view_mode)
        self.right_panel.priority_changed.connect(
            self._mosaic_service.set_priority_mode)
        self.right_panel.priority_changed.connect(
            self.recording.note_priority_mode)
        self.right_panel.display_changed.connect(self._on_display_changed)
        self.right_panel.resolution_changed.connect(self._on_resolution_changed)
        self.right_panel.depth_mode_changed.connect(self._on_depth_mode_changed)
        self.right_panel.clear_overlays_clicked.connect(
            self._on_clear_overlays)
        self.right_panel.clear_sss_clicked.connect(self._on_clear_sss)
        self.right_panel.sss_opacity_changed.connect(
            self.mosaic_layer.set_opacity)
        self.right_panel.range_apply_requested.connect(self._on_range_apply)
        s.sonar_params_result.connect(self._on_sonar_params_result)
        self._mosaic_service.cleared.connect(self.mosaic_layer.clear)
        self.recording.recording_state.connect(
            self.toolbar.on_recording_state)
        self.seabed_imager.image_ready.connect(self._on_seabed_image)
        self.toolbar.open_svlog_clicked.connect(self._on_open_svlog)

        # Bottom toolbar.
        self.toolbar.start_clicked.connect(self._on_start)
        self.toolbar.stop_clicked.connect(self._on_stop)
        self.toolbar.record_toggled.connect(self._on_record_toggled)
        self.toolbar.console_toggled.connect(self._console_dock.setVisible)

        # Apply the initial (all-disabled) layer states to the scene so
        # checkboxes and items agree at startup.
        for key, cb in self.left_panel._checks.items():
            self._on_layer_toggled(key, cb.isChecked())

    # ---- data slots (GUI thread) ---------------------------------------------------
    def _on_raster(self, image: QImage, extent: tuple, cell: float) -> None:
        self.mosaic_layer.update(image, extent, cell)

    def _on_sonar_ping_data(self, ping: SonarPing) -> None:
        """Single entry point for SSS data, gated on the START button.

        The pipeline runs from application startup, so pings may arrive
        at any time — but the mosaic, waterfall, range line and altitude
        plot only update while live visualization is enabled.
        """
        if not self._viz_enabled:
            return
        self._apply_backpressure(ping)
        ping = self._align_ping_pose(ping)
        self._mosaic_service.on_sonar_ping(ping)
        self.waterfall_service.on_sonar_ping(ping)
        self.seabed_imager.on_sonar_ping(ping)
        self.right_panel.altitude_plot.append(ping.water_depth)
        # Sonar range line: extent taken from the actual samples, so it
        # tracks the current sonar configuration automatically. Stored
        # latest-wins and flushed by _swath_timer, never drawn per ping.
        if ping.y_local.size:
            self._pending_swath = (ping.robot_x, ping.robot_y, ping.yaw,
                                   float(np.abs(ping.y_local).max()))

    def _flush_swath(self) -> None:
        if self._pending_swath is not None:
            self.swath_layer.update(*self._pending_swath)
            self._pending_swath = None

    def _on_planned_path(self, path: PlannedPath) -> None:
        # A new message fully replaces the previously displayed path.
        self.planned_path_layer.set_path(path.points)

    def _on_robot_state(self, state: RobotState) -> None:
        self._last_robot_state = state
        self._last_state_walltime = time.monotonic()
        self.geo.on_robot_state(state)          # freshest pose for pairing
        self._heading.update_odom(self._last_state_walltime, state.yaw)
        if self._telemetry_stale:
            # Telemetry resumed: the break armed by the watchdog makes
            # this pose start a fresh polyline segment — instant re-sync.
            self._telemetry_stale = False
            self.trajectory_layer.set_stale(False)
            self.statusBar().showMessage("Telemetry resumed.", 4000)
        self.left_panel.on_robot_state(state)
        marker_yaw = self._heading.heading(time.monotonic())
        self.trajectory_layer.add_pose(
            state.x, state.y,
            state.yaw if marker_yaw is None else marker_yaw)
        # Live distances (#6/#7): selected point + both measure points +
        # the pinger panel update continuously as the robot moves.
        for card in (self.left_panel.coordinate_card,
                     self.right_panel.point_a, self.right_panel.point_b):
            card.update_robot_position(state.x, state.y)

    def _on_anchored(self, lat0: float, lon0: float) -> None:
        """The odom<->GPS anchor became valid: open the map."""
        self.world_root.set_ready(True)
        self.map_view.set_waiting(False)
        self.map_view.set_interactive(True)
        self._refresh_tiles()
        self._signals.geo_anchored.emit(lat0, lon0)
        rms = self.geo.rms_m
        self.statusBar().showMessage(
            f"Map anchored at {lat0:.6f}, {lon0:.6f}"
            + (f" (rms {rms:.1f} m)" if rms else ""), 8000)

    def _apply_backpressure(self, ping: SonarPing) -> None:
        """The sonar_ping queue has no backpressure: when the GUI thread
        falls behind, pings pile up in Qt's event queue and latency
        grows without bound ("slower and slower, then dies"). Rows always
        enter the buffers; only *rendering* pauses until the queue has
        drained, and the operator is told once per episode."""
        limit = int(getattr(self._config.mosaic, "ping_lag_throttle", 0))
        latest = int(getattr(self._signals, "sonar_latest_seq", 0))
        if limit <= 0 or not ping.seq or not latest:
            return
        lag = latest - ping.seq
        behind = lag > limit
        if behind != getattr(self, "_ping_lag_throttled", False):
            self._ping_lag_throttled = behind
            self._mosaic_service.throttle(behind)
            self.waterfall_service.throttle(behind)
            if behind:
                self._signals.status_message.emit(
                    f"SONAR: display {lag} pings behind the stream -- "
                    "rendering paused until it catches up (no data lost)")

    def _on_detection(self, det: Detection) -> None:
        self.left_panel.on_detection(det)
        self.detection_layer.upsert(det)
        # Same detection on the waterfall view (row found by ping time).
        self.waterfall_service.add_detection(det.t, det.x, det.y,
                                             det.class_name)

    # ---- display settings ---------------------------------------------------
    def _on_resolution_changed(self, cell_m: float) -> None:
        """0 = auto (best the data supports, kept up to date), else a
        fixed GSD. Either way accumulated data is preserved."""
        if cell_m <= 0.0:
            self._mosaic_service.enable_auto_resolution()
            self.statusBar().showMessage(
                "Mosaic resolution: auto (from the sonar's sample spacing).", 6000)
        else:
            self._mosaic_service.set_fixed_cell_size(cell_m)

    def _on_depth_mode_changed(self, mode: str, manual_m: float) -> None:
        self._config.depth.mode = mode
        self._config.depth.manual_m = manual_m
        self.statusBar().showMessage(
            f"Depth compensation: {mode}"
            + (f" ({manual_m:.1f} m)" if mode == "manual" else ""), 6000)

    # ---- sea-trial pose alignment ------------------------------------------------
    def _align_ping_pose(self, ping: SonarPing) -> SonarPing:
        """Re-stamp the ping's pose from GCS telemetry when required.

        pose_source "embedded": trust the ping. "gcs": always re-stamp.
        "auto" (default): re-stamp only while the embedded poses are
        frozen at the origin although GCS telemetry shows the boat
        elsewhere — the live '(0,0) pings with GPS on' pathology (the
        processor's /blueboat/odom is zeroed or clock-mismatched; rosbag
        replays are sane because their odom is synthesized). A console
        warning identifies the robot-side root cause once.
        """
        mode = self._config.alignment.pose_source
        ping = self._restamp_ping_heading(ping)
        if mode == "embedded":
            return ping
        state = self._last_robot_state
        if mode == "auto":
            engaged_before = self._frozen_detector.engaged
            engaged = self._frozen_detector.update(
                ping.robot_x, ping.robot_y,
                None if state is None else state.x,
                None if state is None else state.y)
            if engaged and not engaged_before:
                self._signals.log_line.emit(
                    "app",
                    "ALIGNMENT: ProcessedSSSPing poses are frozen at (0,0) "
                    "while the boat moves — re-stamping pings from GCS "
                    "telemetry. Root cause is robot-side: /blueboat/odom "
                    "seen by sss_processor_node is zeroed or its stamps "
                    "are on a different clock than the sonar profiles "
                    "(see HANDOVER 'Sea-trial pose alignment').")
            if not engaged:
                return ping
        if state is None or self._telemetry_stale:
            # No replacement pose, or only a stale one: re-stamping a
            # frozen ping from an equally frozen GCS pose would recreate
            # the very pile-up this path exists to break.
            return ping
        return self._restamp_ping_heading(dc_replace(
            ping, robot_x=state.x, robot_y=state.y, yaw=state.yaw))

    def _restamp_ping_heading(self, ping: SonarPing) -> SonarPing:
        """Live heading fix (alignment.heading_source: compass).

        The field bug: the embedded odom-quaternion yaw drew SSS pings
        misaligned with the boat's true heading, while replay (mavlink
        ATTITUDE yaw) was correct. The compass is the true-north
        reference, so live pings are re-stamped from it — replay never
        passes through here.
        """
        if self._config.alignment.heading_source != "compass":
            return ping
        yaw = self._heading.heading(time.monotonic())
        if yaw is None:
            return ping
        return dc_replace(ping, yaw=yaw)

    def _on_pinger(self, fix: PingerFix) -> None:
        # Frame handling (alignment.pinger_frame): "auto" trusts the
        # frame the listener derived from the wire shape (3-vector =
        # body, 2-vector = world); "robot"/"world" force one
        # interpretation. Body-frame fixes rotate [x fwd, y port]
        # through the robot pose nearest the fix.
        mode = self._config.alignment.pinger_frame
        frame = (fix.frame if mode == "auto"
                 else ("body" if mode == "robot" else "world"))
        if frame == "body":
            state = self._last_robot_state
            if state is None:
                return                      # cannot place it yet
            yaw = self._heading.heading(time.monotonic())
            wx, wy = robot_to_world(fix.x, fix.y, state.x, state.y,
                                    state.yaw if yaw is None else yaw)
            fix = dc_replace(fix, x=wx, y=wy, frame="world")
        self._last_pinger_walltime = time.monotonic()
        self.pinger_layer.update(fix)
        self.left_panel.on_pinger(fix.x, fix.y,
                                  self.geo.local_to_gps(fix.x, fix.y))

    # ---- AI seabed imaging (live) -----------------------------------------------
    def _on_seabed_image(self, image) -> None:
        """Every completed seabed image: publish the analysis (metadata +
        detections, never pixels) and show detections on the map."""
        payload = image.analysis_json(getattr(image, "_png_path", None),
                                      getattr(image, "_metadata_path", None))
        # PipelineLauncher exposes the ros manager; Simulator has none.
        ros = getattr(self._acquisition, "_ros", None)
        if ros is not None:
            ros.publish_seabed_analysis(payload)
        self.console.append_line(
            "app", f"seabed image {image.image_id}: "
                   f"{len(image.detections)} detection(s) published")
        for k, det in enumerate(image.detections):
            t_row = float(det.get("t_s", image.row_t[-1]))
            self._signals.detection.emit(Detection(
                uid=1_000_000 + image.image_id * 16 + k,
                t=t_row,
                x=float(det["world"][0]), y=float(det["world"][1]),
                class_name=det["class_name"],
                confidence=float(det["confidence"]), extent_m=1.0))

    def _on_open_svlog(self) -> None:
        from .replay_window import open_svlog_dialog
        win = open_svlog_dialog(self, self._config)
        if win is not None:
            self._replay_windows.append(win)

    # ---- view mode / display -----------------------------------------------------
    def _on_view_mode(self, mode: str) -> None:
        waterfall = (mode == rp.VIEW_WATERFALL)
        self._stack.setCurrentWidget(self.waterfall_view if waterfall
                                     else self.map_view)
        # The waterfall renders only while shown (saves a raster pipeline
        # when the operator lives in the mosaic view, and vice versa the
        # mosaic layers keep updating cheaply in the background).
        self.waterfall_service.set_enabled(waterfall)

    def _check_telemetry_staleness(self) -> None:
        # Pinger staleness: a marker with no recent fix is hidden — the
        # last known position of a silent pinger is not a position.
        stale_s = self._config.alignment.pinger_stale_after_s
        if (stale_s > 0 and self._last_pinger_walltime is not None
                and time.monotonic() - self._last_pinger_walltime > stale_s):
            self._last_pinger_walltime = None
            self.pinger_layer.clear()      # re-shows on the next fix
            self.statusBar().showMessage(
                "Pinger signal lost — marker hidden.", 6000)
        if self._telemetry_stale or self._last_state_walltime is None:
            return
        if time.monotonic() - self._last_state_walltime > self._stale_after_s:
            self._telemetry_stale = True
            self.trajectory_layer.set_stale(True)
            self.trajectory_layer.begin_new_segment()
            self._mosaic_service.reset_tracking()  # no interp across gaps
            self.statusBar().showMessage(
                "No telemetry — displayed robot pose is stale.", 6000)

    def _on_display_changed(self, settings) -> None:
        # One settings object drives both views identically.
        self._mosaic_service.set_display(settings)
        self.waterfall_service.set_display(settings)
        self.recording.note_display_settings(settings)

    def _on_clear_sss(self) -> None:
        """'Clear SSS data': sonar data only; every other layer, the map
        position and the zoom level are untouched (nothing here touches
        the view transform). New pings keep accumulating immediately."""
        self._mosaic_service.clear()          # emits cleared -> layer wipes
        self.waterfall_service.clear()        # emits null image -> view wipes
        self.display_model.reset()            # a fresh mission: re-warm
        self.statusBar().showMessage(
            "SSS data cleared — overlays, map position and zoom preserved.",
            8000)

    # ---- overlay clearing ----------------------------------------------------------
    def _on_clear_overlays(self) -> None:
        """'Clear currently displayed data': clears every overlay that is
        *currently shown*, preserves the mosaic and any hidden overlay,
        and keeps displaying newly received data immediately."""
        cleared = []
        if self.left_panel.is_layer_enabled(lp.LAYER_TRAJECTORY):
            self.trajectory_layer.clear()
            cleared.append("trajectory")
        if self.left_panel.is_layer_enabled(lp.LAYER_DETECTIONS):
            self.detection_layer.clear()
            self.left_panel.reset_detections()
            self.waterfall_service.clear_detections()
            cleared.append("detections")
        if self.left_panel.is_layer_enabled(lp.LAYER_PINGER):
            self.pinger_layer.clear()
            cleared.append("pinger")
        if self.left_panel.is_layer_enabled(lp.LAYER_PLANNED_PATH):
            self.planned_path_layer.clear()
            cleared.append("planned path")
        # Measurements, the selected point and the range line have no
        # visibility checkbox: they are on screen, hence "currently
        # displayed" -> cleared.
        self.selection_layer.clear()
        self.waterfall_view.set_selected(None)
        self.measure_layer.clear()
        self.right_panel.set_measure_active(False)
        self._pending_swath = None
        self.swath_layer.clear()
        cleared.append("measurements")
        self.statusBar().showMessage(
            "Cleared: " + ", ".join(cleared)
            + ".  Mosaic and hidden overlays preserved.", 8000)

    # ---- waterfall click -> map point ------------------------------------------------
    def _on_waterfall_point(self, row: int, col: int) -> None:
        """A click in the waterfall selects the same physical point on
        the world map, with its GPS coordinates."""
        meta = self.waterfall_service.row_meta(row)
        if meta is None:
            self.statusBar().showMessage(
                "No position for that waterfall point (gap row, or the "
                "history was discarded).", 6000)
            return
        _t, rx, ry, yaw, depth = meta
        wx, wy = waterfall_pixel_to_world(
            rx, ry, yaw, depth, self.waterfall_service.pitch_m,
            float(col), self.waterfall_service.columns)
        wx, wy = float(wx), float(wy)
        gps = self.geo.local_to_gps(wx, wy)
        self.selection_layer.show_at(
            wx, wy, format_latlon(*gps) if gps else "")
        self.waterfall_view.set_selected(row, col)
        self.left_panel.coordinate_card.set_point(wx, wy, gps)
        if self.geo.ready:
            self.map_view.center_on_world(*self.geo.world_to_en(wx, wy))
        gps_txt = f"   |   {gps[0]:.7f}, {gps[1]:.7f}" if gps else ""
        self.statusBar().showMessage(
            f"Waterfall point:  x {wx:+.2f} m,  y {wy:+.2f} m{gps_txt}",
            15000)

    # ---- map interaction slots -------------------------------------------------------
    # MapView clicks arrive in EN scene coordinates (y-up); the exact
    # inverse of the fit (world = EN - t) recovers the robot's world
    # frame, and lat/lon comes from the same fit. Distances are frame-
    # independent (the fit is a pure translation).
    def _on_point_clicked(self, ex: float, ey: float) -> None:
        x, y = self.geo.en_to_world(ex, ey)
        gps = self.geo.local_to_gps(x, y)
        self.left_panel.coordinate_card.set_point(x, y, gps)
        gps_txt = f"   |   {gps[0]:.7f}, {gps[1]:.7f}" if gps else ""
        self.statusBar().showMessage(
            f"Point:  x {x:+.2f} m,  y {y:+.2f} m{gps_txt}", 15000)

    def _on_measure_toggled(self, on: bool) -> None:
        self.map_view.set_mode(MapMode.MEASURE if on else MapMode.NAVIGATE)
        if not on:
            self.measure_layer.clear()

    def _on_measure_started(self, ex: float, ey: float) -> None:
        self.measure_layer.show_first(ex, ey)   # measure layer lives in EN
        self.right_panel.on_first_point()
        x, y = self.geo.en_to_world(ex, ey)
        self.right_panel.point_a.set_point(x, y, self.geo.local_to_gps(x, y))
        self.right_panel.point_b.clear()

    def _on_measure_done(self, ex1: float, ey1: float,
                         ex2: float, ey2: float, dist: float) -> None:
        self.measure_layer.show_measurement((ex1, ey1), (ex2, ey2), dist)
        x2, y2 = self.geo.en_to_world(ex2, ey2)
        self.right_panel.point_b.set_point(
            x2, y2, self.geo.local_to_gps(x2, y2))
        self.right_panel.show_distance(dist)

    def _on_viewport_changed(self, world_rect: QRectF, mpp: float) -> None:
        self.tile_layer.update_viewport(world_rect, mpp)

    def _refresh_tiles(self) -> None:
        self.tile_layer.update_viewport(self.map_view.world_viewport_rect(),
                                        self.map_view.metres_per_pixel())

    def _center_robot(self) -> None:
        pos = self.trajectory_layer.current_pos()
        if pos is None and self._last_robot_state is not None:
            pos = (self._last_robot_state.x, self._last_robot_state.y)
        if pos is not None:
            # One-shot centering; the view lives in EN, poses in world.
            self.map_view.center_on_world(*self.geo.world_to_en(*pos))

    # ---- layer toggles ------------------------------------------------------------------
    def _on_layer_toggled(self, key: str, on: bool) -> None:
        if key == lp.LAYER_SATELLITE:
            self.tile_layer.set_visible(on)
            if on:
                self._refresh_tiles()
        elif key == lp.LAYER_TRAJECTORY:
            self.trajectory_layer.set_visible(on)
        elif key == lp.LAYER_PLANNED_PATH:
            self.planned_path_layer.set_visible(on)
        elif key == lp.LAYER_SWATH:
            self.swath_layer.set_visible(on)
        elif key == lp.LAYER_PINGER:
            self.pinger_layer.set_visible(on)
        elif key == lp.LAYER_DETECTIONS:
            self.detection_layer.set_visible(on)
        elif key == lp.LAYER_INTERPOLATION:
            self._mosaic_service.set_interpolation(on)

    # ---- acquisition lifecycle ---------------------------------------------------
    # The processing pipeline is launched at application startup (see
    # main.py). START only enables pinging + live visualization — no
    # node is restarted. STOP terminates the nodes (and START relaunches
    # them if pressed again). Recording is fully independent.
    def _on_start(self) -> None:
        self._mission_start = time.monotonic()
        # Robot-state re-sync (as if the application had just launched),
        # while PRESERVING any displayed trajectory: forget the cached
        # pose and break the track polyline so no phantom segment joins
        # pre/post-stop positions. The marker snaps to the next message.
        self._last_robot_state = None
        self.trajectory_layer.begin_new_segment()
        self._mosaic_service.reset_tracking()  # no interp across the break
        self._pending_swath = None
        self.swath_layer.clear()          # redrawn by the first new ping
        # Per-mission state that must NOT leak into the new session (the
        # "pings pile on one point in sessions 2+" field bug): the frozen-
        # pose detector's engagement, the sonar pair's counter-offset
        # expectations, and any seabed-image rows buffered before STOP.
        self._frozen_detector.reset()
        self._acquisition.reset_stream_health()
        self.seabed_imager.reset()        # discard pre-STOP rows, no emit
        if not getattr(self._acquisition, "running", False):
            # Relaunch after a STOP. A refusal (launcher still in its
            # SIGINT->SIGKILL ladder, or spawn failure) must not flip the
            # viz gate: doing so greyed START out with nothing running,
            # which is how a second field session got stuck.
            if not self._acquisition.start():
                return
        self._acquisition.enable_pinging()
        self._viz_enabled = True
        self.toolbar.on_viz_state(True)
        self.statusBar().showMessage("Live acquisition started.", 6000)

    def _on_record_toggled(self, on: bool) -> None:
        """Record ON/OFF — independent from visualization."""
        if on:
            if not self._acquisition.set_recording(True):
                # The processor never saw log_enable: opening a session
                # would produce a folder with no .svlog ever adopted
                # (the "only the first session records" field bug).
                self.toolbar.on_recording_state(False)
                return
            self.recording.begin()
            # Live seabed images stream into the session from now on.
            self.seabed_imager.set_output_dir(
                self.recording.session_dir / "seabed_images")
        else:
            self._acquisition.set_recording(False)  # close the .svlog
            self.seabed_imager.flush()   # truncated last picture: no data wasted
            self.seabed_imager.set_output_dir(None)
            saved = self.recording.end()            # save every artifact
            if saved is not None:
                # Adoption is deferred so the processor's in-flight
                # log_enable=False lands before the file is moved
                # (recording_session.py module docstring).
                QTimer.singleShot(
                    int(self._config.recording.adopt_delay_s * 1000),
                    self.recording.adopt_now)
                self.statusBar().showMessage(
                    f"Recording session saved to {saved}", 15000)

    def _on_range_apply(self, range_m: float) -> None:
        """Right-panel 'Apply range': hand the dance to the launcher
        (the simulator refuses gracefully — its swath is fixed)."""
        self._acquisition.set_range(range_m)

    def _on_sonar_params_result(self, ok: bool, _detail: str) -> None:
        if ok:
            # Honest seam: the waterfall's column scale follows each
            # ping's slant_range_m, so rows before and after the change
            # are on different horizontal scales — mark the boundary.
            self.waterfall_service.break_row()

    def _on_pipeline_state(self, state: str) -> None:
        """Pipeline died or stopped while a session is open: close the
        session properly (the toolbar only unchecks its button, which is
        cosmetic — RecordingManager would stay active and silently reuse
        this session's folder on the next Record ON)."""
        self.right_panel.set_acquisition_enabled(state == "running")
        if state in ("stopped", "error") and self.recording.active:
            self.seabed_imager.flush()
            self.seabed_imager.set_output_dir(None)
            saved = self.recording.end()
            # The processor is gone — nothing can write any more, so the
            # adoption sweep is safe to run synchronously.
            self.recording.adopt_now()
            if saved is not None:
                self.statusBar().showMessage(
                    f"Pipeline stopped — session saved to {saved}", 15000)

    def _on_stop(self) -> None:
        """Full stop: pinging off, session closed (if any), nodes down."""
        self._acquisition.disable_pinging()
        self._viz_enabled = False
        self._mission_start = None
        self.toolbar.on_viz_state(False)
        if self.recording.active:                  # save ONLY if recording
            self._acquisition.set_recording(False)
            self.seabed_imager.flush()   # truncated last picture
            self.seabed_imager.set_output_dir(None)
            saved = self.recording.end()
            # Pinging is off, so the processor writes nothing more:
            # adopting synchronously here is race-free.
            self.recording.adopt_now()
            if saved is not None:
                self.statusBar().showMessage(f"Saved to {saved}", 15000)
        else:
            self.recording.adopt_now()   # flush a pending deferred adoption
        self._acquisition.stop()                   # terminate ROS 2 nodes

    def _update_mission_time(self) -> None:
        self.left_panel.set_mission_time(
            None if self._mission_start is None
            else time.monotonic() - self._mission_start)

    # ---- shutdown --------------------------------------------------------------------------
    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802
        # Save only if a recording session was actually active; nothing
        # is exported otherwise (acquisition-lifecycle spec).
        if self.recording.active:
            self._acquisition.set_recording(False)
            self.recording.end()
        self.recording.adopt_now()   # never lose a deferred adoption on exit
        self._acquisition.disable_pinging()
        self._acquisition.stop()
        super().closeEvent(event)
