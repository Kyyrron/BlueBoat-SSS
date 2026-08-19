"""Left panel: layer visibility, detection summary, robot information."""

from __future__ import annotations

import math
from typing import Dict, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QCheckBox, QFormLayout, QGroupBox, QLabel,
                               QTableWidget, QTableWidgetItem, QVBoxLayout,
                               QWidget)

from ..models.detection import Detection
from ..models.robot_state import RobotState
from ..utils.geodesy import format_latlon
from .widgets import CoordinateCard

# Stable layer identifiers shared with MainWindow.
LAYER_SATELLITE = "satellite"
LAYER_TRAJECTORY = "trajectory"
LAYER_PLANNED_PATH = "planned_path"
LAYER_SWATH = "swath"
LAYER_PINGER = "pinger"
LAYER_DETECTIONS = "detections"
LAYER_INTERPOLATION = "interpolation"


class LeftPanel(QWidget):
    """Visibility toggles + live mission information."""

    layer_toggled = Signal(str, bool)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        # ---- visibility -----------------------------------------------------
        vis_box = QGroupBox("Layers")
        vis_layout = QVBoxLayout(vis_box)
        self._checks: Dict[str, QCheckBox] = {}
        # All visualization layers start DISABLED (acquisition-lifecycle
        # spec): the operator enables what the experiment needs. The SSS
        # mosaic itself is not a toggle — it appears with START.
        for key, label, default in (
            (LAYER_SATELLITE, "Satellite layer", True),
            (LAYER_TRAJECTORY, "Robot trajectory", True),
            (LAYER_PLANNED_PATH, "Planned mission path", True),
            (LAYER_SWATH, "Sonar range line", False),
            (LAYER_PINGER, "USBL pinger", False),
            (LAYER_DETECTIONS, "AI detections", False),
            (LAYER_INTERPOLATION, "Interpolation", False),
        ):
            cb = QCheckBox(label)
            cb.setChecked(default)
            cb.toggled.connect(
                lambda on, k=key: self.layer_toggled.emit(k, on))
            vis_layout.addWidget(cb)
            self._checks[key] = cb
        root.addWidget(vis_box)

        # ---- detections -----------------------------------------------------
        det_box = QGroupBox("Detections")
        det_layout = QVBoxLayout(det_box)
        self._det_total = QLabel("Total: 0")
        self._det_total.setObjectName("sectionValue")
        det_layout.addWidget(self._det_total)
        self._det_table = QTableWidget(0, 2)
        self._det_table.setHorizontalHeaderLabels(["Class", "Count"])
        self._det_table.horizontalHeader().setStretchLastSection(True)
        self._det_table.verticalHeader().setVisible(False)
        self._det_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._det_table.setSelectionMode(QTableWidget.NoSelection)
        self._det_table.setFixedHeight(120)
        det_layout.addWidget(self._det_table)
        root.addWidget(det_box)
        self._class_counts: Dict[str, int] = {}
        self._seen_uids: set[int] = set()

        # ---- robot state ------------------------------------------------------
        robot_box = QGroupBox("Robot")
        form = QFormLayout(robot_box)
        form.setLabelAlignment(Qt.AlignRight)
        self._gps = self._value_label()
        self._heading = self._value_label()
        self._speed = self._value_label()
        self._mission_time = self._value_label()
        form.addRow("GPS", self._gps)
        form.addRow("Heading", self._heading)
        form.addRow("Speed", self._speed)
        form.addRow("Mission", self._mission_time)
        root.addWidget(robot_box)

        # ---- pinger ---------------------------------------------------------------
        pinger_box = QGroupBox("Pinger")
        pform = QFormLayout(pinger_box)
        pform.setLabelAlignment(Qt.AlignRight)
        self._pinger_world = self._value_label()
        self._pinger_gps = self._value_label()
        self._pinger_dist = self._value_label()
        pform.addRow("World", self._pinger_world)
        pform.addRow("GPS", self._pinger_gps)
        pform.addRow("Robot", self._pinger_dist)
        root.addWidget(pinger_box)
        self._pinger_xy: Optional[tuple] = None
        self._robot_xy: Optional[tuple] = None

        # ---- selected point ------------------------------------------------------
        sel_box = QGroupBox("Selected point")
        sel_layout = QVBoxLayout(sel_box)
        self.coordinate_card = CoordinateCard("Last clicked position")
        sel_layout.addWidget(self.coordinate_card)
        root.addWidget(sel_box)

        root.addStretch(1)

    @staticmethod
    def _value_label() -> QLabel:
        lbl = QLabel("—")
        lbl.setObjectName("valueLabel")
        lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
        return lbl

    # ---- slots -----------------------------------------------------------------
    def is_layer_enabled(self, key: str) -> bool:
        return self._checks[key].isChecked()

    def on_robot_state(self, state: RobotState) -> None:
        if state.lat is not None and state.lon is not None:
            self._gps.setText(format_latlon(state.lat, state.lon))
        else:
            self._gps.setText("no fix")
        if state.heading_deg is not None:
            self._heading.setText(f"{state.heading_deg:.1f}°")
        if state.speed_mps is not None:
            self._speed.setText(f"{state.speed_mps:.2f} m/s")
        self._robot_xy = (state.x, state.y)
        self._refresh_pinger_distance()

    def on_pinger(self, x: float, y: float, gps=None) -> None:
        """Live pinger info: world coords (fixed world frame, after any
        robot->world transform), GPS position, robot distance."""
        self._pinger_xy = (x, y)
        self._pinger_world.setText(f"x {x:+.2f} m   y {y:+.2f} m")
        self._pinger_gps.setText(format_latlon(*gps) if gps else "—")
        self._refresh_pinger_distance()

    def _refresh_pinger_distance(self) -> None:
        # Continuously updated whenever EITHER the robot or the pinger
        # moves — both slots funnel here.
        if self._pinger_xy is None or self._robot_xy is None:
            self._pinger_dist.setText("—")
            return
        d = math.hypot(self._pinger_xy[0] - self._robot_xy[0],
                       self._pinger_xy[1] - self._robot_xy[1])
        self._pinger_dist.setText(f"{d:.2f} m away")

    def set_mission_time(self, seconds: Optional[float]) -> None:
        if seconds is None:
            self._mission_time.setText("—")
        else:
            m, s = divmod(int(seconds), 60)
            h, m = divmod(m, 60)
            self._mission_time.setText(f"{h:02d}:{m:02d}:{s:02d}")

    def on_detection(self, det: Detection) -> None:
        if det.uid in self._seen_uids:  # revisit update: position refined,
            return                      # class counted once.
        self._seen_uids.add(det.uid)
        self._class_counts[det.class_name] = \
            self._class_counts.get(det.class_name, 0) + 1
        self._det_total.setText(f"Total: {len(self._seen_uids)}")
        self._det_table.setRowCount(len(self._class_counts))
        for row, (name, count) in enumerate(sorted(self._class_counts.items())):
            self._det_table.setItem(row, 0, QTableWidgetItem(name))
            self._det_table.setItem(row, 1, QTableWidgetItem(str(count)))

    def reset_detections(self) -> None:
        self._class_counts.clear()
        self._seen_uids.clear()
        self._det_total.setText("Total: 0")
        self._det_table.setRowCount(0)
