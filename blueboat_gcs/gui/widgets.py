"""Small reusable widgets."""

from __future__ import annotations

import math
from typing import Optional, Tuple

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QApplication, QGridLayout, QLabel, QPushButton,
                               QSizePolicy, QWidget)

from ..utils.geodesy import format_latlon


class CoordinateCard(QWidget):
    """Displays one point: world coordinates, GPS, live robot distance.

    All rows are single-click copyable — during experiments coordinates
    are constantly pasted into logs / mission planners. The distance row
    updates continuously as the robot moves (main_window pushes the
    current robot position into every visible card on each RobotState).
    """

    def __init__(self, title: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._local_txt = "—"
        self._gps_txt = "—"
        self._point: Optional[Tuple[float, float]] = None
        self._robot: Optional[Tuple[float, float]] = None

        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(2)

        title_label = QLabel(title)
        title_label.setStyleSheet("color:#8ab4f8; font-weight:bold;")
        grid.addWidget(title_label, 0, 0, 1, 3)

        self._local_value = QLabel("—")
        self._local_value.setObjectName("valueLabel")
        self._local_value.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._gps_value = QLabel("—")
        self._gps_value.setObjectName("valueLabel")
        self._gps_value.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._dist_value = QLabel("—")
        self._dist_value.setObjectName("valueLabel")

        grid.addWidget(QLabel("World"), 1, 0)
        grid.addWidget(self._local_value, 1, 1)
        grid.addWidget(self._copy_button(lambda: self._local_txt), 1, 2)
        grid.addWidget(QLabel("GPS"), 2, 0)
        grid.addWidget(self._gps_value, 2, 1)
        grid.addWidget(self._copy_button(lambda: self._gps_txt), 2, 2)
        grid.addWidget(QLabel("Robot"), 3, 0)
        grid.addWidget(self._dist_value, 3, 1)
        grid.setColumnStretch(1, 1)

    def _copy_button(self, getter) -> QPushButton:
        btn = QPushButton("⧉")
        btn.setFixedSize(24, 22)
        btn.setToolTip("Copy to clipboard")
        btn.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        btn.clicked.connect(
            lambda: QApplication.clipboard().setText(getter()))
        return btn

    def set_point(self, x: float, y: float,
                  gps: Optional[Tuple[float, float]]) -> None:
        self._point = (x, y)
        self._local_txt = f"{x:.2f}, {y:.2f}"
        self._local_value.setText(f"x {x:+.2f} m   y {y:+.2f} m")
        if gps is not None:
            self._gps_txt = format_latlon(*gps)
            self._gps_value.setText(self._gps_txt)
        else:
            self._gps_txt = "—"
            self._gps_value.setText("no GPS origin yet")
        self._refresh_distance()

    def update_robot_position(self, rx: float, ry: float) -> None:
        """Live distance: called on every RobotState."""
        self._robot = (rx, ry)
        self._refresh_distance()

    def _refresh_distance(self) -> None:
        if self._point is None or self._robot is None:
            self._dist_value.setText("—")
            return
        d = math.hypot(self._point[0] - self._robot[0],
                       self._point[1] - self._robot[1])
        self._dist_value.setText(f"{d:.2f} m away")

    def clear(self) -> None:
        self._point = None
        self._local_txt = self._gps_txt = "—"
        self._local_value.setText("—")
        self._gps_value.setText("—")
        self._dist_value.setText("—")
