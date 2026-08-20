"""Dark theme in the style of professional robotics GCS software."""

from __future__ import annotations

from PySide6.QtGui import QColor

# Palette used programmatically by map layers.
COLOR_BACKGROUND = QColor("#14181d")
COLOR_PANEL = QColor("#1c2127")
COLOR_ACCENT = QColor("#2f81f7")
COLOR_TRAJECTORY = QColor("#00e5ff")
COLOR_ROBOT = QColor("#ffffff")
COLOR_ROBOT_OUTLINE = QColor("#ff5252")
COLOR_GRID = QColor(255, 255, 255, 28)
COLOR_GRID_TEXT = QColor(255, 255, 255, 110)
COLOR_DETECTION = QColor("#ffca28")
COLOR_PINGER = QColor("#69f0ae")
COLOR_MEASURE = QColor("#ff8a65")
COLOR_PLANNED_PATH = QColor("#1a4fbf")   # dark blue, distinct from the cyan track

STYLESHEET = """
QMainWindow, QWidget { background-color: #14181d; color: #d7dde4;
    font-family: "DejaVu Sans", "Segoe UI", sans-serif; font-size: 12px; }
QDockWidget { titlebar-close-icon: none; titlebar-normal-icon: none; }
QDockWidget::title { background: #1c2127; padding: 6px; font-weight: bold;
    border-bottom: 1px solid #2a313a; }
QGroupBox { border: 1px solid #2a313a; border-radius: 4px; margin-top: 14px;
    padding: 6px 4px 4px 4px; background: #181d23; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px;
    color: #8ab4f8; font-weight: bold; }
QCheckBox { spacing: 8px; padding: 3px; }
QCheckBox::indicator { width: 14px; height: 14px; border: 1px solid #3c4652;
    border-radius: 3px; background: #10141a; }
QCheckBox::indicator:checked { background: #2f81f7; border-color: #2f81f7; }
QPushButton { background: #232a33; border: 1px solid #3c4652; border-radius: 4px;
    padding: 6px 12px; }
QPushButton:hover { background: #2b3440; }
QPushButton:pressed { background: #1a2028; }
QPushButton:checked { background: #2f81f7; border-color: #2f81f7; color: white; }
QPushButton:disabled { color: #5a6470; }
QPushButton#startButton { background: #1d5c33; border-color: #2e8b57;
    font-weight: bold; padding: 8px 22px; }
QPushButton#startButton:hover { background: #23703f; }
QPushButton#stopButton { background: #6e2323; border-color: #a03030;
    font-weight: bold; padding: 8px 22px; }
QPushButton#stopButton:hover { background: #852b2b; }
QToolBar { background: #1c2127; border-top: 1px solid #2a313a; spacing: 8px;
    padding: 4px; }
QStatusBar { background: #1c2127; border-top: 1px solid #2a313a; color: #8b95a1; }
QLabel#valueLabel { color: #e8eef4; font-family: "DejaVu Sans Mono", monospace; }
QLabel#sectionValue { color: #e8eef4; font-family: "DejaVu Sans Mono", monospace;
    font-size: 13px; }
QTableWidget { background: #10141a; border: 1px solid #2a313a; gridline-color: #232a33; }
QHeaderView::section { background: #1c2127; border: none; padding: 4px;
    color: #8b95a1; }
QToolTip { background: #232a33; color: #d7dde4; border: 1px solid #3c4652; }
"""
