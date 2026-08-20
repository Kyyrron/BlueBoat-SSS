"""Embedded application console (bottom dock).

One place for everything textual: Python prints, application logging,
/rosout messages from every ROS 2 node (including ``sss_processor_node``)
and the raw output of the launch subprocess. The dock starts collapsed
to a one-line status strip; the toolbar "Console" button (or dragging
the dock splitter) expands it.

Kept deliberately simple: a bounded QPlainTextEdit (fast appends, ring
of ``MAX_LINES``), per-source colour tags, autoscroll-when-at-bottom
(same pin philosophy as the waterfall), pause + clear + copy controls.
"""

from __future__ import annotations

from datetime import datetime
from typing import Optional

from PySide6.QtGui import QColor, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (QApplication, QHBoxLayout, QLabel,
                               QPlainTextEdit, QPushButton, QVBoxLayout,
                               QWidget)

MAX_LINES = 5000

_SOURCE_COLORS = {
    "python": "#c7d0d9",       # print()
    "app": "#8ab4f8",          # logging module
    "rosout": "#69f0ae",       # ROS 2 log messages (all nodes)
    "processor": "#ffca28",    # raw sss_processor_node stdout/stderr
    "error": "#ff6e6e",        # stderr / exceptions
}


class LogConsole(QWidget):
    """Bounded, colour-tagged, autoscrolling text console."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)
        root.setContentsMargins(4, 2, 4, 4)
        root.setSpacing(2)

        bar = QHBoxLayout()
        self._counter = QLabel("0 lines")
        self._counter.setStyleSheet("color:#8b95a1;")
        pause = QPushButton("Pause")
        pause.setCheckable(True)
        clear = QPushButton("Clear")
        copy_all = QPushButton("Copy all")
        for b in (pause, clear, copy_all):
            b.setFixedHeight(22)
        bar.addWidget(self._counter)
        bar.addStretch(1)
        bar.addWidget(pause)
        bar.addWidget(clear)
        bar.addWidget(copy_all)
        root.addLayout(bar)

        self._text = QPlainTextEdit()
        self._text.setReadOnly(True)
        self._text.setMaximumBlockCount(MAX_LINES)   # ring buffer
        self._text.setStyleSheet(
            "QPlainTextEdit{background:#10151b; color:#c7d0d9;"
            " font-family:monospace; font-size:11px; border:none;}")
        root.addWidget(self._text, 1)

        self._paused = False
        self._n = 0
        pause.toggled.connect(self._set_paused)
        clear.clicked.connect(self._clear)
        copy_all.clicked.connect(
            lambda: QApplication.clipboard().setText(
                self._text.toPlainText()))

    # ---- slot (queued from any thread via AppSignals.log_line) -----------------
    def append_line(self, source: str, text: str) -> None:
        if self._paused:
            return
        self._n += 1
        self._counter.setText(f"{self._n} lines")
        bar = self._text.verticalScrollBar()
        at_bottom = bar.value() >= bar.maximum() - 4

        cursor = self._text.textCursor()
        cursor.movePosition(QTextCursor.End)
        fmt_src = QTextCharFormat()
        fmt_src.setForeground(QColor(_SOURCE_COLORS.get(source, "#c7d0d9")))
        stamp = datetime.now().strftime("%H:%M:%S")
        cursor.insertText(f"{stamp} [{source:9s}] ", fmt_src)
        fmt_txt = QTextCharFormat()
        fmt_txt.setForeground(QColor(
            _SOURCE_COLORS["error"] if source == "error" else "#c7d0d9"))
        cursor.insertText(text + "\n", fmt_txt)

        if at_bottom:                          # follow only while pinned
            bar.setValue(bar.maximum())

    # ---- controls -----------------------------------------------------------------
    def _set_paused(self, paused: bool) -> None:
        self._paused = paused

    def _clear(self) -> None:
        self._text.clear()
        self._n = 0
        self._counter.setText("0 lines")
