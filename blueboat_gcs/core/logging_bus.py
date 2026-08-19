"""Logging bus: routes every text output into the embedded console.

Captured sources (the operator never needs an external terminal):

* every Python ``print()`` / ``sys.stdout`` / ``sys.stderr`` write — a
  tee: the original stream keeps working (a terminal, if any, still
  shows everything), and each completed line is also emitted on
  ``signals.log_line``;
* the Python ``logging`` module (root handler);
* ROS 2 log messages from **all** nodes — captured via a ``/rosout``
  subscription in ``ros/ros_manager.py`` (this is how messages from
  ``sss_processor_node`` reach the console even though it is a separate
  process);
* the raw stdout/stderr of the launch subprocess — pumped by a reader
  thread in ``ros/pipeline_launcher.py`` (covers plain prints from the
  processor that never go through the ROS logger).

Emission is a plain Qt signal from arbitrary threads: Qt queues it to
the GUI thread, so the console widget stays single-threaded like every
other consumer on the bus.
"""

from __future__ import annotations

import logging
import sys
from typing import Optional, TextIO

from .signals import AppSignals


class _TeeStream:
    """File-like tee: forwards to the original stream + emits lines."""

    def __init__(self, original: Optional[TextIO], signals: AppSignals,
                 source: str) -> None:
        self._orig = original
        self._signals = signals
        self._source = source
        self._buf = ""

    def write(self, text: str) -> int:
        if self._orig is not None:
            try:
                self._orig.write(text)
            except (ValueError, OSError):
                pass
        self._buf += text
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line.strip():
                self._signals.log_line.emit(self._source, line)
        return len(text)

    def flush(self) -> None:
        if self._orig is not None:
            try:
                self._orig.flush()
            except (ValueError, OSError):
                pass

    def isatty(self) -> bool:                     # pragma: no cover
        return False

    def fileno(self) -> int:                      # subprocess inheritance
        if self._orig is not None:
            return self._orig.fileno()
        raise OSError("no underlying stream")


class _SignalLogHandler(logging.Handler):
    """Python logging -> console."""

    def __init__(self, signals: AppSignals) -> None:
        super().__init__()
        self._signals = signals
        self.setFormatter(logging.Formatter(
            "%(levelname)s %(name)s: %(message)s"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._signals.log_line.emit("app", self.format(record))
        except Exception:                          # pragma: no cover
            pass                                   # logging must never raise


def install(signals: AppSignals) -> None:
    """Install stdout/stderr tees + the logging handler. Call once,
    early in main(), before anything prints."""
    sys.stdout = _TeeStream(sys.__stdout__, signals, "python")
    sys.stderr = _TeeStream(sys.__stderr__, signals, "error")
    root = logging.getLogger()
    root.addHandler(_SignalLogHandler(signals))
    if root.level > logging.INFO or root.level == logging.NOTSET:
        root.setLevel(logging.INFO)


def uninstall() -> None:
    """Restore the original streams (shutdown)."""
    sys.stdout = sys.__stdout__
    sys.stderr = sys.__stderr__
