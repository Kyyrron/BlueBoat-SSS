"""Acquisition pipeline lifecycle (bottom-toolbar START/STOP).

Explicit state machine
----------------------
::

            start()                    process up + enables sent
    IDLE ────────────► STARTING ─────────────────────────► RUNNING
     ▲                    │ launch failed                     │ stop()
     │                    ▼                                   ▼
     │◄── (sweep) ──── ERROR                              STOPPING
     │                                                        │
     └────────────◄── sweep leftover processes ◄──────────────┘
                       SIGINT → SIGTERM → SIGKILL ladder

Why the previous implementation could leave ``sss_processor_node`` alive:
the old code escalated straight from SIGINT to SIGKILL **of the process
group**. If ``ros2 launch`` was SIGKILLed before it had forwarded the
shutdown, its child nodes were orphaned (re-parented to init) and never
signalled again. Two fixes:

1. a real escalation ladder — SIGINT (what ``ros2 launch`` expects),
   then SIGTERM after ``stop_grace_s``, then SIGKILL after
   ``stop_term_grace_s`` — each applied to the whole session group;
2. an unconditional **leftover sweep** after the launch process exits
   (and on application start): any process still matching
   ``pipeline.leftover_process_patterns`` (default:
   ``sss_processor_node``) receives SIGTERM, then SIGKILL. This is the
   invariant that makes N Start/Stop cycles safe regardless of how the
   launch tree misbehaved.

START is refused while STOPPING (the state machine forbids overlapping
lifecycles); the GUI simply sees ``pipeline_state`` strings as before,
so no consumer changes: "stopped" | "starting" | "running" | "error".
"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from enum import Enum
from typing import List, Optional

from PySide6.QtCore import QObject, QTimer

from ..config.settings import PipelineConfig
from ..core.signals import AppSignals
from .ros_manager import RosManager


class PipelineState(Enum):
    IDLE = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    ERROR = "error"


class PipelineLauncher(QObject):
    """Owns the `ros2 launch` subprocess for the SSS processing pipeline."""

    def __init__(self, config: PipelineConfig, ros: RosManager,
                 signals: AppSignals) -> None:
        super().__init__()
        self._config = config
        self._ros = ros
        self._signals = signals
        self._proc: Optional[subprocess.Popen] = None
        self._state = PipelineState.IDLE
        self._sigterm_at: Optional[float] = None
        self._sigkill_at: Optional[float] = None

        self._poll = QTimer(self)
        self._poll.setInterval(300)
        self._poll.timeout.connect(self._on_poll)

        # Hygiene at construction: a previous crash of the *application*
        # may itself have orphaned nodes.
        self._sweep_leftovers(announce=False)

    # ---- API -----------------------------------------------------------------
    @property
    def state(self) -> PipelineState:
        return self._state

    @property
    def running(self) -> bool:
        return self._state is PipelineState.RUNNING

    def start(self) -> None:
        if self._state in (PipelineState.STARTING, PipelineState.RUNNING):
            return
        if self._state is PipelineState.STOPPING:
            self._signals.status_message.emit(
                "Pipeline is still stopping — wait for 'stopped'.")
            return
        # Never start on top of leftovers from an earlier unclean stop.
        self._sweep_leftovers(announce=False)
        self._set_state(PipelineState.STARTING)
        cmd = list(self._config.launch_command)
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,        # pumped into the console
                stderr=subprocess.STDOUT,
                text=True, bufsize=1,
                start_new_session=True,  # own group: signal the whole tree
            )
        except (OSError, FileNotFoundError) as exc:
            self._proc = None
            self._set_state(PipelineState.ERROR)
            self._signals.status_message.emit(
                f"Failed to launch pipeline ({' '.join(cmd)}): {exc}")
            self._set_state(PipelineState.IDLE)
            return
        threading.Thread(target=self._pump_output, args=(self._proc,),
                         daemon=True).start()
        self._poll.start()
        QTimer.singleShot(int(self._config.start_delay_s * 1000),
                          self._enable_acquisition)

    def _pump_output(self, proc: subprocess.Popen) -> None:
        """Forward the launch tree's stdout/stderr to the console — this
        is how raw prints from sss_processor_node reach the operator
        without an external terminal (ROS-logger messages additionally
        arrive via the /rosout subscription in ros_manager)."""
        try:
            for line in proc.stdout:            # ends when the pipe closes
                line = line.rstrip()
                if line:
                    self._signals.log_line.emit("processor", line)
        except (ValueError, OSError):            # pragma: no cover
            pass

    def _enable_acquisition(self) -> None:
        # New lifecycle: bring-up completed = pipeline "running", but
        # pinging is NOT enabled here — the START button does that.
        if self._state is not PipelineState.STARTING or not self._alive():
            return
        self._set_state(PipelineState.RUNNING)
        self._signals.status_message.emit(
            "Processing pipeline up — press START for live acquisition.")

    def enable_pinging(self) -> None:
        """START button: begin firing the sonar (no node restart)."""
        if self._config.publish_ping_enable:
            self._ros.publish_ping_enable(True)

    def disable_pinging(self) -> None:
        if self._config.publish_ping_enable:
            self._ros.publish_ping_enable(False)

    def set_recording(self, on: bool) -> None:
        """Record ON/OFF: publish log_enable on the processor's topic
        (equivalent to `ros2 topic pub --once ... std_msgs/msg/Bool`)."""
        if not self.running and on:
            self._signals.status_message.emit(
                "Recording: pipeline is not running.")
            return
        self._ros.publish_svlog_enable(on)

    def stop(self) -> None:
        # Stop firing + close any .svlog immediately, in every state.
        if self._config.publish_ping_enable:
            self._ros.publish_ping_enable(False)
        self._ros.publish_svlog_enable(False)
        if not self._alive():
            self._proc = None
            self._sweep_leftovers()
            self._set_state(PipelineState.IDLE)
            return
        self._set_state(PipelineState.STOPPING)
        self._signal_group(signal.SIGINT)  # what ros2 launch expects
        now = time.monotonic()
        self._sigterm_at = now + self._config.stop_grace_s
        self._sigkill_at = (now + self._config.stop_grace_s
                            + self._config.stop_term_grace_s)

    # ---- polling ---------------------------------------------------------------
    def _on_poll(self) -> None:
        if self._proc is None:
            self._poll.stop()
            return
        code = self._proc.poll()
        if code is not None:
            was_stopping = self._state is PipelineState.STOPPING
            self._proc = None
            self._sigterm_at = self._sigkill_at = None
            self._poll.stop()
            # THE invariant: after the launch tree is gone, nothing
            # matching the pipeline patterns may survive.
            self._sweep_leftovers()
            if was_stopping or code == 0:
                self._set_state(PipelineState.IDLE)
            else:
                self._set_state(PipelineState.ERROR)
                self._signals.status_message.emit(
                    f"Pipeline exited unexpectedly (code {code}).")
                self._set_state(PipelineState.IDLE)
            return
        if self._state is PipelineState.STOPPING:
            now = time.monotonic()
            if self._sigterm_at is not None and now > self._sigterm_at:
                self._signal_group(signal.SIGTERM)
                self._sigterm_at = None
            elif self._sigkill_at is not None and now > self._sigkill_at:
                self._signal_group(signal.SIGKILL)
                self._sigkill_at = None

    # ---- helpers ---------------------------------------------------------------
    def _alive(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _set_state(self, state: PipelineState) -> None:
        self._state = state
        # GUI consumers keep seeing the same strings as before; STOPPING
        # is reported as "starting"-like busy? No: report it verbatim —
        # the toolbar treats anything but "running" conservatively.
        self._signals.pipeline_state.emit(state.value)

    def _signal_group(self, sig: signal.Signals) -> None:
        if self._proc is None:
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass

    def _sweep_leftovers(self, announce: bool = True) -> None:
        """SIGTERM (then SIGKILL) every process matching the patterns."""
        killed: List[str] = []
        for pattern in self._config.leftover_process_patterns:
            pids = self._pgrep(pattern)
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGTERM)
                    killed.append(f"{pattern}[{pid}]")
                except (ProcessLookupError, PermissionError):
                    continue
            if pids:
                # Short synchronous grace, then hard-kill survivors. The
                # sweep runs at most for ~0.5 s and only on stop/error.
                time.sleep(0.5)
                for pid in self._pgrep(pattern):
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except (ProcessLookupError, PermissionError):
                        continue
        if killed and announce:
            self._signals.status_message.emit(
                "Cleaned up leftover processes: " + ", ".join(killed))

    @staticmethod
    def _pgrep(pattern: str) -> List[int]:
        try:
            out = subprocess.run(["pgrep", "-f", pattern],
                                 capture_output=True, text=True, timeout=2.0)
            return [int(p) for p in out.stdout.split()]
        except (OSError, subprocess.TimeoutExpired, ValueError):
            return []
