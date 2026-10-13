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

Sonar source (toolbar selector, :mod:`.sonar_source`): **Simulation**
launches the processor only; **Real** adds ``with_acquisition:=True`` so the
same launch also starts ``sss_node.py`` — unless the process table shows a
simulator or a driver this application did not start, in which case the
launch falls back to the processor alone and says why. Every launched
process carries ``sonar_source.OWNER_ENV``; the sweep kills drivers that
carry it and never one an operator started by hand.
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

from ..config.settings import AcquisitionConfig, PipelineConfig
from ..core.signals import AppSignals
from . import sonar_source as ss
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
                 signals: AppSignals,
                 acquisition: Optional[AcquisitionConfig] = None,
                 sonar_source: Optional[str] = None) -> None:
        super().__init__()
        self._config = config
        self._acq_config = acquisition or AcquisitionConfig()
        self._source = ss.normalise(config.sonar_source if sonar_source is None
                                    else sonar_source)
        self._restart_pending = False
        self._ping_wanted = False
        self._ping_deadline = 0.0
        self._ros = ros
        self._signals = signals
        self._proc: Optional[subprocess.Popen] = None
        self._state = PipelineState.IDLE
        self._sigterm_at: Optional[float] = None
        self._sigkill_at: Optional[float] = None
        self._range_pending = False
        signals.sonar_params_result.connect(self._on_params_result)

        self._poll = QTimer(self)
        self._poll.setInterval(300)
        self._poll.timeout.connect(self._on_poll)

        self._ping_timer = QTimer(self)
        self._ping_timer.setInterval(200)
        self._ping_timer.timeout.connect(self._poll_ping_enable)

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

    @property
    def source(self) -> str:
        """``sonar_source.SIMULATION`` or ``sonar_source.REAL``."""
        return self._source

    def set_source(self, source: str, relaunch: bool = False) -> None:
        """Select the sonar the pipeline serves (toolbar selector).

        With ``relaunch`` the pipeline is (stopped if needed and) launched
        again with the new command once it is down — the caller has already
        confirmed with the operator and closed the session (STOP)."""
        self._source = ss.normalise(source)
        if not relaunch:
            return
        if self._state is PipelineState.IDLE:
            QTimer.singleShot(0, self.start)
            return
        self._restart_pending = True
        if self._state in (PipelineState.STARTING, PipelineState.RUNNING):
            self.stop()

    def start(self) -> bool:
        """Launch the pipeline; False when the request was refused.

        A refusal (still STOPPING, or the subprocess failed to spawn)
        must not be silent to the caller: the START handler used to set
        the viz gate anyway, leaving the toolbar stuck with START greyed
        out and Record disabled until an app restart.
        """
        if self._state in (PipelineState.STARTING, PipelineState.RUNNING):
            return True
        if self._state is PipelineState.STOPPING:
            self._signals.status_message.emit(
                "Pipeline is still stopping — wait for 'stopped', "
                "then press START again.")
            return False
        # Never start on top of leftovers from an earlier unclean stop.
        self._sweep_leftovers(announce=False)
        self._set_state(PipelineState.STARTING)
        cmd = self._command()
        env = dict(os.environ)
        env[ss.OWNER_ENV] = "1"          # marks the tree as ours (sweep)
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,        # pumped into the console
                stderr=subprocess.STDOUT,
                text=True, bufsize=1,
                start_new_session=True,  # own group: signal the whole tree
                env=env,
            )
        except (OSError, FileNotFoundError) as exc:
            self._proc = None
            self._set_state(PipelineState.ERROR)
            self._signals.status_message.emit(
                f"Failed to launch pipeline ({' '.join(cmd)}): {exc}")
            self._set_state(PipelineState.IDLE)
            return False
        threading.Thread(target=self._pump_output, args=(self._proc,),
                         daemon=True).start()
        self._poll.start()
        QTimer.singleShot(int(self._config.start_delay_s * 1000),
                          self._enable_acquisition)
        return True

    def _command(self) -> List[str]:
        """The launch for the selected source, guarded against a second
        ``/side_scan_sonar``."""
        base = list(self._config.launch_command)
        if self._source != ss.REAL:
            return base
        conflict = ss.driver_conflict(self._source, ss.scan_processes())
        if conflict:
            self._signals.status_message.emit(conflict)
            self._signals.log_line.emit("app", conflict)
            return base
        self._signals.log_line.emit(
            "app", "Sonar: Real — the pipeline also starts sss_node.py.")
        return ss.launch_command(base, self._source,
                                 self._config.real_sonar_launch_args)

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
        """START button: begin firing the sonar (no node restart).

        ``ping/enable`` is volatile: an enable published before
        ``/side_scan_sonar`` subscribes is lost, which after a relaunch
        (START after STOP, or a real driver still coming up) left the sonar
        silent. So it goes out once the topic has a subscriber — or after
        ``ping_enable_wait_s`` anyway, with the likely cause for the selected
        source on the console."""
        if not self._config.publish_ping_enable:
            return
        self._ping_wanted = True
        self._ping_deadline = time.monotonic() + self._config.ping_enable_wait_s
        self._poll_ping_enable()
        if self._ping_wanted:
            self._ping_timer.start()

    def _poll_ping_enable(self) -> None:
        if not self._ping_wanted:
            self._ping_timer.stop()
            return
        if self._state is PipelineState.STARTING:
            return                              # bring-up not finished yet
        if self._state is not PipelineState.RUNNING:
            self._ping_wanted = False           # the pipeline went away
            self._ping_timer.stop()
            return
        count = getattr(self._ros, "ping_enable_subscribers", None)
        n = count() if count is not None else None
        if n == 0 and time.monotonic() < self._ping_deadline:
            return
        self._ping_wanted = False
        self._ping_timer.stop()
        self._ros.publish_ping_enable(True)
        if n == 0:
            advice = ss.no_subscriber_advice(self._source)
            self._signals.status_message.emit(advice)
            self._signals.log_line.emit("app", advice)

    def disable_pinging(self) -> None:
        self._ping_wanted = False
        self._ping_timer.stop()
        if self._config.publish_ping_enable:
            self._ros.publish_ping_enable(False)

    def reset_stream_health(self) -> None:
        """START pressed: the sonar pair may have power-cycled, so the
        stream-health expectations are re-established."""
        self._ros.reset_stream_health()

    def reset_planned_path(self) -> None:
        """Planned-path overlay cleared: the listener forgets the last
        path so the publisher's next re-send is drawn again."""
        self._ros.reset_planned_path()

    # ---- runtime range change --------------------------------------------------
    def set_range(self, range_m: float) -> bool:
        """Change the sonar range while the pipeline runs.

        sss_node applies parameters only on the ping/enable RISING edge
        (its worker is idempotent while pinging), so the documented dance
        is performed here: enable=false → settle → set range_length_mm →
        enable=true on the service result. Pinging is ALWAYS resumed,
        success or not — a failed parameter change must never leave
        acquisition silently off."""
        if not self.running:
            self._signals.status_message.emit(
                "Range change refused: pipeline is not running.")
            return False
        if self._range_pending:
            self._signals.status_message.emit(
                "Range change already in progress.")
            return False
        self._range_pending = True
        self.disable_pinging()
        QTimer.singleShot(int(self._acq_config.settle_delay_s * 1000),
                          lambda: self._send_range(range_m))
        return True

    def _send_range(self, range_m: float) -> None:
        if not self._range_pending:
            return
        if not self._ros.set_sonar_range(range_m):
            # The request never left (result signal already emitted with
            # the reason) — _on_params_result resumes pinging.
            return

    def _on_params_result(self, ok: bool, detail: str) -> None:
        if not self._range_pending:
            return
        self._range_pending = False
        self.enable_pinging()          # resume unconditionally
        self._signals.status_message.emit(
            f"Sonar range applied ({detail}) — pinging resumed." if ok
            else f"Sonar range change FAILED: {detail} — pinging resumed "
                 "with the previous range.")

    def set_recording(self, on: bool) -> bool:
        """Record ON/OFF: publish log_enable on the processor's topic
        (equivalent to `ros2 topic pub --once ... std_msgs/msg/Bool`).

        Returns False when Record ON was refused because the pipeline is
        not running — the caller must NOT open a recording session then,
        or it produces a session folder with no .svlog ever adopted."""
        if not self.running and on:
            self._signals.status_message.emit(
                "Recording refused: pipeline is not running — "
                "press START first.")
            return False
        self._ros.publish_svlog_enable(on)
        return True

    def stop(self) -> None:
        # Stop firing + close any .svlog immediately, in every state.
        self._ping_wanted = False
        self._ping_timer.stop()
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
        if state is PipelineState.IDLE and self._restart_pending:
            self._restart_pending = False       # sonar source switched
            QTimer.singleShot(0, self.start)

    def _signal_group(self, sig: signal.Signals) -> None:
        if self._proc is None:
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass

    def _sweep_leftovers(self, announce: bool = True) -> None:
        """SIGTERM (then SIGKILL) every process matching the patterns —
        for ``owned_process_patterns`` only the ones this application
        launched (a driver started by hand is left alone)."""
        killed: List[str] = []
        patterns = ([(p, False) for p in self._config.leftover_process_patterns]
                    + [(p, True) for p in self._config.owned_process_patterns])
        for pattern, owned_only in patterns:
            pids = self._pgrep(pattern)
            if owned_only:
                pids = ss.owned_pids(pids)
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
                survivors = self._pgrep(pattern)
                if owned_only:
                    survivors = ss.owned_pids(survivors)
                for pid in survivors:
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
