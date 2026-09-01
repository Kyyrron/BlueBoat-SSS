"""Unified recording sessions: one experiment = one folder.

Lifecycle
---------
* toolbar "Start recording" → :meth:`RecordingManager.begin`:
  timestamps the session, starts counting pings/detections, and asks the
  launcher to enable .svlog logging in the processor node;
* toolbar "STOP acquisition" (or app close) → :meth:`end`: creates the
  session directory and gathers every artifact into it.

Session folder layout (consumed by future processing scripts)::

    <data_root>/sessions/2026_07_08-14_02_31/
        metadata.json               # times, config snapshot, counters...
        *.svlog                     # adopted from the processor (see below)
        mosaic/
            sonar_mosaic.npz        # raw planes (legacy keys + priorities)
            sonar_mosaic.png
            boat_trajectory.csv     # t, x, y, depth
        waterfall/
            waterfall.png           # display-pipeline quick-look
            waterfall_raw.npz       # untouched ping buffer (AI datasets)
        detections/
            detections.csv          # uid, t, x, y, class, confidence

The .svlog adoption: the file is written by ``sss_processor_node``
wherever *it* decides — the GCS cannot redirect it. After the session
ends, every ``*.svlog`` found under ``data_root`` whose modification
time falls inside the session window is *moved* to the **session root**,
alongside ``metadata.json``. If the processor writes elsewhere, add that
directory to the sweep list.

Adoption timing: ``end()`` saves every artifact immediately but does
NOT adopt — the ``log_enable=False`` message is asynchronous, and
moving the file while the processor still holds its old absolute path
open makes its next append recreate a headerless stub in ``data_root``.
The caller schedules :meth:`adopt_now` after a short settle delay
(``recording.adopt_delay_s``); STOP and app-close call it synchronously
because pinging is already off there. Anything under ``sessions/`` or
``merged_sessions/`` is never adopted (a merged session is derived
data with the same primary-record protection, NC #6).
"""

from __future__ import annotations

import json
import shutil
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import List, Optional

from PySide6.QtCore import QObject, Signal

from ..config.settings import AppConfig
from ..models.detection import Detection
from .mosaic_service import MosaicService
from .signals import AppSignals
from .waterfall_service import WaterfallService

_SVLOG_MTIME_SLACK_S = 10.0     # tolerance around the session window

#: The smallest possible framed Cerulean packet: 8-byte header (BR, u16
#: length, u16 id, src, dst) + empty payload + u16 checksum. Anything
#: below this is an empty/truncated stub (e.g. left by a writer that
#: appended after its file moved) and is skipped — never moved, never
#: deleted — and reported.
_SVLOG_MIN_BYTES = 10


def write_session_metadata(session: Path, meta: dict) -> None:
    """Write ``metadata.json`` with the session schema.

    Single writer for live sessions and rebuilt (merged) ones, so the
    schema — in particular the keys ``ping_count``, ``detection_count``,
    ``adopted_svlogs`` and ``mosaic`` that downstream scripts and the
    regression suite rely on — cannot drift between the two producers.
    """
    with open(session / "metadata.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


def update_session_metadata(session: Path, **updates) -> None:
    """Patch keys into an existing ``metadata.json`` (e.g. the adoption
    result, which is only known after the settle delay)."""
    path = session / "metadata.json"
    try:
        with open(path, "r", encoding="utf-8") as f:
            meta = json.load(f)
    except (OSError, ValueError):
        meta = {}
    meta.update(updates)
    write_session_metadata(session, meta)


class RecordingManager(QObject):
    """Owns recording sessions and assembles their output folders."""

    recording_state = Signal(bool)      # True while a session is active

    def __init__(self, config: AppConfig, signals: AppSignals,
                 mosaic: MosaicService, waterfall: WaterfallService) -> None:
        super().__init__()
        self._config = config
        self._signals = signals
        self._mosaic = mosaic
        self._waterfall = waterfall
        self._start_wall: Optional[float] = None
        self._start_stamp: str = ""
        self.session_dir: Optional[Path] = None
        # (session_dir, start_wall, ping_count) of an ended session whose
        # adoption sweep has not run yet; consumed by adopt_now().
        self._pending_adoption: Optional[tuple] = None
        self._ping_count = 0
        self._detections: List[Detection] = []
        self._priority_mode = "average"
        self._display_settings = None
        signals.sonar_ping.connect(self._count_ping)
        signals.detection.connect(self._log_detection)

    # ---- bookkeeping slots -------------------------------------------------------
    def _count_ping(self, _ping) -> None:
        if self.active:
            self._ping_count += 1

    def _log_detection(self, det: Detection) -> None:
        if self.active:
            self._detections.append(det)

    def note_priority_mode(self, mode: str) -> None:
        self._priority_mode = mode

    def note_display_settings(self, settings) -> None:
        self._display_settings = settings

    # ---- lifecycle -----------------------------------------------------------------
    @property
    def active(self) -> bool:
        return self._start_wall is not None

    def begin(self) -> bool:
        if self.active:
            # Visible, never silent: a stuck "active" state here used to
            # make every later Record ON reuse the first session folder.
            self._signals.status_message.emit(
                f"Recording session {self._start_stamp} is still active — "
                "reusing it (STOP or Record OFF closes it).")
            return False
        # A still-pending adoption belongs to the previous session; run
        # it now so its .svlog cannot be swallowed by this session.
        self.adopt_now()
        self._start_wall = time.time()
        self._start_stamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        self._ping_count = 0
        self._detections.clear()
        # The directory exists from the start of the session so streaming
        # artifacts (live seabed_images/) land inside it as they are made.
        self.session_dir = (Path(self._config.data_root).expanduser()
                            / "sessions" / self._start_stamp)
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.recording_state.emit(True)
        self._signals.status_message.emit(
            f"Recording session {self._start_stamp} started.")
        return True

    def end(self) -> Optional[Path]:
        """Finalize the session folder; returns it (None if not active).

        Adoption of the .svlog is NOT done here — call :meth:`adopt_now`
        after the log_enable=False message has settled (see module
        docstring); until then ``metadata.json`` carries an empty
        ``adopted_svlogs`` list.
        """
        if not self.active:
            return None
        start_wall, self._start_wall = self._start_wall, None
        session = self.session_dir

        self._mosaic.save_into(session / "mosaic")
        self._waterfall.export_into(session / "waterfall")
        self._write_detections(session)
        self._write_metadata(session, start_wall, [])
        self._pending_adoption = (session, start_wall, self._ping_count)

        self.recording_state.emit(False)
        self._signals.status_message.emit(
            f"Recording session saved to {session}")
        return session

    def adopt_now(self) -> List[str]:
        """Run the deferred .svlog adoption of the last ended session.

        Idempotent: consumes the pending context, so extra calls (a
        scheduled timer firing after app-close already adopted, say) are
        no-ops. Rewrites the session's ``adopted_svlogs`` metadata key
        and warns loudly when a session with pings adopted nothing.
        """
        if self._pending_adoption is None:
            return []
        (session, start_wall, ping_count), self._pending_adoption = \
            self._pending_adoption, None
        adopted = self._adopt_svlogs(session, start_wall)
        update_session_metadata(session, adopted_svlogs=adopted)
        if adopted:
            self._signals.status_message.emit(
                f"Adopted {', '.join(adopted)} into {session.name}.")
        elif ping_count > 0:
            self._signals.status_message.emit(
                f"WARNING: session {session.name} recorded {ping_count} "
                "pings but no .svlog was adopted — the processor may not "
                "have been recording (check the console for "
                "'logging ->' from sss_processor).")
        return adopted

    # ---- artifact assembly ------------------------------------------------------
    def _write_detections(self, session: Path) -> None:
        if not self._detections:
            return
        import csv
        d = session / "detections"
        d.mkdir(exist_ok=True)
        with open(d / "detections.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["uid", "t", "x_m", "y_m", "class",
                        "confidence", "extent_m"])
            for det in self._detections:
                w.writerow([det.uid, f"{det.t:.3f}", f"{det.x:.3f}",
                            f"{det.y:.3f}", det.class_name,
                            f"{det.confidence:.3f}", f"{det.extent_m:.2f}"])

    def _adopt_svlogs(self, session: Path, start_wall: float) -> List[str]:
        """Move .svlog files written during the session to the session root."""
        adopted: List[str] = []
        # The processor resolves the shared data_root to an absolute path;
        # do the same so the sweep and the writer agree regardless of cwd.
        root = Path(self._config.data_root).expanduser()
        lo = start_wall - _SVLOG_MTIME_SLACK_S
        hi = time.time() + _SVLOG_MTIME_SLACK_S
        if not root.exists():
            self._signals.status_message.emit(
                f"WARNING: data_root {root} does not exist — "
                "no .svlog adopted.")
            return adopted
        root = root.resolve()
        # Anything already filed in a session — this one or an earlier
        # one — is settled and is never moved again. merged_sessions/
        # holds derived sessions with the same protection. Recorded
        # .svlog are primary field data (CLAUDE.md NON-NEGOTIABLE #6).
        protected = (root / "sessions", root / "merged_sessions")
        for f in root.rglob("*.svlog"):
            try:
                f = f.resolve()
                if any(p in f.parents for p in protected):
                    continue
                st = f.stat()
                if not (lo <= st.st_mtime <= hi):
                    continue
                if st.st_size < _SVLOG_MIN_BYTES:
                    # A headerless stub from a writer that appended after
                    # its file was moved: worthless, but never deleted.
                    self._signals.status_message.emit(
                        f"Skipping stub {f.name} ({st.st_size} B) — "
                        "not a valid .svlog.")
                    continue
                shutil.move(str(f), session / f.name)
                adopted.append(f.name)
            except OSError:
                continue
        return adopted

    def _write_metadata(self, session: Path, start_wall: float,
                        svlogs: List[str]) -> None:
        meta = {
            "session": self._start_stamp,
            "started_utc": datetime.utcfromtimestamp(
                start_wall).isoformat() + "Z",
            "ended_utc": datetime.utcnow().isoformat() + "Z",
            "duration_s": round(time.time() - start_wall, 1),
            "ping_count": self._ping_count,
            "detection_count": len(self._detections),
            "adopted_svlogs": svlogs,
            "mosaic": {
                "cell_size_m": self._config.mosaic.cell_size_m,
                "densify": self._config.mosaic.densify,
                "bilinear_splat": self._config.mosaic.bilinear_splat,
                "priority_mode_displayed": self._priority_mode,
            },
            "display_settings_at_end": (
                asdict(self._display_settings)
                if self._display_settings is not None else None),
            "topics": asdict(self._config.topics),
            "note": ("mosaic/*.npz and waterfall/waterfall_raw.npz contain "
                     "raw, unrendered data; PNGs go through the display "
                     "pipeline and are quick-looks only."),
        }
        write_session_metadata(session, meta)
