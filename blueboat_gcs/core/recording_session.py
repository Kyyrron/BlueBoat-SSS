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
        mosaic/
            sonar_mosaic.npz        # raw planes (legacy keys + priorities)
            sonar_mosaic.png
            boat_trajectory.csv     # t, x, y, depth
        waterfall/
            waterfall.png           # display-pipeline quick-look
            waterfall_raw.npz       # untouched ping buffer (AI datasets)
        detections/
            detections.csv          # uid, t, x, y, class, confidence
        svlog/
            *.svlog                 # adopted from the processor (see below)

The .svlog adoption: the file is written by ``sss_processor_node``
wherever *it* decides — the GCS cannot redirect it. After the session
ends, every ``*.svlog`` found under ``data_root`` whose modification
time falls inside the session window is *moved* into ``svlog/``. If the
processor writes elsewhere, add that directory to the sweep list.
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

    def begin(self) -> None:
        if self.active:
            return
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

    def end(self) -> Optional[Path]:
        """Finalize the session folder; returns it (None if not active)."""
        if not self.active:
            return None
        start_wall, self._start_wall = self._start_wall, None
        session = self.session_dir

        self._mosaic.save_into(session / "mosaic")
        self._waterfall.export_into(session / "waterfall")
        self._write_detections(session)
        adopted = self._adopt_svlogs(session, start_wall)
        self._write_metadata(session, start_wall, adopted)

        self.recording_state.emit(False)
        self._signals.status_message.emit(
            f"Recording session saved to {session}")
        return session

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
        """Move .svlog files written during the session into svlog/."""
        adopted: List[str] = []
        root = Path(self._config.data_root).expanduser()
        lo = start_wall - _SVLOG_MTIME_SLACK_S
        hi = time.time() + _SVLOG_MTIME_SLACK_S
        if not root.exists():
            return adopted
        for f in root.rglob("*.svlog"):
            if session in f.parents:
                continue
            try:
                if lo <= f.stat().st_mtime <= hi:
                    dest = session # / "svlog"
                    # dest.mkdir(exist_ok=True)
                    shutil.move(str(f), dest / f.name)
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
        with open(session / "metadata.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
