"""Rebuild a full recording-session folder from one ``.svlog``, offline.

Used by the svlog merge: the merged log is decoded once and every
artifact a *live* session carries is regenerated through the SAME
writers the live path uses — ``MosaicService.save_into``,
``WaterfallService.export_into``, ``seabed_imager.generate_from_pings``
and ``recording_session.write_session_metadata`` — so a merged session
under ``merged_sessions/`` is a normal session in every way (layout,
metadata schema, npz contents), not a look-alike.

Event feeding mirrors ``ReplayWindow._dispatch`` exactly: pings go to
the mosaic and the waterfall; a ``MissionGap`` breaks the mosaic's
tracking and inserts a waterfall seam (the seabed imager receives the
gaps as ``breaks=`` and closes its window there, so no 256-row training
tile spans two source sessions). Seabed images are numbered in
chronological order by construction — the older log's pings come first
in the merged timeline.

NC #9 note: nothing here is a new live-export path — the input is a
``.svlog`` that already left the GCS through a recording session (or
was supplied by the operator), and the output is a derived folder next
to ``sessions/``.

Qt-only, ROS-free (the services are QObjects; a QApplication must
exist, which it does in every caller — the replay window and the
offscreen test harness).
"""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from ..config.settings import AppConfig
from ..core.seabed_imager import generate_from_pings
from .mosaic_service import MosaicService
from .recording_session import write_session_metadata
from .svlog import SvlogMission, load_svlog
from .waterfall_service import WaterfallService


def rebuild_session(svlog_path: Path, session_dir: Path, config: AppConfig,
                    progress: Optional[Callable[[float], None]] = None,
                    extra_metadata: Optional[dict] = None) -> SvlogMission:
    """Decode ``svlog_path`` and write the standard session artifacts
    (mosaic/, waterfall/, seabed_images/, metadata.json) into
    ``session_dir``. Returns the decoded mission (the caller validates
    it). The ``.svlog`` itself is expected to already sit at the session
    root, matching the live adoption layout.
    """
    svlog_path, session_dir = Path(svlog_path), Path(session_dir)
    session_dir.mkdir(parents=True, exist_ok=True)

    def _p(lo: float, hi: float):
        if progress is None:
            return None
        return lambda f: progress(lo + (hi - lo) * f)

    mission = load_svlog(svlog_path, progress=_p(0.0, 0.4),
                         depth_mode=config.depth.mode,
                         manual_depth_m=config.depth.manual_m,
                         blank_nadir=config.depth.blank_nadir,
                         nadir_blank_m=config.depth.nadir_blank_m,
                         nadir_max_fraction=config.depth.nadir_max_fraction)

    # One display model for the three artifacts (the same rule as a
    # window): fitted over the whole log before anything is rendered.
    from .display_model import DisplayModel
    model = DisplayModel.fit(config, mission.pings)
    mosaic = MosaicService(config, model)
    waterfall = WaterfallService(config, model)
    waterfall.reserve(mission.ping_count + len(mission.gap_times) + 16)
    waterfall.set_enabled(True)
    events = mission.events
    for k, (kind, _t, obj) in enumerate(events):
        if progress is not None and k % 2000 == 0:
            progress(0.4 + 0.3 * k / max(len(events), 1))
        if kind == "ping":
            mosaic.on_sonar_ping(obj)
            waterfall.on_sonar_ping(obj)
        elif kind == "gap":
            # Same accumulator breaks as ReplayWindow._on_gap: no
            # interpolated swath or seam-free waterfall across the
            # boundary between the two source sessions.
            mosaic.reset_tracking()
            waterfall.break_row()

    mosaic.save_into(session_dir / "mosaic")
    waterfall.export_into(session_dir / "waterfall")
    n_images = generate_from_pings(
        mission.pings, session_dir / "seabed_images", config,
        progress=_p(0.7, 0.98), breaks=mission.gap_times, model=model)

    started = datetime.utcnow()
    meta = {
        "session": session_dir.name,
        "started_utc": started.isoformat() + "Z",
        "ended_utc": datetime.utcnow().isoformat() + "Z",
        "duration_s": round(mission.duration_s, 1),
        "ping_count": mission.ping_count,
        "detection_count": 0,
        "adopted_svlogs": [svlog_path.name],
        "mosaic": {
            "cell_size_m": mosaic.cell_size_m,
            "densify": config.mosaic.densify,
            "bilinear_splat": config.mosaic.bilinear_splat,
            "priority_mode_displayed": config.mosaic.priority_mode,
        },
        "display_settings_at_end": None,
        "display_model": model.snapshot().to_json(),
        "topics": asdict(config.topics),
        "note": ("rebuilt offline from the .svlog at the session root; "
                 "mosaic/*.npz and waterfall/waterfall_raw.npz contain "
                 "raw, unrendered data; PNGs are quick-looks only."),
        "seabed_image_count": n_images,
        "segments": len(mission.segments),
    }
    if extra_metadata:
        meta.update(extra_metadata)
    write_session_metadata(session_dir, meta)
    if progress is not None:
        progress(1.0)
    return mission
