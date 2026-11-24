"""Merge two recorded ``.svlog`` files into ONE new log.

Purpose: visualize two related recording sessions in a single replay
run. The merged file is a brand-new artifact — **both sources are opened
read-only and never modified, moved or renamed** (NC #6). The output is
a normal multi-session ``.svlog`` that ``load_svlog`` (and SonarView)
reads with no special handling: the older log becomes session 1, the
newer becomes session 2, exactly like a log recorded in two sittings.

How the loader's structure is exploited (core/svlog.py):

* **Segments come from id-10 session headers and nothing else.** Each
  source keeps its own header(s) (one is synthesized for a log that has
  none), so the two halves land in separate segments — which makes the
  grouping key ``(segment, ping_number)`` collision-free without ever
  touching ``ping_number``, and keeps ``usable_stamps`` from comparing
  the two device clocks against each other.
* **The timeline is the sonar's own ``timestamp_ms``.** The newer log's
  stamps get one constant shift Δ so its first ping lands exactly one
  median ping interval after the older log's last ping ("right after",
  per the operator requirement). The resulting inter-segment gap is one
  PRI > 0, so the loader still emits a ``MissionGap`` and every
  accumulator (mosaic tracking, waterfall seam, seabed-image window,
  trajectory segment) breaks at the boundary instead of stitching two
  survey areas together.
* **Boot skew is one file-wide median** (``estimate_boot_skew``), so the
  newer log's mavlink ``time_boot_ms`` is shifted by
  ``δ = Δ + skew_B − skew_A``: after the rewrite, every one of its skew
  votes reproduces the older log's skew and the single median holds.
* **Local poses are per-boot.** When BOTH logs carry a GPS origin, the
  newer log's ``LOCAL_POSITION_NED`` x/y are shifted by the constant
  that expresses its frame in the older log's (via the two origins'
  geodetic offset) — no consistency checks, per the operator decision;
  without GPS on either side the poses pass through untouched with a
  warning. ``GLOBAL_POSITION_INT`` (absolute) is never rewritten, and
  the loader takes its mission origin from the FIRST fix — the older
  log's — by construction.

ROS-free (struct/json/pathlib/numpy + the stdlib-only verbatim
``tools/svlog_helper`` for framing). NC #7: the tools module is
imported, never edited.
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from ..tools import svlog_helper
from ..utils.geodesy import gps_to_enu
from .svlog import (JSON_WRAPPER_ID, MAVLINK_WRAPPER_ID, OMNISCAN_STATUS_ID,
                    OS_MONO_PROFILE_ID, decode_session_header, ned_to_enu_xyz,
                    walk_packets)

_U32_MAX = 0xFFFFFFFF
#: Frame offsets of the u32 ``timestamp_ms`` (8-byte frame header +
#: payload offset). id-2198: payload offset 12 (``<IIIIIHHHBBffffff``,
#: field 4). id-2194: payload offset 8 (``<ffI3xB``).
_PROFILE_TS_FRAME_OFF = 8 + 12
_STATUS_TS_FRAME_OFF = 8 + 8
_DEFAULT_PRI_MS = 50


@dataclass
class MergeReport:
    """What the merge did — shown to the operator and asserted by tests."""

    older: Path
    newer: Path
    swapped: bool                      # inputs arrived newer-first
    delta_ms: int                      # sonar-clock shift applied to `newer`
    boot_delta_ms: Optional[int]       # mavlink shift (None: no mavlink in B)
    pose_offset_en: Optional[Tuple[float, float]]  # ENU metres, None = identity
    rewritten: Dict[int, int] = field(default_factory=dict)  # packet id -> n
    warnings: List[str] = field(default_factory=list)

    def summary(self) -> str:
        lines = [
            f"older : {self.older.name}",
            f"newer : {self.newer.name} (shifted +{self.delta_ms} ms)",
        ]
        if self.pose_offset_en is not None:
            lines.append("poses : aligned via GPS origins "
                         f"(ΔE {self.pose_offset_en[0]:+.1f} m, "
                         f"ΔN {self.pose_offset_en[1]:+.1f} m)")
        else:
            lines.append("poses : passed through (no GPS on both logs)")
        for w in self.warnings:
            lines.append(f"warning: {w}")
        return "\n".join(lines)


@dataclass
class _FileFacts:
    """Everything one scan pass learns about a source log."""

    path: Path
    packets: List[bytes]
    wall_clock: Optional[datetime] = None
    first_ts: Optional[int] = None     # first/last profile timestamp_ms
    last_ts: Optional[int] = None
    max_ts: int = 0
    pri_ms: int = _DEFAULT_PRI_MS      # median ping interval (best channel)
    skew_ms: Optional[int] = None      # median(profile_ts - time_boot_ms)
    has_header_first: bool = False     # id-10 precedes the first profile
    origin: Optional[Tuple[float, float]] = None       # first GPS fix
    origin_xy: Optional[Tuple[float, float]] = None    # ENU pose at that fix


def _parse_wall_clock(value) -> Optional[datetime]:
    if not isinstance(value, str):
        return None
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None


def _scan(path: Path) -> _FileFacts:
    """One pass over a log: clocks, PRI, boot skew, GPS origin.

    Mirrors the loader's own conventions (nearest-preceding-profile skew
    votes; origin = first GLOBAL_POSITION_INT with a non-zero latitude
    AND a pose already established) so the merged file behaves under
    ``load_svlog`` exactly as this scan predicts.
    """
    facts = _FileFacts(path=path, packets=list(walk_packets(path.read_bytes())))
    per_channel_ts: Dict[int, List[int]] = {}
    skew_votes: List[int] = []
    boot_values: set = set()
    last_profile_ts: Optional[int] = None
    latest_yaw_seen = False
    latest_lpn: Optional[dict] = None
    cur_pose: Optional[Tuple[float, float]] = None
    seen_profile = False
    for pkt in facts.packets:
        pid = struct.unpack_from("<H", pkt, 4)[0]
        payload = pkt[8:-2]
        if pid == JSON_WRAPPER_ID:
            if not seen_profile:
                facts.has_header_first = True
            if facts.wall_clock is None:
                facts.wall_clock = _parse_wall_clock(
                    decode_session_header(payload)["wall_clock"])
        elif pid == OS_MONO_PROFILE_ID:
            if len(payload) < 27:
                continue
            seen_profile = True
            ts = struct.unpack_from("<I", payload, 12)[0]
            ch = payload[26]
            per_channel_ts.setdefault(ch, []).append(ts)
            last_profile_ts = ts
            if facts.first_ts is None:
                facts.first_ts = ts
            facts.last_ts = ts
            facts.max_ts = max(facts.max_ts, ts)
        elif pid == MAVLINK_WRAPPER_ID:
            try:
                m = json.loads(payload.decode("utf-8")).get("message", {})
            except (ValueError, UnicodeDecodeError):
                continue
            boot = m.get("time_boot_ms")
            if boot is not None:
                boot_values.add(int(boot))
                if last_profile_ts is not None:
                    skew_votes.append(last_profile_ts - int(boot))
            mtype = m.get("type")
            if mtype == "ATTITUDE":
                latest_yaw_seen = True
            elif mtype == "LOCAL_POSITION_NED":
                latest_lpn = m
            elif mtype == "GLOBAL_POSITION_INT" and facts.origin is None:
                lat = float(m.get("lat", 0)) / 1e7
                lon = float(m.get("lon", 0)) / 1e7
                if abs(lat) > 1e-6 and cur_pose is not None:
                    facts.origin = (lat, lon)
                    facts.origin_xy = cur_pose
            if latest_yaw_seen and latest_lpn is not None:
                px, py, _ = ned_to_enu_xyz(float(latest_lpn.get("x", 0.0)),
                                           float(latest_lpn.get("y", 0.0)),
                                           float(latest_lpn.get("z", 0.0)))
                cur_pose = (px, py)
    # PRI: the median positive interval of the busiest channel.
    best = max(per_channel_ts.values(), key=len, default=[])
    if len(best) >= 3:
        diffs = np.diff(np.asarray(best, dtype=np.int64))
        diffs = diffs[diffs > 0]
        if diffs.size:
            facts.pri_ms = int(max(1, np.median(diffs)))
    # Boot skew: same rule as the loader — a single frozen time_boot_ms
    # carries no clock, so the skew is only trusted past one value.
    if skew_votes and len(boot_values) > 1:
        facts.skew_ms = int(np.median(np.asarray(skew_votes, dtype=np.int64)))
    return facts


def _patch_u32(pkt: bytes, frame_off: int, value: int) -> bytes:
    """Rewrite one u32 field in a framed packet + fix the checksum
    (same recompute as ``svlog_helper.retag_packet_src_device_id``)."""
    buf = bytearray(pkt)
    struct.pack_into("<I", buf, frame_off, value & _U32_MAX)
    plen = struct.unpack_from("<H", buf, 2)[0]
    struct.pack_into("<H", buf, 8 + plen, sum(buf[:8 + plen]) & 0xFFFF)
    return bytes(buf)


def _default_name(older: _FileFacts, newer: _FileFacts) -> str:
    """``merged_<DATE>_<MMSS of the older>_<MMSS of the newer>``."""
    def mmss(facts: _FileFacts) -> str:
        return (facts.wall_clock.strftime("%M%S")
                if facts.wall_clock is not None
                else datetime.fromtimestamp(
                    facts.path.stat().st_mtime).strftime("%M%S"))

    date = (older.wall_clock.strftime("%Y_%m_%d")
            if older.wall_clock is not None
            else datetime.fromtimestamp(
                older.path.stat().st_mtime).strftime("%Y_%m_%d"))
    return f"merged_{date}_{mmss(older)}_{mmss(newer)}"


def default_merge_name(path_a: Path, path_b: Path) -> str:
    """The suggested merged-session name for two logs (older first)."""
    a, b = _scan(Path(path_a)), _scan(Path(path_b))
    if _b_is_older(a, b):
        a, b = b, a
    return _default_name(a, b)


def _b_is_older(a: _FileFacts, b: _FileFacts) -> bool:
    """True when ``b`` was recorded before ``a`` (wall clock, else mtime)."""
    if a.wall_clock is not None and b.wall_clock is not None:
        return b.wall_clock < a.wall_clock
    return b.path.stat().st_mtime < a.path.stat().st_mtime


def merge_svlogs(path_a: Path, path_b: Path, out_path: Path,
                 progress: Optional[Callable[[float], None]] = None
                 ) -> MergeReport:
    """Write ``out_path`` = older log + shifted newer log; see module doc.

    The inputs may arrive in either order — the older one (first id-10
    wall clock, falling back to file mtime) is auto-detected and always
    goes first. Raises ValueError for a self-merge or a log with no
    sonar profiles.
    """
    path_a, path_b, out_path = Path(path_a), Path(path_b), Path(out_path)
    if path_a.resolve() == path_b.resolve():
        raise ValueError("cannot merge a log with itself")
    if progress:
        progress(0.05)
    a, b = _scan(path_a), _scan(path_b)
    if progress:
        progress(0.35)
    swapped = _b_is_older(a, b)
    if swapped:
        a, b = b, a
    for facts in (a, b):
        if facts.first_ts is None:
            raise ValueError(
                f"{facts.path.name} contains no sonar profiles — nothing "
                "to merge")

    report = MergeReport(older=a.path, newer=b.path, swapped=swapped,
                         delta_ms=0, boot_delta_ms=None, pose_offset_en=None)

    # --- the three constants ------------------------------------------------
    delta = (a.last_ts + a.pri_ms) - b.first_ts
    report.delta_ms = int(delta)
    if b.max_ts + delta > _U32_MAX:
        report.warnings.append(
            "shifted timestamp_ms exceeds the u32 range — the merged "
            "timeline will wrap and fall back to a synthetic clock")
    if a.skew_ms is not None and b.skew_ms is not None:
        boot_delta = delta + b.skew_ms - a.skew_ms
    else:
        boot_delta = delta
        if b.skew_ms is None or a.skew_ms is None:
            report.warnings.append(
                "boot skew unrecoverable on one log (frozen or missing "
                "time_boot_ms) — mavlink stamps shifted by Δ only")
    report.boot_delta_ms = int(boot_delta)

    dx_ned = dy_ned = 0.0
    if (a.origin is not None and b.origin is not None
            and a.origin_xy is not None and b.origin_xy is not None):
        d_e, d_n = gps_to_enu(a.origin[0], a.origin[1],
                              b.origin[0], b.origin[1])
        off_e = a.origin_xy[0] + d_e - b.origin_xy[0]
        off_n = a.origin_xy[1] + d_n - b.origin_xy[1]
        report.pose_offset_en = (float(off_e), float(off_n))
        # ENU -> NED: ned_x = north, ned_y = east (ned_to_enu_xyz inverse).
        dx_ned, dy_ned = off_n, off_e
    else:
        report.warnings.append(
            "no GPS origin on both logs — local poses passed through; the "
            "two areas draw in their own frames")

    # --- write --------------------------------------------------------------
    rewritten = report.rewritten
    part = out_path.with_name(out_path.name + ".part")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    n_total = len(a.packets) + len(b.packets)
    with open(part, "wb") as f:
        for pkt in a.packets:              # the older log, byte-identical
            f.write(pkt)
        if not b.has_header_first:
            # Without its own header the newer log's profiles would bleed
            # into the older log's last segment (segments exist only where
            # an id-10 packet does).
            stamp = (b.wall_clock or
                     datetime.now(timezone.utc)).strftime(
                         "%Y-%m-%dT%H:%M:%S.000Z")
            payload = json.dumps({
                "session_id": f"merged:{b.path.stem}",
                "timestamp": stamp,
                "session_devices": None,
                "merged_from": [a.path.name, b.path.name],
            }).encode()
            f.write(svlog_helper.frame_packet(
                JSON_WRAPPER_ID, payload, src=0,
                dst=svlog_helper.DST_BROADCAST))
            rewritten[JSON_WRAPPER_ID] = rewritten.get(JSON_WRAPPER_ID, 0) + 1
        for k, pkt in enumerate(b.packets):
            if progress and k % 2000 == 0:
                progress(0.35 + 0.6 * (len(a.packets) + k) / max(n_total, 1))
            pid = struct.unpack_from("<H", pkt, 4)[0]
            payload = pkt[8:-2]
            if pid == OS_MONO_PROFILE_ID and len(payload) >= 16:
                ts = struct.unpack_from("<I", payload, 12)[0]
                pkt = _patch_u32(pkt, _PROFILE_TS_FRAME_OFF, ts + delta)
                rewritten[pid] = rewritten.get(pid, 0) + 1
            elif pid == OMNISCAN_STATUS_ID and len(payload) >= 12:
                ts = struct.unpack_from("<I", payload, 8)[0]
                pkt = _patch_u32(pkt, _STATUS_TS_FRAME_OFF, ts + delta)
                rewritten[pid] = rewritten.get(pid, 0) + 1
            elif pid == MAVLINK_WRAPPER_ID:
                try:
                    doc = json.loads(payload.decode("utf-8"))
                    msg = doc.get("message", {})
                except (ValueError, UnicodeDecodeError):
                    doc = msg = None
                if isinstance(msg, dict):
                    if msg.get("time_boot_ms") is not None:
                        msg["time_boot_ms"] = int(msg["time_boot_ms"]) \
                            + boot_delta
                    if (msg.get("type") == "LOCAL_POSITION_NED"
                            and (dx_ned or dy_ned)):
                        msg["x"] = float(msg.get("x", 0.0)) + dx_ned
                        msg["y"] = float(msg.get("y", 0.0)) + dy_ned
                    pkt = svlog_helper.frame_packet(
                        MAVLINK_WRAPPER_ID, json.dumps(doc).encode(),
                        src=pkt[6], dst=pkt[7])
                    rewritten[pid] = rewritten.get(pid, 0) + 1
                # undecodable JSON: copy verbatim (counted nowhere — the
                # loader skips it identically in both source and merge)
            elif pid == JSON_WRAPPER_ID:
                try:
                    doc = json.loads(
                        payload.decode("utf-8", "replace").rstrip("\x00"))
                except ValueError:
                    doc = None
                if isinstance(doc, dict):
                    doc["merged_from"] = [a.path.name, b.path.name]
                    doc["merge_time_shift_ms"] = int(delta)
                    if report.pose_offset_en is not None:
                        doc["merge_pose_offset_en"] = list(
                            report.pose_offset_en)
                    pkt = svlog_helper.frame_packet(
                        JSON_WRAPPER_ID, json.dumps(doc).encode(),
                        src=pkt[6], dst=pkt[7])
                    rewritten[pid] = rewritten.get(pid, 0) + 1
            f.write(pkt)
    part.replace(out_path)
    if progress:
        progress(1.0)
    return report
