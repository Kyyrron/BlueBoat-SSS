"""Offline .svlog reading and processing.

Turns a SonarView .svlog (as written by sss_processor_node via
svlog_helper.py) into the same model stream the live application
consumes — SonarPing + RobotState + GPS origin — so the replay window
and the dataset generator reuse the entire existing GUI/service stack
unchanged.

Format layer (ports from the team's ``svlog_to_rosbag.py``, kept
byte-identical in behaviour):

* ``walk_packets``            — BR-framed Cerulean Ping Protocol stream;
* ``decode_os_mono_profile``  — OS_MONO_PROFILE (id 2198) payload,
  ``<IIIIIHHHBBffffff`` head + u16 pwr_results;
* burst-aware synthetic clock — mavlink messages sharing a
  ``time_boot_ms`` share a stamp; any new tick advances by 20 ms
  (``Converter._tick`` semantics);
* NED→ENU position and attitude conversion (REP-103/105, identical
  formulas).

Processing layer (faithful port of the ``sss_processor_node`` pipeline +
``sss_helper.py``, constants copied verbatim — keep in sync with the
robot-side repo if those are ever retuned):

    raw u16 → scale_to_db → ringing-adaptive noise window → FBR
    detection per side → dual-side FBRTracker (independent bootstrap,
    max() fusion) → slant-range correction dropping the water column →
    port/stbd pairing within 50 ms → pose snap from the synthesized odom.
"""

from __future__ import annotations

import json
import math
import struct
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Deque, Iterator, List, Optional, Tuple

import numpy as np

from ..models.robot_state import RobotState
from ..models.sonar import SonarPing
from ..utils.geodesy import yaw_to_compass_deg

# ---------------------------------------------------------------------------
# Packet constants (svlog_helper.py conventions)
# ---------------------------------------------------------------------------
JSON_WRAPPER_ID = 10
MAVLINK_WRAPPER_ID = 150
OS_MONO_PROFILE_ID = 2198
DEVICE_ID_PORT = 1
DEVICE_ID_STBD = 2

NS_PER_TICK = 20_000_000          # 20 ms — svlog_to_rosbag.Converter
_HEAD_FMT = "<IIIIIHHHBBffffff"   # OS_MONO_PROFILE head
_HEAD_SIZE = struct.calcsize(_HEAD_FMT)

# Processing constants — verbatim from sss_processor_node.py.
NOISE_FLOOR_WINDOW = 20
FBR_THRESHOLD_DELTA_DB = 8.0
WITHIN_PING_PERSISTENCE = 3
RINGING_SEARCH_MAX = 60
RINGING_DROP_DB = 10.0
RINGING_PERSISTENCE = 5
BOOTSTRAP_PINGS = 10
ALTITUDE_AGREEMENT_TOL_M = 0.30
OUTLIER_TOL_M = 1.0
RELOCK_AFTER = 15
TRANSDUCER_SUBMERSION_M = 0.0
TRANSDUCER_Y_OFFSET_PORT_M = 0.0
TRANSDUCER_Y_OFFSET_STBD_M = 0.0
PAIR_TOLERANCE_NS = 50_000_000    # 50 ms processor tolerance


# ---------------------------------------------------------------------------
# Format layer
# ---------------------------------------------------------------------------
def walk_packets(data: bytes) -> Iterator[bytes]:
    """Yield framed packets from a svlog stream, skipping junk bytes."""
    pos, n = 0, len(data)
    while pos < n - 8:
        if data[pos:pos + 2] != b"BR":
            pos += 1
            continue
        plen = struct.unpack_from("<H", data, pos + 2)[0]
        total = 8 + plen + 2
        if pos + total > n:
            return                                  # truncated tail
        yield data[pos:pos + total]
        pos += total


def decode_os_mono_profile(payload: bytes) -> dict:
    """OS_MONO_PROFILE payload -> named fields (raises ValueError)."""
    if len(payload) < _HEAD_SIZE:
        raise ValueError("payload too short")
    (ping_number, start_mm, length_mm, timestamp_ms, ping_hz,
     gain_index, num_results, sos_dmps, channel_number, _res,
     pulse_duration_sec, analog_gain, max_pwr_db, min_pwr_db,
     transducer_heading_deg, vehicle_heading_deg) = struct.unpack(
        _HEAD_FMT, payload[:_HEAD_SIZE])
    expected = _HEAD_SIZE + 2 * num_results
    if len(payload) < expected:
        raise ValueError("payload truncated")
    pwr = np.frombuffer(payload, dtype="<u2", count=num_results,
                        offset=_HEAD_SIZE).astype(np.float32)
    return {"start_mm": start_mm, "length_mm": length_mm,
            "num_results": num_results, "max_pwr_db": max_pwr_db,
            "min_pwr_db": min_pwr_db, "pwr": pwr,
            "timestamp_ms": timestamp_ms, "ping_number": ping_number,
            # Authoritative side identity (see load_svlog): the device's
            # own channel tag, plus the transducer bearing as fallback.
            "channel_number": channel_number, "gain_index": gain_index,
            "transducer_heading_deg": transducer_heading_deg}


def ned_to_enu_xyz(x_n: float, y_e: float, z_d: float):
    return (y_e, x_n, -z_d)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    return math.pi / 2.0 - yaw_ned


# ---------------------------------------------------------------------------
# Processing layer (ports of sss_helper.py)
# ---------------------------------------------------------------------------
def scale_to_db(pwr: np.ndarray, min_db: float, max_db: float) -> np.ndarray:
    return (min_db + (pwr / 65535.0) * (max_db - min_db)).astype(np.float32)


def find_noise_window_start(db: np.ndarray, search_max=RINGING_SEARCH_MAX,
                            drop_db=RINGING_DROP_DB,
                            persistence=RINGING_PERSISTENCE,
                            fallback=30) -> int:
    n = len(db)
    if n < search_max + persistence:
        return fallback
    target = float(db[:search_max].max()) - drop_db
    below = db < target
    for i in range(search_max - persistence + 1):
        if below[i:i + persistence].all():
            return i
    return fallback


def detect_fbr_slant_m(db: np.ndarray, start_mm: int, length_mm: int,
                       num_results: int) -> Optional[float]:
    nw_start = find_noise_window_start(db)
    nw_end = nw_start + NOISE_FLOOR_WINDOW
    if len(db) < nw_end + WITHIN_PING_PERSISTENCE:
        return None
    threshold = float(db[nw_start:nw_end].mean()) + FBR_THRESHOLD_DELTA_DB
    above = db > threshold
    for i in range(nw_end, len(db) - WITHIN_PING_PERSISTENCE + 1):
        if above[i:i + WITHIN_PING_PERSISTENCE].all():
            slant_mm = start_mm + (i / max(num_results - 1, 1)) * length_mm
            return slant_mm / 1000.0
    return None


def project_side(db: np.ndarray, start_mm: int, length_mm: int,
                 num_results: int, altitude_m: float,
                 y_offset_m: float, side_sign: float
                 ) -> Tuple[np.ndarray, np.ndarray]:
    """Slant-range correction dropping the water column (vectorized)."""
    i = np.arange(len(db), dtype=np.float64)
    slant = start_mm / 1000.0 + (i / max(num_results - 1, 1)) * (length_mm / 1000.0)
    keep = slant > altitude_m
    ground = np.sqrt(np.maximum(slant[keep] ** 2 - altitude_m ** 2, 0.0))
    y = side_sign * (y_offset_m + ground)
    return y.astype(np.float64), db[keep].astype(np.float32)


class _SideTracker:
    def __init__(self) -> None:
        self._window: Deque[float] = deque(maxlen=BOOTSTRAP_PINGS)
        self._altitude: Optional[float] = None
        self._reject = 0
        self._miss = 0

    def update(self, fbr: Optional[float]) -> Optional[float]:
        if self._altitude is None:
            if fbr is None:
                self._miss += 1
                if self._miss >= RELOCK_AFTER:
                    self._window.clear()
                    self._miss = 0
                return None
            self._miss = 0
            self._window.append(fbr)
            if (len(self._window) == BOOTSTRAP_PINGS
                    and max(self._window) - min(self._window)
                    <= ALTITUDE_AGREEMENT_TOL_M):
                self._altitude = sum(self._window) / len(self._window)
            return self._altitude
        if fbr is not None and abs(fbr - self._altitude) <= OUTLIER_TOL_M:
            self._altitude = fbr
            self._reject = 0
        else:
            self._reject += 1
        if self._reject >= RELOCK_AFTER:
            self._altitude = None
            self._window.clear()
            self._reject = self._miss = 0
        return self._altitude


class FBRTracker:
    """Dual-side altitude tracker with a *provisional* output.

    The original tracker returned ``None`` until ten consecutive
    detections agreed to within 0.30 m, and the caller dropped every
    ping until then — which silently discarded the start of every
    mission and, whenever the lock was lost, arbitrary chunks in the
    middle. SonarView displays data from the very first ping, so we
    match that: ``update`` always returns the best altitude available
    (locked > provisional > last known), and ``locked`` reports whether
    the strict agreement criterion is currently satisfied, so callers
    can flag quality without throwing data away.
    """

    def __init__(self) -> None:
        self._port = _SideTracker()
        self._stbd = _SideTracker()
        self._altitude: Optional[float] = None
        self._last_known: Optional[float] = None
        self.locked = False

    def update(self, port_alt, stbd_alt) -> Optional[float]:
        p, s = self._port.update(port_alt), self._stbd.update(stbd_alt)
        if p is not None and s is not None:
            self._altitude = max(p, s)
        elif p is not None:
            self._altitude = p
        elif s is not None:
            self._altitude = s
        else:
            self._altitude = None
        self.locked = self._altitude is not None
        if self._altitude is not None:
            self._last_known = self._altitude
            return self._altitude
        # Not locked: provisional estimate from the raw detections so the
        # ping is still usable, then fall back to the last known value.
        raw = [v for v in (port_alt, stbd_alt) if v is not None]
        if raw:
            self._last_known = max(raw)
            return self._last_known
        return self._last_known


def resolve_altitude(tracker: "FBRTracker", port_alt, stbd_alt,
                     mode: str = "auto",
                     manual_m: float = 0.0) -> Tuple[float, bool]:
    """Depth-compensation policy -> (altitude_m, locked).

    Mirrors SonarView's "Depth Compensation" source selector:

    * ``auto``   — bottom detection (our FBR tracker), provisional value
      accepted so no ping is ever discarded; falls back to 0.0 (no
      correction) while nothing has ever been detected;
    * ``manual`` — a fixed operator-supplied altitude;
    * ``off``    — altitude 0: ground range = slant range, no water
      column removed. This is SonarView's "Manual / 0 m" mode, which on
      shallow data (h << R) is very close to the corrected geometry and
      is far more robust than a wrong altitude, because an over-estimated
      altitude both deletes real samples and warps the near range.
    """
    if mode == "off":
        return 0.0, True
    if mode == "manual":
        return float(manual_m), True
    alt = tracker.update(port_alt, stbd_alt)
    if alt is None:
        return 0.0, False
    return float(alt), tracker.locked


# ---------------------------------------------------------------------------
# Mission model + reader
# ---------------------------------------------------------------------------
@dataclass
class SvlogMission:
    """Fully decoded + processed mission, ready to feed the GUI stack.

    ``events`` is time-sorted: ("ping", t_s, SonarPing) and
    ("state", t_s, RobotState); t_s is seconds from mission start on the
    synthetic burst-aware clock.
    """

    path: Path
    events: List[tuple] = field(default_factory=list)
    duration_s: float = 0.0
    origin: Optional[Tuple[float, float]] = None    # (lat, lon) at (x0, y0)
    origin_xy: Tuple[float, float] = (0.0, 0.0)
    ping_count: int = 0
    dropped_bootstrap: int = 0      # legacy field, kept for compatibility
    dropped_no_pose: int = 0        # no odom had arrived yet
    unlocked_pings: int = 0         # emitted with a provisional altitude
    both_sides: int = 0             # rows carrying port AND starboard

    @property
    def pings(self) -> List[SonarPing]:
        return [e[2] for e in self.events if e[0] == "ping"]


def load_svlog(path: Path,
               progress: Optional[Callable[[float], None]] = None,
               depth_mode: str = "auto",
               manual_depth_m: float = 0.0) -> SvlogMission:
    """Read + process an entire .svlog into a SvlogMission.

    Two behaviours differ deliberately from the original implementation,
    both driven by measurements on real logs (see
    docs/SONARVIEW_SVLOG_ANALYSIS.md):

    1. **Side routing uses the packet's own ``channel_number``**, never
       the device/``src`` tag. In a real dual-Omniscan recording 19.8 %
       of packets carried the wrong ``src``, which put starboard data on
       the port side (and vice-versa) and produced the mirrored mosaic.
       ``channel_number`` and ``transducer_heading_deg`` agree on 100 %
       of packets in every log inspected, so the packet is trusted over
       its envelope.
    2. **Pings are assembled, never paired-and-dropped.** Profiles are
       grouped by ``ping_number`` and every group is emitted, even when
       only one side is present (that also makes single-transducer logs,
       like Cerulean's own harbour demo, load normally). The previous
       50 ms pairing threw away 10.4 % of the available rows.

    ``progress`` (0..1) is called periodically for GUI progress dialogs.
    """
    data = Path(path).read_bytes()
    mission = SvlogMission(path=Path(path))

    clock_ns = 0
    last_boot_ms: Optional[int] = None

    def tick(boot_ms: Optional[int]) -> int:
        nonlocal clock_ns, last_boot_ms
        if boot_ms is not None and boot_ms == last_boot_ms:
            return clock_ns
        clock_ns += NS_PER_TICK
        last_boot_ms = boot_ms
        return clock_ns

    # Pose synthesis state (ATTITUDE + LOCAL_POSITION_NED -> odom).
    latest_yaw: Optional[float] = None
    latest_lpn: Optional[dict] = None
    last_state_emit_ns = -10**18
    state_period_ns = int(1e9 / 5.0)                # 5 Hz, like live GUI
    cur_pose: Optional[Tuple[float, float, float]] = None
    cur_speed = 0.0

    #: ping_number -> {channel: profile}; poses/clock captured on arrival.
    groups: dict = {}
    order: List[int] = []

    def emit_state(t_ns: int) -> None:
        nonlocal last_state_emit_ns
        if cur_pose is None or t_ns - last_state_emit_ns < state_period_ns:
            return
        last_state_emit_ns = t_ns
        x, y, yaw = cur_pose
        lat = lon = None
        if mission.origin is not None:
            from ..utils.geodesy import enu_to_gps
            lat, lon = enu_to_gps(mission.origin[0], mission.origin[1],
                                  x - mission.origin_xy[0],
                                  y - mission.origin_xy[1])
        mission.events.append(("state", t_ns / 1e9, RobotState(
            t=t_ns / 1e9, x=x, y=y, yaw=yaw, lat=lat, lon=lon,
            heading_deg=yaw_to_compass_deg(yaw), speed_mps=cur_speed)))

    packets = list(walk_packets(data))
    for k, pkt in enumerate(packets):
        if progress is not None and k % 2000 == 0:
            progress(0.5 * k / max(len(packets), 1))
        pid = struct.unpack_from("<H", pkt, 4)[0]
        payload = pkt[8:-2]
        if pid == OS_MONO_PROFILE_ID:
            t_ns = tick(None)
            try:
                d = decode_os_mono_profile(payload)
            except ValueError:
                continue
            # Authoritative side: the packet's own channel_number, with
            # the transducer bearing as a fallback for exotic writers.
            ch = d["channel_number"]
            if ch not in (0, 1):
                ch = 1 if d["transducer_heading_deg"] > 0 else 0
            key = d["ping_number"]
            g = groups.get(key)
            if g is None:
                g = groups[key] = {"t_ns": t_ns, "pose": cur_pose}
                order.append(key)
            g[ch] = d
        elif pid == MAVLINK_WRAPPER_ID:
            try:
                m = json.loads(payload.decode("utf-8")).get("message", {})
            except (ValueError, UnicodeDecodeError):
                continue
            t_ns = tick(m.get("time_boot_ms"))
            mtype = m.get("type")
            if mtype == "ATTITUDE":
                latest_yaw = yaw_ned_to_enu(float(m.get("yaw", 0.0)))
            elif mtype == "LOCAL_POSITION_NED":
                latest_lpn = m
            elif mtype == "GLOBAL_POSITION_INT" and mission.origin is None:
                lat = float(m.get("lat", 0)) / 1e7
                lon = float(m.get("lon", 0)) / 1e7
                if abs(lat) > 1e-6 and cur_pose is not None:
                    mission.origin = (lat, lon)
                    mission.origin_xy = (cur_pose[0], cur_pose[1])
            if latest_yaw is not None and latest_lpn is not None:
                px, py, _ = ned_to_enu_xyz(float(latest_lpn.get("x", 0.0)),
                                           float(latest_lpn.get("y", 0.0)),
                                           float(latest_lpn.get("z", 0.0)))
                vx, vy, _ = ned_to_enu_xyz(float(latest_lpn.get("vx", 0.0)),
                                           float(latest_lpn.get("vy", 0.0)),
                                           float(latest_lpn.get("vz", 0.0)))
                cur_pose = (px, py, latest_yaw)
                cur_speed = math.hypot(vx, vy)
                emit_state(t_ns)

    # ---- assemble: one row per ping_number, one-sided rows included ----
    fbr = FBRTracker()
    order.sort()                       # restores acquisition order exactly
    for i, key in enumerate(order):
        if progress is not None and i % 500 == 0:
            progress(0.5 + 0.5 * i / max(len(order), 1))
        g = groups[key]
        pose = g["pose"]
        if pose is None:
            mission.dropped_no_pose += 1
            continue                   # genuinely unplaceable (no odom yet)
        p, s = g.get(0), g.get(1)
        db_p = alt_p = db_s = alt_s = None
        if p is not None:
            db_p = scale_to_db(p["pwr"], p["min_pwr_db"], p["max_pwr_db"])
            alt_p = detect_fbr_slant_m(db_p, p["start_mm"], p["length_mm"],
                                       p["num_results"])
        if s is not None:
            db_s = scale_to_db(s["pwr"], s["min_pwr_db"], s["max_pwr_db"])
            alt_s = detect_fbr_slant_m(db_s, s["start_mm"], s["length_mm"],
                                       s["num_results"])
        altitude, locked = resolve_altitude(fbr, alt_p, alt_s,
                                            depth_mode, manual_depth_m)
        if not locked:
            mission.unlocked_pings += 1
        ys, ins = [], []
        if p is not None:
            y, iv = project_side(db_p, p["start_mm"], p["length_mm"],
                                 p["num_results"], altitude,
                                 TRANSDUCER_Y_OFFSET_PORT_M, +1.0)
            ys.append(y); ins.append(iv)
        if s is not None:
            y, iv = project_side(db_s, s["start_mm"], s["length_mm"],
                                 s["num_results"], altitude,
                                 TRANSDUCER_Y_OFFSET_STBD_M, -1.0)
            ys.append(y); ins.append(iv)
        if not ys:
            continue
        ref = p if p is not None else s
        x, y0, yaw = pose
        mission.events.append(("ping", g["t_ns"] / 1e9, SonarPing(
            t=g["t_ns"] / 1e9, robot_x=x, robot_y=y0, yaw=yaw,
            water_depth=altitude + TRANSDUCER_SUBMERSION_M,
            y_local=np.concatenate(ys),
            intensity_db=np.concatenate(ins),
            slant_range_m=ref["length_mm"] / 1000.0,
            sides=("both" if (p is not None and s is not None)
                   else ("port" if p is not None else "starboard")))))
        mission.ping_count += 1
        if p is not None and s is not None:
            mission.both_sides += 1

    mission.events.sort(key=lambda e: e[1])
    if mission.events:
        from dataclasses import replace as dc_replace
        t0 = mission.events[0][1]
        mission.events = [(kind, t - t0, dc_replace(obj, t=obj.t - t0))
                          for kind, t, obj in mission.events]
        mission.duration_s = mission.events[-1][1]
    if progress is not None:
        progress(1.0)
    return mission
