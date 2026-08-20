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
            "timestamp_ms": timestamp_ms}


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
    def __init__(self) -> None:
        self._port = _SideTracker()
        self._stbd = _SideTracker()
        self._altitude: Optional[float] = None

    def update(self, port_alt, stbd_alt) -> Optional[float]:
        p, s = self._port.update(port_alt), self._stbd.update(stbd_alt)
        if p is not None and s is not None:
            self._altitude = max(p, s)
        elif p is not None:
            self._altitude = p
        elif s is not None:
            self._altitude = s
        return self._altitude


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
    dropped_bootstrap: int = 0

    @property
    def pings(self) -> List[SonarPing]:
        return [e[2] for e in self.events if e[0] == "ping"]


def load_svlog(path: Path,
               progress: Optional[Callable[[float], None]] = None
               ) -> SvlogMission:
    """Read + process an entire .svlog into a SvlogMission.

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

    # Pairing + processing state.
    port_buf: Deque[Tuple[int, dict]] = deque()
    stbd_buf: Deque[Tuple[int, dict]] = deque()
    fbr = FBRTracker()
    cur_pose: Optional[Tuple[float, float, float]] = None   # x, y, yaw
    cur_speed = 0.0

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

    def try_pair() -> None:
        while port_buf and stbd_buf:
            (p_ns, p), (s_ns, s) = port_buf[0], stbd_buf[0]
            dt = p_ns - s_ns
            if abs(dt) <= PAIR_TOLERANCE_NS:
                port_buf.popleft()
                stbd_buf.popleft()
                process_pair(p_ns, p, s)
            elif dt > 0:
                stbd_buf.popleft()
            else:
                port_buf.popleft()

    def process_pair(t_ns: int, p: dict, s: dict) -> None:
        if cur_pose is None:
            return                                   # no odom yet: drop
        p_db = scale_to_db(p["pwr"], p["min_pwr_db"], p["max_pwr_db"])
        s_db = scale_to_db(s["pwr"], s["min_pwr_db"], s["max_pwr_db"])
        p_alt = detect_fbr_slant_m(p_db, p["start_mm"], p["length_mm"],
                                   p["num_results"])
        s_alt = detect_fbr_slant_m(s_db, s["start_mm"], s["length_mm"],
                                   s["num_results"])
        altitude = fbr.update(p_alt, s_alt)
        if altitude is None:
            mission.dropped_bootstrap += 1
            return
        p_y, p_i = project_side(p_db, p["start_mm"], p["length_mm"],
                                p["num_results"], altitude,
                                TRANSDUCER_Y_OFFSET_PORT_M, +1.0)
        s_y, s_i = project_side(s_db, s["start_mm"], s["length_mm"],
                                s["num_results"], altitude,
                                TRANSDUCER_Y_OFFSET_STBD_M, -1.0)
        x, y, yaw = cur_pose
        mission.events.append(("ping", t_ns / 1e9, SonarPing(
            t=t_ns / 1e9, robot_x=x, robot_y=y, yaw=yaw,
            water_depth=altitude + TRANSDUCER_SUBMERSION_M,
            y_local=np.concatenate([p_y, s_y]),
            intensity_db=np.concatenate([p_i, s_i]))))
        mission.ping_count += 1

    packets = list(walk_packets(data))
    for k, pkt in enumerate(packets):
        if progress is not None and k % 2000 == 0:
            progress(k / max(len(packets), 1))
        pid = struct.unpack_from("<H", pkt, 4)[0]
        src = pkt[6]
        payload = pkt[8:-2]
        if pid == OS_MONO_PROFILE_ID and src in (DEVICE_ID_PORT,
                                                 DEVICE_ID_STBD):
            t_ns = tick(None)
            try:
                d = decode_os_mono_profile(payload)
            except ValueError:
                continue
            (port_buf if src == DEVICE_ID_PORT else stbd_buf).append((t_ns, d))
            try_pair()
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
