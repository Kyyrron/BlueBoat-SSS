"""The planned mission path, asked of the path server that is actually running.

How the Mission Control Station does it, and now the GCS too
(``mcs/gui/main_window.py`` ``_request_mission_path``, ``mcs/ros/bridge_node.py``
``request_mission_path``): read the duration of the trajectory the mission was
launched with, then make ONE ``/path_request`` over ``linspace(0, duration, n)``
and draw the poses that come back. They are in the run's world (odom) frame --
the frame of ``/set_path``, of ``/blueboat/odom`` and of every other overlay --
so nothing is projected, and the path is the whole mission because the window
is the mission's own duration.

What this replaced (2026-09-18 -> 2026-09-28). The map drew the *newest*
``~/.config/blueboat_mcs/trajectories/.deployed/*.yaml``, once, at the first GPS
anchor, re-projected through the *designer* file's ``geo_anchor``. Both halves
were wrong:

* **Which mission.** MCS deletes the current mission's deployed file at launch
  and rewrites it only once its own GPS fit converges; in a simulation the GCS
  anchors first, so "newest" was another mission (``plage_lawnmower`` drawn
  during a ``plage_bathymetry2`` run) -- after which ``/set_path`` was ignored
  for the rest of the session. A built-in or non-anchored run writes no
  deployed file at all, so the newest was always some older mission.
* **Which frame.** ``deploy_mission`` (``mcs/designer/io_yaml.py``) has already
  converted the points into the run's world frame and dropped the anchor;
  re-projecting them through the designer's anchor shifted the path by the
  world's ``design_offset`` (24.5 m on every ``plage_bathymetry2`` world).

Asking ``path_generation`` which trajectory it serves (its ``trajectory`` /
``yaml_path`` parameters, exactly as it parses them) answers the first; drawing
its own answer answers the second. ``/set_path`` stays the fallback for a graph
with no path server (a rosbag replay): it is ``path_publisher``'s fixed
``total_time`` window onto the same server, truncated past it.

ROS-free and Qt-free on purpose: ``ros/path_listener.py`` is the glue, and the
tests drive :class:`PlannedPathSource` without ROS.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import yaml

_LOADER = getattr(yaml, "CSafeLoader", yaml.SafeLoader)

#: ``path_generation``'s prefix for a designer trajectory (``trajectory:=from_yaml:<file>``).
FROM_YAML = "from_yaml"
#: A failed or timed-out request is retried after this long, not every poll.
RETRY_AFTER_S = 5.0


def _read_yaml(path: Path) -> Optional[dict]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.load(f, Loader=_LOADER)
        return doc if isinstance(doc, dict) else None
    except (OSError, yaml.YAMLError):
        return None


def trajectory_file(trajectory: str, yaml_path: str = "") -> Optional[Path]:
    """The file ``path_generation`` serves, or None for a built-in shape.

    Mirrors ``path_generation.py`` verbatim: a ``from_yaml`` trajectory reads
    ``yaml_path`` when set, else what follows the first ``:``.
    """
    if not isinstance(trajectory, str) or not trajectory.startswith(FROM_YAML):
        return None
    raw = yaml_path or trajectory.partition(":")[2]
    return Path(raw).expanduser() if raw else None


@dataclass(frozen=True)
class TrajectoryInfo:
    """What the request needs to know about a trajectory file."""

    name: str
    file: Path
    #: Time of the last sample -- what ``yaml_trajectory.py`` uses as the
    #: duration (a looped mission wraps ``t`` modulo it).
    duration_s: float
    loop: bool
    mtime: float


def read_trajectory_info(path: Path) -> Optional[TrajectoryInfo]:
    """Name, duration and loop flag of a ``blueboat_trajectory/1`` file;
    None when it is missing or unreadable (not deployed yet)."""
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return None
    doc = _read_yaml(path)
    if not doc:
        return None
    duration = 0.0
    pts = doc.get("points")
    if isinstance(pts, Sequence) and pts:
        try:
            duration = float(pts[-1][0])
        except (IndexError, TypeError, ValueError):
            duration = 0.0
    if duration <= 0.0:
        try:
            duration = float(doc.get("duration_s") or 0.0)
        except (TypeError, ValueError):
            duration = 0.0
    return TrajectoryInfo(name=str(doc.get("name") or Path(path).stem),
                          file=Path(path), duration_s=max(duration, 0.0),
                          loop=bool(doc.get("loop", False)), mtime=mtime)


def request_times(horizon_s: float, dt_s: float,
                  max_points: int) -> Tuple[float, ...]:
    """``linspace(0, horizon, n)``, endpoint included, ``n`` capped.

    The endpoint matters: a looped mission's last sample is its start again,
    so the drawn loop closes.
    """
    horizon = max(float(horizon_s), 0.0)
    n = int(horizon / max(float(dt_s), 1e-6)) + 1
    n = max(2, min(n, max(int(max_points), 2)))
    return tuple(horizon * i / (n - 1) for i in range(n))


def is_holding(points, tol_m: float = 0.01) -> bool:
    """True when every pose sits on the first one.

    That is ``path_generation``'s answer while its ``from_yaml`` file is not
    loaded yet (a station-keeping pose at the origin), or a station-keeping
    trajectory -- a point, not a path to draw.
    """
    if not points:
        return True
    x0, y0 = points[0]
    return all(abs(x - x0) <= tol_m and abs(y - y0) <= tol_m
               for x, y in points)


@dataclass(frozen=True)
class PathRequest:
    """One step :meth:`PlannedPathSource.poll` asks the ROS glue to take."""

    key: tuple
    #: Path parameters to request; empty = nothing to ask (``waiting``).
    times: Tuple[float, ...]
    #: Human-readable mission description for the status line.
    label: str
    #: The trajectory file is not deployed yet: clear the overlay, say so.
    waiting: bool = False


class PlannedPathSource:
    """Which path to ask for, and when -- the ROS-free half of the listener.

    One request per deployment, like MCS: a request is keyed by the
    trajectory and, for a file, its path and mtime, and is not repeated for
    an unchanged key. That matters on the real boat, where ``master_control``
    asks the same single-threaded server at 20 Hz.
    """

    def __init__(self, dt_s: float = 0.5, default_horizon_s: float = 1000.0,
                 max_points: int = 20000) -> None:
        self.dt_s = float(dt_s)
        self.default_horizon_s = float(default_horizon_s)
        self.max_points = int(max_points)
        self._ready = False
        self._trajectory: Optional[str] = None
        self._yaml_path = ""
        self._served_key: Optional[tuple] = None
        self._retry_at = 0.0
        self._info: Optional[TrajectoryInfo] = None

    # ---- server state --------------------------------------------------------
    def service_ready(self, ready: bool) -> None:
        """The path service is (or is no longer) reachable. Losing it
        forgets the mission, so ``/set_path`` draws again."""
        ready = bool(ready)
        if self._ready and not ready:
            self._trajectory = None
            self._yaml_path = ""
            self._served_key = None
            self._info = None
        self._ready = ready

    @property
    def engaged(self) -> bool:
        """The server is up and we know what it serves."""
        return self._ready and self._trajectory is not None

    def needs_parameters(self) -> bool:
        return self._ready and self._trajectory is None

    def on_parameters(self, trajectory: str, yaml_path: str = "") -> None:
        trajectory = str(trajectory or "")
        yaml_path = str(yaml_path or "")
        if (trajectory, yaml_path) != (self._trajectory, self._yaml_path):
            self._served_key = None
        self._trajectory, self._yaml_path = trajectory, yaml_path

    def accept_set_path(self) -> bool:
        """``/set_path`` is a truncated view of the same server: it draws only
        while there is no server we can ask ourselves."""
        return not self.engaged

    def reset(self) -> None:
        """The operator cleared the overlay: ask again at the next poll."""
        self._served_key = None
        self._retry_at = 0.0

    # ---- the decision --------------------------------------------------------
    def poll(self, now: float) -> Optional[PathRequest]:
        """The next step, or None when there is nothing new to ask."""
        if not self.engaged or now < self._retry_at:
            return None
        f = trajectory_file(self._trajectory, self._yaml_path)
        if f is None:                               # built-in shape
            key = (self._trajectory,)
            if key == self._served_key:
                return None
            return PathRequest(
                key=key,
                times=request_times(self.default_horizon_s, self.dt_s,
                                    self.max_points),
                label=f"'{self._trajectory}' (first "
                      f"{self.default_horizon_s:.0f} s)")
        try:
            mtime = os.path.getmtime(f)
        except OSError:
            key = ("waiting", self._trajectory, str(f))
            if key == self._served_key:
                return None
            self._served_key = key                  # announce it once
            return PathRequest(key=key, times=(), waiting=True,
                               label=f"'{f.stem}' ({f})")
        key = (self._trajectory, str(f), mtime)
        if key == self._served_key:
            return None
        info = self._info
        if info is None or info.file != f or info.mtime != mtime:
            info = self._info = read_trajectory_info(f)
        if info is None or info.duration_s <= 0.0:
            horizon, desc = self.default_horizon_s, f"'{f.stem}'"
        else:
            horizon = info.duration_s
            desc = (f"'{info.name}' ({info.duration_s:.0f} s"
                    f"{', one lap' if info.loop else ''})")
        return PathRequest(key=key,
                           times=request_times(horizon, self.dt_s,
                                               self.max_points),
                           label=desc)

    def on_response(self, key: tuple, points) -> tuple:
        """The server answered ``key``: the points to draw (``()`` = clear)."""
        self._served_key = key
        points = tuple(points)
        return () if is_holding(points) else points

    def on_failure(self, now: float) -> None:
        """The request failed or timed out: retry later, not every poll."""
        self._retry_at = now + RETRY_AFTER_S
