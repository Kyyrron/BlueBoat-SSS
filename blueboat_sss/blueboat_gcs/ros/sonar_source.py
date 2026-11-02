"""Which sonar the processing pipeline serves: the real pair or a simulator.

Everything runs on the operator laptop. The toolbar's **Sonar** selector
chooses what ``/side_scan_sonar`` will be:

* **Simulation** — the pipeline launch starts ``sss_processor_node`` only;
  ``/side_scan_sonar`` is the simulator's ``sss_sim_node``, started with the
  Gazebo world (by the Mission Control Station, or by hand). **A bag replay**
  (MCS "From a ros2bag", BlueBoat-Control ``bag_replay.py``) starts no sonar
  by design (owner, 2026-10-27): since 2026-11-02 START starts the simulator
  itself over the replay's world folder (:func:`find_replay`,
  :func:`replay_simulator_plan`), so the recorded real motion images the
  simulated seabed — the documented manual ``sss_sim_launch.py
  with_mavros_shim:=false auto_ping:=false``, done by the station.
* **Real** — the same launch with ``with_acquisition:=True`` also starts
  ``sss_node.py``, which drives the two Omniscan 450 SS units over the
  BlueBoat Base Station WiFi.

Guard: two nodes named ``/side_scan_sonar`` must never run at once. Before a
Real launch the process table is read — immediate, where a ROS graph query
at application start would race discovery — and a simulator, or a driver
this application did not start, makes the launch fall back to the processor
alone, with the reason on the console. Every process the pipeline launches
carries :data:`OWNER_ENV` in its environment, which is how a driver orphaned
by an earlier unclean stop is told apart from one an operator started in a
terminal: the leftover sweep kills only the former.

The functions below are pure except :func:`scan_processes` (reads ``/proc``)
and the two :class:`QSettings` helpers; none imports ROS.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

SIMULATION = "simulation"
REAL = "real"
SOURCES = (SIMULATION, REAL)
LABELS = {SIMULATION: "Simulation", REAL: "Real sonars"}

#: Set in the environment of every process the pipeline launches.
OWNER_ENV = "BLUEBOAT_GCS_PIPELINE"
#: Script names of the two nodes that can be /side_scan_sonar.
DRIVER_PATTERN = "sss_node.py"
SIMULATOR_PATTERN = "sss_sim_node"
#: BlueBoat-Control's bag-replay node (MCS "From a ros2bag").
REPLAY_PATTERN = "bag_replay.py"

_SETTINGS_KEY = "pipeline/sonar_source"


def normalise(source: object) -> str:
    """A known source name; anything else is Simulation (the safe choice:
    it starts no driver)."""
    s = str(source or "").strip().lower()
    return s if s in SOURCES else SIMULATION


def launch_command(base: Sequence[str], source: str,
                   real_args: Sequence[str]) -> List[str]:
    """The pipeline command for ``source``: ``base`` plus ``real_args`` for
    the real sonars."""
    cmd = list(base)
    if normalise(source) == REAL:
        cmd += [a for a in real_args if a not in cmd]
    return cmd


@dataclass(frozen=True)
class SonarProcess:
    """A running process that is, or will be, ``/side_scan_sonar``."""

    pid: int
    kind: str          # "driver" | "simulator"
    owned: bool        # launched by this application (OWNER_ENV set)


def classify(pid: int, argv: Sequence[str], environ: Iterable[str]
             ) -> Optional[SonarProcess]:
    """A :class:`SonarProcess` when ``argv`` runs a sonar node, else None.

    A node is a Python interpreter running the script (``ros2 launch`` /
    ``ros2 run`` both look like that); an editor with the file open is not.
    ``environ`` is the process's ``KEY=value`` strings."""
    if not argv or not os.path.basename(argv[0]).startswith("python"):
        return None
    names = {os.path.basename(a) for a in argv[1:]}
    if SIMULATOR_PATTERN in names or SIMULATOR_PATTERN + ".py" in names:
        kind = "simulator"
    elif DRIVER_PATTERN in names:
        kind = "driver"
    else:
        return None
    owned = any(e.startswith(OWNER_ENV + "=") for e in environ)
    return SonarProcess(pid=pid, kind=kind, owned=owned)


def driver_conflict(source: str, processes: Sequence[SonarProcess]
                    ) -> Optional[str]:
    """Why a Real launch must not start the driver, or None.

    Only a process this application did not start counts: an owned driver
    is a leftover the sweep removes before every launch."""
    if normalise(source) != REAL:
        return None
    for p in processes:
        if p.kind == "simulator":
            return (f"sss_sim_node (pid {p.pid}) is already /side_scan_sonar "
                    "— a simulated run is up. Starting the processor only; "
                    "select Sonar: Simulation, or stop the simulator first.")
        if not p.owned:
            return (f"an sss_node.py started outside this application "
                    f"(pid {p.pid}) is already /side_scan_sonar. Starting "
                    "the processor only.")
    return None


def world_dir_from_argv(argv: Sequence[str]) -> Optional[str]:
    """The simulator world folder a Gazebo command line runs, or None.

    ``gz sim <folder>/world.sdf …`` (Replay_launch, full_mission_launch); a
    folder counts only when it holds the acoustic scene the simulator renders
    (``scene.npz`` + ``scene_manifest.yaml``), so BlueBoat-Control's stock
    worlds never qualify."""
    # Measured 2026-11-02: Gazebo runs as `/bin/sh -c "ruby …/gz sim
    # <folder>/world.sdf -r -s …"` and the ruby process rewrites its own
    # command line to the single string `gz sim <folder>/world.sdf -r -s`, so
    # the path sits INSIDE one argv element: every element is split.
    for a in (t for arg in argv for t in arg.split()):
        if os.path.basename(a) != "world.sdf":
            continue
        d = Path(a).expanduser().parent
        if (d / "scene.npz").is_file() and (d / "scene_manifest.yaml").is_file():
            return str(d)
    return None


def find_replay(proc_root: Path = Path("/proc")
                ) -> Tuple[Optional[int], Optional[str]]:
    """``(pid of a running bag_replay.py, world folder Gazebo runs)`` from the
    process table; either may be None. Immediate, like :func:`scan_processes`
    (a ROS graph query would race discovery)."""
    replay: Optional[int] = None
    world: Optional[str] = None
    try:
        entries = [e for e in os.listdir(proc_root) if e.isdigit()]
    except OSError:
        return None, None
    for e in entries:
        argv = _read_cmdline(proc_root, int(e))
        if not argv:
            continue
        if (os.path.basename(argv[0]).startswith("python")
                and any(os.path.basename(a) == REPLAY_PATTERN for a in argv[1:])):
            replay = int(e)
        w = world_dir_from_argv(argv)
        if w is not None:
            world = w
    return replay, world


def replay_simulator_plan(source: str, processes: Sequence[SonarProcess],
                          replay_pid: Optional[int], world_dir: Optional[str],
                          enabled: bool = True) -> Tuple[Optional[str], str]:
    """Whether START must start the simulator for a bag replay.

    Returns ``(world folder to start sss_sim_node over, or None, console
    message or "")``. Only for **Simulation**, only while a bag replay runs,
    and never beside an existing ``/side_scan_sonar`` (a simulator — ours from
    an earlier START included — or a driver)."""
    if not enabled or normalise(source) != SIMULATION or replay_pid is None:
        return None, ""
    for p in processes:
        if p.kind == "simulator":
            return None, ""
        if p.kind == "driver":
            return None, (f"Bag replay: sss_node.py (pid {p.pid}) is already "
                          "/side_scan_sonar — no simulator started.")
    if world_dir is None:
        return None, ("Bag replay without a simulator world folder (the stock "
                      "Gazebo world has no acoustic scene): there is no seabed "
                      "to image. Launch the replay on a world folder (MCS world "
                      "dialog) to record a simulated sonar.")
    return world_dir, ("Sonar: Simulation — a bag replay is running on "
                       f"{world_dir} with no sonar: starting sss_sim_node over "
                       "that world (the replayed real pose, no MAVROS shim; "
                       "this START pings it).")


def simulator_command(launch: Sequence[str], world_dir: str) -> List[str]:
    """The simulator launch for ``world_dir`` (``launch`` = the configured
    ``replay_simulator_launch``)."""
    return list(launch) + [f"world_dir:={world_dir}"]


def no_subscriber_advice(source: str) -> str:
    """What START says when nothing subscribes to ``ping/enable``."""
    if normalise(source) == REAL:
        return ("START: no /side_scan_sonar node subscribes to ping/enable — "
                "the driver did not come up (see the console: it reaches the "
                "Omniscan units at 192.168.2.92 / .93 through the BlueBoat "
                "Base Station WiFi). Enable sent anyway.")
    return ("START: no /side_scan_sonar node subscribes to ping/enable — in "
            "Simulation the simulator provides it: is the Gazebo world "
            "running (a bag replay needs a world folder; the stock world has "
            "no sonar)? For the real sonars select Sonar: Real. Enable sent "
            "anyway.")


def owned_pids(pids: Iterable[int], proc_root: Path = Path("/proc")
               ) -> List[int]:
    """The subset of ``pids`` this application launched."""
    return [pid for pid in pids
            if any(e.startswith(OWNER_ENV + "=")
                   for e in _read_environ(proc_root, pid))]


def scan_processes(proc_root: Path = Path("/proc")) -> List[SonarProcess]:
    """Every running sonar node, from the process table (Linux)."""
    out: List[SonarProcess] = []
    try:
        entries = [e for e in os.listdir(proc_root) if e.isdigit()]
    except OSError:
        return out
    for e in entries:
        pid = int(e)
        argv = _read_cmdline(proc_root, pid)
        if not argv:
            continue
        p = classify(pid, argv, _read_environ(proc_root, pid))
        if p is not None:
            out.append(p)
    return out


def _read_cmdline(proc_root: Path, pid: int) -> Tuple[str, ...]:
    try:
        raw = (proc_root / str(pid) / "cmdline").read_bytes()
    except OSError:
        return ()
    return tuple(s.decode("utf-8", "replace") for s in raw.split(b"\0") if s)


def _read_environ(proc_root: Path, pid: int) -> Tuple[str, ...]:
    try:
        raw = (proc_root / str(pid) / "environ").read_bytes()
    except OSError:
        return ()
    return tuple(s.decode("utf-8", "replace") for s in raw.split(b"\0") if s)


# ---- the operator's last choice (QSettings, per user) --------------------------
def stored_source(default: str) -> str:
    """The source the operator last selected, else ``default``."""
    try:
        from PySide6.QtCore import QSettings
        value = QSettings("BlueBoat", "BlueBoat GCS").value(_SETTINGS_KEY,
                                                             default)
    except Exception:              # pragma: no cover - no Qt / no settings
        value = default
    return normalise(value)


def store_source(source: str) -> None:
    """Remember ``source`` for the next application start."""
    try:
        from PySide6.QtCore import QSettings
        QSettings("BlueBoat", "BlueBoat GCS").setValue(_SETTINGS_KEY,
                                                       normalise(source))
    except Exception:              # pragma: no cover
        pass
