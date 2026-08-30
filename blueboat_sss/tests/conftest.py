"""Shared fixtures for the headless GCS regression suite.

Everything here is laptop-only: no ROS, no boat, no display. The suite
formalises the ad-hoc ``QT_QPA_PLATFORM=offscreen`` scripts that this
project has driven by hand after every update.

Two invariants this file exists to protect:

* **Tests never touch the real ``data_root``.** ``RecordingManager``
  *moves* ``.svlog`` files into a session folder
  (``core/recording_session.py``), so a test pointed at the field data
  root would destroy primary field record — CLAUDE.md NC #6 / root
  CM-7. ``tmp_config`` redirects ``data_root`` *and* the tile cache into
  the per-test ``tmp_path``.
* **"Imports without ROS" is simulated, never observed.** The Linux
  development machine sources ROS 2 Jazzy globally, so ``rclpy`` and the
  message packages import fine here; a sweep that merely *looked* would
  report every module clean and silently stop testing NC #10. The
  ``no_ros`` fixture blocks them on ``sys.meta_path`` instead.
"""

from __future__ import annotations

import os

# Must precede the first PySide6 import: Qt reads the platform plugin at
# QApplication construction, and any earlier import can pin it.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import importlib
import sys
from pathlib import Path
from typing import Iterator, List

import pytest

# pytest puts tests/ on sys.path (rootdir-relative, no __init__.py here),
# not the package parent, so `import blueboat_gcs` needs this.
PKG_PARENT = Path(__file__).resolve().parents[1]
if str(PKG_PARENT) not in sys.path:
    sys.path.insert(0, str(PKG_PARENT))

# The robot-side helpers are installed flat into lib/blueboat_sss/ and import
# each other with no package prefix, so tests reach them the same way the
# nodes do: by directory, not by package.
HELPERS = PKG_PARENT / "src" / "_custom_libraries"
if str(HELPERS) not in sys.path:
    sys.path.insert(0, str(HELPERS))

GCS_ROOT = PKG_PARENT / "blueboat_gcs"
REPO_ROOT = PKG_PARENT.parent

# ``tools/`` is excluded from both sweeps: it holds the verbatim copies of
# the robot-side converter, which need a sourced ROS 2 environment to
# import (CLAUDE.md NC #7). Their integrity is checked by hash instead.
SWEEP_EXCLUDED_PARTS = ("tools",)

# The ROS distribution modules the GCS may reach for. Blocking exactly
# this set reproduces a ROS-free basestation laptop.
ROS_MODULE_PREFIXES = (
    "rclpy",
    "std_msgs",
    "nav_msgs",
    "sensor_msgs",
    "geometry_msgs",
    "rcl_interfaces",
    "vision_msgs",
    "mavros_msgs",
    "geographic_msgs",
    "blueboat_interfaces",
    "rosbag2_py",
)


def gcs_modules() -> List[str]:
    """Dotted names of every swept GCS module, in stable order."""
    names = []
    for path in sorted(GCS_ROOT.rglob("*.py")):
        rel = path.relative_to(PKG_PARENT)
        if any(part in SWEEP_EXCLUDED_PARTS for part in rel.parts):
            continue
        parts = list(rel.with_suffix("").parts)
        if parts[-1] == "__init__":
            parts.pop()
        names.append(".".join(parts))
    return names


def gcs_files() -> List[Path]:
    """Every swept GCS source file, in stable order."""
    out = []
    for path in sorted(GCS_ROOT.rglob("*.py")):
        rel = path.relative_to(PKG_PARENT)
        if any(part in SWEEP_EXCLUDED_PARTS for part in rel.parts):
            continue
        out.append(path)
    return out


class _BlockedImportFinder:
    """Meta-path finder that makes a set of top-level packages unimportable."""

    def __init__(self, prefixes: tuple) -> None:
        self._prefixes = prefixes

    def find_module(self, fullname, path=None):        # pragma: no cover - py2 API
        return None

    def find_spec(self, fullname, path=None, target=None):
        root = fullname.split(".")[0]
        if root in self._prefixes:
            raise ImportError(f"No module named {fullname!r} (blocked by test harness)")
        return None


@pytest.fixture
def no_ros() -> Iterator[None]:
    """Make the ROS packages unimportable for the duration of one test.

    Snapshots and restores ``sys.modules`` so a blocked import cannot
    leave a half-initialised GCS module cached for later tests.
    """
    finder = _BlockedImportFinder(ROS_MODULE_PREFIXES)
    saved_modules = dict(sys.modules)
    for name in list(sys.modules):
        if name.split(".")[0] in ROS_MODULE_PREFIXES:
            del sys.modules[name]
    sys.meta_path.insert(0, finder)
    try:
        yield
    finally:
        sys.meta_path.remove(finder)
        sys.modules.clear()
        sys.modules.update(saved_modules)


@pytest.fixture
def purge_gcs_modules() -> Iterator[None]:
    """Drop cached ``blueboat_gcs`` modules before and after a test.

    An import sweep must import each module for real, not read a cache
    populated by an earlier test that ran with ROS available.
    """
    def _purge() -> None:
        for name in [n for n in sys.modules if n.split(".")[0] == "blueboat_gcs"]:
            del sys.modules[name]

    _purge()
    try:
        yield
    finally:
        _purge()
        importlib.invalidate_caches()


@pytest.fixture(scope="session")
def qapp():
    """One QApplication for the whole session.

    Qt permits exactly one per process, and tearing one down between
    tests is what makes offscreen Qt suites flaky.
    """
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    yield app
    app.processEvents()


@pytest.fixture
def tmp_config(tmp_path):
    """A real config with every writing path redirected into tmp_path.

    ``data_root`` is the one that matters: recording sessions are created
    under it and ``_adopt_svlogs`` *moves* files into them.
    """
    from blueboat_gcs.config.settings import load_config

    cfg = load_config()                       # config/default.yaml, as shipped
    cfg.data_root = str(tmp_path / "data_root")
    cfg.map.tile_cache_dir = str(tmp_path / "tile_cache")
    return cfg


@pytest.fixture
def no_modal_dialogs(monkeypatch):
    """Neutralise every modal dialog before the GUI is driven.

    A modal dialog blocks an offscreen run forever. Only
    ``gui/replay_window.py`` opens one in the current tree, so the
    START/STOP and recording tests do not strictly need this — it is the
    documented pattern (CLAUDE.md, Headless GUI testing) and the guard
    for any future test that drives the replay window.
    """
    from PySide6.QtWidgets import QFileDialog, QInputDialog, QMessageBox

    for name in ("information", "warning", "critical", "question"):
        monkeypatch.setattr(QMessageBox, name,
                            staticmethod(lambda *a, **k: QMessageBox.StandardButton.Ok))
    monkeypatch.setattr(QInputDialog, "getText",
                        staticmethod(lambda *a, **k: ("test", True)))
    monkeypatch.setattr(QFileDialog, "getOpenFileName",
                        staticmethod(lambda *a, **k: ("", "")))
    monkeypatch.setattr(QFileDialog, "getSaveFileName",
                        staticmethod(lambda *a, **k: ("", "")))
