"""Static regression floor: no GUI, no ROS, no event loop.

These four tests are the cheapest thing that can catch a regression here,
and three of them encode a NON-NEGOTIABLE from CLAUDE.md directly.
"""

from __future__ import annotations

import hashlib
import importlib
import py_compile
import re

import pytest

from conftest import GCS_ROOT, PKG_PARENT, gcs_files, gcs_modules

# Recorded counts. If either moves, update CLAUDE.md in the same commit
# rather than loosening the assertion — the number *is* the regression
# signal.
EXPECTED_FILE_COUNT = 62        # 2026-09-05: +core/display_model.py, +core/live_native.py, +analysis/check_dois.py, -core/contrast.py
EXPECTED_ROS_FREE_IMPORTABLE = 58

# The modules that legitimately cannot import without ROS: exactly the
# four ros/ listeners that take `from rclpy.node import Node` at module
# level. main.py imports them lazily and only outside --sim.
#
# The other four files in ros/ must keep importing ROS-free:
# path_listener and ros_manager guard their ROS imports in try/except,
# pipeline_launcher imports PySide6 rather than rclpy, and __init__ is
# empty. Asserting the *identity* of the failures — not just the count —
# is what protects NC #10 (no rclpy past the ros/ boundary).
EXPECTED_ROS_FREE_FAILURES = {
    "blueboat_gcs.ros.detections_listener",
    "blueboat_gcs.ros.pinger_listener",
    "blueboat_gcs.ros.sonar_listener",
    "blueboat_gcs.ros.telemetry_listener",
}

# NC #7: byte-identical copies. GCS-side path -> robot-side path.
VERBATIM_COPIES = [
    ("blueboat_gcs/tools/svlog_helper.py", "src/_custom_libraries/svlog_helper.py"),
    ("blueboat_gcs/tools/svlog_to_rosbag.py", "custom_scripts/svlog_to_rosbag.py"),
]


def test_compile_sweep():
    """Every swept GCS file compiles."""
    files = gcs_files()
    assert len(files) == EXPECTED_FILE_COUNT, (
        f"swept {len(files)} files, expected {EXPECTED_FILE_COUNT}. "
        "If the package genuinely gained or lost a module, update this "
        "constant and CLAUDE.md together."
    )
    failures = []
    for path in files:
        try:
            py_compile.compile(str(path), doraise=True)
        except py_compile.PyCompileError as exc:
            failures.append(f"{path.relative_to(PKG_PARENT)}: {exc}")
    assert not failures, "compile failures:\n" + "\n".join(failures)


def test_import_sweep_without_ros(no_ros, purge_gcs_modules):
    """49 of 53 modules import with no ROS at all; the 4 that don't are the listeners.

    ``no_ros`` blocks the ROS packages on sys.meta_path. It is not enough
    to observe this machine: ROS 2 Jazzy is sourced globally here, so an
    observational sweep would report 53/53 and quietly stop testing
    anything.
    """
    importable, failed = [], {}
    for name in gcs_modules():
        try:
            importlib.import_module(name)
            importable.append(name)
        except Exception as exc:                # noqa: BLE001 - reporting the class matters
            failed[name] = f"{type(exc).__name__}: {exc}"

    assert set(failed) == EXPECTED_ROS_FREE_FAILURES, (
        "ROS-free import failures changed.\n"
        f"  unexpected: {sorted(set(failed) - EXPECTED_ROS_FREE_FAILURES)}\n"
        f"  now importing: {sorted(EXPECTED_ROS_FREE_FAILURES - set(failed))}\n"
        f"  details: {failed}"
    )
    # Every failure must be the ROS import itself, not an unrelated break.
    for name, err in failed.items():
        assert "ModuleNotFoundError" in err or "ImportError" in err, (name, err)
    assert len(importable) == EXPECTED_ROS_FREE_IMPORTABLE, (
        f"{len(importable)} modules imported ROS-free, "
        f"expected {EXPECTED_ROS_FREE_IMPORTABLE}"
    )


@pytest.mark.parametrize("gcs_rel,robot_rel", VERBATIM_COPIES)
def test_verbatim_tool_copies_are_identical(gcs_rel, robot_rel):
    """NC #7: the tools/ copies are byte-identical to the robot-side files.

    ``tools/`` is excluded from both sweeps because it needs ROS to
    import, so without this test nothing checks it at all. Update only by
    re-copying; never hand-edit either side.
    """
    gcs_path = PKG_PARENT / gcs_rel
    robot_path = PKG_PARENT / robot_rel
    assert gcs_path.is_file(), gcs_path
    assert robot_path.is_file(), robot_path

    gcs_bytes = gcs_path.read_bytes()
    robot_bytes = robot_path.read_bytes()
    assert hashlib.sha256(gcs_bytes).hexdigest() == hashlib.sha256(robot_bytes).hexdigest(), (
        f"{gcs_rel} and {robot_rel} have diverged "
        f"({len(gcs_bytes)} B vs {len(robot_bytes)} B). "
        "Re-copy rather than hand-editing (CLAUDE.md NC #7)."
    )


def test_processor_name_is_load_bearing():
    """NC #8: the orphan sweep matches a literal the launch file must keep.

    The GCS runs `pgrep -f sss_processor_node` to sweep orphans on STOP,
    and pumps the launch tree's stdout into the embedded console, so both
    the executable name and output='screen' are load-bearing across the
    config/launch boundary.
    """
    from blueboat_gcs.config.settings import load_config

    cfg = load_config()
    launch_name = cfg.pipeline.launch_command[-1]
    launch_file = PKG_PARENT / "launch" / launch_name
    assert launch_file.is_file(), (
        f"pipeline.launch_command names {launch_name}, which does not exist"
    )
    source = launch_file.read_text(encoding="utf-8")

    for pattern in cfg.pipeline.leftover_process_patterns:
        assert pattern in source, (
            f"leftover_process_patterns has {pattern!r}, absent from {launch_name}; "
            "the pgrep orphan sweep would no longer match anything"
        )

    node_calls = re.findall(r"sl\.node\((.*?)\)", source, flags=re.S)
    processor_calls = [c for c in node_calls if "sss_processor_node" in c]
    assert processor_calls, "no sss_processor_node node in the launch file"
    for call in processor_calls:
        assert "output='screen'" in call or 'output="screen"' in call, (
            "sss_processor_node lost output='screen'; the embedded console "
            "would go dark (NC #8)"
        )
