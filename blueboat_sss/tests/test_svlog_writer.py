"""``SvlogWriter``: a recording either exists and is named, or says why not.

The writer is the robot-side half of one question — *does a recorded
`.svlog` exist, and where?* — and every case below is a failure mode that
once looked like success:

* the log directory is absent, so the first ``open(..., "ab")`` raises and
  Record sits ON over an empty disk;
* the target becomes unwritable mid-mission and ``write`` swallows the
  ``OSError``;
* a name collision deletes the existing file, which is primary field data
  (``CLAUDE.md`` NON-NEGOTIABLE #6).

``svlog_helper`` is ROS-free by contract (it ships into the GCS as the
NC #7 verbatim copy), so these run on a laptop with no ROS, no boat and no
display. ``conftest.py`` puts ``src/_custom_libraries`` on ``sys.path``.

Every path here is under ``tmp_path``. The field corpus is never opened:
this module *writes*, and NC #6 puts recorded logs out of its reach.
"""

from __future__ import annotations

import stat
from datetime import datetime
from pathlib import Path
from typing import List

import pytest

import svlog_helper
from svlog_helper import SvlogWriter


def _writer(log_dir: Path, errors: List[str], **kw) -> SvlogWriter:
    return SvlogWriter(log_dir=log_dir,
                       metadata_provider=lambda: b"METADATA",
                       error_reporter=errors.append, **kw)


def _stamp() -> str:
    """The writer's own file-naming stamp, to build a collision with."""
    return datetime.now().strftime("%Y-%m-%d-%H-%M-%S")


# --- the recording exists ---------------------------------------------------

def test_start_creates_a_missing_log_directory(tmp_path):
    """A missing directory is created, not a silent no-op.

    ``../../../../data/SSS_data`` is resolved against the launch working
    directory, so "the directory is not there" is an ordinary field
    condition, not an exotic one.
    """
    errors: List[str] = []
    log_dir = tmp_path / "data" / "SSS_data"
    w = _writer(log_dir, errors)

    path = w.start()

    assert path is not None, f"start() failed on a creatable directory: {errors}"
    assert log_dir.is_dir()
    assert path.is_file() and path.suffix == ".svlog"
    assert path.read_bytes() == b"METADATA", "session header was not written"
    assert w.active and not errors


def test_the_file_grows_as_packets_arrive(tmp_path):
    errors: List[str] = []
    w = _writer(tmp_path / "logs", errors)
    path = w.start()
    header = path.stat().st_size

    w.write(b"AAAA")
    w.write(b"BBBBBB")

    assert path.stat().st_size == header + 10
    assert w.active and not errors


# --- failure is loud --------------------------------------------------------

def test_start_reports_and_returns_none_when_the_directory_is_unusable(tmp_path):
    """The silent-success case: Record ON, nothing written, nothing said."""
    errors: List[str] = []
    parent = tmp_path / "readonly"
    parent.mkdir()
    parent.chmod(stat.S_IRUSR | stat.S_IXUSR)          # no write permission
    try:
        w = _writer(parent / "SSS_data", errors)

        assert w.start() is None, "start() claimed success on an unusable path"
        assert not w.active
        assert len(errors) == 1 and "recording NOT started" in errors[0]
        assert str(parent) in errors[0], "the message must name the path"
    finally:
        parent.chmod(stat.S_IRWXU)


def test_write_failure_stops_recording_loudly_and_once(tmp_path):
    """``write`` used to swallow ``OSError`` and set ``_active = False``."""
    errors: List[str] = []
    w = _writer(tmp_path / "logs", errors)
    path = w.start()
    path.chmod(stat.S_IRUSR)                            # unwritable target
    try:
        w.write(b"AAAA")

        assert not w.active, "recording must not look active after a failure"
        assert w.current_path is None
        assert len(errors) == 1 and "recording STOPPED" in errors[0]
        assert str(path) in errors[0]

        # Once per failure, not once per ping: the sonar threads call write()
        # thousands of times a minute and must not flood /rosout.
        for _ in range(50):
            w.write(b"AAAA")
        assert len(errors) == 1
    finally:
        path.chmod(stat.S_IRUSR | stat.S_IWUSR)


def test_without_a_reporter_the_failure_goes_to_stderr(tmp_path, capsys):
    """The default path still speaks — ``svlog_helper`` has no logger of its own."""
    parent = tmp_path / "readonly"
    parent.mkdir()
    parent.chmod(stat.S_IRUSR | stat.S_IXUSR)
    try:
        w = SvlogWriter(log_dir=parent / "SSS_data",
                        metadata_provider=lambda: b"METADATA")
        assert w.start() is None
        assert "recording NOT started" in capsys.readouterr().err
    finally:
        parent.chmod(stat.S_IRWXU)


# --- an existing recording is never destroyed (NC #6) -----------------------

def test_a_name_collision_never_touches_the_existing_file(tmp_path):
    """``_roll_unlocked`` used to ``path.unlink()`` the collision.

    Two recordings inside one second is the everyday case; the 500 MB roll
    inside ``write`` is the dangerous one, because there the file about to
    be unlinked is the one just filled.
    """
    errors: List[str] = []
    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    victim = log_dir / f"{_stamp()}.svlog"
    original = b"PRIMARY FIELD DATA" * 64
    victim.write_bytes(original)

    path = _writer(log_dir, errors).start()

    assert victim.read_bytes() == original, "NC #6: an existing .svlog was destroyed"
    assert path != victim
    assert path.stem.startswith(victim.stem) and path.stem.endswith("-001")


def test_repeated_collisions_keep_counting(tmp_path):
    errors: List[str] = []
    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    stamp = _stamp()
    (log_dir / f"{stamp}.svlog").write_bytes(b"first")

    names = [_writer(log_dir, errors).start().name for _ in range(3)]

    assert names == [f"{stamp}-{n:03d}.svlog" for n in (1, 2, 3)], names
    assert (log_dir / f"{stamp}.svlog").read_bytes() == b"first"
    assert not errors


def test_the_size_roll_does_not_delete_the_file_it_just_filled(tmp_path):
    """The roll inside ``write``, at the same-second boundary."""
    errors: List[str] = []
    w = _writer(tmp_path / "logs", errors, max_size_bytes=8)
    first = w.start()

    w.write(b"0123456789")          # takes it past max_size_bytes
    w.write(b"rolled")              # this call rolls
    second = w.current_path

    assert second != first, "no roll happened; widen the test, not the writer"
    assert first.is_file(), "NC #6: the rolled-from file was deleted"
    assert first.read_bytes() == b"METADATA0123456789"
    assert second.read_bytes() == b"METADATArolled"
    assert not errors


# --- the shipped copy is the same writer (NC #7) ----------------------------

def test_the_gcs_copy_is_the_same_module(tmp_path):
    """Guards against the two halves of NC #7 drifting behaviourally.

    ``test_sweeps.py`` pins the bytes; this pins that the file the GCS ships
    is the one these tests exercised.
    """
    gcs_copy = Path(svlog_helper.__file__).resolve()
    assert gcs_copy.parent.name == "_custom_libraries", (
        f"tests imported svlog_helper from {gcs_copy}, not the robot-side "
        "source; conftest's sys.path order changed")


@pytest.mark.parametrize("packet_id", [svlog_helper.JSON_WRAPPER_ID,
                                       svlog_helper.OS_MONO_PROFILE_ID])
def test_written_bytes_stay_a_readable_packet_stream(tmp_path, packet_id):
    """What lands on disk must still walk as framed packets."""
    errors: List[str] = []
    w = _writer(tmp_path / "logs", errors)
    w.start()
    packet = svlog_helper.frame_packet(packet_id, b"\x01\x02\x03", src=1, dst=0)
    w.write(packet)

    data = w.current_path.read_bytes()
    assert data.endswith(packet)
    assert data.startswith(b"METADATA")
