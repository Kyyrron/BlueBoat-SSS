"""Console batching: a /rosout storm must not become per-line GUI edits.

Field finding: master_control logs a multi-line INFO at 20 Hz to
/rosout; the old per-line ``append_line`` did ~60 document edits +
repaints per second on the GUI thread and was a main contributor to the
GCS slowdown against the simulator. Lines are now buffered and flushed
in one edit block.
"""

from __future__ import annotations

from blueboat_gcs.gui.log_console import MAX_PENDING, LogConsole


def test_lines_buffer_then_flush_in_order(qapp):
    console = LogConsole()
    for k in range(500):
        console.append_line("rosout", f"line {k}")
    # Nothing hits the document until the flush timer fires.
    assert console._text.blockCount() == 1          # the empty document
    console._flush()
    text = console._text.toPlainText()
    assert "line 0" in text and "line 499" in text
    assert text.index("line 0") < text.index("line 499")
    assert console._counter.text() == "500 lines"
    # Timer stops itself once drained.
    console._flush()
    assert not console._flush_timer.isActive()


def test_storm_is_capped_with_drop_marker(qapp):
    console = LogConsole()
    for k in range(MAX_PENDING + 500):
        console.append_line("rosout", f"line {k}")
    assert len(console._pending) == MAX_PENDING
    console._flush()
    text = console._text.toPlainText()
    assert "500 lines dropped" in text
    assert f"line {MAX_PENDING + 499}" in text       # newest kept
    assert console._counter.text() == f"{MAX_PENDING + 500} lines"


def test_pause_discards_and_clear_resets(qapp):
    console = LogConsole()
    console._set_paused(True)
    console.append_line("rosout", "while paused")
    assert not console._pending
    console._set_paused(False)
    console.append_line("rosout", "after resume")
    console._flush()
    assert "after resume" in console._text.toPlainText()
    console._clear()
    assert console._text.toPlainText() == ""
    assert console._counter.text() == "0 lines"
