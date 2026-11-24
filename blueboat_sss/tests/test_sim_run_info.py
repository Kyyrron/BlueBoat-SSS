"""The simulator's run record for a session's metadata (core/sim_run_info.py)."""

from __future__ import annotations

import json

from blueboat_gcs.core import sim_run_info as S


def setup_function(_):
    S.clear()
    S.set_alive_probe(None)


def test_nothing_fed_means_no_record():
    assert S.latest() is None                      # a real-robot graph


def test_the_last_good_record_is_kept_and_copied():
    assert S.feed(json.dumps({"run_id": "a", "renderer": "raycast"}))
    assert not S.feed("not json")                  # ignored, the record stays
    assert not S.feed(json.dumps([1, 2]))          # not an object
    doc = S.latest()
    assert doc == {"run_id": "a", "renderer": "raycast"}
    doc["run_id"] = "changed"                      # a copy: the holder is untouched
    assert S.latest()["run_id"] == "a"


def test_a_record_whose_simulator_is_gone_is_not_reported():
    S.feed(json.dumps({"run_id": "a"}))
    alive = {"v": True}
    S.set_alive_probe(lambda: alive["v"])
    assert S.latest() == {"run_id": "a"}
    alive["v"] = False                             # the launch was stopped
    assert S.latest() is None
    alive["v"] = True
    assert S.latest() == {"run_id": "a"}


def test_the_topic_is_the_simulated_sonars_latched_one():
    assert S.TOPIC == "/side_scan_sonar/ground_truth/run_info"
