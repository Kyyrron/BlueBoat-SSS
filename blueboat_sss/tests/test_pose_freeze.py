"""FrozenPoseDetector — the generalized frozen-ping-pose policy.

Field history pinned here: the first detector only recognized a freeze
at the exact origin, disengaged instantly on any non-origin constant,
and kept its engagement across acquisition STARTs. The processor's
nearest-stamp latch can pin poses at *any* stale constant, so the rule
is now "ping pose constant while GCS telemetry moved", with release
hysteresis and an explicit reset.

Pure python — no ROS, no Qt.
"""

from __future__ import annotations

from blueboat_gcs.utils.pose_alignment import (FROZEN_AFTER_PINGS,
                                               RELEASE_AFTER_PINGS,
                                               FrozenPoseDetector)


def _feed(det, n, ping, robot):
    """Feed the same ping pose n times while the robot walks east at
    1 m per ping (well past the moved_min threshold immediately)."""
    out = False
    for k in range(n):
        out = det.update(ping[0], ping[1], robot[0] + 1.0 * k, robot[1])
    return out


#: Enough updates for a non-origin freeze: one latches the reference,
#: the displacement gate opens two pings later, then `after` counts.
_ENGAGE_FEED = FROZEN_AFTER_PINGS + 5


def _engage(det, ping=(7.5, -3.2), robot=(40.0, 12.0)):
    assert _feed(det, _ENGAGE_FEED, ping, robot) is True
    return det


def test_origin_freeze_still_engages():
    """The classic pathology: pings at (0,0), boat out on the line."""
    det = FrozenPoseDetector()
    engaged = False
    for _ in range(FROZEN_AFTER_PINGS + 1):
        engaged = det.update(0.0, 0.0, 44.0, 10.0)
    assert engaged, "the origin freeze no longer engages"


def test_non_origin_constant_freeze_engages():
    """The second-session variant: poses pinned at an arbitrary stale
    constant. The old origin-only test missed this entirely."""
    det = FrozenPoseDetector()
    assert _feed(det, _ENGAGE_FEED, (7.5, -3.2), (40.0, 12.0))


def test_stationary_boat_never_engages():
    """Ping pose constant AND robot constant = the boat is parked, not
    a pathology."""
    det = FrozenPoseDetector()
    engaged = False
    for _ in range(5 * FROZEN_AFTER_PINGS):
        engaged = det.update(7.5, -3.2, 7.5, -3.2)
    assert not engaged


def test_release_hysteresis():
    """A burst of moving pings shorter than the release threshold keeps
    re-stamping on; a full burst releases it."""
    det = _engage(FrozenPoseDetector())
    for k in range(RELEASE_AFTER_PINGS - 1):
        det.update(10.0 + k, 5.0 + k, 50.0, 12.0)
    assert det.engaged, "released too eagerly (flapping)"
    det.update(20.0, 15.0, 50.0, 12.0)   # the RELEASE_AFTER-th moving ping
    assert not det.engaged


def test_freeze_at_a_new_constant_does_not_release():
    """A latch that jumps from one stale pose to another is still the
    same defect; the single jump must not disengage re-stamping."""
    det = _engage(FrozenPoseDetector())
    _feed(det, _ENGAGE_FEED, (99.0, 99.0), (60.0, 12.0))
    assert det.engaged


def test_reset_clears_engagement():
    """START presses reset() — session 1's engagement must not re-stamp
    session 2's healthy pings."""
    det = _engage(FrozenPoseDetector())
    det.reset()
    assert not det.engaged
    # And one healthy ping does not instantly re-engage.
    assert det.update(1.0, 2.0, 1.0, 2.0) is False


def test_no_telemetry_is_inconclusive():
    """With no RobotState at all there is no evidence of motion: the
    detector must neither engage nor crash."""
    det = FrozenPoseDetector()
    for _ in range(3 * FROZEN_AFTER_PINGS):
        assert det.update(0.0, 0.0, None, None) is False
