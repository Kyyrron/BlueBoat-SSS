"""NavSatFix acceptance gate (``utils/geodesy.navsat_fix_ok``).

The MCS-sim regression this pins: since ROS 2 Iron,
``sensor_msgs/NavSatStatus`` defaults ``status`` to **-2**
(STATUS_UNKNOWN), and the MCS bridge's simulated GPS — the ONLY GPS
publisher in a Gazebo run of a GPS-anchored mission — never sets the
field. The old listener check ``status < 0`` therefore discarded every
one of those fixes: MCS (which never reads ``status``) anchored its own
map while the GCS reported "no fix" forever. Only an explicit NO_FIX
(-1), non-finite coordinates, or the documented ``(0, 0)`` no-fix
sentinel may be rejected.

Pure function, no ROS: runs everywhere the suite runs.
"""

from __future__ import annotations

import math

import pytest

from blueboat_gcs.utils.geodesy import (NAVSAT_STATUS_NO_FIX,
                                        NAVSAT_STATUS_UNKNOWN, navsat_fix_ok)

LAT, LON = 43.6961, 7.3080


def test_status_unknown_accepted():
    """-2 is the message default — the value the MCS sim GPS sends."""
    ok, reason = navsat_fix_ok(NAVSAT_STATUS_UNKNOWN, LAT, LON)
    assert ok and reason == ""


@pytest.mark.parametrize("status", [0, 1, 2])
def test_augmented_and_plain_fixes_accepted(status):
    ok, _ = navsat_fix_ok(status, LAT, LON)
    assert ok


def test_status_no_fix_rejected():
    ok, reason = navsat_fix_ok(NAVSAT_STATUS_NO_FIX, LAT, LON)
    assert not ok and "NO_FIX" in reason


@pytest.mark.parametrize("lat,lon", [
    (math.nan, LON), (LAT, math.nan), (math.nan, math.nan),
    (math.inf, LON), (LAT, -math.inf),
])
def test_non_finite_coordinates_rejected(lat, lon):
    ok, reason = navsat_fix_ok(0, lat, lon)
    assert not ok and "non-finite" in reason


def test_zero_zero_sentinel_rejected():
    """The cross-module contract: lat==0 and lon==0 means no fix."""
    ok, reason = navsat_fix_ok(0, 0.0, 0.0)
    assert not ok and "sentinel" in reason


@pytest.mark.parametrize("lat,lon", [(0.0, LON), (LAT, 0.0)])
def test_single_zero_axis_is_a_real_position(lat, lon):
    """Only the exact (0, 0) pair is the sentinel — a point on the
    equator or the prime meridian is a legitimate fix."""
    ok, _ = navsat_fix_ok(0, lat, lon)
    assert ok
