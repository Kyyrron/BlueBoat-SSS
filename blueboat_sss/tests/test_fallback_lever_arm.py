"""Bug fix 2026-10-16: the live profile-miss fallback and the lever arm.

When a row's raw ``OmniscanProfile`` misses its ``profile_wait_ms``, the
live listener re-projects that side from the processed row
(``core/live_native.attach_native`` -> ``models/sonar.side_bins_from_ground``).
Since ``994a74b`` (2026-10-13) the processor projects
``|y| = 0.285 + sqrt(s² − h²)`` with ``h`` = its altitude =
``water_depth − 0.15``; the fallback still inverted with ``h = water_depth``
and no lever arm. This file shows, **on the same input** (the processor's
own ``project_side``), the row position before and after the fix.
"""

from __future__ import annotations

import numpy as np
import pytest

import sss_helper
from blueboat_gcs.core.live_native import ProcessedFields, ProfileCache, attach_native
from blueboat_gcs.core.svlog import (TRANSDUCER_SUBMERSION_M,
                                     TRANSDUCER_Y_OFFSET_PORT_M,
                                     TRANSDUCER_Y_OFFSET_STBD_M)
from blueboat_gcs.models.sonar import native_from_profile, side_bins_from_ground

N, LENGTH_MM, ALTITUDE = 1200, 20000, 1.2


def processed_row(seed=1):
    """One two-sided row exactly as sss_processor_node emits it."""
    rng = np.random.default_rng(seed)
    out = {}
    for side, sign, lever in ((0, +1.0, TRANSDUCER_Y_OFFSET_PORT_M),
                              (1, -1.0, TRANSDUCER_Y_OFFSET_STBD_M)):
        db = rng.uniform(0.0, 60.0, N).astype(np.float32)
        y, v = sss_helper.project_side(db.tolist(), 0, LENGTH_MM, N, ALTITUDE,
                                       lever, sign)
        out[side] = (db, np.asarray(y), np.asarray(v, np.float32))
    f = ProcessedFields(port_pn=7, stbd_pn=8,
                        water_depth=ALTITUDE + TRANSDUCER_SUBMERSION_M,
                        port_y=out[0][1], port_db=out[0][2],
                        stbd_y=out[1][1], stbd_db=out[1][2])
    return f, out


def test_row_position_before_and_after_the_fix():
    f, sides = processed_row()
    _, pitch = native_from_profile(0, LENGTH_MM, N)
    first_kept = int(np.argmax(np.arange(N) * pitch > ALTITUDE))     # device truth
    assert first_kept == 72

    # BEFORE (the call attach_native made until 2026-10-16): water_depth as
    # the altitude, no lever arm. The row lands 11 bins (0.18 m) too far out
    # and its pitch is off, so the far end sits at 20.16 m instead of 20.0.
    b0, bv, bd = side_bins_from_ground(np.abs(f.port_y), f.port_db, f.water_depth)
    assert b0 == 83 and b0 - first_kept == 11
    assert b0 * bd == pytest.approx(1.383, abs=1e-3)
    assert (b0 + bv.size - 1) * bd == pytest.approx(20.163, abs=1e-3)
    assert bd != pytest.approx(pitch, rel=1e-6)

    # AFTER: attach_native inverts the processor's own geometry.
    out, hits = attach_native(f, ProfileCache())                    # both missed
    assert hits == (False, False)
    for side, key in ((0, "port"), (1, "stbd")):
        db, _y, v = sides[side]
        assert out[f"{key}_bin0"] == first_kept
        np.testing.assert_array_equal(out[f"{key}_db"], db[first_kept:])
    assert out["bin_size_m"] == pytest.approx(pitch, rel=1e-12)
    assert out["bottom_slant_m"] == pytest.approx(ALTITUDE)


def test_the_defaults_are_the_old_inverse_bit_for_bit():
    """Lever 0 (the default) is the pre-994a74b inverse, unchanged -- what
    native_bins' reconstruction from y_local still uses."""
    rng = np.random.default_rng(3)
    y = np.sort(rng.uniform(0.5, 20.0, 300))
    v = rng.uniform(0, 60, 300).astype(np.float32)
    order = np.argsort(np.abs(y))
    s_old = np.hypot(np.abs(y[order]), 1.7)
    d = np.diff(s_old)
    delta = float(np.median(d[d > 0]))
    b0, vals, size = side_bins_from_ground(y, v, 1.7)
    assert size == delta and b0 == int(np.rint(s_old / delta)[0])
    np.testing.assert_array_equal(vals, v[order])


def test_a_hit_is_untouched_by_the_fix():
    """The verbatim-profile path never used the inverse: same as before."""
    from test_live_native import P, fields
    c = ProfileCache()
    port, stbd = P(0, 10), P(1, 12)
    c.put(0, 10, port.prof)
    c.put(1, 12, stbd.prof)
    out, hits = attach_native(fields(port, stbd), c)
    assert hits == (True, True)
    np.testing.assert_array_equal(out["port_db"], port.prof.db)
