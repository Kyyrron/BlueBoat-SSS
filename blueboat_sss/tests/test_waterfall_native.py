"""Native slant-bin waterfall: loss visibility and display equalization.

Companions to test_waterfall_buffer / test_waterfall_pick for the two
behaviours added with the native-bin rewrite:

* ``SonarPing.gap_before`` (a jump in the device's own ping counter =
  genuine acquisition/QoS loss) inserts blank rows, capped, so real
  dropouts are visible without a huge gap scrolling the image away;
* seabed-referenced EGN (``mosaic.nadir_contrast``, on by default) is
  render-time only: the buffer keeps raw dB, and the equalized auto
  contrast window is much tighter than the plain raw window when the
  ping carries a strong range falloff (the pre-TVG device stream),
  because the per-column seabed reference removes that falloff.
"""

from __future__ import annotations

import numpy as np
import pytest

from blueboat_gcs.core.waterfall_service import _MAX_GAP_ROWS, WaterfallService
from blueboat_gcs.models.sonar import SonarPing

N_BINS = 300
PITCH = 0.05
DEPTH = 2.0


def make_ping(i: int, gap_before: int = 0,
              falloff: bool = False) -> SonarPing:
    slant = np.arange(N_BINS) * PITCH
    keep = slant > DEPTH
    ground = np.sqrt(slant[keep] ** 2 - DEPTH ** 2)
    y = np.concatenate([ground, -ground])
    if falloff:
        # The physical pre-TVG falloff of the field logs (~65 dB/decade of
        # slant range: two-way spreading + absorption + grazing angle).
        r = slant[keep]
        prof = (60.0 - 40.0 * np.log10(r) - 0.2 * r
                - 25.0 * np.log10(r / DEPTH)).astype(np.float32)
    else:
        prof = np.full(int(keep.sum()), 40.0, np.float32)
    v = np.concatenate([prof, prof])
    return SonarPing(t=float(i), robot_x=float(i), robot_y=0.0, yaw=0.0,
                     water_depth=DEPTH, y_local=y, intensity_db=v,
                     gap_before=gap_before)


def test_gap_before_inserts_blank_rows(qapp, tmp_config):
    svc = WaterfallService(tmp_config)
    svc.on_sonar_ping(make_ping(0))
    svc.on_sonar_ping(make_ping(1, gap_before=3))
    buf = svc.chronological()
    assert buf.shape[0] == 5                     # ping, 3 blanks, ping
    assert np.isfinite(buf[0]).any() and np.isfinite(buf[4]).any()
    for r in (1, 2, 3):
        assert not np.isfinite(buf[r]).any(), f"row {r} should be blank"
        assert svc.row_meta(r) is None


def test_gap_rows_are_capped(qapp, tmp_config):
    svc = WaterfallService(tmp_config)
    svc.on_sonar_ping(make_ping(0))
    svc.on_sonar_ping(make_ping(1, gap_before=500))
    assert svc.chronological().shape[0] == 2 + _MAX_GAP_ROWS


def test_range_falloff_renders_uniform(qapp, tmp_config):
    """The physical falloff spans >50 dB across the ping. Through the
    display model (nadir_contrast on) near and far seabed render to the
    same grey; the buffered dB is untouched (render-time only)."""
    svc = WaterfallService(tmp_config)           # nadir_contrast on
    for i in range(40):
        svc.on_sonar_ping(make_ping(i, falloff=True))
    raw = svc.chronological().copy()
    svc.model.freeze()
    unit = svc.render_unit(raw, svc._meta_chrono()[:, 5])
    half = svc.columns // 2
    q = half // 3
    fin = np.isfinite(unit)
    near = unit[:, half:half + q][fin[:, half:half + q]]
    far = unit[:, -q:][fin[:, -q:]]
    assert near.size and far.size
    assert abs(near.mean() - far.mean()) < 0.15 * max(near.mean(), far.mean())
    # Render-time only: the buffered dB values are untouched.
    assert np.array_equal(svc.chronological(), raw, equal_nan=True)
    # The raw window (nadir_contrast off) is dominated by the falloff.
    svc._nadir_contrast = False
    lo, hi = svc._global_limits()
    assert hi - lo > 30.0


# ---------------------------------------------------------------------------
# Shared native-bin construction (both paths): per-side pitch, start_mm
# ---------------------------------------------------------------------------
def test_native_from_profile_is_the_processor_grid():
    from blueboat_gcs.models.sonar import native_from_profile
    bin0, pitch = native_from_profile(0, 20000, 600)
    assert bin0 == 0
    assert pitch == pytest.approx(20.0 / 599)       # project_side's grid
    # range_start_mm lands the first sample on its true slant column.
    bin0, pitch2 = native_from_profile(1500, 20000, 600)
    assert pitch2 == pitch
    assert bin0 == round(1.5 / pitch)
    assert native_from_profile(0, 0, 600) == (0, 0.0)


def test_two_pitch_row_is_two_sided():
    """Two units at different ranges: the starboard side is NOT dropped;
    it paints by pixel stretch on the finer layout pitch."""
    from blueboat_gcs.models.sonar import build_slant_row, native_bins
    port = np.full(100, 10.0, np.float32)
    stbd = np.full(50, 20.0, np.float32)
    ping = SonarPing(t=0.0, robot_x=0.0, robot_y=0.0, yaw=0.0,
                     water_depth=0.0, y_local=np.zeros(0),
                     intensity_db=np.zeros(0, np.float32),
                     bin_size_m=0.05, port_bin0=0, port_db=port,
                     stbd_bin0=0, stbd_db=stbd, stbd_bin_size_m=0.10)
    nb = native_bins(ping)
    assert nb.bin_size_m == pytest.approx(0.05)     # finest pitch = layout
    assert nb.extent_bins == 100                    # both reach 5 m
    row = build_slant_row(nb, nb.bin_size_m, nb.extent_bins)
    half = nb.extent_bins
    assert np.isfinite(row).all()
    assert (row[:half] == 10.0).all()
    assert (row[half:] == 20.0).all()               # each stbd bin -> 2 columns


def test_start_mm_offsets_bin0_on_replay(tmp_path):
    """A profile recorded with range_start_mm > 0 leaves its first columns
    empty (the device did not sample them) instead of shifting the swath."""
    from blueboat_gcs.core.svlog import load_svlog
    from blueboat_gcs.models.sonar import build_slant_row, native_bins
    from test_svlog_replay import build_log
    m = load_svlog(build_log(tmp_path / "start.svlog", 5, start_mm=1000),
                   depth_mode="off")
    pings = [e[2] for e in m.events if e[0] == "ping"]
    assert pings, "fixture produced no pings (pose missing?)"
    p = pings[0]
    nb = native_bins(p)
    assert p.port_bin0 == round(1.0 / p.bin_size_m) == p.stbd_bin0
    row = build_slant_row(nb, nb.bin_size_m, nb.extent_bins)
    half = nb.extent_bins
    assert np.isnan(row[half:half + p.stbd_bin0]).all()      # unsampled
    assert np.isfinite(row[half + p.stbd_bin0:]).all()
