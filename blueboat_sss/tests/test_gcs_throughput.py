"""Live-pipeline throughput guards (2026-09-03 GCS slowdown/crash).

With every ping finally arriving as one two-sided row at 20 Hz the GCS
grew slower and died: the mosaic derived its cell size from the *merged*
port+starboard sample spacing (interleaved by any transducer asymmetry ->
3x the cells), its grid grew without a cap before the budget was checked,
and the waterfall re-colormapped its whole buffer on every window move.
These tests pin the fixes on the service/grid level (no GUI).
"""

from __future__ import annotations

import numpy as np
import pytest

from blueboat_gcs.analysis.ping_bench import make_ping
from blueboat_gcs.config.settings import AppConfig
from blueboat_gcs.core.mosaic_service import MosaicService
from blueboat_gcs.core.waterfall_service import WaterfallService, _TILE_ROWS
from blueboat_gcs.mapping.mosaic import MosaicGrid
from blueboat_gcs.mapping.renderer import DisplaySettings, MosaicRenderer

pytestmark = pytest.mark.usefixtures("qapp")


@pytest.fixture(scope="module")
def qapp():
    import os
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def _cfg() -> AppConfig:
    return AppConfig()


def test_cell_size_is_derived_from_one_side_not_the_interleaved_pair():
    svc = _cfg()
    one = MosaicService(svc)
    two = MosaicService(_cfg())
    kw = dict(range_m=15.0, bins=600, depth=4.0, speed=0.5, rate_hz=20.0, turn=False)
    c1 = one._derive_cell_size(make_ping(0, one_sided=True, **kw))
    c2 = two._derive_cell_size(make_ping(0, one_sided=False, **kw))
    assert c1 is not None and c2 is not None
    assert abs(c1 - c2) < 1e-6, (c1, c2)
    assert c1 > svc.mosaic.min_cell_size_m * 1.2      # not clamped


def test_grid_growth_honours_the_budget_before_allocating():
    g = MosaicGrid(cell_size_m=0.02, initial_half_extent_m=30.0)
    g.max_cells = 2_000_000
    x = np.linspace(-30.0, 30.0, 600); y = np.zeros_like(x)
    g.add_samples(x, y, np.full(x.size, -30.0, dtype=np.float32),
                  slant_range=np.abs(x))
    cell_before = g.cell_size_m
    # A growth to +-200 m would need 400 m / 0.02 = 20 000 cells a side
    # (400 M cells); the grid must coarsen first and never allocate that.
    x2 = np.linspace(-200.0, 200.0, 600); y2 = np.linspace(-200.0, 200.0, 600)
    g.add_samples(x2, y2, np.full(x2.size, -30.0, dtype=np.float32),
                  slant_range=np.abs(x2))
    h, w = g.shape
    assert h * w <= g.max_cells, (h, w)
    assert g.cell_size_m > cell_before and g.coarsenings >= 1
    # Data survived the coarsening.
    assert np.isfinite(g.render("closest")).sum() > 0


def test_pose_glitch_growth_is_refused_not_allocated():
    g = MosaicGrid(cell_size_m=0.05, initial_half_extent_m=30.0)
    g.max_cells = 1_000_000
    g.max_extent_m = 1000.0
    shape = g.shape
    x = np.array([0.0, 50_000.0]); y = np.zeros(2)
    g.add_samples(x, y, np.zeros(2, dtype=np.float32))
    assert g.shape == shape and g.refused_growths == 1


def test_coarsen_preserves_means_and_counts():
    g = MosaicGrid(cell_size_m=0.1, initial_half_extent_m=5.0)
    x = np.linspace(-4.0, 4.0, 400); y = np.zeros_like(x)
    v = np.linspace(-50.0, -20.0, 400).astype(np.float32)
    g.add_samples(x, y, v, slant_range=np.abs(x), bilinear=False)
    total_count = int(g.count.sum())
    mean_before = np.nanmean(g.render("average"))
    g.coarsen(4)
    assert int(g.count.sum()) == total_count
    assert abs(np.nanmean(g.render("average")) - mean_before) < 1.0
    assert abs(g.cell_size_m - 0.4) < 1e-12


def test_set_cell_size_coarsens_in_place_without_a_second_grid():
    svc = MosaicService(_cfg())
    kw = dict(range_m=15.0, bins=600, depth=4.0, speed=0.5, rate_hz=20.0,
              turn=False, one_sided=False)
    for k in range(30):
        svc.on_sonar_ping(make_ping(k, **kw))
    grid = svc._grid
    svc.set_cell_size(svc.cell_size_m * 2.0)
    assert svc._grid is grid and grid.coarsenings == 1     # same object


def test_to_rgba_matches_the_reference_implementation():
    r = MosaicRenderer(percentiles=(2.0, 98.0))
    r.settings = DisplaySettings()
    rng = np.random.default_rng(1)
    vals = rng.normal(-40.0, 8.0, (64, 96)).astype(np.float32)
    vals[5:9, 10:20] = np.nan
    lim = (-55.0, -25.0)
    got = r.to_rgba(vals, limits=lim)
    # Reference: the pre-2026-09-03 masked implementation.
    from blueboat_gcs.mapping.renderer import lut
    finite = np.isfinite(vals)
    norm = np.zeros_like(vals, dtype=np.float32)
    norm[finite] = np.clip((vals[finite] - lim[0]) / (lim[1] - lim[0]), 0.0, 1.0)
    idx = (norm * 255).astype(np.uint8)
    ref = np.zeros((*vals.shape, 4), dtype=np.uint8)
    ref[..., :3] = lut(r.settings.colormap)[idx]
    ref[..., 3] = np.where(finite, 255, 0)
    assert np.array_equal(got, ref)


def test_waterfall_window_move_renders_a_few_tiles_per_pass(qapp):
    cfg = _cfg()
    wf = WaterfallService(cfg)
    wf.set_enabled(True)
    tiles = []
    wf.tile_updated.connect(lambda first, img: tiles.append(first))
    kw = dict(range_m=15.0, bins=600, depth=4.0, speed=0.5, rate_hz=20.0,
              turn=False, one_sided=False)
    for k in range(6 * _TILE_ROWS):
        wf.on_sonar_ping(make_ping(k, **kw))
    wf._force_render()
    tiles.clear()
    # Force a window move: every tile becomes stale...
    wf._last_limits = (-999.0, -998.0)
    wf._force_render()
    # ...but at most dirty + _STALE_PER_PASS tiles are rendered per pass.
    assert 0 < len(tiles) <= 1 + wf._STALE_PER_PASS, tiles
    assert wf._stale_tiles                              # the rest follow
    n_passes = 0
    while wf._stale_tiles and n_passes < 20:
        wf._force_render(); n_passes += 1
    assert not wf._stale_tiles


def test_detection_row_is_resolved_once(qapp):
    wf = WaterfallService(_cfg())
    kw = dict(range_m=15.0, bins=600, depth=4.0, speed=0.5, rate_hz=20.0,
              turn=False, one_sided=False)
    for k in range(200):
        wf.on_sonar_ping(make_ping(k, **kw))
    wf.add_detection(100 / 20.0, 100 / 20.0 * 0.5, 3.0, "drum")
    assert wf._detections[-1]["row"] == 100
    ov = wf._overlay()
    assert ov and ov[0]["row"] == 100


def test_waterfall_capacity_is_bounded_in_samples(qapp):
    cfg = _cfg()
    cfg.mosaic.waterfall_max_samples = 600 * 1200      # ~600 rows at 1200 cols
    wf = WaterfallService(cfg)
    kw = dict(range_m=15.0, bins=600, depth=4.0, speed=0.5, rate_hz=20.0,
              turn=False, one_sided=False)
    for k in range(3 * _TILE_ROWS):
        wf.on_sonar_ping(make_ping(k, **kw))
    assert wf.total_rows - wf.first_row <= 2 * _TILE_ROWS + 1
