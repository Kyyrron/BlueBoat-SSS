"""WaterfallService growable tiled buffer.

The old 1500-row ring silently discarded the head of any longer mission
and re-colormapped the whole image per render. The tiled buffer grows
with the mission (row index == ping index forever), renders dirty tiles
only, evicts whole tiles past the memory cap, and keeps the export
contract (`waterfall_raw.npz` carries the full float32 buffer).
"""

from __future__ import annotations

import numpy as np
import pytest

from blueboat_gcs.core import waterfall_service as ws_mod
from blueboat_gcs.core.waterfall_service import _TILE_ROWS, WaterfallService
from blueboat_gcs.models.sonar import SonarPing

RANGE_M = 18.0
N_BINS = 64
DEPTH = 2.0


def make_ping(i: int) -> SonarPing:
    """Uniform slant-bin ping (water column cut), like the processor's."""
    pitch = RANGE_M / N_BINS
    slant = np.arange(N_BINS) * pitch
    keep = slant > DEPTH
    ground = np.sqrt(slant[keep] ** 2 - DEPTH ** 2)
    y = np.concatenate([ground, -ground])
    return SonarPing(
        t=float(i), robot_x=float(i), robot_y=2.0 * i, yaw=0.1,
        water_depth=DEPTH, y_local=y,
        intensity_db=np.full(y.size, -20.0 - (i % 50) * 0.5, np.float32),
        slant_range_m=RANGE_M)


@pytest.fixture
def service(qapp, tmp_config):
    tmp_config.mosaic.waterfall_max_rows = 100_000
    return WaterfallService(tmp_config)


def _row_value(chrono: np.ndarray, i: int) -> float:
    vals = chrono[i][np.isfinite(chrono[i])]
    assert vals.size, f"row {i} is empty"
    return float(vals[0])


def test_buffer_grows_past_the_old_ring_size(service):
    n = 2000                                  # > the retired 1500-row ring
    for i in range(n):
        service.on_sonar_ping(make_ping(i))
    assert service.total_rows == n
    chrono = service.chronological()
    assert chrono.shape == (n, service.columns)
    # The head of the mission is still there — the whole point.
    assert _row_value(chrono, 0) == pytest.approx(-20.0)
    assert _row_value(chrono, n - 1) == pytest.approx(-20.0 - ((n - 1) % 50) * 0.5)


def test_rows_are_continuous_across_tile_boundaries(service):
    for i in range(_TILE_ROWS + 3):
        service.on_sonar_ping(make_ping(i))
    chrono = service.chronological()
    for i in (_TILE_ROWS - 1, _TILE_ROWS, _TILE_ROWS + 1):
        assert _row_value(chrono, i) == pytest.approx(-20.0 - (i % 50) * 0.5)
        assert service.row_meta(i)[0] == pytest.approx(float(i))


def test_memory_cap_evicts_oldest_whole_tiles(qapp, tmp_config):
    tmp_config.mosaic.waterfall_max_rows = 1000
    svc = WaterfallService(tmp_config)
    n = 2000
    for i in range(n):
        svc.on_sonar_ping(make_ping(i))
    assert svc.total_rows == n                      # indices stay absolute
    assert svc.first_row > 0
    assert svc.first_row % _TILE_ROWS == 0          # whole tiles only
    assert n - svc.first_row <= 1000
    # Evicted rows answer None; surviving rows keep their own metadata.
    assert svc.row_meta(0) is None
    meta = svc.row_meta(n - 1)
    assert meta is not None and meta[0] == pytest.approx(float(n - 1))
    chrono = svc.chronological()
    assert chrono.shape[0] == n - svc.first_row
    assert _row_value(chrono, 0) == pytest.approx(
        -20.0 - (svc.first_row % 50) * 0.5)


def test_reserve_lifts_the_cap_for_replay(qapp, tmp_config):
    tmp_config.mosaic.waterfall_max_rows = 1000
    svc = WaterfallService(tmp_config)
    svc.reserve(5000)
    for i in range(3000):
        svc.on_sonar_ping(make_ping(i))
    assert svc.first_row == 0, "reserve() did not prevent eviction"
    assert svc.total_rows == 3000


def test_break_row_is_a_nan_seam_with_no_pose(service):
    for i in range(10):
        service.on_sonar_ping(make_ping(i))
    service.break_row()
    service.on_sonar_ping(make_ping(11))
    chrono = service.chronological()
    assert not np.isfinite(chrono[10]).any(), "the gap row is not blank"
    assert service.row_meta(10) is None
    assert service.row_meta(9) is not None
    assert service.row_meta(11) is not None


def test_render_emits_tiles_and_layout(service):
    layouts, tiles = [], []
    service.layout_changed.connect(lambda *a: layouts.append(a))
    service.tile_updated.connect(lambda first, img: tiles.append(first))
    for i in range(2 * _TILE_ROWS + 10):
        service.on_sonar_ping(make_ping(i))
    service.set_enabled(True)                       # forces a full render
    assert layouts and layouts[-1][:3] == (0, 2 * _TILE_ROWS + 10,
                                           service.columns)
    assert sorted(tiles) == [0, _TILE_ROWS, 2 * _TILE_ROWS]

    # A new ping dirties only the tail tile (the global contrast window
    # is unchanged — the intensities repeat), so the next render is O(1).
    tiles.clear()
    service.on_sonar_ping(make_ping(2 * _TILE_ROWS + 10))
    service._force_render()
    assert tiles == [2 * _TILE_ROWS]


def test_clear_resets_to_an_empty_layout(service):
    layouts = []
    service.layout_changed.connect(lambda *a: layouts.append(a))
    for i in range(5):
        service.on_sonar_ping(make_ping(i))
    service.clear()
    assert service.chronological() is None
    assert service.total_rows == 0 and service.first_row == 0
    assert layouts[-1][:2] == (0, 0)


def test_export_full_npz_and_decimated_png(qapp, tmp_config, tmp_path,
                                           monkeypatch):
    import cv2
    monkeypatch.setattr(ws_mod, "_PNG_MAX_ROWS", 100)
    svc = WaterfallService(tmp_config)
    n = 300
    for i in range(n):
        svc.on_sonar_ping(make_ping(i))
    assert svc.export_into(tmp_path / "waterfall")
    with np.load(tmp_path / "waterfall" / "waterfall_raw.npz") as npz:
        assert npz["intensity_db"].shape == (n, svc.columns), (
            "the npz must stay full fidelity — the archival raw record")
        assert float(npz["slant_pitch_m"]) == pytest.approx(RANGE_M / N_BINS)
        assert int(npz["png_row_stride"]) == 3
    png = cv2.imread(str(tmp_path / "waterfall" / "waterfall.png"),
                     cv2.IMREAD_UNCHANGED)
    assert png.shape[0] == 100, "the PNG quick-look was not decimated"
