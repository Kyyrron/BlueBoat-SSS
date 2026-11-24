"""Resolution changes preserve the mosaic (field report fix).

The old ``set_cell_size`` rebuilt an empty grid and emitted ``cleared``,
wiping everything recorded so far off the screen. It now resamples the
old grid into the new one (old data keeps its native resolution, like
SonarView) and auto mode continuously keeps the best data-supported cell
size, refine-only.
"""

from __future__ import annotations

import numpy as np
import pytest

from blueboat_gcs.core.mosaic_service import MosaicService
from blueboat_gcs.mapping.mosaic import MosaicGrid
from blueboat_gcs.models.sonar import SonarPing


def _ping(t: float, x: float, spacing_m: float = 0.05,
          half_swath_m: float = 10.0) -> SonarPing:
    n = max(8, int(half_swath_m / spacing_m))
    y = np.linspace(spacing_m, half_swath_m, n)
    y_local = np.concatenate([y, -y])
    return SonarPing(
        t=t, robot_x=x, robot_y=0.0, yaw=0.0, water_depth=2.0,
        y_local=y_local,
        intensity_db=np.full(y_local.size, -30.0, dtype=np.float32),
        slant_range_m=half_swath_m)


def _feed(service: MosaicService, n: int, spacing: float = 0.05,
          t0: float = 0.0, x0: float = 0.0) -> None:
    for k in range(n):
        service.on_sonar_ping(_ping(t0 + 0.1 * k, x0 + 0.1 * k, spacing))


def _valid_cells(service: MosaicService) -> int:
    return int(np.isfinite(service._grid.render()).sum())


# --------------------------------------------------------------- grid level
def test_resample_refine_preserves_mean():
    coarse = MosaicGrid(cell_size_m=0.20, initial_half_extent_m=5.0)
    xs = np.array([0.0, 1.0, 2.0])
    coarse.add_samples(xs, np.zeros(3), np.full(3, -30.0), bilinear=False)
    fine = MosaicGrid(cell_size_m=0.05, initial_half_extent_m=5.0)
    fine.resample_from(coarse)
    img = fine.render()
    valid = img[np.isfinite(img)]
    assert valid.size > 0
    assert np.allclose(valid, -30.0)
    # A coarse cell paints the whole block of finer cells it covered:
    # each 20 cm cell covers ~4x4 5 cm cells.
    assert valid.size >= 3 * 16


def test_resample_coarsen_aggregates():
    fine = MosaicGrid(cell_size_m=0.05, initial_half_extent_m=5.0)
    xs = np.linspace(0.0, 0.15, 4)                # all inside one 20 cm cell
    fine.add_samples(xs, np.zeros(4), np.array([-20.0, -30.0, -30.0, -40.0]),
                     bilinear=False)
    coarse = MosaicGrid(cell_size_m=0.20, initial_half_extent_m=5.0)
    coarse.resample_from(fine)
    img = coarse.render()
    valid = img[np.isfinite(img)]
    assert valid.size >= 1
    assert np.isclose(valid.min(), -30.0)          # weighted mean preserved


# ------------------------------------------------------------ service level
def test_manual_change_preserves_data(qapp, tmp_config):
    tmp_config.mosaic.auto_cell_size = False
    service = MosaicService(tmp_config)
    cleared = []
    service.cleared.connect(lambda: cleared.append(True))
    _feed(service, 30)
    before = _valid_cells(service)
    assert before > 0
    extent_before = service._grid.extent

    service.set_fixed_cell_size(0.05)              # refine from 0.10
    assert service.cell_size_m == pytest.approx(0.05)
    assert cleared == [], "resolution change must not clear the mosaic"
    after = _valid_cells(service)
    assert after > 0
    # The data footprint survives (finer grid -> more, never zero).
    assert after >= before
    e = service._grid.extent
    assert e[0] <= extent_before[1] and e[1] >= extent_before[0]

    service.set_fixed_cell_size(0.25)              # coarsen back
    assert cleared == []
    assert _valid_cells(service) > 0


def test_auto_refines_but_never_coarsens(qapp, tmp_config):
    tmp_config.mosaic.auto_cell_size = True
    service = MosaicService(tmp_config)
    _feed(service, 2, spacing=0.10)                # first tune ~0.10 m
    tuned = service.cell_size_m
    assert tuned == pytest.approx(0.10, rel=0.3)

    # Coarser acquisition mid-mission: cell size must NOT degrade.
    _feed(service, 25, spacing=0.30, t0=10.0)
    assert service.cell_size_m == pytest.approx(tuned)

    # Finer acquisition: auto refines, data preserved.
    _feed(service, 25, spacing=0.03, t0=20.0, x0=5.0)
    assert service.cell_size_m < tuned * 0.85
    assert _valid_cells(service) > 0


def test_manual_disables_auto_and_auto_reenables(qapp, tmp_config):
    tmp_config.mosaic.auto_cell_size = True
    service = MosaicService(tmp_config)
    _feed(service, 2, spacing=0.10)
    service.set_fixed_cell_size(0.50)
    _feed(service, 30, spacing=0.03, t0=10.0)      # would refine in auto
    assert service.cell_size_m == pytest.approx(0.50)

    service.enable_auto_resolution()
    _feed(service, 2, spacing=0.03, t0=20.0)
    assert service.cell_size_m < 0.10


def test_trajectory_and_depth_series_survive_changes(qapp, tmp_config):
    tmp_config.mosaic.auto_cell_size = False
    service = MosaicService(tmp_config)
    _feed(service, 20)
    service.set_fixed_cell_size(0.05)
    service.set_fixed_cell_size(0.25)
    assert len(service._traj) == 20
    assert len(service._depth_t) == 20
