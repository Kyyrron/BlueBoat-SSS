"""Offline throughput benchmark of the live SSS pipeline (no ROS, no GUI).

Feeds synthetic two-sided (or one-sided) pings at survey geometry into the
three services the main window drives per ping -- MosaicService,
WaterfallService, SeabedImager -- and reports the ingest cost per ping,
the render cost per pass, the derived mosaic cell size and the process
RSS, all as functions of the buffer size. The 2026-09-03 crash (an
unbounded mosaic grid, three times larger with two-sided pings) was found
and is guarded with this script:

    QT_QPA_PLATFORM=offscreen python3 -m blueboat_gcs.analysis.ping_bench \\
        --pings 5000 [--one-sided] [--range 30 --bins 1200] [--turn]

Targets (see .claude/CLAUDE.md): ingest <= 2 ms/ping at 600 bins, render
<= 15 ms/pass, RSS flat once the caps are reached.
"""

from __future__ import annotations

import argparse
import math
import os
import resource
import sys
import time

import numpy as np


def make_ping(k: int, *, range_m: float, bins: int, depth: float,
              speed: float, rate_hz: float, one_sided: bool, turn: bool,
              seq: int = 0):
    """A ping shaped like the live stream after the raw-profile attach:
    the FULL native row per side (ringing core, dark water column, seabed
    with a 40 log r + Lambert falloff, texture) plus the ground samples
    the processor publishes (seabed part only, with a 1 cm transducer
    asymmetry between the sides)."""
    from blueboat_gcs.models.sonar import SonarPing
    bin_m = range_m / bins
    slant = (np.arange(bins) + 0.5) * bin_m
    ground = np.sqrt(np.clip(slant ** 2 - depth ** 2, 0.0, None))
    keep = slant > depth
    port_y = ground[keep] + 0.20
    stbd_y = ground[keep] + 0.19             # 1 cm transducer asymmetry
    rng = np.random.default_rng(k)

    def side():
        row = np.maximum(55.0 - 22.0 * slant,
                         26.0 + rng.normal(0, 3, bins))
        x = np.maximum(slant[keep] / depth, 1.0)
        row[keep] = (60.0 + 40.0 * np.log10(depth) + 0.2 * depth
                     - 25.0 * np.log10(x) - 40.0 * np.log10(slant[keep])
                     - 0.2 * slant[keep] + rng.normal(0, 4, keep.sum()))
        return row.astype(np.float32)

    p_row, s_row = side(), side()
    p_db, s_db = p_row[keep], s_row[keep]
    t = k / rate_hz
    if turn:
        leg = 60.0
        x, y = (t * speed) % leg, 3.0 * int((t * speed) // leg)
        yaw = 0.0 if int((t * speed) // leg) % 2 == 0 else math.pi
    else:
        x, y, yaw = t * speed, 0.0, 0.0
    if one_sided:
        y_local, inten, sides = port_y, p_db, "port"
        s_row = None
    else:
        y_local = np.concatenate([port_y, -stbd_y])
        inten = np.concatenate([p_db, s_db])
        sides = "both"
    return SonarPing(t=t, robot_x=x, robot_y=y, yaw=yaw, water_depth=depth,
                     y_local=y_local.astype(np.float64), intensity_db=inten,
                     slant_range_m=range_m, sides=sides, gap_before=0,
                     bin_size_m=bin_m, port_bin0=0, port_db=p_row,
                     stbd_bin0=0, stbd_db=s_row,
                     bottom_slant_m=depth, seq=seq, gain_index=4)


def rss_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--pings", type=int, default=3000)
    ap.add_argument("--range", type=float, default=15.0)
    ap.add_argument("--bins", type=int, default=600)
    ap.add_argument("--depth", type=float, default=4.0)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--one-sided", action="store_true")
    ap.add_argument("--turn", action="store_true", help="lawnmower legs")
    ap.add_argument("--report-every", type=int, default=1000)
    args = ap.parse_args(argv)

    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    from blueboat_gcs.config.settings import AppConfig
    from blueboat_gcs.core.mosaic_service import MosaicService
    from blueboat_gcs.core.seabed_imager import SeabedImager
    from blueboat_gcs.core.waterfall_service import WaterfallService

    from blueboat_gcs.core.display_model import DisplayModel
    cfg = AppConfig()
    # One shared display model, as the main window wires it: the waterfall
    # feeds it, the mosaic and the imager render through it.
    model = DisplayModel(cfg)
    mosaic = MosaicService(cfg, model)
    waterfall = WaterfallService(cfg, model)
    waterfall.set_enabled(True)
    imager = SeabedImager(cfg, model=model)
    t_ing = {"mosaic": 0.0, "waterfall": 0.0, "imager": 0.0}
    t_render = {"mosaic": [], "waterfall": []}
    print(f"{'ping':>6} {'ingest ms':>9} {'mosaic':>7} {'wfall':>6} {'imager':>6} "
          f"{'render ms (mos/wf)':>18} {'cell m':>7} {'grid':>12} {'rows':>6} {'RSS MB':>7}")
    for k in range(args.pings):
        ping = make_ping(k, range_m=args.range, bins=args.bins, depth=args.depth,
                         speed=args.speed, rate_hz=args.rate,
                         one_sided=args.one_sided, turn=args.turn, seq=k + 1)
        t0 = time.perf_counter(); mosaic.on_sonar_ping(ping)
        t1 = time.perf_counter(); waterfall.on_sonar_ping(ping)
        t2 = time.perf_counter(); imager.on_sonar_ping(ping)
        t3 = time.perf_counter()
        t_ing["mosaic"] += t1 - t0; t_ing["waterfall"] += t2 - t1; t_ing["imager"] += t3 - t2
        if k % int(args.rate / 4) == 0:              # the 4 Hz waterfall timer
            r0 = time.perf_counter(); waterfall._render_if_dirty(); t_render["waterfall"].append(time.perf_counter() - r0)
        if k % int(args.rate / cfg.mosaic.mosaic_render_hz) == 0:
            r0 = time.perf_counter(); mosaic._render_if_dirty(); t_render["mosaic"].append(time.perf_counter() - r0)
        app.processEvents()
        if (k + 1) % args.report_every == 0:
            n = k + 1
            tot = sum(t_ing.values()) / n * 1e3
            print(f"{n:>6} {tot:>9.2f} {t_ing['mosaic']/n*1e3:>7.2f} "
                  f"{t_ing['waterfall']/n*1e3:>6.2f} {t_ing['imager']/n*1e3:>6.2f} "
                  f"{np.mean(t_render['mosaic'][-8:])*1e3:>8.1f}/{np.mean(t_render['waterfall'][-8:])*1e3:<8.1f} "
                  f"{mosaic.cell_size_m:>7.4f} {str(mosaic._grid.shape):>12} "
                  f"{waterfall.total_rows - waterfall.first_row:>6} {rss_mb():>7.0f}")
    n = args.pings
    print(f"\ningest {sum(t_ing.values())/n*1e3:.2f} ms/ping; renders: mosaic "
          f"{np.mean(t_render['mosaic'])*1e3:.1f} ms (max {np.max(t_render['mosaic'])*1e3:.0f}), "
          f"waterfall {np.mean(t_render['waterfall'])*1e3:.1f} ms (max "
          f"{np.max(t_render['waterfall'])*1e3:.0f}); grid {mosaic._grid.shape} cells "
          f"{np.prod(mosaic._grid.shape)/1e6:.1f} M (coarsenings {mosaic._grid.coarsenings}, "
          f"refused {mosaic._grid.refused_growths}); RSS {rss_mb():.0f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
