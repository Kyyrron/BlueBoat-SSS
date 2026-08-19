#!/usr/bin/env python3
"""
Offline plotter for the rasterised sonar mosaic produced by
processed_sss_listener.py.

The listener writes:
    sonar_mosaic.npz       -- mean_intensity, count, cell_size_m, x0, y0
    sonar_mosaic.png       -- quick-look preview
    boat_trajectory.csv    -- per-ping (t, x, y, depth)

This script reloads the npz + trajectory and re-renders with proper
contrast control and a trajectory overlay. Run after a survey:

    python3 processed_sss_printer.py

"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main() -> None:

    path_from_ws = "data/SSS_data/"
    prefix = path_from_ws + "2026_06_12-17_12/" # Name of the experiment's folder
    default_mosaic = prefix + "sonar_mosaic.npz" # Output of processed_sss_listener.py node.
    default_trajectory = prefix + "boat_trajectory.csv" # Same

    ap = argparse.ArgumentParser()
    ap.add_argument("--mosaic",     default=default_mosaic, type=Path)
    ap.add_argument("--trajectory", default=default_trajectory, type=Path)
    ap.add_argument("--cmap", default="copper")
    args = ap.parse_args()

    data = np.load(args.mosaic)
    img         = data["mean_intensity"]
    count       = data["count"]
    cell        = float(data["cell_size_m"])
    x0          = float(data["x0"])
    y0          = float(data["y0"])
    h, w = img.shape
    extent = (x0, x0 + w * cell, y0, y0 + h * cell)

    valid = img[np.isfinite(img)]
    vmin, vmax = np.percentile(valid, [2, 98])

    coverage = 100.0 * np.count_nonzero(count) / count.size
    print(f"loaded mosaic: {w}×{h} cells at {cell*100:.0f} cm/cell "
          f"({coverage:.1f}% covered)")
    print(f"intensity clip: [{vmin:.1f}, {vmax:.1f}] dB")

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(img, origin="lower", extent=extent,
              cmap=args.cmap, vmin=vmin, vmax=vmax,
              interpolation="nearest")

    if args.trajectory.exists():
        xs, ys = [], []
        with open(args.trajectory) as f:
            r = csv.DictReader(f)
            for row in r:
                xs.append(float(row["x_m"]))
                ys.append(float(row["y_m"]))
        ax.plot(xs, ys, "-", color="cyan", linewidth=1.2,
                alpha=0.9, label=f"boat track ({len(xs)} pings)")
        if xs:
            ax.plot(xs[0],  ys[0],  "o", color="lime",  markersize=8, label="start")
            ax.plot(xs[-1], ys[-1], "o", color="red",   markersize=8, label="end")
        ax.legend(loc="upper right")

    ax.set_xlabel("X world (m)")
    ax.set_ylabel("Y world (m)")
    ax.set_title("Side-scan sonar mosaic")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    plt.savefig(args.mosaic.with_suffix(".png"), dpi=300) # Saves the plot
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
