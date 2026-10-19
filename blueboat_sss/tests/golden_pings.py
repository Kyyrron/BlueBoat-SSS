"""Deterministic synthetic ping sequence for the Perception-integration golden.

``tests/fixtures/perception_golden.json`` was captured from the station code as
it stood **before** the BlueBoat-Perception changes (2026-10-16), by
``tests/capture_perception_golden.py`` over exactly these pings.
``test_perception_golden.py`` replays them through today's code and checks
what must not have changed:
- the pictures (raw dB and PNG, bit for bit);
- the metadata (except the keys added since);
- the analysis payload;
- the convention-1 world grids, which the new pixel→world reproduces with
  its default arguments.

The sequence is built to exercise every image path at once:
- a straight leg, then a U-turn, then a return leg;
- heave on the altitude, plus a slow depth ramp;
- a receive-gain step;
- a few one-sided pings;
- a range change, which ends the window;
- a session break, also ending the window;
- a bright target with its shadow.

Everything is drawn from one seeded ``numpy.random.Generator``, so the
sequence is a pure function of the code below. A SHA-256 of the generated
arrays is stored in the golden, so a generator drift is reported as such
rather than as a station regression.
"""

from __future__ import annotations

import hashlib
import math
from typing import List

import numpy as np

from blueboat_gcs.models.sonar import SonarPing

N_BINS = 600                 # device bins per side
RANGE_A_M = 20.0             # first range setting
RANGE_B_M = 15.0             # after the mid-run range change
DT_S = 0.05                  # 20 Hz
SUBMERSION_M = 0.15          # water_depth = altitude + submersion (processor convention)
GAIN_REF = 74.55             # analog gain at index 4
GAIN_UP = 142.8              # one index up: +5.65 dB on the wire

T_TURN_START = 24.0
TURN_RATE = 0.2              # rad/s
T_TURN_END = T_TURN_START + math.pi / TURN_RATE
T_RANGE_CHANGE = T_TURN_END + 6.0
T_BREAK = T_RANGE_CHANGE + 8.0      # a session gap: feed_pings(breaks=[T_BREAK])
T_END = T_BREAK + 10.0


def _pose(t: float):
    """(x, y, yaw, speed): east leg, U-turn to the north, west leg."""
    if t < T_TURN_START:
        v = 0.6
        return v * t, 0.0, 0.0, v
    x0 = 0.6 * T_TURN_START
    v = 0.5
    radius = v / TURN_RATE
    if t < T_TURN_END:
        a = TURN_RATE * (t - T_TURN_START)
        return x0 + radius * math.sin(a), radius * (1.0 - math.cos(a)), a, v
    v = 0.6
    yaw = math.pi
    return x0 - v * (t - T_TURN_END), 2.0 * radius, yaw, v


def _altitude(t: float) -> float:
    return 1.2 + 0.004 * t + 0.05 * math.sin(2.0 * math.pi * t / 3.0)


def _side(rng, h: float, pitch: float, target: bool) -> np.ndarray:
    s = (np.arange(N_BINS) + 0.5) * pitch
    ring = 53.0 - 22.0 * s
    wc = 26.0 + rng.normal(0.0, 3.0, N_BINS)
    out = np.maximum(ring, wc)
    sb = s >= h
    x = np.maximum(s[sb] / h, 1.0)
    tl = 40.0 * np.log10(np.maximum(s[sb], 0.01)) + 2.0 * 0.1 * s[sb]
    out[sb] = (90.0 - 25.0 * np.log10(x) - tl
               + rng.normal(0.0, 4.0, int(sb.sum())))
    if target:
        hi = (s >= 6.0) & (s < 6.3)
        sh = (s >= 6.3) & (s < 8.0)
        out[hi] += 15.0
        out[sh] = -5.0 + rng.normal(0.0, 3.0, int(sh.sum()))
    return out.astype(np.float32)


def make_golden_pings() -> List[SonarPing]:
    rng = np.random.default_rng(20261016)
    pings: List[SonarPing] = []
    n = int(round(T_END / DT_S))
    for i in range(n):
        t = i * DT_S
        x, y, yaw, _v = _pose(t)
        h = _altitude(t)
        rng_m = RANGE_A_M if t < T_RANGE_CHANGE else RANGE_B_M
        pitch = rng_m / (N_BINS - 1)
        target = 100 <= i < 112
        port = _side(rng, h, pitch, target)
        stbd = _side(rng, h, pitch, False)
        gain = GAIN_UP if 300 <= i < 360 else GAIN_REF
        if gain != GAIN_REF:
            g = np.float32(20.0 * math.log10(gain / GAIN_REF))
            port = port + g
            stbd = stbd + g
        one_sided = i in (50, 51, 52)
        pings.append(SonarPing(
            t=t, robot_x=x, robot_y=y, yaw=yaw,
            water_depth=h + SUBMERSION_M,
            y_local=np.zeros(0), intensity_db=np.zeros(0, np.float32),
            slant_range_m=rng_m,
            sides="port" if one_sided else "both",
            bin_size_m=pitch, port_bin0=0, port_db=port,
            stbd_bin0=0, stbd_db=None if one_sided else stbd,
            bottom_slant_m=h, gain_index=5 if gain != GAIN_REF else 4,
            port_analog_gain=gain,
            stbd_analog_gain=0.0 if one_sided else gain,
            seq=i + 1))
    return pings


def golden_breaks() -> List[float]:
    return [T_BREAK]


def pings_sha256(pings: List[SonarPing]) -> str:
    h = hashlib.sha256()
    for p in pings:
        h.update(np.float64([p.t, p.robot_x, p.robot_y, p.yaw, p.water_depth,
                             p.bin_size_m, p.bottom_slant_m]).tobytes())
        h.update(p.port_db.tobytes())
        if p.stbd_db is not None:
            h.update(p.stbd_db.tobytes())
    return h.hexdigest()


def sha256_array(a: np.ndarray) -> str:
    a = np.ascontiguousarray(a)
    return hashlib.sha256(str(a.dtype).encode() + str(a.shape).encode()
                          + a.tobytes()).hexdigest()
