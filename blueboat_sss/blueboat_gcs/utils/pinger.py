"""USBL pinger message gating (pure, ROS-free, unit-testable).

The producer (``robot_interface``) has two payload shapes on the same
``Float32MultiArray`` topic:

* ``[x, y, z]`` (normal path): **vehicle/body frame** (x forward,
  y port) — the Water Linked USBL vector, dead-reckoned between fixes;
* ``[x, y]`` (``fixed_pinger`` path): **world/odom frame** corrected
  position.

It also streams an all-zero 3-vector at odom rate (~20 Hz) *before the
pinger has ever been detected* — the field bug where the marker rode the
boat. :func:`parse_pinger` drops those, so the GUI only ever sees
genuine fixes.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

#: |value| below this in every component counts as "no detection yet"
#: (the publisher's zeros(3) placeholder).
ZERO_EPS = 1e-6


def parse_pinger(data: Sequence[float]) -> Optional[Tuple[float, float, str]]:
    """Validate one pinger message; ``(x, y, frame)`` or None to drop.

    * fewer than 2 values, or NaN -> None;
    * all values ~0 -> None (pre-detection placeholder, not a fix —
      a genuine fix exactly at the transducer/world origin is not a
      real case);
    * >= 3 values -> ``"body"`` (native USBL vector);
    * exactly 2   -> ``"world"`` (corrected position).
    """
    if len(data) < 2:
        return None
    vals = [float(v) for v in data[:3]]
    if any(math.isnan(v) for v in vals):
        return None
    if all(abs(v) < ZERO_EPS for v in vals):
        return None
    frame = "body" if len(data) >= 3 else "world"
    return vals[0], vals[1], frame
