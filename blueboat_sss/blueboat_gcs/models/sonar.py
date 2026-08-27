"""ROS-free representation of one processed side-scan ping.

`ros/sonar_listener.py` converts `blueboat_interfaces/ProcessedSSSPing`
into this dataclass at the ROS/Qt boundary, so that everything past the
signal bus (mosaic, renderer, GUI, simulator) has *zero* dependency on
ROS message types. This is what makes the whole GUI testable on a laptop
with `--sim` and no ROS installation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True)
class SonarPing:
    """One merged port+starboard ping, already slant-range corrected.

    Attributes
    ----------
    t:
        Ping timestamp in seconds (ROS time of the port packet).
    robot_x, robot_y:
        Robot position in the local odom frame [m] snapped at ping time.
    yaw:
        Robot heading in the local frame [rad], REP-103 (CCW from +x).
    water_depth:
        Estimated water depth under the boat [m] (FBR altitude + draft).
    y_local:
        Lateral sample coordinates in base_link [m]; +y = port,
        -y = starboard (concatenation of the two sides).
    intensity_db:
        Per-sample intensity [dB], aligned with ``y_local``.
    slant_range_m:
        The sonar's *configured* range for this ping [m] (``length_mm``
        / 1000). Optional, 0.0 when unknown. This is the geometrically
        stable across-track extent: unlike ``max|y_local|`` it does not
        move when the altitude estimate wobbles, so the waterfall uses
        it to keep a fixed column scale (a wandering altitude estimate
        otherwise rescales every row and makes the image ripple).
    sides:
        Which sides this ping actually carries: "both", "port" or
        "starboard". Pings are never dropped just because one side is
        missing, so consumers may see one-sided rows.
    """

    t: float
    robot_x: float
    robot_y: float
    yaw: float
    water_depth: float
    y_local: np.ndarray  # float64, shape (N,)
    intensity_db: np.ndarray  # float32, shape (N,)
    slant_range_m: float = 0.0
    sides: str = "both"
