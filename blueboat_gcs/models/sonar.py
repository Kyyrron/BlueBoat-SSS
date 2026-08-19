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
    """

    t: float
    robot_x: float
    robot_y: float
    yaw: float
    water_depth: float
    y_local: np.ndarray  # float64, shape (N,)
    intensity_db: np.ndarray  # float32, shape (N,)
