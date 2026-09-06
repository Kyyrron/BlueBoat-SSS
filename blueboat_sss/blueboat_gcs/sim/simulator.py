"""Built-in data simulator (``--sim``).

Purpose: develop and demo the *entire* GUI — mosaic rendering, layers,
panels, coordinate conversion, tiles, detections, pinger — on any laptop
with no ROS installation and no boat. It emits exactly the same models on
the same signal bus as the ROS listeners, so the GUI cannot tell the
difference; this is also the executable specification of the placeholder
streams (detections / pinger) for the other repositories.

The synthetic scene: a lawnmower survey over a seabed with smooth
intensity texture, a few bright point targets with acoustic shadows, and
a wandering "pinger". Not physically rigorous — just realistic enough to
exercise rendering and interpolation.
"""

from __future__ import annotations

import math
import random
import time
from typing import Optional

import numpy as np
from PySide6.QtCore import QObject, QTimer

from ..config.settings import AppConfig
from ..core.signals import AppSignals
from ..models.detection import Detection, PingerFix
from ..models.path import PlannedPath
from ..models.robot_state import RobotState
from ..models.sonar import SonarPing
from ..utils.geodesy import enu_to_gps, yaw_to_compass_deg

_SWATH_HALF_M = 18.0
_SAMPLES_PER_SIDE = 220
_LINE_LENGTH_M = 90.0
_LINE_SPACING_M = 14.0
_TARGET_CLASSES = ("tire", "block", "chain", "pipe")
_TELEMETRY_HZ = 5.0


class Simulator(QObject):
    """Emits SonarPing / RobotState / Detection / PingerFix like real listeners."""

    def __init__(self, config: AppConfig, signals: AppSignals) -> None:
        super().__init__()
        self._cfg = config.sim
        self._signals = signals
        self._t = 0.0
        self._dist = 0.0
        self._targets = [(random.uniform(5, _LINE_LENGTH_M - 5),
                          random.uniform(2, 40),
                          random.choice(_TARGET_CLASSES))
                         for _ in range(6)]
        self._reported: set[int] = set()
        self._shadow_sign = 1.0

        self._timer = QTimer(self)
        self._timer.setInterval(int(1000.0 / self._cfg.ping_hz))
        self._timer.timeout.connect(self._tick)
        self._telemetry_timer = QTimer(self)
        self._telemetry_timer.setInterval(int(1000.0 / _TELEMETRY_HZ))
        self._telemetry_timer.timeout.connect(self._telemetry_tick)

    def start(self) -> bool:
        """Startup: 'pipeline up' — telemetry flows, sonar not firing."""
        if not self._telemetry_timer.isActive():
            self._telemetry_timer.start()
        self._signals.pipeline_state.emit("running")
        self._signals.status_message.emit(
            "Simulator pipeline up — press START for pings.")
        self._emit_planned_path()
        return True

    @property
    def running(self) -> bool:
        """Pipeline-up equivalent (interface parity with the launcher)."""
        return self._telemetry_timer.isActive()

    def enable_pinging(self) -> None:
        self._timer.start()
        self._signals.status_message.emit("Simulator pinging.")

    def disable_pinging(self) -> None:
        self._timer.stop()

    def set_recording(self, on: bool) -> bool:
        """Interface parity with PipelineLauncher (no-op in simulation)."""
        self._signals.log_line.emit(
            "processor", f"[sim] log_enable <- {str(on).lower()}")
        return True

    def reset_stream_health(self) -> None:
        """Interface parity with PipelineLauncher (no-op in simulation)."""

    def set_range(self, range_m: float) -> bool:
        """Interface parity with PipelineLauncher: the simulator's swath
        is fixed, so the request is refused gracefully."""
        self._signals.status_message.emit(
            "Simulator: the sonar range is fixed — range changes apply "
            "to the real sonar only.")
        return False

    def _emit_planned_path(self) -> None:
        """Publish the lawnmower plan, like path_publisher.py would."""
        points = []
        for pair in range(3):
            y0 = pair * 2 * _LINE_SPACING_M
            points += [(0.0, y0), (_LINE_LENGTH_M, y0),
                       (_LINE_LENGTH_M, y0 + _LINE_SPACING_M),
                       (0.0, y0 + _LINE_SPACING_M)]
        self._signals.planned_path.emit(
            PlannedPath(t=self._t, points=tuple(points)))

    def stop(self) -> None:
        self._timer.stop()
        self._telemetry_timer.stop()
        self._signals.pipeline_state.emit("stopped")

    # ---- lawnmower kinematics --------------------------------------------------
    def _pose(self) -> tuple[float, float, float]:
        leg_len = _LINE_LENGTH_M
        turn_len = _LINE_SPACING_M * math.pi / 2.0
        cycle = 2 * (leg_len + turn_len)
        d = self._dist % cycle
        line_pair = int(self._dist // cycle)
        y0 = line_pair * 2 * _LINE_SPACING_M
        r = _LINE_SPACING_M / 2.0
        if d < leg_len:                                   # east-bound leg
            return d, y0, 0.0
        d -= leg_len
        if d < turn_len:                                  # first U-turn
            a = d / r
            return (leg_len + math.sin(a) * r, y0 + r - math.cos(a) * r, a)
        d -= turn_len
        if d < leg_len:                                   # west-bound leg
            return leg_len - d, y0 + _LINE_SPACING_M, math.pi
        d -= leg_len
        a = d / r                                          # second U-turn
        return (-math.sin(a) * r,
                y0 + _LINE_SPACING_M + r - math.cos(a) * r, math.pi - a)

    # ---- seabed model ------------------------------------------------------------
    #: Native-row acoustics, shaped like the field logs and the Gazebo
    #: simulator (docs/SCIENTIFIC_BACKGROUND.md §8): transmit ringing at
    #: the transducer decaying through the first metre, a dark water
    #: column of volume reverberation, a bright first bottom return, then
    #: seabed texture falling with two-way spreading + absorption and a
    #: Lambert-like grazing-angle term; shadows sit at the receiver floor.
    _RING_DB, _RING_DECAY_DB_PER_M = 55.0, 22.0
    _WATER_COLUMN_DB, _WATER_COLUMN_SIGMA = 26.0, 3.0
    _SEABED_ONSET_DB = 60.0            # first bottom return at 2.5 m altitude
    _BS_DB_PER_DECADE = -25.0          # grazing-angle part of the falloff
    _FLOOR_DB = -5.0                   # receiver noise floor (shadows)
    _TEXTURE_SIGMA_DB = 4.0

    def _texture(self, xw: np.ndarray, yw: np.ndarray) -> np.ndarray:
        """Seabed texture + targets, in dB relative to the local mean;
        ``np.nan`` marks a shadow (rendered at the receiver floor)."""
        base = (4.0 * np.sin(xw * 0.11) * np.cos(yw * 0.07)
                + 2.0 * np.sin(xw * 0.53 + yw * 0.31))
        img = base + np.random.normal(0.0, self._TEXTURE_SIGMA_DB, xw.shape)
        shadowed = np.zeros(xw.shape, dtype=bool)
        for tx, ty, _cls in self._targets:
            d2 = (xw - tx) ** 2 + (yw - ty) ** 2
            img += 16.0 * np.exp(-d2 / 0.8)              # bright return
            shadowed |= ((np.abs(xw - tx) < 0.8) & (d2 < 25.0)
                         & ((yw - ty) * self._shadow_sign > 0.6))
        img[shadowed] = np.nan
        return img

    def _native_side(self, slant: np.ndarray, depth: float, xw: np.ndarray,
                     yw: np.ndarray, sign: float) -> np.ndarray:
        """One side's full native row (bin 0 at the transducer) in dB."""
        from ..core.display_model import transmission_loss_db
        ring = self._RING_DB - self._RING_DECAY_DB_PER_M * slant
        wc = self._WATER_COLUMN_DB + np.random.normal(
            0.0, self._WATER_COLUMN_SIGMA, slant.size)
        side = np.maximum(ring, wc)
        sb = slant > depth
        x = np.maximum(slant[sb] / depth, 1.0)
        level = (self._SEABED_ONSET_DB + transmission_loss_db(np.array([2.5]), 40.0, 0.1)[0]
                 + self._BS_DB_PER_DECADE * np.log10(x)
                 - transmission_loss_db(slant[sb], 40.0, 0.1))
        self._shadow_sign = sign
        tex = self._texture(xw, yw)
        seabed = level + np.nan_to_num(tex, nan=0.0)
        seabed[np.isnan(tex)] = self._FLOOR_DB + np.random.normal(
            0.0, 3.0, int(np.isnan(tex).sum()))
        side[sb] = seabed
        return side.astype(np.float32)

    # ---- ticks -----------------------------------------------------------------
    # Boat motion + telemetry run on their own timer as soon as the
    # "pipeline" is up (matches the real system: odom/mavros publish
    # regardless of the sonar); pings are emitted only while firing.
    def _telemetry_tick(self) -> None:
        dt = 1.0 / _TELEMETRY_HZ
        self._t += dt
        self._dist += self._cfg.speed_mps * dt
        x, y, yaw = self._pose()
        lat, lon = enu_to_gps(self._cfg.origin_lat, self._cfg.origin_lon, x, y)
        self._signals.robot_state.emit(RobotState(
            t=self._t, x=x, y=y, yaw=yaw, lat=lat, lon=lon,
            heading_deg=yaw_to_compass_deg(yaw),
            speed_mps=self._cfg.speed_mps))
        # Same auxiliary streams the real listeners emit, so the GPS
        # anchor and the compass heading policy are exercised on every
        # bench run (the anchor converges with t ≈ 0 within ~1 s).
        now = time.monotonic()
        self._signals.gps_fix.emit(now, lat, lon)
        self._signals.compass_heading.emit(now, yaw_to_compass_deg(yaw))
        self._maybe_emit_detection(x, y)
        if int(self._t * _TELEMETRY_HZ) % int(2 * _TELEMETRY_HZ) == 0:
            self._signals.pinger_fix.emit(PingerFix(
                t=self._t,
                x=45.0 + 2.0 * math.sin(self._t * 0.05),
                y=25.0 + 2.0 * math.cos(self._t * 0.04),
                accuracy_m=1.5, frame="world"))

    def _tick(self) -> None:
        x, y, yaw = self._pose()
        depth = 2.5 + 0.3 * math.sin(self._t * 0.2)
        # Uniform slant-bin grid like the real device (bin i at slant
        # i*Δ), the FULL row from the transducer outward (ringing, water
        # column, seabed) exactly as the raw profile carries it; the
        # ground samples for the mosaic are the seabed part (slant >
        # depth), cut as the processor cuts them.
        delta = _SWATH_HALF_M / _SAMPLES_PER_SIDE
        slant = np.arange(_SAMPLES_PER_SIDE, dtype=np.float64) * delta
        keep = slant > depth
        ground = np.sqrt(np.maximum(slant[keep] ** 2 - depth ** 2, 0.0))
        rows = []
        for sign in (+1.0, -1.0):
            yl = sign * ground
            xw = x - math.sin(yaw) * yl
            yw = y + math.cos(yaw) * yl
            rows.append(self._native_side(slant, depth, xw, yw, sign))
        port_db, stbd_db = rows
        y_local = np.concatenate([ground, -ground])
        intensity = np.concatenate([port_db[keep], stbd_db[keep]])
        self._signals.sonar_ping.emit(SonarPing(
            t=self._t, robot_x=x, robot_y=y, yaw=yaw,
            water_depth=depth,
            y_local=y_local,
            intensity_db=intensity,
            slant_range_m=_SWATH_HALF_M,
            bin_size_m=delta,
            port_bin0=0, port_db=port_db,
            stbd_bin0=0, stbd_db=stbd_db,
            bottom_slant_m=depth, gain_index=4))

    def _maybe_emit_detection(self, x: float, y: float) -> None:
        for uid, (tx, ty, cls) in enumerate(self._targets):
            if uid in self._reported:
                continue
            if math.hypot(x - tx, y - ty) < _SWATH_HALF_M * 0.8:
                self._reported.add(uid)
                self._signals.detection.emit(Detection(
                    uid=uid, t=self._t,
                    x=tx + random.uniform(-0.7, 0.7),
                    y=ty + random.uniform(-0.7, 0.7),
                    class_name=cls,
                    confidence=random.uniform(0.55, 0.95),
                    extent_m=random.uniform(0.6, 1.6)))
