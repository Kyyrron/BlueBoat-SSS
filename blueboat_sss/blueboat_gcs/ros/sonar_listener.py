"""Subscriber for the processed SSS stream + the raw per-side profiles.

Topics    : config ``topics.processed_ping`` (default /sss_processor/processed)
            and ``topics.port_profile`` / ``topics.starboard_profile``
            (the raw ``OmniscanProfile`` the processor itself consumes)
Types     : blueboat_interfaces/ProcessedSSSPing, blueboat_interfaces/OmniscanProfile
Rate      : ~20 Hz (one merged port+starboard row; two profiles per row)
Publishers: sss_processor_node.py, sss_node.py / sss_sim_node (unchanged)

This is the ROS/Qt boundary for sonar data: the message is converted to
the ROS-free ``SonarPing`` dataclass here and emitted on the signal bus.
Everything downstream — mosaic, waterfall, imager, GUI — is ROS-free.

Live/replay parity (2026-09-05): ``ProcessedSSSPing`` carries ground
samples with the water column already deleted, so the native slant-bin
row the waterfall and the AI pictures draw is taken from the raw
profiles instead — cached per side by device ``ping_number`` (the
processed message carries both raw counters) and attached to the row by
the ROS-free ``core/live_native.py``, exactly as the replay decoder
builds it. A missing profile falls back to the re-projection for that
side; no row is ever withheld (CM-6). Both subscriptions are BEST_EFFORT
to match the publishers (NC #4).
"""

from __future__ import annotations

import time
from collections import deque
from typing import Optional, Sequence

import numpy as np
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ..core.live_native import (PendingRows, ProcessedFields, ProfileCache,
                                attach_native, profile_from_msg)
from ..core.signals import AppSignals
from ..models.sonar import SonarPing
from ..utils.geodesy import quat_to_yaw

try:  # pragma: no cover - environment dependent
    from blueboat_interfaces.msg import OmniscanProfile, ProcessedSSSPing
    INTERFACES_AVAILABLE = True
except ImportError:  # pragma: no cover
    INTERFACES_AVAILABLE = False


#: Torn-row alarm window and threshold (same values as the processor's).
TORN_WINDOW_ROWS = 100
TORN_WARN_SHARE = 0.25

#: Profile-miss alarm: share of present sides over the last 100 rows that
#: had to fall back to the re-projection before one console line goes out.
MISS_WINDOW = 100
MISS_WARN_SHARE = 0.05


class SonarListener:
    """Converts ProcessedSSSPing messages into SonarPing signals."""

    def __init__(self, node: Node, signals: AppSignals, topic: str,
                 queue_depth: int = 200,
                 warn_on_ping_gap: bool = True,
                 profile_topics: Optional[Sequence[str]] = None,
                 profile_queue_depth: int = 200,
                 profile_cache_per_side: int = 512,
                 profile_wait_ms: int = 60) -> None:
        self._signals = signals
        self._warn_gaps = warn_on_ping_gap
        # Raw-profile attach (core/live_native): cache + in-order FIFO,
        # all touched on the executor thread only.
        self._cache = ProfileCache(per_side=profile_cache_per_side)
        self._pending = PendingRows(self._cache, wait_s=profile_wait_ms / 1000.0)
        self._profiles_enabled = bool(profile_topics)
        self.profile_hits = 0
        self.profile_misses = 0
        self.profile_late = 0
        self._miss_recent: deque = deque(maxlen=MISS_WINDOW)
        self._miss_warned = False
        # Stream health counters (also read by tests).
        self.received = 0
        self.device_gaps = 0        # pings the device numbered but we never saw
        self.crossed_pairs = 0      # halves whose counter gap left its usual value
        self.one_sided = 0          # rows carrying a single side
        self._pair_delta: Optional[int] = None   # the boat's steady counter offset
        self._last_port_pn: Optional[int] = None
        self._last_key: Optional[int] = None     # newest normalised ping number
        self._warned_cross = False
        self._next_gap_warn = 50
        # Torn-row alarm (mirrors the processor's): share of one-sided rows
        # over the recent window above which one console line is emitted.
        # Measured 2026-09-03: a whole session ran at 100 % one-sided rows
        # (every ping published as two half-rows) and nothing said so.
        self._torn_recent: deque = deque(maxlen=TORN_WINDOW_ROWS)
        self._torn_warned = False
        #: Newest arrival number (ROS thread writes, GUI thread reads: a
        #: single int assignment, atomic under the GIL).
        self.latest_seq = 0
        if not INTERFACES_AVAILABLE:
            signals.status_message.emit(
                "blueboat_interfaces not found — sonar stream disabled.")
            return
        # The processor publishes BEST_EFFORT; a RELIABLE subscriber would
        # be QoS-incompatible and receive nothing. BEST_EFFORT never
        # retransmits, so a deep queue is the only protection against
        # losing pings while the GUI thread renders (see
        # config.sonar_stream).
        if self._profiles_enabled:
            # Created BEFORE the processed subscription: rclpy's wait set
            # hands ready subscriptions over in creation order, so a
            # profile and its processed row waking the executor together
            # are delivered profile-first (the common, cache-hit case).
            pq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST,
                            depth=int(profile_queue_depth))
            for t in profile_topics:
                node.create_subscription(OmniscanProfile, t, self._on_profile, pq)
            node.create_timer(0.02, self._on_timer)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=int(queue_depth))
        node.create_subscription(ProcessedSSSPing, topic, self._on_msg, qos)

    def reset_pairing(self) -> None:
        """New acquisition START: forget the per-power-up expectations.

        The device counter offset (``_pair_delta``) and the last seen
        ``port_ping_number`` belong to one power-up of the sonar pair; a
        pipeline relaunch may reset either, and carrying them across
        sessions produced spurious crossed-pair and huge bogus gap
        warnings. The cumulative counters (received / device_gaps /
        crossed_pairs / one_sided) are per-application and stay."""
        self._pair_delta = None
        self._last_port_pn = None
        self._last_key = None
        self._warned_cross = False
        self._cache.clear()
        self._miss_warned = False

    # ---- raw profiles --------------------------------------------------------
    def _on_profile(self, msg: "OmniscanProfile") -> None:
        try:
            side, pn, prof = profile_from_msg(msg)
        except (AttributeError, ValueError, TypeError):
            return
        self._cache.put(side, pn, prof)

    def _on_timer(self) -> None:
        for item, f, timed_out in self._pending.drain(time.monotonic()):
            self._emit(item, f, timed_out)

    def _emit(self, base: dict, f: ProcessedFields, timed_out: bool) -> None:
        out, hits = attach_native(f, self._cache)
        if self._profiles_enabled:
            for present, hit in ((f.port_pn, hits[0]), (f.stbd_pn, hits[1])):
                if not present:
                    continue
                self._miss_recent.append(not hit)
                if hit:
                    self.profile_hits += 1
                else:
                    self.profile_misses += 1
            if timed_out:
                self.profile_late += 1
            if len(self._miss_recent) == MISS_WINDOW:
                share = sum(self._miss_recent) / MISS_WINDOW
                if share > MISS_WARN_SHARE and not self._miss_warned:
                    self._miss_warned = True
                    if self._warn_gaps:
                        self._signals.log_line.emit(
                            "app",
                            f"SONAR: {share * 100:.0f}% of the last {MISS_WINDOW} "
                            "rows had no raw profile in time — the waterfall "
                            "falls back to the re-projected row for those "
                            "(water column absent). Check the profile topics "
                            "reach the GCS and that the link is not saturated.")
                elif share < MISS_WARN_SHARE / 2:
                    self._miss_warned = False
        self._signals.sonar_ping.emit(SonarPing(**base, **out))

    def _on_msg(self, msg: "ProcessedSSSPing") -> None:
        # Same merging convention as the legacy listener: the sign of
        # y_local encodes the side (port=+, stbd=-), so one array suffices.
        y_local = np.concatenate([
            np.asarray(msg.port_y, dtype=np.float64),
            np.asarray(msg.starboard_y, dtype=np.float64),
        ])
        intensity = np.concatenate([
            np.asarray(msg.port_intensity_db, dtype=np.float32),
            np.asarray(msg.starboard_intensity_db, dtype=np.float32),
        ])
        # --- stream health -------------------------------------------------
        self.received += 1
        pn = int(getattr(msg, "port_ping_number", 0))
        spn = int(getattr(msg, "starboard_ping_number", 0))
        torn = not (pn and spn)
        if torn:
            self.one_sided += 1
        self._torn_recent.append(torn)
        if len(self._torn_recent) == TORN_WINDOW_ROWS:
            share = sum(self._torn_recent) / TORN_WINDOW_ROWS
            if share > TORN_WARN_SHARE and not self._torn_warned:
                self._torn_warned = True
                if self._warn_gaps:
                    self._signals.log_line.emit(
                        "app",
                        f"SONAR: {share * 100:.0f}% of the last {TORN_WINDOW_ROWS} "
                        "rows carry one side only — port and starboard are not "
                        "being paired upstream (the waterfall shows one black "
                        "row in two per side). Restart the sonar node and the "
                        "processor together, or check that both run on the "
                        "same clock.")
            elif share < TORN_WARN_SHARE / 2:
                self._torn_warned = False
        if not torn:
            # The two Omniscan units run independent ping counters, so the
            # gap between the halves of a correctly assembled row is the
            # boat's constant device offset (0, -1 and +60 all measured in
            # the field corpus) -- NOT zero. What signals a torn row is the
            # gap *changing*, so the first row sets the expectation and
            # departures from it are the defect.
            if self._pair_delta is None:
                self._pair_delta = pn - spn
            elif pn - spn != self._pair_delta:
                self.crossed_pairs += 1
                if self._warn_gaps and not self._warned_cross:
                    self._warned_cross = True
                    self._signals.log_line.emit(
                        "app",
                        "SONAR: port/starboard halves are no longer a fixed "
                        f"counter offset apart (#{pn} vs #{spn}; expected a gap "
                        f"of {self._pair_delta}). Either a device restarted its "
                        "counter or rows are being torn.")
        # Blank-row gaps key on the row's *normalised* ping number (the
        # newest present side, on the port counter): a jump there means
        # an instant where BOTH halves vanished — a fully lost ping. A
        # single-side loss keeps the keys contiguous and is already
        # visible as a one-sided (half-dark) row, so it must not insert
        # a blank line too (measured on the field corpus: the misspings
        # log loses 211 port + 49 stbd packets with zero fully-lost
        # instants).
        key: Optional[int] = None
        if pn and spn and self._pair_delta is not None:
            key = max(pn, spn + self._pair_delta)
        elif pn:
            key = pn
        elif spn and self._pair_delta is not None:
            key = spn + self._pair_delta
        gap_before = 0
        if key is not None and self._last_key is not None:
            k_missing = key - self._last_key - 1
            if 0 < k_missing < 1000:
                gap_before = k_missing
        if key is not None:
            self._last_key = key if self._last_key is None \
                else max(self._last_key, key)
        if pn and self._last_port_pn is not None:
            missing = pn - self._last_port_pn - 1
            if 0 < missing < 1000:
                first = self.device_gaps == 0
                self.device_gaps += missing
                if self._warn_gaps and (first or
                                        self.device_gaps >= self._next_gap_warn):
                    while self.device_gaps >= self._next_gap_warn:
                        self._next_gap_warn *= 4
                    self._signals.log_line.emit(
                        "app",
                        f"SONAR: {self.device_gaps} ping(s) lost upstream of "
                        "the GCS (gap in the device's own ping_number). This "
                        "is acquisition/QoS loss, not a display problem.")
        if pn:
            self._last_port_pn = pn

        q = msg.robot_orientation
        # Recover the sonar's CONFIGURED slant range exactly, even when
        # the altitude estimate wobbles: ground_max = sqrt(R^2 - h^2), so
        # R = hypot(ground_max, h). This gives the live waterfall the
        # same stable column scale the replay path gets from length_mm.
        ground_max = float(np.abs(y_local).max()) if y_local.size else 0.0
        depth = float(msg.water_depth)
        slant_range = float(np.hypot(ground_max, depth))
        # One-sided rows zero the absent side's stamp (presence is
        # `*_ping_number != 0`), so take the stamp of a side that is there.
        stamp = msg.port_stamp if pn else msg.starboard_stamp
        # Native slant bins, recovered exactly: the processor maps device
        # bin i (slant i*Δ) to ground = sqrt(slant² − h²), so
        # s = sqrt(ground² + h²) restores the uniform device grid. The
        # waterfall and the seabed imager draw these verbatim (SonarView
        # raw-domain convention) instead of scattering ground samples.
        py = np.asarray(msg.port_y, dtype=np.float64)
        pi = np.asarray(msg.port_intensity_db, dtype=np.float32)
        sy = np.asarray(msg.starboard_y, dtype=np.float64)
        si = np.asarray(msg.starboard_intensity_db, dtype=np.float32)
        self.latest_seq += 1
        # Published on the bus object too, so the GUI can measure its lag
        # without holding a reference to the ROS-side listener.
        self._signals.sonar_latest_seq = self.latest_seq
        base = dict(
            seq=self.latest_seq,
            t=stamp.sec + stamp.nanosec * 1e-9,
            robot_x=float(msg.robot_x),
            robot_y=float(msg.robot_y),
            yaw=quat_to_yaw(q.x, q.y, q.z, q.w),
            water_depth=float(msg.water_depth),
            y_local=y_local,
            intensity_db=intensity,
            slant_range_m=slant_range,
            # Presence is the ping number, not the array length: a present
            # side legitimately projects to zero samples when the altitude
            # estimate covers the whole swath.
            sides=("both" if (pn and spn) else ("port" if pn else "starboard")),
            gap_before=gap_before,
        )
        fields = ProcessedFields(port_pn=pn, stbd_pn=spn, water_depth=depth,
                                 port_y=py, port_db=pi, stbd_y=sy, stbd_db=si)
        # The native payload (raw profile bins, or the re-projection when
        # a profile is missing) is attached in arrival order; rows whose
        # profiles are still in flight wait at most profile_wait_ms.
        for item, f, timed_out in self._pending.push(time.monotonic(), base, fields):
            self._emit(item, f, timed_out)
