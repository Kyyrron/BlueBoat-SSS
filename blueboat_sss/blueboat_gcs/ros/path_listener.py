"""Planned mission path: asked of the path server, ``/set_path`` as fallback.

**Primary source, like the Mission Control Station (2026-09-28).** With a
``PlannedPathConfig`` whose ``from_service`` is on, the listener asks
``path_generation`` which trajectory it serves (``get_parameters``:
``trajectory``, ``yaml_path``), reads that file's duration, and makes ONE
``/path_request`` over the whole of it -- the call MCS makes for its own
preview. The answer is the mission the controller is actually following, all
of it, in the run's world (odom) frame. It is asked again only when the
deployment changes (the file's mtime, the server restarting, the operator
clearing the overlay), never on a timer: on the real boat ``master_control``
shares that single-threaded server at 20 Hz. The decisions live in
``core/deployed_path.PlannedPathSource`` (ROS-free, tested there); this module
is only the ROS glue, all of it on the one ROS thread, so no locks.

**Fallback: ``nav_msgs/Path`` on ``topics.planned_path``** (default
``/set_path``) -- the message ``path_publisher.py`` publishes for RViz every
``refresh_period`` (5 s). It is a fixed ``total_time`` window onto the same
server (1000 s by default, truncating longer missions), so it is dropped while
the server can be asked directly, and draws only without one (a rosbag
replay). The publisher re-sends the same saved path every period (~4700
poses on a full mission), so an unchanged path is dropped before the map
repaint it would trigger. The comparison is the **full point tuple** (built
once, reused for the emission): the previous ``(len, first, last)``
fingerprint could not tell apart two missions sharing their endpoints and
pose count -- a re-deployed trajectory with a changed interior never reached
the map (project_review.md §3.3). At 4 700 poses the tuple build + compare is
well under a millisecond.

``reset()`` forgets the last path so the next one is forwarded again -- the
GUI calls it when the operator clears the planned-path overlay, otherwise the
cleared mission never comes back.

Frame: both sources carry poses from the path-generation service in the local
odom frame (the frame RViz displays them in), so no conversion is needed.
Only ``pose.position.x/.y`` are used.
"""

from __future__ import annotations

import time
from typing import Optional

from ..core.deployed_path import PlannedPathSource, RETRY_AFTER_S
from ..core.signals import AppSignals
from ..models.path import PlannedPath

try:
    from nav_msgs.msg import Path  # noqa: F401
    from rclpy.node import Node
    _MSGS_AVAILABLE = True
except ImportError:                                # pragma: no cover
    _MSGS_AVAILABLE = False

try:                                               # pragma: no cover - env
    from blueboat_interfaces.srv import RequestPath
    from rcl_interfaces.srv import GetParameters
    _SERVICE_AVAILABLE = True
except ImportError:                                # pragma: no cover
    _SERVICE_AVAILABLE = False


def path_points(poses) -> tuple:
    """The ``(x, y)`` tuple of a pose sequence — both the identity used
    to drop verbatim re-sends and the payload forwarded on the bus."""
    return tuple((p.pose.position.x, p.pose.position.y) for p in poses)


def path_fingerprint(poses) -> tuple:
    """Identity of a pose sequence: the full point tuple.

    Kept under its historical name for the callers/tests that compare
    fingerprints; it is no longer a length + endpoints digest (which
    missed a changed interior).
    """
    return path_points(poses)


def _param_string(value) -> str:
    """A ``rcl_interfaces/ParameterValue`` as a string ('' if not one)."""
    return str(getattr(value, "string_value", "") or "")


class PathListener:
    """Path server / nav_msgs/Path -> models.path.PlannedPath on the bus."""

    def __init__(self, node: "Node", signals: AppSignals,
                 topic: str, config=None) -> None:
        self._signals = signals
        self._last_points: tuple = ()
        self._source: Optional[PlannedPathSource] = None
        if not _MSGS_AVAILABLE:
            signals.status_message.emit(
                "nav_msgs not available — planned path display disabled.")
            return
        node.create_subscription(Path, topic, self._on_path, 10)
        node.get_logger().info(f"Planned path listener on {topic}")
        if config is not None and getattr(config, "from_service", False):
            self._start_service_source(node, config)

    # ---- the path server (primary) -----------------------------------------
    def _start_service_source(self, node: "Node", config) -> None:
        if not _SERVICE_AVAILABLE:
            self._signals.status_message.emit(
                "blueboat_interfaces not available — the planned path "
                f"comes from {config.service} only through /set_path.")
            return
        self._node = node
        self._timeout_s = float(config.request_timeout_s)
        self._source = PlannedPathSource(
            dt_s=config.dt_s, default_horizon_s=config.default_horizon_s,
            max_points=config.max_points)
        generator = "/" + config.generator_node.strip("/")
        self._path_client = node.create_client(RequestPath, config.service)
        self._param_client = node.create_client(
            GetParameters, f"{generator}/get_parameters")
        self._generator = generator
        self._future = None                # the one in-flight call
        self._future_kind = ""             # "params" | "path"
        self._future_req = None            # the PathRequest it answers
        self._issued_at = 0.0
        self._param_retry_at = 0.0
        self._timer = node.create_timer(float(config.poll_s), self._tick)
        node.get_logger().info(
            f"Planned path asked of {config.service} "
            f"(trajectory from {generator})")

    def _tick(self) -> None:
        """ROS-thread timer: harvest the in-flight call, else issue the next."""
        now = time.monotonic()
        if self._future is not None:
            if self._future.done():
                self._harvest(now)
            elif now - self._issued_at > self._timeout_s:
                self._future.cancel()
                self._fail(now, f"no reply within {self._timeout_s:.0f} s")
            return
        src = self._source
        src.service_ready(self._path_client.service_is_ready())
        if src.needs_parameters():
            if now < self._param_retry_at:
                return
            if not self._param_client.service_is_ready():
                return
            req = GetParameters.Request()
            req.names = ["trajectory", "yaml_path"]
            self._issue("params", self._param_client.call_async(req), None, now)
            return
        step = src.poll(now)
        if step is None:
            return
        if step.waiting:
            self._forward(())
            self._signals.status_message.emit(
                f"Planned path: waiting for {step.label} to be deployed.")
            return
        req = RequestPath.Request()
        req.path_request.data = [float(t) for t in step.times]
        self._issue("path", self._path_client.call_async(req), step, now)

    def _issue(self, kind: str, future, step, now: float) -> None:
        self._future, self._future_kind = future, kind
        self._future_req, self._issued_at = step, now

    def _harvest(self, now: float) -> None:
        future, kind, step = self._future, self._future_kind, self._future_req
        self._future = self._future_req = None
        try:
            result = future.result()
        except Exception as exc:          # noqa: BLE001 — surfaced, not raised
            self._fail(now, str(exc), kind=kind)
            return
        if result is None:
            self._fail(now, "empty reply", kind=kind)
            return
        if kind == "params":
            values = list(result.values)
            if not values:
                self._fail(now, f"{self._generator} returned no parameters",
                           kind=kind)
                return
            trajectory = _param_string(values[0])
            yaml_path = _param_string(values[1]) if len(values) > 1 else ""
            self._source.on_parameters(trajectory, yaml_path)
            self._node.get_logger().info(
                f"Planned path: {self._generator} serves '{trajectory}'")
            return
        points = self._source.on_response(step.key,
                                          path_points(result.path.poses))
        self._forward(points)
        if points:
            self._signals.status_message.emit(
                f"Planned path {step.label} from {self._generator}: "
                f"{len(points)} poses.")

    def _fail(self, now: float, detail: str, kind: Optional[str] = None) -> None:
        kind = kind or self._future_kind
        self._future = self._future_req = None
        if kind == "params":
            self._param_retry_at = now + RETRY_AFTER_S
        else:
            self._source.on_failure(now)
        self._node.get_logger().warn(
            f"Planned path request ({kind}) failed: {detail}")

    # ---- /set_path (fallback) ------------------------------------------------
    def reset(self) -> None:
        """Forget the last forwarded path: the next one, verbatim or not,
        is forwarded again (operator cleared the overlay)."""
        self._last_points = ()
        if self._source is not None:
            self._source.reset()

    def _on_path(self, msg: "Path") -> None:
        if self._source is not None and not self._source.accept_set_path():
            return            # the server itself is asked: whole mission
        self._forward(path_points(msg.poses))

    def _forward(self, points: tuple) -> None:
        if points == self._last_points:
            return
        self._last_points = points
        self._signals.planned_path.emit(
            PlannedPath(t=time.time(), points=points))
