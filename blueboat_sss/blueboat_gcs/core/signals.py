"""Application-wide signal bus.

Design decision
---------------
All data sources (ROS listeners running in the rclpy thread, the
simulator, the pipeline launcher) publish into this single ``AppSignals``
object; all consumers (map layers, panels, services) subscribe to it.

Because the emitting threads are not the GUI thread, Qt automatically
delivers these signals via *queued connections*: every slot therefore
runs in the GUI thread. This gives us a lock-free architecture — the
mosaic grid, the trajectory buffers and every QGraphicsItem are only
ever touched from one thread.

Adding a new ROS topic later = add a model dataclass, a signal here,
a listener in ``ros/``, and connect it in ``gui/main_window.py``.
"""

from __future__ import annotations

from PySide6.QtCore import QObject, Signal


class AppSignals(QObject):
    """Typed signal hub. One instance per application, injected everywhere."""

    # --- data streams -----------------------------------------------------
    sonar_ping = Signal(object)        # models.sonar.SonarPing
    robot_state = Signal(object)       # models.robot_state.RobotState
    detection = Signal(object)         # models.detection.Detection
    pinger_fix = Signal(object)        # models.detection.PingerFix
    planned_path = Signal(object)      # models.path.PlannedPath (replaces previous)

    # --- geo referencing ----------------------------------------------------
    origin_bound = Signal(float, float)  # (lat0, lon0) once local<->GPS is known

    # --- pipeline / lifecycle ----------------------------------------------
    pipeline_state = Signal(str)       # "stopped" | "starting" | "running" | "error"
    ros_connected = Signal(bool)       # rclpy context up / down
    status_message = Signal(str)       # transient message for the status bar
    #: Console line: (source, text). Sources: "python", "app", "rosout",
    #: "processor", "error". Emitted from any thread (queued to the GUI).
    log_line = Signal(str, str)
