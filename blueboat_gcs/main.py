#!/usr/bin/env python3
"""BlueBoat GCS entry point.

Usage
-----
    python -m blueboat_gcs.main [--config path.yaml] [--sim]

``--sim`` runs the built-in simulator instead of ROS — full GUI on any
laptop, no boat, no ROS installation required.
"""

from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path
from typing import Optional

from PySide6.QtWidgets import QApplication

from .config.settings import load_config
from .core.mosaic_service import MosaicService
from .core.signals import AppSignals
from .gui.main_window import MainWindow
from .gui.theme import STYLESHEET


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="BlueBoat GCS")
    parser.add_argument("--config", type=Path, default=None,
                        help="YAML config overriding config/default.yaml")
    parser.add_argument("--sim", action="store_true",
                        help="run the built-in simulator (no ROS needed)")
    args = parser.parse_args(argv)

    config = load_config(args.config)
    app = QApplication(sys.argv)
    app.setApplicationName("BlueBoat GCS")
    app.setStyleSheet(STYLESHEET)
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # Ctrl-C closes the app

    signals = AppSignals()
    # Route print()/logging into the embedded console before anything
    # else runs, so startup output is captured too.
    from .core import logging_bus
    logging_bus.install(signals)
    mosaic_service = MosaicService(config)

    ros_manager = None
    if args.sim:
        from .sim.simulator import Simulator
        acquisition = Simulator(config, signals)
    else:
        from .ros.pipeline_launcher import PipelineLauncher
        from .ros.ros_manager import RosManager
        ros_manager = RosManager(config, signals)
        acquisition = PipelineLauncher(config.pipeline, ros_manager, signals)
        if ros_manager.start() and ros_manager.node is not None:
            # Listener construction = subscription registration.
            from .ros.detections_listener import DetectionsListener
            from .ros.path_listener import PathListener
            from .ros.pinger_listener import PingerListener
            from .ros.sonar_listener import SonarListener
            from .ros.telemetry_listener import TelemetryListener
            node = ros_manager.node
            SonarListener(node, signals, config.topics.processed_ping)
            TelemetryListener(node, signals, config.topics)
            DetectionsListener(node, signals, config.topics.detections)
            PingerListener(node, signals, config.topics.pinger)
            PathListener(node, signals, config.topics.planned_path)

    window = MainWindow(config, signals, mosaic_service, acquisition)
    window.show()
    # Acquisition lifecycle: the processing pipeline is launched
    # automatically at startup (sss_processor_node in field mode; the
    # simulator's telemetry in --sim). Visualization layers stay off and
    # no recording session is active — START enables live acquisition.
    acquisition.start()
    code = app.exec()

    if ros_manager is not None:
        ros_manager.shutdown()
    from .core import logging_bus
    logging_bus.uninstall()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
