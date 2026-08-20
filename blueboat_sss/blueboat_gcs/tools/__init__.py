"""Verbatim duplicates of the team's working conversion tools.

``svlog_to_rosbag.py`` (+ its flat-import dependency ``svlog_helper.py``)
are byte-identical copies of the repo versions — they are known to work
and are NOT modified here. The replay window's "Save as rosbag" wrapper
adds this directory to sys.path and drives the module's own Converter /
walk_packets / setup_topics, replicating its main() (mcap storage,
rename-if-exists) with a GUI progress dialog. Requires a sourced ROS 2
environment (rclpy, rosbag2_py, blueboat_interfaces, mavros_msgs,
geographic_msgs); without one the wrapper shows an explanatory error.

To update: re-copy the files from the robot repo, nothing else.
"""
