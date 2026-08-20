"""Launch file for the SSS *processing* pipeline — the one the GCS starts.

What it starts by default
-------------------------
* ``sss_processor_node.py`` only.

What it deliberately does NOT start
-----------------------------------
* ``sss_node.py`` — by default, because the acquisition driver normally
  runs on the robot and is controlled at runtime by the GCS publishing
  ``/side_scan_sonar/ping/enable`` (START = true, STOP = false). Pass
  ``with_acquisition:=True`` to start it here as well when everything
  runs on one machine.
  --- we start this manually 

Usage
-----
    ros2 launch blueboat_sss SSS_processing_launch.py
    ros2 launch blueboat_sss SSS_processing_launch.py with_acquisition:=True
    ros2 launch blueboat_sss SSS_processing_launch.py with_acquisition:=True \
        range_length_mm:=25000 num_results:=1200
    ros2 launch blueboat_sss SSS_processing_launch.py will_use_rosbag:=True

Install alongside the existing ``SSS_launch.py`` in the blueboat_sss
package (add it to the launch install rule in CMakeLists.txt / setup.py).

Note on acquisition settings (see docs/SONARVIEW_SVLOG_ANALYSIS.md): the
Cerulean harbour reference log uses a 25.4 m range with 1200 samples per
ping at 20 Hz, which gives both a wider swath and finer range sampling
than our 15 m / 600 defaults. The defaults below are left unchanged so
this file does not silently alter field behaviour; override them on the
command line when you want to reproduce that configuration.
"""

from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    # Skip live acquisition entirely when replaying a rosbag.
    sl.declare_arg('will_use_rosbag', default_value=False)
    # Also start the acquisition driver here (single-machine setups).
    sl.declare_arg('with_acquisition', default_value=False)

    # ---- sss_node parameters (only used when with_acquisition:=True) --------
    sl_range_start_mm    = sl.declare_arg('range_start_mm',    default_value=0)
    sl_range_length_mm   = sl.declare_arg('range_length_mm',   default_value=30000)
    sl_msec_per_ping     = sl.declare_arg('msec_per_ping',     default_value=0)
    sl_gain_index        = sl.declare_arg('gain_index',        default_value=-1)
    sl_num_results       = sl.declare_arg('num_results',       default_value=600)
    sl_pulse_len_percent = sl.declare_arg('pulse_len_percent', default_value=0.002)

    # ---- acquisition (optional, off by default) -----------------------------
    if sl.arg('with_acquisition') and not sl.arg('will_use_rosbag'):
        sl.node('blueboat_sss', 'sss_node.py',
                name='side_scan_sonar',
                output='screen',
                parameters={
                    'range_start_mm':    sl_range_start_mm,
                    'range_length_mm':   sl_range_length_mm,
                    'msec_per_ping':     sl_msec_per_ping,
                    'gain_index':        sl_gain_index,
                    'num_results':       sl_num_results,
                    'pulse_len_percent': sl_pulse_len_percent,
                })

    # ---- processing (always) ------------------------------------------------
    # Publishes /sss_processor/processed, consumed by the GCS; obeys
    # log/enable for .svlog recording (Record ON/OFF in the GCS).
    sl.node('blueboat_sss', 'sss_processor_node.py',
            name='sss_processor',
            output='screen')

    return sl.launch_description()
