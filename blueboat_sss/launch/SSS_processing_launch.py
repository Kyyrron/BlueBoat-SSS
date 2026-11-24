"""Launch file for the SSS *processing* pipeline — the one the GCS starts.

What it starts by default
-------------------------
* ``sss_processor_node.py`` only.

What it starts only on request
------------------------------
* ``sss_node.py`` — with ``with_acquisition:=True``. It stays off by default
  because the same launch serves simulated runs, where the simulator's
  ``sss_sim_node`` already is ``/side_scan_sonar`` and a second driver must
  not start. The GCS passes the flag when its toolbar selector says
  "Sonar: Real" (everything runs on the operator laptop; the driver reaches
  the two Omniscan units over the BlueBoat Base Station WiFi). Pinging is
  controlled at runtime by the GCS publishing ``/side_scan_sonar/ping/enable``
  (START = true, STOP = false). ``will_use_rosbag:=True`` overrides it: a
  rosbag replay never starts the driver. Both flags are evaluated at launch
  time as substitutions — fixed 2026-09-15; before that the Python
  truthiness test made ``with_acquisition:=True`` a silent no-op.

Usage
-----
    ros2 launch blueboat_sss SSS_processing_launch.py
    ros2 launch blueboat_sss SSS_processing_launch.py with_acquisition:=True
    ros2 launch blueboat_sss SSS_processing_launch.py with_acquisition:=True \
        range_length_mm:=30000
    ros2 launch blueboat_sss SSS_processing_launch.py will_use_rosbag:=True
    ros2 launch blueboat_sss SSS_processing_launch.py will_use_rosbag:=True \
        transducer_x_offset_m:=0.0

Transducer fore/aft arm (2026-11-15). ``transducer_x_offset_m`` (default
-0.345, + forward of the /blueboat/odom pose) is the processor's parameter
of the same name: the value it publishes in every
``ProcessedSSSPing.transducer_x_offset``, from which the GCS (and any other
consumer) places the ping. The default is ``sss_processor_node``'s own
``TRANSDUCER_X_OFFSET_M`` = the simulator's ``model.mount_x_m`` (adopted, not
measured). ``:=0.0`` re-processes a recording as it was placed before
2026-11-15.

Clock (2026-11-07). ``SimpleLauncher(use_sim_time='auto')``: when a
``/clock`` publisher exists at launch (a simulation graph -- Gazebo), every
node here gets ``use_sim_time:=True``, so the processor judges "the stream
stopped" and its rate watchdog on sim time and a simulation slowed below
real time (the ray-cast sonar) neither tears rows nor warns at the wall
pace. A real-robot graph has no ``/clock`` publisher (mavros runs on
``use_sim_time`` with none), so nothing changes there.

This file and ``SSS_simple_launch.py`` are the two launch files in the
package; both are installed by ``install(DIRECTORY launch ...)`` in
CMakeLists.txt.

Note on acquisition settings (see docs/SONARVIEW_SVLOG_ANALYSIS.md): the
defaults below are sss_node.py's own and SonarView's — 20 m, 50 ms per ping
(20 pings/s, SonarView's command; 0 would free-run the device at 2R/c +
~2.3 ms, 34.5 Hz at 20 m), device auto-gain, 1200 bins (16.7 mm range
sampling, the device maximum) and pulse 0.00125, which keeps the transmit
pulse on the device's 44.3 us floor up to ~27 m (measured law
pulse = max(44.3 us, pulse_len_percent x 2R/c); 33 mm true range
resolution). The range is sized from water depth (~4x the deepest expected)
rather than from the area to cover; project_synthesis.md 8.5 sets 20 m per
side for every survey condition (owner, 2026-10-20).
"""

from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher(use_sim_time='auto')

    # Skip live acquisition entirely when replaying a rosbag.
    sl.declare_arg('will_use_rosbag', default_value=False)
    # Also start the acquisition driver (real-sonar runs; the GCS passes it
    # when its toolbar selector says "Sonar: Real").
    sl.declare_arg('with_acquisition', default_value=False)

    # ---- sss_node parameters (only used when with_acquisition:=True) --------
    sl_range_start_mm    = sl.declare_arg('range_start_mm',    default_value=0)
    sl_range_length_mm   = sl.declare_arg('range_length_mm',   default_value=20000)
    sl_msec_per_ping     = sl.declare_arg('msec_per_ping',     default_value=50)
    sl_gain_index        = sl.declare_arg('gain_index',        default_value=-1)
    sl_num_results       = sl.declare_arg('num_results',       default_value=1200)
    sl_pulse_len_percent = sl.declare_arg('pulse_len_percent', default_value=0.00125)

    # ---- sss_processor_node parameter ---------------------------------------
    # Fore/aft arm of the transducers [m, + forward]; = the node's own
    # TRANSDUCER_X_OFFSET_M (tests/test_fore_aft_arm.py holds them equal).
    sl_transducer_x_offset_m = sl.declare_arg('transducer_x_offset_m',
                                              default_value=-0.345)

    # ---- acquisition (optional, off by default) -----------------------------
    # Launch-time conditions, one per group (simple_launch evaluates
    # if_arg/unless_arg as substitutions when the description runs).
    # The previous `if sl.arg('with_acquisition') and not sl.arg(...)` was
    # Python truthiness on two LaunchConfiguration objects at file-eval
    # time — always truthy, so `not ...` was always False and
    # with_acquisition:=True could never start the driver.
    with sl.group(unless_arg='will_use_rosbag'):
        with sl.group(if_arg='with_acquisition'):
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
            output='screen',
            parameters={
                'transducer_x_offset_m': sl_transducer_x_offset_m,
            })

    return sl.launch_description()
