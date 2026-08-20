"""Launch file for the side scan sonar node.

Standalone usage:
    ros2 launch blueboat_sss SSS_launch.py
    ros2 launch blueboat_sss SSS_launch.py will_use_rosbag:=True
    ros2 launch blueboat_sss SSS_launch.py range_length_mm:=50000 gain_index:=4

    
Nodes started by this launch file:
- sss_node.py: Acquires sonar data from the device and publishes pre-processed pings.
- sss_processor_node.py: Subscribes to pre-processed pings, processes them to allow mosaic generation downstream, and publishes these processed messages.
- processed_sss_listener.py: Subscribes to the processed messages, builds the mosaic in real-time, and live-visualizes it using Matplotlib. Also tracks depth and boat trajectory over time.

"""

from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    #sl_will_use_rosbag = sl.declare_arg('will_use_rosbag', default_value=False)

    # ---- sss_node : Acquisition (re-read on every ping enable) ----------------------
    sl_range_start_mm    = sl.declare_arg('range_start_mm',    default_value=0)
    sl_range_length_mm   = sl.declare_arg('range_length_mm',   default_value=30000)
    sl_msec_per_ping     = sl.declare_arg('msec_per_ping',     default_value=0)
    sl_gain_index        = sl.declare_arg('gain_index',        default_value=-1)
    sl_num_results       = sl.declare_arg('num_results',       default_value=600)
    sl_pulse_len_percent = sl.declare_arg('pulse_len_percent', default_value=0.002)

    #if not sl.arg('will_use_rosbag'):
    sl.node('blueboat_sss', 'sss_node.py',
            name='side_scan_sonar',
            output='screen',
            parameters={
                'range_start_mm':     sl_range_start_mm,
                'range_length_mm':    sl_range_length_mm,
                'msec_per_ping':      sl_msec_per_ping,
                'gain_index':         sl_gain_index,
                'num_results':        sl_num_results,
                'pulse_len_percent':  sl_pulse_len_percent,
            })

    '''sl.node('blueboat_sss', 'sss_processor_node.py',
            name='sss_processor',
            output='screen',
            )
    
    sl.node('blueboat_sss', 'processed_sss_listener.py',
            name='processed_sss_listener',
            output='screen',
            )'''

    return sl.launch_description()
