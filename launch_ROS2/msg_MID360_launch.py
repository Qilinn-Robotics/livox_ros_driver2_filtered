import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

# Embedded self-filter for the real Ranger Mini V3 + IMPS setup.
# The driver still publishes /livox/lidar, but self/body points are removed
# before FAST-LIO receives them.
self_filter_box_filters = [
    "base_body:0.0,0.0,-0.10,0.0,0.0,0.0,0.50,0.35,0.20",
    "body_top_deck:0.0,0.0,0.05,0.0,0.0,0.0,0.72,0.50,0.08",
    "body_upper_self_envelope:0.0,0.0,0.28,0.0,0.0,0.0,0.72,0.50,0.46",
    "low_imp_vertical:-0.21,0.11,0.20,0.0,0.0,0.0,0.04,0.04,0.30",
    "low_imp_horizontal:-0.21,0.33,0.33,0.0,0.0,0.0,0.04,0.40,0.04",
    "high_imp_vertical:-0.21,-0.11,0.35,0.0,0.0,0.0,0.04,0.04,0.60",
    "high_imp_horizontal:-0.21,-0.38,0.63,0.0,0.0,0.0,0.04,0.50,0.04",
    "low_imp_ray_guard:0.0,0.055,0.28,0.0,0.0,2.885,0.58,0.12,0.50",
    "high_imp_ray_guard:0.0,-0.055,0.28,0.0,0.0,-2.885,0.58,0.12,0.50",
]

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
    {"force_host_timestamps": True},
    {"self_filter_enabled": True},
    {"self_filter_box_filters": self_filter_box_filters},
    {"self_filter_box_padding": 0.08},
    {"self_filter_lidar_to_base_x": 0.21},
    {"self_filter_lidar_to_base_y": 0.0},
    {"self_filter_lidar_to_base_z": 0.13},
    {"self_filter_lidar_to_base_roll": 0.0},
    {"self_filter_lidar_to_base_pitch": 0.0},
    {"self_filter_lidar_to_base_yaw": 1.5708},
    {"self_filter_front_crop_enabled": False},
    {"self_filter_enforce_monotonic_timestamps": True},
    {"self_filter_drop_stale_stamps": False},
    {"self_filter_max_stamp_age_s": 1.0},
    {"self_filter_stamp_regression_tolerance_s": 0.02},
    {"self_filter_timebase_regression_tolerance_s": 0.02},
    {"self_filter_min_input_points": 1000},
    {"self_filter_min_output_points": 1000},
    {"self_filter_stats_log_period_s": 2.0},
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([
        livox_driver,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
