import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Use PointCloud2 output so downstream nodes (e.g. merge_cloud) can subscribe.
    xfer_format = 0  # 0-Pointcloud2, 1-customized pointcloud format
    multi_topic = 1  # 1-One LiDAR one topic
    data_src = 0
    publish_freq = 10.0
    output_type = 0
    frame_id = 'sensor'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    livox_share = get_package_share_directory('livox_ros_driver2')
    user_config_path = os.path.join(livox_share, 'config', 'MID360_config.json')

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
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        respawn=True,
        respawn_delay=1,
        output='screen',
        parameters=livox_ros2_params,
    )

    return LaunchDescription([
        livox_driver,
    ])
