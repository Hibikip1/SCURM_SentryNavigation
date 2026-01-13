#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取配置文件路径
    pkg_share = get_package_share_directory('cmd_chassis')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_imu.yaml')
    
    return LaunchDescription([
        # rot_imu 节点：转发/预处理IMU数据
        Node(
            package='cmd_chassis',
            executable='rot_imu',
            name='rot_imu_node',
            output='screen',
        ),
        
        # robot_localization EKF 节点：融合IMU得到orientation
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_imu_fusion',
            output='screen',
            parameters=[ekf_config],
            remappings=[
                ('odometry/filtered', '/imu/odometry')  # 输出融合后的里程计
            ]
        ),
        
        # twist_transformer 节点：用EKF输出的yaw做速度变换
        Node(
            package='cmd_chassis',
            executable='twist_transformer',
            name='twist_transformer',
            output='screen'
        )
    ])
