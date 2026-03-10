#!/usr/bin/env python3
# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('sentry_bringup')
    ekf_config = os.path.join(
        get_package_share_directory('cmd_chassis'), 'config', 'ekf_imu.yaml')
    ws_root = os.path.abspath(os.path.join(bringup_dir, '..', '..', '..', '..'))
    non_relocalization_map_link = os.path.join(ws_root, 'non_relocalization_map.yaml')
    nav2_params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')

    non_relocalization_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'yaml_filename': non_relocalization_map_link,
        },
        convert_types=True,
    )

    fake_joint_node=Node(
        package='cmd_chassis',
        executable='fake_joint',
        output='screen'
    )
    
    twist_transformer_node=Node(
        package='cmd_chassis',
        executable='twist_transformer',
        output='screen',
            arguments=['--ros-args', '-p', 'chassis_frame:=chassis_link'],
    )

    ekf_imu_fusion = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_imu_fusion',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/imu/odometry')
        ]
    )

    map_odom_trans = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['--frame-id', 'map','--child-frame-id', 'odom'],
    output='screen'
    )
    
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir,'launch','mapping.launch.py'
            )
        )
    )
    
    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir,'launch','navigation.launch.py'
            )
        ),
        launch_arguments={
            'params_file': non_relocalization_params,
        }.items()
    )

    start_decision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rm_decision_cpp'),'launch','run.launch.py'
            )
        )
    )

    start_control_panel = Node(
        package='control_panel',
        executable='control_panel',
        output='screen'
    )

    delayed_start_navigation = TimerAction(
        period=10.0,
        actions=[
            # start_slope_and_stair_detection,
            start_navigation,
            # start_control_panel
        ]
    )

    delayed_start_decision = TimerAction(
        period=15.0,
        actions=[
            start_decision
        ]
    )
    
    ld = LaunchDescription()

    ld.add_action(fake_joint_node)
    ld.add_action(ekf_imu_fusion)
    ld.add_action(twist_transformer_node)
    ld.add_action(map_odom_trans)
    ld.add_action(mapping)
    ld.add_action(delayed_start_navigation)
    ld.add_action(delayed_start_decision)  # 不用自动决策的时候这个不用开

    return ld
