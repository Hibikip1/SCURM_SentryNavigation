#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory("sentry_bringup")

    # 读取模式配置
    mode_config_path = os.path.join(bringup_dir, "params", "system_mode.yaml")
    with open(mode_config_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    mode = cfg.get("mode", "sim")

    actions = []

    # 仿真模式下启动 Gazebo 仿真环境
    if mode == "sim":
        sim_pkg_dir = get_package_share_directory("rmu_gazebo_simulator")
        sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg_dir, "launch", "bringup_sim.launch.py")
            )
        )
        actions.append(sim_launch)
    else:
        # 实机模式：这里预留，将来可以 Include 硬件 bringup 的 launch
        pass

    # 无论实机 / 仿真，都复用原有的建图与导航决策流程
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "mapping.launch.py")
        )
    )

    all_in_one_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "bringup_all_in_one.launch.py")
        )
    )

    actions.append(mapping_launch)
    actions.append(all_in_one_launch)

    return LaunchDescription(actions)
