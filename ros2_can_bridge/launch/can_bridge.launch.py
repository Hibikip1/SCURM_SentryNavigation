from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ros2_can_bridge')
    
    # Define the path to the config file
    config_file = os.path.join(pkg_dir, 'config', 'can_config.yaml')
    
    # Declare the launch arguments
    can_interface = LaunchConfiguration('can_interface')
    declare_can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface to use (e.g., can0, vcan0)'
    )
    
    # Define the node
    can_bridge_node = Node(
        package='ros2_can_bridge',
        executable='can_bridge_node',
        name='can_bridge_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('cmd_vel_in_yaw', '/cmd_vel_in_yaw'),
            ('shoot_cmd', '/shoot_cmd'),
            ('game_state', '/game_state'),
            ('odom', '/odom'),
            ('imu/data', '/imu/data')
        ]
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_can_interface,
        can_bridge_node
    ])
