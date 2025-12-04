from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotcontrol_can',
            executable='robot_can_node',
            name='robot_can',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/cmd_vel_in_yaw'
            }]
        )
    ])
