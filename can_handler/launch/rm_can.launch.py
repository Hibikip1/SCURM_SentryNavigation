from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_handler',
            executable='rm_can',
            name='rm_can',
            output='screen',
            parameters=[{
                'cmd_vel_topic': '/chassis_cmd',
                'chassis_cmd_id': 0x520,
                'mode_switch_id': 0x203,
                'can_device_index': 0,
                'referee_ids': [0x301, 0x302, 0x303]
            }],
            # 设置日志级别
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
