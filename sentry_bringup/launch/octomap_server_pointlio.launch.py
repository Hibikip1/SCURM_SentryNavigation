import os.path

from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Point-LIO使用odom作为世界坐标系
    # 需要发布map->odom的静态TF作为全局坐标系原点
    
    # 发布 map -> odom 静态TF
    odom_map_trans = Node(
        name="odom_map_trans",
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='--frame-id map --child-frame-id odom --qx 0.0 --qw 1.0'.split(' '),
        output='screen'
    )
    
    # terrain analysis
    terrain_analysis_ext = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
        get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
        ),
    )
    
    # exchange intensity and height filed of terrain analysis
    exchange_filed=Node(
        package='terrain_analysis',
        executable='exchangeField',
        output='screen',
        remappings=[('/input_topic', '/terrain_map_ext'),
                        ('/output_topic', '/terrain_map_ext_exchanged')] 
    )
    
    # transform terrain_map_ext to sensor frame
    sensor_scan_generation = Node(
        package='sensor_scan_generation',
        executable='sensorScanGeneration',
        output='screen',
        remappings=[('/registered_scan', '/terrain_map_ext_exchanged'),
                        ('/sensor_scan', '/terrain_map_at_scan')]
    )
    
    # build octomap use terrain analysis result
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        output='screen',
        parameters=[{
            "frame_id": "map",
            "base_frame_id": "sensor_at_scan",
            "point_cloud_min_z":0.15,
            "filter_speckles":True,
            "filter_ground_plane":False,
            "resolution": 0.1,
            "latch": True,
        }],
        remappings=[('/cloud_in', '/terrain_map_at_scan')]
    )
            
    ld = LaunchDescription()
    # 添加map->odom TF
    ld.add_action(odom_map_trans)
    ld.add_action(terrain_analysis_ext)
    ld.add_action(exchange_filed)
    ld.add_action(sensor_scan_generation)
    ld.add_action(octomap_server_node)

    return ld
