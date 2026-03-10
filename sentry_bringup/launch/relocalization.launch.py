import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():

  config_path = os.path.join(
      get_package_share_directory('sentry_bringup'), 'params')
  ekf_config = os.path.join(
      get_package_share_directory('cmd_chassis'), 'config', 'ekf_imu.yaml')
  
  # 读取SLAM算法选择配置
  slam_selector_path = os.path.join(config_path, 'slam_selector.yaml')
  with open(slam_selector_path, 'r') as f:
    slam_config = yaml.safe_load(f)
    slam_algorithm = slam_config.get('slam_algorithm', 'fast_lio') 
  
  twist2chassis_cmd_node=Node(
    package='cmd_chassis',
    executable='twist2chassis_cmd',
    output='screen',
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

  rot_imu=Node(
    package='cmd_chassis',
    executable='rot_imu',
    output='screen'
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
 
  sentry_description = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sentry_description'), 'launch', 'view_model.launch.py')])
  )

  # mid360
  mid360_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
          get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')])
  )

  # icp relocalization
  map_odom_trans = Node(
      package='icp_relocalization',
      executable='transform_publisher',
      name='transform_publisher',
      output='screen'
  )

  icp_node = Node(
      package='icp_relocalization',
      executable='icp_node',
      name='icp_node',
      output='screen',
      parameters=[
          # --- Blue ---
          # {'initial_x':14.16},
          # {'initial_y':5.35},
          # {'initial_z':0.0},
          # {'initial_a':3.14},

          # --- Red ---
          {'initial_x':0.0},
          {'initial_y':0.0},
          {'initial_z':0.0},
          {'initial_a':0.0},

          {'map_voxel_leaf_size':0.5},
          {'cloud_voxel_leaf_size':0.3},
          {'map_frame_id':'map'},
          {'solver_max_iter':100},
          {'max_correspondence_distance':0.1},
          {'RANSAC_outlier_rejection_threshold':0.5},
          # {'map_path':'/home/sentry_ws/src/sentry_bringup/maps/CC#0.pcd'},
          {'map_path':'/home/lab/sentry_ws/test.pcd'},
          {'fitness_score_thre':0.9}, # 是最近点距离的平均值，越小越严格
          {'converged_count_thre':40}, # pcl pub at 20 hz, 2s
          {'pcl_type':'livox'},  # 订阅原始雷达数据
          # 外参矫正 - 45度倾斜安装
          {'extrinsic_T': [-0.011, -0.02329, 0.04412]},
          {'extrinsic_R': [1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0]},
      ],
  )
  
  # SLAM节点配置 - 根据slam_selector.yaml动态选择
  if slam_algorithm == 'point_lio':
    slam_param = os.path.join(config_path, 'point_lio_relocalization_param.yaml')
    slam_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        parameters=[slam_param],
        output='screen',
        remappings=[('/aft_mapped_to_init','/state_estimation')]
    )
    # Point-LIO使用camera_init作为世界坐标系，需要添加map->camera_init的静态TF
    map_to_camera_init_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init']
    )
  else:  # fast_lio
    slam_param = os.path.join(config_path, 'fast_lio_relocalization_param.yaml')
    slam_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[slam_param],
        output='screen',
        remappings=[('/Odometry','/state_estimation')]
    )
    map_to_camera_init_tf = None  # FAST-LIO不需要此TF
        
  rviz_config_file = os.path.join(
    get_package_share_directory('sentry_bringup'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'warn'],
    output='screen'
  )

  delayed_start_lio = TimerAction(
    period=5.0,
    actions=[
      icp_node,
      slam_node
    ]
  )

  ld = LaunchDescription()

  ld.add_action(twist2chassis_cmd_node)
  ld.add_action(fake_joint_node)
  ld.add_action(rot_imu)
  ld.add_action(ekf_imu_fusion)
  ld.add_action(twist_transformer_node)
  ld.add_action(sentry_description)
  ld.add_action(mid360_node)
  ld.add_action(map_odom_trans)
  ld.add_action(start_rviz)
  ld.add_action(delayed_start_lio)
  
  # 仅当使用Point-LIO时添加map->camera_init的TF
  if map_to_camera_init_tf is not None:
    ld.add_action(map_to_camera_init_tf)

  return ld