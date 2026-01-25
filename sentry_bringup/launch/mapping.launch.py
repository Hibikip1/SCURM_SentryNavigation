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
  
  # 读取系统模式配置（real / sim）
  system_mode_path = os.path.join(config_path, 'system_mode.yaml')
  system_mode = 'real'
  use_sim_time = False
  if os.path.exists(system_mode_path):
    with open(system_mode_path, 'r') as f:
      mode_cfg = yaml.safe_load(f) or {}
      system_mode = mode_cfg.get('mode', 'real')
      use_sim_time = (system_mode == 'sim')
  
  print(f"[mapping.launch.py] 模式: {system_mode}, use_sim_time: {use_sim_time}")
  
  # 读取SLAM算法选择配置
  slam_selector_path = os.path.join(config_path, 'slam_selector.yaml')
  with open(slam_selector_path, 'r') as f:
    slam_config = yaml.safe_load(f)
    slam_algorithm = slam_config.get('slam_algorithm', 'fast_lio') 
  
  twist2chassis_cmd_node=Node(
    package='cmd_chassis',
    executable='twist2chassis_cmd',
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
  )
  
  fake_joint_node=Node(
    package='cmd_chassis',
    executable='fake_joint',
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
  )
  
  twist_transformer_node=Node(
    package='cmd_chassis',
    executable='twist_transformer',
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
  )

  # 尝试加载 IMU bias 校准文件
  imu_bias_file = os.path.join(config_path, 'imu_bias_calibration.yaml')
  imu_bias_params = {}
  if os.path.exists(imu_bias_file):
    with open(imu_bias_file, 'r') as f:
      imu_bias_config = yaml.safe_load(f)
      if 'imu_bias_calibration' in imu_bias_config:
        bias = imu_bias_config['imu_bias_calibration']
        imu_bias_params = {
          'enable_bias_compensation': True,
          'gyro_bias_x': bias.get('gyro_bias_x', 0.0),
          'gyro_bias_y': bias.get('gyro_bias_y', 0.0),
          'gyro_bias_z': bias.get('gyro_bias_z', 0.0),
          'acc_bias_x': bias.get('acc_bias_x', 0.0),
          'acc_bias_y': bias.get('acc_bias_y', 0.0),
          'acc_bias_z': bias.get('acc_bias_z', 0.0),
        }
        print(f"[mapping.launch.py] 已加载 IMU bias 校准: {imu_bias_file}")
  else:
    imu_bias_params = {'enable_bias_compensation': False}
    print(f"[mapping.launch.py] 未找到 IMU bias 校准文件，bias 补偿已禁用")

  # 合并 IMU bias 参数和 use_sim_time
  rot_imu_params = imu_bias_params.copy()
  rot_imu_params['use_sim_time'] = use_sim_time
  
  rot_imu=Node(
    package='cmd_chassis',
    executable='rot_imu',
    parameters=[rot_imu_params],
    output='screen'
  )

  sentry_description = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sentry_description'), 'launch', 'view_model.launch.py')])
  )

  # mid360 真实雷达驱动：仅在 real 模式下启动
  mid360_node = None
  if system_mode == 'real':
    mid360_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')])
    )
  
  # SLAM节点配置 - 根据slam_selector.yaml动态选择
  if slam_algorithm == 'point_lio':
    slam_param = os.path.join(config_path, 'point_lio_mapping_param.yaml')
    slam_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        parameters=[slam_param, {'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[('/aft_mapped_to_init','/state_estimation')]
    )
    # Point-LIO现在使用odom作为世界坐标系，与FAST-LIO一致
    # 使用专用的octomap启动文件（不发布map->odom TF）
    start_octomap_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sentry_bringup'), 'launch', 'octomap_server_pointlio.launch.py')])
    )
  else:  # fast_lio
    slam_param = os.path.join(config_path, 'fast_lio_mapping_param.yaml')
    slam_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[slam_param, {'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[('/Odometry','/state_estimation')]
    )
    # FAST-LIO使用原有的octomap启动文件
    start_octomap_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sentry_bringup'), 'launch', 'octomap_server_intensity.launch.py')])
    )
        
  rviz_config_file = os.path.join(
    get_package_share_directory('sentry_bringup'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'warn'],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
  )

  delayed_start_mapping = TimerAction(
    period=12.0,                                    # 增加到12秒确保IMU充分初始化
    actions=[slam_node, start_octomap_server]
  )

  ld = LaunchDescription()

  ld.add_action(twist2chassis_cmd_node)
  ld.add_action(fake_joint_node)
  ld.add_action(twist_transformer_node)
  ld.add_action(rot_imu)
  ld.add_action(sentry_description)
  if mid360_node is not None:
    ld.add_action(mid360_node)
  ld.add_action(start_rviz)
  ld.add_action(delayed_start_mapping)

  return ld