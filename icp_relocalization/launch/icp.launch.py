from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='icp_relocalization',
            executable='transform_publisher',
            name='transform_publisher',
            output='screen'
        ),
        Node(
            package='icp_relocalization',
            executable='icp_node',
            name='icp_node',
            output='screen',
            parameters=[
                {'initial_x':0.0},
                {'initial_y':0.0},
                {'initial_z':0.1},
                {'initial_a':-2.2},
                # {'initial_roll':0.0},
                # {'initial_pitch':0.0},
                # {'initial_yaw':0.0},
                {'map_voxel_leaf_size':0.2},
                {'cloud_voxel_leaf_size':0.2},
                {'map_frame_id':'map'},
                {'solver_max_iter':30},
                {'correspondence_levels':[1.0, 0.5, 0.25]},
                {'max_correspondence_distance':0.5},
                {'RANSAC_outlier_rejection_threshold':1.0},
                {'map_path':'/home/lab/sentry_ws/src/sentry_bringup/maps/GlobalMap.pcd'},
                # 判定改用 内点率 + 内点 RMSE，fitness_score_thre 已废弃
                {'blind':0.5},
                {'inlier_dist':0.2},
                {'min_inlier_ratio':0.65},
                {'max_rmse':0.15},
                {'max_median_dist':0.15},
                {'converged_count_thre':5},
                {'pcl_type':'livox'},
            ],
        )
    ])
