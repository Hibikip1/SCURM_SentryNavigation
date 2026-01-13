#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <deque>
#include <cmath>
#include <vector>
#include <numeric>
#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

// 关键帧结构
struct KeyFrame {
    Eigen::Matrix4f pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    double timestamp;
    double fitness_score;
    int frame_id;
    bool is_degenerate;
};

// 运动观测性状态
struct MotionObservability {
    bool is_degenerate;
    double linear_motion_magnitude;
    double angular_motion_magnitude;
    double observability_score;
};

// IMU数据缓存结构
struct IMUData {
    double timestamp;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d gyro_bias;
};

// 时间同步管理
struct TimeSyncManager {
    std::deque<std::pair<double, double>> time_offset_buffer;
    static constexpr int MAX_TIME_OFFSET_BUFFER_SIZE = 10;
    double estimated_time_offset = 0.0;
    
    void update_offset(double lidar_time, double imu_time) {
        double offset = lidar_time - imu_time;
        time_offset_buffer.push_back({lidar_time, offset});
        if (time_offset_buffer.size() > MAX_TIME_OFFSET_BUFFER_SIZE) {
            time_offset_buffer.pop_front();
        }
        
        // 计算中位数来估计时间偏移
        std::vector<double> offsets;
        for (const auto& p : time_offset_buffer) {
            offsets.push_back(p.second);
        }
        std::sort(offsets.begin(), offsets.end());
        estimated_time_offset = offsets[offsets.size() / 2];
    }
};

class ICPNode : public rclcpp::Node
{
public:
    ICPNode()
        : Node("icp_node")
    {
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_z", 0.0);
        this->declare_parameter("initial_a", 0.0);
        this->declare_parameter("solver_max_iter", 75);
        this->declare_parameter("max_correspondence_distance", 0.1);
        this->declare_parameter("RANSAC_outlier_rejection_threshold", 1.0);
        this->declare_parameter("map_path", "");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("fitness_score_thre", 0.0);
        this->declare_parameter("map_voxel_leaf_size", 0.1);
        this->declare_parameter("cloud_voxel_leaf_size", 0.1);
        this->declare_parameter("converged_count_thre", 20);
        this->declare_parameter("pcl_type","livox");
        this->declare_parameter("extrinsic_T", std::vector<double>());
        this->declare_parameter("extrinsic_R", std::vector<double>());
        // 新增参数：退化检测和关键帧
        this->declare_parameter("enable_degenerate_detection", true);
        this->declare_parameter("enable_keyframe_mechanism", true);
        this->declare_parameter("keyframe_min_translation", 0.1);
        this->declare_parameter("keyframe_min_rotation", 0.05);
        this->declare_parameter("degenerate_motion_threshold", 0.05);
        this->declare_parameter("enable_time_sync", true);
        this->declare_parameter("enable_imu_subscription", false);
        this->declare_parameter("map_weight_factor", 1.0);

        this->get_parameter("initial_x", initial_x);
        this->get_parameter("initial_y", initial_y);
        this->get_parameter("initial_z", initial_z);
        this->get_parameter("initial_a", initial_a);
        this->get_parameter("solver_max_iter", solver_max_iter);
        this->get_parameter("max_correspondence_distance", max_correspondence_distance);
        this->get_parameter("RANSAC_outlier_rejection_threshold", RANSAC_outlier_rejection_threshold);
        this->get_parameter("map_path", map_path);
        this->get_parameter("map_frame_id", map_frame);
        this->get_parameter("fitness_score_thre", fitness_score_thre);
        this->get_parameter("map_voxel_leaf_size", map_voxel_leaf_size);
        this->get_parameter("cloud_voxel_leaf_size", cloud_voxel_leaf_size);
        this->get_parameter("converged_count_thre", converged_count_thre);
        this->get_parameter("pcl_type", pcl_type);
        
        std::vector<double> extrinsic_T_vec, extrinsic_R_vec;
        this->get_parameter("extrinsic_T", extrinsic_T_vec);
        this->get_parameter("extrinsic_R", extrinsic_R_vec);
        
        // 获取新增参数
        this->get_parameter("enable_degenerate_detection", enable_degenerate_detection_);
        this->get_parameter("enable_keyframe_mechanism", enable_keyframe_mechanism_);
        this->get_parameter("keyframe_min_translation", keyframe_min_translation_);
        this->get_parameter("keyframe_min_rotation", keyframe_min_rotation_);
        this->get_parameter("degenerate_motion_threshold", degenerate_motion_threshold_);
        this->get_parameter("enable_time_sync", enable_time_sync_);
        this->get_parameter("enable_imu_subscription", enable_imu_subscription_);
        this->get_parameter("map_weight_factor", map_weight_factor_);
        
        RCLCPP_INFO(this->get_logger(), "Degenerate Detection: %s", 
                    enable_degenerate_detection_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "Keyframe Mechanism: %s", 
                    enable_keyframe_mechanism_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "Time Synchronization: %s", 
                    enable_time_sync_ ? "enabled" : "disabled");
        
        // 设置外参矩阵
        lidar_to_imu_transform_ = Eigen::Matrix4f::Identity();
        if (extrinsic_R_vec.size() == 9 && extrinsic_T_vec.size() == 3) {
            lidar_to_imu_transform_(0, 0) = extrinsic_R_vec[0];
            lidar_to_imu_transform_(0, 1) = extrinsic_R_vec[1];
            lidar_to_imu_transform_(0, 2) = extrinsic_R_vec[2];
            lidar_to_imu_transform_(1, 0) = extrinsic_R_vec[3];
            lidar_to_imu_transform_(1, 1) = extrinsic_R_vec[4];
            lidar_to_imu_transform_(1, 2) = extrinsic_R_vec[5];
            lidar_to_imu_transform_(2, 0) = extrinsic_R_vec[6];
            lidar_to_imu_transform_(2, 1) = extrinsic_R_vec[7];
            lidar_to_imu_transform_(2, 2) = extrinsic_R_vec[8];
            lidar_to_imu_transform_(0, 3) = extrinsic_T_vec[0];
            lidar_to_imu_transform_(1, 3) = extrinsic_T_vec[1];
            lidar_to_imu_transform_(2, 3) = extrinsic_T_vec[2];
            RCLCPP_INFO(this->get_logger(), "Loaded extrinsic parameters");
        } else {
            RCLCPP_WARN(this->get_logger(), "Extrinsic parameters not set, using identity transform");
        }

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_result", 10);
#ifdef USE_LIVOX
        if(pcl_type == "livox")
        {
            lvx_cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 10, std::bind(&ICPNode::lvx_cloud_callback, this, std::placeholders::_1));
        }
        else
        {
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", 10, std::bind(&ICPNode::cloud_callback, this, std::placeholders::_1));
        }
#else
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", 10, std::bind(&ICPNode::cloud_callback, this, std::placeholders::_1));
#endif
        // IMU订阅用于时间同步和可观性检测
        if (enable_imu_subscription_) {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data", 100, std::bind(&ICPNode::imu_callback, this, std::placeholders::_1));
        }
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&ICPNode::pose_callback, this, std::placeholders::_1));
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("prior_map", 10);
        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);
        
        // init guess
        initGuess = Eigen::Matrix4f::Identity();
        last_pose_ = initGuess;
        initGuess(0, 3) = initial_x;
        initGuess(1, 3) = initial_y;
        initGuess(2, 3) = initial_z;
        // You need to convert the quaternion to a rotation matrix and set it to the upper-left 3x3 part of the matrix
        tf2::Quaternion q;
        q.setRPY(0, 0, initial_a);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                initGuess(i, j) = rot_mat[i][j];
            }
        }
        RCLCPP_INFO(this->get_logger(), "Initial guess: \n x: %f, y: %f, z: %f, a: %f", initial_x, initial_y, initial_z, initial_a);
        // Load the target point cloud from a PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *target_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file target.pcd");
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %d data points from target.pcd", target_cloud_->width * target_cloud_->height);

        // downsample the target cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_map;
        sor_map.setInputCloud(target_cloud_);
        sor_map.setLeafSize(map_voxel_leaf_size, map_voxel_leaf_size, map_voxel_leaf_size);
        sor_map.filter(*target_cloud_);
        RCLCPP_INFO(this->get_logger(), "Downsampled target cloud to %d data points", target_cloud_->width * target_cloud_->height);
        // Publish the downsampled target cloud

        pcl::toROSMsg(*target_cloud_, target_cloud_msg);
        target_cloud_msg.header.stamp = this->now();
        target_cloud_msg.header.frame_id = map_frame;
        map_pub_->publish(target_cloud_msg);
    }

private:
    // 现有回调函数
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the incoming point cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        // Downsample the input cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
        sor_scan.setInputCloud(input_cloud);
        sor_scan.setLeafSize(cloud_voxel_leaf_size, cloud_voxel_leaf_size, cloud_voxel_leaf_size);
        sor_scan.filter(*input_cloud);
        RCLCPP_INFO(this->get_logger(), "Downsampled input cloud to %d data points", input_cloud->width * input_cloud->height);

        // 时间同步检查（如果启用IMU）
        if (enable_time_sync_ && enable_imu_subscription_) {
            double lidar_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            sync_time_offset(lidar_time);
        }

        // Apply lidar to IMU extrinsic transform
        pcl::transformPointCloud(*input_cloud, *input_cloud, lidar_to_imu_transform_);

        // Perform ICP alignment
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(input_cloud);
        icp.setInputTarget(target_cloud_);
        icp.setMaximumIterations(solver_max_iter);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance);
        icp.setRANSACOutlierRejectionThreshold(RANSAC_outlier_rejection_threshold);
        
        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        icp.align(final_cloud, initGuess);

        // Get fitness score
        double fitness_score = icp.getFitnessScore();
        frame_count_++;
        
        // 退化运动检测
        MotionObservability motion_obs = detect_degenerate_motion(initGuess, last_pose_);
        bool is_degenerate = enable_degenerate_detection_ && motion_obs.is_degenerate;
        
        RCLCPP_INFO(this->get_logger(), "Frame %d - ICP fitness: %.4f, Linear motion: %.4f, Angular motion: %.4f, Observability: %.2f, Degenerate: %s",
                    frame_count_, fitness_score, motion_obs.linear_motion_magnitude, 
                    motion_obs.angular_motion_magnitude, motion_obs.observability_score,
                    is_degenerate ? "YES" : "NO");

        // 处理退化运动
        if (is_degenerate) {
            RCLCPP_WARN(this->get_logger(), "Degenerate motion detected! Applying robust strategy...");
            handle_degenerate_motion(icp, fitness_score, initGuess);
            // 重新对齐
            icp.align(final_cloud, initGuess);
            fitness_score = icp.getFitnessScore();
        }

        // 应用鲁棒加权
        apply_robust_weighting(icp, fitness_score, is_degenerate);

        if (fitness_score < fitness_score_thre && icp.hasConverged())
        {
            converged_count++;
            RCLCPP_INFO(this->get_logger(), "ICP converged, count: %d/%d", converged_count, converged_count_thre);
            
            Eigen::Matrix4f transformation_result = icp.getFinalTransformation();
            
            // 关键帧机制
            if (enable_keyframe_mechanism_) {
                bool add_keyframe = false;
                if (keyframe_buffer_.empty()) {
                    add_keyframe = true;
                } else {
                    add_keyframe = should_add_keyframe(
                        transformation_result, 
                        keyframe_buffer_.back().pose,
                        fitness_score);
                }
                
                if (add_keyframe) {
                    update_keyframe_buffer(input_cloud, transformation_result, 
                                         msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
                                         fitness_score, is_degenerate);
                    RCLCPP_INFO(this->get_logger(), "Added new keyframe (total: %zu)", keyframe_buffer_.size());
                }
            }
            
            if(converged_count < converged_count_thre)
            {
                RCLCPP_INFO(this->get_logger(), "ICP converged, but not enough count, no pose is published");
                last_pose_ = transformation_result;
                initGuess = transformation_result;
                return;
            }
            
            // Convert the transformation_result result to a PoseWithCovarianceStamped message and publish it
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = map_frame;
            pose_msg.pose.pose.position.x = transformation_result(0, 3);
            pose_msg.pose.pose.position.y = transformation_result(1, 3);
            pose_msg.pose.pose.position.z = transformation_result(2, 3);
            // set orientation
            Eigen::Matrix3f rotation = transformation_result.block<3, 3>(0, 0);
            Eigen::Quaternionf q(rotation);
            pose_msg.pose.pose.orientation.x = q.x();
            pose_msg.pose.pose.orientation.y = q.y();
            pose_msg.pose.pose.orientation.z = q.z();
            pose_msg.pose.pose.orientation.w = q.w();
            publisher_->publish(pose_msg);

            // Transform the input cloud using the ICP result
            pcl::transformPointCloud(*input_cloud, *input_cloud, transformation_result);
            // Publish the transformed input cloud
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
            pcl::toROSMsg(*input_cloud, transformed_cloud_msg);
            transformed_cloud_msg.header.stamp = this->now();
            transformed_cloud_msg.header.frame_id = map_frame;
            transformed_cloud_pub_->publish(transformed_cloud_msg);
            
            last_pose_ = transformation_result;
            rclcpp::shutdown();
        }
        else
        {
            converged_count = 0;
            Eigen::Matrix4f transformation_result = initGuess;
            
            // 即使没有收敛，在非退化场景下也更新初始猜测
            if (!is_degenerate) {
                initGuess = transformation_result;
            } else {
                RCLCPP_WARN(this->get_logger(), "Not updating guess due to degenerate motion");
            }
            
            last_pose_ = transformation_result;
            
            // 记录运动历史用于观察性检测
            pose_history_.push_back(transformation_result);
            if (pose_history_.size() > POSE_HISTORY_SIZE) {
                pose_history_.pop_front();
            }
            
            pcl::transformPointCloud(*input_cloud, *input_cloud, transformation_result);
            // Publish the transformed input cloud
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
            pcl::toROSMsg(*input_cloud, transformed_cloud_msg);
            transformed_cloud_msg.header.stamp = this->now();
            transformed_cloud_msg.header.frame_id = map_frame;
            transformed_cloud_pub_->publish(transformed_cloud_msg);
            RCLCPP_INFO(this->get_logger(), "ICP fitness score is higher than the threshold, no pose is published");
        }
        target_cloud_msg.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg);
    }

#ifdef USE_LIVOX
    void lvx_cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        // Convert the incoming point cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < msg->point_num; i++)
        {
            pcl::PointXYZ point;
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            input_cloud->push_back(point);
        }
        input_cloud->width = input_cloud->size();
        input_cloud->height = 1;

        // Downsample the input cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
        sor_scan.setInputCloud(input_cloud);
        sor_scan.setLeafSize(cloud_voxel_leaf_size, cloud_voxel_leaf_size, cloud_voxel_leaf_size);
        sor_scan.filter(*input_cloud);
        RCLCPP_INFO(this->get_logger(), "Downsampled input cloud to %d data points", input_cloud->width * input_cloud->height);

        // 时间同步检查（如果启用IMU）
        if (enable_time_sync_ && enable_imu_subscription_) {
            double lidar_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            sync_time_offset(lidar_time);
        }

        // Apply lidar to IMU extrinsic transform
        pcl::transformPointCloud(*input_cloud, *input_cloud, lidar_to_imu_transform_);

        // Perform ICP alignment
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(input_cloud);
        icp.setInputTarget(target_cloud_);
        icp.setMaximumIterations(solver_max_iter);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance);
        icp.setRANSACOutlierRejectionThreshold(RANSAC_outlier_rejection_threshold);
        
        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        icp.align(final_cloud, initGuess);

        // Get fitness score
        double fitness_score = icp.getFitnessScore();
        frame_count_++;
        
        // 退化运动检测
        MotionObservability motion_obs = detect_degenerate_motion(initGuess, last_pose_);
        bool is_degenerate = enable_degenerate_detection_ && motion_obs.is_degenerate;
        
        RCLCPP_INFO(this->get_logger(), "Frame %d - ICP fitness: %.4f, Linear motion: %.4f, Angular motion: %.4f, Observability: %.2f, Degenerate: %s",
                    frame_count_, fitness_score, motion_obs.linear_motion_magnitude, 
                    motion_obs.angular_motion_magnitude, motion_obs.observability_score,
                    is_degenerate ? "YES" : "NO");

        // 处理退化运动
        if (is_degenerate) {
            RCLCPP_WARN(this->get_logger(), "Degenerate motion detected! Applying robust strategy...");
            handle_degenerate_motion(icp, fitness_score, initGuess);
            // 重新对齐
            icp.align(final_cloud, initGuess);
            fitness_score = icp.getFitnessScore();
        }

        // 应用鲁棒加权
        apply_robust_weighting(icp, fitness_score, is_degenerate);

        if (icp.hasConverged() && fitness_score < fitness_score_thre && converged_count > converged_count_thre)
        {
            RCLCPP_INFO(this->get_logger(), "ICP converged!!!");
            Eigen::Matrix4f transformation_result = icp.getFinalTransformation();
            
            // 关键帧机制
            if (enable_keyframe_mechanism_) {
                bool add_keyframe = false;
                if (keyframe_buffer_.empty()) {
                    add_keyframe = true;
                } else {
                    add_keyframe = should_add_keyframe(
                        transformation_result, 
                        keyframe_buffer_.back().pose,
                        fitness_score);
                }
                
                if (add_keyframe) {
                    update_keyframe_buffer(input_cloud, transformation_result, 
                                         msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9,
                                         fitness_score, is_degenerate);
                    RCLCPP_INFO(this->get_logger(), "Added new keyframe (total: %zu)", keyframe_buffer_.size());
                }
            }
            
            // Convert the transformation_result result to a PoseWithCovarianceStamped message and publish it
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = map_frame;
            pose_msg.pose.pose.position.x = transformation_result(0, 3);
            pose_msg.pose.pose.position.y = transformation_result(1, 3);
            pose_msg.pose.pose.position.z = transformation_result(2, 3);
            // set orientation
            Eigen::Matrix3f rotation = transformation_result.block<3, 3>(0, 0);
            Eigen::Quaternionf q(rotation);
            pose_msg.pose.pose.orientation.x = q.x();
            pose_msg.pose.pose.orientation.y = q.y();
            pose_msg.pose.pose.orientation.z = q.z();
            pose_msg.pose.pose.orientation.w = q.w();
            publisher_->publish(pose_msg);

            // Transform the input cloud using ICP result
            pcl::transformPointCloud(*input_cloud, *input_cloud, transformation_result);
            // Publish the transformed input cloud for the last time
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
            pcl::toROSMsg(*input_cloud, transformed_cloud_msg);
            transformed_cloud_msg.header.stamp = this->now();
            transformed_cloud_msg.header.frame_id = map_frame;
            transformed_cloud_pub_->publish(transformed_cloud_msg);
            
            last_pose_ = transformation_result;
            rclcpp::shutdown();
        }
        else if(icp.hasConverged() && fitness_score < fitness_score_thre && converged_count <= converged_count_thre)
        {
            converged_count++;
            Eigen::Matrix4f transformation_result = icp.getFinalTransformation();
            
            // 在非退化场景下更新初始猜测
            if (!is_degenerate) {
                initGuess = transformation_result;
            }
            
            last_pose_ = transformation_result;
            
            pcl::transformPointCloud(*input_cloud, *input_cloud, initGuess);
            RCLCPP_INFO(this->get_logger(), "ICP converged with low error, count %d/%d", converged_count, converged_count_thre);
        }
        else if(icp.hasConverged() && fitness_score >= fitness_score_thre)
        {
            converged_count = 0;
            Eigen::Matrix4f transformation_result = icp.getFinalTransformation();
            
            if (!is_degenerate) {
                initGuess = transformation_result;
            }
            
            last_pose_ = transformation_result;
            
            // 记录运动历史
            pose_history_.push_back(transformation_result);
            if (pose_history_.size() > POSE_HISTORY_SIZE) {
                pose_history_.pop_front();
            }
            
            pcl::transformPointCloud(*input_cloud, *input_cloud, initGuess);
            RCLCPP_INFO(this->get_logger(), "ICP converged with high error, no pose is published");
        }
        else // if ICP doesn't converge
        {
            converged_count = 0;
            last_pose_ = initGuess;
            
            pose_history_.push_back(initGuess);
            if (pose_history_.size() > POSE_HISTORY_SIZE) {
                pose_history_.pop_front();
            }
            
            pcl::transformPointCloud(*input_cloud, *input_cloud, initGuess);
            RCLCPP_WARN(this->get_logger(), "ICP doesn't converge, frame %d", frame_count_);
        }

        // Publish the transformed input cloud
        sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
        pcl::toROSMsg(*input_cloud, transformed_cloud_msg);
        transformed_cloud_msg.header.stamp = this->now();
        transformed_cloud_msg.header.frame_id = map_frame;
        transformed_cloud_pub_->publish(transformed_cloud_msg);
        target_cloud_msg.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg);
    }
#endif

    // 前置声明必要的方法供回调使用
    
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Convert the incoming pose to an Eigen matrix
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = msg->pose.pose.position.x;
        initGuess(1, 3) = msg->pose.pose.position.y;
        initGuess(2, 3) = msg->pose.pose.position.z;
        // You need to convert the quaternion to a rotation matrix and set it to the upper-left 3x3 part of the matrix
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                initGuess(i, j) = rot_mat[i][j];
            }
        }
        double r,p,yaw;
        rot_mat.getRPY(r, p, yaw);
        RCLCPP_INFO(this->get_logger(), "Initial guess: \n x: %f, y: %f, z: %f, a: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw);
    }
    
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
        
        IMUData imu_data;
        imu_data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        imu_data.angular_velocity = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        imu_data.linear_acceleration = Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);
        imu_data.gyro_bias = Eigen::Vector3d::Zero();
        
        imu_buffer_.push_back(imu_data);
        
        // 保持缓冲区大小
        if (imu_buffer_.size() > MAX_IMU_BUFFER_SIZE) {
            imu_buffer_.pop_front();
        }
        
        // 监测IMU漂移
        double ang_vel_norm = imu_data.angular_velocity.norm();
        last_imu_angular_velocity_norm_ = ang_vel_norm;
        
        if (ang_vel_norm > 2.0) {
            RCLCPP_WARN(this->get_logger(), "High angular velocity detected: %.3f rad/s", ang_vel_norm);
        }
    }
    
    void sync_time_offset(double lidar_timestamp) {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
        
        if (!imu_buffer_.empty()) {
            // 查找最接近的IMU时间戳
            double closest_imu_time = imu_buffer_.back().timestamp;
            double min_diff = std::abs(lidar_timestamp - closest_imu_time);
            
            for (const auto& imu_data : imu_buffer_) {
                double diff = std::abs(lidar_timestamp - imu_data.timestamp);
                if (diff < min_diff) {
                    min_diff = diff;
                    closest_imu_time = imu_data.timestamp;
                }
            }
            
            // 更新时间偏移估计
            time_sync_manager_.update_offset(lidar_timestamp, closest_imu_time);
            
            if (std::abs(time_sync_manager_.estimated_time_offset) > 0.01) {
                RCLCPP_WARN(this->get_logger(), "Large time offset detected: %.3f s", 
                             time_sync_manager_.estimated_time_offset);
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_{new pcl::PointCloud<pcl::PointXYZ>};

    // 原有成员变量
    Eigen::Matrix4f initGuess;
    Eigen::Matrix4f last_pose_;
    double initial_x, initial_y, initial_z, initial_a;
    int solver_max_iter;
    double max_correspondence_distance, RANSAC_outlier_rejection_threshold;
    std::string map_path, map_frame;
    double fitness_score_thre;
    double map_voxel_leaf_size, cloud_voxel_leaf_size;
    sensor_msgs::msg::PointCloud2 target_cloud_msg;
    int converged_count = 0;
    int converged_count_thre;
    std::string pcl_type;
    Eigen::Matrix4f lidar_to_imu_transform_;
    
    // 新增成员变量：退化检测和关键帧管理
    bool enable_degenerate_detection_ = true;
    bool enable_keyframe_mechanism_ = true;
    double keyframe_min_translation_ = 0.1;
    double keyframe_min_rotation_ = 0.05;
    double degenerate_motion_threshold_ = 0.05;
    bool enable_time_sync_ = true;
    bool enable_imu_subscription_ = false;
    double map_weight_factor_ = 1.0;
    
    // 关键帧缓冲
    std::deque<KeyFrame> keyframe_buffer_;
    static constexpr int MAX_KEYFRAME_BUFFER_SIZE = 5;
    
    // IMU数据缓冲
    std::deque<IMUData> imu_buffer_;
    static constexpr int MAX_IMU_BUFFER_SIZE = 1000;
    
    // 互斥锁保护并发访问
    std::mutex imu_buffer_mutex_;
    std::mutex keyframe_buffer_mutex_;
    
    // 时间同步管理
    TimeSyncManager time_sync_manager_;
    
    // 运动历史用于观察性检测
    std::deque<Eigen::Matrix4f> pose_history_;
    static constexpr int POSE_HISTORY_SIZE = 10;
    
    // 帧计数
    int frame_count_ = 0;
    double last_imu_angular_velocity_norm_ = 0.0;
    
    // 实现优化方法
    MotionObservability detect_degenerate_motion(
        const Eigen::Matrix4f& current_pose, 
        const Eigen::Matrix4f& previous_pose) {
        
        MotionObservability obs;
        
        // 计算位置变化
        Eigen::Vector3f delta_t = current_pose.block<3, 1>(0, 3) - previous_pose.block<3, 1>(0, 3);
        double translation_magnitude = delta_t.norm();
        
        // 计算旋转变化
        Eigen::Matrix3f R_current = current_pose.block<3, 3>(0, 0);
        Eigen::Matrix3f R_previous = previous_pose.block<3, 3>(0, 0);
        Eigen::Matrix3f R_delta = R_current * R_previous.transpose();
        
        // 通过旋转矩阵的迹和角度计算角运动大小
        double cos_angle = (R_delta.trace() - 1.0) / 2.0;
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        double rotation_angle = std::acos(cos_angle);
        
        obs.linear_motion_magnitude = translation_magnitude;
        obs.angular_motion_magnitude = rotation_angle;
        
        // 计算可观性分数：水平面运动 / 总运动
        Eigen::Vector3f xy_motion = delta_t;
        xy_motion.z() = 0;
        double xy_translation = xy_motion.norm();
        
        if (translation_magnitude > 1e-6) {
            obs.observability_score = xy_translation / translation_magnitude;
        } else {
            obs.observability_score = 0.0;
        }
        
        // 判断是否退化
        const double TRANSLATION_THRESHOLD = degenerate_motion_threshold_;
        const double ROTATION_THRESHOLD = 0.2;
        const double OBSERVABILITY_THRESHOLD = 0.3;
        
        obs.is_degenerate = (translation_magnitude < TRANSLATION_THRESHOLD && rotation_angle > ROTATION_THRESHOLD) ||
                            (obs.observability_score < OBSERVABILITY_THRESHOLD && rotation_angle > ROTATION_THRESHOLD);
        
        return obs;
    }
    
    bool should_add_keyframe(
        const Eigen::Matrix4f& current_pose,
        const Eigen::Matrix4f& last_keyframe_pose,
        double fitness_score) {
        
        // 计算与上一关键帧的运动
        Eigen::Vector3f delta_t = current_pose.block<3, 1>(0, 3) - last_keyframe_pose.block<3, 1>(0, 3);
        double translation = delta_t.norm();
        
        // 计算旋转角度
        Eigen::Matrix3f R_current = current_pose.block<3, 3>(0, 0);
        Eigen::Matrix3f R_last = last_keyframe_pose.block<3, 3>(0, 0);
        Eigen::Matrix3f R_delta = R_current * R_last.transpose();
        
        double cos_angle = (R_delta.trace() - 1.0) / 2.0;
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        double rotation = std::acos(cos_angle);
        
        // 如果运动足够大或者质量足够好，添加关键帧
        bool should_add = (translation > keyframe_min_translation_) || 
                          (rotation > keyframe_min_rotation_) ||
                          (fitness_score < 0.05);
        
        return should_add;
    }
    
    void update_keyframe_buffer(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Eigen::Matrix4f& pose,
        double timestamp,
        double fitness_score,
        bool is_degenerate) {
        
        std::lock_guard<std::mutex> lock(keyframe_buffer_mutex_);
        
        KeyFrame kf;
        kf.pose = pose;
        kf.cloud = cloud;
        kf.timestamp = timestamp;
        kf.fitness_score = fitness_score;
        kf.frame_id = frame_count_;
        kf.is_degenerate = is_degenerate;
        
        keyframe_buffer_.push_back(kf);
        
        // 保持缓冲区大小
        if (keyframe_buffer_.size() > MAX_KEYFRAME_BUFFER_SIZE) {
            keyframe_buffer_.pop_front();
        }
    }
    
    void handle_degenerate_motion(
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp,
        double& fitness_score,
        Eigen::Matrix4f& initial_guess) {
        
        // 对于退化运动，使用更严格的参数
        icp.setMaxCorrespondenceDistance(max_correspondence_distance * 0.5);
        icp.setMaximumIterations(100);
        icp.setRANSACOutlierRejectionThreshold(max_correspondence_distance * 0.3);
        
        RCLCPP_DEBUG(this->get_logger(), "Applied degenerate motion handling: reduced distance threshold to %.4f", 
                     max_correspondence_distance * 0.5);
    }
    
    void apply_robust_weighting(
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp,
        double fitness_score,
        bool is_degenerate) {
        
        // 基于fitness score和退化状态调整参数
        double base_distance = max_correspondence_distance;
        
        if (is_degenerate) {
            // 退化场景：更严格的匹配
            icp.setMaxCorrespondenceDistance(base_distance * 0.6);
        } else if (fitness_score > 0.1) {
            // 质量差的匹配：降低权重
            icp.setMaxCorrespondenceDistance(base_distance * 1.2);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}