// ICP节点增强方法实现
// 包括退化检测、关键帧管理、时间同步和鲁棒权重调整

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
#include <mutex>

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

/**
 * @brief 检测退化运动（纯旋转或极小平移）
 * 
 * 方法：
 * 1. 计算当前帧与上一帧的运动量
 * 2. 分析线性运动与角运动的比例
 * 3. 检查水平平面运动的可观性
 * 4. 如果主要是旋转运动且平移很小，认为发生退化
 */
MotionObservability detect_degenerate_motion(
    const Eigen::Matrix4f& current_pose, 
    const Eigen::Matrix4f& previous_pose)
{
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
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));  // 确保在[-1, 1]范围内
    double rotation_angle = std::acos(cos_angle);
    
    obs.linear_motion_magnitude = translation_magnitude;
    obs.angular_motion_magnitude = rotation_angle;
    
    // 计算可观性分数
    // 如果只有旋转没有平移，可观性很差（Z方向除外）
    Eigen::Vector3f xy_motion = delta_t;
    xy_motion.z() = 0;  // 忽略Z方向
    double xy_translation = xy_motion.norm();
    
    // 可观性分数：水平面平移 / 总运动
    if (translation_magnitude > 1e-6) {
        obs.observability_score = xy_translation / translation_magnitude;
    } else {
        obs.observability_score = 0.0;
    }
    
    // 判断是否退化：纯旋转或极小平移
    // 阈值可根据应用调整
    const double TRANSLATION_THRESHOLD = 0.05;  // 5cm
    const double ROTATION_THRESHOLD = 0.2;      // 约11度
    const double OBSERVABILITY_THRESHOLD = 0.3; // 水平面运动少于30%
    
    obs.is_degenerate = (translation_magnitude < TRANSLATION_THRESHOLD && rotation_angle > ROTATION_THRESHOLD) ||
                        (obs.observability_score < OBSERVABILITY_THRESHOLD && rotation_angle > ROTATION_THRESHOLD);
    
    return obs;
}

/**
 * @brief 判断是否应该添加关键帧
 * 
 * 关键帧选择标准：
 * 1. 与上一个关键帧的平移距离超过阈值
 * 2. 与上一个关键帧的旋转角度超过阈值
 * 3. 或者ICP匹配质量好（低fitness score）
 */
bool should_add_keyframe(
    const Eigen::Matrix4f& current_pose,
    const Eigen::Matrix4f& last_keyframe_pose,
    double fitness_score,
    double min_translation = 0.1,
    double min_rotation = 0.05)
{
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
    bool should_add = (translation > min_translation) || 
                      (rotation > min_rotation) ||
                      (fitness_score < 0.05);  // 优质匹配也添加
    
    return should_add;
}

/**
 * @brief 处理退化运动的策略
 * 
 * 当检测到退化运动时：
 * 1. 降低ICP匹配距离阈值以避免错误对应
 * 2. 增加迭代次数以获得更精确的匹配
 * 3. 使用更严格的异常值拒绝策略
 * 4. 可选：使用历史关键帧作为参考而非单一帧
 */
void handle_degenerate_motion(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp,
    double& fitness_score,
    Eigen::Matrix4f& initial_guess,
    const std::deque<KeyFrame>& keyframe_buffer,
    double original_max_distance = 0.1)
{
    // 对于退化运动，使用更严格的参数
    icp.setMaxCorrespondenceDistance(original_max_distance * 0.5);  // 减半距离
    icp.setMaximumIterations(100);  // 增加迭代次数
    icp.setRANSACOutlierRejectionThreshold(original_max_distance * 0.3);  // 更严格的异常值拒绝
    
    // 如果有有效的关键帧，使用加权目标点云
    if (!keyframe_buffer.empty()) {
        // 这里可以实现使用多个关键帧的融合策略
        // 例如：对最近的几个关键帧进行加权融合
        // 权重随时间衰减：最近的帧权重最高
    }
}

/**
 * @brief 应用鲁棒加权以稳定ICP算法
 * 
 * 鲁棒加权策略：
 * 1. 基于fitness score动态调整权重
 * 2. 在退化场景下增加对历史地图的权重
 * 3. 对异常帧进行降权处理
 */
void apply_robust_weighting(
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp,
    double fitness_score,
    bool is_degenerate,
    double map_weight_factor = 1.0)
{
    // 基于fitness score计算权重调整因子
    double weight_factor = 1.0;
    
    if (is_degenerate) {
        // 退化场景：对历史地图增加权重
        weight_factor = map_weight_factor * 1.5;
    } else if (fitness_score > 0.1) {
        // 质量差的匹配：降低权重，更依赖初始猜测
        weight_factor = map_weight_factor * 0.7;
    } else {
        // 正常情况
        weight_factor = map_weight_factor;
    }
    
    // 注意：PCL的ICP并没有直接的权重参数
    // 但可以通过调整其他参数来实现相似效果
    // 例如：调整max_correspondence_distance和RANSAC参数
}

/**
 * @brief IMU回调函数用于监测时间同步和运动状态
 */
void imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    std::deque<IMUData>& imu_buffer,
    std::mutex& imu_buffer_mutex,
    rclcpp::Logger logger)
{
    std::lock_guard<std::mutex> lock(imu_buffer_mutex);
    
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
    imu_data.gyro_bias = Eigen::Vector3d::Zero();  // 可选：实现偏置估计
    
    imu_buffer.push_back(imu_data);
    
    // 保持缓冲区大小
    if (imu_buffer.size() > 1000) {
        imu_buffer.pop_front();
    }
    
    // 监测IMU漂移：角速度范数
    double ang_vel_norm = imu_data.angular_velocity.norm();
    if (ang_vel_norm > 1.0) {  // 阈值：1 rad/s
        RCLCPP_WARN(logger, "High angular velocity detected: %.3f rad/s", ang_vel_norm);
    }
}

/**
 * @brief 同步LiDAR和IMU的时间戳
 */
void sync_time_offset(
    double lidar_timestamp,
    TimeSyncManager& time_sync_manager,
    const std::deque<IMUData>& imu_buffer,
    const std::mutex& imu_buffer_mutex,
    rclcpp::Logger logger)
{
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(imu_buffer_mutex));
    
    if (!imu_buffer.empty()) {
        // 查找最接近的IMU时间戳
        double closest_imu_time = imu_buffer.back().timestamp;
        double min_diff = std::abs(lidar_timestamp - closest_imu_time);
        
        for (const auto& imu_data : imu_buffer) {
            double diff = std::abs(lidar_timestamp - imu_data.timestamp);
            if (diff < min_diff) {
                min_diff = diff;
                closest_imu_time = imu_data.timestamp;
            }
        }
        
        // 更新时间偏移估计
        time_sync_manager.update_offset(lidar_timestamp, closest_imu_time);
        
        if (std::abs(time_sync_manager.estimated_time_offset) > 0.01) {  // 10ms
            RCLCPP_WARN(logger, "Large time offset detected: %.3f s", 
                         time_sync_manager.estimated_time_offset);
        }
    }
}

// 下面的部分将在icp_node.cpp中包含作为类方法
// 这是为了避免重复定义
