#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace
{
/**
 * @brief 一次匹配的质量评估结果
 *
 * 注意：PCL 的 getFitnessScore() 是"所有源点最近邻距离平方的平均值"，
 * 没有上限、没有内点比例，一个完美对齐的结果只要有 10% 的点在地图外就会飙到 2.0，
 * 完全不能拿来跟一个固定阈值比较。这里改成 内点率 + 内点上的截断 RMSE。
 */
struct MatchQuality
{
    int    points        = 0;     // 参与评估的源点数
    int    inliers       = 0;     // 最近邻距离 < inlier_dist 的点数
    double inlier_ratio  = 0.0;   // 内点率
    double rmse          = 1e9;   // 仅在内点上统计的 RMSE (m)
    double p50 = 0.0, p90 = 0.0, p99 = 0.0, dmax = 0.0;
};
}  // namespace

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
        this->declare_parameter("solver_max_iter", 30);
        this->declare_parameter("max_correspondence_distance", 0.1);
        this->declare_parameter("correspondence_levels", std::vector<double>{1.0, 0.5, 0.25});
        this->declare_parameter("RANSAC_outlier_rejection_threshold", 1.0);
        this->declare_parameter("map_path", "");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("map_voxel_leaf_size", 0.1);
        this->declare_parameter("cloud_voxel_leaf_size", 0.1);
        this->declare_parameter("converged_count_thre", 20);
        this->declare_parameter("pcl_type", "livox");
        // 质量判定参数（核心）
        this->declare_parameter("blind", 0.5);              // 近处裁剪半径,去掉机器人自身点
        this->declare_parameter("inlier_dist", 0.2);        // 内点判定距离 (m)
        this->declare_parameter("min_inlier_ratio", 0.65);  // 最少内点率
        this->declare_parameter("max_rmse", 0.15);          // 内点 RMSE 上限 (m)
        this->declare_parameter("max_median_dist", 0.15);   // 最近邻距离中位数上限 (m)
        // 已废弃：保留声明只是为了兼容旧的 launch,不会被使用
        this->declare_parameter("fitness_score_thre", 0.0);

        this->get_parameter("initial_x", initial_x);
        this->get_parameter("initial_y", initial_y);
        this->get_parameter("initial_z", initial_z);
        this->get_parameter("initial_a", initial_a);
        this->get_parameter("solver_max_iter", solver_max_iter);
        this->get_parameter("max_correspondence_distance", max_correspondence_distance);
        this->get_parameter("correspondence_levels", correspondence_levels);
        this->get_parameter("RANSAC_outlier_rejection_threshold", RANSAC_outlier_rejection_threshold);
        this->get_parameter("map_path", map_path);
        this->get_parameter("map_frame_id", map_frame);
        this->get_parameter("map_voxel_leaf_size", map_voxel_leaf_size);
        this->get_parameter("cloud_voxel_leaf_size", cloud_voxel_leaf_size);
        this->get_parameter("converged_count_thre", converged_count_thre);
        this->get_parameter("pcl_type", pcl_type);
        this->get_parameter("blind", blind_);
        this->get_parameter("inlier_dist", inlier_dist_);
        this->get_parameter("min_inlier_ratio", min_inlier_ratio_);
        this->get_parameter("max_rmse", max_rmse_);
        this->get_parameter("max_median_dist", max_median_dist_);

        double deprecated_thre = 0.0;
        this->get_parameter("fitness_score_thre", deprecated_thre);
        if (deprecated_thre > 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                "参数 fitness_score_thre(%.3f) 已废弃并被忽略：PCL 的 fitness 是全体源点最近邻"
                "距离平方的平均值,没有上限也没有内点率,不能作为判定依据。现在使用 "
                "min_inlier_ratio / max_rmse 判定。", deprecated_thre);
        }
        if (correspondence_levels.empty())
        {
            correspondence_levels.push_back(max_correspondence_distance);
        }

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_result", 10);
#ifdef USE_LIVOX
        if (pcl_type == "livox")
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
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&ICPNode::pose_callback, this, std::placeholders::_1));
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("prior_map", 10);
        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);

        // init guess
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = initial_x;
        initGuess(1, 3) = initial_y;
        initGuess(2, 3) = initial_z;
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
        RCLCPP_INFO(this->get_logger(), "Initial guess: \n x: %f, y: %f, z: %f, a: %f",
            initial_x, initial_y, initial_z, initial_a);

        // Load the target point cloud from a PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *target_cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", map_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %d data points from %s",
            (int)(target_cloud_->width * target_cloud_->height), map_path.c_str());

        // downsample the target cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_map;
        sor_map.setInputCloud(target_cloud_);
        sor_map.setLeafSize(map_voxel_leaf_size, map_voxel_leaf_size, map_voxel_leaf_size);
        sor_map.filter(*target_cloud_);
        RCLCPP_INFO(this->get_logger(), "Downsampled target cloud to %d data points (leaf %.2f m)",
            (int)(target_cloud_->width * target_cloud_->height), map_voxel_leaf_size);

        // 质量评估用的 KD-tree,建一次反复用
        target_tree_.setInputCloud(target_cloud_);
        target_ready_ = true;

        pcl::toROSMsg(*target_cloud_, target_cloud_msg);
        target_cloud_msg.header.stamp = this->now();
        target_cloud_msg.header.frame_id = map_frame;
        map_pub_->publish(target_cloud_msg);

        std::string lv;
        for (double d : correspondence_levels) lv += std::to_string(d) + " ";
        RCLCPP_INFO(this->get_logger(),
            "Judge: inlier_dist=%.2f m, min_inlier_ratio=%.2f, max_rmse=%.3f m, "
            "max_median=%.3f m, blind=%.2f m, correspondence_levels=[ %s], "
            "iters/stage=%d, need %d consecutive frames",
            inlier_dist_, min_inlier_ratio_, max_rmse_, max_median_dist_, blind_, lv.c_str(),
            solver_max_iter, converged_count_thre);
    }

private:
    // ------------------------------------------------------------------
    // 点云预处理
    // ------------------------------------------------------------------
    static bool isValidPoint(double x, double y, double z, double blind)
    {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
        if (x == 0.0 && y == 0.0 && z == 0.0) return false;  // Livox 无效点
        return (x * x + y * y + z * z) > blind * blind;      // 去掉机器人自身
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *raw);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        filtered->reserve(raw->size());
        for (const auto & p : raw->points)
        {
            if (isValidPoint(p.x, p.y, p.z, blind_)) filtered->push_back(p);
        }
        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = true;
        handleCloud(filtered);
    }

#ifdef USE_LIVOX
    void lvx_cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        filtered->reserve(msg->point_num);
        for (uint32_t i = 0; i < msg->point_num; ++i)
        {
            const auto & pt = msg->points[i];
            // 与 FAST-LIO preprocess 一致：只保留 tag 0x30 为 0x10(高置信) 或 0x00 的点
            const uint8_t t = pt.tag & 0x30;
            if (t != 0x10 && t != 0x00) continue;
            if (!isValidPoint(pt.x, pt.y, pt.z, blind_)) continue;
            filtered->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }
        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = true;
        handleCloud(filtered);
    }
#endif

    // ------------------------------------------------------------------
    // 多分辨率 ICP: 由粗到细,避免 0.1 m 对应距离"根本拉不动"
    // ------------------------------------------------------------------
    Eigen::Matrix4f alignMultiResolution(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
        const Eigen::Matrix4f & guess,
        bool & converged,
        double & pcl_fitness)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(source);
        icp.setInputTarget(target_cloud_);
        icp.setMaximumIterations(solver_max_iter);
        icp.setRANSACOutlierRejectionThreshold(RANSAC_outlier_rejection_threshold);

        Eigen::Matrix4f T = guess;
        pcl::PointCloud<pcl::PointXYZ> aligned;
        for (size_t i = 0; i < correspondence_levels.size(); ++i)
        {
            icp.setMaxCorrespondenceDistance(correspondence_levels[i]);
            icp.align(aligned, T);
            T = icp.getFinalTransformation();
        }
        converged = icp.hasConverged();
        pcl_fitness = icp.getFitnessScore();
        return T;
    }

    // ------------------------------------------------------------------
    // 质量评估: 内点率 + 截断 RMSE + 距离分位数
    // ------------------------------------------------------------------
    MatchQuality evaluate(const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
                          const Eigen::Matrix4f & T)
    {
        MatchQuality q;
        if (!target_ready_ || source->empty()) return q;

        pcl::PointCloud<pcl::PointXYZ> aligned;
        pcl::transformPointCloud(*source, aligned, T);

        std::vector<float> dists;
        dists.reserve(aligned.size());
        std::vector<int> idx(1);
        std::vector<float> d2(1);
        double se = 0.0;
        int inliers = 0;
        for (const auto & p : aligned.points)
        {
            if (target_tree_.nearestKSearch(p, 1, idx, d2) < 1) continue;
            const float d = std::sqrt(d2[0]);
            dists.push_back(d);
            if (d < inlier_dist_)
            {
                ++inliers;
                se += d2[0];
            }
        }
        q.points = static_cast<int>(aligned.size());
        q.inliers = inliers;
        if (!dists.empty())
        {
            std::sort(dists.begin(), dists.end());
            const size_t n = dists.size();
            q.p50 = dists[static_cast<size_t>(0.50 * (n - 1))];
            q.p90 = dists[static_cast<size_t>(0.90 * (n - 1))];
            q.p99 = dists[static_cast<size_t>(0.99 * (n - 1))];
            q.dmax = dists.back();
            q.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(n);
        }
        q.rmse = inliers > 0 ? std::sqrt(se / inliers) : 1e9;
        return q;
    }

    static double yawOf(const Eigen::Matrix4f & T)
    {
        return std::atan2(T(1, 0), T(0, 0));
    }

    // ------------------------------------------------------------------
    // 主流程
    // ------------------------------------------------------------------
    void handleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & raw)
    {
        if (!target_ready_ || raw->size() < 50)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Too few points after filtering (%zu), skip", raw->size());
            return;
        }

        auto t_start = this->now();

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
        sor_scan.setInputCloud(raw);
        sor_scan.setLeafSize(cloud_voxel_leaf_size, cloud_voxel_leaf_size, cloud_voxel_leaf_size);
        sor_scan.filter(*scan);

        bool converged = false;
        double pcl_fitness = 0.0;
        Eigen::Matrix4f T = alignMultiResolution(scan, initGuess, converged, pcl_fitness);
        const MatchQuality q = evaluate(scan, T);
        const double yaw = yawOf(T);
        const double dt_ms = (this->now() - t_start).nanoseconds() / 1e6;

        RCLCPP_INFO(this->get_logger(),
            "scan=%d | nn p50=%.3f p90=%.3f p99=%.3f | inlier@%.2fm=%.1f%% rmse=%.3f | "
            "pcl_fitness=%.3f | pose x=%.3f y=%.3f z=%.3f yaw=%.1fdeg | %s | %.0f ms",
            q.points, q.p50, q.p90, q.p99, inlier_dist_, 100.0 * q.inlier_ratio, q.rmse,
            pcl_fitness, T(0, 3), T(1, 3), T(2, 3), yaw * 180.0 / M_PI,
            converged ? "converged" : "no-converge", dt_ms);

        // 三个条件同时满足：足够多的点贴上了地图、贴上的点足够准、且"典型"点也足够准。
        // 只看内点率会被地面/天花板骗过（旋转误差不改变地面点对地面的距离），
        // 所以再加上最近邻中位数，用来卡住"原地打转"的位姿。
        const bool accept = (q.inlier_ratio >= min_inlier_ratio_) &&
                            (q.rmse <= max_rmse_) &&
                            (q.p50 <= max_median_dist_);

        // 只有"确实更好"的结果才允许更新初值,避免一帧坏结果把位姿永久带偏
        const bool improved = !best_valid_ ||
            (q.inlier_ratio > best_inlier_ratio_ + 0.02) ||
            (std::abs(q.inlier_ratio - best_inlier_ratio_) <= 0.02 && q.rmse < best_rmse_);
        if (improved)
        {
            best_valid_ = true;
            best_inlier_ratio_ = q.inlier_ratio;
            best_rmse_ = q.rmse;
            initGuess = T;
        }

        if (accept)
        {
            ++converged_count;
            RCLCPP_INFO(this->get_logger(), "ICP accepted (%d/%d)", converged_count, converged_count_thre);
            if (converged_count < converged_count_thre)
            {
                publishDebugCloud(scan, initGuess);
                publishMap();
                return;
            }
            publishPose(T);
            publishDebugCloud(scan, T);
            publishMap();
            RCLCPP_INFO(this->get_logger(), "Relocalization done, shutting down icp_node");
            rclcpp::shutdown();
            return;
        }

        converged_count = 0;
        RCLCPP_WARN(this->get_logger(),
            "ICP rejected: inlier %.1f%% (need %.1f%%), rmse %.3f m (need <= %.3f), "
            "median %.3f m (need <= %.3f); %s",
            100.0 * q.inlier_ratio, 100.0 * min_inlier_ratio_, q.rmse, max_rmse_,
            q.p50, max_median_dist_, improved ? "guess updated" : "guess kept");
        publishDebugCloud(scan, initGuess);
        publishMap();
    }

    void publishPose(const Eigen::Matrix4f & T)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = map_frame;
        pose_msg.pose.pose.position.x = T(0, 3);
        pose_msg.pose.pose.position.y = T(1, 3);
        pose_msg.pose.pose.position.z = T(2, 3);
        Eigen::Matrix3f rotation = T.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);
        q.normalize();
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();
        publisher_->publish(pose_msg);
    }

    void publishDebugCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan,
                           const Eigen::Matrix4f & T)
    {
        pcl::PointCloud<pcl::PointXYZ> out;
        pcl::transformPointCloud(*scan, out, T);
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(out, msg);
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame;
        transformed_cloud_pub_->publish(msg);
    }

    void publishMap()
    {
        target_cloud_msg.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg);
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = msg->pose.pose.position.x;
        initGuess(1, 3) = msg->pose.pose.position.y;
        initGuess(2, 3) = msg->pose.pose.position.z;
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
        // 新的初值意味着重新开始计数
        converged_count = 0;
        best_valid_ = false;
        double r, p, yaw;
        rot_mat.getRPY(r, p, yaw);
        RCLCPP_INFO(this->get_logger(), "New initial guess: \n x: %f, y: %f, z: %f, a: %f",
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::KdTreeFLANN<pcl::PointXYZ> target_tree_;
    bool target_ready_ = false;

    Eigen::Matrix4f initGuess;
    double initial_x, initial_y, initial_z, initial_a;
    int solver_max_iter;
    double max_correspondence_distance, RANSAC_outlier_rejection_threshold;
    std::vector<double> correspondence_levels;
    std::string map_path, map_frame;
    double map_voxel_leaf_size, cloud_voxel_leaf_size;
    sensor_msgs::msg::PointCloud2 target_cloud_msg;
    int converged_count = 0;
    int converged_count_thre;
    std::string pcl_type;
    double blind_, inlier_dist_, min_inlier_ratio_, max_rmse_, max_median_dist_;

    bool best_valid_ = false;
    double best_inlier_ratio_ = -1.0;
    double best_rmse_ = 1e9;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ICPNode>();
    if (rclcpp::ok())
    {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
