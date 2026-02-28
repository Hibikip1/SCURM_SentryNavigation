#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <message_filters/subscriber.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense> // 用于构建变换矩阵

class MergeCloudNode : public rclcpp::Node
{
public:
  MergeCloudNode() : Node("merge_cloud_node")
  {
    cloud1_topic_ = this->declare_parameter<std::string>("cloud1_topic", "/livox/lidar_192_168_1_172");
    cloud2_topic_ = this->declare_parameter<std::string>("cloud2_topic", "/livox/lidar_192_168_1_115");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/merged_cloud");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "");
    voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.0);
    output_qos_depth_ = this->declare_parameter<int>("output_qos_depth", 10);
    // 默认用 RELIABLE，确保 RViz/部分算法节点(默认Reliable订阅)能直接看到点云。
    // 若追求更低延迟/更少回传，可在启动时设置为 false (BEST_EFFORT)。
    output_reliable_qos_ = this->declare_parameter<bool>("output_reliable_qos", true);

    // 订阅两个 Livox 点云话题
    cloud1_sub_.subscribe(this, cloud1_topic_);
    cloud2_sub_.subscribe(this, cloud2_topic_);

    // 使用 ApproximateTime 同步器
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), cloud1_sub_, cloud2_sub_);
    sync_->registerCallback(
      std::bind(&MergeCloudNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    rclcpp::QoS out_qos(rclcpp::KeepLast(std::max(1, output_qos_depth_)));
    out_qos.durability_volatile();
    if (output_reliable_qos_) {
      out_qos.reliable();
    } else {
      out_qos.best_effort();
    }
    merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, out_qos);
  }

private:
  void syncCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg)
  {
    // 将 ROS2 PointCloud2消息转换为 PCL 点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud1_msg, *cloud1);
    pcl::fromROSMsg(*cloud2_msg, *cloud2);


    // 定义变换矩阵
    Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity(); // cloud1 的变换矩阵
    Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity(); // cloud2 的变换矩阵

    // 设置 cloud1 的变换矩阵（欧拉角 + 平移）
    setTransformMatrix(transform1, roll1_, pitch1_, yaw1_, tx1_, ty1_, tz1_);

    // 设置 cloud2 的变换矩阵（欧拉角 + 平移）
    setTransformMatrix(transform2, roll2_, pitch2_, yaw2_, tx2_, ty2_, tz2_);

    // 对点云进行坐标变换
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud1, *cloud1_transformed, transform1);
    pcl::transformPointCloud(*cloud2, *cloud2_transformed, transform2);

    // 合并点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *merged_cloud = *cloud1_transformed + *cloud2_transformed;

    if (voxel_leaf_size_ > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZI> voxel;
      voxel.setLeafSize(
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_));
      voxel.setInputCloud(merged_cloud);
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
      voxel.filter(*filtered);
      merged_cloud = filtered;
    }
    
    // 将 PCL 点云转换为 ROS 2 消息
    sensor_msgs::msg::PointCloud2 merged_msg;
    pcl::toROSMsg(*merged_cloud, merged_msg);

    if (!output_frame_id_.empty()) {
      merged_msg.header.frame_id = output_frame_id_;
    } else {
      merged_msg.header.frame_id = cloud1_msg->header.frame_id;
    }

    // 重要：保留来自雷达的时间戳，FAST_LIO 依赖该时间与 IMU 对齐
    const rclcpp::Time t1(cloud1_msg->header.stamp);
    const rclcpp::Time t2(cloud2_msg->header.stamp);
    merged_msg.header.stamp = (t1 >= t2) ? cloud1_msg->header.stamp : cloud2_msg->header.stamp;

    merged_cloud_pub_->publish(merged_msg);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Published merged cloud: %zu pts (leaf=%.3f)",
      merged_cloud->size(), voxel_leaf_size_);
    //RCLCPP_INFO(this->get_logger(), "Published merged cloud with %zu points.", merged_cloud->size());
  }

  // 设置变换矩阵（欧拉角 + 平移）
  void setTransformMatrix(Eigen::Matrix4f& transform, float roll, float pitch, float yaw, float tx, float ty, float tz)
  {
    // 计算旋转矩阵
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3f rotationMatrix = q.matrix();

    // 设置变换矩阵的旋转部分
    transform.block<3, 3>(0, 0) = rotationMatrix;

    // 设置变换矩阵的平移部分
    transform(0, 3) = tx;
    transform(1, 3) = ty;
    transform(2, 3) = tz;
  }

  // 定义同步策略
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> SyncPolicy;

  // 订阅器
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud1_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud2_sub_;

  // 同步器
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // 发布器
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;

  // 变换参数（欧拉角 + 平移
  // 注意两个topic对应的变换矩阵的不同
  float roll1_ = 0.0f, pitch1_ = 0.0f, yaw1_ = 0.0f; // cloud1 的欧拉角（弧度）
  float tx1_ = 0.0f, ty1_ = 0.0f, tz1_ = 0.0f;       // cloud1 的平移（米）


  std::string cloud1_topic_;
  std::string cloud2_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  double voxel_leaf_size_ = 0.0;
  int output_qos_depth_ = 10;
  bool output_reliable_qos_ = false;
  float roll2_ = 0.0f, pitch2_ = 0.0f, yaw2_ = 0.0f; // cloud2 的欧拉角（弧度）
  float tx2_ = 0.0f, ty2_ = 0.0f, tz2_ = 0.0f;       // cloud2 的平移（米）
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MergeCloudNode>());
  rclcpp::shutdown();
  return 0;
}
