#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense> // 用于构建变换矩阵

class MergeCloudNode : public rclcpp::Node
{
public:
  MergeCloudNode() : Node("merge_cloud_node")
  {
    // 订阅两个 Livox 点云话题
    cloud1_sub_.subscribe(this, "/livox/lidar_192_168_1_172");
    cloud2_sub_.subscribe(this, "/livox/lidar_192_168_1_115");

    // 使用 ApproximateTime 同步器
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), cloud1_sub_, cloud2_sub_);
    sync_->registerCallback(
      std::bind(&MergeCloudNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 发布合并后的点云
    merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merged_cloud", 10);
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
    
    // 将 PCL 点云转换为 ROS 2 消息
    sensor_msgs::msg::PointCloud2 merged_msg;
    pcl::toROSMsg(*merged_cloud, merged_msg);
    // 重要：保留来自雷达的时间戳，FAST_LIO 依赖该时间与 IMU 对齐
    merged_msg.header.frame_id = "sensor";
    const rclcpp::Time t1(cloud1_msg->header.stamp);
    const rclcpp::Time t2(cloud2_msg->header.stamp);
    merged_msg.header.stamp = (t1 >= t2) ? cloud1_msg->header.stamp : cloud2_msg->header.stamp;

    // 发布合并后的点云
    merged_cloud_pub_->publish(merged_msg);
    RCLCPP_INFO(this->get_logger(), "Published merged cloud with %zu points.", merged_cloud->size());
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
