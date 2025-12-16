#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class TransformPublisherNode : public rclcpp::Node
{
public:
    TransformPublisherNode()
    : Node("transform_publisher_node")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "state_estimation", 200,
            std::bind(&TransformPublisherNode::listener_callback, this, std::placeholders::_1));
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void listener_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto transform = std::make_shared<geometry_msgs::msg::TransformStamped>();

        transform->header.stamp = this->now();
        transform->header.frame_id = "odom";
        transform->child_frame_id = "chassis_link";
        transform->transform.translation.x = msg->pose.pose.position.x;
        transform->transform.translation.y = msg->pose.pose.position.y;
        transform->transform.translation.z = msg->pose.pose.position.z;
        
        // 从FAST_LIO的orientation中只提取yaw角度（忽略可能错误的roll/pitch）
        // 这对导航和重定位很重要：底盘应该保持水平（roll=pitch=0）
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 重新构造只包含yaw的四元数（roll=0, pitch=0）
        tf2::Quaternion q_yaw_only;
        q_yaw_only.setRPY(0.0, 0.0, yaw);
        
        transform->transform.rotation.x = q_yaw_only.x();
        transform->transform.rotation.y = q_yaw_only.y();
        transform->transform.rotation.z = q_yaw_only.z();
        transform->transform.rotation.w = q_yaw_only.w();

        // Publish the transform
        broadcaster_->sendTransform(*transform);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TransformPublisherNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}