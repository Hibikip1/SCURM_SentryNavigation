#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>

class TwistTransformer : public rclcpp::Node
{
public:
    TwistTransformer() : Node("twist_transformer"), current_yaw_(0.0), yaw_valid_(false)
    {
        cmd_in_yaw_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_in_yaw", 10);
        
        // 订阅EKF融合后的odometry获取云台yaw角（带真实orientation）
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/imu/odometry", 10, std::bind(&TwistTransformer::odom_callback, this, std::placeholders::_1));
        
        // 订阅cmd_vel进行转换
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 20, std::bind(&TwistTransformer::cmd_vel_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "TwistTransformer started - using EKF odometry yaw for velocity transformation");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 从EKF输出的odometry四元数提取yaw角（云台相对于初始方向的角度）
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        std::lock_guard<std::mutex> lock(yaw_mutex_);
        current_yaw_ = yaw;
        yaw_valid_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "Updated yaw from EKF: %.3f rad (%.1f deg)", yaw, yaw * 180.0 / M_PI);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double yaw;
        bool valid;
        {
            std::lock_guard<std::mutex> lock(yaw_mutex_);
            yaw = current_yaw_;
            valid = yaw_valid_;
        }
        
        if (!valid) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "No valid yaw from EKF yet, skipping velocity transformation");
            return;
        }
        
        // 将map坐标系下的速度转换到云台(yaw_link)坐标系
        // 这是2D旋转变换: 将世界坐标系的速度转到旋转了yaw角的坐标系
        auto twist_in_yaw = std::make_shared<geometry_msgs::msg::Twist>();
        twist_in_yaw->linear.x = msg->linear.x * cos(yaw) + msg->linear.y * sin(yaw);
        twist_in_yaw->linear.y = msg->linear.y * cos(yaw) - msg->linear.x * sin(yaw);
        twist_in_yaw->angular.x = msg->angular.x;
        twist_in_yaw->angular.y = msg->angular.y;
        twist_in_yaw->angular.z = msg->angular.z;

        cmd_in_yaw_publisher_->publish(*twist_in_yaw);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Transform: yaw=%.3f rad, in:(%.3f,%.3f) -> out:(%.3f,%.3f)", 
            yaw, msg->linear.x, msg->linear.y, 
            twist_in_yaw->linear.x, twist_in_yaw->linear.y);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_in_yaw_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    
    double current_yaw_;
    bool yaw_valid_;
    std::mutex yaw_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistTransformer>());
    rclcpp::shutdown();
    return 0;
}
