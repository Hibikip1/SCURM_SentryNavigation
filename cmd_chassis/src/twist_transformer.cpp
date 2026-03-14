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
        // Nav2 的 cmd_vel 默认已是机器人本体坐标系（base_link/云台系），无需再按 yaw 旋转
        this->declare_parameter<bool>("use_yaw_transform", false);
        use_yaw_transform_ = this->get_parameter("use_yaw_transform").as_bool();

        cmd_in_yaw_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_in_yaw", 10);

        // 仅当启用 map→body 变换时才订阅 EKF odometry
        if (use_yaw_transform_) {
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/imu/odometry", 10, std::bind(&TwistTransformer::odom_callback, this, std::placeholders::_1));
        }

        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 20, std::bind(&TwistTransformer::cmd_vel_callback, this, std::placeholders::_1));

        if (use_yaw_transform_) {
            RCLCPP_INFO(this->get_logger(),
                "TwistTransformer: use_yaw_transform=true, map->body velocity transform enabled (EKF yaw)");
        } else {
            RCLCPP_INFO(this->get_logger(),
                "TwistTransformer: pass-through mode (cmd_vel already in body frame, no rotation)");
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
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
        geometry_msgs::msg::Twist twist_out;
        twist_out.angular = msg->angular;

        if (use_yaw_transform_) {
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
            // map -> body 旋转变换
            twist_out.linear.x = msg->linear.x * cos(yaw) + msg->linear.y * sin(yaw);
            twist_out.linear.y = msg->linear.y * cos(yaw) - msg->linear.x * sin(yaw);
        } else {
            // 透传：Nav2 的 cmd_vel 已是本体（云台）坐标系，直接转发给底盘
            twist_out.linear = msg->linear;
        }

        cmd_in_yaw_publisher_->publish(twist_out);

        RCLCPP_DEBUG(this->get_logger(),
            "Out: linear=(%.3f,%.3f) angular.z=%.3f", twist_out.linear.x, twist_out.linear.y, twist_out.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_in_yaw_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

    bool use_yaw_transform_;
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
