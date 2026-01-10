#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class IMURotateNode : public rclcpp::Node
{
public:
    IMURotateNode() : Node("imu_rotate_node", rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        // 参数：是否启用TF变换（预留mid360倾斜放置时使用）
        this->declare_parameter<bool>("use_tf_transform", false);
        this->declare_parameter<double>("roll_offset", 0.0);
        this->declare_parameter<double>("pitch_offset", 0.0);
        this->declare_parameter<double>("yaw_offset", 0.0);
        
        use_tf_transform_ = this->get_parameter("use_tf_transform").as_bool();
        roll_offset_ = this->get_parameter("roll_offset").as_double();
        pitch_offset_ = this->get_parameter("pitch_offset").as_double();
        yaw_offset_ = this->get_parameter("yaw_offset").as_double();
        
        // 计算旋转四元数（预留框架，现在平放不需要）
        if (use_tf_transform_) {
            transform_quat_.setRPY(roll_offset_, pitch_offset_, yaw_offset_);
            RCLCPP_INFO(this->get_logger(), 
                "IMU transform enabled: roll=%.3f, pitch=%.3f, yaw=%.3f", 
                roll_offset_, pitch_offset_, yaw_offset_);
        } else {
            RCLCPP_INFO(this->get_logger(), "IMU transform disabled (mid360 mounted flat)");
        }
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "livox/imu", 10, std::bind(&IMURotateNode::listener_callback, this, std::placeholders::_1));
        
        
    }

private:
    void listener_callback(const sensor_msgs::msg::Imu::UniquePtr msg)
    {
        auto output_msg = std::make_unique<sensor_msgs::msg::Imu>(*msg);
        
        if (use_tf_transform_) {
            // 当mid360倾斜放置时，应用TF变换
            // 变换角速度
            tf2::Vector3 angular_vel(msg->angular_velocity.x, 
                                     msg->angular_velocity.y, 
                                     msg->angular_velocity.z);
            angular_vel = tf2::quatRotate(transform_quat_, angular_vel);
            output_msg->angular_velocity.x = angular_vel.x();
            output_msg->angular_velocity.y = angular_vel.y();
            output_msg->angular_velocity.z = angular_vel.z();
            
            // 变换线加速度
            tf2::Vector3 linear_accel(msg->linear_acceleration.x,
                                      msg->linear_acceleration.y,
                                      msg->linear_acceleration.z);
            linear_accel = tf2::quatRotate(transform_quat_, linear_accel);
            output_msg->linear_acceleration.x = linear_accel.x();
            output_msg->linear_acceleration.y = linear_accel.y();
            output_msg->linear_acceleration.z = linear_accel.z();
            
            RCLCPP_DEBUG(this->get_logger(), "Applied IMU transform");
        }
        // else: 平放时直接转发原始数据
        
        // 设置协方差（robot_localization要求协方差不能全为0）
        // orientation协方差 - 因为livox不提供orientation，设为大值表示不可信
        output_msg->orientation_covariance[0] = 0.01;  // roll
        output_msg->orientation_covariance[4] = 0.01;  // pitch
        output_msg->orientation_covariance[8] = 0.01;  // yaw
        
        // angular_velocity协方差 - mid360陀螺仪精度约0.005 rad/s
        output_msg->angular_velocity_covariance[0] = 0.0001;  // x
        output_msg->angular_velocity_covariance[4] = 0.0001;  // y
        output_msg->angular_velocity_covariance[8] = 0.0001;  // z
        
        // linear_acceleration协方差 - mid360加速度计精度约0.01 m/s²
        output_msg->linear_acceleration_covariance[0] = 0.01;  // x
        output_msg->linear_acceleration_covariance[4] = 0.01;  // y
        output_msg->linear_acceleration_covariance[8] = 0.01;  // z
        
        publisher_->publish(std::move(output_msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    
    bool use_tf_transform_;
    double roll_offset_;
    double pitch_offset_;
    double yaw_offset_;
    tf2::Quaternion transform_quat_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMURotateNode>());
    rclcpp::shutdown();
    return 0;
}
