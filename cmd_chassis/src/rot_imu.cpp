#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class IMURotateNode : public rclcpp::Node
{
public:
    IMURotateNode() : Node("imu_rotate_node", rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        // MID360 实际安装：roll=-48°, pitch=1°
        // 建图需要变换到水平向下：roll=132° (即 -48° + 180°)
        // 所以这里应用 roll=132°, pitch=1° 的变换
        use_tf_transform_ = true;
        roll_offset_ = -0.837758041;  // -48° = -0.837758041 rad
        pitch_offset_ = 0.017453; // 1° = 0.017453 rad
        yaw_offset_ = 0.0;
        transform_quat_.setRPY(roll_offset_, pitch_offset_, yaw_offset_);
        
        // 声明并获取 IMU bias 补偿参数
        this->declare_parameter("enable_bias_compensation", false);
        this->declare_parameter("gyro_bias_x", 0.0);
        this->declare_parameter("gyro_bias_y", 0.0);
        this->declare_parameter("gyro_bias_z", 0.0);
        this->declare_parameter("acc_bias_x", 0.0);
        this->declare_parameter("acc_bias_y", 0.0);
        this->declare_parameter("acc_bias_z", 0.0);
        
        enable_bias_compensation_ = this->get_parameter("enable_bias_compensation").as_bool();
        gyro_bias_x_ = this->get_parameter("gyro_bias_x").as_double();
        gyro_bias_y_ = this->get_parameter("gyro_bias_y").as_double();
        gyro_bias_z_ = this->get_parameter("gyro_bias_z").as_double();
        acc_bias_x_ = this->get_parameter("acc_bias_x").as_double();
        acc_bias_y_ = this->get_parameter("acc_bias_y").as_double();
        acc_bias_z_ = this->get_parameter("acc_bias_z").as_double();
        
        RCLCPP_INFO(this->get_logger(), "IMU transform固定: roll=%.6f, pitch=%.3f, yaw=%.3f", 
                    roll_offset_, pitch_offset_, yaw_offset_);
        
        if (enable_bias_compensation_) {
            RCLCPP_INFO(this->get_logger(), "IMU bias 补偿已启用:");
            RCLCPP_INFO(this->get_logger(), "  陀螺仪 bias: [%.6f, %.6f, %.6f] rad/s", 
                        gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
            RCLCPP_INFO(this->get_logger(), "  加速度 bias: [%.6f, %.6f, %.6f] m/s²", 
                        acc_bias_x_, acc_bias_y_, acc_bias_z_);
        } else {
            RCLCPP_INFO(this->get_logger(), "IMU bias 补偿未启用");
        }
        
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu_192_168_1_172", 10, std::bind(&IMURotateNode::listener_callback, this, std::placeholders::_1));
        
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
        
        // 应用 bias 补偿（在坐标变换之后）
        if (enable_bias_compensation_) {
            output_msg->angular_velocity.x -= gyro_bias_x_;
            output_msg->angular_velocity.y -= gyro_bias_y_;
            output_msg->angular_velocity.z -= gyro_bias_z_;
            output_msg->linear_acceleration.x -= acc_bias_x_;
            output_msg->linear_acceleration.y -= acc_bias_y_;
            output_msg->linear_acceleration.z -= acc_bias_z_;
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

    // IMU bias 补偿参数
    bool enable_bias_compensation_;
    double gyro_bias_x_;
    double gyro_bias_y_;
    double gyro_bias_z_;
    double acc_bias_x_;
    double acc_bias_y_;
    double acc_bias_z_;
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
