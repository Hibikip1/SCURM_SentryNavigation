#include <rclcpp/rclcpp.hpp>
#include "robotcontrol/bmcan_bus.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rm_interfaces/msg/game_state.hpp>
#include <rm_interfaces/msg/shoot_cmd.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

using namespace std::chrono_literals;

// 与 protocol_parser.hpp 中保持一致的 CAN ID 定义（可后续抽取到公共头文件）
namespace can_id {
constexpr uint32_t IMU_DATA      = 0x100;
constexpr uint32_t ODOM_DATA     = 0x101;
constexpr uint32_t GAME_STATE    = 0x102;
constexpr uint32_t CHASSIS_CMD   = 0x200;
constexpr uint32_t SHOOTER_CMD   = 0x202;
}

class RobotCan : public rclcpp::Node
{ 
public:
  RobotCan() : Node("robot_can")
  {
    RCLCPP_INFO(this->get_logger(), "%s 节点已启动", this->get_name());

    // 参数
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_in_yaw");
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

    // 打开 CAN 设备
    BM_NotificationHandle result = canbus_.open(channel_handle_, "BM-CANFD-X1(5850)");
    if (result == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "CAN 设备打开失败");
    } else {
      RCLCPP_INFO(this->get_logger(), "CAN 设备已打开");
    }

    // 发布者
    imu_pub_        = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    odom_pub_       = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    game_state_pub_ = this->create_publisher<rm_interfaces::msg::GameState>("game_state", 10);

    // 订阅者
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10, std::bind(&RobotCan::on_cmd_vel, this, std::placeholders::_1));
    shoot_cmd_sub_ = this->create_subscription<rm_interfaces::msg::ShootCmd>(
      "shoot_cmd", 10, std::bind(&RobotCan::on_shoot_cmd, this, std::placeholders::_1));

    // 启动接收线程
    running_.store(true);
    rx_thread_ = std::thread(&RobotCan::rx_loop, this);
  }

  ~RobotCan() override
  {
    running_.store(false);
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    canbus_.close(channel_handle_);
  }

private:
  /* ================= 编码 / 解码 工具函数 ================= */
  static void float_to_bytes(float value, uint8_t *data, size_t offset)
  {
    std::memcpy(&data[offset], &value, sizeof(float));
  }
  static float bytes_to_float(const uint8_t *data, size_t offset)
  {
    float v; std::memcpy(&v, &data[offset], sizeof(float)); return v;
  }
  static void uint16_to_bytes(uint16_t value, uint8_t *data, size_t offset)
  {
    data[offset] = static_cast<uint8_t>(value & 0xFF);
    data[offset+1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  }
  static uint16_t bytes_to_uint16(const uint8_t *data, size_t offset)
  {
    return static_cast<uint16_t>(data[offset]) | (static_cast<uint16_t>(data[offset+1]) << 8);
  }

  /* ================= ROS 回调：编码后发送 ================= */
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    BM_CanMessageTypeDef can_msg{};
    can_msg.id = can_id::CHASSIS_CMD;
    can_msg.ide = 0; // 标准帧
    can_msg.rtr = 0; // 数据帧
    can_msg.dlc = 8; // vx + wz 两个 float
    float_to_bytes(static_cast<float>(msg->linear.x), can_msg.data, 0);
    float_to_bytes(static_cast<float>(msg->angular.z), can_msg.data, 4);

    if (BM_WriteCanMessage(channel_handle_, &can_msg, 0, 5, nullptr) != BM_ERROR_OK) {
      RCLCPP_WARN(this->get_logger(), "发送底盘速度指令失败");
    }
  }

  void on_shoot_cmd(const rm_interfaces::msg::ShootCmd::SharedPtr msg)
  {
    BM_CanMessageTypeDef can_msg{};
    can_msg.id = can_id::SHOOTER_CMD;
    can_msg.ide = 0; can_msg.rtr = 0;
    can_msg.dlc = 3; // type 1B + projectile_num 2B
    can_msg.data[0] = msg->type;
    uint16_to_bytes(msg->projectile_num, can_msg.data, 1);

    if (BM_WriteCanMessage(channel_handle_, &can_msg, 0, 5, nullptr) != BM_ERROR_OK) {
      RCLCPP_WARN(this->get_logger(), "发送射击指令失败");
    }
  }

  /* ================= 接收线程：读取并解码 ================= */
  void rx_loop()
  {
    RCLCPP_INFO(this->get_logger(), "CAN 接收线程启动");
    while (rclcpp::ok() && running_.load()) {
      BM_CanMessageTypeDef msg{};
      auto status = BM_ReadCanMessage(channel_handle_, &msg, nullptr, nullptr);
      if (status == BM_ERROR_QRCVEMPTY) {
        std::this_thread::sleep_for(1ms);
        continue;
      } else if (status != BM_ERROR_OK) {
        // 其它错误，打印一次即可
        RCLCPP_DEBUG(this->get_logger(), "BM_ReadCanMessage 返回状态: %d", status);
        std::this_thread::sleep_for(2ms);
        continue;
      }
      handle_rx_can(msg);
    }
    RCLCPP_INFO(this->get_logger(), "CAN 接收线程退出");
  }

  void handle_rx_can(const BM_CanMessageTypeDef &msg)
  {
    switch (msg.id) {
      case can_id::IMU_DATA: {
        if (msg.dlc < 8) return; // 示例只用了两个 float
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = this->now();
        imu.header.frame_id = "imu_link";
        imu.orientation.x = bytes_to_float(msg.data, 0);
        imu.orientation.y = bytes_to_float(msg.data, 4);
        imu_pub_->publish(imu);
        break; }
      case can_id::ODOM_DATA: {
        if (msg.dlc < 8) return; // 示例两个 float 位置
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = bytes_to_float(msg.data, 0);
        odom.pose.pose.position.y = bytes_to_float(msg.data, 4);
        odom_pub_->publish(odom);
        break; }
      case can_id::GAME_STATE: {
        if (msg.dlc < 5) return; // game_progress(1) + stage_remain_time(2) + hp(2)
        rm_interfaces::msg::GameState gs;
        gs.game_progress = msg.data[0];
        gs.stage_remain_time = bytes_to_uint16(msg.data, 1);
        gs.current_hp = bytes_to_uint16(msg.data, 3);
        game_state_pub_->publish(gs);
        break; }
      default:
        // 可根据需要降低日志频率
        // RCLCPP_DEBUG(this->get_logger(), "未知或未处理的 CAN ID: 0x%X", msg.id);
        break;
    }
  }

  /* ================ 成员变量 ================ */
  BMCANTool canbus_;
  BM_ChannelHandle channel_handle_{};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr game_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<rm_interfaces::msg::ShootCmd>::SharedPtr shoot_cmd_sub_;

  std::thread rx_thread_;
  std::atomic<bool> running_ {false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotCan>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}