/*
协议解析器，负责将 ROS2 消息与 CAN 帧进行转换。
负责将 ROS2 消息（如 /cmd_vel、/shoot_cmd）编码为 CAN 帧，通过 can_driver 发送到 STM32。
负责将 STM32 通过 CAN 发来的帧解码为 ROS2 消息（如 /imu/data、/odom、/game_// ROS2 订阅回调：收到速度指令，编码为 CAN 帧并发送
void ProtocolParser::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 在仿真模式下只打印日志，不编码和发送
  if (simulate_hardware_) {
    RCLCPP_INFO(node_->get_logger(), 
      "Simulated CAN send: Chassis Command vx=%.2f, wz=%.2f", 
      msg->linear.x, msg->angular.z);
    return;  // 直接返回，不调用 CAN 驱动
  }
  
  // 实际硬件模式
  auto can_msg = encode_chassis_cmd(*msg);
  if (!can_driver_->send(can_msg)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send chassis command");
  }
}布到 ROS2。
内部维护了 ROS2 的发布者和订阅者，并实现了消息的编码/解码逻辑。
*/

#include "ros2_can_bridge/protocol_parser.hpp"
#include <cstring>
#include <iostream>

namespace ros2_can_bridge
{


// 构造函数，初始化 ROS2 节点和 CAN 驱动指针
ProtocolParser::ProtocolParser(
  const rclcpp::Node::SharedPtr & node,
  std::shared_ptr<CanDriver> can_driver)
: node_(node), can_driver_(can_driver)
{
}

// 初始化发布者、订阅者和 CAN 回调
void ProtocolParser::init()
{
  // 获取仿真模式参数（已经在 node 中声明，这里直接获取）
  if (node_->has_parameter("simulate_hardware")) {
    simulate_hardware_ = node_->get_parameter("simulate_hardware").as_bool();
  } else {
    node_->declare_parameter<bool>("simulate_hardware", false);
    simulate_hardware_ = false;
  }

  // 声明参数，允许通过参数配置话题名称
  node_->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_in_yaw");
  std::string cmd_vel_topic = node_->get_parameter("cmd_vel_topic").as_string();
  RCLCPP_INFO(node_->get_logger(), "Subscribing to %s for chassis control", cmd_vel_topic.c_str());

  // 创建 IMU、里程计、比赛状态的 ROS2 发布者
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  game_state_pub_ = node_->create_publisher<rm_interfaces::msg::GameState>("game_state", 10);

  // 创建底盘速度和射击命令的 ROS2 订阅者（使用经过云台姿态补偿后的速度）
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10, std::bind(&ProtocolParser::on_cmd_vel, this, std::placeholders::_1));

  shoot_cmd_sub_ = node_->create_subscription<rm_interfaces::msg::ShootCmd>(
    "shoot_cmd", 10, std::bind(&ProtocolParser::on_shoot_cmd, this, std::placeholders::_1));

  // 设置 CAN 驱动的接收回调，收到 CAN 帧时自动调用 on_can_receive
  can_driver_->set_receive_callback(
    std::bind(&ProtocolParser::on_can_receive, this, std::placeholders::_1));
}

// CAN 接收回调，根据帧 ID 解码并发布到对应 ROS2 话题
void ProtocolParser::on_can_receive(const CanMessage & message)
{
  switch (message.id) {
    case CAN_ID_IMU_DATA: {
      // 解码 IMU 数据并发布
      auto imu_msg = decode_imu_data(message);
      imu_pub_->publish(imu_msg);
      break;
    }
    case CAN_ID_ODOM_DATA: {
      // 解码里程计数据并发布
      auto odom_msg = decode_odom_data(message);
      odom_pub_->publish(odom_msg);
      break;
    }
    case CAN_ID_GAME_STATE: {
      // 解码比赛状态并发布
      auto game_state_msg = decode_game_state(message);
      game_state_pub_->publish(game_state_msg);
      break;
    }
    case CAN_ID_GIMBAL_STATE:
      // TODO: 实现云台状态解码
      break;
    case CAN_ID_SHOOTER_STATE:
      // TODO: 实现射击机构状态解码
      break;
    default:
      // 未知 ID，打印警告
      RCLCPP_WARN(node_->get_logger(), "Unknown CAN ID: 0x%X", message.id);
      break;
  }
}

// 编码底盘速度指令，将 ROS2 Twist 转为 CAN 帧
CanMessage ProtocolParser::encode_chassis_cmd(const geometry_msgs::msg::Twist & msg)
{
  CanMessage can_msg;
  can_msg.id = CAN_ID_CHASSIS_CMD;
  can_msg.dlc = 8;

  // 编码线速度x和角速度z为float（小端序）
  float_to_can(static_cast<float>(msg.linear.x), can_msg.data, 0);  // vx
  float_to_can(static_cast<float>(msg.angular.z), can_msg.data, 4); // wz

  return can_msg;
}

// 编码射击指令，将 ROS2 ShootCmd 转为 CAN 帧
CanMessage ProtocolParser::encode_shooter_cmd(const rm_interfaces::msg::ShootCmd & msg)
{
  CanMessage can_msg;
  can_msg.id = CAN_ID_SHOOTER_CMD;
  can_msg.dlc = 3;  // 1 字节类型，2 字节弹丸数

  can_msg.data[0] = msg.type;
  uint16_to_can(msg.projectile_num, can_msg.data, 1);

  return can_msg;
}

// 解码 IMU 数据，将 CAN 帧转为 ROS2 Imu 消息
sensor_msgs::msg::Imu ProtocolParser::decode_imu_data(const CanMessage & msg)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = node_->now();
  imu_msg.header.frame_id = "imu_link";

  // 示例：解码四元数 x, y，实际协议需根据 STM32 端定义调整
  imu_msg.orientation.x = float_from_can(msg.data, 0);
  imu_msg.orientation.y = float_from_can(msg.data, 4);
  // 其余分量可在后续帧补充

  return imu_msg;
}

// 解码里程计数据，将 CAN 帧转为 ROS2 Odometry 消息
nav_msgs::msg::Odometry ProtocolParser::decode_odom_data(const CanMessage & msg)
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = node_->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  // 示例：解码位置 x, y，实际协议需根据 STM32 端定义调整
  odom_msg.pose.pose.position.x = float_from_can(msg.data, 0);
  odom_msg.pose.pose.position.y = float_from_can(msg.data, 4);
  // 其余分量可在后续帧补充

  return odom_msg;
}

// 解码比赛状态，将 CAN 帧转为 ROS2 GameState 消息
rm_interfaces::msg::GameState ProtocolParser::decode_game_state(const CanMessage & msg)
{
  rm_interfaces::msg::GameState state_msg;

  // 示例：解码比赛进度、剩余时间、血量，实际协议需根据 STM32 端定义调整
  state_msg.game_progress = msg.data[0];
  state_msg.stage_remain_time = uint16_from_can(msg.data, 1);
  state_msg.current_hp = uint16_from_can(msg.data, 3);
  // 其余字段可在后续帧补充

  return state_msg;
}

// ROS2 订阅回调：收到底盘速度指令，编码为 CAN 帧并发送
void ProtocolParser::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 在仿真模式下只打印日志，不编码和发送
  if (simulate_hardware_) {
    RCLCPP_INFO(node_->get_logger(), 
      "Simulated CAN send: Chassis Command vx=%.2f, wz=%.2f", 
      msg->linear.x, msg->angular.z);
    return;  // 直接返回，不调用 CAN 驱动
  }
  
  // 实际硬件模式
  auto can_msg = encode_chassis_cmd(*msg);
  if (!can_driver_->send(can_msg)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send chassis command");
  }
}

// ROS2 订阅回调：收到射击指令，编码为 CAN 帧并发送
void ProtocolParser::on_shoot_cmd(const rm_interfaces::msg::ShootCmd::SharedPtr msg)
{
  // 在仿真模式下只打印日志，不编码和发送
  if (simulate_hardware_) {
    RCLCPP_INFO(node_->get_logger(), 
      "Simulated CAN send: Shooter Command type=%d, projectile_num=%d", 
      msg->type, msg->projectile_num);
    return;  // 直接返回，不调用 CAN 驱动
  }
  
  // 实际硬件模式
  auto can_msg = encode_shooter_cmd(*msg);
  if (!can_driver_->send(can_msg)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send shooter command");
  }
}

// Utility functions for data conversion

// 工具函数：从 CAN 数据解 float
float ProtocolParser::float_from_can(const uint8_t * data, size_t offset)
{
  float value;
  std::memcpy(&value, &data[offset], sizeof(float));
  return value;
}

// 工具函数：float 写入 CAN 数据
void ProtocolParser::float_to_can(float value, uint8_t * data, size_t offset)
{
  std::memcpy(&data[offset], &value, sizeof(float));
}

// 工具函数：从 CAN 数据解 uint16
uint16_t ProtocolParser::uint16_from_can(const uint8_t * data, size_t offset)
{
  return static_cast<uint16_t>(data[offset]) | 
         (static_cast<uint16_t>(data[offset + 1]) << 8);
}

// 工具函数：uint16 写入 CAN 数据
void ProtocolParser::uint16_to_can(uint16_t value, uint8_t * data, size_t offset)
{
  data[offset] = static_cast<uint8_t>(value & 0xFF);
  data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

}  // namespace ros2_can_bridge
