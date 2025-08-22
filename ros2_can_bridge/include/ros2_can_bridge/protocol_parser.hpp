#ifndef ROS2_CAN_BRIDGE_PROTOCOL_PARSER_HPP_
#define ROS2_CAN_BRIDGE_PROTOCOL_PARSER_HPP_

#include <vector>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rm_interfaces/msg/game_state.hpp>
#include <rm_interfaces/msg/shoot_cmd.hpp>
#include "ros2_can_bridge/can_driver.hpp"

namespace ros2_can_bridge
{

// CAN Frame IDs (customize based on your protocol)
enum CanFrameId : uint32_t {
  // Received from STM32
  CAN_ID_IMU_DATA = 0x100,
  CAN_ID_ODOM_DATA = 0x101,
  CAN_ID_GAME_STATE = 0x102,
  CAN_ID_GIMBAL_STATE = 0x103,
  CAN_ID_SHOOTER_STATE = 0x104,

  // Sent to STM32
  CAN_ID_CHASSIS_CMD = 0x200,
  CAN_ID_GIMBAL_CMD = 0x201,
  CAN_ID_SHOOTER_CMD = 0x202,
  CAN_ID_MODE_SWITCH = 0x203,
  CAN_ID_SYSTEM_CMD = 0x204,
};

/**
 * @class ProtocolParser
 * @brief Parser class to handle CAN protocol for STM32 communication
 */
class ProtocolParser
{
public:
  /**
   * @brief Constructor
   * @param node ROS2 node handle
   * @param can_driver CAN driver instance
   */
  ProtocolParser(
    const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<CanDriver> can_driver);

  /**
   * @brief Initialize publishers and subscribers
   */
  void init();

  /**
   * @brief CAN message receive callback
   * @param message Received CAN message
   */
  void on_can_receive(const CanMessage & message);

  // Encode methods: Convert ROS messages to CAN messages

  /**
   * @brief Encode chassis command
   * @param msg Twist message
   * @return Encoded CAN message
   */
  CanMessage encode_chassis_cmd(const geometry_msgs::msg::Twist & msg);

  /**
   * @brief Encode shooter command
   * @param msg Shoot command message
   * @return Encoded CAN message
   */
  CanMessage encode_shooter_cmd(const rm_interfaces::msg::ShootCmd & msg);

  // Decode methods: Convert CAN messages to ROS messages

  /**
   * @brief Decode IMU data
   * @param msg CAN message
   * @return Decoded IMU message
   */
  sensor_msgs::msg::Imu decode_imu_data(const CanMessage & msg);

  /**
   * @brief Decode odometry data
   * @param msg CAN message
   * @return Decoded odometry message
   */
  nav_msgs::msg::Odometry decode_odom_data(const CanMessage & msg);

  /**
   * @brief Decode game state data
   * @param msg CAN message
   * @return Decoded game state message
   */
  rm_interfaces::msg::GameState decode_game_state(const CanMessage & msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<CanDriver> can_driver_;
  bool simulate_hardware_;  // 仿真模式标志

  // ROS Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr game_state_pub_;

  // ROS Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<rm_interfaces::msg::ShootCmd>::SharedPtr shoot_cmd_sub_;

  // Callback functions for ROS subscribers
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_shoot_cmd(const rm_interfaces::msg::ShootCmd::SharedPtr msg);

  // Utility functions
  float float_from_can(const uint8_t * data, size_t offset);
  void float_to_can(float value, uint8_t * data, size_t offset);
  uint16_t uint16_from_can(const uint8_t * data, size_t offset);
  void uint16_to_can(uint16_t value, uint8_t * data, size_t offset);
};

}  // namespace ros2_can_bridge

#endif  // ROS2_CAN_BRIDGE_PROTOCOL_PARSER_HPP_
