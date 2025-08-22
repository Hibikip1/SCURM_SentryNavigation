/*
// 这是整个包的主入口节点，负责初始化 CAN 驱动、协议解析器，并与 ROS2 话题进行交互。
// 主要作用是实现 ROS2 <-> CAN 总线的双向桥接。

// 读取参数（如 can_interface，默认 can1）。
// 初始化 CAN 驱动（can_driver），负责底层 CAN 帧的收发。
// 初始化协议解析器（protocol_parser），负责将 ROS2 消息与 CAN 帧互相转换。
// 启动 CAN 接收线程，异步监听 CAN 总线数据。
// 进入 ROS2 spin 循环，处理 ROS2 话题的发布与订阅。
*/

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include "ros2_can_bridge/can_driver.hpp"
#include "ros2_can_bridge/protocol_parser.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("can_bridge_node");

  // Get parameters
  std::string can_interface = node->declare_parameter<std::string>("can_interface", "can1");
  bool simulate_hardware = node->declare_parameter<bool>("simulate_hardware", false);
  
  RCLCPP_INFO(node->get_logger(), "Starting CAN bridge on interface: %s", can_interface.c_str());
  if (simulate_hardware) {
    RCLCPP_INFO(node->get_logger(), "Running in simulation mode (hardware operations will be skipped)");
  }

  // Initialize CAN driver
  auto can_driver = std::make_shared<ros2_can_bridge::CanDriver>(can_interface);
  if (!simulate_hardware && !can_driver->init()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize CAN driver");
    rclcpp::shutdown();
    return 1;
  } else if (simulate_hardware) {
    RCLCPP_INFO(node->get_logger(), "Skipping CAN hardware initialization in simulation mode");
  }

  // Initialize protocol parser
  auto protocol_parser = std::make_shared<ros2_can_bridge::ProtocolParser>(node, can_driver);
  
  // Protocol parser will read the simulate_hardware parameter that was already declared above
  protocol_parser->init();

  // Start CAN receiver thread
  if (!simulate_hardware && !can_driver->start()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to start CAN receiver");
    rclcpp::shutdown();
    return 1;
  } else if (simulate_hardware) {
    RCLCPP_INFO(node->get_logger(), "Skipping CAN receiver thread in simulation mode");
  }

  RCLCPP_INFO(node->get_logger(), "CAN bridge started successfully");

  // Spin node
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  // Stop CAN receiver before exiting
  can_driver->stop();
  
  return 0;
}
