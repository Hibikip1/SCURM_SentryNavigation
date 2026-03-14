#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <rm_interfaces/msg/chassis_cmd.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rm_interfaces/msg/game_state.hpp>
#include <vector>
#include <chrono>
#include <cstdio>
#include <thread>
#include "pub_user.h"

using namespace std::chrono_literals;

// ✅ 前向声明
class SentinelCanNode;

// ✅ 全局指针（用于回调函数访问节点）
static SentinelCanNode* g_node_instance = nullptr;

// ✅ 全局回调函数
void sent_callback(usb_rx_frame_t* frame)
{
    // printf("Sent callback, packet id: 0x%x\n", frame->head.can_id);
}

void rec_callback(usb_rx_frame_t* frame);  // 稍后实现

class SentinelCanNode : public rclcpp::Node
{
public:
  SentinelCanNode()
  : Node("rm_can"), can_opened_(false), handle_(nullptr), dev_(nullptr)
  {
    g_node_instance = this;  // ✅ 设置全局指针
    
    // ...existing code...（参数声明等）
    this->declare_parameter<std::string>("cmd_vel_topic", "chassis_cmd");
    this->declare_parameter<int>("chassis_cmd_id", 0x520);
    this->declare_parameter<int>("mode_switch_id", 0x203);
    this->declare_parameter<std::vector<long int>>("referee_ids", std::vector<long int>{0x400});

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    chassis_cmd_id_ = this->get_parameter("chassis_cmd_id").as_int();
    mode_switch_id_ = this->get_parameter("mode_switch_id").as_int();
    referee_ids_ = this->get_parameter("referee_ids").as_integer_array();

    RCLCPP_INFO(this->get_logger(), "Sentinel CAN node started");

    if (!init_can_device()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN device");
      return;
    }

    cmd_vel_sub_ = this->create_subscription<rm_interfaces::msg::ChassisCmd>(
      cmd_vel_topic_, 10,
      std::bind(&SentinelCanNode::on_cmd_vel, this, std::placeholders::_1));

    mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "sentry_mode_cmd", 10,
      std::bind(&SentinelCanNode::on_mode_cmd, this, std::placeholders::_1));

    game_state_pub_ = this->create_publisher<rm_interfaces::msg::GameState>("game_state", 10);

    RCLCPP_INFO(this->get_logger(), "chassis_cmd_id=0x%X, mode_switch_id=0x%X", chassis_cmd_id_, mode_switch_id_);
  }

  ~SentinelCanNode() override
  {
    g_node_instance = nullptr;  // ✅ 清空全局指针
    
    if (can_opened_ && dev_) {
      device_close_channel(dev_, 0);
      device_close(dev_);
      RCLCPP_INFO(this->get_logger(), "CAN 设备已关闭");
    }
    if (handle_) {
      damiao_handle_destroy(handle_);
    }
  }

  // ✅ 提供公有访问接口
  const std::vector<long int>& get_referee_ids() const { return referee_ids_; }
  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr get_pub() { return game_state_pub_; }

private:
  // ...existing code...
  bool init_can_device()
  {
    handle_ = damiao_handle_create(DEV_USB2CANFD);
    if (!handle_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create damiao handle");
      return false;
    }

    damiao_print_version(handle_);

    int device_cnt = damiao_handle_find_devices(handle_);
    if (device_cnt == 0) {
      RCLCPP_ERROR(this->get_logger(), "No CAN device found!");
      damiao_handle_destroy(handle_);
      return false;
    }

    device_handle* dev_list[16];
    int handle_cnt = 0;
    damiao_handle_get_devices(handle_, dev_list, &handle_cnt);
    
    if (handle_cnt == 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get devices");
      damiao_handle_destroy(handle_);
      return false;
    }

    dev_ = dev_list[0];

    if (!device_open(dev_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CAN device");
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      dev_ = nullptr;
      return false;
    }

    char strBuf[255] = {0};
    device_get_version(dev_, strBuf, sizeof(strBuf));
    RCLCPP_INFO(this->get_logger(), "Device version: %s", strBuf);
    
    device_get_serial_number(dev_, strBuf, sizeof(strBuf));
    RCLCPP_INFO(this->get_logger(), "Device SN: %s", strBuf);

    // ✅ 设置波特率
    device_channel_set_baud_with_sp(dev_, 0, false, 1000000, 1000000, 0.75f, 0.75f);

    if (!device_open_channel(dev_, 0)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open channel 0");
      device_close(dev_);
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      dev_ = nullptr;
      return false;
    }

    // ✅ 注册全局回调函数
    device_hook_to_sent(dev_, sent_callback);
    device_hook_to_rec(dev_, rec_callback);

    can_opened_ = true;
    RCLCPP_INFO(this->get_logger(), "CAN device initialized successfully");
    return true;
  }

  // ...existing code...（其他成员函数）
  std::string cmd_vel_topic_;
  int chassis_cmd_id_;
  int mode_switch_id_;
  std::vector<long int> referee_ids_;

  bool can_opened_;
  damiao_handle* handle_;
  device_handle* dev_;

  rclcpp::Subscription<rm_interfaces::msg::ChassisCmd>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr game_state_pub_;

  void on_cmd_vel(const rm_interfaces::msg::ChassisCmd::SharedPtr msg)
  {
    if (!can_opened_ || !dev_) return;

    const float scale = 1000.0f;
    const auto & twist = msg->twist;
    int16_t vx = static_cast<int16_t>(std::round(twist.linear.x * scale));
    int16_t vy = static_cast<int16_t>(std::round(twist.linear.y * scale));
    int16_t wz = static_cast<int16_t>(std::round(twist.angular.z * scale));

    uint8_t data[8] = {0};
    data[0] = msg->type;
    data[1] = 0;
    data[2] = (vx >> 8) & 0xFF;
    data[3] = vx & 0xFF;
    data[4] = (vy >> 8) & 0xFF;
    data[5] = vy & 0xFF;
    data[6] = (wz >> 8) & 0xFF;
    data[7] = wz & 0xFF;

    device_channel_send_fast(dev_, 0, chassis_cmd_id_, 1, false, false, false, 8, data);
  }

  void on_mode_cmd(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    // ...existing code...
    if (!can_opened_ || !dev_) return;
    
    uint8_t data[8] = {0};
    data[0] = msg->data;
    
    device_channel_send_fast(dev_, 0, mode_switch_id_, 1, false, false, false, 8, data);
  }
};

void rec_callback(usb_rx_frame_t* frame)
{
    if (!frame || !g_node_instance) return;
    
    const uint32_t can_id = frame->head.can_id;
    
    auto& ids = g_node_instance->get_referee_ids();
    for (size_t i = 0; i < ids.size(); ++i) {
        if (can_id == static_cast<uint32_t>(ids[i])) {
            if (frame->head.dlc < 3) {
                RCLCPP_WARN(g_node_instance->get_logger(), 
                    "Invalid data length: %d for ID 0x%X", frame->head.dlc, can_id);
                return;
            }
            
            if (i == 0) {  
                uint8_t game_progress = frame->payload[0];
                uint16_t current_hp = static_cast<uint16_t>(
                    frame->payload[1] | (frame->payload[2] << 8)
                );
                
                auto msg = rm_interfaces::msg::GameState();
                msg.game_progress = game_progress;
                msg.current_hp = current_hp;
                
                g_node_instance->get_pub()->publish(msg);
                
                RCLCPP_INFO(g_node_instance->get_logger(), 
                    "Game state: progress=%d, hp=%d", game_progress, current_hp);
            }
            break;
        }
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SentinelCanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}