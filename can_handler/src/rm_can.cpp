#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <rm_interfaces/msg/chassis_cmd.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rm_interfaces/msg/game_state.hpp>
#include <vector>
#include <chrono>
#include <cstdio>
#include <thread>
#include <atomic>
#include <mutex>
#include <algorithm>
#include <cstdlib>
#include <cctype>
#include "pub_user.h"

using namespace std::chrono_literals;

// ✅ 前向声明
class SentinelCanNode;

// ✅ 全局指针（用于回调函数访问节点）
static std::atomic<SentinelCanNode *> g_node_instance{nullptr};

namespace
{
std::string trim_copy(const std::string &s)
{
  auto begin = std::find_if_not(s.begin(), s.end(), [](unsigned char ch)
                                { return std::isspace(ch); });
  auto end = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char ch)
                              { return std::isspace(ch); })
                 .base();
  if (begin >= end)
    return std::string();
  return std::string(begin, end);
}
}

// ✅ 全局回调函数
void sent_callback(usb_rx_frame_t *frame)
{
  if (!frame)
    return;
  SentinelCanNode *node = g_node_instance.load(std::memory_order_acquire);
  // if (!node)
  // {
  //   // fallback to stdout if node not ready
  //   printf("[SENT] id=0x%X ch=%u dlc=%u: ", frame->head.can_id, (unsigned)frame->head.channel, (unsigned)frame->head.dlc);
  //   for (unsigned i = 0; i < frame->head.dlc && i < 64; ++i)
  //     printf("%02X ", frame->payload[i]);
  //   printf("\n");
  //   return;
  // }

  // Use node logger to print confirmed send
  std::string hex;
  char buf[8];
  for (unsigned i = 0; i < frame->head.dlc && i < 64; ++i)
  {
    snprintf(buf, sizeof(buf), "%02X", frame->payload[i]);
    if (!hex.empty())
      hex += ' ';
    hex += buf;
  }
  // RCLCPP_INFO(rclcpp::get_logger("rm_can"), "[SENT] id=0x%X ch=%u dlc=%u data=%s",
  //             frame->head.can_id,
  //             static_cast<unsigned>(frame->head.channel),
  //             static_cast<unsigned>(frame->head.dlc),
  //             hex.c_str());
}

void rec_callback(usb_rx_frame_t *frame); // 稍后实现
void err_callback(usb_rx_frame_t *frame);

class SentinelCanNode : public rclcpp::Node
{
public:
  SentinelCanNode()
      : Node("rm_can"), can_opened_(false), params_loaded_(false), handle_(nullptr), dev_(nullptr)
  {
    g_node_instance.store(this, std::memory_order_release); // ✅ 设置全局指针

    // ...existing code...（参数声明等）
    this->declare_parameter<std::string>("cmd_vel_topic", "chassis_cmd");
    this->declare_parameter<int>("chassis_cmd_id", 0x520);
    this->declare_parameter<int>("mode_switch_id", 0x203);
    this->declare_parameter<int>("can_device_index", 0);
    this->declare_parameter<std::string>("can_device_sn", "");
    this->declare_parameter<int>("can_tx_channel", 0);
    this->declare_parameter<int>("referee_can_channel", 0);  // 只处理该通道的裁判数据；-1=两通道都收
    this->declare_parameter<std::vector<long int>>("referee_ids", std::vector<long int>{0x400});

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    chassis_cmd_id_ = this->get_parameter("chassis_cmd_id").as_int();
    mode_switch_id_ = this->get_parameter("mode_switch_id").as_int();
    can_device_index_ = this->get_parameter("can_device_index").as_int();
    can_device_sn_ = trim_copy(this->get_parameter("can_device_sn").as_string());
    can_tx_channel_ = this->get_parameter("can_tx_channel").as_int();
    referee_can_channel_ = this->get_parameter("referee_can_channel").as_int();
    referee_ids_ = this->get_parameter("referee_ids").as_integer_array();

    if (const char *env_sn = std::getenv("DM_CAN_DEVICE_SN"))
    {
      const auto sn = trim_copy(env_sn);
      if (!sn.empty())
      {
        can_device_sn_ = sn;
      }
    }

    // if (can_tx_channel_ < 0 || can_tx_channel_ > 1)
    // {
    //   RCLCPP_WARN(this->get_logger(), "Invalid can_tx_channel=%d, fallback to 0", can_tx_channel_);
    //   can_tx_channel_ = 0;
    // }

    RCLCPP_INFO(this->get_logger(), "Sentinel CAN node started");

    if (!init_can_device())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN device");
      return;
    }

    cmd_vel_sub_ = this->create_subscription<rm_interfaces::msg::ChassisCmd>(
        cmd_vel_topic_, 10,
        std::bind(&SentinelCanNode::on_cmd_vel, this, std::placeholders::_1));

    chassis_tx_timer_ = this->create_wall_timer(
      50ms,
      std::bind(&SentinelCanNode::on_chassis_tx_timer, this));

    mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "sentry_mode_cmd", 10,
        std::bind(&SentinelCanNode::on_mode_cmd, this, std::placeholders::_1));

    game_state_pub_ = this->create_publisher<rm_interfaces::msg::GameState>("game_state", 10);

    params_loaded_.store(true, std::memory_order_release);
    std::string id_list;
    for (size_t i = 0; i < referee_ids_.size(); ++i)
    {
      char idbuf[16];
      snprintf(idbuf, sizeof(idbuf), "0x%lX", referee_ids_[i]);
      if (!id_list.empty())
        id_list += ",";
      id_list += idbuf;
    }

    RCLCPP_INFO(this->get_logger(), "chassis_cmd_id=0x%X, mode_switch_id=0x%X, can_device_index=%d, can_device_sn=%s, can_tx_channel=%d, referee_can_channel=%d, referee_ids=[%s]",
          chassis_cmd_id_, mode_switch_id_, can_device_index_, can_device_sn_.empty() ? "<empty>" : can_device_sn_.c_str(), can_tx_channel_, referee_can_channel_, id_list.c_str());
    if (referee_can_channel_ >= 0)
      RCLCPP_INFO(this->get_logger(), "0x400 仅接收通道 %d，另一条线的 0x400 不接收", referee_can_channel_);
  }

  ~SentinelCanNode() override
  {
    g_node_instance.store(nullptr, std::memory_order_release); // ✅ 清空全局指针
    can_opened_.store(false, std::memory_order_release);

    if (can_opened_ && dev_)
    {
      if (channel_opened_[0])
        device_close_channel(dev_, 0);
      if (channel_opened_[1])
        device_close_channel(dev_, 1);
      device_close(dev_);
      RCLCPP_INFO(this->get_logger(), "CAN 设备已关闭");
    }
    if (handle_)
    {
      damiao_handle_destroy(handle_);
    }
  }

  // ✅ 提供公有访问接口
  const std::vector<long int> &get_referee_ids() const { return referee_ids_; }
  int get_referee_can_channel() const { return referee_can_channel_; }
  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr get_pub() { return game_state_pub_; }
  bool is_params_loaded() const { return params_loaded_.load(std::memory_order_acquire); }
  bool is_referee_id(uint32_t can_id) const
  {
    return std::any_of(referee_ids_.begin(), referee_ids_.end(),
                       [can_id](long int id)
                       { return static_cast<uint32_t>(id) == can_id; });
  }

  bool publish_game_state(uint8_t channel, const rm_interfaces::msg::GameState &msg)
  {
    if (referee_can_channel_ >= 0 && static_cast<uint8_t>(referee_can_channel_) != channel)
      return false;
    game_state_pub_->publish(msg);
    return true;
  }

private:
  bool init_can_device_with_type(device_def_t type)
  {
    handle_ = damiao_handle_create(type);
    if (!handle_)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create damiao handle for type=%d", static_cast<int>(type));
      return false;
    }

    damiao_print_version(handle_);

    int device_cnt = damiao_handle_find_devices(handle_);
    if (device_cnt <= 0)
    {
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      return false;
    }

    device_handle *dev_list[16] = {nullptr};
    int handle_cnt = 0;
    damiao_handle_get_devices(handle_, dev_list, &handle_cnt);

    if (handle_cnt <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get devices for type=%d", static_cast<int>(type));
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Found %d CAN device(s)", handle_cnt);
    for (int i = 0; i < handle_cnt; ++i)
    {
      char serial[255] = {0};
      device_get_serial_number(dev_list[i], serial, sizeof(serial));
      RCLCPP_INFO(this->get_logger(), "  device[%d] sn=%s", i, serial);
    }

    int selected_index = -1;
    if (!can_device_sn_.empty())
    {
      for (int i = 0; i < handle_cnt; ++i)
      {
        char serial[255] = {0};
        device_get_serial_number(dev_list[i], serial, sizeof(serial));
        if (can_device_sn_ == serial)
        {
          selected_index = i;
          break;
        }
      }

      if (selected_index < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "No device matched can_device_sn=%s", can_device_sn_.c_str());
        damiao_handle_destroy(handle_);
        handle_ = nullptr;
        return false;
      }
    }
    else
    {
      selected_index = can_device_index_;
      if (selected_index < 0 || selected_index >= handle_cnt || !dev_list[selected_index])
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid can_device_index=%d, discovered device count=%d (type=%d)",
                     can_device_index_, handle_cnt, static_cast<int>(type));
        damiao_handle_destroy(handle_);
        handle_ = nullptr;
        return false;
      }
      if (handle_cnt > 1)
      {
        RCLCPP_WARN(this->get_logger(), "Multiple devices found but can_device_sn is empty; using index=%d", selected_index);
      }
    }

    dev_ = dev_list[selected_index];

    if (!device_open(dev_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CAN device for type=%d", static_cast<int>(type));
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      dev_ = nullptr;
      return false;
    }

    char strBuf[255] = {0};
    device_get_version(dev_, strBuf, sizeof(strBuf));
    RCLCPP_INFO(this->get_logger(), "Device version: %s", strBuf);

    device_get_serial_number(dev_, strBuf, sizeof(strBuf));
    RCLCPP_INFO(this->get_logger(), "Selected device[%d] SN: %s", selected_index, strBuf);

    if (!device_channel_set_baud_with_sp(dev_, 0, false, 1000000, 1000000, 0.75f, 0.75f))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to set channel baudrate");
      device_close(dev_);
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      dev_ = nullptr;
      return false;
    }

    if (!device_open_channel(dev_, 0))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open channel 0");
      device_close(dev_);
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      dev_ = nullptr;
      return false;
    }
    channel_opened_[0] = true;

    if (device_channel_set_baud_with_sp(dev_, 1, false, 1000000, 1000000, 0.75f, 0.75f) &&
        device_open_channel(dev_, 1))
    {
      channel_opened_[1] = true;
      RCLCPP_INFO(this->get_logger(), "Channel 1 opened for RX/TX");
    }
    else
    {
      channel_opened_[1] = false;
      RCLCPP_INFO(this->get_logger(), "Channel 1 not opened (single-channel device or unavailable)");
    }

    device_hook_to_sent(dev_, sent_callback);
    device_hook_to_rec(dev_, rec_callback);
    device_hook_to_err(dev_, err_callback);

    RCLCPP_INFO(this->get_logger(), "CAN device opened by type=%d", static_cast<int>(type));
    return true;
  }

  // ...existing code...
  bool init_can_device()
  {
    if (init_can_device_with_type(DEV_USB2CANFD_DUAL) || init_can_device_with_type(DEV_USB2CANFD))
    {
      can_opened_.store(true, std::memory_order_release);
      RCLCPP_INFO(this->get_logger(), "CAN device initialized successfully");
      return true;
    }

    RCLCPP_ERROR(this->get_logger(), "No CAN device found or channel open failed (tried DUAL then SINGLE)");
    return false;
  }

  // ...existing code...（其他成员函数）
  std::string cmd_vel_topic_;
  int chassis_cmd_id_;
  int mode_switch_id_;
  int can_device_index_;
  std::string can_device_sn_;
  int can_tx_channel_;
  int referee_can_channel_;
  std::vector<long int> referee_ids_;
  bool channel_opened_[2]{false, false};

  std::atomic<bool> can_opened_;
  std::atomic<bool> params_loaded_;
  damiao_handle *handle_;
  device_handle *dev_;
  std::mutex io_mutex_;
  std::mutex cmd_mutex_;
  rm_interfaces::msg::ChassisCmd latest_cmd_;
  bool has_latest_cmd_{false};

  rclcpp::Subscription<rm_interfaces::msg::ChassisCmd>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr game_state_pub_;
  rclcpp::TimerBase::SharedPtr chassis_tx_timer_;

  void on_cmd_vel(const rm_interfaces::msg::ChassisCmd::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_cmd_ = *msg;
    has_latest_cmd_ = true;
  }

  void on_chassis_tx_timer()
  {
    if (!can_opened_.load(std::memory_order_acquire) || !dev_)
      return;

    rm_interfaces::msg::ChassisCmd cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (!has_latest_cmd_)
        return;
      cmd = latest_cmd_;
    }

    const float scale = 1000.0f;
    const auto &twist = cmd.twist;
    int16_t vx = static_cast<int16_t>(std::round(twist.linear.x * scale));
    int16_t vy = static_cast<int16_t>(std::round(twist.linear.y * scale));
    int16_t wz = static_cast<int16_t>(std::round(twist.angular.z * scale));

    uint8_t data[8] = {0};
    data[0] = cmd.type;
    data[1] = 0;
    data[2] = (vx >> 8) & 0xFF;
    data[3] = vx & 0xFF;
    data[4] = (vy >> 8) & 0xFF;
    data[5] = vy & 0xFF;
    data[6] = (wz >> 8) & 0xFF;
    data[7] = wz & 0xFF;

    std::lock_guard<std::mutex> lock(io_mutex_);
    device_channel_send_fast(dev_, can_tx_channel_, chassis_cmd_id_, 1, false, false, false, 8, data);
    RCLCPP_INFO(this->get_logger(), "[SEND_REQ@10Hz] id=0x%X ch=%d dlc=8 data=%02X %02X %02X %02X %02X %02X %02X %02X",
                chassis_cmd_id_, can_tx_channel_, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }

  void on_mode_cmd(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (!can_opened_.load(std::memory_order_acquire) || !dev_)
      return;

    uint8_t data[8] = {0};
    data[0] = msg->data;

    std::lock_guard<std::mutex> lock(io_mutex_);
    device_channel_send_fast(dev_, can_tx_channel_, mode_switch_id_, 1, false, false, false, 8, data);
    RCLCPP_INFO(this->get_logger(), "[SEND_REQ] id=0x%X dlc=8 data=%02X %02X %02X %02X %02X %02X %02X %02X",
                mode_switch_id_, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }
};

void rec_callback(usb_rx_frame_t *frame)
{
  if (!frame)
    return;

  // 打印所有接收的 CAN 帧（用于诊断/确认已收到）
  {
    std::string hex;
    char buf[8];
    for (unsigned i = 0; i < frame->head.dlc && i < 64; ++i)
    {
      snprintf(buf, sizeof(buf), "%02X", frame->payload[i]);
      if (!hex.empty())
        hex += ' ';
      hex += buf;
    }
    // RCLCPP_INFO(rclcpp::get_logger("rm_can"), "[REC] ID=0x%X ch=%u dlc=%u ext=%u rtr=%u canfd=%u dir=%u ack=%u data=%s",
    //             frame->head.can_id,
    //             static_cast<unsigned>(frame->head.channel),
    //             static_cast<unsigned>(frame->head.dlc),
    //       static_cast<unsigned>(frame->head.ext),
    //       static_cast<unsigned>(frame->head.rtr),
    //       static_cast<unsigned>(frame->head.canfd),
    //       static_cast<unsigned>(frame->head.dir),
    //       static_cast<unsigned>(frame->head.ack),
    //             hex.c_str());
  }

  SentinelCanNode *node = g_node_instance.load(std::memory_order_acquire);
  if (!node || !node->is_params_loaded())
    return;

  if (!node->is_referee_id(frame->head.can_id))
    return;

  // 只接收指定通道的 0x400，另一条线的 0x400 直接不接收
  int ref_ch = node->get_referee_can_channel();
  if (ref_ch >= 0 && static_cast<unsigned>(frame->head.channel) != static_cast<unsigned>(ref_ch))
    return;

  if (frame->head.dlc < 3)
  {
    RCLCPP_WARN(node->get_logger(), "Matched referee ID=0x%X but DLC=%u < 3",
                frame->head.can_id, static_cast<unsigned>(frame->head.dlc));
    return;
  }

  uint8_t game_progress = frame->payload[0];
  uint16_t current_hp = static_cast<uint16_t>(
      frame->payload[1] | (frame->payload[2] << 8));
  uint8_t alive_status = (frame->head.dlc >= 4) ? frame->payload[3] : 0;
  uint8_t armor_state = (frame->head.dlc >= 5) ? frame->payload[4] : 0;

  auto msg = rm_interfaces::msg::GameState();
  msg.game_progress = game_progress;
  msg.current_hp = current_hp;
  msg.alive_status = alive_status;
  msg.armor_state = armor_state;

  if (node->publish_game_state(frame->head.channel, msg))
    RCLCPP_INFO(node->get_logger(), "[PUB] ID=0x%X ch=%u -> GameState: progress=%d, hp=%d, alive=%d, armor_state=%d",
                frame->head.can_id, static_cast<unsigned>(frame->head.channel), game_progress, current_hp, alive_status, armor_state);
}

void err_callback(usb_rx_frame_t *frame)
{
  if (!frame)
    return;

  SentinelCanNode *node = g_node_instance.load(std::memory_order_acquire);
  if (!node)
    return;

  // RCLCPP_WARN(node->get_logger(),
  //             "CAN error frame: id=0x%X ch=%u dlc=%u canfd=%u ack=%u",
  //             frame->head.can_id,
  //             static_cast<unsigned>(frame->head.channel),
  //             static_cast<unsigned>(frame->head.dlc),
  //             static_cast<unsigned>(frame->head.canfd),
  //             static_cast<unsigned>(frame->head.ack));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SentinelCanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 