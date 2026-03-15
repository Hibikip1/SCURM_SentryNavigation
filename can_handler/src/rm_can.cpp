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
#include "pub_user.h"

using namespace std::chrono_literals;

// ✅ 前向声明
class SentinelCanNode;

// ✅ 全局指针（用于回调函数访问节点）
static std::atomic<SentinelCanNode *> g_node_instance{nullptr};

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
    this->declare_parameter<int>("can_tx_channel", 0);
    this->declare_parameter<int>("referee_can_channel", 0);  // 只处理该通道的裁判数据；-1=两通道都收
    this->declare_parameter<std::vector<long int>>("referee_ids", std::vector<long int>{0x400});

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    chassis_cmd_id_ = this->get_parameter("chassis_cmd_id").as_int();
    mode_switch_id_ = this->get_parameter("mode_switch_id").as_int();
    can_device_index_ = this->get_parameter("can_device_index").as_int();
    can_tx_channel_ = this->get_parameter("can_tx_channel").as_int();
    referee_can_channel_ = this->get_parameter("referee_can_channel").as_int();
    referee_ids_ = this->get_parameter("referee_ids").as_integer_array();

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

    RCLCPP_INFO(this->get_logger(), "chassis_cmd_id=0x%X, mode_switch_id=0x%X, can_device_index=%d, can_tx_channel=%d, referee_can_channel=%d, referee_ids=[%s]",
                chassis_cmd_id_, mode_switch_id_, can_device_index_, can_tx_channel_, referee_can_channel_, id_list.c_str());
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

  // 同通道下两条 0x400 时：连续 20 次相同才锁定该源，锁定后该状态每帧都发布（快速连续且无杂数据）
  bool publish_game_state(uint8_t channel, const rm_interfaces::msg::GameState &msg)
  {
    if (referee_can_channel_ >= 0 && static_cast<uint8_t>(referee_can_channel_) != channel)
      return false;
    std::lock_guard<std::mutex> lock(game_state_mutex_);
    int gp = static_cast<int>(msg.game_progress);
    int hp = static_cast<int>(msg.current_hp);
    int al = static_cast<int>(msg.alive_status);
    int ar = static_cast<int>(msg.armor_state);
    bool same_as_candidate = (candidate_progress_ == gp && candidate_hp_ == hp && candidate_alive_ == al && candidate_armor_ == ar);
    if (!same_as_candidate)
    {
      candidate_progress_ = gp;
      candidate_hp_ = hp;
      candidate_alive_ = al;
      candidate_armor_ = ar;
      candidate_count_ = 1;
      locked_ = false;
      return false;
    }
    candidate_count_++;
    if (!locked_)
    {
      if (candidate_count_ < kGameStateStableCount)
        return false;
      locked_ = true;  // 连续 20 次相同，锁定该源
    }
    game_state_pub_->publish(msg);
    return true;
  }

private:
  static constexpr int kGameStateStableCount = 20;

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

    if (can_device_index_ < 0 || can_device_index_ >= handle_cnt || !dev_list[can_device_index_])
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid can_device_index=%d, discovered device count=%d (type=%d)",
                   can_device_index_, handle_cnt, static_cast<int>(type));
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      return false;
    }

    dev_ = dev_list[can_device_index_];

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
    RCLCPP_INFO(this->get_logger(), "Device SN: %s", strBuf);

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
  int can_tx_channel_;
  int referee_can_channel_;
  std::vector<long int> referee_ids_;
  bool channel_opened_[2]{false, false};

  std::mutex game_state_mutex_;
  int candidate_progress_{-1};
  int candidate_hp_{-1};
  int candidate_alive_{-1};
  int candidate_armor_{-1};
  int candidate_count_{0};
  bool locked_{false};

  std::atomic<bool> can_opened_;
  std::atomic<bool> params_loaded_;
  damiao_handle *handle_;
  device_handle *dev_;
  std::mutex io_mutex_;

  static constexpr std::chrono::milliseconds kChassisSendMinInterval{25};  // 40Hz 上限，避免 send box 缓冲区满
  static constexpr std::chrono::milliseconds kModeSendMinInterval{80};
  std::chrono::steady_clock::time_point last_chassis_send_time_{};
  std::chrono::steady_clock::time_point last_mode_send_time_{};

  rclcpp::Subscription<rm_interfaces::msg::ChassisCmd>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr game_state_pub_;

  void on_cmd_vel(const rm_interfaces::msg::ChassisCmd::SharedPtr msg)
  {
    if (!can_opened_.load(std::memory_order_acquire) || !dev_)
      return;

    const float scale = 1000.0f;
    const auto &twist = msg->twist;
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

    std::lock_guard<std::mutex> lock(io_mutex_);
    auto now = std::chrono::steady_clock::now();
    if (now - last_chassis_send_time_ < kChassisSendMinInterval)
      return;
    last_chassis_send_time_ = now;
    device_channel_send_fast(dev_, static_cast<uint8_t>(can_tx_channel_), chassis_cmd_id_, 1, false, false, false, 8, data);
    RCLCPP_INFO(this->get_logger(), "[SEND_REQ] id=0x%X dlc=8 data=%02X %02X %02X %02X %02X %02X %02X %02X",
                chassis_cmd_id_, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }

  void on_mode_cmd(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (!can_opened_.load(std::memory_order_acquire) || !dev_)
      return;

    uint8_t data[8] = {0};
    data[0] = msg->data;

    std::lock_guard<std::mutex> lock(io_mutex_);
    auto now = std::chrono::steady_clock::now();
    if (now - last_mode_send_time_ < kModeSendMinInterval)
      return;
    last_mode_send_time_ = now;
    device_channel_send_fast(dev_, static_cast<uint8_t>(can_tx_channel_), mode_switch_id_, 1, false, false, false, 8, data);
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