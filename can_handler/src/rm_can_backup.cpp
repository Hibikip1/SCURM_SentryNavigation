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
#include "pub_user.h"

using namespace std::chrono_literals;

// ✅ 前向声明
class SentinelCanNode;

// ✅ 全局指针（用于回调函数访问节点）
static std::atomic<SentinelCanNode*> g_node_instance{nullptr};

// ✅ 全局回调函数
void sent_callback(usb_rx_frame_t* frame)
{
    // printf("Sent callback, packet id: 0x%x\n", frame->head.can_id);
}

void rec_callback(usb_rx_frame_t* frame);  // 稍后实现
void err_callback(usb_rx_frame_t* frame);

class SentinelCanNode : public rclcpp::Node
{
public:
  SentinelCanNode()
  : Node("rm_can"), can_opened_(false), handle_(nullptr), dev_(nullptr), params_loaded_(false)
  {
    g_node_instance.store(this, std::memory_order_release);  // ✅ 设置全局指针
    
    // ...existing code...（参数声明等）
    this->declare_parameter<std::string>("cmd_vel_topic", "chassis_cmd");
    this->declare_parameter<int>("chassis_cmd_id", 0x520);
    this->declare_parameter<int>("mode_switch_id", 0x203);
    this->declare_parameter<std::vector<long int>>("referee_ids", std::vector<long int>{0x301, 0x302, 0x303});

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    chassis_cmd_id_ = this->get_parameter("chassis_cmd_id").as_int();
    mode_switch_id_ = this->get_parameter("mode_switch_id").as_int();
    referee_ids_ = this->get_parameter("referee_ids").as_integer_array();
    
    printf("[DEBUG] Parameters loaded: cmd_vel_topic=%s, chassis_cmd_id=0x%X, mode_switch_id=0x%X, num_referee_ids=%zu\n",
           cmd_vel_topic_.c_str(), chassis_cmd_id_, mode_switch_id_, referee_ids_.size());
    for (size_t i = 0; i < referee_ids_.size(); i++) {
        printf("[DEBUG]   referee_ids[%zu] = 0x%X\n", i, static_cast<unsigned int>(referee_ids_[i]));
    }

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

    params_loaded_.store(true, std::memory_order_release);  // ✅ 标记参数加载完成
    printf("[DEBUG] All initialization complete and params marked ready\n");
    RCLCPP_INFO(this->get_logger(), "chassis_cmd_id=0x%X, mode_switch_id=0x%X", chassis_cmd_id_, mode_switch_id_);
  }

  ~SentinelCanNode() override
  {
    g_node_instance.store(nullptr, std::memory_order_release);  // ✅ 清空全局指针
    can_opened_.store(false, std::memory_order_release);
    
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
  bool is_params_loaded() const { return params_loaded_.load(std::memory_order_acquire); }

private:
  bool init_can_device_with_type(device_def_t type)
  {
    printf("[DEBUG] Trying device type %d...\n", static_cast<int>(type));
    
    handle_ = damiao_handle_create(type);
    if (!handle_) {
      printf("[DEBUG] Failed to create damiao handle for type=%d\n", static_cast<int>(type));
      RCLCPP_ERROR(this->get_logger(), "Failed to create damiao handle for type=%d", static_cast<int>(type));
      return false;
    }
    printf("[DEBUG] Handle created successfully\n");

    damiao_print_version(handle_);

    int device_cnt = damiao_handle_find_devices(handle_);
    printf("[DEBUG] Found %d devices of type %d\n", device_cnt, static_cast<int>(type));
    if (device_cnt <= 0) {
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      return false;
    }

    device_handle* dev_list[16] = {nullptr};
    int handle_cnt = 0;
    damiao_handle_get_devices(handle_, dev_list, &handle_cnt);
    printf("[DEBUG] Got %d device handles\n", handle_cnt);

    if (handle_cnt <= 0 || !dev_list[0]) {
      printf("[DEBUG] Failed to get valid device list\n");
      RCLCPP_ERROR(this->get_logger(), "Failed to get devices for type=%d", static_cast<int>(type));
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      return false;
    }

    dev_ = dev_list[0];

    if (!device_open(dev_)) {
      printf("[DEBUG] Failed to open device\n");
      RCLCPP_ERROR(this->get_logger(), "Failed to open CAN device for type=%d", static_cast<int>(type));
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      dev_ = nullptr;
      return false;
    }
    printf("[DEBUG] Device opened successfully\n");

    char strBuf[255] = {0};
    device_get_version(dev_, strBuf, sizeof(strBuf));
    printf("[DEBUG] Device version: %s\n", strBuf);
    RCLCPP_INFO(this->get_logger(), "Device version: %s", strBuf);

    device_get_serial_number(dev_, strBuf, sizeof(strBuf));
    printf("[DEBUG] Device SN: %s\n", strBuf);
    RCLCPP_INFO(this->get_logger(), "Device SN: %s", strBuf);

    bool baud_result = device_channel_set_baud_with_sp(dev_, 0, false, 1000000, 1000000, 0.75f, 0.75f);
    printf("[DEBUG] Set baudrate result: %d\n", baud_result);
    if (!baud_result) {
      printf("[DEBUG] Failed to set channel baudrate\n");
      RCLCPP_ERROR(this->get_logger(), "Failed to set channel baudrate");
      device_close(dev_);
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      dev_ = nullptr;
      return false;
    }

    bool open_ch_result = device_open_channel(dev_, 0);
    printf("[DEBUG] Open channel result: %d\n", open_ch_result);
    if (!open_ch_result) {
      printf("[DEBUG] Failed to open channel 0\n");
      RCLCPP_ERROR(this->get_logger(), "Failed to open channel 0");
      device_close(dev_);
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      dev_ = nullptr;
      return false;
    }
    printf("[DEBUG] Channel 0 opened successfully\n");

    device_hook_to_sent(dev_, sent_callback);
    printf("[DEBUG] Sent callback registered\n");
    device_hook_to_rec(dev_, rec_callback);
    printf("[DEBUG] Rec callback registered\n");
    device_hook_to_err(dev_, err_callback);
    printf("[DEBUG] Err callback registered\n");

    RCLCPP_INFO(this->get_logger(), "CAN device opened by type=%d", static_cast<int>(type));
    return true;
  }

  // ...existing code...
  bool init_can_device()
  {
    if (init_can_device_with_type(DEV_USB2CANFD_DUAL) || init_can_device_with_type(DEV_USB2CANFD)) {
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
  std::vector<long int> referee_ids_;

  std::atomic<bool> can_opened_;
  std::atomic<bool> params_loaded_;  // 标记参数和回调是否都已就绪
  damiao_handle* handle_;
  device_handle* dev_;
  std::mutex io_mutex_;

  rclcpp::Subscription<rm_interfaces::msg::ChassisCmd>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr game_state_pub_;

  void on_cmd_vel(const rm_interfaces::msg::ChassisCmd::SharedPtr msg)
  {
    if (!can_opened_.load(std::memory_order_acquire) || !dev_) return;

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

    std::lock_guard<std::mutex> lock(io_mutex_);
    device_channel_send_fast(dev_, 0, chassis_cmd_id_, 1, false, false, false, 8, data);
  }

  void on_mode_cmd(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    // ...existing code...
    if (!can_opened_.load(std::memory_order_acquire) || !dev_) return;
    
    uint8_t data[8] = {0};
    data[0] = msg->data;
    
    std::lock_guard<std::mutex> lock(io_mutex_);
    device_channel_send_fast(dev_, 0, mode_switch_id_, 1, false, false, false, 8, data);
  }
};

void rec_callback(usb_rx_frame_t* frame)
{
    if (!frame) {
        printf("[REC] NULL frame pointer!\n");
        return;
    }
    
    // 总是打印接收到的所有帧，用于诊断
    printf("[REC] ID=0x%X ch=%u dlc=%u canfd=%u dir=%u: ",
           frame->head.can_id,
           frame->head.channel,
           frame->head.dlc,
           frame->head.canfd,
           frame->head.dir);
    for (int i = 0; i < frame->head.dlc && i < 8; i++) {
        printf("%02X ", frame->payload[i]);
    }
    printf("\n");
    fflush(stdout);
    
    SentinelCanNode* node = g_node_instance.load(std::memory_order_acquire);
    if (!node) {
        printf("[REC] Node instance is NULL\n");
        return;
    }
    
    if (!node->is_params_loaded()) {
        printf("[REC] WARNING: Params not loaded yet, skipping processing\n");
        return;  // 参数还没加载完，不处理
    }
    
    const uint32_t can_id = frame->head.can_id;
    
    auto& ids = node->get_referee_ids();
    printf("[REC] Checking against %zu referee IDs\n", ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
        printf("[REC]   Comparing 0x%X with referee_ids[%zu]=0x%X\n", 
               can_id, i, static_cast<unsigned int>(ids[i]));
        if (can_id == static_cast<uint32_t>(ids[i])) {
            printf("[REC] MATCH! ID=0x%X at index %zu\n", can_id, i);
            
            if (frame->head.dlc < 3) {
                RCLCPP_WARN(node->get_logger(), 
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
                
                node->get_pub()->publish(msg);
                
                printf("[REC] Published: progress=%d, hp=%d\n", game_progress, current_hp);
                RCLCPP_INFO(node->get_logger(), 
                    "Game state: progress=%d, hp=%d", game_progress, current_hp);
            }
            break;
        }
    }
}

void err_callback(usb_rx_frame_t* frame)
{
    if (!frame) {
        printf("[ERR] NULL error frame\n");
        return;
    }
    
    printf("[ERR_FRAME] ID=0x%X ch=%u dlc=%u canfd=%u ack=%u: ",
           frame->head.can_id,
           frame->head.channel,
           frame->head.dlc,
           frame->head.canfd,
           frame->head.ack);
    for (int i = 0; i < frame->head.dlc && i < 8; i++) {
        printf("%02X ", frame->payload[i]);
    }
    printf("\n");
    fflush(stdout);
    
    SentinelCanNode* node = g_node_instance.load(std::memory_order_acquire);
    if (!node) return;

    RCLCPP_WARN(node->get_logger(),
        "CAN error frame: id=0x%X ch=%u dlc=%u canfd=%u ack=%u",
        frame->head.can_id,
        static_cast<unsigned>(frame->head.channel),
        static_cast<unsigned>(frame->head.dlc),
        static_cast<unsigned>(frame->head.canfd),
        static_cast<unsigned>(frame->head.ack));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SentinelCanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}