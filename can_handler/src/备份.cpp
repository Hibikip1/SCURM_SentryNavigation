/*
 * sentinel_can_node.cpp
 * 简单的 USB-CAN 封装节点，用于：
 * - 订阅速度指令（geometry_msgs::msg::Twist），编码并通过 CAN 发给电控（底盘）
 * - 接收裁判系统的 3 个 CAN ID，解析并发布为 rm_interfaces::msg::GameState
 * - 订阅哨兵模式切换指令（std_msgs::msg::UInt8），发送到电控
 *
 * 说明：CAN 帧与字段的具体协议请与电控/裁判端保持一致。这里提供一个合理的映射和可配置参数。
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rm_interfaces/msg/game_state.hpp>
#include <vector>
#include <chrono>
#include <cstdio>
#include <thread>
#include "pub_user.h"





using namespace std::chrono_literals;

class SentinelCanNode : public rclcpp::Node
{
public:
  SentinelCanNode()
  : Node("rm_can"), can_opened_(false), handle_(nullptr), dev_(nullptr)
  {
    // params (可通过 launch/param 覆盖)
    // 注意：导航发布的/cmd_vel已经在chassis_link坐标系下
    // 不需要twist_transformer的yaw转换，直接使用/cmd_vel
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<int>("chassis_cmd_id", 0x520);
    this->declare_parameter<int>("mode_switch_id", 0x203);
    this->declare_parameter<std::vector<long int>>("referee_ids", std::vector<long int>{0x301, 0x302, 0x303});

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    chassis_cmd_id_ = this->get_parameter("chassis_cmd_id").as_int();
    mode_switch_id_ = this->get_parameter("mode_switch_id").as_int();
    referee_ids_ = this->get_parameter("referee_ids").as_integer_array();

    RCLCPP_INFO(this->get_logger(), "Sentinel CAN node started");


    if (!init_can_device()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN device");
      return;
    }


    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&SentinelCanNode::on_cmd_vel, this, std::placeholders::_1));

    mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "sentry_mode_cmd", 10,
      std::bind(&SentinelCanNode::on_mode_cmd, this, std::placeholders::_1));

 
    game_state_pub_ = this->create_publisher<rm_interfaces::msg::GameState>("game_state", 10);

   
    timer_ = this->create_wall_timer(50ms, std::bind(&SentinelCanNode::timer_poll_can, this));

    RCLCPP_INFO(this->get_logger(), "chassis_cmd_id=0x%X, mode_switch_id=0x%X", chassis_cmd_id_, mode_switch_id_);
    std::string ids = "referee_ids: ";
    for (auto id : referee_ids_) { char buf[16]; snprintf(buf, sizeof(buf), "0x%lX ", id); ids += buf; }
    RCLCPP_INFO(this->get_logger(), "%s", ids.c_str());
  }



  ~SentinelCanNode() override
  {
    if (can_opened_ && dev_) {
      device_close_channel(dev_, 0);
      device_close(dev_);

      RCLCPP_INFO(this->get_logger(), "CAN 设备已关闭");
    }
    if (handle_) {
      damiao_handle_destroy(handle_);
    }
  }

private:
  // CAN 设备初始化
  bool init_can_device()
  {
    // 初始化模块句柄
    // 注意：使用 DEV_USB2CANFD 而不是 DEV_USB2CANFD_DUAL
    handle_ = damiao_handle_create(DEV_USB2CANFD);
    if (!handle_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create damiao handle");
      return false;
    }

    // 打印 SDK 版本信息
    damiao_print_version(handle_);

    // 查找对应类型模块的设备数量
    int device_cnt = damiao_handle_find_devices(handle_);
    if (device_cnt == 0) {
      RCLCPP_ERROR(this->get_logger(), "No CAN device found!");
      damiao_handle_destroy(handle_);
      dev_ = nullptr;
      return false;
    }

    // 获取设备信息
    device_handle* dev_list[16];
    int handle_cnt = 0;
    damiao_handle_get_devices(handle_, dev_list, &handle_cnt);
    
    if (handle_cnt == 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get devices");
      damiao_handle_destroy(handle_);
      dev_ = nullptr;
      return false;
    }

    dev_ = dev_list[0];

    // 打开设备
    if (!device_open(dev_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CAN device");
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      dev_ = nullptr;
      return false;
    }

    // 获取设备信息
    char strBuf[255] = {0};
    device_get_version(dev_, strBuf, sizeof(strBuf));
    RCLCPP_INFO(this->get_logger(), "Device version: %s", strBuf);
    
    device_get_serial_number(dev_, strBuf, sizeof(strBuf));
    RCLCPP_INFO(this->get_logger(), "Device SN: %s", strBuf);

    // 设置通道 0 的波特率
    // CAN 1Mbps, , 采样点 0.75
    //device_channel_set_baud_with_sp(dev_, 0, false, 1000000, 1000000, 0.75f, 0.75f);

    char buf[255];
    device_get_version(dev_, buf, sizeof(buf));
    device_get_serial_number(dev_, buf, sizeof(buf));
    RCLCPP_INFO(this->get_logger(), "CAN Device Version: %s, SN: %s", buf, buf);  
    // 开启 CAN 通道 0
    if (!device_open_channel(dev_, 0)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open channel 0");
      device_close(dev_);
      damiao_handle_destroy(handle_);
      handle_ = nullptr;
      dev_ = nullptr;
      return false;
    }

    // 注册回调函数
    device_hook_to_sent(dev_, sent_callback_static);
    device_hook_to_rec(dev_, rec_callback_static);
  



    can_opened_ = true;
    RCLCPP_INFO(this->get_logger(), "CAN device initialized successfully");
    return true;
  }

  // 静态回调函数（用于 C 接口）
  static void sent_callback_static(usb_rx_frame_t* frame)
  {
    // 发送确认回调
    // printf("Sent callback, packet id: 0x%x\n", frame->head.can_id);
  }

  static void rec_callback_static(usb_rx_frame_t* frame)
  {
    // 接收回调 - 这里可以处理接收到的数据
    // printf("Rec callback, packet id: 0x%x\n", frame->head.can_id);
  }

  // 参数
  std::string cmd_vel_topic_;
  int chassis_cmd_id_;
  int mode_switch_id_;
  std::vector<long int> referee_ids_;

  // CAN 设备句柄
  bool can_opened_;
  damiao_handle* handle_;
  device_handle* dev_;

  // ROS 接口
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr game_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 裁判系统数据缓存
  rm_interfaces::msg::GameState state_msg_;
  
  // 将 Twist 编码为 8 字节 CAN 帧（示例协议）：
  // Byte0: control type (1 = VELOCITY)
  // Byte1: reserved
  // Byte2-3: vx (int16) = linear.x * scale (ROS X轴 = 前方)
  // Byte4-5: vy (int16) = linear.y * scale (ROS Y轴 = 左方)
  // Byte6-7: wz (int16) = angular.z * scale
  // 
  // 注意：如果电控期望的坐标系不同，需要转换：
  // - 若电控X=右，Y=前 → 需交换并取反：vy_can=-vx_ros, vx_can=vy_ros
  // scale 取 1000（即 mm/s 或 m/s*1000）
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!can_opened_ || !dev_) return;
    
    const float scale = 1000.0f;
    
    // ROS标准坐标系: X=前, Y=左, Z=上
    // 如果电控坐标系不同，在这里转换
    int16_t vx = static_cast<int16_t>(std::round(msg->linear.x * scale));
    int16_t vy = static_cast<int16_t>(std::round(msg->linear.y * scale));
    int16_t wz = static_cast<int16_t>(std::round(msg->angular.z * scale));

    uint8_t data[8] = {0};
    data[0] = 1; // VELOCITY
    data[1] = 0;
    data[2] = (vx >> 8) & 0xFF;
    data[3] = vx & 0xFF;
    data[4] = (vy >> 8) & 0xFF;
    data[5] = vy & 0xFF;
    data[6] = (wz >> 8) & 0xFF;
    data[7] = wz & 0xFF;

    device_channel_send_fast(
        dev_,                // 设备句柄
        0,                   // 通道号
        chassis_cmd_id_,     // CAN ID
        1,                   // 发送次数
        false,               // 是否扩展帧
        false,               // 是否 CAN FD
        false,               // 是否启用 BRS（波特率切换）
        8,                   // 数据长度
        data                 // 数据内容
    );

    RCLCPP_INFO(this->get_logger(), "Sent chassis cmd id=0x%X vx=%.3f vy=%.3f wz=%.3f", 
                 chassis_cmd_id_, msg->linear.x, msg->linear.y, msg->angular.z);
  }

  // 哨兵模式切换：直接把 uint8 放到 data[0]
  void on_mode_cmd(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (!can_opened_ || !dev_) return;
    
    uint8_t data[8] = {0};
    data[0] = msg->data;
    
    device_channel_send_fast(
        dev_,                // 设备句柄
        0,                   // 通道号
        mode_switch_id_,     // CAN ID
        1,                   // 发送次数
        false,               // 是否扩展帧
        false,               // 是否 CAN FD
        false,               // 是否启用 BRS（波特率切换）
        8,                   // 数据长度
        data                 // 数据内容
    );
    
    RCLCPP_INFO(this->get_logger(), "Sent mode switch %u to id=0x%X", msg->data, mode_switch_id_);
  }

  // 轮询接收 CAN（按配置的 referee_ids_），解析成 GameState 并发布
  void timer_poll_can()
  {
    if (!can_opened_ || !dev_) return;

    // 注意：这个示例使用轮询方式
    // 实际上回调函数 rec_callback_static 也会被调用
    // 你可以选择在回调中处理或在这里轮询
    // 这里仅作为示例，实际应用中建议在回调中处理
    
    // 由于 SDK 的回调已经在后台线程中处理接收
    // 这里主要用于周期性发布状态或其他任务
    // 如果需要在定时器中主动读取，需要查看 SDK 是否提供轮询接口
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SentinelCanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
