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
#include "robotcontrol/bmcan_bus.hpp"
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class SentinelCanNode : public rclcpp::Node
{
public:
  SentinelCanNode()
  : Node("sentinel_can_node")
  {
    // params (可通过 launch/param 覆盖)
    // 注意：导航发布的/cmd_vel已经在chassis_link坐标系下
    // 不需要twist_transformer的yaw转换，直接使用/cmd_vel
//    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
//  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_in_yaw");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<int>("chassis_cmd_id", 0x520);
    this->declare_parameter<int>("mode_switch_id", 0x203);
    this->declare_parameter<std::vector<long int>>("referee_ids", std::vector<long int>{0x301, 0x302, 0x303});

    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    chassis_cmd_id_ = this->get_parameter("chassis_cmd_id").as_int();
    mode_switch_id_ = this->get_parameter("mode_switch_id").as_int();
    referee_ids_ = this->get_parameter("referee_ids").as_integer_array();

    RCLCPP_INFO(this->get_logger(), "Sentinel CAN node started");

    // 打开 CAN 设备（传 nullptr 由库自动选第一个通道）
    BM_NotificationHandle ret = canbus_.open(channel_, nullptr);
    if (ret == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "打开 CAN 设备失败");
      can_opened_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "CAN 设备打开成功");
      can_opened_ = true;
    }

    // 订阅速度指令
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&SentinelCanNode::on_cmd_vel, this, std::placeholders::_1));

    // 订阅哨兵模式切换请求（uint8，定义由你们决定）
    mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "sentry_mode_cmd", 10,
      std::bind(&SentinelCanNode::on_mode_cmd, this, std::placeholders::_1));

    // 发布解析后的比赛状态
    game_state_pub_ = this->create_publisher<rm_interfaces::msg::GameState>("game_state", 10);

    // 启动定时器轮询 CAN 接收（包括裁判 ID）
    timer_ = this->create_wall_timer(50ms, std::bind(&SentinelCanNode::timer_poll_can, this));

    // 打印映射信息
    RCLCPP_INFO(this->get_logger(), "chassis_cmd_id=0x%X, mode_switch_id=0x%X", chassis_cmd_id_, mode_switch_id_);
    std::string ids = "referee_ids: ";
    for (auto id : referee_ids_) { char buf[16]; snprintf(buf, sizeof(buf), "0x%lX ", id); ids += buf; }
    RCLCPP_INFO(this->get_logger(), ids.c_str());
  }

  ~SentinelCanNode() override
  {
    if (can_opened_) {
      canbus_.close(channel_);
      RCLCPP_INFO(this->get_logger(), "CAN 设备已关闭");
    }
  }

private:
  // CAN
  BMCANTool canbus_;
  BM_ChannelHandle channel_ = nullptr;
  bool can_opened_ = false;

  // 参数
  std::string cmd_vel_topic_;
  int chassis_cmd_id_;
  int mode_switch_id_;
  std::vector<long int> referee_ids_;

  // ROS 接口
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr game_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 健康监测
  int health_check_counter_ = 0;
  static constexpr int HEALTH_CHECK_INTERVAL = 600; // 每 30 秒检查一次（50ms * 600）

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
    if (!can_opened_) return;
    const float scale = 1000.0f;
    
    // ROS标准坐标系: X=前, Y=左, Z=上
    // 如果电控坐标系不同，在这里转换
    int16_t vx = static_cast<int16_t>(std::round(msg->linear.x * scale*2));
    int16_t vy = static_cast<int16_t>(std::round(msg->linear.y * scale*2));
    int16_t wz = static_cast<int16_t>(std::round(msg->angular.z * scale*2));

    uint8_t data[8] = {0};
    data[0] = 1; // VELOCITY
    data[1] = 0;
    data[2] = (vx >> 8) & 0xFF;
    data[3] = vx & 0xFF;
    data[4] = (vy >> 8) & 0xFF;
    data[5] = vy & 0xFF;
    data[6] = (wz >> 8) & 0xFF;
    data[7] = wz & 0xFF;

    canbus_.can_send(channel_, chassis_cmd_id_, data, 1000);
    RCLCPP_INFO(this->get_logger(), "Sent chassis cmd id=0x%X vx=%.3f vy=%.3f wz=%.3f", chassis_cmd_id_, msg->linear.x, msg->linear.y, msg->angular.z);
  }

  // 哨兵模式切换：直接把 uint8 放到 data[0]91tv.com
  void on_mode_cmd(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (!can_opened_) return;
    uint8_t data[8] = {0};
    data[0] = msg->data;
    canbus_.can_send(channel_, mode_switch_id_, data, 1000);
    RCLCPP_INFO(this->get_logger(), "Sent mode switch %u to id=0x%X", msg->data, mode_switch_id_);
  }

  // 轮询接收 CAN（按配置的 referee_ids_），解析成 GameState 并发布
  void timer_poll_can()
  {
    if (!can_opened_) return;
    
    // 定期健康检查
    health_check_counter_++;
    if (health_check_counter_ >= HEALTH_CHECK_INTERVAL) {
      health_check_counter_ = 0;
      
      // 获取 CAN 状态
      BM_CanStatusInfoTypedef status;
      BM_StatusTypeDef ret = canbus_.get_status(channel_, &status);
      if (ret == BM_ERROR_OK) {
        RCLCPP_INFO(this->get_logger(), 
          "CAN 状态: TEC=%u, REC=%u, BUSOFF=%u, TXBP=%u, RXBP=%u",
          status.TEC, status.REC, status.TXBO, status.TXBP, status.RXBP);
        
        // 如果总线关闭或错误严重，重置通道
        if (status.TXBO || status.TEC > 200 || status.REC > 200) {
          RCLCPP_WARN(this->get_logger(), "检测到 CAN 错误状态，重置通道...");
          canbus_.reset_channel(channel_);
        }
      }
    }

    // 我们假设裁判数据通过三个帧发送并合并成一个 GameState
    // 帧映射（示例）：
    // referee_ids_[0]: [0] game_progress (u8), [1-2] stage_remain_time (u16), [3-4] current_hp (u16), [5] armor_id (u8), [6] hurt_type (u8)
    // referee_ids_[1]: [0-1] my_outpost_hp (u16), [2-3] enemy_outpost_hp (u16), [4-5] my_base_hp (u16), [6-7] enemy_base_hp (u16)
    // referee_ids_[2]: [0-1] projectile_allowance_17mm (u16)

    rm_interfaces::msg::GameState state_msg;
    bool updated = false;

    for (int id : referee_ids_) {
      uint8_t rx[8] = {0};
      BM_StatusTypeDef ret = canbus_.can_receive(channel_, id, rx, 2);
      if (ret == BM_ERROR_OK) {
        updated = true;
        RCLCPP_DEBUG(this->get_logger(), "Received referee id=0x%X", id);
        if (id == referee_ids_[0]) {
          state_msg.game_progress = rx[0];
          state_msg.stage_remain_time = static_cast<uint16_t>(rx[1]) | (static_cast<uint16_t>(rx[2]) << 8);
          state_msg.current_hp = static_cast<uint16_t>(rx[3]) | (static_cast<uint16_t>(rx[4]) << 8);
          state_msg.armor_id = rx[5];
          state_msg.hurt_type = rx[6];
        } else if (id == referee_ids_[1]) {
          state_msg.my_outpost_hp = static_cast<uint16_t>(rx[0]) | (static_cast<uint16_t>(rx[1]) << 8);
          state_msg.enemy_outpost_hp = static_cast<uint16_t>(rx[2]) | (static_cast<uint16_t>(rx[3]) << 8);
          state_msg.my_base_hp = static_cast<uint16_t>(rx[4]) | (static_cast<uint16_t>(rx[5]) << 8);
          state_msg.enemy_base_hp = static_cast<uint16_t>(rx[6]) | (static_cast<uint16_t>(rx[7]) << 8);
        } else if (id == referee_ids_[2]) {
          state_msg.projectile_allowance_17mm = static_cast<uint16_t>(rx[0]) | (static_cast<uint16_t>(rx[1]) << 8);
        }
      }
    }

    if (updated) {
      // 发布解析结果
      game_state_pub_->publish(state_msg);
      RCLCPP_DEBUG(this->get_logger(), "Published GameState (maybe partial)");
    }
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
