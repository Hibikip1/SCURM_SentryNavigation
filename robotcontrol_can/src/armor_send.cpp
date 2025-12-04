
#include <rclcpp/rclcpp.hpp>
#include "robotcontrol/bmcan_bus.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <vector>


class ArmorSend : public rclcpp::Node {
public:
    ArmorSend() : Node("armor_send_node") {
        RCLCPP_INFO(this->get_logger(), "%s节点已启动.", this->get_name());
        // 打开CAN通道
        BM_NotificationHandle result = canbus.open(channelhandle, "BM-CANFD-X1(5850)");
        if (result == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "CAN设备打开失败!");
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN设备已打开");
        }

        // 订阅目标差值
        delta_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "target_delta", 10,
            std::bind(&ArmorSend::delta_callback, this, std::placeholders::_1)
        );

        // 订阅目标信息
        target_info_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "target_info", 10,
            std::bind(&ArmorSend::target_info_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "ros2_can节点已启动 - 等待接收检测数据");
    }

    ~ArmorSend() {
        // 关闭CAN通道
        canbus.close(channelhandle);
        RCLCPP_INFO(this->get_logger(), "CAN设备已关闭");
    }

    // 发送电机控制命令（示例，实际协议可根据需求调整）
    void send_motor_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
        // 只发送 x, y 偏移量，数据帧前 4 字节分别为 x、y 的高低位
        uint8_t can_send_data[8] = {0};
        can_send_data[0] = (motor1 >> 8) & 0xFF; // x 高位
        can_send_data[1] = motor1 & 0xFF;        // x 低位
        can_send_data[2] = (motor2 >> 8) & 0xFF; // y 高位
        can_send_data[3] = motor2 & 0xFF;        // y 低位
        // 其余字节填 0

        RCLCPP_INFO(this->get_logger(), "发送自瞄偏移量: x=%d, y=%d", motor1, motor2);
        try {
            canbus.can_send(channelhandle, cantx_id, can_send_data, 1000);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送自瞄偏移量失败: %s", e.what());
        }
    }

private:
    BMCANTool canbus;
    BM_ChannelHandle channelhandle;
    // CAN通信相关ID
    const int cantx_id = 0x100; // 发送ID
    const int canrx_id = 0x101; // 接收ID

    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr delta_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_info_subscription_;

    // 目标差值回调
    void delta_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到目标差值: (%.1f, %.1f)", msg->x, msg->y);
        // 可在此处调用 send_motor_cmd 或其他处理逻辑
    }

    // 目标信息回调
    void target_info_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 6) {
            double delta_x = msg->data[4];
            double delta_y = msg->data[5];
            RCLCPP_INFO(this->get_logger(),
                "目标:(%.1f,%.1f) 光心:(%.1f,%.1f) 差值:(%.1f,%.1f)",
                msg->data[0], msg->data[1], msg->data[2], msg->data[3], delta_x, delta_y);
            // 可在此处调用 send_motor_cmd 或其他处理逻辑
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto armor_send_node = std::make_shared<ArmorSend>();
    rclcpp::spin(armor_send_node);
    rclcpp::shutdown();
    return 0;
}
