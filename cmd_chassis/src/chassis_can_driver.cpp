#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/chassis_cmd.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <cstring>
#include <unistd.h>

class ChassisCANDriver : public rclcpp::Node
{
public:
    ChassisCANDriver()
    : Node("chassis_can_driver")
    {
        this->declare_parameter<std::string>("can_interface", "can0");
        std::string can_interface = this->get_parameter("can_interface").as_string();

        // 打开 CAN 设备
        if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN socket create failed");
            throw std::runtime_error("CAN socket create failed");
        }
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ);
        ioctl(can_socket_, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN socket bind failed");
            throw std::runtime_error("CAN socket bind failed");
        }

        chassis_sub_ = this->create_subscription<rm_interfaces::msg::ChassisCmd>(
            "chassis_cmd", 10,
            std::bind(&ChassisCANDriver::chassis_cmd_callback, this, std::placeholders::_1));
    }

    ~ChassisCANDriver()
    {
        if (can_socket_ > 0) {
            close(can_socket_);
        }
    }

private:
    void chassis_cmd_callback(const rm_interfaces::msg::ChassisCmd::SharedPtr msg)
    {
        struct can_frame frame;
        frame.can_id = 0x123; // 根据实际协议设置
        frame.can_dlc = 8;

        // 数据打包（示例：只打包线速度和角速度，实际需按协议调整）
        float vx = msg->twist.linear.x;
        float wz = msg->twist.angular.z;
        std::memcpy(frame.data, &vx, sizeof(float));
        std::memcpy(frame.data + 4, &wz, sizeof(float));

        int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
        if (nbytes != sizeof(struct can_frame)) {
            RCLCPP_WARN(this->get_logger(), "CAN frame send failed");
        }
    }

    int can_socket_;
    rclcpp::Subscription<rm_interfaces::msg::ChassisCmd>::SharedPtr chassis_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisCANDriver>());
    rclcpp::shutdown();
    return 0;
}