/*
封装了 Linux SocketCAN 的底层操作，负责与物理 CAN 设备（如 can0）进行数据收发。
提供 send、start、stop、set_receive_callback 等接口。
内部有一个接收线程，收到 CAN 帧后通过回调传递给协议解析器。
*/
#include "ros2_can_bridge/can_driver.hpp"
#include <cstring>
#include <iostream>

namespace ros2_can_bridge
{

CanDriver::CanDriver(const std::string & interface)
: interface_(interface), socket_fd_(-1), is_running_(false)
{
}

CanDriver::~CanDriver()
{
  stop();
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanDriver::init()
{
  // Create socket
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "Error creating CAN socket" << std::endl;
    return false;
  }

  // Set up interface request
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "Error getting interface index for " << interface_ << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Bind socket to the CAN interface
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    std::cerr << "Error binding CAN socket to interface " << interface_ << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  std::cout << "Successfully initialized CAN interface " << interface_ << std::endl;
  return true;
}

bool CanDriver::send(const CanMessage & message)
{
  if (socket_fd_ < 0) {
    std::cerr << "CAN socket not initialized" << std::endl;
    return false;
  }

  struct can_frame frame;
  frame.can_id = message.id;
  frame.can_dlc = message.dlc;
  std::memcpy(frame.data, message.data, message.dlc);

  ssize_t bytes_sent = write(socket_fd_, &frame, sizeof(struct can_frame));
  if (bytes_sent != sizeof(struct can_frame)) {
    std::cerr << "Failed to send CAN frame" << std::endl;
    return false;
  }

  return true;
}

void CanDriver::set_receive_callback(CanReceiveCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  receive_callback_ = std::move(callback);
}

bool CanDriver::start()
{
  if (is_running_ || socket_fd_ < 0) {
    return false;
  }

  is_running_ = true;
  receive_thread_ = std::thread(&CanDriver::receive_thread_func, this);
  return true;
}

void CanDriver::stop()
{
  is_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

void CanDriver::receive_thread_func()
{
  struct can_frame frame;
  fd_set read_fds;
  struct timeval tv;

  while (is_running_) {
    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    // Set timeout for select (100ms)
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    int ret = select(socket_fd_ + 1, &read_fds, NULL, NULL, &tv);
    if (ret < 0) {
      std::cerr << "Error in select" << std::endl;
      break;
    } else if (ret == 0) {
      // Timeout, no data
      continue;
    }

    if (FD_ISSET(socket_fd_, &read_fds)) {
      ssize_t bytes_read = read(socket_fd_, &frame, sizeof(struct can_frame));
      if (bytes_read < 0) {
        std::cerr << "Error reading CAN frame" << std::endl;
        continue;
      }

      if (bytes_read == sizeof(struct can_frame)) {
        CanMessage message;
        message.id = frame.can_id;
        message.dlc = frame.can_dlc;
        std::memcpy(message.data, frame.data, frame.can_dlc);

        std::lock_guard<std::mutex> lock(mutex_);
        if (receive_callback_) {
          receive_callback_(message);
        }
      }
    }
  }
}

}  // namespace ros2_can_bridge
