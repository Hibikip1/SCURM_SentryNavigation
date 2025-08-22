#ifndef ROS2_CAN_BRIDGE_CAN_DRIVER_HPP_
#define ROS2_CAN_BRIDGE_CAN_DRIVER_HPP_

#include <string>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <functional>

namespace ros2_can_bridge
{

/**
 * @brief Structure representing a CAN message
 */
struct CanMessage {
  uint32_t id;            // CAN ID
  uint8_t data[8];        // Data bytes (max 8 for standard CAN)
  uint8_t dlc;            // Data Length Code
};

/**
 * @brief Callback type for received CAN messages
 */
using CanReceiveCallback = std::function<void(const CanMessage &)>;

/**
 * @class CanDriver
 * @brief Driver class for interacting with CAN bus
 */
class CanDriver
{
public:
  /**
   * @brief Constructor
   * @param interface CAN interface name (e.g., "can0")
   */
  explicit CanDriver(const std::string & interface);

  /**
   * @brief Destructor
   */
  ~CanDriver();

  /**
   * @brief Initialize the CAN interface
   * @return true if successful, false otherwise
   */
  bool init();

  /**
   * @brief Send a CAN message
   * @param message CanMessage to send
   * @return true if successfully sent, false otherwise
   */
  bool send(const CanMessage & message);

  /**
   * @brief Set callback for received messages
   * @param callback Function to call when a message is received
   */
  void set_receive_callback(CanReceiveCallback callback);

  /**
   * @brief Start the receive thread
   * @return true if started successfully, false otherwise
   */
  bool start();

  /**
   * @brief Stop the receive thread
   */
  void stop();

private:
  std::string interface_;           // CAN interface name
  int socket_fd_;                   // Socket file descriptor
  bool is_running_;                 // Flag to control receive thread
  std::thread receive_thread_;      // Thread for receiving CAN frames
  CanReceiveCallback receive_callback_;  // Callback for received messages
  std::mutex mutex_;                // Mutex for thread safety

  /**
   * @brief Thread function to receive CAN messages
   */
  void receive_thread_func();
};

}  // namespace ros2_can_bridge

#endif  // ROS2_CAN_BRIDGE_CAN_DRIVER_HPP_
