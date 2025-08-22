cmake_minimum_required(VERSION 3.8)
project(ros2_can_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

include_directories(
  include
)

# Add executable
add_executable(can_bridge_node 
  src/can_driver.cpp
  src/protocol_parser.cpp
  src/ros2_can_bridge_node.cpp
)

ament_target_dependencies(can_bridge_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  rm_interfaces
)

# Install targets
install(
  TARGETS can_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
