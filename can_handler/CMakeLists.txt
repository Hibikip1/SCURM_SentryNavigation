cmake_minimum_required(VERSION 3.8)
project(can_handler)

# 使用 GCC-13
set(CMAKE_C_COMPILER /usr/bin/gcc-13)
set(CMAKE_CXX_COMPILER /usr/bin/g++-13)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(rm_can src/rm_can.cpp)
  
ament_target_dependencies(rm_can rclcpp std_msgs geometry_msgs rm_interfaces)

# 添加头文件路径
target_include_directories(rm_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
  $<INSTALL_INTERFACE:include>
  ${LIBUSB_INCLUDE_DIRS}
)

# 链接 USB-CAN 动态库和 libusb
target_link_libraries(rm_can
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/libdm_device.so
  ${LIBUSB_LIBRARIES}
  -lstdc++
  -pthread
)

# 安装可执行文件
install(TARGETS rm_can
  DESTINATION lib/${PROJECT_NAME}
)

# 安装动态库到 lib 目录
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/libdm_device.so
  DESTINATION lib
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
