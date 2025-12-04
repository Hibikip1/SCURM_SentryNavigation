
cmake_minimum_required(VERSION 3.8)
project(robotcan_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 检测系统位数并设置相应的路径和库
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(BMCAN_INCLUDE_DIR "./lib/bin/unix64/release/")
    set(BMCAN_LIB_NAME "bmapi64")
else()
    set(BMCAN_INCLUDE_DIR "./lib/bin/unix32/release")
    set(BMCAN_LIB_NAME "bmapi")
endif()

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
# 添加bmcan include和库搜索路径
# 添加bmcan include和库搜索路径
# 添加bmcan include和库搜索路径
include_directories(${EXTERNAL_INCLUDE_DIR_1})
include_directories(${EXTERNAL_INCLUDE_DIR_2})
include_directories("./lib/include")
link_directories(${BMCAN_INCLUDE_DIR})
include_directories("./include/robotcontrol")



add_executable(sentinel_can_node
  src/sentinel_can_node.cpp
  src/bmcan_bus.cpp
)

target_link_libraries(sentinel_can_node ${BMCAN_LIB_NAME} usb-1.0)

target_include_directories(sentinel_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sentinel_can_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  sentinel_can_node
  rclcpp
  geometry_msgs
  std_msgs
  rm_interfaces
)

# 设置 RPATH 以便运行时找到共享库
set_target_properties(sentinel_can_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/../../lib/${PROJECT_NAME}"
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS
  sentinel_can_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 BMCAN 共享库到 lib 目录
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/${BMCAN_INCLUDE_DIR}/lib${BMCAN_LIB_NAME}.so
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python脚本
install(
  PROGRAMS
  scripts/motor_test.py
  DESTINATION lib/${PROJECT_NAME}
)


# 安装launch文件夹
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


