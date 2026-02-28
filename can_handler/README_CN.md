# RM CAN Handler - USB转CAN通信节点
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" -r 50
## 功能说明

这个 ROS2 节点实现了通过 USB-CAN 设备与电控系统的通信功能。

### 主要功能
- ✅ 订阅底盘速度指令 (`/chassis_cmd`) 并通过 CAN 发送给电控
- ✅ 订阅哨兵模式切换指令 (`/sentry_mode_cmd`) 并发送到电控
- ✅ 接收裁判系统数据并发布为 `/game_state` 话题
- ✅ 支持 USB2CANFD_DUAL 设备

## 编译说明

### 依赖项
- ROS2 (Humble)
- GCC 13 或更高版本
- libusb-1.0-dev
- rm_interfaces (自定义消息包)

### 编译步骤

```bash
# 1. 安装 GCC-13（如果尚未安装）
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install -y gcc-13 g++-13 libstdc++-13-dev

# 2. 安装 libusb
sudo apt-get install -y libusb-1.0-dev

# 3. 编译
cd ~/sentry_ws
colcon build --packages-select can_handler --cmake-args -DCMAKE_C_COMPILER=/usr/bin/gcc-13 -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
```

## 运行说明

### 启动节点

```bash
# Source 环境
source ~/sentry_ws/install/setup.bash

# 运行节点
ros2 run can_handler rm_can
```

### 参数配置

节点支持以下参数（可通过 launch 文件或命令行设置）：

```yaml
cmd_vel_topic: "/chassis_cmd"          # 底盘速度命令话题
chassis_cmd_id: 0x520                  # 底盘CAN ID
mode_switch_id: 0x203                  # 模式切换CAN ID
referee_ids: [0x301, 0x302, 0x303]    # 裁判系统CAN ID列表
```

### 订阅的话题

- `/chassis_cmd` (geometry_msgs/Twist) - 底盘速度指令
- `/sentry_mode_cmd` (std_msgs/UInt8) - 哨兵模式切换

### 发布的话题

- `/game_state` (rm_interfaces/GameState) - 裁判系统游戏状态

## 测试

```bash
# 运行测试脚本
./test_rm_can.sh
```

或手动测试：

```bash
# 发送速度指令
ros2 topic pub --once /chassis_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# 发送模式切换
ros2 topic pub --once /sentry_mode_cmd std_msgs/msg/UInt8 "{data: 1}"
```

## CAN 协议说明

### 底盘控制帧 (chassis_cmd_id: 0x520)

| Byte | 说明 | 类型 | 备注 |
|------|------|------|------|
| 0 | 控制类型 | uint8 | 1 = VELOCITY |
| 1 | 保留 | uint8 | - |
| 2-3 | vx | int16 | linear.x * 1000 (mm/s) |
| 4-5 | vy | int16 | linear.y * 1000 (mm/s) |
| 6-7 | wz | int16 | angular.z * 1000 (mrad/s) |

### 模式切换帧 (mode_switch_id: 0x203)

| Byte | 说明 | 类型 | 备注 |
|------|------|------|------|
| 0 | 模式值 | uint8 | 由电控定义 |
| 1-7 | 保留 | - | - |

### 裁判系统接收帧

详见代码注释，包含三个 CAN ID 的数据合并为一个 GameState 消息。

## 坐标系说明

### ROS2 标准坐标系
- X 轴：前方
- Y 轴：左方
- Z 轴：上方

如果电控使用不同的坐标系，需要在代码中进行坐标转换。

## 故障排查

### 编译错误

**错误**: `undefined reference to 'std::ios_base_library_init()@GLIBCXX_3.4.32'`

**解决**: 安装 GCC-13 并使用它编译：
```bash
sudo apt-get install -y gcc-13 g++-13 libstdc++-13-dev
colcon build --packages-select can_handler --cmake-args -DCMAKE_C_COMPILER=/usr/bin/gcc-13 -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
```

### 运行错误

**错误**: `No CAN device found!`

**原因**: 没有连接 USB-CAN 设备或设备权限不足

**解决**:
1. 检查设备是否连接：`lsusb`
2. 添加 USB 设备权限：
```bash
sudo usermod -a -G dialout $USER
# 注销后重新登录
```

## 开发者信息

- 基于达妙 USB-CAN SDK
- 设备类型：DEV_USB2CANFD_DUAL
- CAN 波特率：1 Mbps
- CANFD 波特率：5 Mbps
- 采样点：0.75

## 文件结构

```
can_handler/
├── CMakeLists.txt          # CMake 配置
├── package.xml             # ROS2 包配置
├── lib/                    # 库文件
│   ├── pub_user.h         # SDK 头文件
│   └── linux/
│       └── libdm_device.so # USB-CAN 动态库
└── src/
    └── rm_can.cpp         # 主节点实现
```

## 许可

请遵循项目许可证。
