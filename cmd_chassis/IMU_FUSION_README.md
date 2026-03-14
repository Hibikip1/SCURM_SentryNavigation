# IMU 姿态融合和速度变换系统使用说明

## 系统架构

本系统通过 robot_localization 的 EKF 节点融合 Livox IMU 原始数据，得到真实的 orientation（姿态），然后用于速度坐标系转换。

```
/livox/imu → rot_imu → /imu/data → ekf_node → /imu/odometry (带真实orientation)
                                                      ↓
/cmd_vel (导航输出) → twist_transformer (用EKF的yaw转换) → /cmd_vel_in_yaw → 底盘控制
```

## 文件说明

### 1. 配置文件
- `config/ekf_imu.yaml`: robot_localization EKF 配置
  - 融合 IMU 的角速度和加速度
  - 输出带真实 orientation 的 odometry

### 2. 节点说明

#### rot_imu (预处理节点)
- **输入**: `/livox/imu` (Livox 原始 IMU 数据)
- **输出**: `/imu/data` (预处理后的 IMU 数据)
- **功能**: 
  - 平放模式: 直接转发原始数据
  - 倾斜模式: 应用 TF 变换修正坐标系
- **参数**:
  ```yaml
  use_tf_transform: false  # 是否启用TF变换（mid360倾斜时设为true）
  roll_offset: 0.0         # Roll偏移角（弧度）
  pitch_offset: 0.0        # Pitch偏移角（弧度）
  yaw_offset: 0.0          # Yaw偏移角（弧度）
  ```

#### ekf_node (姿态融合节点)
- **输入**: `/livox/imu`
- **输出**: `/imu/odometry` (带真实 orientation 的里程计)
- **功能**: 用 EKF 算法融合 IMU 数据，估算真实姿态

#### twist_transformer (速度转发/转换节点)
- **输入**: 
  - `/cmd_vel` (导航输出，**本体坐标系** base_link，即云台/雷达朝向)
  - `/imu/odometry` (可选，仅当 `use_yaw_transform:=true` 时用于 map→body 变换)
- **输出**: `/cmd_vel_in_yaw` (底盘/云台坐标系速度)
- **功能**: 
  - **默认（推荐）**: 透传。Nav2 的 cmd_vel 已是机器人本体（云台）系，直接转发给底盘，无需再按 yaw 旋转，否则会导致导航转圈。
  - 可选 `use_yaw_transform:=true`: 将 cmd_vel 视为 map 系，用 EKF 的 yaw 转到云台系（仅当你的 cmd_vel 确实在 map 系时使用）。

## 使用方法

### 1. 启动 IMU 融合系统
```bash
ros2 launch cmd_chassis imu_fusion.launch.py
```

### 2. 验证系统工作
```bash
# 查看 EKF 输出的姿态（应该有变化的 orientation）
ros2 topic echo /imu/odometry --field pose.pose.orientation

# 对比转换前后的速度
ros2 topic echo /cmd_vel | head -20 &
ros2 topic echo /cmd_vel_in_yaw | head -20
```

### 3. 调试信息
```bash
# 查看 twist_transformer 的日志（包含 yaw 角信息）
ros2 run rqt_console rqt_console
# 或
ros2 topic echo /rosout | grep twist_transformer
```

## mid360 倾斜放置时的配置

当 mid360 需要倾斜安装时（例如向前倾斜 30 度）：

### 方法 1: 修改 launch 文件参数
编辑 `launch/imu_fusion.launch.py`:
```python
Node(
    package='cmd_chassis',
    executable='rot_imu',
    name='rot_imu_node',
    output='screen',
    parameters=[{
        'use_tf_transform': True,      # 启用变换
        'roll_offset': 0.0,             # 绕X轴旋转角度
        'pitch_offset': 0.52,           # 绕Y轴旋转角度 (30度 = 0.52弧度)
        'yaw_offset': 0.0               # 绕Z轴旋转角度
    }]
),
```

### 方法 2: 命令行指定参数
```bash
ros2 run cmd_chassis rot_imu --ros-args \
  -p use_tf_transform:=true \
  -p pitch_offset:=0.52
```

## 故障排查

### 问题 1: twist_transformer 报告 "No valid yaw from EKF"
- **原因**: EKF 节点未启动或未输出数据
- **解决**: 
  1. 检查 `/imu/odometry` 话题是否发布: `ros2 topic hz /imu/odometry`
  2. 检查 ekf_node 日志: `ros2 node info /ekf_imu_fusion`

### 问题 2: /cmd_vel 和 /cmd_vel_in_yaw 没有区别
- **原因**: EKF 输出的 yaw 始终为 0（可能 IMU 没有旋转）
- **解决**: 
  1. 旋转云台，观察 `/imu/odometry` 的 orientation 是否变化
  2. 检查 EKF 配置中 yaw 是否启用

### 问题 3: 导航还是到不了目标点
- **检查**: 
  1. `/cmd_vel_in_yaw` 是否正确发布
  2. `sentinel_can_node` 是否订阅了正确的话题
  3. 底盘控制是否正常响应

## 参数调优

### EKF 参数 (ekf_imu.yaml)
- `frequency`: 更新频率 (默认 50Hz)
- `sensor_timeout`: 传感器超时时间
- `imu0_config`: 指定使用 IMU 的哪些数据
  - 当前配置: 只使用 yaw、yaw角速度和加速度

### 坐标系偏移参数 (rot_imu)
- 根据实际安装角度调整 `roll_offset`, `pitch_offset`, `yaw_offset`
- 单位: 弧度 (1度 ≈ 0.01745弧度)

## 技术细节

### 为什么需要 EKF？
- Livox IMU 只提供原始加速度和角速度，orientation 字段为空
- EKF 通过融合这些数据，估算真实的 roll/pitch/yaw
- 特别是 yaw 角，用于速度坐标系转换

### 速度与坐标系说明
- **Nav2 的 cmd_vel**：标准实现中是在**机器人本体坐标系**（base_link，即云台/雷达朝向）下给出的，x 前、y 左、angular.z 绕竖轴。因此云台解算默认只需透传，无需再做 map→body 旋转；否则会错误地多旋转一次，导致导航转圈或乱走。
- 仅当 cmd_vel 明确在 map 系下输出时，可设 `use_yaw_transform:=true`，此时使用下述公式：
```cpp
// 2D 旋转变换（map坐标系 → 云台坐标系）
vx_yaw = vx_map * cos(yaw) + vy_map * sin(yaw)
vy_yaw = vy_map * cos(yaw) - vx_map * sin(yaw)
```

### TF 变换（倾斜放置时）
当 mid360 倾斜安装时，rot_imu 会应用四元数旋转：
```cpp
angular_velocity = quaternion_rotate(transform_quat, angular_velocity)
linear_acceleration = quaternion_rotate(transform_quat, linear_acceleration)
```
