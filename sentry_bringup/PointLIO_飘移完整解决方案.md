# Point-LIO 飘移问题完整解决方案

## 问题确认

从你的 IMU 数据看到：
- ✅ 重力对齐正常（9.71 m/s²，Z轴9.69）
- ⚠️ **角速度存在零漂**：X轴约 -0.011 rad/s
- ⚠️ 后续出现运动噪声（机器人在动）

**核心问题**：角速度零漂 0.011 rad/s 看似很小，但经过积分会导致严重的姿态漂移！
- 1分钟漂移：0.011 × 60 = **0.66 弧度 ≈ 38度**
- 这就是为什么"轻轻一动整个坐标系就飘走"

## 解决方案

### 方案一：IMU 零漂校准（推荐）

已实现自动校准和补偿系统。

#### 步骤 1：校准 IMU

```bash
cd /home/lab/sentry_ws
source install/setup.bash

# 确保系统正在运行（需要 IMU 数据）
# 如果没运行，先启动：
ros2 launch sentry_bringup mapping.launch.py

# 新开终端，运行校准程序
# ⚠️ 重要：机器人必须完全静止在水平面上！
python3 src/sentry_bringup/scripts/calibrate_imu_bias.py
```

**等待 10-15 秒**，程序会：
- 采集 2000 个 IMU 样本
- 计算角速度和加速度零漂
- 自动保存到 `sentry_bringup/params/imu_bias_calibration.yaml`
- 给出 Point-LIO 参数建议

#### 步骤 2：重新编译（应用 bias 补偿）

```bash
cd /home/lab/sentry_ws

# 编译修改的 rot_imu 节点
colcon build --packages-select cmd_chassis

source install/setup.bash
```

#### 步骤 3：重新启动建图

```bash
# 关闭之前的进程（Ctrl+C）

# 重新启动（现在会自动加载 bias 补偿）
ros2 launch sentry_bringup mapping.launch.py
```

启动后会看到类似输出：
```
[mapping.launch.py] 已加载 IMU bias 校准: .../imu_bias_calibration.yaml
[rot_imu]: IMU bias 补偿已启用:
[rot_imu]:   陀螺仪 bias: [-0.011000, -0.000400, 0.004600] rad/s
```

#### 步骤 4：验证效果

**静止测试**：
```bash
# 新终端运行验证
python3 src/sentry_bringup/scripts/check_imu_stability.py
```

期望看到补偿后的角速度均值 < 0.001 rad/s

**建图测试**：
1. 保持机器人静止 15 秒
2. 慢速移动（< 0.2 m/s）
3. 观察 RViz 中的坐标系是否稳定

---

### 方案二：调整 Point-LIO 参数（辅助）

即使有 bias 补偿，也建议优化参数以提高鲁棒性。

已优化的参数说明：

```yaml
# 1. 降低 IMU 权重，提高激光约束
lidar_meas_cov: 0.001    # 原 0.01，提高激光权重 10 倍
acc_cov_input: 0.5        # 原 0.1，降低 IMU 加速度权重
gyr_cov_input: 0.05       # 原 0.01，降低陀螺仪权重

# 2. 允许估计更慢变化的 bias
b_acc_cov: 0.00001       # 原 0.0001，bias 变化更慢
b_gyr_cov: 0.00001       # 原 0.0001

# 3. 提高匹配质量
point_filter_num: 3       # 原 4，减少点数提高稳定性
plane_thr: 0.05          # 原 0.1，要求更平的平面
filter_size_surf: 0.5    # 原 0.4，增大滤波减少噪声
ivox_grid_resolution: 1.5 # 原 2.0，提高地图精度
```

这些参数的核心思路：**在 IMU 不够可靠时，更依赖激光数据**

---

### 方案三：使用 FAST-LIO（备选）

如果 Point-LIO 仍不稳定，可切换到 FAST-LIO：

```bash
nano src/sentry_bringup/params/slam_selector.yaml

# 修改为：
slam_algorithm: 'fast_lio'
```

FAST-LIO 对 IMU 噪声的容忍度更高，但精度可能稍低。

---

## 建图最佳实践

### 启动流程

1. **启动系统并等待**：
   ```bash
   ros2 launch sentry_bringup mapping.launch.py
   ```
   
2. **观察日志**：
   - 等待 "Livox MID360 connected"
   - 等待 Point-LIO 初始化完成
   
3. **保持静止 15 秒**：
   - 让 IMU 完成重力对齐
   - 让 Point-LIO 建立初始地图

4. **慢速启动**：
   - 前 30 秒线速度 < 0.2 m/s
   - 避免急加速、急转弯

### 运动约束

| 参数 | 建议值 | 说明 |
|------|--------|------|
| 线速度 | < 0.5 m/s | 太快会导致点云畸变 |
| 角速度 | < 1.0 rad/s | 快速旋转易失去跟踪 |
| 加速度 | 平滑渐变 | 避免突然加减速 |
| 环境 | 特征丰富 | 避免空旷走廊、玻璃墙 |

### 监控指标

**实时监控**：
```bash
# 查看状态估计频率（应 > 5 Hz）
ros2 topic hz /state_estimation

# 查看点云频率（应 = 10 Hz）
ros2 topic hz /livox/lidar

# 监控 IMU 数据
ros2 topic echo /imu/data --no-arr
```

**RViz 观察**：
- ✅ 点云应该稳定不抖动
- ✅ 轨迹路径应该平滑连续
- ⚠️ 如果看到突然跳变，说明跟踪失败

---

## 故障排查

### Q1: 校准后仍然飘移？

**可能原因**：
1. 校准时机器人没有完全静止
2. IMU 温漂严重（需要预热）
3. 硬件连接松动

**解决方法**：
```bash
# 1. 重新校准（确保静止）
python3 src/sentry_bringup/scripts/calibrate_imu_bias.py

# 2. 预热 IMU（运行 5 分钟后再校准）
# 3. 检查硬件连接

# 4. 如果零漂 > 0.02 rad/s，考虑硬件问题
```

### Q2: 编译失败？

```bash
# 清理并重新编译
cd /home/lab/sentry_ws
rm -rf build/ install/ log/
colcon build --packages-select cmd_chassis
source install/setup.bash
```

### Q3: 启动时找不到 bias 文件？

这是正常的，如果没有校准文件，系统会禁用 bias 补偿：
```
[mapping.launch.py] 未找到 IMU bias 校准文件，bias 补偿已禁用
```

运行校准后会自动创建。

### Q4: 如何验证 bias 补偿是否生效？

```bash
# 方法 1：查看启动日志
# 应该看到：
# [rot_imu]: IMU bias 补偿已启用

# 方法 2：对比补偿前后
# 补偿前：
ros2 topic echo /livox/imu --field angular_velocity

# 补偿后：
ros2 topic echo /imu/data --field angular_velocity

# 角速度均值应该明显减小
```

### Q5: 还是不稳定怎么办？

1. **检查 IMU 数据质量**：
   ```bash
   python3 src/sentry_bringup/scripts/check_imu_stability.py
   ```
   - 如果加速度噪声 > 0.1，考虑减震
   - 如果角速度持续漂移，可能是温漂

2. **切换到 FAST-LIO**（更鲁棒）

3. **调整参数**：
   ```yaml
   # 进一步降低 IMU 权重
   gyr_cov_input: 0.1    # 从 0.05 增大到 0.1
   acc_cov_input: 1.0    # 从 0.5 增大到 1.0
   ```

4. **硬件升级**：
   - 考虑使用高精度 IMU
   - 改善减震设计

---

## 技术原理

### 为什么小零漂会导致大飘移？

角速度零漂通过积分累积：
```
姿态误差 = ∫(角速度零漂) dt
```

例如 0.011 rad/s 的零漂：
- 10 秒：0.11 rad ≈ 6.3°
- 60 秒：0.66 rad ≈ 37.8°
- 5 分钟：3.3 rad ≈ 189° （翻转！）

### bias 补偿原理

```
ω_真实 = ω_测量 - ω_bias
```

通过在静止状态下测量 `ω_测量`（理论应为 0），得到 `ω_bias`，
然后在运行时减去这个 bias，得到真实角速度。

### Point-LIO vs FAST-LIO

| 特性 | Point-LIO | FAST-LIO |
|------|-----------|----------|
| IMU 融合方式 | 紧耦合 | 松耦合 |
| 对 IMU 要求 | 较高 | 较低 |
| 精度 | 更高 | 略低 |
| 鲁棒性 | 一般 | 更好 |
| 计算量 | 较大 | 较小 |

---

## 文件清单

### 新增文件

1. `/home/lab/sentry_ws/src/sentry_bringup/scripts/calibrate_imu_bias.py`
   - IMU 零漂自动校准工具
   
2. `/home/lab/sentry_ws/src/sentry_bringup/scripts/check_imu_stability.py`
   - IMU 数据质量检查工具
   
3. `/home/lab/sentry_ws/src/sentry_bringup/params/imu_bias_calibration.yaml`
   - 校准后自动生成（需运行校准程序）

### 修改文件

1. `/home/lab/sentry_ws/src/cmd_chassis/src/rot_imu.cpp`
   - 增加 IMU bias 补偿功能
   
2. `/home/lab/sentry_ws/src/sentry_bringup/launch/mapping.launch.py`
   - 自动加载 bias 校准文件
   - 延长初始化时间到 12 秒
   
3. `/home/lab/sentry_ws/src/sentry_bringup/params/point_lio_mapping_param.yaml`
   - 优化协方差和滤波参数

---

## 快速开始

**完整操作流程**：

```bash
# 1. 编译修改的代码
cd /home/lab/sentry_ws
colcon build --packages-select cmd_chassis
source install/setup.bash

# 2. 启动建图系统
ros2 launch sentry_bringup mapping.launch.py

# 3. 新开终端，进行 IMU 校准（机器人保持静止！）
cd /home/lab/sentry_ws
source install/setup.bash
python3 src/sentry_bringup/scripts/calibrate_imu_bias.py
# 等待 15 秒完成校准

# 4. 重启建图系统（Ctrl+C 停止，然后重新启动）
ros2 launch sentry_bringup mapping.launch.py

# 5. 等待 15 秒后开始慢速移动建图
```

现在飘移问题应该大幅改善！🎉
