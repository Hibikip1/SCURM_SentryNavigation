# Point-LIO 建图飘移问题解决方案

## 问题描述
Point-LIO 建图时轻轻一动整个坐标系和点云就飘走，说明系统存在稳定性问题。

## 根本原因

1. **IMU 初始化不充分**：Point-LIO 依赖 IMU 进行状态估计，启动时需要静止状态下进行重力方向估计
2. **参数配置不当**：默认参数可能不适合当前的 IMU 噪声特性
3. **坐标系变换问题**：雷达倾斜放置后的坐标变换可能引入误差

## 已实施的优化

### 1. 参数优化 (point_lio_mapping_param.yaml)

```yaml
# 关键参数调整：
point_filter_num: 3              # 从 4 降到 3，提高稳定性
filter_size_surf: 0.5            # 从 0.4 增大到 0.5，减少噪声
filter_size_map: 0.5             # 从 0.4 增大到 0.5
lidar_meas_cov: 0.001           # 从 0.01 降到 0.001，提高点云约束权重
acc_cov_input: 0.5              # 从 0.1 增到 0.5，降低 IMU 预测权重
gyr_cov_input: 0.05             # 从 0.01 增到 0.05
plane_thr: 0.05                 # 从 0.1 降到 0.05，要求更平的平面
ivox_grid_resolution: 1.5       # 从 2.0 降到 1.5，提高地图精度
```

**核心思路**：降低 IMU 权重，提高激光点云约束权重，使系统更依赖可靠的激光数据。

### 2. 延长初始化时间 (mapping.launch.py)

```python
delayed_start_mapping = TimerAction(
    period=12.0,  # 从 8 秒增加到 12 秒
    actions=[slam_node, start_octomap_server]
)
```

## 使用建议

### 启动前准备

1. **确保机器人完全静止**：
   - 放置在平稳地面
   - 启动后 **保持静止至少 15 秒**
   - 等待 Point-LIO 节点启动后再等待 3-5 秒

2. **检查 IMU 数据质量**：
   ```bash
   cd /home/lab/sentry_ws
   source install/setup.bash
   
   # 运行 IMU 稳定性检查（保持机器人静止）
   python3 src/sentry_bringup/scripts/check_imu_stability.py
   ```

   期望输出：
   - 角速度均值应接近 0（< 0.01 rad/s）
   - 加速度模长应接近 9.81 m/s²
   - Z 轴加速度应 > 8.0 m/s²

### 启动流程

```bash
# 1. 启动建图
ros2 launch sentry_bringup mapping.launch.py

# 2. 观察终端输出，等待以下信息：
#    - "Livox MID360 connected"
#    - Point-LIO 初始化完成

# 3. 等待至少 15 秒后再移动机器人

# 4. 缓慢移动机器人开始建图
#    - 初始移动速度 < 0.2 m/s
#    - 初始旋转速度 < 0.3 rad/s
```

### 建图过程注意事项

1. **慢速启动**：
   - 前 30 秒内保持低速移动
   - 避免急加速、急转弯
   - 让系统有时间建立稳定的地图

2. **避免激烈运动**：
   - 线速度 < 0.5 m/s
   - 角速度 < 1.0 rad/s
   - 避免颠簸路面

3. **保持特征丰富的环境**：
   - 避免长走廊、空旷区域
   - 保持周围有足够的几何特征
   - 避免动态物体过多的场景

## 诊断工具

### 1. IMU 稳定性检查
```bash
python3 src/sentry_bringup/scripts/check_imu_stability.py
```

输出包括：
- 原始 IMU 数据统计
- 转换后 IMU 数据统计
- 自动诊断建议

### 2. 实时监控话题

```bash
# 监控 Point-LIO 状态估计
ros2 topic echo /state_estimation --no-arr

# 检查 IMU 数据
ros2 topic echo /imu/data --no-arr

# 检查点云数据
ros2 topic hz /livox/lidar
```

### 3. 查看 TF 树
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

检查是否有异常的 TF 跳变。

## 进一步优化选项

如果问题仍然存在，可以尝试：

### 1. 禁用外参估计
如果外参已经校准好，可以设置：
```yaml
extrinsic_est_en: False  # 在 point_lio_mapping_param.yaml 中
```

### 2. 调整 IMU 积分频率
对于噪声较大的 IMU：
```yaml
imu_time_inte: 0.01  # 从 0.005 增加到 0.01
```

### 3. 增加地图分辨率
如果环境特征丰富：
```yaml
ivox_grid_resolution: 1.0  # 从 1.5 降到 1.0
```

### 4. 使用 IMU 作为输入（实验性）
如果 IMU 质量很好：
```yaml
use_imu_as_input: True
```

## 与 FAST-LIO 对比

如果 Point-LIO 仍不稳定，可以切换回 FAST-LIO：

```bash
# 编辑 slam_selector.yaml
nano src/sentry_bringup/params/slam_selector.yaml

# 修改为：
slam_algorithm: 'fast_lio'
```

FAST-LIO 通常更稳定，但精度可能稍低。

## 常见问题

### Q1: 为什么需要静止这么久？
A: Point-LIO 需要在静止状态下估计 IMU 的零漂和重力方向，这是准确状态估计的基础。

### Q2: 参数调整后还是飘怎么办？
A: 可能是硬件问题：
- 检查 IMU 连接是否牢固
- 检查雷达安装是否稳固
- 尝试重新校准外参

### Q3: 如何知道初始化成功？
A: 观察以下指标：
- RViz 中点云稳定不抖动
- `/state_estimation` 话题协方差收敛
- IMU 检查脚本显示正常

## 文件修改记录

1. `/home/lab/sentry_ws/src/sentry_bringup/params/point_lio_mapping_param.yaml`
   - 优化滤波和协方差参数
   
2. `/home/lab/sentry_ws/src/sentry_bringup/launch/mapping.launch.py`
   - 延长启动延迟时间到 12 秒
   
3. `/home/lab/sentry_ws/src/sentry_bringup/scripts/check_imu_stability.py`
   - 新增 IMU 诊断工具
