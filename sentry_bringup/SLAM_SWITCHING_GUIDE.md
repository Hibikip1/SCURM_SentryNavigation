# SLAM算法切换指南

本仓库支持在FAST-LIO和Point-LIO之间动态切换，Point-LIO对mid360在退化运动场景下更鲁棒。

## 快速切换方法

只需修改一个配置文件即可切换SLAM算法：

**文件位置**: `sentry_bringup/params/slam_selector.yaml`

```yaml
slam_algorithm: 'point_lio'  # 选项: 'fast_lio' 或 'point_lio'
```

## 配置说明

### 使用Point-LIO（推荐用于mid360）
```yaml
slam_algorithm: 'point_lio'
```

**优势**:
- 对退化运动更鲁棒（如长走廊、空旷场景）
- 更好的IMU-LiDAR融合策略
- 适合mid360雷达特性

### 使用FAST-LIO
```yaml
slam_algorithm: 'fast_lio'
```

**优势**:
- 计算效率更高
- 在特征丰富环境表现好

## 参数文件对应关系

系统会根据选择自动加载对应的参数文件：

| 模式 | SLAM算法 | 参数文件 |
|------|----------|---------|
| 建图 | Point-LIO | `point_lio_mapping_param.yaml` |
| 建图 | FAST-LIO | `fast_lio_mapping_param.yaml` |
| 重定位 | Point-LIO | `point_lio_relocalization_param.yaml` |
| 重定位 | FAST-LIO | `fast_lio_relocalization_param.yaml` |

## 使用步骤

1. **修改配置文件**
   ```bash
   nano ~/sentry_ws/src/sentry_bringup/params/slam_selector.yaml
   ```

2. **编译工作空间**（首次使用Point-LIO需要）
   ```bash
   cd ~/sentry_ws
   colcon build --packages-select point_lio
   source install/setup.bash
   ```

3. **启动建图**
   ```bash
   ros2 launch sentry_bringup mapping.launch.py
   ```

4. **启动重定位**
   ```bash
   ros2 launch sentry_bringup relocalization.launch.py
   ```

## 调参建议

### Point-LIO关键参数（在对应yaml文件中）

- **退化场景优化**:
  ```yaml
  filter_size_surf: 0.4    # 降采样大小，退化场景可适当增大
  ivox_nearby_type: 6      # 邻域搜索类型，6点效率高
  ```

- **IMU融合**:
  ```yaml
  imu_en: True
  lidar_meas_cov: 0.01     # 雷达测量协方差，退化时可增大
  ```

- **外参估计**:
  ```yaml
  extrinsic_est_en: False  # 重定位时建议关闭，建图时可开启
  ```

## 性能对比

| 特性 | FAST-LIO | Point-LIO |
|------|----------|-----------|
| 退化鲁棒性 | 中 | 高 |
| 计算效率 | 高 | 中 |
| IMU融合 | ESKF | 误差状态卡尔曼滤波 |
| 适用场景 | 特征丰富 | 通用/退化 |

## 注意事项

1. 确保Point-LIO包已正确编译
2. 两种算法的话题名称已统一为`/state_estimation`
3. 外参配置在各自的yaml文件中保持一致
4. 建议在实际场景中测试对比效果

## 故障排查

- **找不到point_lio包**: 确保已编译并source
  ```bash
  colcon build --packages-select point_lio
  source install/setup.bash
  ```

- **参数文件错误**: 检查yaml文件格式和路径
- **话题不匹配**: 确认remapping配置正确
