# Point-LIO 障碍物方块（Octomap）显示指南

## ✅ 已完成的修复

### 1. TF链配置
创建完整的TF链：`map -> odom -> camera_init -> aft_mapped`
- Point-LIO发布：`camera_init -> aft_mapped`（动态）
- 静态TF：`map -> odom` 和 `odom -> camera_init`

### 2. Octomap启动配置
- 创建了专用的`octomap_server_pointlio.launch.py`
- Point-LIO模式：使用不冲突的octomap配置
- FAST-LIO模式：保持原有配置

### 3. 验证结果
```
✅ 点云话题发布正常：~10 Hz
✅ Octomap话题发布正常：3个发布者
✅ Terrain map发布正常：2个发布者  
✅ TF链连接正常：map -> odom -> camera_init
✅ Octomap服务器运行正常
✅ Point-LIO节点运行正常
```

## RViz中查看障碍物方块

### 启动系统
```bash
cd ~/sentry_ws
source install/setup.bash
ros2 launch sentry_bringup mapping.launch.py
```

### RViz配置检查

1. **Fixed Frame设置**
   - Global Options → Fixed Frame → 选择 **`map`**（推荐）
   - 或者选择 `camera_init`（也可以）

2. **启用OccupancyGrid显示**
   - 在Displays面板中找到 "OccupancyGrid"
   - 确保已勾选✓启用
   - Topic应该是 `/octomap_binary`

3. **点云显示**
   - CloudRegistered应该已启用
   - Topic: `/cloud_registered`
   - Frame: `camera_init`

### 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cloud_registered` | PointCloud2 | Point-LIO发布的配准点云 |
| `/octomap_binary` | Octomap | 二进制八叉树地图 |
| `/octomap_point_cloud_centers` | PointCloud2 | Octomap点云中心 |
| `/terrain_map_at_scan` | PointCloud2 | 地形分析结果 |
| `/state_estimation` | Odometry | 状态估计（重映射自/Odometry） |

### TF树结构

```
map (世界坐标系)
 │
 └─ odom (里程计坐标系，静态连接)
     │
     └─ camera_init (Point-LIO世界坐标系，静态连接)
         │
         └─ aft_mapped (Point-LIO估计的位姿，动态)
             │
             └─ body (机器人本体)
```

## 故障排查

### 1. 看不到障碍物方块

**检查Octomap服务器**：
```bash
ros2 node list | grep octomap
# 应该看到: /octomap_server
```

**检查话题**：
```bash
ros2 topic list | grep octomap
# 应该看到: /octomap_binary, /octomap_point_cloud_centers
```

**检查发布频率**：
```bash
ros2 topic hz /octomap_binary
# 应该有数据发布
```

### 2. Octomap显示但没有障碍物

**原因**：可能点云数据太少或高度过滤设置不当

**解决**：检查参数
```yaml
# 在octomap_server_pointlio.launch.py中
point_cloud_min_z: 0.15  # 最小高度，降低此值可以包含更多点
resolution: 0.1           # 分辨率，降低可以更精细
```

### 3. TF错误

**检查TF链**：
```bash
ros2 run rqt_tf_tree rqt_tf_tree
# 应该看到完整的 map->odom->camera_init->aft_mapped 链
```

**检查TF转换**：
```bash
ros2 run tf2_ros tf2_echo map camera_init
# 应该能看到变换数据
```

### 4. 点云有但Octomap空

**原因**：地形分析或传感器扫描生成失败

**检查中间话题**：
```bash
# 地形分析扩展
ros2 topic hz /terrain_map_ext

# 交换后的地形图
ros2 topic hz /terrain_map_ext_exchanged

# 传感器扫描
ros2 topic hz /terrain_map_at_scan
```

## 导航使用

障碍物方块（Octomap）可用于：
- **Costmap**：导航规划的障碍物层
- **地图保存**：保存当前环境地图
- **障碍物检测**：实时更新的3D障碍物信息

### 集成到Nav2

在`nav2_params.yaml`中配置：
```yaml
global_costmap:
  plugins: ["static_layer", "obstacle_layer", "voxel_layer"]
  obstacle_layer:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    observation_sources: octomap_scan
    octomap_scan:
      topic: /octomap_point_cloud_centers
      sensor_frame: camera_init
```

## 性能优化

### 降低计算负载
```yaml
# 调整octomap分辨率（更大=更快）
resolution: 0.2  # 默认0.1

# 调整更新频率
# 在terrain_analysis参数中降低频率
```

### 提高精度
```yaml
# 更小的分辨率
resolution: 0.05

# 启用滤波
filter_speckles: True
```

## 与FAST-LIO的对比

| 特性 | FAST-LIO | Point-LIO |
|------|----------|-----------|
| 世界坐标系 | odom | camera_init |
| Octomap支持 | 原生支持 | 需要TF桥接 |
| TF链 | map->odom直接 | map->odom->camera_init |
| 退化鲁棒性 | 中 | 高 |

## 当前配置文件

- Launch: [mapping.launch.py](../launch/mapping.launch.py)
- Octomap (Point-LIO): [octomap_server_pointlio.launch.py](../launch/octomap_server_pointlio.launch.py)
- Octomap (FAST-LIO): [octomap_server_intensity.launch.py](../launch/octomap_server_intensity.launch.py)
- SLAM选择器: [slam_selector.yaml](../params/slam_selector.yaml)
