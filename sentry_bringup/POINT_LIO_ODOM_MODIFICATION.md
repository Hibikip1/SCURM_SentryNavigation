# Point-LIO 坐标系修改说明

## ✅ 已完成的修改

### 核心修改
将Point-LIO的世界坐标系从 **`camera_init`** 改为 **`odom`**，与FAST-LIO保持完全一致。

### 修改文件
- **源码**: [point_lio/src/laserMapping.cpp](../../point_lio/src/laserMapping.cpp)
  - 6处frame_id从 `"camera_init"` 改为 `"odom"`
  
- **Launch**: [mapping.launch.py](../launch/mapping.launch.py)
  - 移除了Point-LIO的额外TF配置
  - 使用统一的octomap配置

## 验证结果

```
✅ Point-LIO节点正常运行
✅ 点云frame_id: odom
✅ 里程计frame_id: odom  
✅ TF链: map -> odom -> aft_mapped -> body
✅ Octomap服务器正常工作
✅ 障碍物方块正常生成
```

## TF树结构（统一后）

### Point-LIO
```
map
 └─ odom (来自octomap_server)
     └─ aft_mapped (Point-LIO发布，动态)
         └─ body
```

### FAST-LIO
```
map
 └─ odom (来自octomap_server)
     └─ body/base_link (FAST-LIO发布，动态)
```

## 优势

1. **简化配置** - 无需额外的TF桥接
2. **完全兼容** - 与FAST-LIO使用相同的坐标系
3. **导航就绪** - Octomap、Nav2等直接可用
4. **统一管理** - 所有SLAM算法使用相同的TF结构

## 使用方法

### 启动建图
```bash
cd ~/sentry_ws
source install/setup.bash
ros2 launch sentry_bringup mapping.launch.py
```

### RViz配置
- **Fixed Frame**: `map` (推荐) 或 `odom`
- **点云显示**: CloudRegistered (frame: odom)
- **障碍物**: OccupancyGrid (frame: map)

### 切换SLAM算法
修改 [slam_selector.yaml](../params/slam_selector.yaml):
```yaml
slam_algorithm: 'point_lio'  # 或 'fast_lio'
```

两种算法现在使用完全相同的TF结构，切换无缝！

## 对比：修改前后

| 项目 | 修改前 | 修改后 |
|------|--------|--------|
| 世界坐标系 | camera_init | odom |
| TF链 | map->odom->camera_init->aft_mapped | map->odom->aft_mapped |
| 与FAST-LIO兼容性 | 需要TF桥接 | 完全一致 |
| Octomap支持 | 需要特殊配置 | 原生支持 |
| RViz Fixed Frame | camera_init/需调整 | map/odom通用 |

## 技术细节

### 修改的代码位置
```cpp
// 点云发布 (2处)
laserCloudmsg.header.frame_id = "odom";

// 里程计发布
odomAftMapped.header.frame_id = "odom";

// TF发布
transform.header.frame_id = "odom";
transform.child_frame_id = "aft_mapped";

// 路径发布 (2处)
msg_body_pose.header.frame_id = "odom";
path.header.frame_id = "odom";
```

### 为什么选择odom而不是map？
1. **兼容性**: ROS导航栈标准是 map -> odom -> base_link
2. **一致性**: 与FAST-LIO保持一致
3. **灵活性**: octomap_server负责发布map->odom，SLAM只需关注odom坐标系

## 注意事项

1. **重新编译**: 修改后必须重新编译Point-LIO
   ```bash
   cd ~/sentry_ws
   rm -rf build/point_lio install/point_lio
   colcon build --packages-select point_lio
   ```

2. **保存地图**: 如果使用先验地图，确保地图frame_id与新配置一致

3. **导航集成**: Nav2配置中的odom_frame应该设为"odom"

## 故障排查

### 点云不显示
- 检查RViz Fixed Frame设置
- 确认 `/cloud_registered` 话题的frame_id是"odom"

### Octomap不生成
- 检查octomap_server是否运行：`ros2 node list | grep octomap`
- 查看日志：`tail -f /tmp/pointlio_odom_test.log`

### TF错误
- 检查TF树：`ros2 run rqt_tf_tree rqt_tf_tree`
- 应该看到：map -> odom -> aft_mapped

## 性能影响

修改坐标系名称**不影响算法性能**，仅改变frame_id字符串，Point-LIO的：
- ✅ 退化鲁棒性保持不变
- ✅ 计算效率保持不变  
- ✅ 定位精度保持不变
- ✅ IMU融合策略保持不变

## 相关文件

- 源码修改: [point_lio/src/laserMapping.cpp](../../point_lio/src/laserMapping.cpp)
- Launch配置: [mapping.launch.py](../launch/mapping.launch.py)
- SLAM选择: [slam_selector.yaml](../params/slam_selector.yaml)
- Octomap配置: [octomap_server_pointlio.launch.py](../launch/octomap_server_pointlio.launch.py)
