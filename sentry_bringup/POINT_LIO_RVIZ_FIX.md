# Point-LIO RViz可视化问题解决方案

## 问题原因
Point-LIO使用`camera_init`作为世界坐标系发布点云，而RViz的Fixed Frame设置为`map`，导致点云无法显示。

## 已实施的解决方案

### 1. 自动添加TF支持
在`mapping.launch.py`和`relocalization.launch.py`中：
- Point-LIO模式：自动发布`map` -> `camera_init`的静态TF
- FAST-LIO模式：保持原有配置

### 2. 避免TF冲突  
Point-LIO模式下禁用octomap_server（它会发布冲突的`map` -> `odom` TF）

## RViz配置方法（两种选择）

### 方法1：修改RViz Fixed Frame（推荐）
在RViz中：
1. 点击左侧 "Displays" 面板
2. 展开 "Global Options"
3. 将 "Fixed Frame" 从 `map` 改为 `camera_init`
4. 点云应该立即显示

### 方法2：使用TF桥接（自动，已实现）
系统已自动添加`map` -> `camera_init`的TF，理论上RViz使用`map`作为Fixed Frame也能工作。

## 验证步骤

```bash
# 1. 启动系统
cd ~/sentry_ws
source install/setup.bash
ros2 launch sentry_bringup mapping.launch.py

# 2. 在另一个终端检查TF
source ~/sentry_ws/install/setup.bash
ros2 run tf2_ros tf2_echo camera_init aft_mapped

# 3. 检查点云话题
ros2 topic hz /cloud_registered

# 4. 查看TF树
ros2 run rqt_tf_tree rqt_tf_tree
```

## 预期的TF树结构（Point-LIO）

```
map
 └─ camera_init (static)
     └─ aft_mapped (dynamic, from Point-LIO)
         └─ body
```

## 注意事项

1. **Point-LIO模式**：
   - 世界坐标系：`camera_init`
   - 点云frame：`camera_init`
   - Odometry child_frame：`body`
   
2. **FAST-LIO模式**：
   - 世界坐标系：`camera_init`（FAST-LIO内部）
   - 外部使用：`map` -> `odom` -> `base_link`

3. **如果点云仍不显示**：
   - 检查RViz的CloudRegistered是否启用（勾选）
   - 确认Fixed Frame设置正确
   - 检查`/cloud_registered`话题是否有数据：`ros2 topic echo /cloud_registered --once`

## 临时解决方案（如果还有问题）

如果TF桥接仍有问题，可以直接修改Point-LIO源码中的frame_id：

```cpp
// 在 point_lio/src/laserMapping.cpp 中
// 将 "camera_init" 改为 "map"
laserCloudmsg.header.frame_id = "map";  // 原来是 "camera_init"
odomAftMapped.header.frame_id = "map";  // 原来是 "camera_init"
```

但这不推荐，因为会破坏Point-LIO的原始设计。

## 快速测试

```bash
# 一键测试脚本
cd ~/sentry_ws
source install/setup.bash

# 后台启动
ros2 launch sentry_bringup mapping.launch.py &

# 等待5秒
sleep 5

# 检查点云发布
ros2 topic hz /cloud_registered
```
