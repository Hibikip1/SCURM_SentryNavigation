# Point-LIO 启动问题诊断

## 问题现象
1. RViz中没有点云显示
2. 坐标系轻微移动就跑飞  
3. 没有障碍物方块

## 根本原因

**Point-LIO节点没有收到LiDAR/IMU数据**，导致：
- ❌ 没有发布 `odom` frame（因为SLAM没有运行）
- ❌ 没有发布 `/cloud_registered` 点云
- ❌ 没有里程计输出
- ❌ octomap没有输入数据，所以没有障碍物

## 正确的启动流程

### 1. 清理所有旧进程
```bash
killall -9 ros2 pointlio_mapping 2>/dev/null
sleep 2
```

### 2. 完整启动系统
```bash
cd ~/sentry_ws
source install/setup.bash
ros2 launch sentry_bringup mapping.launch.py
```

**重要：不要中途Ctrl+C！** 让系统完全启动，等待8秒延迟启动完成。

### 3. 验证所有节点正常运行

#### 3.1 检查关键节点（另开终端）
```bash
cd ~/sentry_ws && source install/setup.bash
ros2 node list | grep -E "(livox|pointlio|octomap)"
```

**应该看到：**
```
/livox_lidar_publisher
/pointlio_mapping
/octomap_server
```

#### 3.2 检查LiDAR数据
```bash
ros2 topic hz /livox/lidar
```
**应该看到:** `average rate: 10.000`

#### 3.3 检查IMU数据  
```bash
ros2 topic hz /imu/data
```
**应该看到:** `average rate: 200.000` (或类似值)

#### 3.4 检查点云输出
```bash
ros2 topic hz /cloud_registered
```
**应该看到:** `average rate: 10.000`

#### 3.5 检查TF树
```bash
ros2 run tf2_ros tf2_echo map odom
```
**应该看到：** 平移和旋转数据更新

```bash
ros2 run tf2_ros tf2_echo odom aft_mapped
```
**应该看到：** 随机器人运动变化的TF

## RViz配置

### Fixed Frame设置
- **推荐:** `map`
- 备选: `odom`

### 需要显示的项目
1. **CloudRegistered** (PointCloud2)
   - Topic: `/cloud_registered`
   - Size: 0.01
   - Style: Points

2. **OccupancyGrid**  
   - Topic: `/projected_map`
   - Color Scheme: map

3. **TF** - 显示坐标系
   - 应该看到: map -> odom -> aft_mapped -> body

## 常见问题

### Q1: Point-LIO进程存在但不发布数据
**原因:** 没有收到LiDAR/IMU输入
**解决:** 检查livox_ros_driver2是否正常运行

### Q2: 看到 "imu loop back, clear deque" 错误
**原因:** IMU时间戳异常或顺序错乱
**解决:** 
1. 检查IMU话题质量: `ros2 topic hz /imu/data`
2. 确认 `rot_imu` 节点正常运行
3. 等待IMU初始化完成（约3-5秒）

### Q3: RViz显示camera_init frame
**原因:** RViz配置文件保存了旧的TF显示
**解决:** 
1. 在RViz左侧TF显示中取消勾选camera_init
2. File -> Save Config As 保存新配置

### Q4: Octomap没有障碍物方块
**原因:** Point-LIO的点云质量或octomap配置问题
**解决:**
1. 确认 `/cloud_registered` 有数据: `ros2 topic hz /cloud_registered`
2. 确认terrain_analysis正常: `ros2 node list | grep terrain`
3. 检查octomap参数中的高度过滤: `point_cloud_min_z: 0.15`

## 完整诊断脚本

创建文件 `/tmp/check_pointlio.sh`:

```bash
#!/bin/bash

echo "========== Point-LIO 系统诊断 =========="
cd ~/sentry_ws && source install/setup.bash

echo -e "\n1. 检查关键节点..."
NODES=$(ros2 node list 2>/dev/null)
echo "$NODES" | grep -q "livox_lidar_publisher" && echo "✅ Livox驱动运行中" || echo "❌ Livox驱动未运行"
echo "$NODES" | grep -q "pointlio_mapping" && echo "✅ Point-LIO运行中" || echo "❌ Point-LIO未运行"  
echo "$NODES" | grep -q "octomap_server" && echo "✅ Octomap运行中" || echo "❌ Octomap未运行"

echo -e "\n2. 检查话题数据..."
timeout 2 ros2 topic hz /livox/lidar 2>&1 | grep -q "average rate" && echo "✅ LiDAR数据正常" || echo "❌ LiDAR无数据"
timeout 2 ros2 topic hz /imu/data 2>&1 | grep -q "average rate" && echo "✅ IMU数据正常" || echo "❌ IMU无数据"
timeout 2 ros2 topic hz /cloud_registered 2>&1 | grep -q "average rate" && echo "✅ 点云输出正常" || echo "❌ 点云无输出"

echo -e "\n3. 检查TF树..."
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "Translation" && echo "✅ map->odom TF存在" || echo "❌ map->odom TF缺失"
timeout 2 ros2 run tf2_ros tf2_echo odom aft_mapped 2>&1 | grep -q "Translation" && echo "✅ odom->aft_mapped TF存在" || echo "❌ odom->aft_mapped TF缺失"

echo -e "\n4. 检查点云frame_id..."
FRAME=$(timeout 2 ros2 topic echo /cloud_registered --field header.frame_id --once 2>/dev/null)
if [ "$FRAME" == "odom" ]; then
    echo "✅ 点云frame_id正确: odom"
elif [ -z "$FRAME" ]; then
    echo "❌ 无法获取点云frame_id（话题无数据）"
else
    echo "⚠️  点云frame_id: $FRAME（应该是odom）"
fi

echo -e "\n========== 诊断完成 =========="
```

运行诊断:
```bash
chmod +x /tmp/check_pointlio.sh
bash /tmp/check_pointlio.sh
```

## 成功的标志

当系统正常运行时，你应该看到：

1. **终端输出:**
   ```
   [pointlio_mapping-8] [INFO] [xxx]: IMU Initializing: 100.0 %
   [pointlio_mapping-8] [INFO] [xxx]: Publish pointcloud, avg time: xxx ms
   ```

2. **RViz显示:**
   - ✅ 白色/彩色点云实时更新
   - ✅ 绿色障碍物方块（octomap）
   - ✅ TF轴: map(红绿蓝) -> odom -> aft_mapped
   - ✅ 机器人路径轨迹

3. **话题频率:**
   - `/livox/lidar`: ~10 Hz
   - `/imu/data`: ~200 Hz
   - `/cloud_registered`: ~10 Hz
   - `/projected_map`: ~0.5-1 Hz

## 如果问题仍然存在

1. **重新编译Point-LIO**
   ```bash
   cd ~/sentry_ws
   rm -rf build/point_lio install/point_lio
   colcon build --packages-select point_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

2. **检查日志**
   ```bash
   ls -lt ~/.ros/log/ | head -5
   # 进入最新的日志目录查看launch.log
   ```

3. **对比FAST-LIO**
   ```bash
   # 修改slam_selector.yaml
   slam_algorithm: 'fast_lio'
   # 重新启动，确认FAST-LIO是否正常
   ```

## 相关文档
- [SLAM切换指南](SLAM_SWITCHING_GUIDE.md)
- [Point-LIO坐标系修改说明](POINT_LIO_ODOM_MODIFICATION.md)
- [Octomap配置指南](POINT_LIO_OCTOMAP_GUIDE.md)
