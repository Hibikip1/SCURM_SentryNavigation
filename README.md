# 

开源的初衷是想把自己的一些好的想法和大家一起分享，所以欢迎各位联系我和我讨论，欢迎PR，提issue🫠~

主要的创新：

1. 实现FAST-LIO2的重定位模式，不需要另外运行重定位算法。算力需求小，使得整个框架在NUC12上的资源占用在30%左右；精度高，定位误差小；内存占用不会随着时间增长。

2. 改进navigation2的故障恢复行为，卡住时机器人会向无碰撞的方向运动。

算法框架和思路详见技术报告

Gazebo仿真指路👉[SCURM_Nav_Tutorial](https://github.com/PolarisXQ/SCURM_Nav_Tutorial.git)

Docker镜像使用方法参阅[DevcontainterGuide](./DevcontainterGuide.md)

## 包说明

| Package Name | Description |
|--------------|-------------|
| ✅auto_aim_interfaces | 自瞄接口 |
| ✅[autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) | 地形分析包terrain_analysis和terrain_analysis_ext，其他的是小工具无关紧要 |
| ✅[BehaviourTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) | <span style="color:red">**MODIFIED**</span> BehaviourTree lib |
| ✅cmd_chassis | twist2chassis_cmd：将twist加上底盘的控制方式（如是否小陀螺），发出到串口接收的话题；<br> twist_transformer， fake_joint用于实现底盘到云台的速度解算 |
| ✅control_panel | 模仿裁判系统发消息 |
| ✅FAST_LIO | 修改版fastlio，具备建图和重定位功能（须配合icp_relocalizatiion使用） |
| ✅icp_relocalization | 基于icp实现的重定位，须配合修改版FAST_LIO使用 |
| ✅livox_ros_driver2 | livox雷达驱动 |
| ✅nav2_plugins <br> - behavior_ext_plugins <br> - costmap_intensity | Costume nav2 plugins <br> - an enhenced back_up action that move toward free space <br> - 2 costmap_2d layer that use intensity filed of pointcloud msg rather than height (use with terrain analysis in autonomous_exploration_development_environment) |
| ✅rm_decision_cpp | 烧饼决策系统 |
| ✅rm_interfaces | 通讯协议 |
| ✅sentry_bringup | 哨兵启动文件 |
| ✅sentry_description | 烧饼urdf |

## LAUNCH

### MAPPING

- launch mapping node

```bash
ros2 launch sentry_bringup mapping.launch.py
```

- save map

```bash
# occupancy grid map
ros2 run nav2_map_server map_saver_cli -t /projected_map -f test_map --fmt png
# save pcd
ros2 service call /map_save std_srvs/srv/Trigger
```

- then terminate all nodes, pcd file will be saved in /PCD/scans.pcd
    
### MAP PROCESSING(使用CloudCompare删除建图中的人物残影, 如不处理可以跳过，但记得按照最后一步的提示替换掉pcd文件)

process pcd file
- drag the pcd file to CloudCompare(globalmap or scans.pcd), select the pointcloud in the left panel, then tools->clean->SOR filter, set the parameters (25,1 is a baseline) and apply
- select the processed pointcloud from last step, then tools->segmentation->Label Connected Components, set the parameters and apply
- pick out the CC#0(ususally this one), then tools->Other->Remove duplicate points, keep 1 point per 0.01-0.1m to reduce the size of the pointcloud
- select the processed pointcloud, then file->save as, select .pcd format
- replace the original pcd file with the processed one. Globally seach and replace '/home/sentry_ws/src/sentry_bringup/maps/GlobalMap.pcd' with '/path/to/your/map'

### LAUNCH ALL

```bash
ros2 launch sentry_bringup bringup_all_in_one.launch.py
```
