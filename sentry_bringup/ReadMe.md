# sentry_bringup

## Launch文件说明

| Launch File Name | Description |
|--------------|-------------|
| ✅tb3_simulation_launch + bringup.launch.py + localization_launch.py + navigation_launch.py | nav2 routines, notice the call order |
| ✅fastlio_mapping.launch.py | urdf + mid360 + fastlio mapping |
| sentry_launch_tup_gh.py | ? |
| ✅sentry_bringup.launch.py + navigation_launch.py | final launch file, bring up sensor driver, odom, relocalization and nav2 stack |

> 25赛季更新：适应赛场地形的MPPI参数；对于先验地图不理想的情况可以用手动摆正起始点、无需重定位的方案