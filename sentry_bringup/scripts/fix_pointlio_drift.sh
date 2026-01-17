#!/bin/bash
# Point-LIO 建图飘移问题 - 快速修复脚本

echo "=========================================="
echo "Point-LIO 飘移问题快速修复"
echo "=========================================="
echo ""

# 检查是否在正确的目录
if [ ! -d "/home/lab/sentry_ws/src" ]; then
    echo "❌ 错误：请在 /home/lab/sentry_ws 目录下运行此脚本"
    exit 1
fi

cd /home/lab/sentry_ws

echo "📝 步骤 1/4: 编译修改的代码"
echo ""
colcon build --packages-select cmd_chassis
if [ $? -ne 0 ]; then
    echo "❌ 编译失败，请检查错误信息"
    exit 1
fi
source install/setup.bash
echo "✅ 编译完成"
echo ""

echo "📝 步骤 2/4: 启动建图系统"
echo ""
echo "⚠️  请在新终端运行以下命令："
echo "   cd /home/lab/sentry_ws"
echo "   source install/setup.bash"
echo "   ros2 launch sentry_bringup mapping.launch.py"
echo ""
echo "按任意键继续..."
read -n 1

echo ""
echo "📝 步骤 3/4: IMU 零漂校准"
echo ""
echo "⚠️⚠️⚠️ 重要提示 ⚠️⚠️⚠️"
echo "   1. 确保机器人完全静止在水平面上"
echo "   2. 不要触碰或移动机器人"
echo "   3. 校准需要约 15 秒"
echo ""
echo "准备好了吗？按任意键开始校准..."
read -n 1

echo ""
echo "🔄 开始校准..."
source install/setup.bash
python3 src/sentry_bringup/scripts/calibrate_imu_bias.py

if [ ! -f "src/sentry_bringup/params/imu_bias_calibration.yaml" ]; then
    echo ""
    echo "❌ 校准失败：未生成校准文件"
    echo "   可能原因："
    echo "   - IMU 数据未发布（检查 /imu/data 话题）"
    echo "   - 机器人在移动"
    exit 1
fi

echo ""
echo "✅ 校准完成！"
echo ""

echo "📝 步骤 4/4: 重启建图系统"
echo ""
echo "⚠️  请执行以下操作："
echo "   1. 在运行 mapping.launch.py 的终端按 Ctrl+C 停止"
echo "   2. 重新运行："
echo "      ros2 launch sentry_bringup mapping.launch.py"
echo "   3. 等待 15 秒让 IMU 初始化"
echo "   4. 慢速移动开始建图（< 0.2 m/s）"
echo ""
echo "✅ 修复完成！"
echo ""
echo "=========================================="
echo "验证修复效果"
echo "=========================================="
echo ""
echo "可以运行以下命令验证："
echo "  python3 src/sentry_bringup/scripts/check_imu_stability.py"
echo ""
echo "期望看到："
echo "  - 角速度均值 < 0.001 rad/s"
echo "  - 加速度模长 ≈ 9.81 m/s²"
echo ""
