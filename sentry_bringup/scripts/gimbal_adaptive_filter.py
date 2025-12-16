#!/usr/bin/env python3
"""
云台运动检测与FAST-LIO参数动态调整节点
功能：检测云台旋转状态，动态调整FAST-LIO噪声参数
注意：需要先编译安装后才能用ros2 run运行
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
import math

class GimbalMotionAdaptiveFilter(Node):
    def __init__(self):
        super().__init__('gimbal_motion_adaptive_filter')
        
        # 参数
        self.declare_parameter('gimbal_angular_threshold', 0.3)  # rad/s，云台角速度阈值
        self.declare_parameter('normal_acc_cov', 0.2)
        self.declare_parameter('normal_gyr_cov', 0.3)
        self.declare_parameter('motion_acc_cov', 0.8)  # 云台运动时增大
        self.declare_parameter('motion_gyr_cov', 1.0)
        self.declare_parameter('check_interval', 0.2)  # 检查间隔(秒)
        
        self.threshold = self.get_parameter('gimbal_angular_threshold').value
        self.normal_acc = self.get_parameter('normal_acc_cov').value
        self.normal_gyr = self.get_parameter('normal_gyr_cov').value
        self.motion_acc = self.get_parameter('motion_acc_cov').value
        self.motion_gyr = self.get_parameter('motion_gyr_cov').value
        
        # 状态
        self.is_gimbal_moving = False
        self.current_mode = 'normal'
        self.gimbal_angular_vel = 0.0
        
        # 订阅IMU获取角速度（云台IMU）
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # 可选：如果有专门的云台状态话题
        # self.gimbal_sub = self.create_subscription(
        #     ..., '/gimbal/state', self.gimbal_callback, 10)
        
        # FAST-LIO参数客户端
        self.param_client = self.create_client(
            SetParameters, '/laserMapping/set_parameters')
        
        # 定时器
        interval = self.get_parameter('check_interval').value
        self.timer = self.create_timer(interval, self.check_and_adjust)
        
        self.get_logger().info('云台运动自适应滤波器已启动')
        self.get_logger().info(f'角速度阈值: {self.threshold} rad/s ({math.degrees(self.threshold):.1f}°/s)')
        
    def imu_callback(self, msg):
        # 计算角速度幅值
        gyr = msg.angular_velocity
        self.gimbal_angular_vel = math.sqrt(gyr.x**2 + gyr.y**2 + gyr.z**2)
        
        # 判断云台是否在运动（主要看z轴yaw）
        self.is_gimbal_moving = abs(gyr.z) > self.threshold
        
    def check_and_adjust(self):
        # 判断是否需要切换模式
        target_mode = 'motion' if self.is_gimbal_moving else 'normal'
        
        if target_mode != self.current_mode:
            self.current_mode = target_mode
            
            if target_mode == 'motion':
                self.get_logger().warn(
                    f'🔄 检测到云台运动 ({math.degrees(self.gimbal_angular_vel):.1f}°/s)，'
                    f'增大噪声参数')
                self.set_fastlio_params(self.motion_acc, self.motion_gyr)
            else:
                self.get_logger().info('✅ 云台停止，恢复正常参数')
                self.set_fastlio_params(self.normal_acc, self.normal_gyr)
                
    def set_fastlio_params(self, acc_cov, gyr_cov):
        """动态设置FAST-LIO参数"""
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('FAST-LIO参数服务不可用')
            return
            
        request = SetParameters.Request()
        
        # 设置加速度噪声
        param_acc = Parameter()
        param_acc.name = 'acc_cov'
        param_acc.value.type = ParameterType.PARAMETER_DOUBLE
        param_acc.value.double_value = acc_cov
        
        # 设置角速度噪声
        param_gyr = Parameter()
        param_gyr.name = 'gyr_cov'
        param_gyr.value.type = ParameterType.PARAMETER_DOUBLE
        param_gyr.value.double_value = gyr_cov
        
        request.parameters = [param_acc, param_gyr]
        
        future = self.param_client.call_async(request)
        future.add_done_callback(self.param_callback)
        
    def param_callback(self, future):
        try:
            response = future.result()
            if response.results[0].successful:
                self.get_logger().debug('参数更新成功')
            else:
                self.get_logger().warn(f'参数更新失败: {response.results[0].reason}')
        except Exception as e:
            self.get_logger().error(f'参数更新异常: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GimbalMotionAdaptiveFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
