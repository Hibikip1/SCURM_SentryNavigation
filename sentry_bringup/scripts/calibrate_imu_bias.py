#!/usr/bin/env python3
"""
IMU 零漂校准脚本
在机器人静止状态下运行，计算 IMU 的角速度和加速度零漂
将结果保存到 YAML 文件供 Point-LIO 使用
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class IMUBiasCalibrator(Node):
    def __init__(self):
        super().__init__('imu_bias_calibrator')
        
        # 订阅转换后的 IMU 数据（已经过坐标变换）
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # 数据缓冲区
        self.acc_buffer = []
        self.gyro_buffer = []
        
        # 校准参数
        self.calibration_samples = 2000  # 采集2000个样本（约10秒@200Hz）
        self.is_calibrating = True
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('IMU 零漂校准程序')
        self.get_logger().info('=' * 70)
        self.get_logger().info('⚠️  请确保机器人完全静止在水平面上！')
        self.get_logger().info(f'📊 将采集 {self.calibration_samples} 个样本进行校准...')
        self.get_logger().info('')
    
    def imu_callback(self, msg):
        if not self.is_calibrating:
            return
        
        # 收集数据
        self.acc_buffer.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.gyro_buffer.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # 显示进度
        if len(self.gyro_buffer) % 200 == 0:
            progress = len(self.gyro_buffer) / self.calibration_samples * 100
            self.get_logger().info(f'📈 采集进度: {progress:.1f}% ({len(self.gyro_buffer)}/{self.calibration_samples})')
        
        # 完成校准
        if len(self.gyro_buffer) >= self.calibration_samples:
            self.is_calibrating = False
            self.compute_and_save_bias()
    
    def compute_and_save_bias(self):
        # 计算统计信息
        acc_data = np.array(self.acc_buffer)
        gyro_data = np.array(self.gyro_buffer)
        
        gyro_bias = gyro_data.mean(axis=0)
        acc_bias = acc_data.mean(axis=0)
        
        gyro_std = gyro_data.std(axis=0)
        acc_std = acc_data.std(axis=0)
        
        # 计算重力方向（用于验证）
        gravity_measured = np.linalg.norm(acc_bias)
        gravity_z = acc_bias[2]
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('✅ 校准完成！')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('📊 校准结果:')
        self.get_logger().info(f'  角速度零漂 (rad/s):')
        self.get_logger().info(f'    X: {gyro_bias[0]:+.6f} ± {gyro_std[0]:.6f}')
        self.get_logger().info(f'    Y: {gyro_bias[1]:+.6f} ± {gyro_std[1]:.6f}')
        self.get_logger().info(f'    Z: {gyro_bias[2]:+.6f} ± {gyro_std[2]:.6f}')
        self.get_logger().info('')
        self.get_logger().info(f'  加速度偏置 (m/s²):')
        self.get_logger().info(f'    X: {acc_bias[0]:+.6f} ± {acc_std[0]:.6f}')
        self.get_logger().info(f'    Y: {acc_bias[1]:+.6f} ± {acc_std[1]:.6f}')
        self.get_logger().info(f'    Z: {acc_bias[2]:+.6f} ± {acc_std[2]:.6f}')
        self.get_logger().info('')
        self.get_logger().info(f'  重力加速度: {gravity_measured:.3f} m/s² (理论值: 9.81)')
        self.get_logger().info(f'  Z 轴重力分量: {gravity_z:.3f} m/s²')
        self.get_logger().info('')
        
        # 诊断建议
        self.get_logger().info('🔍 诊断:')
        if abs(gravity_measured - 9.81) > 0.5:
            self.get_logger().warn(f'  ⚠️  重力加速度异常！请检查 IMU 安装')
        else:
            self.get_logger().info(f'  ✓ 重力加速度正常')
        
        if gravity_z < 8.0:
            self.get_logger().warn(f'  ⚠️  Z 轴重力分量过小，坐标系可能不正确')
        else:
            self.get_logger().info(f'  ✓ Z 轴对齐正常')
        
        if np.any(np.abs(gyro_bias) > 0.02):
            self.get_logger().warn(f'  ⚠️  角速度零漂较大，建议硬件校准')
        else:
            self.get_logger().info(f'  ✓ 角速度零漂在可接受范围')
        
        self.get_logger().info('')
        
        # 创建补偿后的 rot_imu 节点配置
        self.save_imu_bias_config(gyro_bias, acc_bias)
        
        # 更新 Point-LIO 参数建议
        self.suggest_pointlio_params(gyro_bias, acc_bias, gyro_std, acc_std)
        
        self.get_logger().info('=' * 70)
    
    def save_imu_bias_config(self, gyro_bias, acc_bias):
        """保存 IMU bias 到配置文件"""
        config = {
            'imu_bias_calibration': {
                'gyro_bias_x': float(gyro_bias[0]),
                'gyro_bias_y': float(gyro_bias[1]),
                'gyro_bias_z': float(gyro_bias[2]),
                'acc_bias_x': float(acc_bias[0]),
                'acc_bias_y': float(acc_bias[1]),
                'acc_bias_z': float(acc_bias[2] - 9.81),  # 减去重力
                'calibration_time': str(self.get_clock().now().to_msg())
            }
        }
        
        # 保存到 workspace
        config_dir = '/home/lab/sentry_ws/src/sentry_bringup/params'
        config_file = os.path.join(config_dir, 'imu_bias_calibration.yaml')
        
        with open(config_file, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
        
        self.get_logger().info(f'💾 IMU bias 配置已保存到:')
        self.get_logger().info(f'   {config_file}')
        self.get_logger().info('')
    
    def suggest_pointlio_params(self, gyro_bias, acc_bias, gyro_std, acc_std):
        """根据校准结果建议 Point-LIO 参数"""
        self.get_logger().info('💡 Point-LIO 参数建议:')
        self.get_logger().info('')
        self.get_logger().info('  建议在 point_lio_mapping_param.yaml 中设置:')
        self.get_logger().info('')
        
        # 根据测量的噪声水平建议协方差
        gyro_cov = max(0.01, float(np.mean(gyro_std)) * 10)
        acc_cov = max(0.1, float(np.mean(acc_std)) * 10)
        
        self.get_logger().info(f'    gyr_cov_input: {gyro_cov:.4f}')
        self.get_logger().info(f'    acc_cov_input: {acc_cov:.4f}')
        self.get_logger().info(f'    imu_meas_omg_cov: {gyro_cov:.4f}')
        self.get_logger().info(f'    imu_meas_acc_cov: {acc_cov:.4f}')
        self.get_logger().info('')
        
        # 如果零漂大，建议调整 bias 协方差
        if np.any(np.abs(gyro_bias) > 0.01):
            self.get_logger().info('  ⚠️  由于角速度零漂较大，还建议:')
            self.get_logger().info('    b_gyr_cov: 0.0001  # 允许估计更大的 bias 变化')
            self.get_logger().info('')

def main(args=None):
    rclpy.init(args=args)
    node = IMUBiasCalibrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
