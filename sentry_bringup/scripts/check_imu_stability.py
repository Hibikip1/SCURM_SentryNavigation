#!/usr/bin/env python3
"""
IMU 稳定性检查脚本
用于诊断 Point-LIO 建图时的 IMU 数据质量
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from collections import deque
import time

class IMUStabilityChecker(Node):
    def __init__(self):
        super().__init__('imu_stability_checker')
        
        # 订阅原始 IMU 和转换后的 IMU
        self.raw_imu_sub = self.create_subscription(
            Imu, '/livox/imu', self.raw_imu_callback, 10)
        self.transformed_imu_sub = self.create_subscription(
            Imu, '/imu/data', self.transformed_imu_callback, 10)
        
        # 数据缓冲区 (5秒数据)
        self.raw_acc_buffer = deque(maxlen=1000)
        self.raw_gyro_buffer = deque(maxlen=1000)
        self.trans_acc_buffer = deque(maxlen=1000)
        self.trans_gyro_buffer = deque(maxlen=1000)
        
        # 创建定时器每5秒输出统计信息
        self.timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info('IMU 稳定性检查器已启动')
        self.get_logger().info('请保持机器人静止 10 秒以检查 IMU 零漂...')
        
    def raw_imu_callback(self, msg):
        self.raw_acc_buffer.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.raw_gyro_buffer.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
    
    def transformed_imu_callback(self, msg):
        self.trans_acc_buffer.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.trans_gyro_buffer.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
    
    def print_statistics(self):
        if len(self.raw_acc_buffer) < 100:
            self.get_logger().warn('等待 IMU 数据...')
            return
        
        # 计算统计信息
        raw_acc = np.array(self.raw_acc_buffer)
        raw_gyro = np.array(self.raw_gyro_buffer)
        trans_acc = np.array(self.trans_acc_buffer)
        trans_gyro = np.array(self.trans_gyro_buffer)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('原始 IMU 数据统计 (/livox/imu):')
        self.get_logger().info(f'  加速度均值: [{raw_acc.mean(axis=0)[0]:.3f}, {raw_acc.mean(axis=0)[1]:.3f}, {raw_acc.mean(axis=0)[2]:.3f}] m/s²')
        self.get_logger().info(f'  加速度标准差: [{raw_acc.std(axis=0)[0]:.4f}, {raw_acc.std(axis=0)[1]:.4f}, {raw_acc.std(axis=0)[2]:.4f}] m/s²')
        self.get_logger().info(f'  加速度模长: {np.linalg.norm(raw_acc.mean(axis=0)):.3f} m/s² (应接近 9.81)')
        self.get_logger().info(f'  角速度均值: [{raw_gyro.mean(axis=0)[0]:.4f}, {raw_gyro.mean(axis=0)[1]:.4f}, {raw_gyro.mean(axis=0)[2]:.4f}] rad/s')
        self.get_logger().info(f'  角速度标准差: [{raw_gyro.std(axis=0)[0]:.5f}, {raw_gyro.std(axis=0)[1]:.5f}, {raw_gyro.std(axis=0)[2]:.5f}] rad/s')
        
        self.get_logger().info('-' * 60)
        self.get_logger().info('转换后 IMU 数据统计 (/imu/data):')
        self.get_logger().info(f'  加速度均值: [{trans_acc.mean(axis=0)[0]:.3f}, {trans_acc.mean(axis=0)[1]:.3f}, {trans_acc.mean(axis=0)[2]:.3f}] m/s²')
        self.get_logger().info(f'  加速度标准差: [{trans_acc.std(axis=0)[0]:.4f}, {trans_acc.std(axis=0)[1]:.4f}, {trans_acc.std(axis=0)[2]:.4f}] m/s²')
        self.get_logger().info(f'  加速度模长: {np.linalg.norm(trans_acc.mean(axis=0)):.3f} m/s² (应接近 9.81)')
        self.get_logger().info(f'  角速度均值: [{trans_gyro.mean(axis=0)[0]:.4f}, {trans_gyro.mean(axis=0)[1]:.4f}, {trans_gyro.mean(axis=0)[2]:.4f}] rad/s')
        self.get_logger().info(f'  角速度标准差: [{trans_gyro.std(axis=0)[0]:.5f}, {trans_gyro.std(axis=0)[1]:.5f}, {trans_gyro.std(axis=0)[2]:.5f}] rad/s')
        
        # 诊断建议
        self.get_logger().info('-' * 60)
        self.get_logger().info('诊断建议:')
        
        # 检查角速度零漂
        gyro_mean = np.abs(trans_gyro.mean(axis=0))
        if np.any(gyro_mean > 0.01):
            self.get_logger().warn(f'  ⚠ 角速度存在较大零漂 {gyro_mean}，建议检查 IMU 校准')
        else:
            self.get_logger().info(f'  ✓ 角速度零漂正常 {gyro_mean}')
        
        # 检查加速度噪声
        acc_std = trans_acc.std(axis=0)
        if np.any(acc_std > 0.05):
            self.get_logger().warn(f'  ⚠ 加速度噪声较大 {acc_std}，建议减震或调整滤波参数')
        else:
            self.get_logger().info(f'  ✓ 加速度噪声正常 {acc_std}')
        
        # 检查重力方向
        gravity_norm = np.linalg.norm(trans_acc.mean(axis=0))
        if abs(gravity_norm - 9.81) > 0.5:
            self.get_logger().warn(f'  ⚠ 重力加速度异常 {gravity_norm:.3f}，应接近 9.81 m/s²')
        else:
            self.get_logger().info(f'  ✓ 重力加速度正常 {gravity_norm:.3f} m/s²')
        
        # 检查 Z 轴是否对齐重力
        z_acc = abs(trans_acc.mean(axis=0)[2])
        if z_acc < 8.0:
            self.get_logger().warn(f'  ⚠ Z 轴重力分量过小 {z_acc:.3f}，IMU 坐标系可能不正确')
        else:
            self.get_logger().info(f'  ✓ Z 轴重力对齐正常 {z_acc:.3f} m/s²')
        
        self.get_logger().info('=' * 60)

def main(args=None):
    rclpy.init(args=args)
    node = IMUStabilityChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
