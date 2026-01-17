#!/usr/bin/env python3
"""
计算 Point-LIO 外参旋转矩阵
根据实际安装角度计算正确的 LiDAR -> IMU 旋转矩阵
"""

import numpy as np
import math

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    根据 ZYX 欧拉角（Yaw-Pitch-Roll）计算旋转矩阵
    
    Args:
        roll: 绕 X 轴旋转 (rad)
        pitch: 绕 Y 轴旋转 (rad)
        yaw: 绕 Z 轴旋转 (rad)
    
    Returns:
        3x3 旋转矩阵
    """
    # Roll (X 轴)
    R_x = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])
    
    # Pitch (Y 轴)
    R_y = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])
    
    # Yaw (Z 轴)
    R_z = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # 组合旋转：R = R_z * R_y * R_x
    R = R_z @ R_y @ R_x
    return R

def rotation_matrix_to_euler(R):
    """
    从旋转矩阵计算 ZYX 欧拉角
    """
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        roll = math.atan2(R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw = math.atan2(R[1,0], R[0,0])
    else:
        roll = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw = 0
    
    return roll, pitch, yaw

# 用户提供的信息
print("=" * 70)
print("Point-LIO 外参旋转矩阵计算")
print("=" * 70)
print()

# 场景 1：实际物理安装角度
physical_roll = -48.0  # 度
physical_pitch = 1.0   # 度
physical_yaw = 0.0     # 度

print(f"📐 实际物理安装角度:")
print(f"   Roll:  {physical_roll:+.1f}°")
print(f"   Pitch: {physical_pitch:+.1f}°")
print(f"   Yaw:   {physical_yaw:+.1f}°")
print()

# 场景 2：建图所需角度（翻转 180°）
mapping_roll = 132.0   # -48° + 180°
mapping_pitch = 1.0    # 度
mapping_yaw = 0.0      # 度

print(f"🗺️  建图所需角度（点云向下投影）:")
print(f"   Roll:  {mapping_roll:+.1f}°")
print(f"   Pitch: {mapping_pitch:+.1f}°")
print(f"   Yaw:   {mapping_yaw:+.1f}°")
print()

# 转换为弧度
roll_rad = math.radians(mapping_roll)
pitch_rad = math.radians(mapping_pitch)
yaw_rad = math.radians(mapping_yaw)

# 计算旋转矩阵
R = euler_to_rotation_matrix(roll_rad, pitch_rad, yaw_rad)

print("🔧 计算得到的旋转矩阵 (LiDAR -> IMU):")
print()
print("   extrinsic_R: [")
for i in range(3):
    values = ", ".join([f"{R[i,j]:+.16f}" for j in range(3)])
    if i < 2:
        print(f"       {values},")
    else:
        print(f"       {values}")
print("   ]")
print()

# 扁平化为 Point-LIO 格式（按行展开）
flat_R = R.flatten().tolist()
print("📋 Point-LIO YAML 格式（复制到配置文件）:")
print()
print("   extrinsic_R: [", end="")
for i in range(9):
    if i == 0:
        print(f" {flat_R[i]:.16f},", end="")
    elif i == 8:
        print(f" {flat_R[i]:.16f}", end="")
    elif i % 3 == 0:
        print()
        print(f"                 {flat_R[i]:.16f},", end="")
    else:
        print(f" {flat_R[i]:.16f},", end="")
print(" ]")
print()

# 验证：测试重力方向变换
print("=" * 70)
print("验证：重力方向变换")
print("=" * 70)
print()

# LiDAR 坐标系下的向上方向（假设 Z 轴向上，但由于倾斜，重力分量在多个轴上）
# 实际测量的原始 IMU 加速度（静止时）
raw_acc_lidar = np.array([0.007, -7.256, 6.493])  # m/s²
print(f"📊 原始 IMU 加速度 (LiDAR 坐标系):")
print(f"   [{raw_acc_lidar[0]:+.3f}, {raw_acc_lidar[1]:+.3f}, {raw_acc_lidar[2]:+.3f}] m/s²")
print(f"   模长: {np.linalg.norm(raw_acc_lidar):.3f} m/s²")
print()

# 应用旋转矩阵变换到水平坐标系
acc_horizontal = R @ raw_acc_lidar
print(f"🔄 变换后加速度 (水平坐标系):")
print(f"   [{acc_horizontal[0]:+.3f}, {acc_horizontal[1]:+.3f}, {acc_horizontal[2]:+.3f}] m/s²")
print(f"   模长: {np.linalg.norm(acc_horizontal):.3f} m/s²")
print()

# 理想情况：Z 轴应该接近 -9.81 m/s² (向下)
print(f"💡 期望结果:")
print(f"   X, Y 应接近 0")
print(f"   Z 应接近 -9.81 m/s² (重力向下)")
print()

if abs(acc_horizontal[0]) < 0.5 and abs(acc_horizontal[1]) < 0.5:
    print(f"   ✅ X, Y 分量正常 ({acc_horizontal[0]:.3f}, {acc_horizontal[1]:.3f})")
else:
    print(f"   ⚠️  X, Y 分量较大，可能需要微调 pitch/yaw")

if -10.5 < acc_horizontal[2] < -9.0:
    print(f"   ✅ Z 分量正常 ({acc_horizontal[2]:.3f})")
else:
    print(f"   ⚠️  Z 分量异常 ({acc_horizontal[2]:.3f}，应接近 -9.81)")

print()
print("=" * 70)

# 当前配置的旋转矩阵（用于对比）
print()
print("🔍 当前配置的旋转矩阵对比:")
print()
current_R = np.array([
    [0.9998476951563913, -0.01297244663297738, 0.01167644684282871],
    [0.0, -0.6691306063588582, -0.7431448254773942],
    [0.01745240643728351, 0.743000432798103, -0.6690083827513697]
])

# 计算当前配置的欧拉角
curr_roll, curr_pitch, curr_yaw = rotation_matrix_to_euler(current_R)
print(f"   当前配置对应的角度:")
print(f"   Roll:  {math.degrees(curr_roll):+.2f}°")
print(f"   Pitch: {math.degrees(curr_pitch):+.2f}°")
print(f"   Yaw:   {math.degrees(curr_yaw):+.2f}°")
print()

# 用当前矩阵变换
acc_current = current_R @ raw_acc_lidar
print(f"   当前配置变换结果:")
print(f"   [{acc_current[0]:+.3f}, {acc_current[1]:+.3f}, {acc_current[2]:+.3f}] m/s²")
print()

print("✨ 建议使用新计算的旋转矩阵替换当前配置")
print("=" * 70)
