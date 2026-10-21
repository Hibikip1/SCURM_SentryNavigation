#!/usr/bin/env python3
"""从先验地图合成一帧 Mid360 扫描并发布到 /livox/lidar，用于离线验证 icp_node。

用法:
  python3 fake_scan_publisher.py --map /home/lab/sentry_ws/test.pcd \
      --x 0 --y 0 --z 0 --yaw 0 --far-ratio 0.10 --rate 10
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg, CustomPoint


def load_pcd(path):
    raw = open(path, 'rb').read()
    i = raw.find(b'DATA binary')
    assert i > 0, 'only binary pcd supported'
    start = i + len(b'DATA binary') + 1
    return np.frombuffer(raw[start:], dtype=np.float32).reshape(-1, 8)[:, :3].astype(np.float64)


def voxel(points, leaf):
    key = np.floor(points / leaf).astype(np.int64)
    uni, inv = np.unique(key, axis=0, return_inverse=True)
    out = np.zeros((len(uni), 3))
    np.add.at(out, inv, points)
    return out / np.bincount(inv)[:, None]


class FakeScan(Node):
    def __init__(self, args):
        super().__init__('fake_scan_publisher')
        self.pub = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        rng = np.random.default_rng(7)
        world = load_pcd(args.map)
        c, s = np.cos(args.yaw), np.sin(args.yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        p = (world - np.array([args.x, args.y, args.z])) @ R
        r = np.linalg.norm(p, axis=1)
        p = voxel(p[(r > 0.6) & (r < args.range)], 0.15)
        if len(p) > args.points:
            p = p[rng.choice(len(p), args.points, replace=False)]

        # 地图里没有对应物的远点：模拟未建图区域 / 远距噪声（这正是 fitness 被拉爆的来源）
        if args.far_ratio > 0:
            from scipy.spatial import cKDTree
            tree = cKDTree(voxel(world, 0.2))
            k = int(len(p) * args.far_ratio / (1.0 - args.far_ratio))
            far = []
            while len(far) < k:
                v = rng.normal(size=(max(k * 4, 64), 3))
                v /= np.linalg.norm(v, axis=1)[:, None]
                v *= rng.uniform(4.0, 20.0, size=(len(v), 1))
                d, _ = tree.query(v)
                far += list(v[d > 3.0])
            p = np.vstack([p, np.asarray(far[:k])])

        self.points = p
        self.get_logger().info(
            'scan %d pts, gt pose x=%.2f y=%.2f z=%.2f yaw=%.1f deg, far_ratio=%.2f'
            % (len(p), args.x, args.y, args.z, np.degrees(args.yaw), args.far_ratio))
        self.timer = self.create_timer(1.0 / args.rate, self.tick)
        self.count = 0

    def tick(self):
        msg = CustomMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'livox_frame'
        msg.timebase = 0
        msg.point_num = len(self.points)
        msg.lidar_id = 0
        pts = []
        for x, y, z in self.points:
            cp = CustomPoint()
            cp.offset_time = 0
            cp.x, cp.y, cp.z = float(x), float(y), float(z)
            cp.reflectivity = 100
            cp.tag = 0x10
            cp.line = 0
            pts.append(cp)
        msg.points = pts
        self.pub.publish(msg)
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info('published %d frames' % self.count)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='/home/lab/sentry_ws/test.pcd')
    ap.add_argument('--x', type=float, default=0.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--z', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0, help='rad')
    ap.add_argument('--points', type=int, default=1800)
    ap.add_argument('--range', type=float, default=12.0)
    ap.add_argument('--far-ratio', type=float, default=0.10)
    ap.add_argument('--rate', type=float, default=10.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = FakeScan(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
