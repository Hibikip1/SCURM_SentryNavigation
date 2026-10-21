#!/usr/bin/env python3
"""离线标定 ICP 判定阈值。

用真实先验地图 (test.pcd) 生成"合成扫描"（地图上 12 m 内的点 + 一定比例地图外远点），
然后在不同位姿误差下统计候选指标，用来选 min_inlier_ratio / max_rmse。

用法:
    python3 metric_study.py /home/lab/sentry_ws/test.pcd
"""
import sys

import numpy as np
from scipy.spatial import cKDTree


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


def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def T_of(dx=0.0, dy=0.0, dz=0.0, yaw=0.0):
    T = np.eye(4)
    T[:3, :3] = rot_z(yaw)
    T[:3, 3] = [dx, dy, dz]
    return T


class Study:
    def __init__(self, map_path, map_leaf=0.2, seed=7):
        self.map = load_pcd(map_path)
        self.target = voxel(self.map, map_leaf)
        self.tree = cKDTree(self.target)
        self.rng = np.random.default_rng(seed)

    def make_scan(self, pos=(0, 0, 0), yaw=0.0, n=1800, range_m=12.0, far_frac=0.10):
        """在传感器位姿 (pos, yaw) 处，从地图生成一帧 360° 扫描。"""
        R = rot_z(yaw)
        p = (self.map - np.asarray(pos)) @ R
        r = np.linalg.norm(p, axis=1)
        p = voxel(p[(r > 0.6) & (r < range_m)], 0.15)
        if len(p) > n:
            p = p[self.rng.choice(len(p), n, replace=False)]
        # 只加入"地图里确实没有对应物"的远点，模拟未建图区域 / 远距噪声
        k = int(len(p) * far_frac / (1.0 - far_frac))
        far = []
        while len(far) < k:
            c = self.rng.normal(size=(max(k * 4, 64), 3))
            c /= np.linalg.norm(c, axis=1)[:, None]
            c *= self.rng.uniform(4.0, 20.0, size=(len(c), 1))
            d, _ = self.tree.query(c)
            far += list(c[d > 3.0])
        return np.vstack([p, np.asarray(far[:k])])

    def metric(self, scan, T, inlier_dist=0.3):
        a = scan @ T[:3, :3].T + T[:3, 3]
        d, _ = self.tree.query(a)
        inl = d < inlier_dist
        ratio = 100.0 * inl.mean()
        rmse = np.sqrt((d[inl] ** 2).mean()) if inl.any() else 9.9
        return ratio, rmse, np.median(d), np.percentile(d, 90)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/home/lab/sentry_ws/test.pcd'
    s = Study(path)
    print('地图 %s: 原始 %d 点 -> 0.2 m 体素 %d 点' % (path, len(s.map), len(s.target)))

    cases = [
        ('完美位姿 +10% 远点', T_of(), 0.10),
        ('完美位姿 +25% 远点', T_of(), 0.25),
        ('平移 0.3 m', T_of(dx=0.3), 0.10),
        ('平移 0.8 m', T_of(dx=0.8), 0.10),
        ('z 偏 0.3 m', T_of(dz=0.3), 0.10),
        ('yaw 10 deg', T_of(yaw=np.radians(10)), 0.10),
        ('yaw 20 deg', T_of(yaw=np.radians(20)), 0.10),
        ('yaw 40 deg', T_of(yaw=np.radians(40)), 0.10),
    ]
    for dist in (0.2, 0.3):
        print('\n===== inlier_dist = %.2f m =====' % dist)
        print('%-22s %7s %7s %7s %7s' % ('情况', '内点%', 'rmse', '中位NN', 'p90'))
        for name, T, far in cases:
            scan = s.make_scan((0, 0, 0), 0.0, far_frac=far)
            r, q, m, p90 = s.metric(scan, T, dist)
            print('%-22s %7.1f %7.3f %7.3f %7.3f' % (name, r, q, m, p90))


if __name__ == '__main__':
    main()
