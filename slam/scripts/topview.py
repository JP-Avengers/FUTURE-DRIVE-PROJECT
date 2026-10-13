#!/usr/bin/env python3
"""탑뷰 PNG: 포인트 클라우드(밝기=높이) + 오도메트리 궤적(빨강) + 최적화 궤적(초록) + 출발점(파랑)/도착점(노랑).
사용: topview.py <run> <out.png> [px_per_m=10]"""
import sys
from pathlib import Path
import numpy as np, open3d as o3d
run, out = sys.argv[1], sys.argv[2]; s = float(sys.argv[3]) if len(sys.argv) > 3 else 10
E = Path.home() / f'b_slam/exports/{run}'
P = np.asarray(o3d.io.read_point_cloud(str(E / 'cloud_cloud.ply')).points)
T = {k: np.loadtxt(E / f'traj_{k}_poses.txt', comments='#', ndmin=2)[:, 1:4] for k in ('odom', 'optimized')}
allxy = np.vstack([P[:, :2], T['odom'][:, :2], T['optimized'][:, :2]])
lo = allxy.min(0) - 2; hi = allxy.max(0) + 2
W, H = (np.ceil((hi - lo) * s)).astype(int) + 1
img = np.zeros((H, W, 3), np.uint8) + 255
px = lambda xy: (((xy[:, 0] - lo[0]) * s).astype(int), (H - 1 - (xy[:, 1] - lo[1]) * s).astype(int))
z = P[:, 2]; zn = np.clip((z - np.percentile(z, 2)) / (np.percentile(z, 98) - np.percentile(z, 2) + 1e-9), 0, 1)
u, v = px(P[:, :2]); g = (200 - 170 * zn).astype(np.uint8)
order = np.argsort(z); img[v[order], u[order]] = np.stack([g[order]] * 3, 1)
def line(xy, c, r=1):
    for a, b in zip(xy[:-1], xy[1:]):
        n = int(np.linalg.norm(b - a) * s * 2) + 2
        for t in np.linspace(0, 1, n):
            p = a + (b - a) * t; uu, vv = int((p[0] - lo[0]) * s), int(H - 1 - (p[1] - lo[1]) * s)
            img[max(vv - r, 0):vv + r + 1, max(uu - r, 0):uu + r + 1] = c
line(T['odom'][:, :2], (220, 40, 40)); line(T['optimized'][:, :2], (20, 160, 40), 2)
for p, c in ((T['optimized'][0], (30, 60, 230)), (T['optimized'][-1], (240, 190, 0))):
    uu, vv = int((p[0] - lo[0]) * s), int(H - 1 - (p[1] - lo[1]) * s); img[vv - 5:vv + 6, uu - 5:uu + 6] = c
# 10m 눈금자
img[H - 12:H - 9, 10:10 + int(10 * s)] = (0, 0, 0)
o3d.io.write_image(out, o3d.geometry.Image(np.ascontiguousarray(img)))
print(f'{out}: {W}x{H}px, 1px={1/s:.2f}m, 하단 검은 막대=10m')
