#!/usr/bin/env python3
"""연석 근접 촬영 확인: 궤적에서 수평 r m 이내, 바닥(카메라-1.51m) 위 0.12~0.5m 높이 점 개수와 최근접 거리.
사용: curb_check.py <run> [r=3]"""
import sys
from pathlib import Path
import numpy as np, open3d as o3d
run = sys.argv[1]; r = float(sys.argv[2]) if len(sys.argv) > 2 else 3
E = Path.home() / f'b_slam/exports/{run}'
P = np.asarray(o3d.io.read_point_cloud(str(E / 'cloud_cloud.ply')).points)
T = np.loadtxt(E / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]
idx, dist = [], []
for j in range(0, len(P), 20000):
    d2 = ((P[j:j+20000, None, :2] - T[None, :, :2]) ** 2).sum(-1); k = d2.argmin(1)
    idx.append(k); dist.append(np.sqrt(d2[np.arange(len(k)), k]))
idx = np.concatenate(idx); dist = np.concatenate(dist)
h = 1.51 - (T[idx, 2] - P[:, 2])  # 바닥 기준 높이
near = dist < r
curb = near & (h > 0.12) & (h < 0.5)
print(f'{run}: 궤적 {r}m 이내 점 {near.sum()}, 그중 연석 높이대(0.12~0.5m) {curb.sum()} ({100*curb.sum()/max(near.sum(),1):.1f}%), '
      f'연석 높이대 점까지 거리 중앙값 {np.median(dist[curb]) if curb.any() else float("nan"):.2f} m, 궤적 길이 {np.linalg.norm(np.diff(T,axis=0),axis=1).sum():.0f} m')
