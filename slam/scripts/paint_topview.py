#!/usr/bin/env python3
"""바닥 흰색 페인트(주차선) 점만 추출해 탑뷰 PNG + npy 저장. 바닥 = 가장 가까운 궤적점보다 1.0~2.2m 아래.
사용: paint_topview.py <run> [px_per_m=20] [white_thr=120]"""
import sys
from pathlib import Path
import numpy as np, open3d as o3d

run = sys.argv[1]; s = float(sys.argv[2]) if len(sys.argv) > 2 else 20; thr = int(sys.argv[3]) if len(sys.argv) > 3 else 120
E = Path.home() / f'b_slam/exports/{run}'
pc = o3d.io.read_point_cloud(str(E / 'cloud_cloud.ply'))
P = np.asarray(pc.points); C = (np.asarray(pc.colors) * 255).astype(int)
T = np.loadtxt(E / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]
i = np.concatenate([np.argmin(((P[j:j+20000, None, :2] - T[None, :, :2]) ** 2).sum(-1), 1) for j in range(0, len(P), 20000)]); dz = T[i, 2] - P[:, 2]
ground = (dz > 1.0) & (dz < 2.2)
white = (C.mean(1) > thr) & (C.max(1) - C.min(1) < 40)
W = P[ground & white]
print(f'points {len(P)}, ground {ground.sum()}, white-ground {len(W)}, 카메라-바닥 높이 중앙값 {np.median(dz[ground]):.2f} m')
np.save(E / 'paint_xy.npy', W[:, :2])
lo = W[:, :2].min(0) - 1; hi = W[:, :2].max(0) + 1
w, h = np.ceil((hi - lo) * s).astype(int) + 1
img = np.full((h, w, 3), 255, np.uint8)
G = P[ground]; gu = ((G[:, 0] - lo[0]) * s).astype(int); gv = (h - 1 - (G[:, 1] - lo[1]) * s).astype(int)
k = (gu >= 0) & (gu < w) & (gv >= 0) & (gv < h); img[gv[k], gu[k]] = (225, 225, 225)
u = ((W[:, 0] - lo[0]) * s).astype(int); v = (h - 1 - (W[:, 1] - lo[1]) * s).astype(int)
img[v, u] = (0, 0, 0)
for m in range(0, int((hi - lo)[0]) + 1, 5):  # 5m 격자
    img[:, min(int(m * s), w - 1)] = (255, 160, 160)
for m in range(0, int((hi - lo)[1]) + 1, 5):
    img[max(h - 1 - int(m * s), 0), :] = (255, 160, 160)
o3d.io.write_image(str(E / 'paint_topview.png'), o3d.geometry.Image(np.ascontiguousarray(img)))
print(f'{E}/paint_topview.png {w}x{h}px, 1px={100/s:.0f}cm, 빨간 격자 5m, 원점(좌하단)=({lo[0]:.1f},{lo[1]:.1f})')
