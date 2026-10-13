#!/usr/bin/env python3
"""새 맵(자기 좌표계)을 임의의 기준 맵(같은 좌표계의 바닥 점·주차선)에 강체 정합.
 수평: align_paint2d.py (주차선 전수 탐색 + ICP) → exports/<out>/T_src_to_ref_2d.npy
 높이·기울기: 2D 변환 후 새 맵 바닥과 기준 바닥의 1m 칸 높이 차를 평면으로 맞춤(잘린 최소제곱) → exports/<out>/T_src_to_ref.npy (4x4)
사용: align_to_ref.py <src_run> <src_paint_dir> <ref_paint_dir> <ref_floor.npy> <out_dir> <GLO,GHI(새 맵 카메라-바닥 띠)>   (~/tsdf_env)"""
import sys, subprocess
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
E = Path.home() / 'b_slam/exports'; S = Path.home() / 'b_slam/scripts'
run, sp, rp, rf, out, band = sys.argv[1:7]; glo, ghi = map(float, band.split(','))
subprocess.run([sys.executable, str(S / 'align_paint2d.py'), sp, rp, out], check=True)
T2 = np.load(E / out / 'T_src_to_ref_2d.npy'); T = np.eye(4); T[:2, :2] = T2[:2, :2]; T[:2, 3] = T2[:2, 2]
P = np.asarray(o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).points); t = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]
_, j = cKDTree(t[:, :2]).query(P[:, :2]); h = P[:, 2] - t[j, 2]; G = P[(h > glo) & (h < ghi)]; G = G @ T[:3, :3].T + T[:3, 3]
R = np.load(rf)
def grid(F):
    c = np.floor(F[:, :2]).astype(np.int64); key = c[:, 0] * 100003 + c[:, 1]; o = np.argsort(key); key, z, xy = key[o], F[o, 2], F[o, :2]
    u, st, cnt = np.unique(key, return_index=True, return_counts=True); k = cnt >= 20
    return dict(zip(u[k], [(xy[a:a + n].mean(0), np.median(z[a:a + n])) for a, n in zip(st[k], cnt[k])]))
gs, gr = grid(G), grid(R); ks = sorted(set(gs) & set(gr))
X = np.array([gs[k][0] for k in ks]); dz = np.array([gr[k][1] - gs[k][1] for k in ks])
A = np.c_[X, np.ones(len(X))]; w = np.ones(len(X), bool)
for _ in range(5):
    coef = np.linalg.lstsq(A[w], dz[w], rcond=None)[0]; r = dz - A @ coef; w = np.abs(r) < max(3 * np.median(np.abs(r[w])), 0.02)
a, b, c0 = coef; ax, ay = np.arctan(b), -np.arctan(a)
Rx = np.array([[1, 0, 0], [0, np.cos(ax), -np.sin(ax)], [0, np.sin(ax), np.cos(ax)]]); Ry = np.array([[np.cos(ay), 0, np.sin(ay)], [0, 1, 0], [-np.sin(ay), 0, np.cos(ay)]])
Rt = np.eye(4); Rt[:3, :3] = Ry @ Rx; Rt[2, 3] = c0; T3 = Rt @ T; np.save(E / out / 'T_src_to_ref.npy', T3)
r = dz - A @ coef
msg = f'높이 평면: 겹치는 1m 칸 {len(ks)}개, 맞추기 전 차 중앙 {np.median(dz)*100:+.1f}cm → 남은 |차| 중앙 {np.median(np.abs(r))*100:.1f}cm, 90% {np.percentile(np.abs(r),90)*100:.1f}cm (기울기 {np.degrees(np.arctan(np.hypot(a,b))):.2f}°)'
print(msg); (E / out / 'report.txt').open('a').write(msg + '\n')
