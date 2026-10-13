#!/usr/bin/env python3
"""2D 정합(yaw+수평 이동) 뒤 높이·기울기 맞춤: 새 맵 바닥 점과 v0.3 형상 메시 바닥의 높이 차를 평면 dz = a·x + b·y + c 로 맞춰
3D 변환(T_src_to_ref.npy, 4x4)을 만든다. 맞춘 뒤 남은 높이 차를 구역별로 출력.
사용: align_height.py <src_run> <align_dir> <ref_mesh.ply> [src 카메라-바닥 높이 범위 기본 "-1.4,-0.7"]
(~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
H = Path.home() / 'b_slam/exports'
run, adir, refmesh = sys.argv[1], H / sys.argv[2], sys.argv[3]
glo, ghi = map(float, (sys.argv[4] if len(sys.argv) > 4 else '-1.25,-0.85').split(','))  # 새 맵(본네트 1.05m) 바닥
RLO, RHI = -1.85, -1.25  # v0.3(지붕 1.55m) 바닥: v0.3 카메라 궤적 기준
T2 = np.load(adir / 'T_src_to_ref_2d.npy')
T = np.eye(4); T[:2, :2] = T2[:2, :2]; T[:2, 3] = T2[:2, 2]
P = np.asarray(o3d.io.read_point_cloud(str(H / run / 'cloud_cloud.ply')).points)
tr = np.loadtxt(H / run / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]
_, j = cKDTree(tr[:, :2]).query(P[:, :2]); h = P[:, 2] - tr[j, 2]
G = P[(h > glo) & (h < ghi)]                                    # 새 맵 바닥 점 (자기 좌표계)
G = G @ T[:3, :3].T + T[:3, 3]                                  # 2D 변환 적용
V = np.asarray(o3d.io.read_triangle_mesh(refmesh).vertices)
# 기준 바닥: v0.3 메시 꼭짓점 중 v0.3 카메라 궤적(시계+반시계) 기준 바닥 높이 범위 → 0.3m 칸별 중앙값
D3 = Path(refmesh).parent
TR = np.vstack([np.loadtxt(D3 / f, comments='#', ndmin=2)[:, 1:4] for f in ('trajectory_camera_link.txt', 'trajectory_ccw_camera_link.txt') if (D3 / f).exists()])
_, jj = cKDTree(TR[:, :2]).query(V[:, :2]); hv = V[:, 2] - TR[jj, 2]; Vg = V[(hv > RLO) & (hv < RHI)]
cell = np.floor(Vg[:, :2] / 0.3).astype(np.int64); key = cell[:, 0] * 100003 + cell[:, 1]
uk, inv = np.unique(key, return_inverse=True); cnt = np.bincount(inv)
F = np.c_[np.bincount(inv, Vg[:, 0]) / cnt, np.bincount(inv, Vg[:, 1]) / cnt, np.array([np.median(Vg[inv == i, 2]) for i in range(len(uk))])] if len(uk) < 30000 else None
if F is None:
    o = np.argsort(inv); sp = np.split(Vg[o, 2], np.cumsum(cnt)[:-1]); F = np.c_[np.bincount(inv, Vg[:, 0]) / cnt, np.bincount(inv, Vg[:, 1]) / cnt, [np.median(x) for x in sp]]
tree = cKDTree(F[:, :2])
d, k = tree.query(G[:, :2]); m = d < 0.3
dz = F[k[m], 2] - G[m, 2]; X = G[m]
print(f'겹치는 바닥 점 {m.sum()}개 / 새 맵 바닥 점 {len(G)}개')
print(f'높이 차(v0.3 − 새 맵) 맞추기 전: 중앙 {np.median(dz)*100:+.1f}cm, 10~90% {np.percentile(dz,10)*100:+.1f}~{np.percentile(dz,90)*100:+.1f}cm')
# 평면 맞춤 (잘린 최소제곱)
A = np.c_[X[:, 0], X[:, 1], np.ones(len(X))]; w = np.ones(len(X), bool)
for _ in range(5):
    coef, *_ = np.linalg.lstsq(A[w], dz[w], rcond=None); r = dz - A @ coef; w = np.abs(r) < max(3 * np.median(np.abs(r[w])), 0.02)
a, b, c0 = coef
print(f'평면: dz = {a*100:+.3f}cm/m·x {b*100:+.3f}cm/m·y {c0*100:+.1f}cm  (기울기 {np.degrees(np.arctan(np.hypot(a, b))):.3f}°)')
# 기울기는 작은 회전으로: z' = z + a x + b y + c  ≈ 회전 (롤·피치) + 이동
Rt = np.eye(4); ang_x = np.arctan(b); ang_y = -np.arctan(a)
cx, sx, cy, sy = np.cos(ang_x), np.sin(ang_x), np.cos(ang_y), np.sin(ang_y)
Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]); Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
Rt[:3, :3] = Ry @ Rx; Rt[2, 3] = c0
T3 = Rt @ T
np.save(adir / 'T_src_to_ref.npy', T3)
G2 = (G - T[:3, 3]) @ np.linalg.inv(T[:3, :3]).T @ T3[:3, :3].T + T3[:3, 3]
d2, k2 = tree.query(G2[:, :2]); m2 = d2 < 0.3; dz2 = F[k2[m2], 2] - G2[m2, 2]
print(f'맞춘 뒤 남은 높이 차: 중앙 {np.median(dz2)*100:+.1f}cm, |차| 중앙 {np.median(np.abs(dz2))*100:.1f}cm, 90% {np.percentile(np.abs(dz2),90)*100:.1f}cm')
Xm = G2[m2]
for nm, (x0, x1, y0, y1) in {'북쪽 줄': (-27, -7, 5, 12), '동쪽 줄': (0, 6, -14, 6), '가운데 남쪽': (-30, -10, -22, -12), '북서 대각': (-42, -25, -9, 6)}.items():
    s = (Xm[:, 0] > x0) & (Xm[:, 0] < x1) & (Xm[:, 1] > y0) & (Xm[:, 1] < y1)
    if s.sum() > 50: print(f'  {nm}: 점 {s.sum()}, 남은 높이 차 중앙 {np.median(dz2[s])*100:+.1f}cm')
(adir / 'report.txt').open('a').write(f'높이·기울기 맞춤: dz 평면 a={a:.5f} b={b:.5f} c={c0:.4f}; 남은 |차| 중앙 {np.median(np.abs(dz2))*100:.1f}cm\n')
