#!/usr/bin/env python3
"""v0.3 바닥 구멍(floor_holes.csv 번호)을 새 맵 바닥 점이 얼마나 덮는지.
구멍 영역은 holes_pkg.py와 같은 방식(v0.3 형상 메시, 궤적 0~6m 띠, 10cm 칸)으로 다시 만들고, csv 중심이 속한(없으면 가장 가까운) 빈칸 덩어리를 그 번호로 본다.
새 맵 바닥: exports/<run>/cloud_cloud.ply 중 자기 궤적 기준 높이 GLO~GHI → align_<run>/T_src_to_ref.npy 로 v0.3 좌표계
사용: hole_cover.py <run>:<GLO,GHI> [...] [--holes 2,3,13]   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
from scipy import ndimage
D = Path.home() / 'b_slam/deliver/map_v0.3'; E = Path.home() / 'b_slam/exports'; res = 0.10
args = [a for a in sys.argv[1:] if not a.startswith('--')]
hsel = [int(x) for x in (sys.argv[sys.argv.index('--holes') + 1] if '--holes' in sys.argv else '2,3,13').split(',')]
args = [a for a in args if not a.replace(',', '').isdigit()]
V = np.asarray(o3d.io.read_triangle_mesh(str(D / 'geometry_rawdepth_tsdf_5cm.ply')).vertices)
T = np.loadtxt(D / 'trajectory_camera_link.txt', ndmin=2); T = T[np.argsort(T[:, 0])][:, 1:4]
s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(T[:, :2], axis=0), axis=1))]; ss = np.arange(0, s[-1], 0.1)
Ti = np.c_[[np.interp(ss, s, T[:, k]) for k in range(3)]].T; tree = cKDTree(Ti[:, :2])
d, j = tree.query(V[:, :2]); dz = Ti[j, 2] - V[:, 2]; G = V[(dz > 1.0) & (dz < 2.2) & (d < 6.5)]
lo = Ti[:, :2].min(0) - 8; hi = Ti[:, :2].max(0) + 8; W, H = np.ceil((hi - lo) / res).astype(int)
def cells(P):
    ij = np.floor((P[:, :2] - lo) / res).astype(int); k = (ij[:, 0] >= 0) & (ij[:, 0] < W) & (ij[:, 1] >= 0) & (ij[:, 1] < H)
    m = np.zeros((H, W), bool); m[H - 1 - ij[k, 1], ij[k, 0]] = True; return m
occ = cells(G)
gy, gx = np.mgrid[0:H, 0:W]; X = lo[0] + (gx + .5) * res; Y = lo[1] + (H - 1 - gy + .5) * res
dc = tree.query(np.c_[X.ravel(), Y.ravel()])[0].reshape(H, W)
holes = np.zeros((H, W), int)
for a, b in ((0, 3), (3, 6)):
    lab, n = ndimage.label((dc >= a) & (dc < b) & ~occ); sz = np.bincount(lab.ravel())
    holes =np.where((lab > 0) & (sz[lab] * res * res >= 1.0), lab + (100000 if a else 0), holes)
csv = {int(r[0]): (float(r[2]), float(r[3]), float(r[4])) for r in (l.split(',') for l in (D / 'floor_holes.csv').read_text().splitlines()[1:])}
ids = np.unique(holes[holes > 0]); cen = np.array([[X[holes == i].mean(), Y[holes == i].mean()] for i in ids]); area = np.array([(holes == i).sum() * res * res for i in ids])
masks = {}
for h in hsel:
    x, y, a = csv[h]; k = np.argmin(np.hypot(cen[:, 0] - x, cen[:, 1] - y) + 0.5 * np.abs(area - a))
    masks[h] = holes == ids[k]
    print(f'구멍 {h}: csv ({x:+.1f},{y:+.1f}) {a}㎡ ↔ 재구성 ({cen[k,0]:+.1f},{cen[k,1]:+.1f}) {area[k]:.1f}㎡')
tot = np.zeros((H, W), bool)
for spec in args:
    run, rng_ = spec.split(':'); glo, ghi = map(float, rng_.split(','))
    P = np.asarray(o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).points)
    tr = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]
    _, j = cKDTree(tr[:, :2]).query(P[:, :2]); h = P[:, 2] - tr[j, 2]; F = P[(h > glo) & (h < ghi)]
    Tm = np.load(E / f'align_{run}/T_src_to_ref.npy'); F = F @ Tm[:3, :3].T + Tm[:3, 3]
    c = ndimage.binary_dilation(cells(F)); tot |= c   # 10cm 칸, 1칸 팽창(점 간격 3cm 복셀 → 빈틈 메움)
    print(f'{run}: 바닥 점 {len(F):,} | ' + ', '.join(f'{h}번 {(c & m).sum() / m.sum() * 100:.0f}%' for h, m in masks.items()))
if len(args) > 1: print('합계: ' + ', '.join(f'{h}번 {(tot & m).sum() / m.sum() * 100:.0f}%' for h, m in masks.items()))
