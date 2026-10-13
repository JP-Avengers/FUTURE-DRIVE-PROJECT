#!/usr/bin/env python3
"""기준 맵 검증 2: (a) 평면 기울기를 뺀 높이 차, (b) 여러 주차 줄의 칸 간격(맵별, 강체만 = 내부 모양 그대로)."""
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
E = Path.home() / 'b_slam/exports'; V3 = Path.home() / 'b_slam/deliver/map_v0.3'; CELL = 1.0
def floor_pts(run, band):
    P = np.asarray(o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).points); t = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]
    _, j = cKDTree(t[:, :2]).query(P[:, :2]); h = P[:, 2] - t[j, 2]; return P[(h > band[0]) & (h < band[1])]
def grid(F):
    c = np.floor(F[:, :2] / CELL).astype(np.int64); key = c[:, 0] * 100003 + c[:, 1]; o = np.argsort(key); key, z = key[o], F[o, 2]
    u, st, cnt = np.unique(key, return_index=True, return_counts=True); med = np.array([np.median(z[a:a + n]) for a, n in zip(st, cnt)]); k = cnt >= 30
    return dict(zip(u[k], med[k]))
def tf(T, X): return X @ T[:3, :3].T + T[:3, 3]
V = np.asarray(o3d.io.read_triangle_mesh(str(V3 / 'geometry_rawdepth_tsdf_5cm.ply')).vertices)
TR = np.vstack([np.loadtxt(V3 / f, comments='#', ndmin=2)[:, 1:4] for f in ('trajectory_camera_link.txt', 'trajectory_ccw_camera_link.txt')])
_, j = cKDTree(TR[:, :2]).query(V[:, :2]); h = V[:, 2] - TR[j, 2]; g = {'v0.3': grid(V[(h > -1.85) & (h < -1.25)])}
g['gn03'] = grid(tf(np.load(E / 'align_gn03/T_src_to_ref.npy'), floor_pts('gn03', (-1.25, -0.85))))
g['gm03'] = grid(tf(np.load(E / 'align_gm03/T_src_to_ref.npy'), floor_pts('gm03', (-1.25, -0.85))))
print('[a] 1m 칸 높이 차에서 평면(기울기+높이)을 뺀 나머지 — 국소 출렁임')
for x, y in (('gn03', 'v0.3'), ('gm03', 'v0.3'), ('gn03', 'gm03')):
    ks = np.array(sorted(set(g[x]) & set(g[y]))); d = np.array([g[x][k] - g[y][k] for k in ks])
    ij = np.c_[ks // 100003, ks % 100003].astype(float); ij[ij[:, 1] > 50000, 1] -= 100003
    A = np.c_[ij, np.ones(len(ij))]; r = d - A @ np.linalg.lstsq(A, d, rcond=None)[0]
    print(f'    {x} − {y}: 칸 {len(ks)}개, 중앙절대 {np.median(np.abs(r))*100:4.1f}cm, 10~90% 폭 {(np.percentile(r,90)-np.percentile(r,10))*100:5.1f}cm, |차|>10cm 칸 {np.mean(np.abs(r)>0.10)*100:3.0f}%')
# (b) 칸 간격
def spacing(Q, box):
    x0, x1, y0, y1 = box; Q = Q[(Q[:, 0] > x0) & (Q[:, 0] < x1) & (Q[:, 1] > y0) & (Q[:, 1] < y1)]
    if len(Q) < 2000: return None
    best = None
    for a in np.radians(np.arange(0, 180, 0.5)):
        n = np.array([-np.sin(a), np.cos(a)]); hgm, _ = np.histogram(Q @ n, np.arange((Q @ n).min(), (Q @ n).max() + 0.04, 0.04))
        sc = (hgm.astype(float) ** 2).sum()
        if best is None or sc > best[0]: best = (sc, a)
    a = best[1]; u = Q @ np.array([np.cos(a), np.sin(a)])                       # 긴 선 방향으로 투영 → 칸막이 봉우리
    hgm, e = np.histogram(u, np.arange(u.min(), u.max() + 0.05, 0.05)); hs = gaussian_filter1d(hgm.astype(float), 1.5)
    pk, _ = find_peaks(hs, distance=30, prominence=hs.max() * 0.15); d = np.diff(e[pk]); d = d[(d > 1.8) & (d < 3.2)]
    return (np.median(d), len(d)) if len(d) >= 2 else None
P = {'v0.3 시계(지붕)': np.load(E / 'v03cw_paintkf/paint_xy.npy'), '10/9 시계(본네트)': np.load(E / 'gn03_v03frame/paint_xy.npy'), '10/9 반시계(본네트)': np.load(E / 'gm03_v03frame/paint_xy.npy')}
ROWS = {'줄A': (-1, 6, -13, 10), '북쪽 줄': (-34, -17, 3, 14), '북동 줄': (-17, -3, 3, 14), '남서 줄': (-45, -26, -23, -15), '남쪽 줄': (-26, -8, -26, -16)}
print('[b] 주차 줄별 칸 간격 중앙값 (실측 2.50m) — 강체만 적용(맵 내부 모양 그대로)')
for rn, box in ROWS.items():
    s = []
    for nm, Q in P.items():
        r = spacing(Q, box); s.append(f'{nm} {r[0]:.2f}m({r[1]})' if r else f'{nm}   -   ')
    print(f'    {rn:5s}: ' + ' | '.join(s))
