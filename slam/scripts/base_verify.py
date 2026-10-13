#!/usr/bin/env python3
"""기준 맵 선택 검증: v0.3(지붕) vs 본네트(10/9 gn03 시계, gm03 반시계).
 1) 높이: 1m 칸 바닥 높이(중앙값). 본네트 맵은 강체 정합(T_src_to_ref, 수평+바닥 평면)만 적용 → 내부 모양 그대로.
    gn03−v0.3, gm03−v0.3, gn03−gm03 의 퍼짐과, (gn03−v0.3)·(gm03−v0.3)의 상관.
 2) 내부 일관성: 각 맵 자기 좌표계에서 키프레임마다 '카메라 높이 − 바로 아래(수평 1m 안) 바닥 높이'. 전체·세션별 퍼짐.
 3) C 지적 구역(v0.3 키프레임 361~452 주변 6m): 칸별 높이 차.
출력: 숫자 + 높이 차 지도 그림.   (~/tsdf_env)"""
import sqlite3
from pathlib import Path
import numpy as np, open3d as o3d, cv2
from scipy.spatial import cKDTree
E = Path.home() / 'b_slam/exports'; V3 = Path.home() / 'b_slam/deliver/map_v0.3'; M = Path.home() / 'maps'
SP = '/home/owner7/b_slam/exports/tmp'
CELL = 1.0

def floor_pts(run, band):
    """자기 좌표계 점군에서 바닥 띠(가장 가까운 궤적 기준 높이) 점."""
    P = np.asarray(o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).points)
    t = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]
    _, j = cKDTree(t[:, :2]).query(P[:, :2]); h = P[:, 2] - t[j, 2]
    return P[(h > band[0]) & (h < band[1])]
def grid(F):
    c = np.floor(F[:, :2] / CELL).astype(np.int64); key = c[:, 0] * 100003 + c[:, 1]
    o = np.argsort(key); key, z = key[o], F[o, 2]; u, st, cnt = np.unique(key, return_index=True, return_counts=True)
    med = np.array([np.median(z[a:a + n]) for a, n in zip(st, cnt)])
    k = cnt >= 30; return dict(zip(u[k], med[k]))
def tf(T, X): return X @ T[:3, :3].T + T[:3, 3]

# --- 1) 높이 비교 (v0.3 좌표계) ---
V = np.asarray(o3d.io.read_triangle_mesh(str(V3 / 'geometry_rawdepth_tsdf_5cm.ply')).vertices)
TR = np.vstack([np.loadtxt(V3 / f, comments='#', ndmin=2)[:, 1:4] for f in ('trajectory_camera_link.txt', 'trajectory_ccw_camera_link.txt')])
_, j = cKDTree(TR[:, :2]).query(V[:, :2]); h = V[:, 2] - TR[j, 2]; g03 = grid(V[(h > -1.85) & (h < -1.25)])
gn = grid(tf(np.load(E / 'align_gn03/T_src_to_ref.npy'), floor_pts('gn03', (-1.25, -0.85))))
gm = grid(tf(np.load(E / 'align_gm03/T_src_to_ref.npy'), floor_pts('gm03', (-1.25, -0.85))))
common = sorted(set(g03) & set(gn) & set(gm))
a = np.array([gn[k] - g03[k] for k in common]); b = np.array([gm[k] - g03[k] for k in common]); c = np.array([gn[k] - gm[k] for k in common])
def spread(x): x = x - np.median(x); return f'중앙절대 {np.median(np.abs(x))*100:4.1f}cm, 10~90% 폭 {(np.percentile(x,90)-np.percentile(x,10))*100:5.1f}cm'
print(f'[1] 세 맵 모두 바닥이 있는 1m 칸 {len(common)}개 (강체만, 전체 평균 높이 차는 빼고 비교)')
print(f'    10/9 시계(gn03) − v0.3  : {spread(a)}')
print(f'    10/9 반시계(gm03) − v0.3: {spread(b)}')
print(f'    gn03 − gm03 (본네트끼리): {spread(c)}')
print(f'    (gn03−v0.3)와 (gm03−v0.3)의 상관: {np.corrcoef(a - a.mean(), b - b.mean())[0,1]:+.2f}  → 높으면 두 본네트 맵이 같은 곳에서 같은 방향으로 v0.3과 다름')

# 높이 차 지도
keys = np.array(common); ij = np.c_[keys // 100003, keys % 100003]; ij[ij[:, 1] > 50000, 1] -= 100003
lo = ij.min(0); sz = ij.max(0) - lo + 1; s = 8
def heat(vals, title):
    im = np.full((sz[1] * s, sz[0] * s, 3), 255, np.uint8)
    for (x, y), v in zip(ij - lo, vals):
        col = cv2.applyColorMap(np.uint8([[np.clip((v + 0.2) / 0.4, 0, 1) * 255]]), cv2.COLORMAP_COOL)[0, 0]
        y2 = sz[1] - 1 - y; im[y2 * s:(y2 + 1) * s, x * s:(x + 1) * s] = col
    cv2.putText(im, title, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 0, 0), 1); return im
am, bm, cm = a - np.median(a), b - np.median(b), c - np.median(c)
img = np.hstack([heat(am, 'gn03 - v0.3 (-20..+20cm)'), np.zeros((sz[1] * s, 4, 3), np.uint8), heat(bm, 'gm03 - v0.3'), np.zeros((sz[1] * s, 4, 3), np.uint8), heat(cm, 'gn03 - gm03')])
cv2.imwrite(f'{SP}/height_diff_maps.png', img)

# --- 3) C 지적 구역 ---
K = np.loadtxt(V3 / 'trajectory_camera_optical.txt', comments='#', ndmin=2); sel = K[(K[:, 8] >= 361) & (K[:, 8] <= 452)][:, 1:3]
cc = np.c_[(ij[:, 0] + 0.5) * CELL, (ij[:, 1] + 0.5) * CELL]; inC = cKDTree(sel).query(cc)[0] < 6
print(f'[3] C 지적 구역(v0.3 키프레임 361~452 주변 6m) 칸 {inC.sum()}개: '
      f'gn03−v0.3 중앙 {np.median(am[inC])*100:+.1f}cm | gm03−v0.3 {np.median(bm[inC])*100:+.1f}cm | gn03−gm03 {np.median(cm[inC])*100:+.1f}cm '
      f'(나머지 구역: {np.median(am[~inC])*100:+.1f} / {np.median(bm[~inC])*100:+.1f} / {np.median(cm[~inC])*100:+.1f}cm)')

# --- 2) 내부 일관성: 카메라 높이 − 바로 아래 바닥 ---
print('[2] 맵 내부 일관성: 키프레임마다 (카메라 높이 − 수평 1m 안 바닥 높이). 차가 노면 위를 달리므로 일정해야 정상')
for run, band, db in (('run18', (-1.95, -1.15), 'run18'), ('run20', (-1.95, -1.15), 'run20'), ('gn03', (-1.25, -0.85), 'gn03'), ('gm03', (-1.25, -0.85), 'gm03'), ('g11', (-1.13, -0.73), 'g11'), ('g14', (-1.13, -0.73), 'g14')):
    try:
        F = floor_pts(run, band); tf_ = cKDTree(F[:, :2])
        A = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
        mid = dict(sqlite3.connect(M / f'{db}.db').execute('select id, map_id from Node').fetchall())
        hs, ss = [], []
        for r in A:
            i = tf_.query_ball_point(r[1:3], 1.0)
            if len(i) >= 30: hs.append(r[3] - np.median(F[i, 2])); ss.append(mid.get(int(r[8]), -1))
        hs, ss = np.array(hs), np.array(ss)
        per = ' '.join(f's{m}:{np.median(hs[ss == m]):.3f}' for m in sorted(set(ss)) if (ss == m).sum() >= 10)
        print(f'    {run:6s}: 키프레임 {len(hs)}/{len(A)} | 중앙 {np.median(hs):.3f}m, 10~90% 폭 {(np.percentile(hs,90)-np.percentile(hs,10))*100:4.1f}cm, 중앙절대편차 {np.median(np.abs(hs-np.median(hs)))*100:.1f}cm | 세션별 중앙 {per}')
    except Exception as e:
        print(f'    {run}: 실패 {e}')
