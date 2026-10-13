#!/usr/bin/env python3
"""키프레임별 흰 주차선 픽셀 → 3D(월드) + 관측 거리. 거리 구간별 paint_xy.npy를 만들어 stall_measure.py로 비교.
흰색 = 그 프레임 바닥 영역 밝기 중앙값 + 60 이상, 채도 낮음. 출력: exports/<run>_paint_<lo>-<hi>m/paint_xy.npy
사용: keyframe_paint.py <run> [구간 경계들 m, 기본 5 7 9 12]
환경변수 GROUND="-1.9,-1.2" (카메라 기준 바닥 높이 범위, 지붕 카메라 기본값; 본네트 카메라는 "-1.4,-0.7"), DMIN=3 (최소 depth m)"""
import sys, re
from pathlib import Path
import numpy as np, cv2, os
GLO, GHI = map(float, os.environ.get("GROUND", "-1.9,-1.2").split(",")); DMIN = float(os.environ.get("DMIN", "3"))
run = sys.argv[1]; edges = [float(x) for x in sys.argv[2:]] or [5, 7, 9, 12]
E = Path.home() / f'b_slam/exports/{run}'; K = E / 'keyframes'
def qR(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
tr = np.loadtxt(E / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
rows = []
for r in tr:
    i = int(r[8]); T = np.eye(4); T[:3, :3] = qR(*r[4:8]); T[:3, 3] = r[1:4]
    c = cv2.imread(str(K / f'kf_rgb/{i}.jpg')); d = cv2.imread(str(K / f'kf_depth/{i}.png'), cv2.IMREAD_UNCHANGED).astype(float) / 1000
    t = (K / f'kf_calib/{i}.yaml').read_text()
    a = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', t, re.S).group(1).replace('\n', ' ').split(',')]
    hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV); V = hsv[..., 2].astype(float); S = hsv[..., 1]
    valid = (d > DMIN) & (d < 15); valid[:240] = False  # 화면 아래 절반(바닥)만
    if valid.sum() < 1000: continue
    white = valid & (V > np.median(V[valid]) + 60) & (S < 50)
    v, u = np.nonzero(white); z = d[v, u]
    P = np.c_[(u - a[2]) * z / a[0], (v - a[5]) * z / a[4], z, np.ones(len(z))] @ T.T
    rows.append(np.c_[P[:, :3], z])
A = np.vstack(rows)
# 바닥 높이 근처만 (카메라 궤적 높이 기준 GROUND 범위)
cam = tr[:, 1:4]
j = np.concatenate([np.argmin(((A[k:k+20000, None, :2] - cam[None, :, :2]) ** 2).sum(-1), 1) for k in range(0, len(A), 20000)])
h = A[:, 2] - cam[j, 2]; A = A[(h < GHI) & (h > GLO)]
print(f'흰 바닥 점 {len(A)}개, 관측 거리 분포 p10/p50/p90 = {np.percentile(A[:,3],[10,50,90]).round(1)}')
for lo, hi in zip(edges[:-1], edges[1:]):
    s = (A[:, 3] >= lo) & (A[:, 3] < hi); out = Path.home() / f'b_slam/exports/{run}_paint_{lo:g}-{hi:g}m'; out.mkdir(exist_ok=True)
    np.save(out / 'paint_xy.npy', A[s, :2]); print(f'  {lo:g}-{hi:g}m: {s.sum()}점 → {out.name}')
