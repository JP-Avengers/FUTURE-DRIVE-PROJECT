#!/usr/bin/env python3
"""새 회차(v0.3 좌표계로 정합된 nr 폴더)의 키프레임별 국소 일치도: 키프레임 주변 R m의 그 회차 주차선 → 기준 주차선(v0.3 + 10/9 비강체 3차) 최근접 거리 중앙값.
기준과 겹치는 점(1.5m 안)이 MINPT 미만이면 '판정 불가' → 경로 앞뒤 ±W 키프레임의 판정을 따름(둘 다 불가면 제외).
좋음 = 중앙값 < THR. 출력: exports/<nr>/kf_good.npy (키프레임 순서 = traj_camera_id 파일 순서, bool), 그림.
사용: kf_consistency.py <src_run> <nr_dir> [THR=0.15]   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, cv2
from scipy.spatial import cKDTree
E = Path.home() / 'b_slam/exports'; src, nr = sys.argv[1:3]; THR = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15
RAD, MINPT, W = 8.0, 300, 5
import os
REFS = os.environ.get('KFC_REF')  # 기준 주차선 npy 목록(쉼표). 없으면 v0.3 + 10/9 비강체 3차
ref = np.vstack([np.load(p) for p in REFS.split(',')]) if REFS else np.vstack([np.load(E / 'v03_paintref/paint_xy.npy'), np.load(E / 'gn03nr3/paint_xy.npy'), np.load(E / 'gm03nr3/paint_xy.npy')])
tr = cKDTree(ref)
P = np.load(E / nr / 'paint_xy.npy'); d, _ = tr.query(P); tp = cKDTree(P)
K = np.loadtxt(E / nr / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)[:, 1:3]
res = np.full(len(K), np.nan)
for k, c in enumerate(K):
    i = tp.query_ball_point(c, RAD); dd = d[i]; dd = dd[dd < 1.5]
    if len(dd) >= MINPT: res[k] = np.median(dd)
good = np.zeros(len(K), bool); state = np.where(np.isnan(res), -1, (res < THR).astype(int))
for k in range(len(K)):
    if state[k] >= 0: good[k] = state[k] == 1
    else:
        nb = state[max(0, k - W):k + W + 1]; nb = nb[nb >= 0]
        if len(nb) == 0: good[k] = os.environ.get('KFC_UNKNOWN_KEEP') == '1'  # 비교할 기준이 없는 구간: 채우기용(지붕)이면 유지
        else: good[k] = nb.mean() >= 0.5
np.save(E / nr / 'kf_good.npy', good)
print(f'{nr}: 키프레임 {len(K)}개 | 판정 가능 {np.sum(~np.isnan(res))}개(잔차 중앙 {np.nanmedian(res)*100:.1f}cm) | 좋음 {good.sum()}개, 제외 {np.sum(~good)}개')
# 그림
x0, x1, y0, y1, s = -62, 14, -38, 26, 10; w, h = (x1 - x0) * s, (y1 - y0) * s; im = np.full((h, w, 3), 255, np.uint8)
def px(Q): return np.c_[(Q[:, 0] - x0) * s, h - 1 - (Q[:, 1] - y0) * s].astype(int)
q = px(ref[::10]); k = (q[:, 0] >= 0) & (q[:, 0] < w) & (q[:, 1] >= 0) & (q[:, 1] < h); im[q[k, 1], q[k, 0]] = (200, 200, 200)
for (u, v), g, r in zip(px(K), good, res):
    cv2.circle(im, (int(u), int(v)), 4, (0, 170, 0) if g else (0, 0, 230), -1 if not np.isnan(r) else 1)
cv2.putText(im, f'{nr}: green=use, red=exclude (filled=measured, ring=inherited)', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1)
cv2.imwrite(str(E / nr / 'kf_good.png'), im)
