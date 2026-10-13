#!/usr/bin/env python3
"""새 실행(exports/<run>)의 키프레임 자세가 기준 궤적(v0.3 패키지 trajectory_camera_optical.txt)에서 얼마나 움직였는지,
세션별 높이 범위. 공통 ID로 강체 변환(Umeyama, 스케일 없음)을 구해 잔차를 본다. 사용: frame_drift.py <run> <기준 궤적 txt>"""
import sys, sqlite3
from pathlib import Path
import numpy as np
run, ref = sys.argv[1], sys.argv[2]
E = Path.home() / f'b_slam/exports/{run}'
A = np.loadtxt(E / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); R = np.loadtxt(ref, comments='#', ndmin=2)
pa = {int(r[8]): r[1:4] for r in A}; pr = {int(r[8]): r[1:4] for r in R}
ids = sorted(set(pa) & set(pr)); P = np.array([pa[i] for i in ids]); Q = np.array([pr[i] for i in ids])
print(f'공통 키프레임 {len(ids)}개 (기준 {len(pr)}, 새 실행 {len(pa)})')
d0 = np.linalg.norm(P - Q, axis=1); print(f'변환 없이 위치 차: 중앙 {np.median(d0)*100:.1f}cm, 90% {np.percentile(d0,90)*100:.1f}cm, 최대 {d0.max()*100:.1f}cm')
mp, mq = P.mean(0), Q.mean(0); U, S, Vt = np.linalg.svd((P - mp).T @ (Q - mq)); Dg = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
Rm = Vt.T @ Dg @ U.T; t = mq - Rm @ mp; d1 = np.linalg.norm((P @ Rm.T + t) - Q, axis=1)
yaw = np.degrees(np.arctan2(Rm[1, 0], Rm[0, 0])); tilt = np.degrees(np.arccos(np.clip(Rm[2, 2], -1, 1)))
print(f'강체 맞춤 후 차: 중앙 {np.median(d1)*100:.1f}cm, 90% {np.percentile(d1,90)*100:.1f}cm, 최대 {d1.max()*100:.1f}cm  (yaw {yaw:+.2f}°, 기울기 {tilt:.2f}°, 이동 {np.round(t,3)})')
np.save(E / 'T_to_v03.npy', np.vstack([np.c_[Rm, t], [0, 0, 0, 1]]))
db = sqlite3.connect(Path.home() / f'maps/{run}.db'); mid = dict(db.execute('select id, map_id from Node').fetchall())
for m in sorted(set(mid[i] for i in pa)):
    z = np.array([pa[i][2] for i in pa if mid[i] == m]); print(f'  map{m}: 키프레임 {len(z)}, 높이 {z.min():+.2f} ~ {z.max():+.2f} m')
