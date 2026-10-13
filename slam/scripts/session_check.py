#!/usr/bin/env python3
"""multi-session 맵에서 마지막 세션(새로 붙인 회차)이 이전 세션들과 맞는지: 세션별 주차선을 따로 뽑아(keyframe_paint.py)
새 세션 키프레임마다 주변 8m 주차선 → 이전 세션 주차선 최근접 거리 중앙값. 구역(A·B·C, 아래쪽 변)별로도.
사용: session_check.py <run> [GROUND=-1.4,-0.7]   (~/tsdf_env, 같은 맵 좌표계 안에서 비교)"""
import sys, os, sqlite3, subprocess, shutil
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
run = sys.argv[1]; GROUND = sys.argv[2] if len(sys.argv) > 2 else '-1.4,-0.7'
E = Path.home() / 'b_slam/exports'; S = Path.home() / 'b_slam/scripts'; PY = str(Path.home() / 'tsdf_env/bin/python')
A = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
mid = dict(sqlite3.connect(Path.home() / f'maps/{run}.db').execute('select id, map_id from Node').fetchall())
ms = np.array([mid[int(i)] for i in A[:, 8]]); last = int(os.environ.get('SESSION', ms.max()))  # SESSION=<map_id>로 특정 세션 판정
paints = {}
for tag, sel in (('new', ms == last), ('old', ms != last)):
    d = E / f'{run}_{tag}'; shutil.rmtree(d, ignore_errors=True); d.mkdir()
    (d / 'keyframes').symlink_to(E / run / 'keyframes')
    np.savetxt(d / 'traj_camera_id_camera_poses.txt', A[sel], fmt=['%.6f'] * 8 + ['%d'], header='timestamp x y z qx qy qz qw id', comments='#')
    subprocess.run([PY, str(S / 'keyframe_paint.py'), f'{run}_{tag}', '1.5', '15'], env={**os.environ, 'GROUND': GROUND, 'DMIN': '1.5'}, check=True, capture_output=True)
    paints[tag] = np.load(E / f'{run}_{tag}_paint_1.5-15m/paint_xy.npy')
P, R = paints['new'], paints['old']; d, _ = cKDTree(R).query(P); tp = cKDTree(P)
K = A[ms == last][:, 1:3]; res = np.full(len(K), np.nan)
for k, c in enumerate(K):
    i = tp.query_ball_point(c, 8.0); dd = d[i]; dd = dd[dd < 1.5]
    if len(dd) >= 300: res[k] = np.median(dd)
ok = ~np.isnan(res)
print(f'{run}: 새 세션 map{last} 키프레임 {len(K)}개 (이전 세션 {len(set(ms)) - 1}개) | 판정 가능 {ok.sum()} | '
      f'주변 주차선 잔차 중앙 {np.nanmedian(res)*100:.1f}cm, 15cm 미만 {np.mean(res[ok] < 0.15)*100:.0f}%, 50cm 이상 {np.mean(res[ok] >= 0.5)*100:.0f}% | '
      f'새 주차선 중 이전 것 1m 안 {np.mean(d < 1)*100:.0f}%')
np.save(E / run / 'session_res.npy', res)
