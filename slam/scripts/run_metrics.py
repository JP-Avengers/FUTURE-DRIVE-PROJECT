#!/usr/bin/env python3
"""실행 품질 지표: 노드 수, 루프 클로저 수, 폐루프 오차(출발-도착 거리), 궤적 길이, 오도메트리 끊김.
사용: run_metrics.py <run_name>  (exports/<run>/traj_*.txt, maps/<run>.db, logs/<run>/rtabmap.log 사용)"""
import sys, sqlite3, re
from pathlib import Path
import numpy as np
run = sys.argv[1]; H = Path.home()
db = sqlite3.connect(H / f'maps/{run}.db')
nodes = db.execute('select count(*) from Node').fetchone()[0]
maps = db.execute('select count(distinct map_id) from Node').fetchone()[0]
types = dict(db.execute('select type, count(*) from Link group by type').fetchall())
# RTAB-Map Link 타입: 0 이웃, 1 전역 루프, 2 근접(space), 3 근접(time), 4 사용자, ...
def traj(name):
    p = H / f'b_slam/exports/{run}/{name}_poses.txt'
    a = np.loadtxt(p, comments='#', ndmin=2)
    return a[:, 1:4]
log = (H / f'b_slam/logs/{run}/rtabmap.log').read_text(errors='ignore')
odom_n = len(re.findall(r'Odom: quality', log)); lost = len(re.findall(r'quality=0,', log))
print(f'run={run}  nodes={nodes}  map_sessions={maps}  links={types}')
print(f'odom updates={odom_n}  odom lost frames={lost} ({100*lost/max(odom_n,1):.2f}%)')
for name in ('traj_odom', 'traj_optimized'):
    p = traj(name); seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    gap = np.linalg.norm(p[-1] - p[0]); gap_xy = np.linalg.norm(p[-1, :2] - p[0, :2])
    print(f'{name:15s} 길이 {seg.sum():7.1f} m | 출발-도착 {gap:6.2f} m (수평 {gap_xy:5.2f}, 높이차 {p[-1,2]-p[0,2]:+.2f}) | z 범위 {p[:,2].min():+.2f}~{p[:,2].max():+.2f} m')
print(f'loop closures(전역)={types.get(1, 0)}  근접 링크={types.get(2, 0)}')

# multi-session: 세션별 노드 수와 세션 간 루프 클로저(서로 다른 map_id를 잇는 링크)
if maps > 1:
    mid = dict(db.execute('select id, map_id from Node').fetchall())
    per = db.execute('select map_id, count(*) from Node group by map_id').fetchall()
    inter = [(a, b, t) for a, b, t in db.execute('select from_id, to_id, type from Link') if t != 9 and mid.get(a) != mid.get(b)]
    print('세션별 노드:', ', '.join(f'map{m}={n}' for m, n in per))
    print(f'세션 간 링크 {len(inter)}개 (타입별: {dict((t, sum(1 for x in inter if x[2] == t)) for t in set(x[2] for x in inter))})')
