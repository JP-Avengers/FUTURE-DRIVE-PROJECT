"""RTAB-Map DB 조각별 요약: 조각(map_id)마다 노드 수 · 오도메트리 경로 길이, 전체 노드, 가장 큰 조각 비율, 내보낸 최적화 궤적 길이.
사용: db_frag.py <run> [<run> ...]  (maps/<run>.db, logs/<run>/rtabmap.log, exports/<run>/traj_optimized_poses.txt)"""
import sys, sqlite3, numpy as np
from pathlib import Path
H = Path.home()
print(f"{'run':7s} {'끊김':>6s} {'노드':>5s} {'조각':>4s} {'1개 노드 조각':>11s} {'가장 큰 조각(노드, 경로)':>24s} {'조각 경로 합':>10s} {'최적화 궤적':>9s}")
for run in sys.argv[1:]:
    log = (H / f'b_slam/logs/{run}/rtabmap.log').read_text(errors='ignore')
    n = log.count('Odom: quality'); f = log.count('quality=0,'); lost = 100 * f / max(n, 1)
    db = sqlite3.connect(H / f'maps/{run}.db')
    rows = db.execute('select id, map_id, pose from Node order by id').fetchall()
    frag = {}
    for i, m, p in rows:
        if p is None or len(p) < 48: continue
        P = np.frombuffer(p, np.float32)[:12].reshape(3, 4); frag.setdefault(m, []).append(P[:, 3])
    L = {m: float(np.linalg.norm(np.diff(np.array(v), axis=0), axis=1).sum()) if len(v) > 1 else 0.0 for m, v in frag.items()}
    big = max(frag, key=lambda m: len(frag[m])) if frag else None
    t = H / f'b_slam/exports/{run}/traj_optimized_poses.txt'
    to = 0.0
    if t.exists():
        a = np.loadtxt(t, comments='#', ndmin=2)
        if len(a) > 1: to = float(np.linalg.norm(np.diff(a[:, 1:4], axis=0), axis=1).sum())
    ones = sum(1 for v in frag.values() if len(v) == 1)
    bs = f"{len(frag[big])} ({100 * len(frag[big]) / max(len(rows), 1):.0f}%), {L[big]:.1f} m" if big is not None else '-'
    print(f"{run:7s} {lost:5.1f}% {len(rows):5d} {len(frag):4d} {ones:11d} {bs:>24s} {sum(L.values()):8.1f} m {to:7.1f} m")
