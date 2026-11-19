"""천장 높이: (a) 한 키프레임에서 바닥 · 천장이 모두 IMU 위 방향과 6° 안으로 맞는 경우, (b) 같은 조각 안에서 바닥을 본 키프레임과
천장을 본 키프레임이 10장 안에 있으면 오도메트리 높이 차(IMU로 중력 방향을 맞춘 것)로 이어 붙임.
사용: ceil_height.py <explore_npz 가 있는 폴더> <run>..."""
import sys, sqlite3, struct, re, numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
O = Path(sys.argv[1]); E = Path.home() / 'b_slam/exports'
def pose_T(r): T = np.eye(4); T[:3, :3] = Rot.from_quat(r[4:8]).as_matrix(); T[:3, 3] = r[1:4]; return T
tl = np.loadtxt(E / 'in2/traj_optimized_poses.txt', comments='#', ndmin=2); tc = np.loadtxt(E / 'in2/traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
TLO = np.linalg.inv(pose_T(tl[0])) @ pose_T(tc[int(np.argmin(np.abs(tc[:, 0] - tl[0, 0])))])
ang = lambda X, Y: np.degrees(np.arccos(np.clip((X * Y).sum(-1), -1, 1)))
allA, allB = [], []
for run in sys.argv[2:]:
    z = np.load(O / f'explore_{run}.npz'); ids = z['ids']; up = z['up_imu']
    rows = {x[0]: x for x in sqlite3.connect(str(Path.home() / f'maps/{run}.db')).execute('select id, map_id, stamp, pose from Node') if x[3] is not None and len(bytes(x[3])) == 48}
    TL = np.array([np.r_[np.array(struct.unpack('12f', bytes(rows[i][3]))).reshape(3, 4), [[0, 0, 0, 1]]] for i in ids]); TO = TL @ TLO; mid = np.array([rows[i][1] for i in ids])
    fok = np.isfinite(z['fl_h']) & (ang(z['fl_n'], up) < 6); cok = np.isfinite(z['ce_d']) & (ang(z['ce_n'], up) < 6)
    A = (z['fl_h'] + z['ce_d'])[fok & cok]
    # 위 방향(오도메트리 세계 좌표) = R0 @ up_cam. 이웃 키프레임 사이 높이 차 = (p_j - p_i) · g
    B = []
    for i in np.where(fok)[0]:
        for j in np.where(cok)[0]:
            if mid[i] != mid[j] or abs(i - j) > 10 or i == j: continue
            g = TO[i, :3, :3] @ up[i] + TO[j, :3, :3] @ up[j]; g /= np.linalg.norm(g)
            dz = (TO[j, :3, 3] - TO[i, :3, 3]) @ g; B.append(z['fl_h'][i] + dz + z['ce_d'][j])
    B = np.array(B); allA += list(A); allB += list(B)
    print(f'{run}: 바닥 OK {fok.sum()} · 천장 OK {cok.sum()} | (a) 한 장에서 둘 다 {len(A)}: ' + (f'중앙 {np.median(A):.2f}m' if len(A) else '-') +
          f' | (b) 이웃 짝 {len(B)}: ' + (f'중앙 {np.median(B):.2f}m (25~75% {np.percentile(B, 25):.2f}~{np.percentile(B, 75):.2f})' if len(B) else '-'))
for nm, X in (('(a)', allA), ('(b)', allB)):
    if X: X = np.array(X); print(f'전체 {nm} {len(X)}개: 중앙 {np.median(X):.3f}m, 25~75% {np.percentile(X, 25):.2f}~{np.percentile(X, 75):.2f}, 10~90% {np.percentile(X, 10):.2f}~{np.percentile(X, 90):.2f}')
