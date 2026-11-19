"""오도메트리 원래 요(카메라 앞 방향을 IMU 수평면에 투영) 시간 변화와 조각 경계. 사용: yaw_check.py <run> <explore_npz_dir> <ref_raw=in2>"""
import sys, sqlite3, struct, numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
run, EXD = sys.argv[1], Path(sys.argv[2]); E = Path.home() / 'b_slam/exports'
def pose_T(r): T = np.eye(4); T[:3, :3] = Rot.from_quat(r[4:8]).as_matrix(); T[:3, 3] = r[1:4]; return T
tl = np.loadtxt(E / 'in2/traj_optimized_poses.txt', comments='#', ndmin=2); tc = np.loadtxt(E / 'in2/traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
TLO = np.linalg.inv(pose_T(tl[0])) @ pose_T(tc[int(np.argmin(np.abs(tc[:, 0] - tl[0, 0])))])
rows = [x for x in sqlite3.connect(str(Path.home() / f'maps/{run}.db')).execute('select id, map_id, stamp, pose from Node order by id') if x[3] is not None and len(bytes(x[3])) == 48]
N = len(rows); TL = np.array([np.r_[np.array(struct.unpack('12f', bytes(x[3]))).reshape(3, 4), [[0, 0, 0, 1]]] for x in rows]); TO = TL @ TLO
mid = np.array([x[1] for x in rows]); st = np.array([x[2] for x in rows]); ex = np.load(EXD / f'explore_{run}.npz'); up = ex['up_imu']
yaw = np.full(N, np.nan)
for j in range(N):
    if not np.isfinite(up[j, 0]): continue
    g = TO[j, :3, :3] @ up[j]; f = TO[j, :3, :3] @ np.array([0, -1.0, 0]) if abs(up[j, 2]) > 0.8 else TO[j, :3, :3] @ np.array([0, 0, 1.0])   # 거의 천장을 보면 카메라 위쪽(-y)을 앞으로
    f = f - (f @ g) * g; e1 = np.cross(g, [1.0, 0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(g, e1); yaw[j] = np.degrees(np.arctan2(f @ e2, f @ e1))
yu = np.degrees(np.unwrap(np.radians(np.nan_to_num(yaw, nan=0.0))))
print(f'{run}: 노드 {N}, 조각 바뀌는 곳 {int((np.diff(mid) != 0).sum())}')
for j in range(0, N, 8):
    print(f'  {j:3d} t={st[j]-st[0]:6.1f}s 조각{mid[j]:3d} 요 {yu[j]:+7.1f}°  위치 ({TO[j,0,3]:+6.1f},{TO[j,1,3]:+6.1f},{TO[j,2,3]:+6.1f})')
