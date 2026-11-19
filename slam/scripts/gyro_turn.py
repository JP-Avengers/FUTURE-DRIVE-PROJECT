"""IMU 자이로를 중력 축으로 적분한 누적 요(시각과 무관). 사용: gyro_turn.py <bag_dir> <run> [step_s] [out_npz: t, yaw(도)]"""
import sys, sqlite3, numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
bag, run = Path(sys.argv[1]), sys.argv[2]; step = float(sys.argv[3]) if len(sys.argv) > 3 else 8
st = np.array([x[0] for x in sqlite3.connect(str(Path.home() / f'maps/{run}.db')).execute('select stamp from Node order by id')]); t0 = st[0]
G, A = [], []
with AnyReader([bag]) as r:
    cs = {c.topic: c for c in r.connections}
    for c, ts, raw in r.messages(connections=[cs['/camera/camera/gyro/sample'], cs['/camera/camera/accel/sample']]):
        m = r.deserialize(raw, c.msgtype); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if 'gyro' in c.topic: G.append([t, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        else: A.append([t, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
G = np.array(G); A = np.array(A); G = G[np.argsort(G[:, 0])]; A = A[np.argsort(A[:, 0])]
# 가속도를 1초 창으로 부드럽게 → 위 방향, 자이로 시각에 보간
from scipy.ndimage import uniform_filter1d
up = np.stack([np.interp(G[:, 0], A[:, 0], uniform_filter1d(A[:, k], 100)) for k in (1, 2, 3)], 1); up /= np.linalg.norm(up, axis=1, keepdims=True)
w = (G[:, 1:4] * up).sum(1); dt = np.diff(G[:, 0], prepend=G[0, 0]); dt[dt > 0.05] = 0
bias = np.median(w[(G[:, 0] < t0 + 2)]) if ((G[:, 0] < t0 + 2).sum() > 50) else 0.0
yaw = np.degrees(np.cumsum((w - 0) * dt))
print(f'{run}: 자이로 {len(G)}개, 처음 2초 중앙(정지 가정 아님, 참고) {np.degrees(bias):+.2f}°/s')
for t in np.arange(0, G[-1, 0] - t0, step):
    k = int(np.searchsorted(G[:, 0], t0 + t)); print(f'  t={t:6.1f}s 누적 요 {yaw[min(k, len(yaw) - 1)]:+8.1f}°')
if len(sys.argv) > 4: np.savez(sys.argv[4], t=G[:, 0], yaw=yaw); print('→', sys.argv[4])
