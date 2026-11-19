"""위로 든 회차 탐색: 키프레임마다 바닥 · 천장 평면(RANSAC)과 IMU 가속도 위 방향을 비교. 바닥과 천장이 함께 보이면 천장 높이 = 카메라~바닥 + 카메라~천장.
사용: ceil_explore.py <run> <corr_bag_dir> <out_npz>"""
import sys, re, sqlite3, struct, numpy as np, cv2
from pathlib import Path
from rosbags.highlevel import AnyReader
run, bagd, outp = sys.argv[1], Path(sys.argv[2]), sys.argv[3]; E = Path.home() / 'b_slam/exports'; K = E / run / 'keyframes'
rows = [x for x in sqlite3.connect(str(Path.home() / f'maps/{run}.db')).execute('select id, map_id, stamp, pose from Node order by id') if x[3] is not None and len(bytes(x[3])) == 48]
ids = np.array([x[0] for x in rows]); st = np.array([x[2] for x in rows]); N = len(rows)
# IMU 가속도 전부(카메라 광학 좌표: x 오른쪽, y 아래, z 앞)
with AnyReader([bagd]) as r:
    c = [x for x in r.connections if x.topic == '/camera/camera/accel/sample']; T, A = [], []
    for con, ts, raw in r.messages(connections=c):
        m = r.deserialize(raw, con.msgtype); T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9); A.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
T = np.array(T); A = np.array(A)
def imu_up(t, w=0.25):
    s = (T > t - w) & (T < t + w); u = A[s].mean(0); return u / np.linalg.norm(u), float(np.linalg.norm(A[s], axis=1).std())
rng = np.random.default_rng(0)
def plane(P, upg, want, lo, hi):
    """want=+1 바닥(카메라 아래, 법선 위), -1 천장(카메라 위). upg = 대략 위 방향(카메라 좌표). → (법선(위 쪽), 거리, 점 수)"""
    if P is None or len(P) < 800: return None
    S = P[rng.choice(len(P), min(len(P), 6000), replace=False)]; tri = S[rng.integers(0, len(S), (400, 3))]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); nn = np.linalg.norm(n, axis=1); ok = nn > 1e-6; n[ok] /= nn[ok, None]; n[(n @ upg) < 0] *= -1   # 법선 = 위 방향
    d = -(n * tri[:, 0]).sum(1)          # n·p + d = 0, 위 법선: 바닥이면 d>0 (카메라가 평면 위), 천장이면 d<0
    ok &= (n @ upg > np.cos(np.radians(25))) & (want * d > lo) & (want * d < hi)
    if not ok.any(): return None
    n, d = n[ok], d[ok]; cnt = (np.abs(S @ n.T + d) < (0.02 + 0.01 * S[:, 2:3])).sum(0); b = int(np.argmax(cnt))
    if cnt[b] < max(250, 0.05 * len(S)): return None
    inl = P[np.abs(P @ n[b] + d[b]) < (0.02 + 0.01 * P[:, 2])]; c = inl.mean(0); _, sv, Vt = np.linalg.svd(inl - c, full_matrices=False)
    nb = Vt[2] * (1 if Vt[2] @ upg > 0 else -1); db = -(nb @ c)
    if sv[1] / np.sqrt(len(inl)) < 0.12: return None
    return nb, abs(db), len(inl)
out = dict(ids=ids, st=st, up_imu=np.full((N, 3), np.nan), acc_sd=np.full(N, np.nan), fl_n=np.full((N, 3), np.nan), fl_h=np.full(N, np.nan), ce_n=np.full((N, 3), np.nan), ce_d=np.full(N, np.nan), has=np.zeros(N, bool))
def calib(i): return [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
for j in range(N):
    i = int(ids[j]); f = K / f'kf_depth/{i}.png'
    if not f.exists(): continue
    out['has'][j] = True; k = calib(i); dimg = cv2.imread(str(f), -1); v, u = np.mgrid[2:dimg.shape[0]:4, 2:dimg.shape[1]:4]; z = dimg[v, u].astype(np.float32) / 1000; m = (z > 0.3) & (z < 5.0)
    P = np.stack([(u[m] - k[2]) / k[0] * z[m], (v[m] - k[5]) / k[4] * z[m], z[m]], 1).astype(np.float64)
    up, sd = imu_up(st[j]); out['up_imu'][j] = up; out['acc_sd'][j] = sd
    fr = plane(P, up, +1, 0.5, 2.0); ce = plane(P, up, -1, 0.4, 2.5)
    if fr: out['fl_n'][j], out['fl_h'][j] = fr[0], fr[1]
    if ce: out['ce_n'][j], out['ce_d'][j] = ce[0], ce[1]
np.savez(outp, **out)
h = out['has']; f_ = np.isfinite(out['fl_h']); c_ = np.isfinite(out['ce_d']); b_ = f_ & c_
ang = lambda X, Y: np.degrees(np.arccos(np.clip((X * Y).sum(1), -1, 1)))
pitch = np.degrees(np.arcsin(np.clip(out['up_imu'][h][:, 2], -1, 1)))
print(f'{run}: 노드 {N}, 키프레임 {h.sum()} | IMU 기울기(위 +) 중앙 {np.median(pitch):+.0f}° | 바닥 잡힘 {f_.sum()} · 천장 잡힘 {c_.sum()} · 둘 다 {b_.sum()}')
if f_.any(): print(f'   바닥: 카메라 높이 중앙 {np.nanmedian(out["fl_h"]):.2f}m | 바닥 법선 vs IMU 위 각도 중앙 {np.median(ang(out["fl_n"][f_], out["up_imu"][f_])):.1f}°')
if c_.any(): print(f'   천장: 카메라~천장 중앙 {np.nanmedian(out["ce_d"]):.2f}m (10~90% {np.nanpercentile(out["ce_d"], 10):.2f}~{np.nanpercentile(out["ce_d"], 90):.2f}) | 천장 법선 vs IMU 위 각도 중앙 {np.median(ang(out["ce_n"][c_], out["up_imu"][c_])):.1f}°')
if b_.any():
    H = out['fl_h'][b_] + out['ce_d'][b_]; print(f'   천장 높이(바닥+천장 함께 본 {b_.sum()}장): 중앙 {np.median(H):.2f}m (10~90% {np.percentile(H, 10):.2f}~{np.percentile(H, 90):.2f}) | 두 법선 사이 각 중앙 {np.median(ang(out["fl_n"][b_], out["ce_n"][b_])):.1f}°')
