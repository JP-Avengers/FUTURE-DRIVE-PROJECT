"""[천장판] 카메라를 위로 든 회차용: 기울기 · 높이를 천장 평면 · IMU로 정함(ceil_explore.py 결과 필요). 나머지는 in_localize.py 와 같음.
실내 회차를 기준 복도 맵(건물 좌표, in_manhattan.py 출력)에 맞춰 넣기 — RTAB-Map 맵이 조각나거나 높이가 흐른 회차용.
 입력은 DB의 노드별 오도메트리 자세(끊긴 조각 포함 전체)와 키프레임 depth.
 (1) 중력: 키프레임마다 depth에서 바닥 평면(RANSAC)을 찾아 기울기(롤·피치)와 카메라 높이를 정하고, 오도메트리 걸음을 다시 적분
 (2) 나침반: 벽 점 주 방향(90° 주기)으로 요 흐름을 바로잡아 다시 적분 → 벽이 축에 나란한 좌표
 (3) 전역 맞춤: 90° 단위 회전 4가지 × 이동을 기준 맵 벽 격자와의 겹침으로 탐색
 (4) 따라가기: 키프레임 순서대로 기준 벽 거리 지도에 이동만 미세 탐색(요는 나침반 값 고정). 벽 방향(앞뒤)으로는 벌점. 벽이 안 맞으면 넓게 다시 탐색
 출력: exports/<run>L/ (건물 좌표 자세·keyframes 링크), <out>/<run>_localize.{json,png}
사용: in_localize_ceil.py <run> <ref_run_m> <out_dir> [--raw in2] [--hmax 2.6] [--hc 2.87] [--ex <explore_npz · gyro_npz 폴더>] [--free 2.0 (통로 밖 벌점 가중, 0=끔)]"""
import sys, re, json, os, sqlite3, struct, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, refrun, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: a[a.index(k) + 1] if k in a else d
RAW = opt('--raw', refrun.rstrip('m')); HMAX = float(opt('--hmax', 2.6)); CELL = 0.03; E = Path.home() / 'b_slam/exports'; K = E / run / 'keyframes'
def calib(Kd, i, pre='kf_'): return [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (Kd / f'{pre}calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
def pose_T(r): T = np.eye(4); T[:3, :3] = Rot.from_quat(r[4:8]).as_matrix(); T[:3, 3] = r[1:4]; return T
tl = np.loadtxt(E / RAW / 'traj_optimized_poses.txt', comments='#', ndmin=2); tc = np.loadtxt(E / RAW / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
j0 = int(np.argmin(np.abs(tc[:, 0] - tl[0, 0]))); TLO = np.linalg.inv(pose_T(tl[0])) @ pose_T(tc[j0])            # camera_link → 광학
rows = [x for x in sqlite3.connect(str(Path.home() / f'maps/{run}.db')).execute('select id, map_id, stamp, pose from Node order by id') if x[3] is not None and len(bytes(x[3])) == 48]
ids = np.array([x[0] for x in rows]); stamps = np.array([x[2] for x in rows]); N = len(rows)
TL = np.tile(np.eye(4), (N, 1, 1)); TL[:, :3, :] = np.array([struct.unpack('12f', bytes(x[3])) for x in rows]).reshape(N, 3, 4); TO = TL @ TLO; R0 = TO[:, :3, :3]; p0 = TO[:, :3, 3]
has = np.array([(K / f'kf_depth/{i}.png').exists() for i in ids])
PC = [None] * N
for j in np.where(has)[0]:
    i = int(ids[j]); k = calib(K, i); d = cv2.imread(str(K / f'kf_depth/{i}.png'), -1); v, u = np.mgrid[2:d.shape[0]:4, 2:d.shape[1]:4]; z = d[v, u].astype(np.float32) / 1000; m = (z > 0.4) & (z < 4.5)
    PC[j] = np.stack([(u[m] - k[2]) / k[0] * z[m], (v[m] - k[5]) / k[4] * z[m], z[m]], 1).astype(np.float64)
# (1) 중력 · 높이 (천장판): ceil_explore.py 결과를 씀. 키프레임마다 위 방향 = 천장 평면 법선(IMU 위 방향과 8° 안) > 바닥 평면 법선(같은 조건) > IMU 가속도(±0.25초 평균)
#     카메라 높이 = 천장 높이 HC - 카메라~천장 거리 (천장이 안 잡히면 바닥 높이, 둘 다 없으면 이웃 보간)
rng = np.random.default_rng(0); HC = float(opt('--hc', 2.87)); EXD = Path(opt('--ex', str(Path.home() / 'b_slam/exports/indoor1119')))
ex = np.load(EXD / f'explore_{run}.npz'); assert len(ex['ids']) == N and (ex['ids'] == ids).all(), '탐색 결과와 노드 순서가 다름'
ang_ = lambda X, Y: np.degrees(np.arccos(np.clip((X * Y).sum(-1), -1, 1)))
upi = ex['up_imu']; okc = np.isfinite(ex['ce_d']) & (ang_(ex['ce_n'], upi) < 8); okf = np.isfinite(ex['fl_h']) & (ang_(ex['fl_n'], upi) < 8) & ~okc
U = np.where(okc[:, None], ex['ce_n'], np.where(okf[:, None], ex['fl_n'], upi)); hraw = np.where(okc, HC - ex['ce_d'], np.where(okf, ex['fl_h'], np.nan))
src = np.where(okc, 1, np.where(okf, 2, np.where(np.isfinite(upi[:, 0]), 3, 0)))
q = np.full((N, 2), np.nan); hc = hraw.copy()
for j in np.where(src > 0)[0]:
    up = R0[j] @ U[j]; ax = np.cross(up, [0, 0, 1.0]); s = np.linalg.norm(ax); ang = np.arctan2(s, up[2]); q[j] = ax[:2] / s * ang if s > 1e-9 else 0
vt = np.isfinite(q[:, 0]); it = np.where(vt)[0]; ar = np.arange(N)
for col in range(2): q[it, col] = ndimage.median_filter(q[it, col], 3, mode='nearest')
qf = np.c_[np.interp(ar, it, q[it, 0]), np.interp(ar, it, q[it, 1])]
vq = np.isfinite(hc); idx = np.where(vq)[0]; hc[idx] = ndimage.median_filter(hc[idx], 5, mode='nearest'); hs = np.interp(ar, idx, hc[idx])
Q = Rot.from_rotvec(np.c_[qf, np.zeros(N)]).as_matrix(); R1 = Q @ R0
print(f'기울기 출처: 천장 {int((src == 1).sum())}장, 바닥 {int((src == 2).sum())}장, IMU {int((src == 3).sum())}장 | 높이 출처 있는 키프레임 {vq.sum()} (천장 높이 {HC:.2f}m 가정)')
# (1b) 조각 사이 방향: 추적이 끊겨 다시 시작하면 오도메트리 방향이 틀어질 수 있음(ic30b 반환점: 실제 162° 회전을 약 90°로 기록).
#      조각 안에서는 오도메트리 요를 그대로 쓰고, 조각마다 IMU 자이로 누적 요(gyro_turn.py)와의 차이(원형 평균)만큼 돌림.
gz = np.load(EXD / f'gyro_{run}.npz'); yaw_g = np.radians(np.interp(stamps, gz['t'], gz['yaw'])); yaw_o = np.arctan2(R1[:, 1, 0], R1[:, 0, 0])
dlt = np.angle(np.exp(1j * (yaw_g - yaw_o))); mids = np.array([x[1] for x in rows]); delta = np.zeros(N); spread = []
for m_ in np.unique(mids):
    s_ = mids == m_; c_ = np.angle(np.mean(np.exp(1j * dlt[s_]))); delta[s_] = c_
    if s_.sum() >= 5: spread.append(np.degrees(np.abs(np.angle(np.exp(1j * (dlt[s_] - c_))))).max())
du = np.degrees(np.unwrap(delta[np.r_[True, mids[1:] != mids[:-1]]]))
print(f'조각 {len(np.unique(mids))}개 | 조각 안 (자이로 - 오도메트리) 요 차이 흩어짐 최대 중앙 {np.median(spread) if spread else float("nan"):.1f}° | 조각 사이 방향 보정 범위 {du.min():+.0f}~{du.max():+.0f}°')
Rzd = Rot.from_euler('z', delta[:, None]).as_matrix(); R1 = Rzd @ R1
p1 = np.zeros((N, 3))
for j in range(N - 1): p1[j + 1] = p1[j] + Rzd[j] @ Q[j] @ (p0[j + 1] - p0[j])
print(f'{run}: 노드 {N}(키프레임 자료 {has.sum()}) | 오도메트리 기울기 오차 중앙 {np.degrees(np.median(np.linalg.norm(qf, axis=1))):.1f}° 최대 {np.degrees(np.linalg.norm(qf, axis=1).max()):.1f}° | 카메라 높이 중앙 {np.median(hs):.2f}m (5~95% {np.percentile(hs, 5):.2f}~{np.percentile(hs, 95):.2f})'
      f' | 다시 적분한 궤적 z 흐름 {np.ptp(p1[:, 2]):.2f}m (오도메트리 {np.ptp(p0[:, 2]):.2f}m)')
pitch = np.degrees(np.arcsin(-R1[:, 2, 2]))
Wl = [np.zeros((0, 2), np.float32)] * N
for j in np.where(has)[0]:
    G_ = PC[j] @ R1[j].T; hgt = hs[j] + G_[:, 2]; qq = G_[(hgt > 0.3) & (hgt < HMAX), :2]
    if len(qq): Wl[j] = (np.unique(np.round(qq / CELL).astype(np.int32), axis=0) * CELL).astype(np.float32)
c2 = p1[:, :2].copy()
# (2) 나침반
def line_score(P, ang):
    n_ = np.array([-np.sin(ang), np.cos(ang)]); s = np.floor((P @ n_ + 8.0) / 0.04).astype(int); s = s[(s >= 0) & (s < 400)]; h = np.bincount(s, minlength=400).astype(float); return float((h * h).sum())
alpha = np.full(N, np.nan); stren = np.zeros(N); A1 = np.radians(np.arange(0, 180, 0.5))
for j in range(N):
    P = np.vstack([Wl[i] + (c2[i] - c2[j]).astype(np.float32) for i in range(max(j - 1, 0), min(j + 2, N))]).astype(np.float64)
    if len(P) < 300: continue
    sc = np.array([line_score(P, t) for t in A1]); b = int(np.argmax(sc)); A2 = A1[b] + np.radians(np.arange(-0.5, 0.51, 0.05)); sc2 = np.array([line_score(P, t) for t in A2])
    alpha[j] = np.degrees(A2[int(np.argmax(sc2))]) % 180; stren[j] = sc.max() / (np.median(sc) + 1e-9)
beta = alpha % 90; okb = np.isfinite(beta) & (stren > 2.0)
# 기준 방향 = 전체 키프레임 벽 방향(90° 주기)의 최빈값. 요 흐름은 시간 순으로 따라가되 직전 흐름에서 10° 넘게 벗어난 값(로비의 비스듬한 구조물 등)은 버림.
# 8장 연속으로 서로 비슷한 값(6° 안)이 버려지면 실제로 요가 튄 것으로 보고 받아들임.
hb = np.bincount((beta[okb] % 90).astype(int), minlength=90).astype(float); hb = ndimage.uniform_filter1d(hb, 5, mode='wrap'); bm = int(np.argmax(hb))
near_ = okb & (np.abs((beta - bm - 0.5 + 45) % 90 - 45) < 4); base = float(bm + 0.5 + np.mean((beta[near_] - bm - 0.5 + 45) % 90 - 45))
psi = np.full(N, np.nan); prev = 0.0; hist = []; rej = []; jumps = 0
for j in np.where(okb)[0]:
    d = (beta[j] - base - prev + 45) % 90 - 45
    if abs(d) < 10: psi[j] = prev + d; hist.append(psi[j]); prev = float(np.median(hist[-5:])); rej = []
    else:
        rej.append((j, d))
        if len(rej) >= 8 and np.ptp([x[1] for x in rej[-8:]]) < 6:
            jump = float(np.median([x[1] for x in rej[-8:]])); prev += jump; jumps += 1
            for jj, dd in rej[-8:]: psi[jj] = prev + (dd - jump)
            hist = [psi[jj] for jj, _ in rej[-8:]]; rej = []
v_ = np.isfinite(psi); psi_f = np.interp(ar, np.where(v_)[0], psi[v_]); psi_s = ndimage.uniform_filter1d(ndimage.median_filter(psi_f, 5, mode='nearest'), 3, mode='nearest')
print(f'벽 방향 쓴 키프레임 {v_.sum()}/{N}(잡힌 것 {okb.sum()}) | 요 흐름 ψ 범위 {psi_s.min():+.1f}~{psi_s.max():+.1f}°, 요 튐 인정 {jumps}회')
pr = np.radians(psi_s); p2 = np.zeros((N, 2))
for i in range(N - 1):
    t = -(pr[i] + pr[i + 1]) / 2; Rz = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]); p2[i + 1] = p2[i] + Rz @ (c2[i + 1] - c2[i])
rot = -np.radians(base); Rb = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]]); pb = p2 @ Rb.T; yawc = rot - pr
def wallpts(j):
    t = yawc[j]; Rz = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]); return Wl[j].astype(np.float64) @ Rz.T + pb[j]
# 기준 맵 벽 격자
rc = np.loadtxt(E / refrun / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); Kr = E / refrun / 'keyframes'; RW = []
for r in rc:
    i = int(r[8]); k = calib(Kr, i); d = cv2.imread(str(Kr / f'kf_depth/{i}.png'), -1); v, u = np.mgrid[2:d.shape[0]:4, 2:d.shape[1]:4]; z = d[v, u].astype(np.float32) / 1000; m = (z > 0.4) & (z < 4.0)
    P = np.stack([(u[m] - k[2]) / k[0] * z[m], (v[m] - k[5]) / k[4] * z[m], z[m]], 1) @ Rot.from_quat(r[4:8]).as_matrix().T + r[1:4]; P = P[(P[:, 2] > 0.3) & (P[:, 2] < 1.9), :2]
    RW.append(np.unique(np.round(P / CELL).astype(np.int32), axis=0))
allr = np.vstack(RW); lo = allr.min(0) * CELL - 12; n = ((allr.max(0) * CELL + 12 - lo) / CELL).astype(int) + 1; cnt = np.zeros(n, np.uint16)
for ij in RW: g = ((ij * CELL - lo) / CELL + 0.5).astype(int); cnt[g[:, 0], g[:, 1]] += 1
occ = cnt >= 2; D = ndimage.distance_transform_edt(~occ) * CELL
def look(P, cap):
    ij = ((P - lo) / CELL).astype(int); okk = (ij >= 0).all(1) & (ij < n).all(1); out = np.full(len(P), cap); out[okk] = np.minimum(D[ij[okk, 0], ij[okk, 1]], cap); return out
# 통로(카메라가 있을 수 있는 곳): 기준 회차 카메라 궤적에서 FREE_R 안, 기준 벽에서 0.25m 넘게 떨어진 곳. 밖으로 나간 거리 = 벌점(--free 0 이면 끔)
FREE_W = float(opt('--free', 2.0)); FREE_R = 2.0
trk = np.zeros(occ.shape, bool); tij = ((rc[:, 1:3] - lo) / CELL).astype(int); trk[tij[:, 0].clip(0, n[0] - 1), tij[:, 1].clip(0, n[1] - 1)] = True
Dt = ndimage.distance_transform_edt(~trk) * CELL; freem = (Dt < FREE_R) & (D > 0.25); Dout = ndimage.distance_transform_edt(~freem) * CELL
def outside(xy):
    ij = ((np.atleast_2d(xy) - lo) / CELL).astype(int); okk = (ij >= 0).all(1) & (ij < n).all(1); o = np.full(len(ij), 5.0); o[okk] = Dout[ij[okk, 0], ij[okk, 1]]; return o
# (3) 전역 맞춤: 6cm 격자 겹침, 90° 회전 4가지
C6 = 0.06; ref6 = cv2.GaussianBlur(cv2.resize(occ.astype(np.float32), None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA), (0, 0), 1.5)
AW = np.vstack([wallpts(j) for j in range(N) if len(Wl[j])]); best = None
for kq in range(4):
    th = kq * np.pi / 2; Rk = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); Pk = AW @ Rk.T; l0 = Pk.min(0) - 0.5; sz = ((Pk.max(0) + 0.5 - l0) / C6).astype(int) + 1
    tm = np.zeros(sz, np.float32); ij = ((Pk - l0) / C6).astype(int); np.add.at(tm, (ij[:, 0], ij[:, 1]), 1); tm = cv2.GaussianBlur(np.minimum(tm, 3) / 3, (0, 0), 1.5)
    pad = np.zeros((ref6.shape[0] + 2 * sz[0], ref6.shape[1] + 2 * sz[1]), np.float32); pad[sz[0]:sz[0] + ref6.shape[0], sz[1]:sz[1] + ref6.shape[1]] = ref6
    res = cv2.matchTemplate(pad, tm, cv2.TM_CCORR); mi = np.unravel_index(np.argmax(res), res.shape); sc = float(res[mi]) / float((tm * tm).sum() ** 0.5 + 1e-9)
    Tk = lo + (np.array(mi) - sz) * C6 - l0
    print(f'  전역 맞춤 회전 {kq * 90:3d}°: 점수 {sc:.2f}, 이동 ({Tk[0]:+.1f},{Tk[1]:+.1f})')
    if best is None or sc > best[0]: best = (sc, kq, Rk, Tk)
_, kq, Rk, Tg = best; base_xy = pb @ Rk.T + Tg
def wp(j, c): return wallpts(j) @ Rk.T + Tg + c
# (4) 따라가기
TRc = 0.6; cj = np.zeros((N, 2)); cur = np.zeros(2); fracs = np.zeros(N); relock = 0
def cost_grid(P, a_, n_, rng_, step, la, ln, cam=None):
    best = (1e9, 0.0, 0.0)
    for da in np.arange(-rng_, rng_ + 1e-6, step):
        for dn in np.arange(-rng_, rng_ + 1e-6, step):
            sh = da * a_ + dn * n_; s = float(look(P + sh, TRc).mean()) + la * abs(da) + ln * abs(dn)
            if cam is not None and FREE_W > 0: s += FREE_W * float(outside(cam + sh).mean())
            if s < best[0]: best = (s, da, dn)
    return best
for j in range(N):
    win = [i for i in range(max(j - 2, 0), min(j + 3, N)) if len(Wl[i])]
    if not win: cj[j] = cur; continue
    P = np.vstack([wp(i, cur) for i in win])
    if len(P) > 4000: P = P[rng.choice(len(P), 4000, replace=False)]
    al = np.radians((alpha[j] if np.isfinite(alpha[j]) else 0.0)) + yawc[j] + kq * np.pi / 2; a_ = np.array([np.cos(al), np.sin(al)]); n_ = np.array([-a_[1], a_[0]])
    d0 = look(P, TRc); near = np.mean(d0 < 0.15); reach = np.mean(d0 < TRc)
    camj = base_xy[j] + cur; outj = float(outside(camj)[0]) if FREE_W > 0 else 0.0
    if j == 0 or ((near < 0.35 or outj > 0.1) and j % 3 == 0):                    # 넓게 다시 탐색(처음, 벽이 안 맞을 때, 카메라가 통로 밖일 때)
        winw = [i for i in range(max(j - 6, 0), min(j + 7, N)) if len(Wl[i])]; Pw = np.vstack([wp(i, cur) for i in winw]); Pw = Pw[rng.choice(len(Pw), min(len(Pw), 6000), replace=False)]
        camw = base_xy[winw] + cur
        s0 = float(look(Pw, TRc).mean()) + (FREE_W * float(outside(camw).mean()) if FREE_W > 0 else 0)
        b = cost_grid(Pw, np.array([1.0, 0]), np.array([0, 1.0]), 2.4 if (j == 0 or outj > 0.1) else 1.2, 0.06, 0.01, 0.01, camw)
        if b[0] < 0.75 * s0 and np.mean(look(Pw + np.array([b[1], b[2]]), TRc) < 0.15) > 0.45: cur = cur + np.array([b[1], b[2]]); relock += 1; P = np.vstack([wp(i, cur) for i in win]); P = P[rng.choice(len(P), min(len(P), 4000), replace=False)]; reach = np.mean(look(P, TRc) < TRc)
    if reach > 0.25:
        b = cost_grid(P, a_, n_, 0.12, 0.03, 0.06, 0.004, base_xy[[j]] + cur); cur = cur + b[1] * a_ + b[2] * n_
    cj[j] = cur; fracs[j] = np.mean(look(np.vstack([wp(i, cur) for i in win]), TRc) < 0.1)
cjs = np.c_[ndimage.uniform_filter1d(ndimage.median_filter(cj[:, 0], 5, mode='nearest'), 3, mode='nearest'), ndimage.uniform_filter1d(ndimage.median_filter(cj[:, 1], 5, mode='nearest'), 3, mode='nearest')]
pf = base_xy + cjs; yawf = yawc + kq * np.pi / 2
allp = np.vstack([wallpts(j) @ Rk.T + Tg + cjs[j] for j in range(N) if len(Wl[j])]); dd = look(allp, 2.0); cov = dd < 0.5
outs = outside(pf) if FREE_W > 0 else np.zeros(N); print(f'카메라 위치가 통로 밖인 키프레임 {int((outs > 0.1).sum())}/{N} (밖으로 나간 거리 최대 {outs.max():.2f}m)')
print(f'전역 맞춤: 회전 {kq * 90}° 이동 ({Tg[0]:+.1f},{Tg[1]:+.1f}) | 따라가기 보정 범위 x {cjs[:, 0].min():+.2f}~{cjs[:, 0].max():+.2f} y {cjs[:, 1].min():+.2f}~{cjs[:, 1].max():+.2f}m, 넓은 재탐색 {relock}회')
print(f'벽 점 중 기준 맵 벽 50cm 안 {cov.mean():.0%} | 그 안에서 5cm 안 {np.mean(dd[cov] < 0.05):.0%}, 10cm 안 {np.mean(dd[cov] < 0.10):.0%}, 중앙 {np.median(dd[cov]) * 100:.1f}cm | 키프레임별 10cm 안 비율 중앙 {np.median(fracs[has]):.0%}')
rep = dict(run=run, nodes=int(N), keyframes=int(has.sum()), height_fits=int(vq.sum()), ceil_fits=int(okc.sum()), tilt_err_med_deg=float(np.degrees(np.median(np.linalg.norm(qf, axis=1)))), cam_height_m=float(np.median(hs)), pitch_med_deg=float(np.median(pitch[has])),
           rot_deg=int(kq * 90), shift=[float(Tg[0]), float(Tg[1])], relock=int(relock), ref_cover=float(cov.mean()), within5=float(np.mean(dd[cov] < 0.05)), within10=float(np.mean(dd[cov] < 0.10)), med_cm=float(np.median(dd[cov]) * 100))
# 그림
X0 = np.minimum(lo + 10, allp.min(0) - 1); X1 = np.maximum(lo + n * CELL - 10, allp.max(0) + 1); sz = ((X1 - X0) / 0.05).astype(int) + 1; img = np.full((sz[1], sz[0], 3), 255, np.uint8)
ro = np.argwhere(occ) * CELL + lo
for P_, col in ((ro, (170, 170, 170)), (allp, (0, 0, 255))):
    u = ((P_[:, 0] - X0[0]) / 0.05).astype(int); v = sz[1] - 1 - ((P_[:, 1] - X0[1]) / 0.05).astype(int); okk = (u >= 0) & (u < sz[0]) & (v >= 0) & (v < sz[1]); img[v[okk], u[okk]] = col
qd = np.c_[((pf[:, 0] - X0[0]) / 0.05), sz[1] - 1 - ((pf[:, 1] - X0[1]) / 0.05)].astype(np.int32); cv2.polylines(img, [qd.reshape(-1, 1, 2)], False, (0, 160, 0), 1)
for k_ in range(0, N, 25): cv2.putText(img, str(int(stamps[k_] - stamps[0])), tuple(qd[k_] + 3), 0, 0.35, (0, 120, 0), 1)
for xg in np.arange(np.ceil(X0[0] / 5) * 5, X1[0], 5): cv2.putText(img, f'{xg:.0f}', (int((xg - X0[0]) / 0.05), 12), 0, 0.4, (0, 120, 255), 1)
cv2.imwrite(str(O / f'{run}_localize.png'), img)
# 내보내기(자료 있는 키프레임만)
F = E / f'{run}L'; F.mkdir(exist_ok=True)
if not (F / 'keyframes').exists(): os.symlink(K, F / 'keyframes')
Rf = Rot.from_euler('z', yawf[:, None]).as_matrix() @ R1; hh = np.where(has)[0]
np.savetxt(F / 'traj_camera_id_camera_poses.txt', np.c_[stamps[hh], pf[hh], hs[hh], Rot.from_matrix(Rf[hh]).as_quat(), ids[hh]], fmt=['%.6f'] * 8 + ['%d'], header='timestamp x y z qx qy qz qw id (건물 좌표, 바닥 z=0, 기준 맵에 맞춤)')
TLOi = np.linalg.inv(TLO); TF = np.tile(np.eye(4), (N, 1, 1)); TF[:, :3, :3] = Rf; TF[:, :2, 3] = pf; TF[:, 2, 3] = hs; TLf = TF @ TLOi
np.savetxt(F / 'traj_optimized_poses.txt', np.c_[stamps[hh], TLf[hh, :3, 3], Rot.from_matrix(TLf[hh, :3, :3]).as_quat()], fmt='%.6f', header='timestamp x y z qx qy qz qw (camera_link, 건물 좌표)')
np.savez(O / f'{run}_localize.npz', ids=ids, has=has, pos=pf, yaw=yawf, h=hs, cj=cjs, frac=fracs, psi=psi_s); json.dump(rep, open(O / f'{run}_localize.json', 'w'), indent=1); print('→', F)
