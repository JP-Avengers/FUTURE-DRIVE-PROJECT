"""실내 회차를 기준 복도 맵(건물 좌표, in_manhattan.py 출력)에 맞춰 넣기 — RTAB-Map 맵이 조각나거나 높이가 흐른 회차용.
 입력은 DB의 노드별 오도메트리 자세(끊긴 조각 포함 전체)와 키프레임 depth.
 (1) 중력: 키프레임마다 depth에서 바닥 평면(RANSAC)을 찾아 기울기(롤·피치)와 카메라 높이를 정하고, 오도메트리 걸음을 다시 적분
 (2) 나침반: 벽 점 주 방향(90° 주기)으로 요 흐름을 바로잡아 다시 적분 → 벽이 축에 나란한 좌표
 (3) 전역 맞춤: 90° 단위 회전 4가지 × 이동을 기준 맵 벽 격자와의 겹침으로 탐색
 (4) 따라가기: 키프레임 순서대로 기준 벽 거리 지도에 이동만 미세 탐색(요는 나침반 값 고정). 벽 방향(앞뒤)으로는 벌점. 벽이 안 맞으면 넓게 다시 탐색
 출력: exports/<run>L/ (건물 좌표 자세·keyframes 링크), <out>/<run>_localize.{json,png}
사용: in_localize.py <run> <ref_run_m> <out_dir> [--raw in2] [--hmax 2.3]"""
import sys, re, json, os, sqlite3, struct, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, refrun, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: a[a.index(k) + 1] if k in a else d
RAW = opt('--raw', refrun.rstrip('m')); HMAX = float(opt('--hmax', 2.3)); CELL = 0.03; E = Path.home() / 'b_slam/exports'; K = E / run / 'keyframes'
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
# (1) 바닥 평면 → 중력
rng = np.random.default_rng(0); HLO, HHI = float(opt('--hlo', 1.0)), float(opt('--hhi', 1.45))      # 카메라 높이 허용 범위(손에 든 높이)
def wall_fit(P, up, thr=0.5):
    """카메라 좌표 점에서 가장 큰 세로 평면(법선·up 내적이 thr 미만) → (법선, 점 비율, 정면도=|법선·광축|). 없으면 None"""
    if P is None or len(P) < 800: return None
    S = P[rng.choice(len(P), min(len(P), 5000), replace=False)]; tri = S[rng.integers(0, len(S), (300, 3))]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); nn = np.linalg.norm(n, axis=1); ok = nn > 1e-6; n[ok] /= nn[ok, None]; ok &= np.abs(n @ up) < thr
    if not ok.any(): return None
    n = n[ok]; d = -(n * tri[ok, 0]).sum(1); cnt = (np.abs(S @ n.T + d) < (0.02 + 0.01 * S[:, 2:3])).sum(0); b = int(np.argmax(cnt))
    if cnt[b] < 0.2 * len(S): return None
    inl = P[np.abs(P @ n[b] + d[b]) < (0.02 + 0.01 * P[:, 2])]; c = inl.mean(0); _, sv, Vt = np.linalg.svd(inl - c, full_matrices=False); return Vt[2], cnt[b] / len(S), abs(Vt[2, 2])
def floor_fit(P):
    if P is None or len(P) < 800: return None
    S = P[rng.choice(len(P), min(len(P), 6000), replace=False)]; tri = S[rng.integers(0, len(S), (400, 3))]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); nn = np.linalg.norm(n, axis=1); ok = nn > 1e-6; n[ok] /= nn[ok, None]; n[n[:, 1] > 0] *= -1          # 위 = -y 쪽
    h = -(n * tri[:, 0]).sum(1); ok &= (-n[:, 1] > np.cos(np.radians(42))) & (h > HLO) & (h < HHI)
    if not ok.any(): return None
    n, h = n[ok], h[ok]; dist = np.abs(S @ n.T + h); cnt = (dist < (0.02 + 0.01 * S[:, 2:3])).sum(0); b = int(np.argmax(cnt))
    if cnt[b] < max(250, 0.05 * len(S)): return None
    for _ in range(2):
        inl = P[np.abs(P @ n[b] + h[b]) < (0.02 + 0.01 * P[:, 2])] if _ == 0 else inl2
        c = inl.mean(0); _, sv, Vt = np.linalg.svd(inl - c, full_matrices=False); nb = Vt[2] * (-1 if Vt[2, 1] > 0 else 1); hb = -(nb @ c)
        inl2 = P[np.abs(P @ nb + hb) < (0.015 + 0.008 * P[:, 2])]
        if len(inl2) < 250: return None
    if sv[1] / np.sqrt(len(inl)) < 0.12: return None                         # 바닥 조각이 너무 가늘면 버림
    return nb, hb, inl2
# 키프레임마다 위 방향(카메라 좌표): 바닥 법선. 벽을 정면으로 크게 보는 키프레임(벽 점 35% 이상, 법선·광축 0.5 넘음)은 바닥 조각이 가늘어
# 벽 쪽으로 기우는 성분을 믿을 수 없으므로, 바닥 법선에서 벽 법선 성분을 빼서(벽 = 수직) 씀. 바닥이 없고 벽만 있으면 이웃 기울기를 보간한 뒤 벽으로 바로잡음.
q = np.full((N, 2), np.nan); hc = np.full(N, np.nan); src = np.zeros(N, int); WN = [None] * N; wt = []
for j in np.where(has)[0]:
    fr = floor_fit(PC[j]); wr = wall_fit(PC[j], fr[0] if fr is not None else np.array([0, -0.94, -0.34]), 0.5 if fr is not None else 0.64)
    strong = wr is not None and wr[1] >= 0.35 and wr[2] > 0.5
    if fr is not None:
        u = fr[0]; h = fr[1]; src[j] = 1
        if strong:
            wt.append(np.degrees(np.arcsin(min(abs(u @ wr[0]), 1.0)))); u = u - (u @ wr[0]) * wr[0]; u /= np.linalg.norm(u); h = float(np.median(-(fr[2] @ u))); src[j] = 2
            if not (0.9 < h < 1.6): src[j] = 3; WN[j] = wr[0]; continue
        up = R0[j] @ u; ax = np.cross(up, [0, 0, 1.0]); s = np.linalg.norm(ax); ang = np.arctan2(s, up[2]); q[j] = ax[:2] / s * ang if s > 1e-9 else 0; hc[j] = h
    elif strong: src[j] = 3; WN[j] = wr[0]
vq = np.isfinite(hc); idx = np.where(vq)[0]
for col in range(2): q[idx, col] = ndimage.median_filter(q[idx, col], 3, mode='nearest')
hc[idx] = ndimage.median_filter(hc[idx], 5, mode='nearest'); ar = np.arange(N); qf = np.c_[np.interp(ar, idx, q[idx, 0]), np.interp(ar, idx, q[idx, 1])]; hs = np.interp(ar, idx, hc[idx])
Q = Rot.from_rotvec(np.c_[qf, np.zeros(N)]).as_matrix(); R1 = Q @ R0; nfix = 0
for j in np.where(src == 3)[0]:
    Nw = R1[j] @ WN[j]; Nh = np.array([Nw[0], Nw[1], 0.0]); Nh /= np.linalg.norm(Nh); ax = np.cross(Nw, Nh); s = np.linalg.norm(ax)
    if s > 1e-9: Qf = Rot.from_rotvec(ax / s * np.arcsin(min(s, 1.0))).as_matrix(); R1[j] = Qf @ R1[j]; Q[j] = Qf @ Q[j]; nfix += 1
print(f'기울기 출처: 바닥 {int((src == 1).sum())}장, 바닥+벽 {int((src == 2).sum())}장(바닥 법선이 벽 쪽으로 기운 각 중앙 {np.median(wt) if wt else float("nan"):.1f}°), 벽만 {nfix}장, 보간 {int((has & (src == 0)).sum())}장')
p1 = np.zeros((N, 3))
for j in range(N - 1): p1[j + 1] = p1[j] + Q[j] @ (p0[j + 1] - p0[j])
print(f'{run}: 노드 {N}(키프레임 자료 {has.sum()}) | 바닥 잡힌 키프레임 {vq.sum()} | 오도메트리 기울기 오차 중앙 {np.degrees(np.median(np.linalg.norm(qf, axis=1))):.1f}° 최대 {np.degrees(np.linalg.norm(qf, axis=1).max()):.1f}° | 카메라 높이 중앙 {np.median(hs):.2f}m (5~95% {np.percentile(hs, 5):.2f}~{np.percentile(hs, 95):.2f})'
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
def cost_grid(P, a_, n_, rng_, step, la, ln):
    best = (1e9, 0.0, 0.0)
    for da in np.arange(-rng_, rng_ + 1e-6, step):
        for dn in np.arange(-rng_, rng_ + 1e-6, step):
            s = float(look(P + da * a_ + dn * n_, TRc).mean()) + la * abs(da) + ln * abs(dn)
            if s < best[0]: best = (s, da, dn)
    return best
for j in range(N):
    win = [i for i in range(max(j - 2, 0), min(j + 3, N)) if len(Wl[i])]
    if not win: cj[j] = cur; continue
    P = np.vstack([wp(i, cur) for i in win])
    if len(P) > 4000: P = P[rng.choice(len(P), 4000, replace=False)]
    al = np.radians((alpha[j] if np.isfinite(alpha[j]) else 0.0)) + yawc[j] + kq * np.pi / 2; a_ = np.array([np.cos(al), np.sin(al)]); n_ = np.array([-a_[1], a_[0]])
    d0 = look(P, TRc); near = np.mean(d0 < 0.15); reach = np.mean(d0 < TRc)
    if j == 0 or (near < 0.35 and j % 3 == 0):                                    # 넓게 다시 탐색(처음, 또는 벽이 안 맞을 때)
        winw = [i for i in range(max(j - 6, 0), min(j + 7, N)) if len(Wl[i])]; Pw = np.vstack([wp(i, cur) for i in winw]); Pw = Pw[rng.choice(len(Pw), min(len(Pw), 6000), replace=False)]
        s0 = float(look(Pw, TRc).mean()); b = cost_grid(Pw, np.array([1.0, 0]), np.array([0, 1.0]), 2.4 if j == 0 else 1.2, 0.06, 0.01, 0.01)
        if b[0] < 0.75 * s0 and np.mean(look(Pw + np.array([b[1], b[2]]), TRc) < 0.15) > 0.45: cur = cur + np.array([b[1], b[2]]); relock += 1; P = np.vstack([wp(i, cur) for i in win]); P = P[rng.choice(len(P), min(len(P), 4000), replace=False)]; reach = np.mean(look(P, TRc) < TRc)
    if reach > 0.25:
        b = cost_grid(P, a_, n_, 0.12, 0.03, 0.06, 0.004); cur = cur + b[1] * a_ + b[2] * n_
    cj[j] = cur; fracs[j] = np.mean(look(np.vstack([wp(i, cur) for i in win]), TRc) < 0.1)
cjs = np.c_[ndimage.uniform_filter1d(ndimage.median_filter(cj[:, 0], 5, mode='nearest'), 3, mode='nearest'), ndimage.uniform_filter1d(ndimage.median_filter(cj[:, 1], 5, mode='nearest'), 3, mode='nearest')]
pf = base_xy + cjs; yawf = yawc + kq * np.pi / 2
allp = np.vstack([wallpts(j) @ Rk.T + Tg + cjs[j] for j in range(N) if len(Wl[j])]); dd = look(allp, 2.0); cov = dd < 0.5
print(f'전역 맞춤: 회전 {kq * 90}° 이동 ({Tg[0]:+.1f},{Tg[1]:+.1f}) | 따라가기 보정 범위 x {cjs[:, 0].min():+.2f}~{cjs[:, 0].max():+.2f} y {cjs[:, 1].min():+.2f}~{cjs[:, 1].max():+.2f}m, 넓은 재탐색 {relock}회')
print(f'벽 점 중 기준 맵 벽 50cm 안 {cov.mean():.0%} | 그 안에서 5cm 안 {np.mean(dd[cov] < 0.05):.0%}, 10cm 안 {np.mean(dd[cov] < 0.10):.0%}, 중앙 {np.median(dd[cov]) * 100:.1f}cm | 키프레임별 10cm 안 비율 중앙 {np.median(fracs[has]):.0%}')
rep = dict(run=run, nodes=int(N), keyframes=int(has.sum()), floor_fits=int(vq.sum()), tilt_err_med_deg=float(np.degrees(np.median(np.linalg.norm(qf, axis=1)))), cam_height_m=float(np.median(hs)), pitch_med_deg=float(np.median(pitch[has])),
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
