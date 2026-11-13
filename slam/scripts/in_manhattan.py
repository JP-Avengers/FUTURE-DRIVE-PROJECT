"""실내 복도 회차 보정 (건물 벽은 직각·직선이라는 가정):
 (1) 바닥 수평: 키프레임마다 자기 depth의 바닥 높이 → z=0
 (2) 나침반: 키프레임마다 벽 점의 주 방향(선 맞춤 점수 최대 각, 90° 주기)을 구해 맵 요 흐름 ψ 를 재고, 요를 바로잡아 궤적을 다시 적분
 (3) 건물 좌표: 복도 축 = x
 (4) [--snap] 좌우: 키프레임마다 양쪽 벽 선의 y 를 재서 구간(x 2m 칸)별 기준 벽 선에 맞춤
 출력: exports/<run>m/ (보정 자세·keyframes 링크), <out>/<run>_manhattan.{json,npz,png}
사용: in_manhattan.py <run> <out_dir> [--maxd 4.0] [--snap]"""
import sys, re, json, os, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, O = a[1], Path(a[2]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: a[a.index(k) + 1] if k in a else d
MAXD = float(opt('--maxd', 4.0)); STEP = 4; CELL = 0.03
E = Path.home() / 'b_slam/exports'; K = E / run / 'keyframes'
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); cid = cid[np.argsort(cid[:, 0])]; N = len(cid)
def calib(i): return [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
pts = []
for r in cid:
    i = int(r[8]); k = calib(i); d = cv2.imread(str(K / f'kf_depth/{i}.png'), -1)
    v, u = np.mgrid[STEP // 2:d.shape[0]:STEP, STEP // 2:d.shape[1]:STEP]; z = d[v, u].astype(np.float32) / 1000; m = (z > 0.4) & (z < MAXD)
    pc = np.stack([(u[m] - k[2]) / k[0] * z[m], (v[m] - k[5]) / k[4] * z[m], z[m]], 1)
    pts.append((pc @ Rot.from_quat(r[4:8]).as_matrix().T + r[1:4]).astype(np.float32))
# (1) 바닥
cz = cid[:, 3]; f = np.full(N, np.nan)
for j, P in enumerate(pts):
    c = P[P[:, 2] < cz[j] - 0.6]
    if len(c) < 300: continue
    h, e = np.histogram(c[:, 2], bins=np.arange(c[:, 2].min(), cz[j] - 0.6 + 0.02, 0.02))
    if len(h): pk = e[np.argmax(h)] + 0.01; f[j] = np.median(c[np.abs(c[:, 2] - pk) < 0.05, 2])
ok = np.isfinite(f); f = np.interp(np.arange(N), np.where(ok)[0], f[ok]); fs = ndimage.median_filter(f, 9, mode='nearest'); camh = cz - fs
c2 = cid[:, 1:3].copy()
Wl = []                                            # 키프레임별 벽 점(카메라 기준 2D, 맵 축)
for j, P in enumerate(pts):
    h = P[:, 2] - fs[j]; q = P[(h > 0.3) & (h < 1.9), :2] - c2[j].astype(np.float32); Wl.append((np.unique(np.round(q / CELL).astype(np.int32), axis=0) * CELL).astype(np.float32))
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
phi0 = np.degrees(np.angle(np.exp(1j * np.radians(beta[okb] * 4)).mean())) / 4 % 90            # 90° 주기 평균
psi = (beta - phi0 + 45) % 90 - 45; psi[~okb] = np.nan; psi[np.abs(psi) > 15] = np.nan; v_ = np.isfinite(psi)
psi_f = np.interp(np.arange(N), np.where(v_)[0], psi[v_]); psi_s = ndimage.uniform_filter1d(ndimage.median_filter(psi_f, 7, mode='nearest'), 5, mode='nearest')
print(f'{run}: 키프레임 {N} | 바닥 출렁임 {np.ptp(fs):.2f}m, 카메라 높이 중앙 {np.median(camh):.2f}m | 벽 방향 잡힌 키프레임 {v_.sum()}/{N} | 맵 요 흐름 ψ: 범위 {psi_s.min():+.2f}~{psi_s.max():+.2f}°, 끝-처음 {psi_s[-1] - psi_s[0]:+.2f}°')
pr = np.radians(psi_s); p2 = np.zeros((N, 2)); p2[0] = c2[0]
for i in range(N - 1):
    t = -(pr[i] + pr[i + 1]) / 2; Rz = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]); p2[i + 1] = p2[i] + Rz @ (c2[i + 1] - c2[i])
# (3) 건물 좌표: 벽 주 방향 → x 축, 출발점 원점. 복도가 y 로 길면 90° 더 돌림
rot = -np.radians(phi0); Rb = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]]); pb = (p2 - p2[0]) @ Rb.T
if np.ptp(pb[:, 1]) > np.ptp(pb[:, 0]): rot -= np.pi / 2; Rb = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]]); pb = (p2 - p2[0]) @ Rb.T
yawc = rot - pr                                     # 키프레임별 맵→건물 회전(요)
def wallpts(j, y_shift=0.0):
    t = yawc[j]; Rz = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]); return Wl[j].astype(np.float64) @ Rz.T + pb[j] + np.array([0, y_shift])
# 키프레임별 양쪽 벽 선의 y (창 ±2 키프레임, 앞뒤 3m 안)
yL = np.full(N, np.nan); yR = np.full(N, np.nan)
for j in range(N):
    P = np.vstack([wallpts(i) for i in range(max(j - 2, 0), min(j + 3, N))]); P = P[np.abs(P[:, 0] - pb[j, 0]) < 3.0]
    if len(P) < 400: continue
    rel = P[:, 1] - pb[j, 1]; h, e = np.histogram(rel, bins=np.arange(-3.5, 3.52, 0.02)); h = ndimage.uniform_filter1d(h.astype(float), 3); ctr = (e[:-1] + e[1:]) / 2
    for side, m_ in (('L', ctr > 0.25), ('R', ctr < -0.25)):
        hh = np.where(m_, h, 0); b = int(np.argmax(hh))
        if hh[b] >= max(20, 0.02 * len(P)):
            if side == 'L': yL[j] = pb[j, 1] + ctr[b]
            else: yR[j] = pb[j, 1] + ctr[b]
wd = yL - yR; W0 = np.nanmedian(wd)
print(f'건물 좌표: 복도 범위 x {pb[:, 0].min():+.1f}~{pb[:, 0].max():+.1f}m, y {pb[:, 1].min():+.1f}~{pb[:, 1].max():+.1f}m | 양쪽 벽 잡힌 키프레임 {np.isfinite(wd).sum()}/{N}, 복도 폭 중앙 {W0:.2f}m (10~90% {np.nanpercentile(wd, 10):.2f}~{np.nanpercentile(wd, 90):.2f})')
dist0 = np.linalg.norm(pb - pb[0], axis=1); kt = int(np.argmax(dist0)); roundtrip = dist0[-1] < 0.35 * dist0[kt] and kt < N - 10
legs = [np.arange(0, kt + 1), np.arange(kt + 1, N)] if roundtrip else [np.arange(N)]
for li, L in enumerate(legs):
    for nm, yy in (('왼쪽 벽', yL), ('오른쪽 벽', yR)):
        g = [(lo_, np.nanmedian(yy[L][(pb[L, 0] >= lo_) & (pb[L, 0] < lo_ + 5)])) for lo_ in np.arange(np.floor(pb[:, 0].min() / 5) * 5, pb[:, 0].max(), 5) if np.isfinite(yy[L][(pb[L, 0] >= lo_) & (pb[L, 0] < lo_ + 5)]).sum() >= 3]
        print(f'  구간 {li + 1} {nm} y(x 5m 칸): ' + ' '.join(f'{lo_:.0f}:{v:+.2f}' for lo_, v in g))
dy = np.zeros(N)
if '--snap' in a:
    # 복도 벽은 직선 두 줄(폭 W0)이라고 보고, 기준 벽 선 YR·YL = 모든 관측(오른쪽 벽 y, 왼쪽 벽 y - W0)의 중앙값 → 키프레임별 좌우 보정
    both = np.isfinite(wd) & (np.abs(wd - W0) < 0.15); Wc = float(np.median(wd[both])) if both.sum() >= 5 else float(W0)
    obs = np.r_[yR[np.isfinite(yR)], yL[np.isfinite(yL)] - Wc]; YR = float(np.median(obs)); YL = YR + Wc
    eL, eR = YL - yL, YR - yR; eL[np.abs(eL) > 0.8] = np.nan; eR[np.abs(eR) > 0.8] = np.nan
    e1 = np.where(np.isfinite(eL) & np.isfinite(eR) & (np.abs(eL - eR) < 0.15), (eL + eR) / 2, np.nan)
    single = np.where(np.isfinite(eL) & ~np.isfinite(eR), eL, np.where(np.isfinite(eR) & ~np.isfinite(eL), eR, np.nan)); e1 = np.where(np.isfinite(e1), e1, single)
    for L in legs:
        ee = e1[L]; vv = np.isfinite(ee)
        if vv.sum() < 5: continue
        ef = ndimage.median_filter(np.interp(np.arange(len(L)), np.where(vv)[0], ee[vv]), 11, mode='nearest')
        for q_, j in enumerate(L):                                   # 두 벽이 서로 다른 값을 줄 때는 이웃 흐름에 가까운 쪽
            c_ = [x for x in (eL[j], eR[j]) if np.isfinite(x)]
            if len(c_) == 2 and abs(c_[0] - c_[1]) >= 0.15: k_ = int(np.argmin(np.abs(np.array(c_) - ef[q_]))); ee[q_] = c_[k_] if abs(c_[k_] - ef[q_]) < 0.25 else np.nan
            elif len(c_) and np.isfinite(ee[q_]) and abs(ee[q_] - ef[q_]) > 0.25: ee[q_] = np.nan
        vv = np.isfinite(ee); ef = np.interp(np.arange(len(L)), np.where(vv)[0], ee[vv]); dy[L] = ndimage.uniform_filter1d(ndimage.median_filter(ef, 11, mode='nearest'), 5, mode='nearest')
        print(f'  구간 좌우 보정: 벽으로 정한 키프레임 {vv.sum()}/{len(L)}, 보정량 {dy[L].min():+.2f}~{dy[L].max():+.2f}m')
    print(f'좌우 맞춤: 복도 폭 {Wc:.2f}m(양쪽 벽을 함께 본 키프레임 {both.sum()}장), 기준 벽 선 y = {YR:+.2f} · {YL:+.2f}m')
pf = pb + np.c_[np.zeros(N), dy]
rep = dict(run=run, keyframes=N, floor_range_m=float(np.ptp(fs)), cam_height_m=float(np.median(camh)), yaw_drift_range_deg=[float(psi_s.min()), float(psi_s.max())], width_m=float(W0), roundtrip=bool(roundtrip), snap='--snap' in a)
if roundtrip:
    allw = np.vstack([wallpts(j, dy[j]) for j in range(N)]); lo = allw.min(0) - 2; n = ((allw.max(0) + 2 - lo) / CELL).astype(int) + 1; cnt = np.zeros(n, np.uint16)
    for j in legs[0]: ij = np.unique(((wallpts(j, dy[j]) - lo) / CELL).astype(int), axis=0); cnt[ij[:, 0], ij[:, 1]] += 1
    D = ndimage.distance_transform_edt(cnt < 2) * CELL; Pm = np.vstack([wallpts(j, dy[j]) for j in legs[1]]); ij = ((Pm - lo) / CELL).astype(int); dd = np.minimum(D[ij[:, 0], ij[:, 1]], 1.0)
    print(f'오는 길 벽 점 → 가는 길 벽: 중앙 {np.median(dd) * 100:.1f}cm, 5cm 안 {np.mean(dd < 0.05):.0%}, 15cm 안 {np.mean(dd < 0.15):.0%} | 끝-출발 {np.linalg.norm(pf[-1] - pf[0]):.2f}m')
    rep.update(wall_med_cm=float(np.median(dd) * 100), within5=float(np.mean(dd < 0.05)), within15=float(np.mean(dd < 0.15)))
# 그림: 벽 점(가는 길 회색·오는 길 빨강) + 궤적
allw = [wallpts(j, dy[j]) for j in range(N)]; AW = np.vstack(allw); X0 = AW.min(0) - 1; sz = ((AW.max(0) + 1 - X0) / 0.05).astype(int) + 1; img = np.full((sz[1], sz[0], 3), 255, np.uint8)
for li, L in enumerate(legs):
    P_ = np.vstack([allw[j] for j in L]); u = ((P_[:, 0] - X0[0]) / 0.05).astype(int); v = sz[1] - 1 - ((P_[:, 1] - X0[1]) / 0.05).astype(int); img[v, u] = (150, 150, 150) if li == 0 else (0, 0, 255)
q = np.c_[((pf[:, 0] - X0[0]) / 0.05), sz[1] - 1 - ((pf[:, 1] - X0[1]) / 0.05)].astype(np.int32); cv2.polylines(img, [q.reshape(-1, 1, 2)], False, (0, 160, 0), 1)
for xg in np.arange(np.ceil(X0[0] / 5) * 5, X0[0] + sz[0] * 0.05, 5): cv2.putText(img, f'{xg:.0f}', (int((xg - X0[0]) / 0.05), 12), 0, 0.4, (0, 120, 255), 1)
cv2.imwrite(str(O / f'{run}_manhattan.png'), img)
# 보정된 내보내기 (건물 좌표, 바닥 z=0)
F = E / f'{run}m'; F.mkdir(exist_ok=True)
if not (F / 'keyframes').exists(): os.symlink(K, F / 'keyframes')
Rz3 = Rot.from_euler('z', yawc[:, None]).as_matrix(); R0 = Rot.from_quat(cid[:, 4:8]).as_matrix()
np.savetxt(F / 'traj_camera_id_camera_poses.txt', np.c_[cid[:, 0], pf, cz - fs, Rot.from_matrix(Rz3 @ R0).as_quat(), cid[:, 8]], fmt=['%.6f'] * 8 + ['%d'], header='timestamp x y z qx qy qz qw id (건물 좌표: 복도 축 x, 바닥 z=0)')
tr = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2); it = lambda x: np.interp(tr[:, 0], cid[:, 0], x)
yl = it(np.unwrap(yawc)); RzL = Rot.from_euler('z', yl[:, None]).as_matrix(); cc = np.c_[it(c2[:, 0]), it(c2[:, 1])]; pfl = np.c_[it(pf[:, 0]), it(pf[:, 1])]
pl = np.c_[pfl + np.einsum('nij,nj->ni', RzL[:, :2, :2], tr[:, 1:3] - cc), tr[:, 3] - it(fs)]
np.savetxt(F / 'traj_optimized_poses.txt', np.c_[tr[:, 0], pl, Rot.from_matrix(RzL @ Rot.from_quat(tr[:, 4:8]).as_matrix()).as_quat()], fmt='%.6f', header='timestamp x y z qx qy qz qw (camera_link, 건물 좌표, 바닥 z=0)')
np.savez(O / f'{run}_manhattan.npz', psi=psi_s, yawc=yawc, pos=pf, floor=fs, yL=yL, yR=yR, dy=dy, ids=cid[:, 8]); json.dump(rep, open(O / f'{run}_manhattan.json', 'w'), indent=1); print('→', F)
