"""실내 회차 자기 정합 (복도 한 층):
 (1) 바닥 수평 맞춤 — 키프레임마다 자기 depth에서 바닥 높이(카메라보다 0.6m 넘게 낮은 점의 z 히스토그램 최빈)를 구해 바닥이 z=0 이 되게.
 (2) 왕복 회차 — 가는 길(기준) 벽으로 2D 거리 지도를 만들고, 오는 길 키프레임을 반환점부터 차례로 벽에 맞춤(요·좌우·앞뒤 미세 탐색을 누적).
     복도 방향(앞뒤)은 벽만으로는 정해지지 않으므로 벌점을 줘 오도메트리 값을 유지(문·로비처럼 구조가 있을 때만 움직임).
 출력: exports/<run>s/ (보정된 traj_camera_id_camera_poses.txt · traj_optimized_poses.txt · keyframes 링크), <out>/selfalign.json · 위에서 본 전후 그림
사용: in_selfalign.py <run> <out_dir> [--maxd 4.0] [--oneway]"""
import sys, re, json, os, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, O = a[1], Path(a[2]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: a[a.index(k) + 1] if k in a else d
MAXD = float(opt('--maxd', 4.0)); STEP = 4; CELL = 0.03; TR = 0.3
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
cz = cid[:, 3]; f = np.full(N, np.nan); tilt = np.full(N, np.nan)
for j, P in enumerate(pts):
    c = P[P[:, 2] < cz[j] - 0.6]
    if len(c) < 300: continue
    h, e = np.histogram(c[:, 2], bins=np.arange(c[:, 2].min(), cz[j] - 0.6 + 0.02, 0.02))
    if len(h) == 0: continue
    pk = e[np.argmax(h)] + 0.01; fl = c[np.abs(c[:, 2] - pk) < 0.05]; f[j] = np.median(fl[:, 2])
    if len(fl) > 500:
        A = np.c_[fl[:, :2] - fl[:, :2].mean(0), np.ones(len(fl))]; co = np.linalg.lstsq(A, fl[:, 2], rcond=None)[0]; tilt[j] = np.degrees(np.arctan(np.hypot(co[0], co[1])))
ok = np.isfinite(f); f = np.interp(np.arange(N), np.where(ok)[0], f[ok]); fs = ndimage.median_filter(f, 9, mode='nearest')
camh = cz - fs
print(f'{run}: 키프레임 {N} | 바닥 높이(맵 z) 범위 {fs.min():+.2f}~{fs.max():+.2f}m (수평 맞춤 전 출렁임 {np.ptp(fs):.2f}m) | 카메라 높이 중앙 {np.median(camh):.2f}m (5~95% {np.percentile(camh, 5):.2f}~{np.percentile(camh, 95):.2f})'
      f' | 키프레임 바닥 기울기 중앙 {np.nanmedian(tilt):.2f}° 95% {np.nanpercentile(tilt, 95):.2f}°')
W = []
for j, P in enumerate(pts):
    h = P[:, 2] - fs[j]; q = P[(h > 0.3) & (h < 1.9), :2]; W.append((np.unique(np.round(q / CELL).astype(np.int32), axis=0) * CELL).astype(np.float32))
c2 = cid[:, 1:3].copy(); dist0 = np.linalg.norm(c2 - c2[0], axis=1); kt = int(np.argmax(dist0))
roundtrip = ('--oneway' not in a) and dist0[-1] < 0.35 * dist0[kt] and kt < N - 10
yaw_s = np.zeros(N); dxy_s = np.zeros((N, 2)); rep = dict(run=run, keyframes=N, floor_range_m=float(np.ptp(fs)), cam_height_m=float(np.median(camh)), roundtrip=bool(roundtrip))
allw = np.vstack(W); lo = allw.min(0) - 3; n = ((allw.max(0) + 3 - lo) / CELL).astype(int) + 1
def field(idx, mincnt=2):
    cnt = np.zeros(n, np.uint16)
    for j in idx: ij = ((W[j] - lo) / CELL).astype(int); cnt[ij[:, 0], ij[:, 1]] += 1
    return ndimage.distance_transform_edt(cnt < mincnt) * CELL
def look(D, P, cap):
    ij = ((P - lo) / CELL).astype(int); okk = (ij >= 0).all(1) & (ij < n).all(1); out = np.full(len(P), cap, np.float32); out[okk] = np.minimum(D[ij[okk, 0], ij[okk, 1]], cap); return out
if roundtrip:
    ref = np.arange(0, kt + 1); mov = np.arange(kt + 1, N); Dfull = field(ref); D = np.minimum(Dfull, TR).astype(np.float32)
    print(f'왕복: 반환점 키프레임 {kt} (출발점에서 {dist0[kt]:.1f}m), 가는 길 {len(ref)}장 · 오는 길 {len(mov)}장, 끝-출발 {dist0[-1]:.2f}m')
    ths = np.radians(np.arange(-0.8, 0.81, 0.2)); das = np.arange(-0.12, 0.121, 0.03); dns = np.arange(-0.12, 0.121, 0.03); LA, LN, LT = 0.05, 0.01, 0.002
    rng = np.random.default_rng(0); Gp = np.eye(3); G = [np.eye(3)] * N; movset = set(mov.tolist())
    for j in mov:
        win = [i for i in range(j - 2, j + 3) if i in movset]; P = np.vstack([W[i] for i in win]).astype(np.float64)
        if len(P) > 4000: P = P[rng.choice(len(P), 4000, replace=False)]
        P = P @ Gp[:2, :2].T + Gp[:2, 2]; c = Gp[:2, :2] @ c2[j] + Gp[:2, 2]
        av = Gp[:2, :2] @ (c2[min(j + 1, N - 1)] - c2[max(j - 1, 0)]); nav = np.linalg.norm(av); av = av / nav if nav > 1e-3 else np.array([1.0, 0]); nv = np.array([-av[1], av[0]])
        best = (1e9, 0.0, 0.0, 0.0)
        for th in ths:
            Rz = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); Pr = (P - c) @ Rz.T + c
            for da in das:
                for dn in dns:
                    s = float(look(D, Pr + da * av + dn * nv, TR).mean()) + LA * abs(da) + LN * abs(dn) + LT * abs(np.degrees(th))
                    if s < best[0]: best = (s, th, da, dn)
        _, th, da, dn = best; Rz = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); M = np.eye(3); M[:2, :2] = Rz; M[:2, 2] = c - Rz @ c + da * av + dn * nv
        if '--trace' in a and (j - kt) % 4 == 1:
            d0_ = look(D, P, TR); print(f'  #{j} 점 {len(P)} 벽 30cm 안 {np.mean(d0_ < TR):.0%} | 비용 현재 {d0_.mean():.4f} → 최고 {best[0]:.4f} | 요 {np.degrees(th):+.1f}° 앞뒤 {da:+.2f} 좌우 {dn:+.2f} | 누적 요 {np.degrees(np.arctan2(Gp[1, 0], Gp[0, 0])):+.2f}°')
        Gp = M @ Gp; G[j] = Gp
    yaw = np.array([np.arctan2(g[1, 0], g[0, 0]) for g in G]); dxy = np.array([g[:2, :2] @ c2[j] + g[:2, 2] - c2[j] for j, g in enumerate(G)])
    sm = lambda x: ndimage.uniform_filter1d(ndimage.median_filter(x, 5, mode='nearest'), 5, mode='nearest')
    yaw_s[mov] = sm(yaw[mov]); dxy_s[mov, 0] = sm(dxy[mov, 0]); dxy_s[mov, 1] = sm(dxy[mov, 1])
    def movpts(corr):
        out = []
        for j in mov:
            if corr: Rz = np.array([[np.cos(yaw_s[j]), -np.sin(yaw_s[j])], [np.sin(yaw_s[j]), np.cos(yaw_s[j])]]); out.append((W[j] - c2[j]) @ Rz.T + c2[j] + dxy_s[j])
            else: out.append(W[j])
        return np.vstack(out)
    b0, b1 = look(Dfull, movpts(False), 1.0), look(Dfull, movpts(True), 1.0)
    print(f'오는 길 벽 점 → 가는 길 벽까지 거리: 전 중앙 {np.median(b0) * 100:.1f}cm, 5cm 안 {np.mean(b0 < 0.05):.0%}, 15cm 안 {np.mean(b0 < 0.15):.0%} → 후 중앙 {np.median(b1) * 100:.1f}cm, 5cm 안 {np.mean(b1 < 0.05):.0%}, 15cm 안 {np.mean(b1 < 0.15):.0%}')
    print(f'보정량: 끝 키프레임 이동 {np.linalg.norm(dxy_s[-1]):.2f}m, 요 {np.degrees(yaw_s[-1]):+.2f}° | 최대 이동 {np.linalg.norm(dxy_s, axis=1).max():.2f}m')
    rep.update(turn_kf=kt, wall_before_med_cm=float(np.median(b0) * 100), wall_after_med_cm=float(np.median(b1) * 100), within5_before=float(np.mean(b0 < 0.05)), within5_after=float(np.mean(b1 < 0.05)),
               end_shift_m=float(np.linalg.norm(dxy_s[-1])), end_yaw_deg=float(np.degrees(yaw_s[-1])))
    X0 = allw.min(0) - 1; sz = ((allw.max(0) + 1 - X0) / 0.05).astype(int) + 1
    def draw(Pm):
        img = np.full((sz[1], sz[0], 3), 255, np.uint8)
        for P_, col in ((np.vstack([W[j] for j in ref]), (150, 150, 150)), (Pm, (0, 0, 255))):
            u = ((P_[:, 0] - X0[0]) / 0.05).astype(int); v = sz[1] - 1 - ((P_[:, 1] - X0[1]) / 0.05).astype(int); okk = (u >= 0) & (u < sz[0]) & (v >= 0) & (v < sz[1]); img[v[okk], u[okk]] = col
        return img
    cv2.imwrite(str(O / f'{run}_selfalign_top.png'), np.vstack([draw(movpts(False)), np.full((6, sz[0], 3), 0, np.uint8), draw(movpts(True))]))
else: print('왕복 아님(또는 --oneway): 바닥 수평 맞춤만')
# 보정된 내보내기
F = E / f'{run}s'; F.mkdir(exist_ok=True)
if not (F / 'keyframes').exists(): os.symlink(K, F / 'keyframes')
Rz3 = Rot.from_euler('z', yaw_s[:, None]).as_matrix(); R0 = Rot.from_quat(cid[:, 4:8]).as_matrix(); pos = np.c_[c2 + dxy_s, cz - fs]
np.savetxt(F / 'traj_camera_id_camera_poses.txt', np.c_[cid[:, 0], pos, Rot.from_matrix(Rz3 @ R0).as_quat(), cid[:, 8]], fmt=['%.6f'] * 8 + ['%d'], header='timestamp x y z qx qy qz qw id (바닥 z=0, 자기 정합)')
tr = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2); it = lambda x: np.interp(tr[:, 0], cid[:, 0], x)
ys = it(yaw_s); ds = np.c_[it(dxy_s[:, 0]), it(dxy_s[:, 1])]; cc = np.c_[it(c2[:, 0]), it(c2[:, 1])]; fz = it(fs)
RzL = Rot.from_euler('z', ys[:, None]).as_matrix(); rel = tr[:, 1:3] - cc; pl = np.c_[cc + ds + np.einsum('nij,nj->ni', RzL[:, :2, :2], rel), tr[:, 3] - fz]
np.savetxt(F / 'traj_optimized_poses.txt', np.c_[tr[:, 0], pl, Rot.from_matrix(RzL @ Rot.from_quat(tr[:, 4:8]).as_matrix()).as_quat()], fmt='%.6f', header='timestamp x y z qx qy qz qw (camera_link, 바닥 z=0, 자기 정합)')
np.savez(O / f'{run}_selfalign.npz', yaw=yaw_s, dxy=dxy_s, floor=fs, ids=cid[:, 8]); json.dump(rep, open(O / f'{run}_selfalign.json', 'w'), indent=1); print('→', F)
