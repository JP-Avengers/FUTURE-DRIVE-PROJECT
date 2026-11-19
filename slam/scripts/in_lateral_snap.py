"""보조 묶음 좌우 미세 맞춤: 키프레임마다 벽 점(복도 구간, 높이 0.3~2.6m)을 주 묶음 벽면(x 1m 구간별 중앙 y)에 연속값으로 재서,
벽면에서 ±0.10m 안 점의 어긋남 중앙만큼 키프레임을 y 방향으로 옮김(이웃 5장 중앙으로 부드럽게, ±0.08m 제한).
양쪽 벽을 함께 본 키프레임에서 두 벽 어긋남이 반대 부호(둘 다 바깥/안쪽)면 그 차이는 이동으로 못 고치는 성분(depth 축척 등)이라 따로 기록.
사용: in_lateral_snap.py <run(예 inrwbL)> <ref_mesh(주 묶음만, A_in2.ply)> <out_run(예 inrwbLS)> [--qmin 0.5] [--excl id1,id2,...]"""
import sys, re, os, numpy as np, cv2, open3d as o3d
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, refm, outr = a[1], a[2], a[3]; opt = lambda k, d: a[a.index(k) + 1] if k in a else d
QMIN = float(opt('--qmin', 0.5)); EXCL = set(int(x) for x in opt('--excl', '').split(',') if x); E = Path.home() / 'b_slam/exports'; K = E / run / 'keyframes'
Y0, Y1 = -1.10, 1.43; LOBBY = (18.0, 27.0)
# 주 묶음 벽면: x 1m 구간마다 벽 선 ±0.15m 안 꼭짓점(높이 0.4~1.2m)의 y 중앙
V = np.asarray(o3d.io.read_triangle_mesh(refm).vertices); V = V[(V[:, 2] > 0.4) & (V[:, 2] < 1.2)]
XB = np.arange(-2, 49); ref = {}
for nm, yl in (('R', Y0), ('L', Y1)):
    w = V[np.abs(V[:, 1] - yl) < 0.15]; b = np.floor(w[:, 0]).astype(int); yy = np.full(len(XB), np.nan)
    for k, x in enumerate(XB):
        s = w[b == x, 1]
        if len(s) >= 200: yy[k] = np.median(s)
    ok = np.isfinite(yy); ref[nm] = np.interp(XB, XB[ok], yy[ok]); print(f'주 묶음 {nm} 벽면 y: 중앙 {np.median(yy[ok]):+.3f}m, 구간 {ok.sum()}개, 범위 {np.nanmin(yy):+.3f}~{np.nanmax(yy):+.3f}')
def wall_y(nm, x): return np.interp(x, XB + 0.5, ref[nm])
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); cid = cid[np.argsort(cid[:, 0])]
f = E / 'indoor' / f'{run}_localize.npz'; f = f if f.exists() else E / 'indoor' / f'{run[:-1]}_localize.npz'; fr = {}
if QMIN > 0 and f.exists(): z = np.load(f); fr = {int(i): float(q) for i, q in zip(z['ids'], z['frac'])}
keep = np.array([(not fr or fr.get(int(i), 0) >= QMIN) and int(i) not in EXCL for i in cid[:, 8]]); cid = cid[keep]; N = len(cid)
dR = np.full(N, np.nan); dL = np.full(N, np.nan); nR = np.zeros(N, int); nL = np.zeros(N, int)
for j, r in enumerate(cid):
    i = int(r[8]); k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
    d = cv2.imread(str(K / f'kf_depth/{i}.png'), -1); v, u = np.mgrid[2:d.shape[0]:3, 2:d.shape[1]:3]; zz = d[v, u].astype(np.float32) / 1000; m = (zz > 0.4) & (zz < 3.0)
    P = np.stack([(u[m] - k[2]) / k[0] * zz[m], (v[m] - k[5]) / k[4] * zz[m], zz[m]], 1) @ Rot.from_quat(r[4:8]).as_matrix().T + r[1:4]
    P = P[(P[:, 2] > 0.3) & (P[:, 2] < 2.6) & (P[:, 0] > 0) & (P[:, 0] < 46) & ~((P[:, 0] > LOBBY[0]) & (P[:, 0] < LOBBY[1]) & (P[:, 1] > 0))]
    for nm, arr, cnt in (('R', dR, nR), ('L', dL, nL)):
        e = P[:, 1] - wall_y(nm, P[:, 0]); s = np.abs(e) < 0.10
        if s.sum() >= 300: arr[j] = np.median(e[s]); cnt[j] = s.sum()
# 이동량: 한쪽 벽만 보면 그 벽 어긋남을 없앰, 양쪽을 보면 평균(같은 방향 성분)만 없앰
sh = np.where(np.isfinite(dR) & np.isfinite(dL), -(dR + dL) / 2, np.where(np.isfinite(dR), -dR, np.where(np.isfinite(dL), -dL, np.nan)))
both = np.isfinite(dR) & np.isfinite(dL); spread = (dL - dR)[both]          # 양쪽 다 바깥이면 + (폭이 넓게 보임)
okk = np.isfinite(sh); idx = np.where(okk)[0]
if len(idx) == 0: raise SystemExit('벽을 본 키프레임이 없음')
shs = np.interp(np.arange(N), idx, ndimage.median_filter(sh[idx], 5, mode='nearest')); shs = np.clip(shs, -0.08, 0.08)
print(f'{run}: 키프레임 {N} | 벽 어긋남(점 - 주 묶음 벽면) R 벽 중앙 {np.nanmedian(dR) * 100:+.1f}cm ({np.isfinite(dR).sum()}장), L 벽 중앙 {np.nanmedian(dL) * 100:+.1f}cm ({np.isfinite(dL).sum()}장)'
      f' | 양쪽 함께 본 {both.sum()}장의 폭 차이(L-R 어긋남) 중앙 {np.median(spread) * 100 if len(spread) else float("nan"):+.1f}cm')
print(f'   좌우 이동량: 중앙 {np.median(shs) * 100:+.1f}cm, 5~95% {np.percentile(shs, 5) * 100:+.1f}~{np.percentile(shs, 95) * 100:+.1f}cm, ±8cm 제한 걸린 키프레임 {int((np.abs(shs) >= 0.0799).sum())}')
F = E / outr
if F.exists(): raise SystemExit(f'이미 있음: {F}')
F.mkdir(); os.symlink(os.path.realpath(K), F / 'keyframes'); out = cid.copy(); out[:, 2] += shs
np.savetxt(F / 'traj_camera_id_camera_poses.txt', out, fmt=['%.6f'] * 8 + ['%d'], header=f'timestamp x y z qx qy qz qw id (건물 좌표; {run} 에서 벽 일치 {QMIN:.0%} 이상, 좌우 미세 맞춤 in_lateral_snap.py)')
np.savez(E / 'indoor' / f'{outr}_snap.npz', ids=out[:, 8], dR=dR, dL=dL, shift=shs); print('→', F)
