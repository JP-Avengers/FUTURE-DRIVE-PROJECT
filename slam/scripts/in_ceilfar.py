"""바닥 기준 회차에서 멀리 찍힌 천장 높이 — 키프레임 depth 를 건물 좌표로 올려(자세 = 바닥 기준) 복도 안(y 범위) · z 2.2~3.4 m 점의 높이 분포를 봄.
천장 높이 가정(2.87 m)과 따로 재는 확인용. depth 는 멀수록 잡음 · 축척 오차가 커서 --corr k 로 Z/(1+kZ) 보정한 값도 같이 출력.
사용: in_ceilfar.py <keyframes 폴더> <trajectory(시각 x y z qx qy qz qw id)> [--y0 -0.9 --y1 1.2] [--x0 0 --x1 46] [--maxd 8] [--corr 0.007] [--png out.png]"""
import sys, re, numpy as np, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; opt = lambda k, d: a[a.index(k) + 1] if k in a else d
K = Path(a[1]); tr = np.loadtxt(a[2], comments='#', ndmin=2); Y0, Y1, X0, X1 = [float(opt(k, d)) for k, d in (('--y0', -0.9), ('--y1', 1.2), ('--x0', 0), ('--x1', 46))]
MAXD, KC = float(opt('--maxd', 8)), float(opt('--corr', 0.007)); out = {0: [], 1: []}
pre = 'kf_' if (K / 'kf_depth').exists() else ''      # exports 는 kf_depth, 전달 묶음은 depth
for r in tr:
    i = int(r[8]); f = K / f'{pre}depth/{i}.png'
    if not f.exists(): continue
    k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'{pre}calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
    d = cv2.imread(str(f), -1); v, u = np.mgrid[0:d.shape[0]:2, 0:d.shape[1]:2]; z = d[v, u].astype(float) / 1000; m = (z > 0.4) & (z < MAXD)
    R = Rot.from_quat(r[4:8]).as_matrix()
    for c in (0, 1):
        zz = z[m] / (1 + KC * z[m]) if c else z[m]
        P = np.stack([(u[m] - k[2]) / k[0] * zz, (v[m] - k[5]) / k[4] * zz, zz], 1) @ R.T + r[1:4]
        s = (P[:, 1] > Y0) & (P[:, 1] < Y1) & (P[:, 0] > X0) & (P[:, 0] < X1) & (P[:, 2] > 2.2) & (P[:, 2] < 3.4)
        out[c].append(np.c_[P[s], np.linalg.norm(P[s] - r[1:4], axis=1)])
for c in (0, 1):
    A = np.vstack(out[c]) if out[c] else np.zeros((0, 4))
    print(f'[{"Z/(1+%gZ) 보정" % KC if c else "depth 그대로"}] 복도 안 z 2.2~3.4 m 점 {len(A):,}')
    if len(A) < 100: continue
    h, e = np.histogram(A[:, 2], bins=np.arange(2.2, 3.41, 0.02)); pk = np.argsort(h)[::-1][:3]
    print('   높이 봉우리: ' + ', '.join(f'{e[p]+0.01:.2f} m ({h[p]})' for p in pk) + f' | 거리 중앙 {np.median(A[:,3]):.1f} m (5~95% {np.percentile(A[:,3],5):.1f}~{np.percentile(A[:,3],95):.1f})')
    for lo in np.arange(0, MAXD, 1.0):   # 천장이면 거리마다 봉우리 높이가 같아야 함 (화면 위 끝에 걸린 것이면 거리에 따라 올라감)
        s = (A[:, 3] >= lo) & (A[:, 3] < lo + 1)
        if s.sum() > 200: hh, _ = np.histogram(A[s, 2], bins=e); print(f'   거리 {lo:g}~{lo+1:g} m: 점 {s.sum():,} · 봉우리 {e[np.argmax(hh)]+0.01:.2f} m · z 2.6 위 점의 중앙 {np.median(A[s & (A[:,2]>2.6), 2]) if (s & (A[:,2]>2.6)).sum() > 50 else float("nan"):.2f} m')
    if c == 1 and '--png' in a:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        plt.figure(figsize=(8, 4)); plt.hist(A[:, 2], bins=e, color='#5b7fa6'); plt.xlabel('z (m, floor-referenced poses, depth-corrected)'); plt.ylabel('points'); plt.axvline(2.87, color='k', ls='--', lw=1)
        plt.title('far-seen ceiling points from floor-referenced run'); plt.tight_layout(); plt.savefig(opt('--png', 'x.png'), dpi=90)
