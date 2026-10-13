"""주차선 2D ICP (yaw+이동만). 초기값 = run20_to_run18.npy. 구역별 잔차(빨강→가장 가까운 검정 중앙 거리) 비교."""
import numpy as np
from pathlib import Path
from scipy.spatial import cKDTree
H = Path.home()/'b_slam/exports'; T3 = np.load(H/'run20_to_run18.npy')
P0 = np.load(H/'run18/paint_xy.npy'); P1 = np.load(H/'run20/paint_xy.npy')
REG = {'A': (-50, -29, -25, -14), 'B': (-34, -17, 3, 14), 'C': (-4, 7, -16, 11)}
inreg = lambda X, r: (X[:, 0] > r[0]) & (X[:, 0] < r[1]) & (X[:, 1] > r[2]) & (X[:, 1] < r[3])
tree = cKDTree(P0)
def apply(R, t, X): return X @ R.T + t
def report(R, t, tag):
    X = apply(R, t, P1); d = tree.query(X)[0]; s = []
    for k, r in REG.items():
        m = inreg(X, r); s.append(f'{k}: 중앙 {np.median(d[m])*100:.0f}cm (n={m.sum()})')
    print(f'[{tag}] yaw {np.degrees(np.arctan2(R[1,0],R[0,0]))%360:.2f}°, t=({t[0]:.2f},{t[1]:.2f}) | ' + ' | '.join(s))
R = T3[:2, :2] / np.linalg.norm(T3[:2, 0]); t = T3[:2, 3].copy()
report(R, t, 'ICP(3D) 결과')
def icp(R, t, regions=None, iters=60):
    src = P1
    for thr in np.r_[np.full(15, 1.5), np.full(15, 0.8), np.full(30, 0.4)][:iters]:
        X = apply(R, t, src); d, j = tree.query(X); m = d < thr
        if regions is not None: m &= np.any([inreg(X, REG[k]) for k in regions], axis=0)
        a, b = src[m], P0[j[m]]; ca, cb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb)); Rn = (U @ Vt).T
        if np.linalg.det(Rn) < 0: Vt[1] *= -1; Rn = (U @ Vt).T
        R, t = Rn, cb - Rn @ ca
    return R, t
R1, t1 = icp(R, t); report(R1, t1, '주차선 ICP(전체)')
for k in REG:
    Rk, tk = icp(R, t, [k]); report(Rk, tk, f'주차선 ICP({k}만)')
np.save(H/'run20_to_run18_paint2d.npy', np.r_[R1.ravel(), t1])
