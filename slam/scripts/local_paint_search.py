"""초기 변환 주변만 주차선 2D 탐색: src 주차선 점을 yaw ±R°(0.25° 간격), 이동 ±M m(5cm 간격)으로 돌려 기준 주차선 거리장(5cm 격자)에 맞춤.
점수 = 기준 주차선 15cm 안에 들어오는 src 점 비율. 최고 해와 같은 yaw에서 다른 위치 해의 점수 비도 출력(유일성 확인).
사용: local_paint_search.py <src_paint.npy> <ref_paint.npy> <T_init(4x4 .txt/.npy)> <out.npy> [--yaw 8] [--move 4]"""
import sys, numpy as np
from scipy import ndimage
a = sys.argv; src, ref, tin, out = a[1:5]; R = float(a[a.index('--yaw') + 1]) if '--yaw' in a else 8.0; M = float(a[a.index('--move') + 1]) if '--move' in a else 4.0
P = np.load(src)[:, :2]; Q = np.load(ref)[:, :2]; T0 = np.loadtxt(tin) if tin.endswith('.txt') else np.load(tin)
P0 = P @ T0[:2, :2].T + T0[:2, 3]; c = P0.mean(0)
P0 = P0[np.linalg.norm(P0 - c, axis=1) < 80][::max(1, len(P0) // 20000)]
cell = 0.05; lo = Q.min(0) - 10; shp = np.ceil((Q.max(0) + 10 - lo) / cell).astype(int)
G = np.ones(shp, bool); ij = ((Q - lo) / cell).astype(int); G[ij[:, 0], ij[:, 1]] = False; D = ndimage.distance_transform_edt(G) * cell
def score(Y):
    k = ((Y - lo) / cell).astype(int); ok = (k >= 0).all(1) & (k < shp).all(1); s = np.zeros(len(Y)); s[ok] = D[k[ok, 0], k[ok, 1]] < 0.15; return s.mean()
best = (-1, 0, 0, 0); res = []
for dy in np.arange(-R, R + 1e-9, 0.25):
    th = np.radians(dy); Rm = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); Y = (P0 - c) @ Rm.T + c
    # 이동은 거리장 상관으로 한 번에: Y 점을 격자에 찍고 '15cm 안' 마스크와 상관
    near = (D < 0.15).astype(np.float32); H = np.zeros_like(near)
    k = ((Y - lo) / cell).astype(int); ok = (k >= 0).all(1) & (k < shp).all(1); np.add.at(H, (k[ok, 0], k[ok, 1]), 1)
    m = int(M / cell); kx, ky = np.nonzero(H)
    sc = np.zeros((2 * m + 1, 2 * m + 1))
    for ddx in range(-m, m + 1, 2):
        xs = kx + ddx; okx = (xs >= 0) & (xs < shp[0])
        for ddy in range(-m, m + 1, 2):
            ys = ky + ddy; o2 = okx & (ys >= 0) & (ys < shp[1]); sc[ddx + m, ddy + m] = (H[kx[o2], ky[o2]] * near[xs[o2], ys[o2]]).sum()
    i, j = np.unravel_index(sc.argmax(), sc.shape); v = sc[i, j] / len(Y)
    res.append((v, dy, (i - m) * cell, (j - m) * cell, sc))
    if v > best[0]: best = (v, dy, (i - m) * cell, (j - m) * cell)
v, dy, tx, ty = best
# 같은 yaw에서 최고점과 1m 넘게 떨어진 다른 위치의 최고 점수
sc = [r for r in res if r[1] == dy][0][4]; m = int(M / cell); I, J = np.mgrid[-m:m + 1, -m:m + 1] * cell
far = np.hypot(I - tx, J - ty) > 1.0; v2 = sc[far].max() / len(P0) if far.any() else 0
other = max((r[0] for r in res if abs(r[1] - dy) > 2.0), default=0)
print(f'최고: yaw {dy:+.2f}°, 이동 ({tx:+.2f}, {ty:+.2f}) m, 15cm 안 비율 {v:.1%} | 같은 yaw 다른 위치 최고 {v2:.1%} (비 {v/max(v2,1e-9):.2f}) | 2° 넘게 다른 yaw 최고 {other:.1%} (비 {v/max(other,1e-9):.2f})')
th = np.radians(dy); Rm = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
A = np.eye(4); A[:2, :2] = Rm; A[:2, 3] = c - Rm @ c + [tx, ty]; T = A @ T0; np.save(out, T); print('→', out)
