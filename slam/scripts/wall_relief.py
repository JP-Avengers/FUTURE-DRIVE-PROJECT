"""벽면 요철 대조: 두 메쉬의 같은 벽(벽 선 ±0.25m) 꼭짓점을 (x, z) 2cm 격자에 벽 선에서 떨어진 거리(요철)로 그려
겹치는 곳에서 정규 상관이 가장 큰 (dx, dz) 를 찾음 → 높이 · 복도 방향 어긋남.
사용: wall_relief.py <ref.ply> <test.ply> [--y -1.10,1.43] [--zr 0.9,1.8] [--x0 0 --x1 46]"""
import sys, numpy as np, open3d as o3d, cv2
a = sys.argv; opt = lambda k, d: a[a.index(k) + 1] if k in a else d
YS = [float(v) for v in opt('--y', '-1.10,1.43').split(',')]; ZR = [float(v) for v in opt('--zr', '0.9,1.8').split(',')]; X0, X1 = float(opt('--x0', 0)), float(opt('--x1', 46)); C = 0.02
def relief(p, yw):
    V = np.asarray(o3d.io.read_triangle_mesh(p).vertices); s = V[(np.abs(V[:, 1] - yw) < 0.25) & (V[:, 0] > X0) & (V[:, 0] < X1) & (V[:, 2] > ZR[0] - 0.4) & (V[:, 2] < ZR[1] + 0.4)]
    nx, nz = int((X1 - X0) / C) + 1, int((ZR[1] - ZR[0] + 0.8) / C) + 1; S = np.zeros((nz, nx)); Nn = np.zeros((nz, nx))
    i = ((s[:, 0] - X0) / C).astype(int); k = ((s[:, 2] - ZR[0] + 0.4) / C).astype(int); np.add.at(S, (k, i), np.abs(s[:, 1] - yw)); np.add.at(Nn, (k, i), 1)
    R = np.where(Nn > 0, S / np.maximum(Nn, 1), np.nan); return R
for yw in YS:
    A = relief(a[1], yw); B = relief(a[2], yw); best = None; res = {}
    z0 = int(0.4 / C); zn = int((ZR[1] - ZR[0]) / C)
    for dz in range(-15, 16):
        for dx in range(-25, 26):
            Bs = np.roll(np.roll(B, dz, 0), dx, 1)[z0:z0 + zn]; As = A[z0:z0 + zn]; m = np.isfinite(As) & np.isfinite(Bs)
            if m.sum() < 2000: continue
            u, v = As[m] - As[m].mean(), Bs[m] - Bs[m].mean(); r = float((u * v).sum() / np.sqrt((u * u).sum() * (v * v).sum() + 1e-12)); res[(dz, dx)] = r
            if best is None or r > best[0]: best = (r, dz, dx, int(m.sum()))
    if best is None: print(f'벽 y={yw:+.2f}: 겹치는 칸 부족'); continue
    r0 = res.get((0, 0), float('nan')); vals = np.array(list(res.values()))
    print(f'벽 y={yw:+.2f}: 최대 상관 {best[0]:.3f} @ 높이 {-best[1] * C * 100:+.0f}cm · 복도 방향 {-best[2] * C * 100:+.0f}cm (겹친 칸 {best[3]}) | 어긋남 0 에서 {r0:.3f} | 전체 탐색 중앙 {np.median(vals):.3f}')
