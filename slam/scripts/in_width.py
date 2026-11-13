"""복도 폭 점검 — 메쉬별로 벽면(높이 0.4~1.4 m) y 분포의 봉우리를 1 cm 칸으로 찾고, x 구간별 폭도 본다.
사용: in_width.py <mesh.ply> ... [--x0 3 --x1 17] [--seg 5]"""
import sys, numpy as np, open3d as o3d
a = sys.argv; one = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
X0, X1, SEG = one('--x0', -2.0), one('--x1', 48.0), one('--seg', 5.0)
ms = [p for p in a[1:] if p.endswith('.ply')]
def peaks(y):
    if len(y) < 500: return None
    h, e = np.histogram(y, bins=np.arange(-2.2, 2.6, 0.01)); c = (e[:-1] + e[1:]) / 2; h = np.convolve(h, np.ones(3) / 3, 'same')
    lo = c < 0; i = int(np.argmax(np.where(lo, h, 0))); j = int(np.argmax(np.where(~lo, h, 0)))
    def sub(k):  # 봉우리 ±4 cm 가중 평균
        s = slice(max(k - 4, 0), k + 5); return float((c[s] * h[s]).sum() / h[s].sum())
    return sub(i), sub(j), h[i], h[j]
for p in ms:
    V = np.asarray(o3d.io.read_triangle_mesh(p).vertices); w = V[(V[:, 2] > 0.4) & (V[:, 2] < 1.4) & (np.abs(V[:, 1]) < 2.6)]
    r = peaks(w[(w[:, 0] > X0) & (w[:, 0] < X1), 1]); print(f'\n{p.split("/")[-1]}: 전체 x {X0:+.0f}~{X1:+.0f}  y = {r[0]:+.3f}, {r[1]:+.3f}  폭 {r[1] - r[0]:.3f} m')
    out = []
    for x in np.arange(X0, X1, SEG):
        r = peaks(w[(w[:, 0] >= x) & (w[:, 0] < x + SEG), 1])
        out.append(f'  x {x:+5.0f}~{x + SEG:+5.0f}: ' + ('점 부족' if r is None else f'y {r[0]:+.3f} / {r[1]:+.3f}  폭 {r[1] - r[0]:.3f}  (봉우리 높이 {r[2]:.0f} / {r[3]:.0f})'))
    print('\n'.join(out))
