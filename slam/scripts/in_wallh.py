"""벽이 어느 높이까지 있는지 — 복도 두 벽 선(|y - Y| < 0.12 m)을 x 0.5 m 칸으로 나눠 칸마다 꼭짓점 높이 98% 값을 본다.
사용: in_wallh.py <mesh.ply> ... [--x0 -2 --x1 48]"""
import sys, numpy as np, open3d as o3d
a = sys.argv; one = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
X0, X1 = one('--x0', -2.0), one('--x1', 48.0)
for p in [q for q in a[1:] if q.endswith('.ply')]:
    V = np.asarray(o3d.io.read_triangle_mesh(p).vertices); V = V[(V[:, 0] > X0) & (V[:, 0] < X1)]
    w = V[(V[:, 2] > 0.4) & (V[:, 2] < 1.4) & (np.abs(V[:, 1]) < 2.6)]; h, e = np.histogram(w[:, 1], bins=np.arange(-2.2, 2.6, 0.02)); c = (e[:-1] + e[1:]) / 2
    Y = [float(c[np.argmax(np.where(c < 0, h, 0))]), float(c[np.argmax(np.where(c > 0, h, 0))])]; print(f'\n{p.split("/")[-1]}')
    for nm, yw in zip(('-y 벽', '+y 벽'), Y):
        s = V[np.abs(V[:, 1] - yw) < 0.12]; b = ((s[:, 0] - X0) / 0.5).astype(int); top = np.array([np.percentile(s[b == k, 2], 98) for k in np.unique(b) if (b == k).sum() >= 50])
        print(f'  {nm} (y {yw:+.2f}): 칸 {len(top)}개 | 벽 꼭대기 중앙 {np.median(top):.2f} m, 25~75% {np.percentile(top, 25):.2f}~{np.percentile(top, 75):.2f} m | 1.8 m 넘는 칸 {100 * (top >= 1.8).mean():.0f}%, 2.2 m 넘는 칸 {100 * (top >= 2.2).mean():.0f}%')
