"""천장 · 벽 위쪽 채움: 복도 바닥 면적(x 0~46, 벽 선 사이) 10cm 칸 중 천장(높이 HC±0.15m) 메쉬가 덮은 비율, 벽 꼭대기 높이 중앙.
사용: in_ceilcov.py <mesh.ply>... [--hc 2.87] [--y0 -1.10 --y1 1.43]"""
import sys, numpy as np, open3d as o3d
a = sys.argv; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d; HC, Y0, Y1 = opt('--hc', 2.87), opt('--y0', -1.10), opt('--y1', 1.43)
for p in [x for x in a[1:] if x.endswith('.ply')]:
    V = np.asarray(o3d.io.read_triangle_mesh(p).vertices); c = V[(np.abs(V[:, 2] - HC) < 0.15) & (V[:, 0] > 0) & (V[:, 0] < 46) & (V[:, 1] > Y0) & (V[:, 1] < Y1)]
    g = np.zeros((460, int((Y1 - Y0) / 0.1) + 1), bool); ij = ((c[:, :2] - [0, Y0]) / 0.1).astype(int); g[ij[:, 0].clip(0, 459), ij[:, 1].clip(0, g.shape[1] - 1)] = True
    tops = []
    for yw in (Y0, Y1):
        s = V[(np.abs(V[:, 1] - yw) < 0.12) & (V[:, 0] > 0) & (V[:, 0] < 46) & (V[:, 2] < HC - 0.1)]; b = (s[:, 0] / 0.5).astype(int)
        t = [np.percentile(s[b == k, 2], 98) for k in np.unique(b) if (b == k).sum() >= 50]; tops.append(np.median(t) if t else float('nan'))
    print(f'{p.split("/")[-1]}: 복도 천장 덮음 {g.mean():.0%} | 벽 꼭대기 중앙 -y {tops[0]:.2f}m · +y {tops[1]:.2f}m | 메쉬 최고 {V[:, 2].max():.2f}m')
