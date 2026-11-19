"""복도 통로 공간 안에 떠 있는 메쉬(잡음 · 번짐): 벽 선에서 0.25m 넘게 안쪽, 바닥 0.15m 위 ~ 천장 0.2m 아래, x 0~46(로비 입구 x 18~27 는 뺌)의 삼각형 면적.
사용: in_clutter.py <mesh.ply>... [--hc 2.87] [--y0 -1.10 --y1 1.43]"""
import sys, numpy as np, open3d as o3d
a = sys.argv; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d; HC, Y0, Y1 = opt('--hc', 2.87), opt('--y0', -1.10), opt('--y1', 1.43)
for p in [x for x in a[1:] if x.endswith('.ply')]:
    m = o3d.io.read_triangle_mesh(p); V = np.asarray(m.vertices); T = np.asarray(m.triangles); c = V[T].mean(1)
    ins = (c[:, 0] > 0) & (c[:, 0] < 46) & ~((c[:, 0] > 18) & (c[:, 0] < 27)) & (c[:, 1] > Y0 + 0.25) & (c[:, 1] < Y1 - 0.25) & (c[:, 2] > 0.15) & (c[:, 2] < HC - 0.2)
    ar = 0.5 * np.linalg.norm(np.cross(V[T[:, 1]] - V[T[:, 0]], V[T[:, 2]] - V[T[:, 0]]), axis=1)
    lo_, hi_ = ins & (c[:, 2] < 1.5), ins & (c[:, 2] >= 1.5)
    print(f'{p.split("/")[-1]}: 통로 안에 떠 있는 면 {ar[ins].sum():.2f}㎡ (1.5m 아래 {ar[lo_].sum():.2f} · 위 {ar[hi_].sum():.2f}), 삼각형 {ins.sum():,}')
