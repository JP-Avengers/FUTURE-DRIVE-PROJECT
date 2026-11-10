"""덧붙임 메시 거르기: 세 꼭짓점 모두 v0.5 바닥 높이 지도 기준 [--hmin, --hmax) 안인 삼각형만(바닥·카스토퍼·연석), 작은 조각(--minc 삼각형 미만) 제외.
사용: fe_added_filter.py <added.ply> <out.ply> [--hmin -0.3] [--hmax 0.25] [--minc 50]"""
import sys, numpy as np, open3d as o3d
from pathlib import Path
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
a = sys.argv; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
HMIN, HMAX, MINC = opt('--hmin', -0.3), opt('--hmax', 0.25), int(opt('--minc', 50))
F = np.load(Path.home() / 'b_slam/deliver/map_v0.5/floor_height.npz'); G, lo, c = F['grid'].astype(float), F['lo'], float(F['cell'])
m = o3d.io.read_triangle_mesh(a[1]); V = np.asarray(m.vertices); T = np.asarray(m.triangles)
h = V[:, 2] - ndimage.map_coordinates(G, [(V[:, 0] - lo[0]) / c - 0.5, (V[:, 1] - lo[1]) / c - 0.5], order=1, mode='nearest')
okv = (h >= HMIN) & (h < HMAX); keep = okv[T].all(1); T2 = T[keep]
n = len(V); e = np.r_[T2[:, [0, 1]], T2[:, [1, 2]], T2[:, [2, 0]]]
_, lab = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)), directed=False)
cnt = np.bincount(lab[T2[:, 0]]); big = cnt[lab[T2[:, 0]]] >= MINC; T3 = T2[big]
out = o3d.geometry.TriangleMesh(m.vertices, o3d.utility.Vector3iVector(T3))
if m.has_vertex_colors(): out.vertex_colors = m.vertex_colors
out.remove_unreferenced_vertices(); out.compute_vertex_normals(); o3d.io.write_triangle_mesh(a[2], out)
area = out.get_surface_area()
print(f'삼각형 {len(T):,} → 높이 [{HMIN:+.2f}, {HMAX:+.2f})m {keep.sum():,} → 작은 조각 제외 {len(T3):,} (면적 {area:.0f}㎡) → {a[2]}')
