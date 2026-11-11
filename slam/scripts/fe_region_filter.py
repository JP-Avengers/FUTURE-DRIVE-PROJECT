"""덧붙임 메시를 검증된 구역으로 자르기: (1) 상자 [x0,x1]×[y0,y1] 안 (2) v0.5 바닥 관측 칸(conf>=1)에서 --margin m 넘게 떨어진 곳
(차가 서는 주차장 안 = 관측 바닥 옆은 제외, 단 바닥 위 --hlow m 미만 낮은 면은 허용) (3) 작은 조각 제외.
사용: fe_region_filter.py <added.ply> <out.ply> --box x0,x1,y0,y1 [--margin 1.0] [--hlow 0.25] [--minc 50]"""
import sys, numpy as np, open3d as o3d
from pathlib import Path
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
a = sys.argv; opt = lambda k, d: a[a.index(k) + 1] if k in a else d
x0, x1, y0, y1 = map(float, opt('--box', '-1e9,1e9,-1e9,1e9').split(',')); MG, HL, MINC = float(opt('--margin', 1.0)), float(opt('--hlow', 0.25)), int(opt('--minc', 50))
F = np.load(Path.home() / 'b_slam/deliver/map_v0.5/floor_height.npz'); G, lo, c, conf = F['grid'].astype(float), F['lo'], float(F['cell']), F['conf']
dist = ndimage.distance_transform_edt(conf < 1) * c                              # 관측 바닥 칸까지 거리(m)
m = o3d.io.read_triangle_mesh(a[1]); V = np.asarray(m.vertices); T = np.asarray(m.triangles)
ix = np.clip(((V[:, 0] - lo[0]) / c).astype(int), 0, G.shape[0] - 1); iy = np.clip(((V[:, 1] - lo[1]) / c).astype(int), 0, G.shape[1] - 1)
h = V[:, 2] - ndimage.map_coordinates(G, [(V[:, 0] - lo[0]) / c - 0.5, (V[:, 1] - lo[1]) / c - 0.5], order=1, mode='nearest')
inbox = (V[:, 0] >= x0) & (V[:, 0] <= x1) & (V[:, 1] >= y0) & (V[:, 1] <= y1)
okv = inbox & ((dist[ix, iy] > MG) | (h < HL))
keep = okv[T].all(1); T2 = T[keep]; n = len(V); e = np.r_[T2[:, [0, 1]], T2[:, [1, 2]], T2[:, [2, 0]]]
_, lab = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)), directed=False)
cnt = np.bincount(lab[T2[:, 0]]); T3 = T2[cnt[lab[T2[:, 0]]] >= MINC]
out = o3d.geometry.TriangleMesh(m.vertices, o3d.utility.Vector3iVector(T3))
if m.has_vertex_colors(): out.vertex_colors = m.vertex_colors
out.remove_unreferenced_vertices(); out.compute_vertex_normals(); o3d.io.write_triangle_mesh(a[2], out)
print(f'삼각형 {len(T):,} → 상자·관측 바닥 {MG:g}m 밖(낮은 면 허용) {keep.sum():,} → 작은 조각 제외 {len(T3):,} (면적 {out.get_surface_area():.0f}㎡)')
