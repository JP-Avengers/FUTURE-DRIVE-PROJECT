"""복도 구간에서 천장 위로 튄 조각 잘라내기: 삼각형 꼭짓점 중 하나라도 z > ztop 이고 (x 가 로비 입구 밖, y 가 복도 폭 안)이면 뺌.
사용: mesh_croptop.py <in.ply> <out.ply> [--ztop 3.27] [--y0 -1.3 --y1 1.65] [--lx0 18 --lx1 27]"""
import sys, numpy as np, open3d as o3d
a = sys.argv; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
ZT, Y0, Y1, L0, L1 = opt('--ztop', 3.27), opt('--y0', -1.3), opt('--y1', 1.65), opt('--lx0', 18), opt('--lx1', 27)
m = o3d.io.read_triangle_mesh(a[1]); V = np.asarray(m.vertices); T = np.asarray(m.triangles)
bad_v = (V[:, 2] > ZT) & (V[:, 1] > Y0) & (V[:, 1] < Y1) & ~((V[:, 0] > L0) & (V[:, 0] < L1)); bad = bad_v[T].any(1)
m.triangles = o3d.utility.Vector3iVector(T[~bad]); m.remove_unreferenced_vertices(); m.compute_vertex_normals(); o3d.io.write_triangle_mesh(a[2], m)
print(f'복도 천장 위({ZT}m) 삼각형 {bad.sum():,}개 잘라냄 → {a[2]} (삼각형 {len(m.triangles):,})')
