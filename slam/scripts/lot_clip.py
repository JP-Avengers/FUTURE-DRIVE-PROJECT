"""둘레 실험층에서 주차장 안(v0.5 관측 바닥을 닫고 구멍을 메운 영역 + --grow m)의 낮은 면(바닥·주차선, 바닥 위 --hlow 미만) 제거.
주차장 안은 v0.5(+지도 밖 바닥 격자)를 기준으로 하고, 손 촬영은 연석 바깥(보도·화단·벽·잔디)만 남김.
사용: lot_clip.py <perimeter.ply> <floor.npz> <out.ply> [--close 3.0] [--grow 0.3] [--hlow 0.25] [--minc 50]"""
import sys, numpy as np, open3d as o3d, cv2
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
a = sys.argv; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
CL, GR, HL, MINC = opt('--close', 3.0), opt('--grow', 0.3), opt('--hlow', 0.25), int(opt('--minc', 50))
F = np.load(a[2]); G, lo, c, conf = F['grid'].astype(float), F['lo'], float(F['cell']), F['conf']
obs = (conf >= 1) & (conf <= 3); r = int(round(CL / c)); st = np.ones((3, 3), bool)
lot = ndimage.binary_closing(np.pad(obs, r), structure=st, iterations=r)[r:-r, r:-r]; lot = ndimage.binary_fill_holes(lot)
lot = ndimage.binary_dilation(lot, structure=st, iterations=int(round(GR / c)))
m = o3d.io.read_triangle_mesh(a[1]); V = np.asarray(m.vertices); T = np.asarray(m.triangles)
ix = np.clip(((V[:, 0] - lo[0]) / c).astype(int), 0, G.shape[0] - 1); iy = np.clip(((V[:, 1] - lo[1]) / c).astype(int), 0, G.shape[1] - 1)
h = V[:, 2] - ndimage.map_coordinates(G, [(V[:, 0] - lo[0]) / c - 0.5, (V[:, 1] - lo[1]) / c - 0.5], order=1, mode='nearest')
drop = lot[ix, iy] & (h < HL); keep = ~drop[T].any(1); T2 = T[keep]; n = len(V)
e = np.r_[T2[:, [0, 1]], T2[:, [1, 2]], T2[:, [2, 0]]]; _, lab = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)), directed=False)
cnt = np.bincount(lab[T2[:, 0]]); T3 = T2[cnt[lab[T2[:, 0]]] >= MINC]
out = o3d.geometry.TriangleMesh(m.vertices, o3d.utility.Vector3iVector(T3))
if m.has_vertex_colors(): out.vertex_colors = m.vertex_colors
out.remove_unreferenced_vertices(); out.compute_vertex_normals(); o3d.io.write_triangle_mesh(a[3], out)
print(f'주차장 영역 {lot.sum() * c * c:.0f}㎡ | 삼각형 {len(T):,} → 주차장 안 낮은 면 제외 {keep.sum():,} → 작은 조각 제외 {len(T3):,} (면적 {out.get_surface_area():.0f}㎡)')
img = np.zeros((G.shape[1], G.shape[0], 3), np.uint8); img[lot.T[::-1]] = (60, 60, 60); img[obs.T[::-1]] = (120, 120, 120)
ok = ~drop; u = ix[ok]; v = G.shape[1] - 1 - iy[ok]; img[v, u] = (0, 200, 0); u = ix[drop]; v = G.shape[1] - 1 - iy[drop]; img[v, u] = (0, 0, 255)
ys, xs = np.where(img.sum(2) > 0); cv2.imwrite(a[3].replace('.ply', '_mask.png'), img[ys.min():ys.max() + 1, xs.min():xs.max() + 1])
