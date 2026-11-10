"""덧붙인 메시가 주행 통로를 건드리는지: D 기준 경로 ±W m 안 덧붙인 삼각형의 바닥 위 높이, 솟은 덩어리(연결 요소) 목록.
--drop 이면 통로 안에서 바닥보다 5cm 넘게 솟은 조각과 통로 안 바닥 조각(v0.5 바닥과 중복)을 덧붙임에서 빼고 저장.
사용: fe_route_guard.py <added.ply> <route_csv> <out_dir> [--w 2.35] [--drop]"""
import sys, json, numpy as np, open3d as o3d
from pathlib import Path
from scipy import ndimage
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
a = sys.argv; M, RC, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True); W = float(a[a.index('--w') + 1]) if '--w' in a else 2.35
P = Path.home() / 'b_slam/deliver/map_v0.5'; F = np.load(P / 'floor_height.npz'); G, lo, c = F['grid'].astype(float), F['lo'], float(F['cell'])
m = o3d.io.read_triangle_mesh(M); V = np.asarray(m.vertices); T = np.asarray(m.triangles)
R = np.loadtxt(RC, delimiter=',', skiprows=1)[:, 2:4]
# 경로 점을 10cm 간격으로 촘촘히
seg = [np.linspace(R[i], R[i + 1], max(2, int(np.linalg.norm(R[i + 1] - R[i]) / 0.1))) for i in range(len(R) - 1)]; Rd = np.vstack(seg)
d, _ = cKDTree(Rd).query(V[:, :2]); h = V[:, 2] - ndimage.map_coordinates(G, [(V[:, 0] - lo[0]) / c - 0.5, (V[:, 1] - lo[1]) / c - 0.5], order=1, mode='nearest')
inl = d <= W; tin = inl[T].any(1)
e = np.r_[T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]]; n = len(V)
ncomp, lab = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)), directed=False)
rep = dict(route_width=W, added_vertices=int(n), in_corridor_vertices=int(inl.sum()))
bad = []
for k in np.unique(lab[inl]):
    vk = (lab == k) & inl
    if vk.sum() < 5: continue
    hk = h[vk]; mx = float(np.percentile(hk, 95))
    if mx > 0.05: bad.append(dict(comp=int(k), n=int(vk.sum()), h95_cm=round(mx * 100, 1), x=round(float(V[vk, 0].mean()), 1), y=round(float(V[vk, 1].mean()), 1)))
bad.sort(key=lambda b: -b['n']); rep['raised_components'] = len(bad); rep['raised_list_top'] = bad[:15]
rep['in_corridor_height_cm'] = {k: round(float(np.percentile(h[inl], q)) * 100, 1) for k, q in (('p5', 5), ('p50', 50), ('p95', 95))} if inl.any() else {}
print(json.dumps(rep, ensure_ascii=False, indent=1))
if '--drop' in a:
    badc = np.zeros(ncomp, bool); badc[[b['comp'] for b in bad]] = True
    dropv = inl & (badc[lab] | (np.abs(h) < 0.05))                     # 통로 안 솟은 조각 + 통로 안 바닥 중복
    keep = ~dropv[T].any(1); m2 = o3d.geometry.TriangleMesh(m); m2.triangles = o3d.utility.Vector3iVector(T[keep])
    if m.has_vertex_colors(): m2.vertex_colors = m.vertex_colors
    m2.remove_unreferenced_vertices(); m2.compute_vertex_normals(); out = O / (Path(M).stem + '_guarded.ply'); o3d.io.write_triangle_mesh(str(out), m2)
    print(f'통로 안 삼각형 {int((~keep).sum()):,}개 뺌 → {out}')
(O / 'route_guard.json').write_text(json.dumps(rep, ensure_ascii=False, indent=1), encoding='utf-8')
