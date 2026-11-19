"""실내 복도 메시에서 통로 안에 떠 있는 작은 조각(잡음 · 번짐)을 지운다. 연결 성분 단위로만 지우고, 큰 구조(벽 · 바닥 · 천장)에 붙은 것은 건드리지 않는다.
 지우는 조건 (A 통로 조각, 모두 만족): 성분 면적 < --max-area, 면적 가중 중심이 통로 빈 공간 안(x --x0~--x1, y 벽 선에서 --margin 안쪽, z 0.15 ~ 천장-0.2),
   가장 낮은 점 > --floor (바닥에 서 있지 않음), 큰 성분(면적 >= --max-area) 꼭짓점까지 최단 거리 > --near (벽 · 바닥 · 문틀에 닿지 않음).
   로비(x --lobby, y > 벽 선 - margin)와 복도 양 끝(x 범위 밖)은 통로 조건에서 빠지므로 A 로는 지우지 않는다.
 (B 외딴 점 조각, --speck N 일 때, 어디서나): 삼각형 N 개 미만이고 다른 성분(삼각형 N 개 이상) 꼭짓점에서 --speck-dist 넘게 떨어진 성분.
사용: mesh_declutter.py <in.ply> <out.ply> [--max-area 0.3] [--floor 0.05] [--near 0.10] [--margin 0.25] [--hc 2.87] [--y0 -1.10 --y1 1.43]
       [--x0 0 --x1 46] [--lobby 18,27] [--speck 0 --speck-dist 0.15] [--removed removed.ply] [--dry]"""
import sys, numpy as np, open3d as o3d
from scipy.spatial import cKDTree
a = sys.argv; IN, OUT = a[1], a[2]; opt = lambda k, d: type(d)(a[a.index(k) + 1]) if k in a else d
MAXA, FLR, NEAR, MRG, HC = opt('--max-area', 0.3), opt('--floor', 0.05), opt('--near', 0.10), opt('--margin', 0.25), opt('--hc', 2.87)
Y0, Y1, X0, X1 = opt('--y0', -1.10), opt('--y1', 1.43), opt('--x0', 0.0), opt('--x1', 46.0); L0, L1 = map(float, opt('--lobby', '18,27').split(','))
NSP, DSP = opt('--speck', 0), opt('--speck-dist', 0.15)
m = o3d.io.read_triangle_mesh(IN); V = np.asarray(m.vertices); T = np.asarray(m.triangles)
cl, n, ca = m.cluster_connected_triangles(); cl = np.asarray(cl); n = np.asarray(n); ca = np.asarray(ca); K = len(n)
ar = 0.5 * np.linalg.norm(np.cross(V[T[:, 1]] - V[T[:, 0]], V[T[:, 2]] - V[T[:, 0]]), axis=1); c = V[T].mean(1)
cen = np.stack([np.bincount(cl, ar * c[:, j], K) for j in range(3)], 1) / np.maximum(ca, 1e-12)[:, None]   # 성분별 면적 가중 중심
zmin = np.full(K, np.inf); np.minimum.at(zmin, cl, V[T].min(1)[:, 2])
print(f'{IN.split("/")[-1]}: 삼각형 {len(T):,}, 성분 {K:,} (면적 >= {MAXA}㎡ 인 큰 성분 {np.sum(ca >= MAXA)})')
def min_dist(cand, ref_mask, ub):
    """후보 성분마다, ref_mask 성분 꼭짓점까지 최단 거리(ub 넘으면 inf)."""
    vr = np.unique(T[ref_mask[cl]]); tree = cKDTree(V[vr]); tc = np.isin(cl, np.flatnonzero(cand))
    vc = T[tc].ravel(); lc = np.repeat(cl[tc], 3); u, iu = np.unique(vc, return_index=True)
    d, _ = tree.query(V[u], distance_upper_bound=ub); out = np.full(K, np.inf); np.minimum.at(out, lc[iu], d); return out
x, y, z = cen.T
free = (x > X0) & (x < X1) & (y > Y0 + MRG) & (y < Y1 - MRG) & (z > 0.15) & (z < HC - 0.2) & ~((x > L0) & (x < L1) & (y > Y1 - MRG))
candA = (ca < MAXA) & free & (zmin > FLR)
dA = min_dist(candA, ca >= MAXA, NEAR + 0.01) if candA.any() else np.full(K, np.inf); remA = candA & (dA > NEAR)
remB = np.zeros(K, bool)
if NSP > 0:
    candB = (n < NSP) & ~remA; dB = min_dist(candB, n >= NSP, DSP + 0.01); remB = candB & (dB > DSP)
def report(tag, r):
    zb = [0.15, 0.5, 1.0, 1.5, 2.0, HC - 0.2]; h = np.histogram(z[r], zb, weights=ca[r])[0]; hs = ' '.join(f'{zb[i]:.2f}-{zb[i + 1]:.2f}m {h[i]:.3f}' for i in range(len(h)))
    print(f' {tag}: 성분 {r.sum():,}, 삼각형 {n[r].sum():,}, 면적 {ca[r].sum():.3f}㎡ | 중심 높이별 면적 {hs}' + (f' | 기타(z<0.15 또는 위) {ca[r].sum() - h.sum():.3f}' if tag.startswith('B') else ''))
report('A 통로 조각', remA)
for k in np.flatnonzero(remA)[np.argsort(-ca[remA])][:12]:
    tk = T[cl == k]; lo, hi = V[tk].reshape(-1, 3).min(0), V[tk].reshape(-1, 3).max(0)
    print(f'   #{k}: 삼각형 {n[k]}, {ca[k]:.3f}㎡, 범위 x {lo[0]:.2f}~{hi[0]:.2f} y {lo[1]:.2f}~{hi[1]:.2f} z {lo[2]:.2f}~{hi[2]:.2f}, 구조까지 {">" if np.isinf(dA[k]) else ""}{min(dA[k], NEAR + 0.01):.2f}m')
if NSP > 0: report(f'B 외딴 점 조각(<{NSP} 삼각형, {DSP}m 넘게 떨어짐)', remB)
kept_cand = candA & ~remA; print(f' 통로 안 작은 성분 중 구조에 {NEAR}m 안으로 붙어 남긴 것: {kept_cand.sum()} 개 {ca[kept_cand].sum():.3f}㎡')
rm = (remA | remB)[cl]
if '--removed' in a:
    r = o3d.geometry.TriangleMesh(m); r.remove_triangles_by_mask(~rm); r.remove_unreferenced_vertices(); o3d.io.write_triangle_mesh(opt('--removed', ''), r)
if '--dry' not in a:
    m.remove_triangles_by_mask(rm); m.remove_unreferenced_vertices(); o3d.io.write_triangle_mesh(OUT, m)
    print(f' 저장 {OUT}: 삼각형 {len(m.triangles):,} (지운 것 {rm.sum():,})')
