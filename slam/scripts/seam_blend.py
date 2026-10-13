#!/usr/bin/env python3
"""이음새 보정: 합친 메시에서 바탕(base)에 없던 꼭짓점(채운 조각)을, 경계에서 잰 높이 차만큼 옮기되 경계에서 멀어질수록 줄임.
 1) 경계 0.5m 칸: 바탕 바닥(|nz|>0.9)과 채운 바닥이 함께 있는 칸마다 차이 = 바탕 중앙값 − 채운 쪽 중앙값
 2) 채운 꼭짓점마다 보정 = Σ w_i·차이_i / Σ w_i (경계 칸까지 거리 가중, 가까운 16칸) × exp(−가장 가까운 경계 칸까지 거리 / L)
바탕 꼭짓점은 그대로. 사용: seam_blend.py <base_mesh> <merged_mesh> <out.ply> [L=2.0]   (~/tsdf_env)"""
import sys
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
base, merged, out = sys.argv[1:4]; L = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
B = o3d.io.read_triangle_mesh(base); B.compute_vertex_normals(); Vb, Nb = np.asarray(B.vertices), np.asarray(B.vertex_normals)
M = o3d.io.read_triangle_mesh(merged); M.compute_vertex_normals(); V, N = np.asarray(M.vertices).copy(), np.asarray(M.vertex_normals)
added = np.isinf(cKDTree(Vb).query(V, distance_upper_bound=0.002)[0])
fb = Vb[np.abs(Nb[:, 2]) > 0.9]; fa = V[added & (np.abs(N[:, 2]) > 0.9)]
def cells(X):
    c = np.floor(X[:, :2] / 0.5).astype(np.int64); k = c[:, 0] * 100003 + c[:, 1]; o = np.argsort(k); k, z = k[o], X[o, 2]
    u, st, n = np.unique(k, return_index=True, return_counts=True); return {a: np.median(z[s:s + m]) for a, s, m in zip(u, st, n) if m >= 5}
cb, ca = cells(fb), cells(fa); ks = np.array(sorted(set(cb) & set(ca)))
off = np.array([cb[k] - ca[k] for k in ks]); ok = np.abs(off) < 0.5; ks, off = ks[ok], off[ok]
cx = (ks // 100003 + 0.5) * 0.5; cy = ((ks % 100003) - np.where(ks % 100003 > 50000, 100003, 0) + 0.5) * 0.5
C = np.c_[cx, cy]; tc = cKDTree(C)
print(f'경계 칸 {len(off):,}개, 보정 전 칸별 차이 |중앙값| {np.median(np.abs(off))*100:.1f}cm')
ia = np.nonzero(added)[0]; d, j = tc.query(V[ia, :2], k=16)
w = 1.0 / np.maximum(d, 0.25) ** 2; corr = (w * off[j]).sum(1) / w.sum(1) * np.exp(-d[:, 0] / L)
V[ia, 2] += corr
M.vertices = o3d.utility.Vector3dVector(V); M.compute_vertex_normals(); o3d.io.write_triangle_mesh(out, M)
print(f'채운 꼭짓점 {len(ia):,}개 보정: |보정| 중앙 {np.median(np.abs(corr))*100:.1f}cm, 90% {np.percentile(np.abs(corr),90)*100:.1f}cm → {out}')
