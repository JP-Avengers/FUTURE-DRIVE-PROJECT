#!/usr/bin/env python3
"""바닥 조각 평탄화: 조각 꼭짓점 높이를 바탕 메시의 주변 바닥(수평면 |nz|>0.95, 조각에서 15cm 밖) 300점 국소 평면 값으로 바꿈.
수평 범위(바닥이 있는 곳)는 조각 그대로. 바닥 높이 띠는 조각 높이 중앙값 ±0.3m.
사용: flatten_patch.py <base_mesh> <patch_floor.ply> <out.ply>   (~/tsdf_env)"""
import sys
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
base, patch, out = sys.argv[1:4]
p = o3d.io.read_triangle_mesh(patch); V = np.asarray(p.vertices).copy()
A = o3d.io.read_triangle_mesh(base); A.compute_vertex_normals(); VA, NA = np.asarray(A.vertices), np.asarray(A.vertex_normals)
lo, hi = V[:, :2].min(0) - 6, V[:, :2].max(0) + 6; zm = np.median(V[:, 2])
g = (np.abs(NA[:, 2]) > 0.95) & (np.abs(VA[:, 2] - zm) < 0.3) & np.all((VA[:, :2] > lo) & (VA[:, :2] < hi), 1); G = VA[g]
G = G[np.isinf(cKDTree(V[:, :2]).query(G[:, :2], distance_upper_bound=0.15)[0])]
d, j = cKDTree(G[:, :2]).query(V[:, :2], k=300); z = np.empty(len(V))
for i in range(len(V)):
    Q = G[j[i]]; A_ = np.c_[Q[:, :2], np.ones(len(Q))]; w = np.ones(len(Q), bool)
    for _ in range(3):
        c = np.linalg.lstsq(A_[w], Q[w, 2], rcond=None)[0]; e = Q[:, 2] - A_ @ c; w = np.abs(e) < max(3 * np.median(np.abs(e[w])), 0.02)
    z[i] = c @ [V[i, 0], V[i, 1], 1]
print(f'꼭짓점 {len(V):,}: 높이 변경 중앙 {np.median(z - V[:, 2])*100:+.1f}cm, |변경| 90% {np.percentile(np.abs(z - V[:, 2]), 90)*100:.1f}cm, 주변 바닥 300점 반경 중앙 {np.median(d[:, -1]):.2f}m')
V[:, 2] = z; p.vertices = o3d.utility.Vector3dVector(V); p.compute_vertex_normals(); o3d.io.write_triangle_mesh(out, p); print('→', out)
