#!/usr/bin/env python3
"""평탄화한 바닥 조각을 기준 메시의 빈 곳에만 붙임 (mesh_fill.py와 같은 15cm 기준, 높이 재조정 없음).
검증: 붙인 꼭짓점과 주변 기준 바닥 꼭짓점 각각의 국소 바닥 평면(주변 기준 바닥 300점, 잘린 최소제곱) 대비 높이 차.
사용: patch_merge_flat.py <base_mesh> <flat_patch> <out_mesh>   (~/tsdf_env)"""
import sys
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
base, patch, out = sys.argv[1:4]
A = o3d.io.read_triangle_mesh(base); B = o3d.io.read_triangle_mesh(patch)
VA = np.asarray(A.vertices); VB, FB = np.asarray(B.vertices), np.asarray(B.triangles)
keep = np.isinf(cKDTree(VA).query(VB[FB].mean(1), distance_upper_bound=0.15)[0])
P = o3d.geometry.TriangleMesh(B.vertices, o3d.utility.Vector3iVector(FB[keep])); P.remove_unreferenced_vertices()
M = A + P; M.remove_duplicated_vertices(); M.compute_vertex_normals(); o3d.io.write_triangle_mesh(out, M)
print(f'조각 삼각형 {len(FB):,} 중 빈 곳 {keep.sum():,} 추가 → {out}')
# 검증
A.compute_vertex_normals(); NA = np.asarray(A.vertex_normals); VP = np.asarray(P.vertices)
lo, hi = VP[:, :2].min(0) - 4, VP[:, :2].max(0) + 4
g = (np.abs(NA[:, 2]) > 0.95) & (VA[:, 2] > -1.6) & (VA[:, 2] < -1.0) & np.all((VA[:, :2] > lo) & (VA[:, :2] < hi), 1); G = VA[g]
tG = cKDTree(G[:, :2])
def dev(X, exclude_self):
    d, j = tG.query(X[:, :2], k=301 if exclude_self else 300); j = j[:, 1:] if exclude_self else j; r = np.empty(len(X))
    for i in range(len(X)):
        Q = G[j[i]]; A_ = np.c_[Q[:, :2], np.ones(len(Q))]; w = np.ones(len(Q), bool)
        for _ in range(3):
            c = np.linalg.lstsq(A_[w], Q[w, 2], rcond=None)[0]; e = Q[:, 2] - A_ @ c; w = np.abs(e) < max(3 * np.median(np.abs(e[w])), 0.02)
        r[i] = X[i, 2] - c @ [X[i, 0], X[i, 1], 1]
    return r
dp = dev(VP, False)
ring = G[np.isinf(cKDTree(VP[:, :2]).query(G[:, :2], distance_upper_bound=1.0)[0]) & (cKDTree(VP[:, :2]).query(G[:, :2])[0] < 3.0)]
ring = ring[np.random.default_rng(0).choice(len(ring), min(len(ring), 4000), replace=False)]
G_ = G; dg = dev(ring, True)
for nm, r in (('붙인 조각', dp), ('주변 기존 바닥(1~3m 둘레, 기준)', dg)):
    print(f'{nm}: 꼭짓점 {len(r):,}, 국소 바닥 평면 대비 |차| 중앙 {np.median(np.abs(r))*100:.1f}cm, 90% {np.percentile(np.abs(r),90)*100:.1f}cm, 5cm 넘는 비율 {np.mean(np.abs(r)>0.05)*100:.0f}%')
