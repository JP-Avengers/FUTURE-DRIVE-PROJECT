#!/usr/bin/env python3
"""v0.3 좌표계 메시·점군에서, 수평으로 가장 가까운 키프레임이 '좋음'(kf_good.npy, kf_consistency.py 결과)인 부분만 남김.
메시는 삼각형 무게중심 기준. 사용: crop_by_kf.py <nr_dir> <in.ply> <out.ply>   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
E = Path.home() / 'b_slam/exports'; nr, src, out = sys.argv[1:4]
K = np.loadtxt(E / nr / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)[:, 1:3]; good = np.load(E / nr / 'kf_good.npy'); t = cKDTree(K)
m = o3d.io.read_triangle_mesh(src)
if len(m.triangles):
    V, F = np.asarray(m.vertices), np.asarray(m.triangles); _, i = t.query(V[F].mean(1)[:, :2]); k = good[i]
    p = o3d.geometry.TriangleMesh(m.vertices, o3d.utility.Vector3iVector(F[k])); p.remove_unreferenced_vertices(); p.compute_vertex_normals()
    o3d.io.write_triangle_mesh(out, p); print(f'{nr}: 삼각형 {len(F):,} 중 {k.sum():,} 남김 ({k.mean()*100:.0f}%) → {out}')
else:
    c = o3d.io.read_point_cloud(src); X = np.asarray(c.points); _, i = t.query(X[:, :2]); k = good[i]
    c2 = c.select_by_index(np.nonzero(k)[0]); o3d.io.write_point_cloud(out, c2); print(f'{nr}: 점 {len(X):,} 중 {k.sum():,} 남김 ({k.mean()*100:.0f}%) → {out}')
