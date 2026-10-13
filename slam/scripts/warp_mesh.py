#!/usr/bin/env python3
"""자기 좌표계 메시(원래 자세로 만든 TSDF)를 비강체 정합 결과(exports/<nr>/pose_delta.npy, kf_xy_src.npy)로 꼭짓점마다 옮김.
각 꼭짓점은 수평으로 가장 가까운 키프레임의 (새 자세)·(원래 자세)^-1 변환을 따름. 휜 자세로 TSDF를 다시 만들면
겹친 키프레임끼리 어긋나 얇은 바닥이 지워질 수 있어, 매끈한 원래 메시를 옮기는 쪽을 씀.
사용: warp_mesh.py <in_mesh(자기 좌표)> <nr_dir> <out.ply>   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
E = Path.home() / 'b_slam/exports'; src, nr, out = sys.argv[1:4]
m = o3d.io.read_triangle_mesh(src); V = np.asarray(m.vertices)
D = np.load(E / nr / 'pose_delta.npy'); kxy = np.load(E / nr / 'kf_xy_src.npy'); _, i = cKDTree(kxy).query(V[:, :2])
m.vertices = o3d.utility.Vector3dVector(np.einsum('nij,nj->ni', D[i, :3, :3], V) + D[i, :3, 3]); m.compute_vertex_normals()
o3d.io.write_triangle_mesh(out, m); print(f'{Path(src).name} → {nr}: 꼭짓점 {len(V):,} → {out}')
