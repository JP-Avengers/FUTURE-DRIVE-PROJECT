#!/usr/bin/env python3
"""새 맵 점군(exports/<src>/cloud_cloud.ply, 자기 좌표계)을 비강체 정합 결과(exports/<nr>/pose_delta.npy, kf_xy_src.npy)로 v0.3 좌표계로 옮김.
각 점은 수평으로 가장 가까운 키프레임의 (새 자세)·(원래 자세)^-1 변환을 따름(nonrigid_to_v03.py 검증과 같은 방식).
사용: warp_cloud.py <src_run> <nr_dir> <out.ply>   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
E = Path.home() / 'b_slam/exports'; src, nr, out = sys.argv[1:4]
pc = o3d.io.read_point_cloud(str(E / src / 'cloud_cloud.ply')); X = np.asarray(pc.points)
D = np.load(E / nr / 'pose_delta.npy'); kxy = np.load(E / nr / 'kf_xy_src.npy')
_, i = cKDTree(kxy).query(X[:, :2]); Y = np.einsum('nij,nj->ni', D[i, :3, :3], X) + D[i, :3, 3]
pc.points = o3d.utility.Vector3dVector(Y); o3d.io.write_point_cloud(out, pc)
print(f'{src} → {nr}: 점 {len(Y):,} → {out}')
