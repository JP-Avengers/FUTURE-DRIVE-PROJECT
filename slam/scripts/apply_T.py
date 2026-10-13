#!/usr/bin/env python3
"""4x4 변환을 PLY(메시·점군)나 TUM 궤적에 적용. 사용: apply_T.py <T.npy> <in> <out>
TUM(.txt): 위치는 T로, 자세는 회전을 곱함. 마지막 열(ID)이 있으면 유지."""
import sys
import numpy as np, open3d as o3d
from scipy.spatial.transform import Rotation as Rot
T = np.load(sys.argv[1]); src, dst = sys.argv[2], sys.argv[3]
if src.endswith('.txt'):
    A = np.loadtxt(src, comments='#', ndmin=2)
    A[:, 1:4] = A[:, 1:4] @ T[:3, :3].T + T[:3, 3]
    A[:, 4:8] = (Rot.from_matrix(T[:3, :3]) * Rot.from_quat(A[:, 4:8])).as_quat()
    hdr = open(src).readline().strip() if open(src).readline().startswith('#') else ''
    fmt = ['%.6f'] + ['%.6f'] * 7 + (['%d'] if A.shape[1] > 8 else [])
    np.savetxt(dst, A, fmt=fmt, header=hdr.lstrip('# '), comments='#')
else:
    m = o3d.io.read_triangle_mesh(src)
    if len(m.triangles):
        m.transform(T); m.compute_vertex_normals(); o3d.io.write_triangle_mesh(dst, m)
    else:
        p = o3d.io.read_point_cloud(src); p.transform(T); o3d.io.write_point_cloud(dst, p)
print(f'{src} → {dst}')
