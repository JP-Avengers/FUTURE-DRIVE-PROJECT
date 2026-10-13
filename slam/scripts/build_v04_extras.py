#!/usr/bin/env python3
"""v0.4 패키지에 새 촬영(10/9) 데이터 추가: 키프레임·궤적(v0.3 좌표계, ID 오프셋)과 점군 합치기.
 - keyframes_new_cw/  : gn03 키프레임 (ID +20000), keyframes_new_ccw/ : gm03 키프레임 (ID +30000)
 - trajectory_new_cw_camera_optical.txt / _camera_link.txt, trajectory_new_ccw_* : v0.3 좌표계로 옮긴 자세
 - cloud_3cm.ply : v0.3 점군 그대로 + 새 시계 점 중 기존 점과 10cm 안에 없는 것 + 새 반시계 점 중 (v0.3+새 시계)와 10cm 안에 없는 것
사용: build_v04_extras.py <pkg_dir>   (~/tsdf_env)"""
import sys, re, shutil
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
P = Path(sys.argv[1]); E = Path.home() / 'b_slam/exports'; W = E / 'v04work'
V03 = Path.home() / 'b_slam/deliver/map_v0.3'
for run, tag, off in (('gn03', 'new_cw', 20000), ('gm03', 'new_ccw', 30000)):
    K = E / run / 'keyframes'; D = P / f'keyframes_{tag}'
    for sub, dst in (('kf_rgb', 'rgb'), ('kf_depth', 'depth'), ('kf_calib', 'calib')):
        (D / dst).mkdir(parents=True, exist_ok=True)
        for f in (K / sub).iterdir():
            g = D / dst / f'{int(f.stem) + off}{f.suffix}'
            if f.suffix == '.yaml':
                g.write_text(re.sub(r'camera_name: "(\d+)"', lambda m: f'camera_name: "{int(m.group(1)) + off}"', f.read_text()))
            else:
                shutil.copy2(f, g)
    A = np.loadtxt(W / f'{run}_traj_camera_id_camera_poses.txt', comments='#', ndmin=2); A[:, 8] += off
    np.savetxt(P / f'trajectory_{tag}_camera_optical.txt', A, fmt=['%.6f'] * 8 + ['%d'], header='timestamp x y z qx qy qz qw id', comments='#')
    shutil.copy2(W / f'{run}_traj_optimized_poses.txt', P / f'trajectory_{tag}_camera_link.txt')
    ids = set(A[:, 8].astype(int)); have = {int(f.stem) for f in (D / 'rgb').iterdir()}
    print(f'{tag}: 키프레임 {len(have)}장, 궤적 {len(A)}개, 궤적에만 있고 영상 없는 ID {len(ids - have)}개')
# 점군
base = o3d.io.read_point_cloud(str(V03 / 'cloud_3cm.ply')); pts = [np.asarray(base.points)]; cols = [np.asarray(base.colors)]
n0 = len(pts[0])
for run in ('gn03', 'gm03'):
    c = o3d.io.read_point_cloud(str(W / f'{run}_cloud_cloud.ply')); X = np.asarray(c.points)
    d, _ = cKDTree(np.vstack(pts)).query(X, distance_upper_bound=0.10); keep = np.isinf(d)
    pts.append(X[keep]); cols.append(np.asarray(c.colors)[keep])
    print(f'{run}: 점 {len(X):,} 중 새로 추가 {keep.sum():,}')
out = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.vstack(pts))); out.colors = o3d.utility.Vector3dVector(np.vstack(cols))
o3d.io.write_point_cloud(str(P / 'cloud_3cm.ply'), out)
print(f'cloud_3cm.ply: {n0:,} → {len(out.points):,}점')
