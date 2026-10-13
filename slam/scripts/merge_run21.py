#!/usr/bin/env python3
"""v0.3용 합친 실행 폴더 exports/run21 만들기 (run18 = 기준, run20nr = 정합된 반시계)
 - traj_optimized_poses.txt : camera_link 자세, 시계 + 반시계 (원본 depth TSDF용, 시각으로 bag 프레임과 짝지음)
 - traj_camera_id_camera_poses.txt : 광학 자세 + ID (반시계 ID는 +10000), keyframes/kf_* 도 같은 ID로
 - traj_camera_id_cw.txt / traj_camera_id_ccw.txt : 방향별 분리본
 - cloud_cloud.ply : 시계 점군 전체 + 반시계 점 중 시계 점과 10cm 안에 없는 것만 (반시계 키프레임에서 생성: depth 2배 간축, 6m 이내, 3cm 복셀)
(~/tsdf_env)"""
import re, shutil
from pathlib import Path
import numpy as np, open3d as o3d, cv2
from scipy.spatial.transform import Rotation as Rot

H = Path.home() / 'b_slam/exports'; A, B, O = H / 'run18', H / 'run20nr', H / 'run21'
if O.exists(): raise SystemExit(f'이미 존재: {O}')
OFF = 10000
for sub in ('kf_rgb', 'kf_depth', 'kf_calib'): (O / 'keyframes' / sub).mkdir(parents=True)
for f in (A / 'keyframes').glob('*/*'): shutil.copy2(f, O / 'keyframes' / f.parent.name / f.name)
K20 = H / 'run20/keyframes'
for f in K20.glob('*/*'): shutil.copy2(f, O / 'keyframes' / f.parent.name / f'{int(f.stem) + OFF}{f.suffix}')

def rows(p): return [l for l in Path(p).read_text().splitlines() if l and not l.startswith('#')]
cw_id, ccw_id = rows(A / 'traj_camera_id_camera_poses.txt'), rows(B / 'traj_camera_id_camera_poses.txt')
ccw_id = [' '.join(l.split()[:8] + [str(int(l.split()[8]) + OFF)]) for l in ccw_id]
hdr_id, hdr = '#timestamp x y z qx qy qz qw id', '#timestamp x y z qx qy qz qw'
(O / 'traj_camera_id_camera_poses.txt').write_text('\n'.join([hdr_id] + cw_id + ccw_id) + '\n')
(O / 'traj_camera_id_cw.txt').write_text('\n'.join([hdr_id] + cw_id) + '\n')
(O / 'traj_camera_id_ccw.txt').write_text('\n'.join([hdr_id] + ccw_id) + '\n')
(O / 'traj_optimized_poses.txt').write_text('\n'.join([hdr] + rows(A / 'traj_optimized_poses.txt') + rows(B / 'traj_optimized_poses.txt')) + '\n')
(O / 'traj_optimized_cw.txt').write_text('\n'.join([hdr] + rows(A / 'traj_optimized_poses.txt')) + '\n')
(O / 'traj_optimized_ccw.txt').write_text('\n'.join([hdr] + rows(B / 'traj_optimized_poses.txt')) + '\n')
shutil.copy2(A / 'traj_odom_poses.txt', O / 'traj_odom_poses.txt')  # 탑뷰용 (시계 오도메트리)

# 반시계 점군 (새 자세) → 시계 점이 없는 곳만 추가
P, C = [], []
for l in ccw_id:
    v = l.split(); i = int(v[8]); T = np.eye(4); T[:3, :3] = Rot.from_quat([float(x) for x in v[4:8]]).as_matrix(); T[:3, 3] = [float(x) for x in v[1:4]]
    t = (O / f'keyframes/kf_calib/{i}.yaml').read_text()
    a = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', t, re.S).group(1).replace('\n', ' ').split(',')]
    d = cv2.imread(str(O / f'keyframes/kf_depth/{i}.png'), -1)[::2, ::2].astype(float) / 1000; rgb = cv2.imread(str(O / f'keyframes/kf_rgb/{i}.jpg'))[::2, ::2, ::-1]
    vv, uu = np.nonzero((d > 0.3) & (d < 6)); z = d[vv, uu]
    X = np.c_[(uu * 2 - a[2]) * z / a[0], (vv * 2 - a[5]) * z / a[4], z]
    P.append(X @ T[:3, :3].T + T[:3, 3]); C.append(rgb[vv, uu] / 255.0)
ccw = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.vstack(P))); ccw.colors = o3d.utility.Vector3dVector(np.vstack(C))
ccw = ccw.voxel_down_sample(0.03)
cw = o3d.io.read_point_cloud(str(A / 'cloud_cloud.ply'))
d = np.asarray(ccw.compute_point_cloud_distance(cw)); add = ccw.select_by_index(np.nonzero(d > 0.10)[0])
add.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
if not cw.has_normals(): cw.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
merged = cw + add
o3d.io.write_point_cloud(str(O / 'cloud_cloud.ply'), merged, write_ascii=False)
o3d.io.write_point_cloud(str(O / 'cloud_ccw_added.ply'), add, write_ascii=False)
print(f'키프레임: 시계 {len(cw_id)} + 반시계 {len(ccw_id)} | 점군: 시계 {len(cw.points)} + 반시계 추가 {len(add.points)} (반시계 전체 {len(ccw.points)}의 {len(add.points)/len(ccw.points)*100:.0f}%) → {O}')
