#!/usr/bin/env python3
"""C용 패키지 검증: 키프레임(RGB, depth mm, 노드별 보정 yaml) + 노드 ID 카메라 궤적으로 Open3D TSDF 복원.
복원 메시 꼭짓점과 RTAB-Map 점군 사이 거리로 정합 여부를 판정.
사용: tsdf_check.py <run> [voxel=0.05] [depth_trunc=6]"""
import sys, re
from pathlib import Path
import numpy as np, open3d as o3d
run = sys.argv[1]; vox = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05; trunc = float(sys.argv[3]) if len(sys.argv) > 3 else 6
E = Path.home() / f'b_slam/exports/{run}'; K = E / 'keyframes'
traj = np.loadtxt(E / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)  # stamp x y z qx qy qz qw id
def K_of(i):
    t = (K / f'kf_calib/{i}.yaml').read_text()
    m = re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', t, re.S); a = [float(x) for x in m.group(1).replace('\n', ' ').split(',')]
    return o3d.camera.PinholeCameraIntrinsic(640, 480, a[0], a[4], a[2], a[5])
vol = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=vox, sdf_trunc=4 * vox,
                                                    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
used = 0
for row in traj:
    i = int(row[8]); rgb = K / f'kf_rgb/{i}.jpg'; dep = K / f'kf_depth/{i}.png'
    if not (rgb.exists() and dep.exists()): continue
    T = np.eye(4); T[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion([row[7], row[4], row[5], row[6]]); T[:3, 3] = row[1:4]
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.io.read_image(str(rgb)), o3d.io.read_image(str(dep)),
                                                              depth_scale=1000.0, depth_trunc=trunc, convert_rgb_to_intensity=False)
    vol.integrate(rgbd, K_of(i), np.linalg.inv(T)); used += 1
mesh = vol.extract_triangle_mesh(); mesh.compute_vertex_normals()
out = E / f'tsdf_check_{int(vox*100)}cm.ply'; o3d.io.write_triangle_mesh(str(out), mesh)
V = np.asarray(mesh.vertices)
cloud = o3d.io.read_point_cloud(str(E / 'cloud_cloud.ply'))
d = np.asarray(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(V)).compute_point_cloud_distance(cloud))
print(f'키프레임 {used}/{len(traj)} 통합, 메시 꼭짓점 {len(V)}, 삼각형 {len(mesh.triangles)} → {out}')
print(f'메시→RTAB-Map 점군 거리: 중앙값 {np.median(d)*100:.1f}cm, 90% {np.percentile(d,90)*100:.1f}cm, 99% {np.percentile(d,99)*100:.1f}cm')
print('판정:', 'OK (궤적·내부파라미터·depth 단위가 점군과 일치)' if np.median(d) < 2 * vox else 'NG (좌표계/단위 불일치 의심)')
