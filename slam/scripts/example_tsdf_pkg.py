#!/usr/bin/env python3
"""키프레임으로 컬러 TSDF 메시 만들기 (이 패키지 폴더 안에서 실행).
입력: keyframes/{rgb,depth,calib}/<id>.*  +  trajectory_camera_optical.txt (TUM + 마지막 열 = 키프레임 ID)
      --ccw 를 주면 keyframes_ccw/ + trajectory_ccw_camera_optical.txt 도 함께 넣음
출력: tsdf_<복셀cm>cm[_ccw].ply, 그리고 cloud_3cm.ply 와의 거리(정합 확인)
사용: python example_tsdf.py [복셀m=0.05] [depth자르기m=10] [--ccw]
필요: pip install "open3d==0.19.0" numpy   (Open3D 0.20.0은 ScalableTSDFVolume이 빈 메시를 내므로 쓰지 말 것)"""
import sys, re
from pathlib import Path
import numpy as np, open3d as o3d

args = [a for a in sys.argv[1:] if not a.startswith('--')]
vox = float(args[0]) if len(args) > 0 else 0.05; trunc = float(args[1]) if len(args) > 1 else 10.0
use_ccw = '--ccw' in sys.argv
D = Path(__file__).resolve().parent
sets = [(D / 'keyframes', D / 'trajectory_camera_optical.txt')]
if use_ccw: sets.append((D / 'keyframes_ccw', D / 'trajectory_ccw_camera_optical.txt'))

def intrinsic(path):
    t = path.read_text(); a = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', t, re.S).group(1).replace('\n', ' ').split(',')]
    return o3d.camera.PinholeCameraIntrinsic(640, 480, a[0], a[4], a[2], a[5])

vol = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=vox, sdf_trunc=4 * vox, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
used = total = 0
for K, traj in sets:
    for row in np.loadtxt(traj, comments='#', ndmin=2):
        i = int(row[8]); total += 1
        rgb, dep, cal = K / f'rgb/{i}.jpg', K / f'depth/{i}.png', K / f'calib/{i}.yaml'
        if not (rgb.exists() and dep.exists() and cal.exists()): continue
        T = np.eye(4); T[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion([row[7], row[4], row[5], row[6]]); T[:3, 3] = row[1:4]
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.io.read_image(str(rgb)), o3d.io.read_image(str(dep)),
                                                                  depth_scale=1000.0, depth_trunc=trunc, convert_rgb_to_intensity=False)
        vol.integrate(rgbd, intrinsic(cal), np.linalg.inv(T)); used += 1
mesh = vol.extract_triangle_mesh(); mesh.compute_vertex_normals()
out = D / f'tsdf_{round(vox * 100)}cm{"_ccw" if use_ccw else ""}.ply'; o3d.io.write_triangle_mesh(str(out), mesh)
print(f'키프레임 {used}/{total}장 통합 → 꼭짓점 {len(mesh.vertices):,}, 삼각형 {len(mesh.triangles):,} → {out.name}')
cloud = o3d.io.read_point_cloud(str(D / 'cloud_3cm.ply'))
if len(cloud.points):
    V = o3d.geometry.PointCloud(mesh.vertices); d = np.asarray(V.compute_point_cloud_distance(cloud))
    near = d < 1.0
    print(f'메시 → cloud_3cm.ply 거리 (점군이 있는 곳, 1m 이내): 중앙 {np.median(d[near]) * 100:.1f}cm, 90% {np.percentile(d[near], 90) * 100:.1f}cm')
