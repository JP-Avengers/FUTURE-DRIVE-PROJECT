import numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial import cKDTree
H = Path.home()/'b_slam/exports'
yaw, cx, cy, tx, ty = np.load('/home/owner7/b_slam/exports/tmp/coarse_T_266.npy'); t = np.radians(yaw)
R2 = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
T0 = np.eye(4); T0[:2, :2] = R2; T0[:2, 3] = np.array([tx, ty]) - R2 @ np.array([cx, cy])
G0 = np.load(H/'run18/ground_xyz.npy'); G1 = np.load(H/'run20/ground_xyz.npy')
g1 = G1 @ T0[:3, :3].T + T0[:3, 3]; d, j = cKDTree(G0[:, :2]).query(g1[:, :2]); m = d < 0.3
T0[2, 3] = np.median(G0[j[m], 2] - g1[m, 2]); print(f'초기: yaw {yaw:.0f}°, 이동 ({T0[0,3]:.2f}, {T0[1,3]:.2f}, {T0[2,3]:.2f}), 바닥 겹침 점 {m.mean()*100:.0f}%')
A = o3d.io.read_point_cloud(str(H/'run18/cloud_cloud.ply')); B = o3d.io.read_point_cloud(str(H/'run20/cloud_cloud.ply'))
T = T0.copy()
for vox, dist in ((0.4, 1.5), (0.2, 0.6), (0.1, 0.3)):
    a = A.voxel_down_sample(vox); b = B.voxel_down_sample(vox)
    for x in (a, b): x.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=vox * 3, max_nn=30))
    r = o3d.pipelines.registration.registration_icp(b, a, dist, T, o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    T = r.transformation; print(f'ICP 복셀 {vox}m: fitness {r.fitness:.3f}, 인라이어 RMSE {r.inlier_rmse*100:.1f}cm')
dT = np.linalg.inv(T0) @ T; ang = np.degrees(np.arccos(np.clip((np.trace(dT[:3, :3]) - 1) / 2, -1, 1)))
tilt = np.degrees(np.arccos(np.clip(T[2, 2], -1, 1)))
print(f'초기값 대비 보정: 회전 {ang:.2f}°, 이동 {np.linalg.norm(dT[:3,3]):.2f}m | 최종 yaw {np.degrees(np.arctan2(T[1,0],T[0,0]))%360:.2f}°, 기울기 {tilt:.2f}°')
np.save(H/'run20_to_run18.npy', T); np.savetxt(H/'run20_to_run18.txt', T, fmt='%.6f')
# 겹침 평가: run20 점 중 run18 점과 10cm 이내 비율 (겹치는 영역만), 그리고 새 영역 면적
P0 = np.asarray(A.voxel_down_sample(0.05).points); P1 = np.asarray(B.voxel_down_sample(0.05).points) @ T[:3, :3].T + T[:3, 3]
d = cKDTree(P0).query(P1)[0]
near = d < 1.0
print(f'run20 점 중 run18과 1m 이내(겹치는 영역) {near.mean()*100:.0f}% → 그중 거리 중앙 {np.median(d[near])*100:.1f}cm, 90% {np.percentile(d[near],90)*100:.1f}cm')
