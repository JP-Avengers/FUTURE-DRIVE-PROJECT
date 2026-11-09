"""feature_* 회차를 초기 변환(사진 매칭)에서 v0.5 점군에 ICP(점-면, 1.0→0.4→0.15m)로 다듬고, 경로 15m 구간별 어긋남을 잼.
구간 어긋남 = 그 구간 키프레임 근처(3m) src 점 중 ref 0.5m 안 점의 거리 중앙값, 겹침 = ref 0.3m 안 비율.
사용: fe_icp.py <run>:<T_init.txt> ...   → exports/align_<run>_x/T_src_to_ref.npy, report.txt"""
import sys, numpy as np, open3d as o3d
from pathlib import Path
from scipy.spatial import cKDTree
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'
ref = o3d.io.read_point_cloud(str(P / 'cloud_3cm.ply')).voxel_down_sample(0.05)
ref.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30)); R = np.asarray(ref.points); rt = cKDTree(R)
for spec in sys.argv[1:]:
    run, tf = spec.split(':'); T0 = np.loadtxt(tf) if tf.endswith('.txt') else np.load(tf)
    src = o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).voxel_down_sample(0.05)
    T = T0.copy()
    for th in (1.0, 0.4, 0.15):
        r = o3d.pipelines.registration.registration_icp(src, ref, th, T, o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
        T = r.transformation
    X = np.asarray(src.points) @ T[:3, :3].T + T[:3, 3]; d, _ = rt.query(X, distance_upper_bound=0.5)
    tr = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4] @ T[:3, :3].T + T[:3, 3]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tr, axis=0), axis=1))]; seg = (s // 15).astype(int)
    _, ki = cKDTree(tr).query(X); ps = seg[ki]; near = np.linalg.norm(X - tr[ki], axis=1) < 3
    dT = T @ np.linalg.inv(T0); mv = np.linalg.norm(dT[:3, 3]); ang = np.degrees(np.arccos(np.clip((np.trace(dT[:3, :3]) - 1) / 2, -1, 1)))
    lines = [f'{run}: ICP fitness {r.fitness:.2f}, RMSE {r.inlier_rmse*100:.1f}cm, 초기값에서 이동 {mv:.2f}m·회전 {ang:.1f}°, 전체 겹침(0.3m 안) {np.mean(d < 0.3):.0%}, 겹친 점 거리 중앙 {np.median(d[d < 0.5])*100:.1f}cm']
    for g in range(seg.max() + 1):
        k = (ps == g) & near
        if k.sum() < 200: lines.append(f'   {g*15:3d}~{g*15+15:3d}m: 점 부족'); continue
        dd = d[k]; ok = dd < 0.5
        lines.append(f'   {g*15:3d}~{g*15+15:3d}m: 겹침 {np.mean(dd < 0.3):4.0%}, 겹친 점 거리 중앙 {np.median(dd[ok])*100 if ok.any() else float("nan"):5.1f}cm')
    out = E / f'align_{run}_x'; out.mkdir(exist_ok=True); np.save(out / 'T_src_to_ref.npy', T); (out / 'report.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines))
