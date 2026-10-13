#!/usr/bin/env python3
"""FPFH + RANSAC 전역 정합 후 ICP. 여러 시드로 반복해 결과가 일관적인지 본다.
기울기(롤·피치)가 3° 넘으면 기각(두 맵 모두 중력 정렬이므로). 원본은 읽기만 함.
사용: align_fpfh.py <ref_run> <src_run> <out_dir> [voxel=0.25]"""
import sys, json
from pathlib import Path
import numpy as np, open3d as o3d
ref_run, src_run, out = sys.argv[1], sys.argv[2], Path(sys.argv[3]); v = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25
out.mkdir(parents=True, exist_ok=True); H = Path.home() / 'b_slam/exports'
def prep(run):
    pc = o3d.io.read_point_cloud(str(H / run / 'cloud_cloud.ply')).voxel_down_sample(v)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=2 * v, max_nn=30))
    f = o3d.pipelines.registration.compute_fpfh_feature(pc, o3d.geometry.KDTreeSearchParamHybrid(radius=5 * v, max_nn=100))
    return pc, f
ref, fr = prep(ref_run); src, fs = prep(src_run)
print(f'점 수 ref {len(ref.points)}, src {len(src.points)} (voxel {v}m)')
res = []
for seed in range(8):
    o3d.utility.random.seed(seed)
    r = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src, ref, fs, fr, True, 1.5 * v, o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9), o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(1.5 * v)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(400000, 0.999))
    T = r.transformation
    for d in (4 * v, 2 * v, v):
        T = o3d.pipelines.registration.registration_icp(src, ref, d, T, o3d.pipelines.registration.TransformationEstimationPointToPlane()).transformation
    ev = o3d.pipelines.registration.evaluate_registration(src, ref, v, T)
    tilt = np.degrees(np.arccos(np.clip(T[2, 2], -1, 1))); yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
    res.append((ev.fitness, ev.inlier_rmse, tilt, yaw, T))
    print(f' seed {seed}: fitness {ev.fitness:.3f}, rmse {ev.inlier_rmse*100:.1f}cm, 기울기 {tilt:.1f}°, yaw {yaw:.1f}°, 이동 ({T[0,3]:.1f}, {T[1,3]:.1f}, {T[2,3]:.1f})')
ok = [r for r in res if r[2] < 3]
best = max(ok or res, key=lambda r: r[0])
np.savetxt(out / 'T_src_to_ref.txt', best[4])
json.dump({'method': 'fpfh_ransac_icp', 'voxel': v, 'fitness': best[0], 'rmse_cm': best[1] * 100, 'tilt_deg': best[2], 'yaw_deg': best[3],
           'n_seeds': len(res), 'n_tilt_ok': len(ok)}, open(out / 'align_report.json', 'w'), indent=2)
S = np.asarray(o3d.io.read_point_cloud(str(H / src_run / 'cloud_cloud.ply')).transform(best[4]).points)
Rp = np.asarray(o3d.io.read_point_cloud(str(H / ref_run / 'cloud_cloud.ply')).points)
allxy = np.vstack([Rp[:, :2], S[:, :2]]); lo = allxy.min(0) - 1; s = 10
w, h = (np.ceil((allxy.max(0) + 1 - lo) * s)).astype(int) + 1
img = np.full((h, w, 3), 255, np.uint8)
for Q, col in ((Rp, (120, 120, 120)), (S, (240, 140, 0))):
    u = ((Q[:, 0] - lo[0]) * s).astype(int); vv = (h - 1 - (Q[:, 1] - lo[1]) * s).astype(int); img[vv, u] = col
img[h - 12:h - 9, 10:110] = 0
o3d.io.write_image(str(out / 'overlay_topview.png'), o3d.geometry.Image(np.ascontiguousarray(img)))
print(f'최선: fitness {best[0]:.3f}, rmse {best[1]*100:.1f}cm, 기울기 {best[2]:.1f}°, yaw {best[3]:.1f}° → {out}')
