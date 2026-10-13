#!/usr/bin/env python3
"""중력 정렬된 두 맵(점군)을 4자유도(x, y, z, yaw)로 맞춘다. 원본은 읽기만 한다.
1) 바닥 위 0.3~2.5m 높이 점(차·연석·화단·벽)을 0.2m 격자 2D 점유 지도로
2) yaw 0~360°(1°)마다 FFT 상관으로 최적 (dx, dy) → 최고 점수 yaw
3) Open3D ICP(point-to-plane)로 미세 조정, 결과 회전의 기울기(롤·피치) 확인
사용: align_maps.py <ref_run> <src_run> <out_dir>   (src를 ref 좌표계로 옮기는 변환을 구함)"""
import sys, json
from pathlib import Path
import numpy as np, open3d as o3d
ref_run, src_run, out = sys.argv[1], sys.argv[2], Path(sys.argv[3]); out.mkdir(parents=True, exist_ok=True)
H = Path.home() / 'b_slam/exports'; CELL = 0.2
def load(run):
    P = np.asarray(o3d.io.read_point_cloud(str(H / run / 'cloud_cloud.ply')).points)
    T = np.loadtxt(H / run / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]
    i = np.concatenate([np.argmin(((P[j:j+20000, None, :2] - T[None, :, :2]) ** 2).sum(-1), 1) for j in range(0, len(P), 20000)])
    h = 1.51 - (T[i, 2] - P[:, 2])  # 바닥 기준 높이(카메라 높이 1.51m 가정)
    return P, h
def occ(xy, lo, shape):
    g = np.zeros(shape); u = ((xy - lo) / CELL).astype(int)
    k = (u[:, 0] >= 0) & (u[:, 0] < shape[0]) & (u[:, 1] >= 0) & (u[:, 1] < shape[1]); g[u[k, 0], u[k, 1]] = 1; return g
Pr, hr = load(ref_run); Ps, hs = load(src_run)
Fr = Pr[(hr > 0.3) & (hr < 2.5), :2]; Fs = Ps[(hs > 0.3) & (hs < 2.5), :2]
cs = Fs.mean(0); Fs0 = Fs - cs
R = max(np.abs(Fs0).max(), 1) + 1
lo = Fr.min(0) - R; hi = Fr.max(0) + R; shape = tuple(np.ceil((hi - lo) / CELL).astype(int) + 1)
A = np.fft.rfft2(occ(Fr, lo, shape))
best = []
for deg in np.arange(0, 360, 1.0):
    t = np.radians(deg); Rm = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    B = occ(Fs0 @ Rm.T - lo + lo, np.array([-R, -R]), shape)  # src 중심을 격자 원점(-R,-R) 기준에 둠
    c = np.fft.irfft2(A * np.conj(np.fft.rfft2(B)), s=shape)
    k = np.unravel_index(np.argmax(c), c.shape); best.append((c[k], deg, k))
best.sort(key=lambda x: -x[0]); s0, deg, k = best[0]
second = next(b for b in best if abs((b[1] - deg + 180) % 360 - 180) > 10)
t = np.radians(deg); Rm = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
shift = np.array(k) * CELL + lo - np.array([-R, -R])  # 격자 이동량 → 월드 이동
# src 점 p → Rm(p - cs) + shift
T0 = np.eye(4); T0[:2, :2] = Rm; T0[:2, 3] = shift - Rm @ cs
# 높이 차: 겹치는 영역 바닥 높이 중앙값 차이로 근사
print(f'2D 상관: 최적 yaw {deg:.0f}° 점수 {s0:.0f} (다음 후보 yaw {second[1]:.0f}° 점수 {second[0]:.0f}, 비율 {s0/second[0]:.2f})')
src = o3d.io.read_point_cloud(str(H / src_run / 'cloud_cloud.ply')).voxel_down_sample(0.1)
ref = o3d.io.read_point_cloud(str(H / ref_run / 'cloud_cloud.ply')).voxel_down_sample(0.1)
for pc in (src, ref): pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
ev0 = o3d.pipelines.registration.evaluate_registration(src, ref, 0.5, T0)
T = T0
for d in (1.0, 0.5, 0.25):
    r = o3d.pipelines.registration.registration_icp(src, ref, d, T, o3d.pipelines.registration.TransformationEstimationPointToPlane())
    T = r.transformation
ev = o3d.pipelines.registration.evaluate_registration(src, ref, 0.25, T)
tilt = np.degrees(np.arccos(np.clip(T[2, 2], -1, 1)))
yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
print(f'초기(2D) fitness@0.5m {ev0.fitness:.3f} → ICP 후 fitness@0.25m {ev.fitness:.3f}, inlier RMSE {ev.inlier_rmse*100:.1f}cm')
print(f'최종 변환: yaw {yaw:.1f}°, 기울기 {tilt:.2f}°, 이동 ({T[0,3]:.2f}, {T[1,3]:.2f}, {T[2,3]:.2f}) m')
np.savetxt(out / 'T_src_to_ref.txt', T)
json.dump({'ref': ref_run, 'src': src_run, 'yaw2d_deg': float(deg), 'peak_ratio': float(s0 / second[0]),
           'fitness_init_0.5m': ev0.fitness, 'fitness_icp_0.25m': ev.fitness, 'rmse_cm': ev.inlier_rmse * 100,
           'tilt_deg': float(tilt)}, open(out / 'align_report.json', 'w'), indent=2)
# 겹쳐 그린 탑뷰: ref 회색, src(변환 후) 주황
S = np.asarray(o3d.io.read_point_cloud(str(H / src_run / 'cloud_cloud.ply')).transform(T).points); Rp = Pr
allxy = np.vstack([Rp[:, :2], S[:, :2]]); lo2 = allxy.min(0) - 1; s = 10
w, h = (np.ceil((allxy.max(0) + 1 - lo2) * s)).astype(int) + 1
img = np.full((h, w, 3), 255, np.uint8)
for Q, col in ((Rp, (120, 120, 120)), (S, (240, 140, 0))):
    u = ((Q[:, 0] - lo2[0]) * s).astype(int); v = (h - 1 - (Q[:, 1] - lo2[1]) * s).astype(int); img[v, u] = col
img[h - 12:h - 9, 10:110] = 0
o3d.io.write_image(str(out / 'overlay_topview.png'), o3d.geometry.Image(np.ascontiguousarray(img)))
print(f'저장: {out}/T_src_to_ref.txt, align_report.json, overlay_topview.png (회색=ref, 주황=src, 막대 10m)')
