"""구간별 ICP 비강체 정합 (손 촬영 회차용): 경로를 --seg m 구간(절반 겹침)으로 나눠 구간 점군을 v0.5 점군에 점-면 ICP(0.8→0.3→0.1m).
fitness ≥ 0.35·RMSE < 8cm·이웃 구간과 차이(1.5m, 6°) 안이면 채택. 키프레임별 변환 = 채택 구간 중심 사이 보간(이동 선형, 회전 slerp), 바깥은 가장 가까운 구간.
출력 exports/<out>/: traj_optimized_poses.txt·traj_camera_id_camera_poses.txt(변환된 자세, 원본과 같은 형식), cloud_cloud.ply(변환), keyframes(링크), report.txt
사용: seg_icp_warp.py <run> <T_init.npy> <out> [--seg 10]"""
import sys, numpy as np, open3d as o3d
from pathlib import Path
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot, Slerp
run, tinit, outn = sys.argv[1:4]; SEG = float(sys.argv[sys.argv.index('--seg') + 1]) if '--seg' in sys.argv else 10.0
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'; O = E / outn; O.mkdir(exist_ok=True)
T0 = np.load(tinit) if tinit.endswith('.npy') else np.loadtxt(tinit)
ref = o3d.io.read_point_cloud(str(P / 'cloud_3cm.ply')).voxel_down_sample(0.05); ref.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
R = np.asarray(ref.points); rt = cKDTree(R)
traj = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2)        # t x y z qx qy qz qw (src 좌표, camera_link)
pc = o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).voxel_down_sample(0.05); X = np.asarray(pc.points)
tp = traj[:, 1:4]; s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tp, axis=0), axis=1))]; _, ki = cKDTree(tp).query(X)
def tf(T, Y): return Y @ T[:3, :3].T + T[:3, 3]
segs = []; c = 0.0; prev = T0
while c < s[-1]:
    k = np.flatnonzero((s >= c) & (s < c + SEG)); c += SEG / 2
    if len(k) < 3: continue
    m = np.isin(ki, k); Y = X[m]
    if len(Y) < 3000: continue
    q = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Y)); T = prev.copy(); ok = True
    for th in (0.8, 0.3, 0.1):
        r = o3d.pipelines.registration.registration_icp(q, ref, th, T, o3d.pipelines.registration.TransformationEstimationPointToPlane(), o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)); T = r.transformation
    d = T @ np.linalg.inv(prev); dm = np.linalg.norm(d[:3, 3]); da = np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1)))
    acc = r.fitness >= 0.35 and r.inlier_rmse < 0.08 and dm < 1.5 and da < 6
    segs.append(dict(s0=c - SEG / 2, sc=float(np.mean(s[k])), T=T, fit=r.fitness, rmse=r.inlier_rmse, dm=dm, da=da, acc=acc))
    if acc: prev = T
A = [g for g in segs if g['acc']]
lines = [f'{run}: 구간 {len(segs)}개 중 채택 {len(A)}개 (구간 {SEG:g}m, 절반 겹침)']
for g in segs: lines.append(f"   {g['s0']:5.0f}~{g['s0']+SEG:5.0f}m: fitness {g['fit']:.2f}, RMSE {g['rmse']*100:4.1f}cm, 직전 채택 대비 {g['dm']:.2f}m·{g['da']:.1f}° {'채택' if g['acc'] else '버림'}")
if not A: A = [dict(sc=0.0, T=T0)]; lines.append('   채택 구간 없음 → 초기 변환 그대로')
sc = np.array([g['sc'] for g in A]); Ts = np.stack([g['T'] for g in A])
def T_at(si):
    if len(A) == 1 or si <= sc[0]: return Ts[0]
    if si >= sc[-1]: return Ts[-1]
    j = np.searchsorted(sc, si) - 1; w = (si - sc[j]) / (sc[j + 1] - sc[j])
    rot = Slerp([0, 1], Rot.from_matrix(Ts[j:j + 2, :3, :3]))(w).as_matrix(); t = (1 - w) * Ts[j, :3, 3] + w * Ts[j + 1, :3, 3]
    M = np.eye(4); M[:3, :3] = rot; M[:3, 3] = t; return M
Tk = np.stack([T_at(v) for v in s])
def warp_poses(rows, tcol=0):
    out = rows.copy()
    for i, r in enumerate(rows):
        j = np.argmin(np.abs(traj[:, 0] - r[tcol])); M = Tk[j]; Rm = M[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix()
        out[i, 1:4] = M[:3, :3] @ r[1:4] + M[:3, 3]; out[i, 4:8] = Rot.from_matrix(Rm).as_quat()
    return out
np.savetxt(O / 'traj_optimized_poses.txt', warp_poses(traj), fmt='%.6f', header='timestamp x y z qx qy qz qw', comments='#')
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
np.savetxt(O / 'traj_camera_id_camera_poses.txt', warp_poses(cid), fmt='%.6f', header='timestamp x y z qx qy qz qw id', comments='#')
Xw = np.stack([Tk[i][:3, :3] @ x + Tk[i][:3, 3] for i, x in zip(ki, X)]) if len(X) < 0 else None
Xw = np.einsum('nij,nj->ni', Tk[ki][:, :3, :3], X) + Tk[ki][:, :3, 3]
pc.points = o3d.utility.Vector3dVector(Xw); o3d.io.write_point_cloud(str(O / 'cloud_cloud.ply'), pc)
if not (O / 'keyframes').exists(): (O / 'keyframes').symlink_to(E / run / 'keyframes')
d, _ = rt.query(Xw, distance_upper_bound=0.5); seg = (s[ki] // 15).astype(int); near = np.linalg.norm(Xw - (np.einsum('nij,nj->ni', Tk[ki][:, :3, :3], tp[ki]) + Tk[ki][:, :3, 3]), axis=1) < 3
lines.append(f'결과: 전체 겹침(0.3m 안) {np.mean(d < 0.3):.0%}, 겹친 점 거리 중앙 {np.median(d[d < 0.5])*100:.1f}cm')
for g in range(seg.max() + 1):
    k = (seg == g) & near
    if k.sum() < 200: continue
    dd = d[k]; okk = dd < 0.5; lines.append(f'   {g*15:3d}~{g*15+15:3d}m: 겹침 {np.mean(dd < 0.3):4.0%}, 겹친 점 거리 중앙 {np.median(dd[okk])*100 if okk.any() else float("nan"):5.1f}cm')
(O / 'report.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8'); print('\n'.join(lines))
