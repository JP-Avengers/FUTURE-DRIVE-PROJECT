"""구간별 ICP 비강체 정합 v2 (손 촬영 회차용). v1에서 넓은 바닥 점이 ICP를 지배해 구간마다 수평으로 미끄러졌던 것을 고침:
  - 구간 점 중 수평 면(|nz|>0.8, 바닥·보도 윗면)은 10%만 남기고(높이 유지용) 세로 형상(연석·벽·계단·차)을 위주로 점-면 ICP (0.8→0.3→0.1m)
  - 구간마다 같은 초기값(전체 변환)에서 따로 풀고, 이웃 구간(앞뒤 2개) 해와 0.5m·3° 안으로 일치하는 구간만 채택(합의)
  - 세로 형상 점의 0.3m 안 비율(구조 겹침) ≥ 0.4, RMSE < 6cm
키프레임별 변환 = 채택 구간 중심 사이 보간(이동 선형, 회전 slerp), 바깥은 가장 가까운 구간.
출력 exports/<out>/: traj_optimized_poses.txt·traj_camera_id_camera_poses.txt(변환된 자세), cloud_cloud.ply, keyframes(링크), report.txt
사용: seg_icp_warp2.py <run> <T_init.npy|txt> <out> [--seg 10]"""
import sys, numpy as np, open3d as o3d
from pathlib import Path
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot, Slerp
run, tinit, outn = sys.argv[1:4]; SEG = float(sys.argv[sys.argv.index('--seg') + 1]) if '--seg' in sys.argv else 10.0
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'; O = E / outn; O.mkdir(exist_ok=True)
T0 = np.load(tinit) if tinit.endswith('.npy') else np.loadtxt(tinit)
ref = o3d.io.read_point_cloud(str(P / 'cloud_3cm.ply')).voxel_down_sample(0.05); ref.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
RN = np.abs(np.asarray(ref.normals)[:, 2]); Rv = np.asarray(ref.points)[RN < 0.6]; rvt = cKDTree(Rv); rt = cKDTree(np.asarray(ref.points))
traj = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2)
pc = o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).voxel_down_sample(0.05); pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
X = np.asarray(pc.points); NZ = np.abs(np.asarray(pc.normals)[:, 2]); vert = NZ < 0.6
rng = np.random.default_rng(0); keepm = vert | ((NZ > 0.8) & (rng.random(len(X)) < 0.10))
tp = traj[:, 1:4]; s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tp, axis=0), axis=1))]; _, ki = cKDTree(tp).query(X)
def tf(T, Y): return Y @ T[:3, :3].T + T[:3, 3]
def dT(A, B):
    d = A @ np.linalg.inv(B); return np.linalg.norm(d[:3, 3]), np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1)))
segs = []; c = 0.0
while c < s[-1]:
    k = np.flatnonzero((s >= c) & (s < c + SEG)); c0 = c; c += SEG / 2
    if len(k) < 3: continue
    m = np.isin(ki, k) & keepm; Y = X[m]; Yv = X[np.isin(ki, k) & vert]
    if len(Yv) < 800: segs.append(dict(s0=c0, sc=float(np.mean(s[k])), T=None, note='세로 형상 부족')); continue
    q = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Y)); T = T0.copy()
    for th in (0.8, 0.3, 0.1):
        r = o3d.pipelines.registration.registration_icp(q, ref, th, T, o3d.pipelines.registration.TransformationEstimationPointToPlane(), o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60)); T = r.transformation
    dv, _ = rvt.query(tf(T, Yv), distance_upper_bound=0.3); sov = float(np.mean(np.isfinite(dv)))
    segs.append(dict(s0=c0, sc=float(np.mean(s[k])), T=T, sov=sov, rmse=r.inlier_rmse, dm=dT(T, T0)[0], da=dT(T, T0)[1]))
# 합의: 구조 겹침·RMSE 통과 + 앞뒤 2구간 중 하나 이상과 0.5m·3° 안
ok = [g['T'] is not None and g['sov'] >= 0.4 and g['rmse'] < 0.06 for g in segs]
for i, g in enumerate(segs):
    g['acc'] = False
    if not ok[i]: continue
    for j in (i - 2, i - 1, i + 1, i + 2):
        if 0 <= j < len(segs) and ok[j] and max(dT(g['T'], segs[j]['T'])[0] / 0.5, dT(g['T'], segs[j]['T'])[1] / 3.0) <= 1: g['acc'] = True; break
A = [g for g in segs if g['acc']]
lines = [f'{run}: 구간 {len(segs)}개 중 채택 {len(A)}개 (구간 {SEG:g}m, 절반 겹침, 세로 형상 위주 ICP)']
for g in segs:
    if g['T'] is None: lines.append(f"   {g['s0']:5.0f}~{g['s0']+SEG:5.0f}m: {g['note']}"); continue
    lines.append(f"   {g['s0']:5.0f}~{g['s0']+SEG:5.0f}m: 구조 겹침 {g['sov']:.0%}, RMSE {g['rmse']*100:4.1f}cm, 초기값 대비 {g['dm']:.2f}m·{g['da']:.1f}° {'채택' if g['acc'] else '버림'}")
if not A: A = [dict(sc=0.0, T=T0)]; lines.append('   채택 구간 없음 → 초기 변환 그대로')
sc = np.array([g['sc'] for g in A]); Ts = np.stack([g['T'] for g in A])
def T_at(si):
    if len(A) == 1 or si <= sc[0]: return Ts[0]
    if si >= sc[-1]: return Ts[-1]
    j = np.searchsorted(sc, si) - 1; w = (si - sc[j]) / max(sc[j + 1] - sc[j], 1e-6)
    M = np.eye(4); M[:3, :3] = Slerp([0, 1], Rot.from_matrix(Ts[j:j + 2, :3, :3]))(w).as_matrix(); M[:3, 3] = (1 - w) * Ts[j, :3, 3] + w * Ts[j + 1, :3, 3]; return M
Tk = np.stack([T_at(v) for v in s])
def warp_poses(rows):
    out = rows.copy()
    for i, r in enumerate(rows):
        M = Tk[np.argmin(np.abs(traj[:, 0] - r[0]))]
        out[i, 1:4] = M[:3, :3] @ r[1:4] + M[:3, 3]; out[i, 4:8] = Rot.from_matrix(M[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix()).as_quat()
    return out
np.savetxt(O / 'traj_optimized_poses.txt', warp_poses(traj), fmt='%.6f', header='timestamp x y z qx qy qz qw', comments='#')
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
np.savetxt(O / 'traj_camera_id_camera_poses.txt', warp_poses(cid), fmt='%.6f', header='timestamp x y z qx qy qz qw id', comments='#')
Xw = np.einsum('nij,nj->ni', Tk[ki][:, :3, :3], X) + Tk[ki][:, :3, 3]
pc.points = o3d.utility.Vector3dVector(Xw); o3d.io.write_point_cloud(str(O / 'cloud_cloud.ply'), pc)
if not (O / 'keyframes').exists(): (O / 'keyframes').symlink_to(E / run / 'keyframes')
dv, _ = rvt.query(Xw[vert], distance_upper_bound=0.3); d, _ = rt.query(Xw, distance_upper_bound=0.5)
seg15 = (s[ki] // 15).astype(int)
lines.append(f'결과: 세로 형상 겹침(0.3m 안) {np.mean(np.isfinite(dv)):.0%}, 그 거리 중앙 {np.median(dv[np.isfinite(dv)])*100:.1f}cm | 전체 겹침 {np.mean(d < 0.3):.0%}')
for g in range(seg15.max() + 1):
    kk = seg15[vert] == g
    if kk.sum() < 300: continue
    dd = dv[kk]; f = np.isfinite(dd); lines.append(f'   {g*15:3d}~{g*15+15:3d}m: 세로 형상 겹침 {np.mean(f):4.0%}, 거리 중앙 {np.median(dd[f])*100 if f.any() else float("nan"):5.1f}cm')
(O / 'report.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8'); print('\n'.join(lines))
