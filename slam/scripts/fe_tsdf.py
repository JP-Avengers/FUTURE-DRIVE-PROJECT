"""손 촬영 묶음 맵 키프레임(컬러 정렬 depth) → 정합 T 적용한 v0.5 좌표 컬러 TSDF 메시 + 같은 좌표의 camera_link 궤적.
사용: fe_tsdf.py <run> <T.npy> <out_dir> [--voxel 0.04] [--maxd 4.0] [--mind 0.4] [--zfix] [--t0 시각 --t1 시각] [--smin m] [--tag _x]
  --t0/--t1: 키프레임 타임스탬프 범위(묶음 맵 세션 하나), --smin: 그 범위 안 경로 거리 m 이후 키프레임만, 출력 이름 <run><tag>_*"""
import sys, re, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, tf, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
VOX, MAXD, MIND = opt('--voxel', 0.04), opt('--maxd', 4.0), opt('--mind', 0.4)
E = Path.home() / 'b_slam/exports'; T = np.load(tf); K = E / run / 'keyframes'
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
# 높이 보정(--zfix): 키프레임마다 (v0.5 바닥 높이 - 회차 자체 점군의 카메라 아래 바닥) → 세션별 궤적 따라 보간·중앙값으로 부드럽게
DZ = np.zeros(len(cid))
if '--zfix' in a:
    from scipy import ndimage
    from scipy.spatial import cKDTree
    P5 = Path.home() / 'b_slam/deliver/map_v0.5'; Fm = np.load(P5 / 'floor_height.npz'); G, glo, gc, conf = Fm['grid'].astype(float), Fm['lo'], float(Fm['cell']), Fm['conf']
    pc = o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).voxel_down_sample(0.05); X = np.asarray(pc.points) @ T[:3, :3].T + T[:3, 3]; xt = cKDTree(X[:, :2])
    cw = cid[:, 1:4] @ T[:3, :3].T + T[:3, 3]; raw = np.full(len(cid), np.nan)
    for j, c in enumerate(cw):
        ix, iy = int((c[0] - glo[0]) / gc), int((c[1] - glo[1]) / gc)
        if not (0 <= ix < G.shape[0] and 0 <= iy < G.shape[1]) or conf[ix, iy] < 1: continue
        idx = xt.query_ball_point(c[:2], 0.6)
        if len(idx) > 20: raw[j] = G[ix, iy] - np.percentile(X[idx, 2], 10)
    # 세션 경계: 카메라 위치가 3m 넘게 뛰는 곳
    jump = np.r_[0, np.where(np.linalg.norm(np.diff(cw, axis=0), axis=1) > 3)[0] + 1, len(cid)]
    for s0, s1 in zip(jump[:-1], jump[1:]):
        seg = raw[s0:s1].copy(); ok = np.isfinite(seg)
        if ok.sum() == 0: DZ[s0:s1] = np.nan; continue
        seg = np.interp(np.arange(len(seg)), np.where(ok)[0], seg[ok]); DZ[s0:s1] = ndimage.median_filter(seg, 9, mode='nearest')
    if np.isnan(DZ).any(): DZ[np.isnan(DZ)] = np.nanmedian(DZ)
    print(f'높이 보정: 관측 {np.isfinite(raw).sum()}/{len(cid)}장, 보정량 중앙 {np.median(DZ):+.2f}m, 범위 {np.percentile(DZ, 5):+.2f}~{np.percentile(DZ, 95):+.2f}m, 세션 구간 {len(jump) - 1}', flush=True)
    np.save(O / f'{run}_zfix.npy', DZ)
T0_, T1_, SMIN = opt('--t0', -np.inf), opt('--t1', np.inf), opt('--smin', 0.0); TAG = a[a.index('--tag') + 1] if '--tag' in a else ''
use = (cid[:, 0] >= T0_) & (cid[:, 0] <= T1_); iu = np.where(use)[0]
su = np.r_[0, np.cumsum(np.linalg.norm(np.diff(cid[iu, 1:4], axis=0), axis=1))]; use[iu[su < SMIN]] = False
if use.sum() < len(cid): print(f'키프레임 선택: 시간 범위 {len(iu)}장 중 경로 {SMIN:g}m 이후 {use.sum()}장', flush=True)
vol = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=VOX, sdf_trunc=4 * VOX, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
n = 0
for jr, r in enumerate(cid):
    if not use[jr]: continue
    i = int(r[8]); dp = K / f'kf_depth/{i}.png'
    if not dp.exists(): continue
    txt = (K / f'kf_calib/{i}.yaml').read_text()
    k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', txt, re.S).group(1).replace('\n', ' ').split(',')]
    col = cv2.cvtColor(cv2.imread(str(K / f'kf_rgb/{i}.jpg')), cv2.COLOR_BGR2RGB); dz = cv2.imread(str(dp), cv2.IMREAD_UNCHANGED)
    if dz.dtype != np.uint16: dz = (dz.astype(np.float32) * 1000).astype(np.uint16)
    dz = dz.copy(); dz[dz < MIND * 1000] = 0; h, w = dz.shape
    if col.shape[:2] != (h, w): col = cv2.resize(col, (w, h))
    Kc = o3d.camera.PinholeCameraIntrinsic(w, h, k[0], k[4], k[2], k[5])
    Tw = np.eye(4); Tw[:3, :3] = T[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix(); Tw[:3, 3] = T[:3, :3] @ r[1:4] + T[:3, 3] + [0, 0, DZ[jr]]
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.ascontiguousarray(col)), o3d.geometry.Image(dz), depth_scale=1000.0, depth_trunc=MAXD, convert_rgb_to_intensity=False)
    vol.integrate(rgbd, Kc, np.linalg.inv(Tw)); n += 1
    if n % 200 == 0: print(f'  {n}장', flush=True)
mesh = vol.extract_triangle_mesh(); mesh.compute_vertex_normals()
o3d.io.write_triangle_mesh(str(O / f'{run}{TAG}_tsdf_{round(VOX * 100)}cm.ply'), mesh)
tr = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2); tr = tr[(tr[:, 0] >= T0_) & (tr[:, 0] <= T1_)]; pw = tr[:, 1:4] @ T[:3, :3].T + T[:3, 3]
o_ = np.argsort(cid[:, 0]); pw[:, 2] += np.interp(tr[:, 0], cid[o_, 0], DZ[o_])          # 높이 보정을 궤적에도
qw = (Rot.from_matrix(T[:3, :3]) * Rot.from_quat(tr[:, 4:8])).as_quat()
np.savetxt(O / f'{run}{TAG}_trajectory_camera_link.txt', np.c_[tr[:, 0], pw, qw], fmt='%.6f', header='timestamp x y z qx qy qz qw (v0.5 좌표)')
print(f'{run}: 키프레임 {n}/{len(cid)} 통합 → 꼭짓점 {len(mesh.vertices):,}, 삼각형 {len(mesh.triangles):,} → {O}')
