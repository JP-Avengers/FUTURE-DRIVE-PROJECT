"""회차 정합 검증: (1) 세로 형상(|nz|<0.6) 점만으로 v0.5 점군과의 겹침(0.1m·0.3m 안)과 거리, 경로 15m 구간별
(2) 키프레임 몇 장의 자세(정합 적용)에서 v0.5 형상 메시를 광선 추적 렌더(면 방향 명암)해 실제 사진과 나란히 (윤곽이 겹치면 정합이 맞음).
사용: verify_align.py <run> <T.npy> <out_dir> [--kf 6]"""
import sys, re, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, tf, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True); NK = int(a[a.index('--kf') + 1]) if '--kf' in a else 6
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'; T = np.load(tf)
ref = o3d.io.read_point_cloud(str(P / 'cloud_3cm.ply')).voxel_down_sample(0.05); ref.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
Rv = np.asarray(ref.points)[np.abs(np.asarray(ref.normals)[:, 2]) < 0.6]; rvt = cKDTree(Rv)
pc = o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).voxel_down_sample(0.05); pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))
X = np.asarray(pc.points); V = np.abs(np.asarray(pc.normals)[:, 2]) < 0.6; Xw = X @ T[:3, :3].T + T[:3, 3]
tr = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2); tp = tr[:, 1:4]; s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tp, axis=0), axis=1))]
_, ki = cKDTree(tp).query(X); seg = (s[ki] // 15).astype(int)
d, _ = rvt.query(Xw[V], distance_upper_bound=0.3); sv = seg[V]
lines = [f'{run}: 세로 형상 점 {V.sum():,} | 0.1m 안 {np.mean(d < 0.1):.0%}, 0.3m 안 {np.mean(np.isfinite(d)):.0%}, 0.3m 안 거리 중앙 {np.median(d[np.isfinite(d)])*100:.1f}cm']
for g in range(seg.max() + 1):
    k = sv == g
    if k.sum() < 300: continue
    dd = d[k]; f = np.isfinite(dd); lines.append(f'   {g*15:3d}~{g*15+15:3d}m: 세로 형상 0.1m 안 {np.mean(dd < 0.1):4.0%}, 0.3m 안 {np.mean(f):4.0%}, 거리 중앙 {np.median(dd[f])*100 if f.any() else float("nan"):5.1f}cm')
print('\n'.join(lines)); (O / f'{run}_vertical.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
# 렌더 비교
m = o3d.io.read_triangle_mesh(str(P / 'geometry_clean_5cm.ply')); sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); K = E / run / 'keyframes'
pick = cid[np.linspace(0, len(cid) - 1, NK + 2).astype(int)[1:-1]]
rows = []
for r in pick:
    i = int(r[8]); txt = (K / f'kf_calib/{i}.yaml').read_text(); k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', txt, re.S).group(1).replace('\n', ' ').split(',')]
    im = cv2.imread(str(K / f'kf_rgb/{i}.jpg')); h, w = im.shape[:2]
    Ropt = Rot.from_quat(r[4:8]).as_matrix(); c = r[1:4]      # traj_camera_id_camera_poses = 광학 좌표계 자세(z 앞, x 오른쪽, y 아래)
    Rw = T[:3, :3] @ Ropt; cw = T[:3, :3] @ c + T[:3, 3]
    u, v = np.meshgrid(np.arange(w), np.arange(h)); dcam = np.stack([(u - k[2]) / k[0], (v - k[5]) / k[4], np.ones_like(u, float)], -1).reshape(-1, 3)
    dw = dcam @ Rw.T; dw /= np.linalg.norm(dw, axis=1, keepdims=True)
    rr = sc.cast_rays(o3d.core.Tensor(np.c_[np.broadcast_to(cw, dw.shape), dw].astype(np.float32))); t = rr['t_hit'].numpy(); n = rr['primitive_normals'].numpy()
    hit = np.isfinite(t); sh = np.zeros(len(t)); nn = np.where((n * dw).sum(1, keepdims=True) > 0, -n, n); sh[hit] = np.clip(0.35 + 0.65 * (nn[hit] @ np.array([0.3, 0.2, 0.93])), 0, 1)
    rend = (sh.reshape(h, w) * 255).astype(np.uint8); rend[~hit.reshape(h, w)] = 0; rend = cv2.cvtColor(rend, cv2.COLOR_GRAY2BGR)
    edges = cv2.Canny(cv2.GaussianBlur(rend, (5, 5), 0), 30, 90); over = im.copy(); over[edges > 0] = (0, 0, 255)
    cv2.putText(im, f'{run} kf {i}', (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    rows.append(np.hstack([im, rend, over]))
cv2.imwrite(str(O / f'{run}_render_check.jpg'), np.vstack(rows)[::2, ::2], [cv2.IMWRITE_JPEG_QUALITY, 80]); print('렌더 비교 →', O / f'{run}_render_check.jpg')
