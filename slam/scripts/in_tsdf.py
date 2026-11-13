"""여러 회차 키프레임을 한 TSDF로 (건물 좌표 내보내기 <run>m · <run>L). 회차 뒤에 :q 를 붙이면 in_localize 의 키프레임별 벽 일치 비율(10cm 안)이 q 이상인 것만.
사용: in_tsdf.py <out.ply> <run[:q]> [<run[:q]> ...] [--voxel 0.02] [--maxd 3.0] [--mind 0.4]"""
import sys, re, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; out = a[1]; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
VOX, MAXD, MIND = opt('--voxel', 0.02), opt('--maxd', 3.0), opt('--mind', 0.4); E = Path.home() / 'b_slam/exports'
runs = [x for i, x in enumerate(a[2:], 2) if not x.startswith('--') and not a[i - 1].startswith('--')]
vol = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=VOX, sdf_trunc=4 * VOX, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
for spec in runs:
    run, qmin = (spec.split(':') + ['0'])[:2]; qmin = float(qmin); K = E / run / 'keyframes'; cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); frac = {}
    f = E / 'indoor' / f'{run[:-1]}_localize.npz'
    if qmin > 0 and f.exists(): z = np.load(f); frac = {int(i): float(q) for i, q in zip(z['ids'], z['frac'])}
    n = 0
    for r in cid:
        i = int(r[8])
        if qmin > 0 and frac.get(i, 0) < qmin: continue
        k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
        col = cv2.cvtColor(cv2.imread(str(K / f'kf_rgb/{i}.jpg')), cv2.COLOR_BGR2RGB); dz = cv2.imread(str(K / f'kf_depth/{i}.png'), -1).copy(); dz[dz < MIND * 1000] = 0; h, w = dz.shape
        Tw = np.eye(4); Tw[:3, :3] = Rot.from_quat(r[4:8]).as_matrix(); Tw[:3, 3] = r[1:4]
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.ascontiguousarray(col)), o3d.geometry.Image(dz), depth_scale=1000.0, depth_trunc=MAXD, convert_rgb_to_intensity=False)
        vol.integrate(rgbd, o3d.camera.PinholeCameraIntrinsic(w, h, k[0], k[4], k[2], k[5]), np.linalg.inv(Tw)); n += 1
    print(f'{run}: 키프레임 {n}/{len(cid)} 통합' + (f' (벽 일치 {qmin:.0%} 이상만)' if qmin > 0 else ''), flush=True)
m = vol.extract_triangle_mesh(); m.compute_vertex_normals(); o3d.io.write_triangle_mesh(out, m); V = np.asarray(m.vertices)
print(f'→ {out}: 꼭짓점 {len(V):,}, 삼각형 {len(m.triangles):,}, z {V[:, 2].min():.2f}~{V[:, 2].max():.2f}m')
