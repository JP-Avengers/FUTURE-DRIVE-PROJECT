"""한 회차 키프레임 중 고른 id 만 TSDF 로 (in_tsdf.py 의 한 회차 · id 목록판). 예: 천장으로 높이를 정한 키프레임만.
id 목록 = npz(id, src) 에서 src 가 --src 값인 것 (in_ceilsrc.py 가 만듦) 또는 공백으로 나눈 id 텍스트 파일.
사용: in_tsdf_ids.py <out.ply> <run> <ids.npz|ids.txt> [--src 1] [--voxel 0.02] [--maxd 3.0] [--mind 0.4]"""
import sys, re, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; out, run, idf = a[1], a[2], a[3]; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
VOX, MAXD, MIND = opt('--voxel', 0.02), opt('--maxd', 3.0), opt('--mind', 0.4); E = Path.home() / 'b_slam/exports'
if idf.endswith('.npz'): z = np.load(idf); want = set(int(i) for i in z['id'][z['src'] == int(opt('--src', 1))])
else: want = set(int(x) for x in Path(idf).read_text().split())
vol = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=VOX, sdf_trunc=4 * VOX, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
K = E / run / 'keyframes'; cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); n = 0
for r in cid:
    i = int(r[8])
    if i not in want: continue
    k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
    col = cv2.cvtColor(cv2.imread(str(K / f'kf_rgb/{i}.jpg')), cv2.COLOR_BGR2RGB); dz = cv2.imread(str(K / f'kf_depth/{i}.png'), -1).copy(); dz[dz < MIND * 1000] = 0; h, w = dz.shape
    Tw = np.eye(4); Tw[:3, :3] = Rot.from_quat(r[4:8]).as_matrix(); Tw[:3, 3] = r[1:4]
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.ascontiguousarray(col)), o3d.geometry.Image(dz), depth_scale=1000.0, depth_trunc=MAXD, convert_rgb_to_intensity=False)
    vol.integrate(rgbd, o3d.camera.PinholeCameraIntrinsic(w, h, k[0], k[4], k[2], k[5]), np.linalg.inv(Tw)); n += 1
m = vol.extract_triangle_mesh(); m.compute_vertex_normals(); o3d.io.write_triangle_mesh(out, m); V = np.asarray(m.vertices)
print(f'{run}: 키프레임 {n}/{len(cid)} (목록 {len(want)}) → {out}: 삼각형 {len(m.triangles):,}, z {V[:, 2].min():.2f}~{V[:, 2].max():.2f}m')
