"""실내 메시 확인 렌더: (1) 위에서 본 그림(천장 제거: z<2.0, 정사영, 꼭짓점 색) (2) 키프레임 자세에서 본 렌더와 실제 사진 나란히.
사용: in_view.py <mesh.ply> <run(자세·사진)> <out_prefix> [--kf 8]"""
import sys, re, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; M, run, OUT = a[1], a[2], a[3]; NK = int(a[a.index('--kf') + 1]) if '--kf' in a else 8
E = Path.home() / 'b_slam/exports'; K = E / run / 'keyframes'
m = o3d.io.read_triangle_mesh(M); V = np.asarray(m.vertices); T = np.asarray(m.triangles); Cc = np.asarray(m.vertex_colors)
print(f'메시: 꼭짓점 {len(V):,}, 삼각형 {len(T):,}, 범위 x {V[:, 0].min():.1f}~{V[:, 0].max():.1f} y {V[:, 1].min():.1f}~{V[:, 1].max():.1f} z {V[:, 2].min():.2f}~{V[:, 2].max():.2f}')
sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))
def shade(rays, H, W):
    r = sc.cast_rays(o3d.core.Tensor(rays.astype(np.float32))); t = r['t_hit'].numpy(); hit = np.isfinite(t); pid = r['primitive_ids'].numpy(); uv = r['primitive_uvs'].numpy()
    img = np.full((H * W, 3), 255, np.uint8); tri = T[pid[hit]]; w0 = 1 - uv[hit, 0] - uv[hit, 1]
    col = Cc[tri[:, 0]] * w0[:, None] + Cc[tri[:, 1]] * uv[hit, :1] + Cc[tri[:, 2]] * uv[hit, 1:]
    img[hit] = np.clip(col[:, ::-1] * 255, 0, 255).astype(np.uint8); return img.reshape(H, W, 3), hit.reshape(H, W)
# (1) 위에서: z=1.9 에서 아래로 정사영, 3cm/px
C = 0.03; x0, y0 = V[:, 0].min() - 0.5, V[:, 1].min() - 0.5; W_, H_ = int((V[:, 0].max() + 0.5 - x0) / C), int((V[:, 1].max() + 0.5 - y0) / C)
u, v = np.meshgrid(np.arange(W_), np.arange(H_)); o = np.stack([x0 + (u + 0.5) * C, y0 + (H_ - v - 0.5) * C, np.full(u.shape, 1.9)], -1).reshape(-1, 3)
top, _ = shade(np.c_[o, np.tile([0, 0, -1.0], (len(o), 1))], H_, W_)
for xg in np.arange(np.ceil(x0 / 5) * 5, x0 + W_ * C, 5): cv2.putText(top, f'{xg:.0f}', (int((xg - x0) / C), 14), 0, 0.5, (0, 120, 255), 1)
cv2.imwrite(OUT + '_top.png', top)
# (2) 키프레임 시점 렌더 vs 사진
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); cid = cid[np.argsort(cid[:, 0])]; rows = []
for r in cid[np.linspace(0, len(cid) - 1, NK + 2).astype(int)[1:-1]]:
    i = int(r[8]); k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
    im = cv2.imread(str(K / f'kf_rgb/{i}.jpg')); h, w = im.shape[:2]; uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    d = np.stack([(uu - k[2]) / k[0], (vv - k[5]) / k[4], np.ones(uu.shape)], -1).reshape(-1, 3) @ Rot.from_quat(r[4:8]).as_matrix().T; d /= np.linalg.norm(d, axis=1, keepdims=True)
    ren, hit = shade(np.c_[np.broadcast_to(r[1:4], d.shape), d], h, w); cv2.putText(im, f'{run} kf{i} photo', (6, 22), 0, 0.6, (0, 255, 255), 2); cv2.putText(ren, f'render (hit {hit.mean():.0%})', (6, 22), 0, 0.6, (0, 0, 255), 2)
    rows.append(np.hstack([im, ren]))
cv2.imwrite(OUT + '_kf.jpg', np.vstack([np.hstack(rows[k:k + 2]) for k in range(0, len(rows) - 1, 2)])[::2, ::2], [cv2.IMWRITE_JPEG_QUALITY, 85])
