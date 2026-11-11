"""둘레 이음새 확인 렌더: 주차장 안 높이 3m에서 둘레 쪽을 내려다보는 시점들. 세트별(메시들+색 점군) 나란히.
색: 맞은 점 15cm 안 가장 가까운 점군 색(없으면 회색) × 면 방향 명암.
사용: seam_view.py <out_dir> <floor.npz> --set 이름:메시[+메시]:점군 ... """
import argparse, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial import cKDTree
ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('floor'); ap.add_argument('--set', action='append', required=True); a = ap.parse_args()
O = Path(a.out); O.mkdir(parents=True, exist_ok=True)
F = np.load(a.floor); G, lo, c = F['grid'].astype(float), F['lo'], float(F['cell'])
fh = lambda x, y: float(ndimage.map_coordinates(G, [[(x - lo[0]) / c - 0.5], [(y - lo[1]) / c - 0.5]], order=1, mode='nearest')[0])
VIEWS = [('N', (-20, 9), (-20, 17)), ('N2', (-8, 11), (-8, 19)), ('NW', (-39, -1), (-45, 4)), ('NW2', (-45, -8), (-52, -6)),
         ('NE', (0, 12), (6, 20)), ('E', (1, 0), (9, 0)), ('S', (-33, -18), (-33, -26)), ('IT1', (-10, -19), (-10, -27))]
W, H, HF = 480, 320, 75
def rays(cam, tgt):
    f = tgt - cam; f /= np.linalg.norm(f); r = np.cross(f, [0, 0, 1]); r /= np.linalg.norm(r); u = np.cross(r, f)
    fx = (W / 2) / np.tan(np.radians(HF) / 2); uu, vv = np.meshgrid(np.arange(W) - W / 2 + 0.5, np.arange(H) - H / 2 + 0.5)
    d = f[None, None] * fx + r[None, None] * uu[..., None] - u[None, None] * vv[..., None]; d /= np.linalg.norm(d, axis=-1, keepdims=True)
    return np.c_[np.broadcast_to(cam, (H * W, 3)), d.reshape(-1, 3)].astype(np.float32)
sets = []
for s in a.set:
    nm, meshes, cl = s.split(':'); sc = o3d.t.geometry.RaycastingScene()
    for mp in meshes.split('+'): sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(o3d.io.read_triangle_mesh(mp)))
    pc = o3d.io.read_point_cloud(cl); sets.append((nm, sc, cKDTree(np.asarray(pc.points)), np.asarray(pc.colors))); print('세트', nm, len(pc.points))
for vn, cxy, txy in VIEWS:
    cam = np.r_[cxy, fh(*cxy) + 3.0]; tgt = np.r_[txy, fh(*txy) + 0.3]; R = rays(cam, tgt); row = []
    for nm, sc, tree, col in sets:
        res = sc.cast_rays(o3d.core.Tensor(R)); t = res['t_hit'].numpy(); n = res['primitive_normals'].numpy(); hit = np.isfinite(t) & (t < 40)
        img = np.full((H * W, 3), (215, 210, 205), np.uint8); P = R[hit, :3] + t[hit, None] * R[hit, 3:]
        dd, ii = tree.query(P, distance_upper_bound=0.15); cc = np.where(np.isfinite(dd)[:, None], col[np.minimum(ii, len(col) - 1)], 0.55)
        nn = n[hit]; nn = np.where((nn * R[hit, 3:]).sum(1, keepdims=True) > 0, -nn, nn); sh = np.clip(0.55 + 0.45 * (nn @ np.array([0.3, 0.2, 0.93])), 0.3, 1)
        img[hit] = np.clip(cc[:, ::-1] * 255 * sh[:, None], 0, 255).astype(np.uint8); img = img.reshape(H, W, 3)
        cv2.putText(img, f'{nm} {vn}', (6, 18), 0, 0.55, (0, 0, 180), 2); row.append(img)
    cv2.imwrite(str(O / f'seam_{vn}.jpg'), np.hstack(row), [cv2.IMWRITE_JPEG_QUALITY, 88]); print('시점', vn)
