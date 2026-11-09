"""Isaac Sim 화면 미리보기 (GPU 없이 광선 추적): D 기준 경로 위 여러 지점에서 로봇 카메라 시점과 뒤따르는 3인칭 시점을 렌더링.
색: 바닥(위 향함·바닥 높이 ±8cm)은 바닥 정사 영상, 나머지는 가장 가까운 점군 점 색(15cm 안), 없으면 회색. 면 방향으로 명암.
C의 색 입히기·조명과는 다르므로 형상(바닥 출렁임·조각·뜬 덩어리·구멍) 비교용.
사용: render_preview.py <out_dir> <route_csv> <floor.npz> <ortho.npz> --set 이름:메시[+메시...]:점군 ... [--s 0,10,...]"""
import argparse
import numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial import cKDTree
ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('route'); ap.add_argument('floor'); ap.add_argument('ortho')
ap.add_argument('--set', action='append', required=True); ap.add_argument('--s', default='2,12,22,32,42,48,52,56,62,72,80,86'); a = ap.parse_args()
O = Path(a.out); O.mkdir(parents=True, exist_ok=True)
R = np.loadtxt(a.route, delimiter=',', skiprows=1); R = R[R[:, 0] == 0]; rs, rxy = R[:, 1], R[:, 2:4]
F = np.load(a.floor); G, lo, cell = F['grid'].astype(float), F['lo'], float(F['cell'])
def fh(xy): return ndimage.map_coordinates(G, [(xy[:, 0] - lo[0]) / cell - 0.5, (xy[:, 1] - lo[1]) / cell - 0.5], order=1, mode='nearest')
Q = np.load(a.ortho); orgb, olo, oc = Q['rgb'], Q['lo'], float(Q['cell']); ovalid = (orgb.sum(2) > 0)
W, H = 640, 400
def cam_rays(c, fwd, pitch_deg, hfov):
    fwd = np.r_[fwd / np.linalg.norm(fwd), 0.0]; up0 = np.array([0, 0, 1.0]); right = np.cross(fwd, up0); right /= np.linalg.norm(right)
    p = np.radians(pitch_deg); f2 = fwd * np.cos(p) + up0 * np.sin(p); up = np.cross(right, f2)
    fx = (W / 2) / np.tan(np.radians(hfov) / 2); u, v = np.meshgrid(np.arange(W) - W / 2 + 0.5, np.arange(H) - H / 2 + 0.5)
    d = f2[None, None] * fx + right[None, None] * u[..., None] - up[None, None] * v[..., None]; d /= np.linalg.norm(d, axis=2, keepdims=True)
    return np.concatenate([np.broadcast_to(c, d.shape), d], 2).reshape(-1, 6).astype(np.float32)
sets = []
for sp in a.set:
    name, meshes, cloud = sp.split(':'); m = o3d.geometry.TriangleMesh()
    for f in meshes.split('+'): m += o3d.io.read_triangle_mesh(f)
    sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))
    pc = o3d.io.read_point_cloud(cloud); sets.append((name, sc, cKDTree(np.asarray(pc.points)), (np.asarray(pc.colors) * 255).astype(np.uint8)))
    print(f'{name}: 삼각형 {len(m.triangles):,}, 점군 {len(pc.points):,}')
L = np.array([0.3, 0.2, 0.93]); L /= np.linalg.norm(L)
def render(sc, kd, col, rays):
    r = sc.cast_rays(o3d.core.Tensor(rays)); t = r['t_hit'].numpy(); n = r['primitive_normals'].numpy(); hit = np.isfinite(t)
    out = np.zeros((len(rays), 3), np.uint8); vv = np.repeat(np.linspace(0, 1, H), W)
    out[:] = np.c_[235 - 30 * (1 - vv), 225 - 20 * (1 - vv), 210 - 10 * (1 - vv)].astype(np.uint8)   # 하늘(BGR 연한 색)
    if not hit.any(): return out.reshape(H, W, 3)
    p = rays[hit, :3] + rays[hit, 3:] * t[hit, None]; nn = n[hit]; nn = np.where((nn * rays[hit, 3:]).sum(1, keepdims=True) > 0, -nn, nn)
    floor = (nn[:, 2] > 0.8) & (np.abs(p[:, 2] - fh(p[:, :2])) < 0.08)
    c = np.full((len(p), 3), 128, np.uint8)
    ix = ((p[:, 0] - olo[0]) / oc).astype(int); iy = ((p[:, 1] - olo[1]) / oc).astype(int); ok = floor & (ix >= 0) & (ix < orgb.shape[0]) & (iy >= 0) & (iy < orgb.shape[1])
    okv = ok.copy(); okv[ok] = ovalid[ix[ok], iy[ok]]; c[okv] = orgb[ix[okv], iy[okv]][:, ::-1]
    rest = ~okv; d, j = kd.query(p[rest], distance_upper_bound=0.15); g = np.isfinite(d); ci = np.flatnonzero(rest)
    c[ci[g]] = col[j[g]][:, ::-1]
    shade = np.clip(0.55 + 0.45 * (nn @ L), 0.35, 1.0); fog = np.clip(1 - (t[hit] - 25) / 60, 0.4, 1)[:, None]
    cc = c * shade[:, None] * fog + np.array([225, 215, 205]) * (1 - fog); out[hit] = np.clip(cc, 0, 255).astype(np.uint8); return out.reshape(H, W, 3)
tg_all = np.gradient(rxy, axis=0)
for s in [float(x) for x in a.s.split(',')]:
    j = int(np.argmin(np.abs(rs - s))); xy = rxy[j]; tg = tg_all[j] / np.linalg.norm(tg_all[j]); z = fh(xy[None])[0]
    views = [('robot', cam_rays(np.r_[xy, z + 0.45], tg, -3, 80)), ('chase', cam_rays(np.r_[xy - 3.0 * tg, z + 1.6], tg, -16, 70))]
    rowimgs = []
    for vn, rays in views:
        imgs = []
        for name, sc, kd, col in sets:
            im = render(sc, kd, col, rays); cv2.putText(im, f'{name} {vn} s={rs[j]:.0f}m', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2); imgs.append(im)
        rowimgs.append(np.hstack(imgs))
    cv2.imwrite(str(O / f'view_s{int(rs[j]):03d}.jpg'), np.vstack(rowimgs), [cv2.IMWRITE_JPEG_QUALITY, 85]); print('s', int(rs[j]))
