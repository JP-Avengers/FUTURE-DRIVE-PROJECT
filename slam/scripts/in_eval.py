"""실내 메시 평가 두 가지.
 (1) 검증 회차 depth 일치: 맵에 안 쓴 회차의 키프레임 자세에서 메시를 렌더한 depth 와 실제 depth 차이(유효 화소 0.4~3.5m) — 중앙, 5cm·10cm 안 비율, 메시가 채운 비율
 (2) 로봇 시점 채움: 복도 축을 따라 높이 0.45m·수평 시선(가로 화각 80°, 세로 53°)으로 양방향을 볼 때 광선이 12m 안에서 메시에 닿는 비율, 화면 위 1/3 만 따로
사용: in_eval.py <val_run> <mesh.ply> [<mesh2.ply> ...] [--y -0.1 (로봇 주행선 y)] [--img out.jpg]"""
import sys, re, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; val = a[1]; meshes = [x for i, x in enumerate(a[2:], 2) if x.endswith('.ply')]; opt = lambda k, d: a[a.index(k) + 1] if k in a else d
YL = float(opt('--y', 0.15)); E = Path.home() / 'b_slam/exports'; K = E / val / 'keyframes'
cid = np.loadtxt(E / val / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); cid = cid[np.argsort(cid[:, 0])][::3]
tiles = []
for mp in meshes:
    m = o3d.io.read_triangle_mesh(mp); sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m)); V = np.asarray(m.vertices); T = np.asarray(m.triangles); Cc = np.asarray(m.vertex_colors)
    dif = []; fill = []
    for r in cid:
        i = int(r[8]); k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
        d = cv2.imread(str(K / f'kf_depth/{i}.png'), -1).astype(np.float32) / 1000; v, u = np.mgrid[2:d.shape[0]:4, 2:d.shape[1]:4]; z = d[v, u]; ok = (z > 0.4) & (z < 3.5)
        dc = np.stack([(u - k[2]) / k[0], (v - k[5]) / k[4], np.ones(u.shape)], -1).reshape(-1, 3); nrm = np.linalg.norm(dc, axis=1); dw = (dc / nrm[:, None]) @ Rot.from_quat(r[4:8]).as_matrix().T
        t = sc.cast_rays(o3d.core.Tensor(np.c_[np.broadcast_to(r[1:4], dw.shape), dw].astype(np.float32)))['t_hit'].numpy(); zr = (t / nrm).reshape(z.shape); hit = np.isfinite(zr)
        fill.append(np.mean(hit[ok])); dif.append(np.abs(zr - z)[ok & hit])
    dif = np.concatenate(dif); print(f'{Path(mp).name}: 검증 {val} 키프레임 {len(cid)}장 | depth 차 중앙 {np.median(dif) * 100:.1f}cm, 5cm 안 {np.mean(dif < 0.05):.0%}, 10cm 안 {np.mean(dif < 0.10):.0%} | 메시가 채운 비율 {np.mean(fill):.0%}')
    W_, H_ = 320, 200; fx = (W_ / 2) / np.tan(np.radians(40)); uu, vv = np.meshgrid(np.arange(W_) - W_ / 2 + 0.5, np.arange(H_) - H_ / 2 + 0.5); hits = []; ups = []; row = []
    for x in np.arange(2, 45.1, 3.0):
        for sgn in (1, -1):
            dirs = np.stack([np.full(uu.shape, sgn * fx), -sgn * uu, -vv], -1).reshape(-1, 3); dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
            res = sc.cast_rays(o3d.core.Tensor(np.c_[np.tile([x, YL, 0.45], (len(dirs), 1)), dirs].astype(np.float32))); t = res['t_hit'].numpy().reshape(H_, W_); h = np.isfinite(t) & (t < 12)
            hits.append(h.mean()); ups.append(h[:H_ // 3].mean())
            if sgn == 1 and abs(x - 11) < 0.1 or sgn == 1 and abs(x - 32) < 0.1:
                pid = res['primitive_ids'].numpy().reshape(H_, W_); uv = res['primitive_uvs'].numpy().reshape(H_, W_, 2); img = np.full((H_, W_, 3), (255, 200, 255), np.uint8); tri = T[pid[h]]; w0 = 1 - uv[h][:, 0] - uv[h][:, 1]
                col = Cc[tri[:, 0]] * w0[:, None] + Cc[tri[:, 1]] * uv[h][:, :1] + Cc[tri[:, 2]] * uv[h][:, 1:]; img[h] = np.clip(col[:, ::-1] * 255, 0, 255).astype(np.uint8)
                cv2.putText(img, f'{Path(mp).stem[:18]} x={x:.0f}', (4, 14), 0, 0.45, (0, 0, 255), 1); row.append(img)
    print(f'   로봇 시점(높이 0.45m, 수평, {len(hits)}곳): 광선이 메시에 닿는 비율 평균 {np.mean(hits):.0%}, 화면 위 1/3 은 {np.mean(ups):.0%}')
    tiles.append(np.hstack(row))
if '--img' in a: cv2.imwrite(opt('--img', ''), np.vstack(tiles), [cv2.IMWRITE_JPEG_QUALITY, 88])
