"""지점별 확인: (1) 그 지점을 3~7m에서 본 본네트 키프레임 사진(지점 표시) (2) 경로에 수직인 단면 — 최종 메시와 자료별 메시(본네트 시계·반시계, 지붕) 꼭짓점.
사용: spot_inspect.py <pkg_dir> <route_csv> <out_dir>"""
import sys, re
import numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

P, RC, O = map(Path, sys.argv[1:4]); O.mkdir(parents=True, exist_ok=True); E = Path.home() / 'b_slam/exports'
R = np.loadtxt(RC, delimiter=',', skiprows=1); R = R[R[:, 0] == 0]; rs, rxy = R[:, 1], R[:, 2:4]; rt = cKDTree(rxy)
SP = [('A_52.5m_stall', -33.65, -6.65), ('B_west_flat', -36.8, -7.35), ('C_s54_obj', -33.0, -4.3), ('D_s31_obj', -22.3, -17.3), ('E_s44_float', -32.8, -13.1), ('F_s49_dip', None, 49.0)]
SP = [(n, *(rxy[np.argmin(np.abs(rs - b))] if a is None else (a, b))) for n, a, b in SP]
SRC = [('final', P / 'geometry_rawdepth_tsdf_5cm.ply', 'k', 2), ('bonnet cw (base)', E / 'gn04base/tsdf_rawdepth_5cm_min1m.ply', 'g', 3),
       ('bonnet ccw', E / 'gm05b/tsdf_rawdepth_5cm_min1m.ply', 'b', 3), ('roof cw', E / 'run18b6/tsdf_rawdepth_5cm_min1m.ply', 'orange', 3), ('roof ccw', E / 'run20b6/tsdf_rawdepth_5cm_min1m.ply', 'r', 3)]
V = {}
for nm, f, *_ in SRC:
    X = np.asarray(o3d.io.read_point_cloud(str(f)).points); k = np.zeros(len(X), bool)
    for _, x, y in SP: k |= np.hypot(X[:, 0] - x, X[:, 1] - y) < 4.5
    V[nm] = X[k]
KF = []
for t in ('bonnet_cw', 'bonnet_ccw'):
    for r in np.loadtxt(P / f'trajectory_{t}_camera_optical.txt', comments='#', ndmin=2): KF.append((t, r))
for nm, x, y in SP:
    _, j = rt.query([x, y]); tg = np.gradient(rxy, axis=0)[j]; tg /= np.linalg.norm(tg); nv = np.array([-tg[1], tg[0]])
    fig, ax = plt.subplots(1, 2, figsize=(18, 6), gridspec_kw=dict(width_ratios=[1.1, 1]))
    for snm, _, col, ms in SRC:
        X = V[snm]; d = X[:, :2] - [x, y]; u = d @ nv; w = d @ tg; k = (np.abs(w) < 0.25) & (np.abs(u) < 4)
        ax[0].scatter(u[k], X[k, 2], s=ms, c=col, label=snm, alpha=0.7 if snm == 'final' else 0.5)
    fz = V['final'][np.hypot(V['final'][:, 0] - x, V['final'][:, 1] - y) < 3, 2]; z0 = np.percentile(fz, 10) if len(fz) else 0
    ax[0].set_ylim(z0 - 0.6, z0 + 1.6); ax[0].axvline(0, c='gray', ls=':'); ax[0].legend(markerscale=4, fontsize=8)
    ax[0].set_title(f'{nm} ({x:.1f}, {y:.1f}) route s={rs[j]:.1f}m  cross-section 0.5m thick, u = across route (+ left of travel)'); ax[0].set_xlabel('u (m)'); ax[0].set_ylabel('z (m)')
    best = None; p = np.array([x, y, z0 + 0.1])
    for t, r in KF:
        Rm = Rot.from_quat(r[4:8]).as_matrix(); pc = Rm.T @ (p - r[1:4])
        if not (2.5 < pc[2] < 7): continue
        K = P / f'keyframes_{t}'; i = int(r[8]); txt = (K / f'calib/{i}.yaml').read_text()
        a = [float(v) for v in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', txt, re.S).group(1).replace('\n', ' ').split(',')]
        uu, vv = a[0] * pc[0] / pc[2] + a[2], a[4] * pc[1] / pc[2] + a[5]
        sc = abs(uu - 320) + abs(vv - 300) + 20 * abs(pc[2] - 4)
        if 40 < uu < 600 and 60 < vv < 470 and (best is None or sc < best[0]): best = (sc, t, i, uu, vv, pc[2])
    if best:
        _, t, i, uu, vv, dz = best; im = cv2.imread(str(P / f'keyframes_{t}/rgb/{i}.jpg'))[:, :, ::-1].copy()
        cv2.circle(im, (int(uu), int(vv)), 22, (255, 0, 255), 2); ax[1].imshow(im); ax[1].set_title(f'{t} keyframe {i}, point at {dz:.1f}m')
    ax[1].axis('off'); plt.tight_layout(); plt.savefig(O / f'{nm}.png', dpi=80); plt.close(); print(nm, 'ok', best[1:3] if best else '사진 없음')
