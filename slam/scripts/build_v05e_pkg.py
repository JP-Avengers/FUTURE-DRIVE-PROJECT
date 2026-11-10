"""v0.5e 패키지 조립.
 island: v0.5 패키지 전체(바뀌지 않은 파일은 하드링크) + 형상·바닥 지도·점군 교체 + 손 촬영 키프레임 묶음(keyframes_hand_fecen) + qa/v05e
 delta : 바뀐 파일만(형상·바닥 지도·점군·덧붙임만·README) — island 폴더에 덮어쓰는 실험판
사용: build_v05e_pkg.py island <build_dir> <out_pkg> <T_ms.npy> <readme.md> <changes_head.md>
      build_v05e_pkg.py delta  <build_dir> <out_dir> <readme.md> <added.ply>"""
import sys, os, shutil, hashlib, json, numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
mode = sys.argv[1]; P = Path.home() / 'b_slam/deliver/map_v0.5'; E = Path.home() / 'b_slam/exports'
def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): h.update(b)
    return h.hexdigest()
def lower_extended(floor_npz, out_ply):
    """지도 밖 바닥(0.5m 격자)을 손 촬영으로 채운 칸(conf=4) 1m 안에서 새 바닥 높이 -1cm 아래로 내림(새 바닥 위로 비치지 않게)"""
    import open3d as o3d
    from scipy import ndimage
    F = np.load(floor_npz); G, lo, c, conf = F['grid'].astype(float), F['lo'], float(F['cell']), F['conf']
    near = ndimage.binary_dilation(conf == 4, iterations=int(round(1.0 / c)))
    m = o3d.io.read_triangle_mesh(str(P / 'ground_extended_0.5m.ply')); V = np.asarray(m.vertices).copy()
    ix = np.clip(((V[:, 0] - lo[0]) / c).astype(int), 0, G.shape[0] - 1); iy = np.clip(((V[:, 1] - lo[1]) / c).astype(int), 0, G.shape[1] - 1)
    gz = ndimage.map_coordinates(G, [(V[:, 0] - lo[0]) / c - 0.5, (V[:, 1] - lo[1]) / c - 0.5], order=1, mode='nearest') - 0.01
    sel = near[ix, iy] & (V[:, 2] > gz); dz = V[sel, 2] - gz[sel]; V[sel, 2] = gz[sel]
    m.vertices = o3d.utility.Vector3dVector(V); m.compute_vertex_normals(); o3d.io.write_triangle_mesh(str(out_ply), m)
    print(f'지도 밖 바닥: 꼭짓점 {len(V):,} 중 {int(sel.sum())}개 내림(중앙 {np.median(dz) * 100 if len(dz) else 0:.1f}cm, 최대 {dz.max() * 100 if len(dz) else 0:.1f}cm)')
def checksums(O):
    lines = [f'{sha(p)}  {p.relative_to(O).as_posix()}' for p in sorted(O.rglob('*')) if p.is_file() and p.name != 'checksums.sha256']
    (O / 'checksums.sha256').write_text('\n'.join(lines) + '\n', encoding='utf-8'); return len(lines)
if mode == 'island':
    W, O, T, RD, CH = Path(sys.argv[2]), Path(sys.argv[3]), np.load(sys.argv[4]), Path(sys.argv[5]), Path(sys.argv[6])
    if O.exists(): raise SystemExit(f'이미 있음: {O}')
    REPL = {'geometry_clean_5cm.ply', 'floor_height.npz', 'floor_height.json', 'cloud_3cm.ply', 'README.md', 'CHANGES.md', 'checksums.sha256', 'ground_extended_0.5m.ply'}
    O.mkdir(parents=True)
    for p in sorted(P.iterdir()):
        if p.name in REPL: continue
        if p.is_dir(): shutil.copytree(p, O / p.name, copy_function=os.link)
        else: os.link(p, O / p.name)
    shutil.copy2(W / 'out/geometry_v05e.ply', O / 'geometry_clean_5cm.ply'); shutil.copy2(W / 'out/floor_height.npz', O / 'floor_height.npz')
    shutil.copy2(W / 'out/floor_height.json', O / 'floor_height.json'); shutil.copy2(W / 'out/cloud_v05e_3cm.ply', O / 'cloud_3cm.ply')
    shutil.copy2(W / 'guard/added_low_guarded.ply', O / 'hand_added_v05e.ply'); lower_extended(W / 'out/floor_height.npz', O / 'ground_extended_0.5m.ply')
    q = O / 'qa/v05e'; q.mkdir(parents=True)
    shutil.copy2(W / 'out/floor_conf_v05e.png', q / 'floor_conf_v05e.png'); shutil.copy2(W / 'guard/route_guard.json', q / 'route_guard.json')
    for f in sorted((W / 'preview').glob('*.jpg')): shutil.copy2(f, q / f.name)
    shutil.copy2(W / 'chk/pair_island.png', q / 'island_ortho_vs_added.png')
    # 손 촬영 키프레임 묶음(fecen 세션, 경로 10m 이후) — v0.5 좌표(T_ms + 높이 보정), 광학 자세
    cid = np.loadtxt(E / 'fems5/traj_camera_id_camera_poses.txt', comments='#', ndmin=2); DZ = np.load(W / 'fems5_zfix.npy')
    t0, t1, smin = 1794126172.0, 1794126316.2, 10.0
    iu = np.where((cid[:, 0] >= t0) & (cid[:, 0] <= t1))[0]; su = np.r_[0, np.cumsum(np.linalg.norm(np.diff(cid[iu, 1:4], axis=0), axis=1))]; iu = iu[su >= smin]
    K = E / 'fems5/keyframes'; D = O / 'keyframes_hand_fecen'
    for sub in ('rgb', 'depth', 'calib'): (D / sub).mkdir(parents=True)
    rows = []
    for j in iu:
        r = cid[j]; i = int(r[8])
        shutil.copy2(K / f'kf_rgb/{i}.jpg', D / f'rgb/{i}.jpg'); shutil.copy2(K / f'kf_depth/{i}.png', D / f'depth/{i}.png'); shutil.copy2(K / f'kf_calib/{i}.yaml', D / f'calib/{i}.yaml')
        pw = T[:3, :3] @ r[1:4] + T[:3, 3] + [0, 0, DZ[j]]; qw = Rot.from_matrix(T[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix()).as_quat()
        rows.append(np.r_[r[0], pw, qw, i])
    np.savetxt(O / 'trajectory_hand_fecen_camera_optical.txt', np.array(rows), fmt=['%.6f'] * 8 + ['%d'], header='timestamp x y z qx qy qz qw id  (손 촬영 11/8 fecen, v0.5 좌표, 광학 자세)')
    shutil.copy2(W / 'fems5_fecen_trajectory_camera_link.txt', O / 'trajectory_hand_fecen_camera_link.txt')
    shutil.copy2(RD, O / 'README.md')
    (O / 'CHANGES.md').write_text(CH.read_text(encoding='utf-8') + '\n' + (P / 'CHANGES.md').read_text(encoding='utf-8').split('\n', 3)[-1], encoding='utf-8')
    n = checksums(O); print(f'island 패키지 → {O} (파일 {n}개, 손 촬영 키프레임 {len(iu)}장)')
else:
    W, O, RD, ADD = Path(sys.argv[2]), Path(sys.argv[3]), Path(sys.argv[4]), Path(sys.argv[5])
    if O.exists(): raise SystemExit(f'이미 있음: {O}')
    O.mkdir(parents=True)
    shutil.copy2(W / 'out/geometry_v05e.ply', O / 'geometry_clean_5cm.ply'); shutil.copy2(W / 'out/floor_height.npz', O / 'floor_height.npz')
    shutil.copy2(W / 'out/floor_height.json', O / 'floor_height.json'); shutil.copy2(W / 'out/cloud_v05e_3cm.ply', O / 'cloud_3cm.ply')
    shutil.copy2(ADD, O / 'hand_added_v05e_full.ply'); shutil.copy2(W / 'out/floor_conf_v05e.png', O / 'floor_conf_v05e_full.png')
    lower_extended(W / 'out/floor_height.npz', O / 'ground_extended_0.5m.ply')
    shutil.copy2(RD, O / 'README_EXPERIMENTAL.md'); n = checksums(O); print(f'실험판(덮어쓰기용) → {O} (파일 {n}개)')
