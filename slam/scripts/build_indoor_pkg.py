"""실내 맵 패키지 조립 — C 실내 틀(recon/configs/templates/indoor_rtabmap.yaml) 입력 형식:
 keyframes/{rgb,depth,calib}/<id>.*  +  trajectory_camera_optical.txt (시각 x y z qx qy qz qw id, 카메라 광학 좌표계 → 맵)   ← 주 묶음
 keyframes_<이름>/ + trajectory_<이름>_camera_optical.txt   ← 보조 묶음(--set 이름:회차[:q], q = in_localize 벽 일치 비율 하한)
 validation/keyframes + validation/trajectory_camera_optical.txt   ← 맵에 안 쓴 검증 회차(--val 회차)
 + trajectory_camera_link.txt · geometry_tsdf_2cm.ply(바닥 아래 --crop m 잘라냄) · topview.png · qa/ · measurements.md · README.md · checksums.sha256
사용: build_indoor_pkg.py <run_m> <mesh.ply> <out_dir> <readme.md> [--set ...] [--val run] [--crop -0.08] [--note 문장 ...] [--qa 파일 ...]"""
import sys, re, shutil, hashlib, numpy as np, open3d as o3d
from pathlib import Path
a = sys.argv; run, mesh, O, RD = a[1], Path(a[2]), Path(a[3]), Path(a[4])
multi = lambda k: [a[i + 1] for i in range(len(a) - 1) if a[i] == k]; one = lambda k, d: a[a.index(k) + 1] if k in a else d
E = Path.home() / 'b_slam/exports'
if O.exists(): raise SystemExit(f'이미 있음: {O}')
(O / 'qa').mkdir(parents=True); cal = set(); summ = []
def put_set(r, kdir, traj, qmin=0.0):
    K = E / r / 'keyframes'; cid = np.loadtxt(E / r / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); cid = cid[np.argsort(cid[:, 0])]
    f = E / 'indoor' / f'{r[:-1]}_localize.npz'
    if qmin > 0 and f.exists(): z = np.load(f); fr = {int(i): float(q) for i, q in zip(z['ids'], z['frac'])}; cid = cid[[fr.get(int(i), 0) >= qmin for i in cid[:, 8]]]
    for s in ('rgb', 'depth', 'calib'): (kdir / s).mkdir(parents=True)
    for r_ in cid:
        i = int(r_[8]); shutil.copy2(K / f'kf_rgb/{i}.jpg', kdir / f'rgb/{i}.jpg'); shutil.copy2(K / f'kf_depth/{i}.png', kdir / f'depth/{i}.png'); shutil.copy2(K / f'kf_calib/{i}.yaml', kdir / f'calib/{i}.yaml')
        cal.add(re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').replace(' ', ''))
    np.savetxt(traj, cid, fmt=['%.6f'] * 8 + ['%d'], header='timestamp x y z qx qy qz qw id  (카메라 광학 좌표계 -> 맵. 맵 = 건물 좌표: 복도 축 x, 바닥 z=0, m)')
    summ.append(f'{kdir.relative_to(O).as_posix()}: {len(cid)}장, 카메라 높이 중앙 {np.median(cid[:, 3]):.2f} m'); return cid
cid = put_set(run, O / 'keyframes', O / 'trajectory_camera_optical.txt')
tl = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2); np.savetxt(O / 'trajectory_camera_link.txt', tl, fmt='%.6f', header='timestamp x y z qx qy qz qw  (camera_link -> 맵)')
for s in multi('--set'):
    nm, r, q = (s.split(':') + ['0'])[:3]; put_set(r, O / f'keyframes_{nm}', O / f'trajectory_{nm}_camera_optical.txt', float(q))
if '--val' in a: (O / 'validation').mkdir(); put_set(one('--val', ''), O / 'validation/keyframes', O / 'validation/trajectory_camera_optical.txt')
m = o3d.io.read_triangle_mesh(str(mesh)); zc = float(one('--crop', -0.08)); V = np.asarray(m.vertices); T = np.asarray(m.triangles); keep = (V[:, 2] >= zc)[T].all(1); n0 = len(T)
m.triangles = o3d.utility.Vector3iVector(T[keep]); m.remove_unreferenced_vertices(); m.compute_vertex_normals(); o3d.io.write_triangle_mesh(str(O / 'geometry_tsdf_2cm.ply'), m)
for q in multi('--qa'): q = Path(q); shutil.copy2(q, O / ('topview.png' if q.name.endswith('_top.png') else f'qa/{q.name}'))
V = np.asarray(m.vertices); w = V[(V[:, 2] > 0.4) & (V[:, 2] < 1.4)]
h, e = np.histogram(w[:, 1], bins=np.arange(w[:, 1].min(), w[:, 1].max(), 0.02)); ctr = (e[:-1] + e[1:]) / 2; p1 = int(np.argmax(h)); h2 = h.copy(); h2[np.abs(ctr - ctr[p1]) < 1.5] = 0; p2 = int(np.argmax(h2)); Y = sorted([float(ctr[p1]), float(ctr[p2])])
lines = ['# 실측 대조용 수치 (맵에서 잰 값)', '', '키프레임 묶음:'] + [f'- {s}' for s in summ] + ['',
         f'- 형상 메쉬: 삼각형 {len(m.triangles):,} (바닥 아래 {zc:+.2f} m 밑 {n0 - int(keep.sum()):,}개 잘라냄), 높이 {V[:, 2].min():+.2f} ~ {V[:, 2].max():+.2f} m',
         f'- 복도 벽 선: y = {Y[0]:+.2f} m, {Y[1]:+.2f} m → **복도 폭 {Y[1] - Y[0]:.2f} m** (걸레받이 위 벽면 기준, 높이 0.4~1.4 m)', '']
for nm, yw in (('y 작은 쪽 벽', Y[0]), ('y 큰 쪽 벽', Y[1])):
    x = w[np.abs(w[:, 1] - yw) < 0.08, 0]; occ = np.zeros(int((x.max() - x.min()) / 0.05) + 1, bool); occ[((x - x.min()) / 0.05).astype(int)] = True; gaps = []; i = 0
    while i < len(occ):
        if not occ[i]:
            j = i
            while j < len(occ) and not occ[j]: j += 1
            if (j - i) * 0.05 >= 0.6: gaps.append((x.min() + i * 0.05, x.min() + j * 0.05))
            i = j
        else: i += 1
    lines.append(f'- {nm}: 벽면이 x {x.min():+.1f} ~ {x.max():+.1f} m 에 있음. 벽면이 60 cm 넘게 끊긴 곳(문 · 로비 입구 · 옆 통로 후보): ' + (', '.join(f'x {a_:+.1f}~{b_:+.1f} m (폭 {b_ - a_:.1f} m)' for a_, b_ in gaps) or '없음'))
lines += [f'- {n}' for n in multi('--note')] + ['', '줄자로 재면 좋은 것: 복도 폭(벽면에서 벽면), 로비 입구 폭, 문과 문 사이 한 구간 길이.']
(O / 'measurements.md').write_text('\n'.join(lines) + '\n', encoding='utf-8'); print('\n'.join(lines)); shutil.copy2(RD, O / 'README.md')
def sha(p):
    hh = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): hh.update(b)
    return hh.hexdigest()
L = [f'{sha(p)}  {p.relative_to(O).as_posix()}' for p in sorted(O.rglob('*')) if p.is_file() and p.name != 'checksums.sha256']
(O / 'checksums.sha256').write_text('\n'.join(L) + '\n', encoding='utf-8'); print(f'내부 파라미터 종류 {len(cal)} | 파일 {len(L)} → {O}')
