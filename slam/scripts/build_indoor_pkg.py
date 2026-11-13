"""실내 맵 패키지 조립 — C 실내 틀(recon/configs/templates/indoor_rtabmap.yaml) 입력 형식:
 keyframes/{rgb,depth,calib}/<id>.*  +  trajectory_camera_optical.txt (시각 x y z qx qy qz qw id, 카메라 광학 좌표계 → 맵)
 + trajectory_camera_link.txt · geometry_tsdf_2cm.ply(선택) · topview.png · qa/ · measurements.md · README.md · checksums.sha256
사용: build_indoor_pkg.py <run_m(보정 내보내기)> <mesh.ply> <out_dir> <readme.md> <qa 파일...>"""
import sys, re, shutil, hashlib, json, numpy as np, open3d as o3d
from pathlib import Path
run, mesh, O, RD, QA = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3]), Path(sys.argv[4]), [Path(p) for p in sys.argv[5:]]
E = Path.home() / 'b_slam/exports'; K = E / run / 'keyframes'
if O.exists(): raise SystemExit(f'이미 있음: {O}')
for s in ('rgb', 'depth', 'calib'): (O / 'keyframes' / s).mkdir(parents=True)
(O / 'qa').mkdir()
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); cid = cid[np.argsort(cid[:, 0])]; cal = set()
for r in cid:
    i = int(r[8]); shutil.copy2(K / f'kf_rgb/{i}.jpg', O / f'keyframes/rgb/{i}.jpg'); shutil.copy2(K / f'kf_depth/{i}.png', O / f'keyframes/depth/{i}.png'); shutil.copy2(K / f'kf_calib/{i}.yaml', O / f'keyframes/calib/{i}.yaml')
    cal.add(re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').replace(' ', ''))
np.savetxt(O / 'trajectory_camera_optical.txt', cid, fmt=['%.6f'] * 8 + ['%d'], header='timestamp x y z qx qy qz qw id  (카메라 광학 좌표계 -> 맵. 맵 = 건물 좌표: 복도 축 x, 바닥 z=0, m)')
tl = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2); np.savetxt(O / 'trajectory_camera_link.txt', tl, fmt='%.6f', header='timestamp x y z qx qy qz qw  (camera_link -> 맵)')
shutil.copy2(mesh, O / 'geometry_tsdf_2cm.ply')
for q in QA: shutil.copy2(q, O / ('topview.png' if q.name.endswith('_top.png') else f'qa/{q.name}'))
# 실측 대조용 수치: 벽 선·폭, 벽 끊긴 곳(로비 입구 등)
m = o3d.io.read_triangle_mesh(str(mesh)); V = np.asarray(m.vertices); w = V[(V[:, 2] > 0.4) & (V[:, 2] < 1.4)]
h, e = np.histogram(w[:, 1], bins=np.arange(w[:, 1].min(), w[:, 1].max(), 0.02)); ctr = (e[:-1] + e[1:]) / 2; p1 = int(np.argmax(h)); h2 = h.copy(); h2[np.abs(ctr - ctr[p1]) < 1.5] = 0; p2 = int(np.argmax(h2)); Y = sorted([float(ctr[p1]), float(ctr[p2])])
lines = [f'# 실측 대조용 수치 (맵에서 잰 값)', '', f'- 키프레임 {len(cid)}장, 카메라 높이 중앙 {np.median(cid[:, 3]):.2f} m, 경로 길이 {np.linalg.norm(np.diff(cid[:, 1:3], axis=0), axis=1).sum():.1f} m',
         f'- 복도 벽 선: y = {Y[0]:+.2f} m, {Y[1]:+.2f} m → **복도 폭 {Y[1] - Y[0]:.2f} m** (걸레받이 위 벽면 기준, 높이 0.4~1.4 m)', '']
for nm, yw in (('y 작은 쪽 벽', Y[0]), ('y 큰 쪽 벽', Y[1])):
    x = w[np.abs(w[:, 1] - yw) < 0.08, 0]; occ = np.zeros(int((x.max() - x.min()) / 0.05) + 1, bool); occ[((x - x.min()) / 0.05).astype(int)] = True
    gaps = []; i = 0
    while i < len(occ):
        if not occ[i]:
            j = i
            while j < len(occ) and not occ[j]: j += 1
            if (j - i) * 0.05 >= 0.6: gaps.append((x.min() + i * 0.05, x.min() + j * 0.05))
            i = j
        else: i += 1
    lines.append(f'- {nm}: 벽면이 x {x.min():+.1f} ~ {x.max():+.1f} m 에 있음. 벽면이 60 cm 넘게 끊긴 곳(문 · 로비 입구 · 옆 통로 후보): ' + (', '.join(f'x {a:+.1f}~{b:+.1f} m (폭 {b - a:.1f} m)' for a, b in gaps) or '없음'))
lines += ['', '줄자로 재면 좋은 것: 복도 폭(벽면에서 벽면), 로비 입구 폭, 문과 문 사이 한 구간 길이.']
(O / 'measurements.md').write_text('\n'.join(lines) + '\n', encoding='utf-8'); print('\n'.join(lines))
shutil.copy2(RD, O / 'README.md')
def sha(p):
    hh = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): hh.update(b)
    return hh.hexdigest()
L = [f'{sha(p)}  {p.relative_to(O).as_posix()}' for p in sorted(O.rglob('*')) if p.is_file() and p.name != 'checksums.sha256']
(O / 'checksums.sha256').write_text('\n'.join(L) + '\n', encoding='utf-8'); print(f'내부 파라미터 종류 {len(cal)} | 파일 {len(L)} → {O}')
