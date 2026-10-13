#!/usr/bin/env python3
"""map_v0.4(본네트 기준) 조립: 형상 메시·점군은 주어진 파일, 키프레임 묶음은 <src:traj_dir:tag:id_offset>.
 - traj_dir(exports 아래)의 traj_camera_id_camera_poses.txt(광학, ID 열) / traj_optimized_poses.txt(camera_link)를 v0.3 좌표 자세로 사용
 - 키프레임 영상은 exports/<src>/keyframes 에서 (ID + 오프셋)으로 복사, calib의 camera_name도 같이 바꿈
 - 문서·사진 등 나머지는 map_v0.3에서 site_photos, HANDOVER(→ HANDOVER_v0.3.*)만 복사 (README 등은 따로 씀)
사용: build_v04b.py <out_pkg> <geometry.ply> <cloud.ply> <src:traj_dir:tag:off> ...   (~/tsdf_env)"""
import sys, re, shutil
from pathlib import Path
import numpy as np
OUT, GEO, CLOUD = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]); SETS = [a.split(':') for a in sys.argv[4:]]
E = Path.home() / 'b_slam/exports'; V03 = Path.home() / 'b_slam/deliver/map_v0.3'
if OUT.exists(): sys.exit(f'이미 존재: {OUT} (덮어쓰지 않음)')
OUT.mkdir(parents=True)
shutil.copytree(V03 / 'site_photos', OUT / 'site_photos')
for f in ('HANDOVER.md', 'HANDOVER.pdf'): shutil.copy2(V03 / f, OUT / f.replace('HANDOVER', 'HANDOVER_v0.3'))
shutil.copy2(GEO, OUT / 'geometry_rawdepth_tsdf_5cm.ply'); shutil.copy2(CLOUD, OUT / 'cloud_3cm.ply')
for src, td, tag, off in SETS:
    off = int(off); K = E / src / 'keyframes'; D = OUT / f'keyframes_{tag}'
    A = np.loadtxt(E / td / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); B = np.loadtxt(E / td / 'traj_optimized_poses.txt', comments='#', ndmin=2)
    o = np.argsort(A[:, 0]); A = A[o]; B = B[np.argsort(B[:, 0])]
    assert len(A) == len(B) and np.allclose(A[:, 0], B[:, 0], atol=1e-4), f'{td}: 궤적 두 파일 시각 불일치'
    gf = E / td / 'kf_good.npy'
    if gf.exists():  # 기준과 맞는 구간만 (kf_consistency.py 결과, traj_camera_id 파일 순서 = 시각 순)
        g = np.load(gf); A0 = np.loadtxt(E / td / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
        good_t = set(np.round(A0[g, 0], 4)); m = np.array([round(t, 4) in good_t for t in A[:, 0]]); A, B = A[m], B[m]
        print(f'  {tag}: 기준과 맞는 구간만 {m.sum()}/{len(m)}')
    keep = set(A[:, 8].astype(int))
    for sub, dst in (('kf_rgb', 'rgb'), ('kf_depth', 'depth'), ('kf_calib', 'calib')):
        (D / dst).mkdir(parents=True, exist_ok=True)
        for f in (K / sub).iterdir():
            if int(f.stem) not in keep: continue
            g = D / dst / f'{int(f.stem) + off}{f.suffix}'
            if f.suffix == '.yaml': g.write_text(re.sub(r'camera_name: "(\d+)"', lambda m: f'camera_name: "{int(m.group(1)) + off}"', f.read_text()))
            else: shutil.copy2(f, g)
    A[:, 8] += off
    np.savetxt(OUT / f'trajectory_{tag}_camera_optical.txt', A, fmt=['%.6f'] * 8 + ['%d'], header='timestamp x y z qx qy qz qw id', comments='#')
    np.savetxt(OUT / f'trajectory_{tag}_camera_link.txt', B, fmt='%.6f', header='timestamp x y z qx qy qz qw', comments='#')
    have = {int(f.stem) for f in (D / 'rgb').iterdir()}; ids = set(A[:, 8].astype(int))
    print(f'{tag}: 키프레임 {len(have)}장(ID {min(have)}~{max(have)}), 궤적 {len(A)}개, 영상 없는 ID {len(ids - have)}개')
print('조립 완료:', OUT)
