#!/usr/bin/env python3
"""map_v0.4 조립: v0.3 패키지를 복사하고(데이터 그대로), 새 회차 키프레임·궤적(v0.3 좌표계, 비강체 정합), 점군(빈 곳만 10cm), 형상 메시를 넣는다.
사용: build_v04.py <out_pkg_dir> <geometry.ply> <src:nr:tag:id_offset> ...   (~/tsdf_env)
  예) build_v04.py ~/b_slam/deliver/map_v0.4 exports/v04build/geometry.ply gn03:gn03nr3:1009_cw:20000 gm03:gm03nr3:1009_ccw:30000
  점군은 exports/v04build/<src>_cloud_v03.ply(warp_cloud.py 결과)를 씀."""
import sys, re, shutil
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
OUT, GEO = Path(sys.argv[1]), Path(sys.argv[2]); SETS = [a.split(':') for a in sys.argv[3:]]
E = Path.home() / 'b_slam/exports'; W = E / 'v04build'; V03 = Path.home() / 'b_slam/deliver/map_v0.3'
if OUT.exists(): sys.exit(f'이미 존재: {OUT} (덮어쓰지 않음)')
shutil.copytree(V03, OUT)
(OUT / 'checksums.sha256').unlink()
for f in ('HANDOVER.md', 'HANDOVER.pdf'): (OUT / f).rename(OUT / f.replace('HANDOVER', 'HANDOVER_v0.3'))
for src, nr, tag, off in SETS:
    off = int(off); K = E / src / 'keyframes'; D = OUT / f'keyframes_{tag}'
    A = np.loadtxt(E / nr / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)
    B = np.loadtxt(E / nr / 'traj_optimized_poses.txt', comments='#', ndmin=2)
    gf = E / nr / 'kf_good.npy'; good = np.load(gf) if gf.exists() else np.ones(len(A), bool)  # 10/10: 기존 데이터와 맞는 구간만
    A, B = A[good], B[good]; keep = set(A[:, 8].astype(int))
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
    ids = set(A[:, 8].astype(int)); have = {int(f.stem) for f in (D / 'rgb').iterdir()}
    print(f'{tag}: 키프레임 {len(have)}장(ID {min(have)}~{max(have)}), 궤적 {len(A)}개(제외 {int((~good).sum())}), 궤적에만 있고 영상 없는 ID {len(ids - have)}개')
# 점군: v0.3 그대로 + 새 회차 점 중 기존 점과 10cm 안에 없는 것 (회차 순서대로)
base = o3d.io.read_point_cloud(str(V03 / 'cloud_3cm.ply')); pts = [np.asarray(base.points)]; cols = [np.asarray(base.colors)]; n0 = len(pts[0])
for src, nr, tag, off in SETS:
    c = o3d.io.read_point_cloud(str(W / f'{src}_cloud_v03.ply')); X = np.asarray(c.points)
    d, _ = cKDTree(np.vstack(pts)).query(X, distance_upper_bound=0.10); keep = np.isinf(d)
    pts.append(X[keep]); cols.append(np.asarray(c.colors)[keep]); print(f'점군 {tag}: {len(X):,} 중 새로 추가 {keep.sum():,}')
pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.vstack(pts))); pc.colors = o3d.utility.Vector3dVector(np.vstack(cols))
o3d.io.write_point_cloud(str(OUT / 'cloud_3cm.ply'), pc); print(f'cloud_3cm.ply: {n0:,} → {len(pc.points):,}점')
shutil.copy2(GEO, OUT / 'geometry_rawdepth_tsdf_5cm.ply'); print('형상 메시 복사:', GEO)
