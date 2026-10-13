#!/bin/bash
# 맵 v0.4b 3단계: 회차별 TSDF → 형상 합치기(빈 곳만, 바닥 겹침 규칙) → R7 평탄 조각 → 경계 보정 → 점군 → 패키지 조립·검증
# 2026-10-11 다섯 번째 조립(최종본) 기록. 앞선 조립과 다른 점: 여러 회차를 TSDF 하나로 합치면 날마다 다른
# 주차 차량 때문에 섬 가장자리 바닥이 지워져서, 회차별 TSDF를 빈 곳 채우기에 더했다.
source /opt/ros/jazzy/setup.bash
set -u
PY=~/tsdf_env/bin/python; S=~/b_slam/scripts; E=~/b_slam/exports; D=~/data_corr/c0.007; W=$E/v04bbuild; DV=~/b_slam/deliver
mkdir -p $W
echo "== 1. 회차별 TSDF $(date +%T)"
# 합친 맵(gn04base, gm05b)의 키프레임 자세를 DB의 세션(map_id)별로 나눔 → exports/<nr>_s<세션>
$PY - <<'X'
import sqlite3, numpy as np
from pathlib import Path
E = Path.home() / 'b_slam/exports'
for nr, db in (('gn04base', 'gn04'), ('gm05b', 'gm05')):
    A = np.loadtxt(E / nr / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); B = np.loadtxt(E / nr / 'traj_optimized_poses.txt', comments='#', ndmin=2)
    mid = dict(sqlite3.connect(Path.home() / f'maps/{db}.db').execute('select id, map_id from Node').fetchall()); ms = np.array([mid[int(i)] for i in A[:, 8]])
    for m in sorted(set(ms)):
        d = E / f'{nr}_s{m}'; d.mkdir(exist_ok=True); ts = set(np.round(A[ms == m, 0], 4)); sel = np.array([round(t, 4) in ts for t in B[:, 0]])
        np.savetxt(d / 'traj_optimized_poses.txt', B[sel], fmt='%.6f', header='timestamp x y z qx qy qz qw', comments='#'); print(f'{d.name}: 키프레임 {sel.sum()}')
X
tsdf() { [ -e $E/$1/tsdf_rawdepth_5cm_min1m.ply ] || $PY $S/raw_depth_tsdf.py $1 $2 | grep "통합 →"; }
tsdf gn04base_s0 "$(ls -d $D/R1_20261009_*/bag)"; tsdf gn04base_s1 "$(ls -d $D/R3_20261009_*/bag)"; tsdf gn04base_s2 "$(ls -d $D/R5_20261009_*/bag)"; tsdf gn04base_s3 "$(ls -d $D/R11_20261010_*/bag)"
tsdf gm05b_s0 "$(ls -d $D/R2_20261009_*/bag)"; tsdf gm05b_s1 "$(ls -d $D/R4_20261009_*/bag)"; tsdf gm05b_s2 "$(ls -d $D/R6_20261009_*/bag)"; tsdf gm05b_s3 "$(ls -d $D/R12_20261010_*/bag)"; tsdf gm05b_s4 "$(ls -d $D/R14_20261010_*/bag)"
echo "== 2. 형상 합치기 $(date +%T)"
# 순서: 기준(본네트 시계) → 반시계 본네트 → 회차별 본네트 9개 → 지붕 시계 → 지붕 반시계
G=$E/gn04base/tsdf_rawdepth_5cm_min1m.ply; i=0
for nm in gm05b gn04base_s0 gn04base_s1 gn04base_s2 gn04base_s3 gm05b_s0 gm05b_s1 gm05b_s2 gm05b_s3 gm05b_s4 BONNETDONE run18b6 run20b6; do
  if [ $nm = BONNETDONE ]; then cp $G $W/geo_bonnet_all.ply; continue; fi
  i=$((i+1)); O=$W/geo_$(printf %02d $i)_${nm}.ply
  (cd ~/b_slam && FILL_MINN=4 $PY $S/mesh_fill2.py $G $E/$nm/tsdf_rawdepth_5cm_min1m.ply $O deliver/map_v0.3/trajectory_camera_link.txt 2>&1 | grep -E "^결과") || { echo "실패 $nm"; exit 1; }
  G=$O
done
$PY $S/flatten_patch.py $G $E/gw01_hole3/gw01_hole3_floor.ply $W/r7_flat.ply | head -1
$PY $S/patch_merge_flat.py $G $W/r7_flat.ply $W/geometry_merged.ply | head -1
echo "-- 경계 보정 (본네트 전체 기준 → 기준 맵 기준, 감쇠 2m)"
$PY $S/seam_blend.py $W/geo_bonnet_all.ply $W/geometry_merged.ply $W/geometry_blend1.ply 2.0
$PY $S/seam_blend.py $E/gn04base/tsdf_rawdepth_5cm_min1m.ply $W/geometry_blend1.ply $W/geometry_final.ply 2.0
echo "== 3. 점군 $(date +%T)"
$PY $S/warp_cloud.py gm05 gm05b $W/gm05_cloud.ply; $PY $S/warp_cloud.py run18 run18b6 $W/run18_cloud.ply; $PY $S/warp_cloud.py run20 run20b6 $W/run20_cloud.ply
$PY - <<'X'
import numpy as np, open3d as o3d
from pathlib import Path
from scipy.spatial import cKDTree
E = Path.home() / 'b_slam/exports'; W = E / 'v04bbuild'; pts, cols = [], []
for nm, f in (('본네트 시계(기준)', E / 'gn04base/cloud_cloud.ply'), ('본네트 반시계', W / 'gm05_cloud.ply'), ('지붕 시계', W / 'run18_cloud.ply'), ('지붕 반시계', W / 'run20_cloud.ply')):
    c = o3d.io.read_point_cloud(str(f)); X, C = np.asarray(c.points), np.asarray(c.colors)
    if pts: d, _ = cKDTree(np.vstack(pts)).query(X, distance_upper_bound=0.10); k = np.isinf(d); X, C = X[k], C[k]
    pts.append(X); cols.append(C)
pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.vstack(pts))); pc.colors = o3d.utility.Vector3dVector(np.vstack(cols)); pc = pc.voxel_down_sample(0.03)
o3d.io.write_point_cloud(str(W / 'cloud_3cm.ply'), pc); print(f'  cloud_3cm: {len(pc.points):,}점')
X
echo "== 4. 조립·검증 $(date +%T)"
$PY $S/build_v04b.py $DV/map_v0.4b $W/geometry_final.ply $W/cloud_3cm.ply gn04:gn04base:bonnet_cw:20000 gm05:gm05b:bonnet_ccw:30000 run18:run18b6:roof_cw:0 run20:run20b6:roof_ccw:10000 | tail -1
P=$DV/map_v0.4b; cp $S/example_tsdf_v04b.py $P/example_tsdf.py
# 바닥 구멍은 v0.3과 같은 기준 경로(v0.3 시계)로 센다
H=$W/holes_tmp; mkdir -p $H; ln -sfn $P/geometry_rawdepth_tsdf_5cm.ply $H/; cp $DV/map_v0.3/trajectory_camera_link.txt $H/
$PY $S/holes_pkg.py $H | grep 빈칸; cp $H/floor_holes.png $H/floor_holes.csv $P/; cat $P/floor_holes.csv
$PY $S/hole_fill_v03.py $P | tail -13
$PY $S/topview_pkg3.py $P > /dev/null
C=$S/c_checks.py
$PY $C seamcell $E/gn04base/tsdf_rawdepth_5cm_min1m.ply $P/geometry_rawdepth_tsdf_5cm.ply "v0.4b"
for t in bonnet_cw bonnet_ccw roof_cw roof_ccw; do $PY $C kf $P keyframes_$t trajectory_${t}_camera_optical.txt $( [ $t = roof_cw ] && echo 361-452 ); done
du -sh $P; echo "완료 $(date +%T)"
