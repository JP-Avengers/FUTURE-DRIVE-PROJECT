#!/bin/bash
# 맵 v0.4b 2단계: 기준 gn04(본네트 시계 R1·R3·R5·R11)를 v0.3 좌표로 강체 1회 배치 → 기준 바닥·주차선·메시
#                 → 반시계 본네트 gm05와 지붕 run18(시계)·run20(반시계)을 기준에 비강체 정합 → TSDF
# 2026-10-11 실행 기록(rebuild 3단계 A·B를 최종 설정으로 정리).
source /opt/ros/jazzy/setup.bash
set -u
PY=~/tsdf_env/bin/python; S=~/b_slam/scripts; E=~/b_slam/exports; D=~/data_corr/c0.007; B=$E/gn04base
# 주차선 점: paint <run> <바닥 높이 띠(카메라 기준)> <최소 거리 m>
paint() { r=$1; g=$2; m=$3; [ -e $E/${r}_paintkf/paint_xy.npy ] || { GROUND=$g DMIN=$m $PY $S/keyframe_paint.py $r 1.5 15 > /dev/null && mkdir -p $E/${r}_paintkf && cp $E/${r}_paint_1.5-15m/paint_xy.npy $E/${r}_paintkf/; }; echo "  주차선 $r: $(python3 -c "import numpy as np;print(len(np.load('$E/${r}_paintkf/paint_xy.npy')))")점"; }
echo "== 1. 기준 gn04 → v0.3 좌표 (강체) $(date +%T)"
paint gn04 -1.4,-0.7 1.5
$PY $S/align_paint2d.py gn04_paintkf v03_paintref align_gn04 | tail -3
$PY $S/align_height.py gn04 align_gn04 ~/b_slam/deliver/map_v0.3/geometry_rawdepth_tsdf_5cm.ply "-1.2,-0.75" | grep -E "맞춘 뒤|평면"
mkdir -p $B; ln -sfn $E/gn04/keyframes $B/keyframes
$PY $S/apply_T.py $E/align_gn04/T_src_to_ref.npy $E/gn04/traj_optimized_poses.txt $B/traj_optimized_poses.txt
$PY $S/apply_T.py $E/align_gn04/T_src_to_ref.npy $E/gn04/traj_camera_id_camera_poses.txt $B/traj_camera_id_camera_poses.txt
$PY $S/apply_T.py $E/align_gn04/T_src_to_ref.npy $E/gn04/cloud_cloud.ply $B/cloud_cloud.ply
$PY - <<'X'
import numpy as np, open3d as o3d
from pathlib import Path
from scipy.spatial import cKDTree
E = Path.home() / 'b_slam/exports'; B = E / 'gn04base'; T = np.load(E / 'align_gn04/T_src_to_ref.npy')
p = np.load(E / 'gn04_paintkf/paint_xy.npy'); np.save(B / 'paint_xy.npy', p @ T[:2, :2].T + T[:2, 3])
P = np.asarray(o3d.io.read_point_cloud(str(B / 'cloud_cloud.ply')).points); t = np.loadtxt(B / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]
_, j = cKDTree(t[:, :2]).query(P[:, :2]); h = P[:, 2] - t[j, 2]; F = P[(h > -1.2) & (h < -0.75)]; np.save(B / 'ref_floor.npy', F)
print(f'기준 바닥 {len(F):,}점, 기준 주차선 {len(p):,}점')
X
echo "== 2. 기준 메시 (gn04: R1·R3·R5·R11) $(date +%T)"
[ -e $B/tsdf_rawdepth_5cm_min1m.ply ] || $PY $S/raw_depth_tsdf.py gn04base $D/R1_20261009_*/bag $D/R3_20261009_*/bag $D/R5_20261009_*/bag $D/R11_20261010_*/bag | tail -6
echo "== 3. 반시계 gm05·지붕 run18·run20 → 기준 $(date +%T)"
paint gm05 -1.4,-0.7 1.5; paint run18 -1.9,-1.2 3; paint run20 -1.9,-1.2 3
for spec in gm05:-1.2,-0.75 run18:-1.95,-1.15 run20:-1.95,-1.15; do
  r=${spec%%:*}; band=${spec#*:}
  $PY $S/align_to_ref.py $r ${r}_paintkf gn04base $B/ref_floor.npy align_${r}_b $band | tail -4
done
ref="--ref_floor $B/ref_floor.npy --ref_paint $B/paint_xy.npy"
# 반시계 본네트는 기본 설정, 지붕은 키프레임 사이 상대 이동 허용을 6cm로(--st_r 0.06)
$PY $S/nonrigid_to_v03.py gm05 -1.2,-0.75 $ref --init $E/align_gm05_b/T_src_to_ref.npy --out gm05b 2>&1 | grep -E "최적화|남은|변형|비강체\]"
for r in run18 run20; do
  $PY $S/nonrigid_to_v03.py $r -1.95,-1.15 $ref --init $E/align_${r}_b/T_src_to_ref.npy --st_r 0.06 --out ${r}b6 2>&1 | grep -E "최적화|남은|변형|비강체\]"
done
echo "== 4. TSDF (기준에 맞춘 자세) $(date +%T)"
[ -e $E/gm05b/tsdf_rawdepth_5cm_min1m.ply ] || $PY $S/raw_depth_tsdf.py gm05b $D/R2_20261009_*/bag $D/R4_20261009_*/bag $D/R6_20261009_*/bag $D/R12_20261010_*/bag $D/R14_20261010_*/bag | tail -6
[ -e $E/run18b6/tsdf_rawdepth_5cm_min1m.ply ] || $PY $S/raw_depth_tsdf.py run18b6 $D/car_test_3_*/bag $D/car_test_2_*/bag $D/car_test1_*/bag | tail -1
[ -e $E/run20b6/tsdf_rawdepth_5cm_min1m.ply ] || $PY $S/raw_depth_tsdf.py run20b6 $D/car_test_reverse_1_*/bag $D/car_test_reverse_stone_*/bag | tail -1
echo "완료 $(date +%T)"
