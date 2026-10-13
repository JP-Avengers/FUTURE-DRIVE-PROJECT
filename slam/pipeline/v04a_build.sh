#!/bin/bash
# 맵 v0.4a(v0.3 기준 빈 곳 채우기판) 조립. 2026-10-11 실행 기록.
# 당시 출력 폴더는 deliver/map_v0.4였고, v0.4b를 함께 내면서 map_v0.4a로 이름을 바꿨다.
# 먼저 있어야 하는 것:
#   - 10/9 단독 맵 gn03(R1→R3→R5 시계)·gm03(R2→R4→R6 반시계): scripts/chain_new_only.sh,
#     v0.3 좌표 강체 정합 결과 align_gn03·align_gm03(scripts/align_paint2d.py, align_height.py)
#   - 10/10 회차 g11·g14·g12와 비강체 정합 결과: pipeline/v04_1010_chain.sh
#   - R7(10/9 도보) 구멍 3번 바닥 조각: exports/gw01_hole3/gw01_hole3_floor.ply
source /opt/ros/jazzy/setup.bash
set -u
PY=~/tsdf_env/bin/python; S=~/b_slam/scripts; E=~/b_slam/exports; W=$E/v04build; D=~/data_corr/c0.007
mkdir -p $W
echo "== 0. 10/9 맵 비강체 정합 → v0.3 좌표 ($(date +%T))"
for r in gn03 gm03; do [ -e $E/${r}nr3 ] || $PY $S/nonrigid_to_v03.py $r --reflocal 0.3 --out ${r}nr3 2>&1 | grep -E "최적화|변형|비강체"; done
echo "== 1. 보정 bag R1~R6 ($(date +%T))"
for cap in ~/data/R[1-6]_20261009_*; do n=$(basename $cap); [ -e $D/$n/bag ] || { mkdir -p $D/$n; python3 $S/depth_correct_bag.py $cap/bag $D/$n/bag 0.007 | tail -1; }; done
echo "== 2. TSDF (비강체 자세, v0.3 좌표) ($(date +%T))"
tsdf() { nr=$1; shift; [ -e $E/$nr/tsdf_rawdepth_5cm_min1m.ply ] || $PY $S/raw_depth_tsdf.py $nr "$@" | tail -7; }
tsdf gn03nr3 $D/R1_20261009_*/bag $D/R3_20261009_*/bag $D/R5_20261009_*/bag
tsdf gm03nr3 $D/R2_20261009_*/bag $D/R4_20261009_*/bag $D/R6_20261009_*/bag
for n in 11 14 12; do
  tsdf g${n}nr3 $D/R${n}_20261010_*/bag
  $PY $S/crop_by_kf.py g${n}nr3 $E/g${n}nr3/tsdf_rawdepth_5cm_min1m.ply $W/g${n}_tsdf_good.ply   # 10/10은 v0.3·10/9와 맞는 키프레임 구간만
done
echo "== 3. 형상 합치기: v0.3 고정, 빈 곳만 ($(date +%T))"
G=~/b_slam/deliver/map_v0.3/geometry_rawdepth_tsdf_5cm.ply; i=0
for spec in gn03:$E/gn03nr3/tsdf_rawdepth_5cm_min1m.ply gm03:$E/gm03nr3/tsdf_rawdepth_5cm_min1m.ply g11:$W/g11_tsdf_good.ply g14:$W/g14_tsdf_good.ply g12:$W/g12_tsdf_good.ply; do
  i=$((i+1)); nm=${spec%%:*}; M=${spec#*:}; O=$W/geo_${i}_${nm}.ply
  echo "-- +$nm $(date +%T)"; [ -e $O ] || (cd ~/b_slam && $PY $S/mesh_fill.py $G $M $O deliver/map_v0.3/trajectory_camera_link.txt 2>&1 | grep -v Warning)
  [ -e $O ] || { echo "합치기 실패 $nm → 중단"; exit 1; }; G=$O
done
echo "-- +R7 구멍 3번 평탄 조각 $(date +%T)"
$PY $S/flatten_patch.py $G $E/gw01_hole3/gw01_hole3_floor.ply $W/r7_hole3_flat.ply
$PY $S/patch_merge_flat.py $G $W/r7_hole3_flat.ply $W/geometry_final.ply
echo "== 4. 점군 v0.3 좌표로 ($(date +%T))"
for s in gn03 gm03; do $PY $S/warp_cloud.py $s ${s}nr3 $W/${s}_cloud_v03.ply; done
for n in 11 14 12; do $PY $S/warp_cloud.py g$n g${n}nr3 $W/g${n}_cloud_all.ply && $PY $S/crop_by_kf.py g${n}nr3 $W/g${n}_cloud_all.ply $W/g${n}_cloud_v03.ply; done
echo "== 5. 패키지 조립 ($(date +%T))"
$PY $S/build_v04.py ~/b_slam/deliver/map_v0.4 $W/geometry_final.ply gn03:gn03nr3:1009_cw:20000 gm03:gm03nr3:1009_ccw:30000 g11:g11nr3:1010_cw:40000 g14:g14nr3:1010_ccw:50000 g12:g12nr3:1010_ccw2:60000
$PY $S/hole_fill_v03.py ~/b_slam/deliver/map_v0.4 | tail -13
$PY $S/topview_pkg2.py ~/b_slam/deliver/map_v0.4 > /dev/null
echo "완료 $(date +%T)"
