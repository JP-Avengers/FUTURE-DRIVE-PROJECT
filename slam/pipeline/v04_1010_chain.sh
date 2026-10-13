#!/bin/bash
# v0.4 준비: 10/10 보강 회차(R11 시계, R14·R12 반시계)를 회차마다
#   depth 보정 → 단독 매핑 → 내보내기 → 주차선 → v0.3 좌표 강체 정합 → 비강체 정합(g<n>nr3)
# 2026-10-10~11 실행 기록. 경로는 B 작업 폴더 기준(~/b_slam, ~/data, ~/data_corr/c0.007, ~/maps).
source /opt/ros/jazzy/setup.bash
set -u
PY=~/tsdf_env/bin/python; S=~/b_slam/scripts; D=~/data_corr/c0.007
for spec in 11:R11_20261010_171239_855457 14:R14_20261010_172124_867532 12:R12_20261010_171702_909577; do
  n=${spec%%:*}; cap=${spec#*:}; RUN=g$n
  echo "######## $RUN = $cap  $(date +%T)"
  [ -e $D/$cap/bag ] || { mkdir -p $D/$cap; python3 $S/depth_correct_bag.py ~/data/$cap/bag $D/$cap/bag 0.007 | tail -1 || { echo 보정실패; continue; }; }
  if [ ! -e ~/maps/$RUN.db ]; then env IMU=true BAG_ALIGNED=true $S/run_rtabmap_retry.sh $D/$cap/bag $RUN 0.5 | tail -2 || { echo "$RUN 매핑 실패"; continue; }; fi
  [ -e ~/b_slam/exports/$RUN ] || $S/export_run.sh $RUN > /dev/null
  ~/slam_env/bin/python $S/run_metrics.py $RUN
  # 바닥 높이 띠: 궤적 3m 안 점의 (점 높이 - 카메라 높이) 히스토그램 최빈값 ±0.35m(주차선), ±0.2m(정합)
  read GP GA <<< $($PY - $RUN <<'X'
import sys, numpy as np, open3d as o3d
from pathlib import Path
from scipy.spatial import cKDTree
E = Path.home() / f'b_slam/exports/{sys.argv[1]}'
P = np.asarray(o3d.io.read_point_cloud(str(E / 'cloud_cloud.ply')).points); t = np.loadtxt(E / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]
d, j = cKDTree(t[:, :2]).query(P[:, :2]); h = P[d < 3, 2] - t[j[d < 3], 2]; hh, e = np.histogram(h, np.arange(-2.5, 0, 0.02)); m = e[hh.argmax()] + 0.01
print(f'{m-0.35:.2f},{m+0.35:.2f} {m-0.2:.2f},{m+0.2:.2f}')
X
)
  echo "  바닥 높이 띠: 주차선 $GP, 정합 $GA"
  [ -e ~/b_slam/exports/${RUN}_paint_1.5-15m ] || GROUND=$GP DMIN=1.5 $PY $S/keyframe_paint.py $RUN 1.5 15 | tail -1
  $PY $S/align_paint2d.py ${RUN}_paint_1.5-15m v03_paintref align_$RUN
  $PY $S/align_height.py $RUN align_$RUN ~/b_slam/deliver/map_v0.3/geometry_rawdepth_tsdf_5cm.ply "$GA" | grep -E "맞춘 뒤|평면"
  mkdir -p ~/b_slam/exports/${RUN}_paintkf && cp ~/b_slam/exports/${RUN}_paint_1.5-15m/paint_xy.npy ~/b_slam/exports/${RUN}_paintkf/
  $PY $S/nonrigid_to_v03.py $RUN "$GA" --reflocal 0.3 --out ${RUN}nr3 2>&1 | grep -E "키프레임|앵커|최적화|변형|비강체|강체"
done
echo "완료 $(date +%T)"
