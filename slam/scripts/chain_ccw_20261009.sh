#!/bin/bash
# 반시계 묶음: run20(v0.3 반시계, c=0.007) + R2 → R4 → R6 (gc01~gc03)
D=~/data_corr/c0.007; PREV=~/maps/run20.db; i=0
for r in R2 R4 R6; do
  i=$((i+1)); RUN=$(printf gc%02d $i); B=$(ls -d $D/${r}_2026*/bag | head -1)
  echo "######## $RUN = $(basename $PREV .db) + $(basename $(dirname $B))  $(date +%H:%M)"
  IMU=true BAG_ALIGNED=true INIT_DB=$PREV ~/b_slam/scripts/run_rtabmap_retry.sh $B $RUN 0.5 || { echo "$RUN 실패 → 중단"; exit 1; }
  ~/slam_env/bin/python ~/b_slam/scripts/db_links.py ~/maps/$RUN.db
  PREV=~/maps/$RUN.db
done
echo "반시계 체인 완료 $(date +%H:%M)"
