#!/bin/bash
# 10/9 추가 촬영 8회차를 run18(v0.3 시계 기준)에 차례로 이어 붙임 (gb01~gb08)
D=~/data_corr/c0.007; PREV=~/maps/run18.db; i=0
for r in R1 R3 R5 R2 R4 R6 R7_20261009_173008 R7_2; do
  i=$((i+1)); RUN=$(printf gb%02d $i); B=$(ls -d $D/${r}_2026*/bag 2>/dev/null | head -1); [ -z "$B" ] && B=$(ls -d $D/${r}*/bag | head -1)
  echo "######## $RUN = $(basename $(dirname $PREV) >/dev/null; basename $PREV .db) + $(basename $(dirname $B))  $(date +%H:%M)"
  IMU=true BAG_ALIGNED=true INIT_DB=$PREV ~/b_slam/scripts/run_rtabmap_retry.sh $B $RUN 0.5 || { echo "$RUN 실패 → 중단"; exit 1; }
  ~/slam_env/bin/python ~/b_slam/scripts/db_links.py ~/maps/$RUN.db
  PREV=~/maps/$RUN.db
done
echo "체인 완료 $(date +%H:%M)"
