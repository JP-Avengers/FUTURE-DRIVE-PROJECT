#!/bin/bash
# 새 촬영(10/9)만으로 맵 만들기. 사용: chain_new_only.sh <접두어> <회차...>   예) chain_new_only.sh gn R1 R3 R5
P=$1; shift; D=~/data_corr/c0.007; PREV=; i=0
for r in "$@"; do
  i=$((i+1)); RUN=$(printf "${P}%02d" $i); B=$(ls -d $D/${r}_2026*/bag | head -1)
  echo "######## $RUN = ${PREV:+$(basename $PREV .db) + }$(basename $(dirname $B))  $(date +%H:%M)"
  env IMU=true BAG_ALIGNED=true ${PREV:+INIT_DB=$PREV} ~/b_slam/scripts/run_rtabmap_retry.sh $B $RUN 0.5 || { echo "$RUN 실패 → 중단"; exit 1; }
  ~/slam_env/bin/python ~/b_slam/scripts/db_links.py ~/maps/$RUN.db
  PREV=~/maps/$RUN.db
done
echo "완료 $(date +%H:%M)"
