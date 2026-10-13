#!/bin/bash
# 맵 v0.4b 1단계: 10/10 회차를 같은 방향 10/9 맵에 이어 붙여 다시 매핑(multi-session, INIT_DB).
#   gn04 = gn03 + R11 (시계), gm04 = gm03 + R12, gm05 = gm04 + R14 (반시계)
# 10/10 단독 맵은 넓은 출구 쪽에서 약 3.5m 휘었는데, 이어 붙이면 사라진다(session_check.py로 판정).
# 2026-10-11 실행 기록. 원본 DB는 바꾸지 않는다(INIT_DB 복사본에서 시작).
source /opt/ros/jazzy/setup.bash
set -u
S=~/b_slam/scripts; D=~/data_corr/c0.007; PY=~/tsdf_env/bin/python
# 비교 기준: 10/9 맵 자체의 마지막 세션 판정
$PY $S/session_check.py gn03 | tail -1; $PY $S/session_check.py gm03 | tail -1
for spec in gn04:gn03:R11_20261010_171239_855457 gm04:gm03:R12_20261010_171702_909577 gm05:gm04:R14_20261010_172124_867532; do
  IFS=: read RUN PREV CAP <<< "$spec"
  echo "######## $RUN = $PREV + $CAP  $(date +%T)"
  [ -e ~/maps/$RUN.db ] || env IMU=true BAG_ALIGNED=true INIT_DB=~/maps/$PREV.db $S/run_rtabmap_retry.sh $D/$CAP/bag $RUN 0.5 | tail -2 || { echo "$RUN 매핑 실패"; continue; }
  ~/slam_env/bin/python $S/db_links.py ~/maps/$RUN.db
  [ -e ~/b_slam/exports/$RUN ] || $S/export_run.sh $RUN > /dev/null
  ~/slam_env/bin/python $S/run_metrics.py $RUN | grep -E "odom lost|traj_opt|loop"
  $PY $S/session_check.py $RUN | tail -1
done
echo "완료 $(date +%T)"
