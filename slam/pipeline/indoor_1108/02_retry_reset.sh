# 실내 실패 3회차 재시도: 바뀐 것은 Odom/ResetCountdown=1 하나(추적이 끊기면 다음 프레임에서 자동으로 다시 시작). 새 이름으로 저장(기존 DB 그대로).
source /opt/ros/jazzy/setup.bash
set -u
S=~/b_slam/scripts; D=~/data_corr/c0.007; L=~/b_slam/logs/in_retry_20261113.log; : > $L
for spec in in1b:inside_1_20261108_175518_090414 inrwb:inside_rightwall_20261108_180317_441342 inlwb:inside_leftwall_20261108_181043_498383; do
  RUN=${spec%%:*}; cap=${spec#*:}
  echo "######## $RUN = $cap (Odom/ResetCountdown 1)  $(date +%T)" | tee -a $L
  [ -e ~/maps/$RUN.db ] || env IMU=true BAG_ALIGNED=true $S/run_rtabmap_retry.sh $D/$cap/bag $RUN 0.5 --Odom/ResetCountdown 1 2>&1 | tail -2 | tee -a $L
  ~/slam_env/bin/python $S/db_links.py ~/maps/$RUN.db 2>&1 | tail -3 | tee -a $L
  [ -e ~/b_slam/exports/$RUN ] || $S/export_run.sh $RUN > /dev/null 2>&1
  ~/slam_env/bin/python $S/run_metrics.py $RUN 2>&1 | grep -E "nodes|odom lost|loop|traj" | tee -a $L
done
echo "완료 $(date +%T)" | tee -a $L
