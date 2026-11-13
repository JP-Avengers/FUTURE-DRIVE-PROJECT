# 실내 복도 5회차: depth 보정(c=0.007) → 단독 매핑(IMU, 정렬 depth) → 내보내기 → 지표
source /opt/ros/jazzy/setup.bash
set -u
S=~/b_slam/scripts; D=~/data_corr/c0.007; L=~/b_slam/logs/in_chain_20261113.log; : > $L
for spec in in1:inside_1_20261108_175518_090414 in1r:inside_1_reverse_20261108_175843_764369 in2:inside_2_20261108_181736_788068 inrw:inside_rightwall_20261108_180317_441342 inlw:inside_leftwall_20261108_181043_498383; do
  RUN=${spec%%:*}; cap=${spec#*:}
  echo "######## $RUN = $cap  $(date +%T)" | tee -a $L
  [ -e $D/$cap/bag ] || { mkdir -p $D/$cap; python3 $S/depth_correct_bag.py ~/data/$cap/bag $D/$cap/bag 0.007 2>&1 | tail -1 | tee -a $L; }
  [ -e ~/maps/$RUN.db ] || env IMU=true BAG_ALIGNED=true $S/run_rtabmap_retry.sh $D/$cap/bag $RUN 0.5 2>&1 | tail -2 | tee -a $L
  [ -e ~/b_slam/exports/$RUN ] || $S/export_run.sh $RUN > /dev/null 2>&1
  ~/slam_env/bin/python $S/run_metrics.py $RUN 2>&1 | grep -E "nodes|odom lost|loop|traj" | tee -a $L
done
echo "완료 $(date +%T)" | tee -a $L
