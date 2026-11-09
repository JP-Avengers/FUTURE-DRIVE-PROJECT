# feature_* 6회차: depth 보정(c=0.007) → 단독 매핑(IMU, 정렬 depth) → 내보내기 → 지표
source /opt/ros/jazzy/setup.bash
set -u
S=~/b_slam/scripts; D=~/data_corr/c0.007; L=~/b_slam/logs/fe_chain_20261109.log
for spec in fe422a:feature_422wall_20261108_170235_350519 fe422b:feature_422wall2_20261108_172638_526487 fecen:feature_centerparking_20261108_172249_346172 feit1:feature_IT1wall_20261108_165748_109350 feit1w:feature_It1west_20261108_170703_236504 feone:feature_onetake_20261108_171307_500612; do
  RUN=${spec%%:*}; cap=${spec#*:}
  echo "######## $RUN = $cap  $(date +%T)" | tee -a $L
  [ -e $D/$cap/bag ] || { mkdir -p $D/$cap; python3 $S/depth_correct_bag.py ~/data/$cap/bag $D/$cap/bag 0.007 2>&1 | tail -1 | tee -a $L; }
  [ -e ~/maps/$RUN.db ] || env IMU=true BAG_ALIGNED=true $S/run_rtabmap_retry.sh $D/$cap/bag $RUN 0.5 2>&1 | tail -2 | tee -a $L
  [ -e ~/b_slam/exports/$RUN ] || $S/export_run.sh $RUN > /dev/null 2>&1
  ~/slam_env/bin/python $S/run_metrics.py $RUN 2>&1 | grep -E "nodes|odom lost|loop|traj" | tee -a $L
done
echo "완료 $(date +%T)" | tee -a $L
