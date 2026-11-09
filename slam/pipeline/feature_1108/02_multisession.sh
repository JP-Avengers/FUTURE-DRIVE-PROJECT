# 손 촬영 feature_* 6회차를 하나의 맵으로: feone 바탕에 나머지 5개를 차례로 이어 붙임 (원본 DB 무변경, INIT_DB 복사본에서 시작)
source /opt/ros/jazzy/setup.bash
set -u
S=~/b_slam/scripts; D=~/data_corr/c0.007; L=~/b_slam/logs/fe_ms_20261109.log; : > $L
PREV=feone
for spec in fems1:feit1:feature_IT1wall_20261108_165748_109350 fems2:fecen:feature_centerparking_20261108_172249_346172 fems3:fe422a:feature_422wall_20261108_170235_350519 fems4:fe422b:feature_422wall2_20261108_172638_526487 fems5:feit1w:feature_It1west_20261108_170703_236504; do
  IFS=: read RUN NAME CAP <<< "$spec"
  echo "######## $RUN = $PREV + $NAME  $(date +%T)" | tee -a $L
  [ -e ~/maps/$RUN.db ] || env IMU=true BAG_ALIGNED=true INIT_DB=~/maps/$PREV.db $S/run_rtabmap_retry.sh $D/$CAP/bag $RUN 0.5 2>&1 | tail -1 | tee -a $L
  ~/slam_env/bin/python $S/db_links.py ~/maps/$RUN.db 2>&1 | tail -3 | tee -a $L
  PREV=$RUN
done
[ -e ~/b_slam/exports/fems5 ] || $S/export_run.sh fems5 > /dev/null 2>&1
~/slam_env/bin/python $S/run_metrics.py fems5 2>&1 | grep -E "nodes|odom lost|loop|traj" | tee -a $L
echo "완료 $(date +%T)" | tee -a $L
