# 실내 복도 추가 촬영 5회차(11/18~19, 카메라를 위로 든 촬영): depth 보정(c=0.007) → 단독 매핑(IMU, 정렬 depth) → 내보내기 → 지표 → 위에서 본 그림
# 이어서 11/8 과 같은 순서로, 추적 끊김이 5% 넘는 회차는 Odom/ResetCountdown 1 로 다시 돌림(새 이름 <run>b, 기존 DB 그대로).
# 원본: ~/data_in1119/meta/<촬영>/bag (bag_0.mcap 은 받은 파일로 가는 링크). 원본은 읽기만 함.
source /opt/ros/jazzy/setup.bash
set -u
S=~/b_slam/scripts; SRC=~/data_in1119/meta; D=~/data_corr/c0.007; L=~/b_slam/logs/in1119_chain.log; : > $L
SPECS="ic0:ceiling_20261118_235234_795968 ic30:30_ceiling_20261118_235837_212621 ic60:60_ceiling_20261119_000823_919636 iw1:wall_45_20261119_001451_047103 iw2:wall_45_2_20261119_002141_204316"
lost_pct() { local log=~/b_slam/logs/$1/rtabmap.log; local n=$(grep -c 'Odom: quality' $log); local f=$(grep -c 'quality=0,' $log); python3 -c "print(int(100*$f/max($n,1)))"; }
one() {  # one <run> <bag> [rtabmap args...]
  local RUN=$1 BAG=$2; shift 2
  echo "######## $RUN  $(date +%T)  $*" | tee -a $L
  [ -e ~/maps/$RUN.db ] || env IMU=true BAG_ALIGNED=true $S/run_rtabmap_retry.sh $BAG $RUN 0.5 "$@" 2>&1 | tail -2 | tee -a $L
  ~/slam_env/bin/python $S/db_links.py ~/maps/$RUN.db 2>&1 | tail -2 | cut -c1-300 | tee -a $L
  [ -e ~/b_slam/exports/$RUN ] || $S/export_run.sh $RUN > /dev/null 2>&1
  ~/slam_env/bin/python $S/run_metrics.py $RUN 2>&1 | grep -E "nodes|odom lost|loop|traj" | tee -a $L
  [ -e ~/b_slam/exports/$RUN/traj_optimized_poses.txt ] && ~/tsdf_env/bin/python -I $S/in_top.py ~/b_slam/exports/indoor1119/top_$RUN.png $RUN 2>&1 | grep -v Warn | tail -2 | tee -a $L
  return 0
}
mkdir -p ~/b_slam/exports/indoor1119
for spec in $SPECS; do
  RUN=${spec%%:*}; cap=${spec#*:}
  [ -e $D/$cap/bag ] || { mkdir -p $D/$cap; python3 $S/depth_correct_bag.py $SRC/$cap/bag $D/$cap/bag 0.007 2>&1 | tail -1 | tee -a $L; }
  one $RUN $D/$cap/bag
done
echo "======== 재시도(추적 끊김 5% 넘는 회차, Odom/ResetCountdown 1)  $(date +%T)" | tee -a $L
for spec in $SPECS; do
  RUN=${spec%%:*}; cap=${spec#*:}; p=$(lost_pct $RUN); echo "$RUN 끊김 ${p}%" | tee -a $L
  [ $p -gt 5 ] && one ${RUN}b $D/$cap/bag --Odom/ResetCountdown 1
done
echo "완료 $(date +%T)" | tee -a $L
