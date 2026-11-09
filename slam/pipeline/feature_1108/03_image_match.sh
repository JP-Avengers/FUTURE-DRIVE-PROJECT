# 손 촬영 지도 ↔ 기준 지도 사진 매칭 (xmap_match2.py: 특징 매칭 + depth PnP + 중력 검사)
# (a) 단독 6회차 → gn04base(v0.5 좌표 기준)
L=~/b_slam/logs/fe_xmatch_20261109.log; : > $L
for r in fe422b fecen feit1w fe422a feit1 feone; do
  echo "######## $r  $(date +%T)" | tee -a $L
  ~/slam_env/bin/python ~/b_slam/scripts/xmap_match2.py gn04base $r ~/b_slam/exports/xmatch_${r}_gn04 2>&1 | grep -v Warning | tail -8 | tee -a $L
done
echo "완료 $(date +%T)" | tee -a $L

# (b) 사진 매칭 실패 회차 → feone(같은 날 둘레 한 바퀴)
L=~/b_slam/logs/fe_xmatch_20261109.log
for r in fe422a fe422b feit1w; do
  echo "######## $r → feone  $(date +%T)" | tee -a $L
  ~/slam_env/bin/python ~/b_slam/scripts/xmap_match2.py feone $r ~/b_slam/exports/xmatch_${r}_feone 2>&1 | grep -v Warning | tail -4 | tee -a $L
done
echo "완료 $(date +%T)" | tee -a $L

# (c) 묶음 맵 fems5 → v0.5 기준 4개
E=~/b_slam/exports; L=~/b_slam/logs/fems_xmatch_20261109.log; : > $L
for p in run18b6:run18 run20b6:run20 gm05b:gm05; do r=${p%%:*}; o=${p#*:}; [ -e $E/$r/keyframes ] || ln -s $E/$o/keyframes $E/$r/keyframes; done
S=$E/fems5s3; mkdir -p $S; [ -e $S/keyframes ] || ln -s $E/fems5/keyframes $S/keyframes
awk 'NR==1 || /^#/ {print; next} (++n % 3) == 1' $E/fems5/traj_camera_id_camera_poses.txt > $S/traj_camera_id_camera_poses.txt
cp $E/fems5/traj_optimized_poses.txt $S/
echo "src 키프레임 $(grep -vc '^#' $S/traj_camera_id_camera_poses.txt)장" | tee -a $L
for ref in run18b6 run20b6 gn04base gm05b; do
  echo "######## fems5s3 → $ref  $(date +%T)" | tee -a $L
  ~/slam_env/bin/python ~/b_slam/scripts/xmap_match2.py $ref fems5s3 $E/xmatch_fems5_$ref 2>&1 | grep -v Warning | grep -vE "^\s+채택" | tail -4 | cut -c1-300 | tee -a $L
done
echo "완료 $(date +%T)" | tee -a $L
