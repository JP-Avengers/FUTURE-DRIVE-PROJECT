# 위로 든 회차(재시도 DB)를 주 묶음 맵(in2m)에 키프레임마다 맞춰 넣기 — 천장판.
#  (1) ceil_explore.py: 키프레임마다 바닥 · 천장 평면 + IMU 가속도 위 방향  (2) ceil_height.py: 천장 높이 추정(이웃 바닥 · 천장 짝) → 2.87 m
#  (3) gyro_turn.py: 자이로를 중력 축으로 적분한 누적 요(조각 사이 방향 잇기)  (4) in_localize_ceil.py: 기울기 = 천장 > 바닥 > IMU, 높이 = 천장 높이 - 천장 거리,
#      조각 사이 방향 = 자이로, 위치 = 주 묶음 벽 거리 지도 + 통로 밖 벌점. 결과 exports/<run>L/, exports/indoor/<run>_localize.*
S=~/b_slam/scripts; D=~/data_corr/c0.007; X=~/b_slam/exports/indoor1119; O=~/b_slam/exports/indoor; mkdir -p $X
SPECS="ic30b:30_ceiling_20261118_235837_212621 ic60b:60_ceiling_20261119_000823_919636 ic0b:ceiling_20261118_235234_795968 iw1b:wall_45_20261119_001451_047103 iw2b:wall_45_2_20261119_002141_204316"
for spec in $SPECS; do r=${spec%%:*}; c=${spec#*:}
  [ -e $X/explore_$r.npz ] || ~/slam_env/bin/python -I $S/ceil_explore.py $r $D/$c/bag $X/explore_$r.npz
  [ -e $X/gyro_$r.npz ] || ~/slam_env/bin/python -I $S/gyro_turn.py $D/$c/bag $r 1000 $X/gyro_$r.npz
done
~/tsdf_env/bin/python -I $S/ceil_height.py $X ic0b ic30b ic60b iw1b iw2b inrwb
for spec in $SPECS; do r=${spec%%:*}; [ -e ~/b_slam/exports/${r}L ] || ~/tsdf_env/bin/python -I $S/in_localize_ceil.py $r in2m $O; done
