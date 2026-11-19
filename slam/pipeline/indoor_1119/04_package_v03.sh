# 실내 복도 맵 v0.3 패키지: v0.2 묶음 + 보조 ceil30(ic30bF) + 메쉬 I_v03c. README 는 docs/deliver/corridor_v0.3_README.md.
set -e
S=~/b_slam/scripts; W=~/b_slam/build_indoor; O=~/b_slam/exports/indoor; DL=~/b_slam/deliver; PY=~/tsdf_env/bin/python
$PY -I $S/in_view.py $W/I_v03.ply in1rL $W/v03; cp $O/ic30b_localize.png $W/
$PY -I $S/build_indoor_pkg.py in2m $W/I_v03c.ply $DL/corridor_v0.3 ~/b_slam/docs/deliver_indoor/README_corridor_v03.md \
  --set in1:in1bL:0.5 --set rightwall:inrwbL:0.5 --set ceil30:ic30bF --val in1rL --crop -0.08 \
  --qa $W/v03_top.png --qa $W/v03_kf.jpg --qa $W/eval_robot_v03.jpg --qa $O/in2_manhattan.png --qa $O/in1b_localize.png --qa $O/inrwb_localize.png --qa $O/in1r_localize.png --qa $W/ic30b_localize.png
# (--note 3줄은 실제 실행 때 넣음: 복도 폭, 벽 꼭대기 · 천장 덮음, 천장 높이 추정)
# C 입력 점검: main 의 recon + extra_inputs 3개(ceil30 · rightwall · in1) → 오류 0 · 경고 1(ceil30 자세 연속성: 키프레임 뺀 자리 6곳)
