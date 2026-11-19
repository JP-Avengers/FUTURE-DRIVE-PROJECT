# 실내 복도 맵 v0.4 패키지 (11/20). 05 단계의 묶음(in1bLS · inrwbLS · ic30bFS · iw1bFS2 · iw2bFS2) + 메쉬 M2.
#  메쉬: in_tsdf(M2) → mesh_croptop(복도 3.27m 위) → mesh_declutter --near 0.05 → build_indoor_pkg(바닥 -0.08m 아래 자름)
#  README 는 docs/deliver/corridor_v0.4_README.md, qa/compare_table.md 는 scripts/map_compare 의 map_compare.py 결과.
set -e
S=~/b_slam/scripts; W=~/b_slam/build_indoor; O=~/b_slam/exports/indoor; DL=~/b_slam/deliver; PY=~/tsdf_env/bin/python; L=~/b_slam/.heavy.lock
flock $L $PY -I $S/in_tsdf.py $W/M2_v04_w.ply in2m in1bLS inrwbLS ic30bFS iw1bFS2 iw2bFS2
flock $L $PY -I $S/mesh_croptop.py $W/M2_v04_w.ply $W/M2_v04_w_c.ply
flock $L $PY -I $S/mesh_declutter.py $W/M2_v04_w_c.ply $W/M2_v04_w_dc5.ply --near 0.05
flock $L $PY -I $S/in_view.py $W/M2_v04_w_dc5.ply in1rL $W/v04
flock $L $PY -I $S/build_indoor_pkg.py in2m $W/M2_v04_w_dc5.ply $DL/corridor_v0.4 ~/b_slam/docs/deliver_indoor/README_corridor_v04.md \
  --set in1:in1bLS --set rightwall:inrwbLS --set ceil30:ic30bFS --set wall45a:iw1bFS2 --set wall45b:iw2bFS2 --val in1rL --crop -0.08 \
  --qa $W/v04_top.png --qa $W/v04_kf.jpg --qa $W/eval_robot_v04.jpg --qa $O/in2_manhattan.png --qa $O/in1b_localize.png --qa $O/inrwb_localize.png --qa $O/in1r_localize.png --qa $W/ic30b_localize.png --qa $O/iw1b_localize.png --qa $O/iw2b_localize.png
# (--note 3줄은 실제 실행 때 넣음: 복도 폭, 벽 위쪽 덮음, 천장 높이 추정)
# 비교 표: bash $S/map_compare/map_compare.sh 또는 map_compare.py <out> v0.2=... v0.3=... v0.4=...  → qa/compare_table.md · rigidval_bins.png 로 복사 후 체크섬 다시
# C 입력 점검(main, extra_inputs 5개): 오류 0 · 경고 3(ceil30 · wall45a · wall45b 자세 연속성 — 모두 키프레임 뺀 자리)
