# v0.4 묶음 다듬기 (11/20). 기존 결과는 덮지 않고 새 이름으로 만듦.
#  B) in1b · inrwb 를 천장판 도구(자이로 방향 잇기 · 통로 벌점)로 다시 맞춤 → in1bL2 · inrwbL2.
#     비교(J = v0.3 구성에서 두 묶음만 교체): 검증 depth 차 · 로봇 시점 같음, 통로 안에 떠 있는 면 0.87 → 1.41㎡ → 채택 안 함.
#  A) 보조 묶음 좌우 미세 맞춤(in_lateral_snap.py): 키프레임 벽 점을 주 묶음 벽면(x 1m 구간별)에 연속값으로 재서 y 만 옮김(±8cm).
#     K = in2m + in1bLS + inrwbLS + ic30bFS: 폭 2.598 → 2.581m, 벽 두께 10.1 → 8.7cm, 떠 있는 면 · 검증 그대로 → 채택.
#  4) 벽 45° 두 회차에서 벽 일치 80% 이상 · 복도/로비 안 키프레임만(iw1bF 81장, iw2bF 93장, 사람 없음 확인) → 좌우 미세 맞춤 → iw1bFS · iw2bFS.
S=~/b_slam/scripts; W=~/b_slam/build_indoor; O=~/b_slam/exports/indoor; PY=~/tsdf_env/bin/python; L=~/b_slam/.heavy.lock
for r in in1b inrwb; do [ -e ~/b_slam/exports/${r}L2 ] || $PY -I $S/in_localize_ceil.py $r in2m $O --tag 2; done
flock $L $PY -I $S/in_tsdf.py $W/J_v03_B.ply in2m in1bL2:0.5 inrwbL2:0.5 ic30bF
for spec in inrwbL:inrwbLS:0.5 in1bL:in1bLS:0.5 ic30bF:ic30bFS:0; do IFS=: read r o q <<< "$spec"; flock $L $PY -I $S/in_lateral_snap.py $r $W/A_in2.ply $o --qmin $q; done
flock $L $PY -I $S/in_tsdf.py $W/K_v03_A.ply in2m in1bLS inrwbLS ic30bFS
# iw1bF · iw2bF: exports/<run>L 자세에서 sel_<run>.txt(벽 일치 >= 0.8, 복도 x 0~46 · y -0.85~1.2 또는 로비 x 18~27.5 · y 1.2~8.5) 키프레임만
for r in iw1b iw2b; do flock $L $PY -I $S/in_lateral_snap.py ${r}F $W/A_in2.ply ${r}FS --qmin 0; done
flock $L $PY -I $S/in_tsdf.py $W/M_v04_w.ply in2m in1bLS inrwbLS ic30bFS iw1bFS iw2bFS
for m in J_v03_B K_v03_A M_v04_w; do flock $L $PY -I $S/in_clutter.py $W/$m.ply; flock $L $PY -I $S/in_ceilcov.py $W/$m.ply; done
flock $L $PY -I $S/in_eval.py in1rL $W/I_v03.ply $W/J_v03_B.ply $W/K_v03_A.ply $W/M_v04_w.ply
