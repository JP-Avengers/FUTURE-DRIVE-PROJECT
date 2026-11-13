# 주 묶음(in2 왕복) 보정: 바닥 수평 → 벽 방향 나침반으로 회전 오차 보정 → 건물 좌표(복도 축 x) → 복도 벽 직선 두 줄에 좌우 맞춤.
# 결과: exports/in2m/ (건물 좌표 자세) + exports/indoor/in2_manhattan.{json,npz,png}. in1r 은 복도 폭을 따로 확인하는 용도.
S=~/b_slam/scripts; O=~/b_slam/exports/indoor; mkdir -p $O
for r in ${RUNS:-in2 in1r}; do ~/tsdf_env/bin/python -I $S/in_manhattan.py $r $O --snap; done
