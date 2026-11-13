# 나머지 회차를 주 묶음 맵(in2m)에 키프레임마다 맞춰 넣기. RTAB-Map 맵이 조각난 회차도 DB 의 오도메트리 자세를 읽어 쓴다.
#  기울기 · 높이 = 키프레임 depth 의 바닥 평면(벽을 정면으로 보면 "벽은 수직" 조건 추가), 회전 = 벽 방향, 위치 = 주 묶음 벽 거리 지도.
# 결과: exports/<run>L/ + exports/indoor/<run>_localize.{json,npz,png}. npz 의 frac = 키프레임별 벽 일치 비율(10 cm 안).
S=~/b_slam/scripts; O=~/b_slam/exports/indoor
for r in ${RUNS:-inrwb inlwb in1b in1r}; do echo "######## $r"; ~/tsdf_env/bin/python -I $S/in_sheet.py $r $O/sheet_$r.jpg 24; ~/tsdf_env/bin/python -I $S/in_localize.py $r in2m $O; done
