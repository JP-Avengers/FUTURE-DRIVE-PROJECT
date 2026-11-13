# 회차 구성 4종을 2 cm TSDF 로 합치고, 맵에 안 쓴 검증 회차(in1rL)와 로봇 시점(높이 0.45 m, 수평)으로 비교.
#  <run>:0.5 = 벽 일치 비율 0.5 이상인 키프레임만 사용. 결과 표는 docs/deliver/corridor_v0.2_README.md.
S=~/b_slam/scripts; W=~/b_slam/build_indoor; PY=~/tsdf_env/bin/python; mkdir -p $W
$PY -I $S/in_tsdf.py $W/A_in2.ply in2m
$PY -I $S/in_tsdf.py $W/B_in2_in1b.ply in2m in1bL:0.5
$PY -I $S/in_tsdf.py $W/C_in2_in1b_rw.ply in2m in1bL:0.5 inrwbL:0.5
$PY -I $S/in_tsdf.py $W/D_in2_in1b_rw_lw.ply in2m in1bL:0.5 inrwbL:0.5 inlwbL:0.5
$PY -I $S/in_eval.py in1rL $W/A_in2.ply $W/B_in2_in1b.ply $W/C_in2_in1b_rw.ply $W/D_in2_in1b_rw_lw.ply --img $W/eval_robot.jpg
$PY -I $S/in_width.py $W/A_in2.ply $W/B_in2_in1b.ply $W/C_in2_in1b_rw.ply     # 복도 폭(구성별 · 5 m 구간별)
$PY -I $S/in_wallh.py $W/A_in2.ply $W/C_in2_in1b_rw.ply                        # 벽 꼭대기 높이
