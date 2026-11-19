# 구성 비교: v0.2(C) / + ceil30(E) / + 벽 45° 두 회차(H). 천장 덮음 · 벽 꼭대기(in_ceilcov), 통로 안에 떠 있는 면(in_clutter), 검증 회차 · 로봇 시점(in_eval).
# 채택: E. 30_ceiling 에서 사람 얼굴 · 유리문 반사 · 노트북 화면 키프레임(id 213~217, 258~264)과 벽 일치 50% 미만을 뺀 묶음 ic30bF 로 v0.3 메쉬(I)를 만듦.
S=~/b_slam/scripts; W=~/b_slam/build_indoor; PY=~/tsdf_env/bin/python
$PY -I $S/in_tsdf.py $W/E_C_ic30.ply in2m in1bL:0.5 inrwbL:0.5 ic30bL:0.5
$PY -I $S/in_tsdf.py $W/H_C_ic30_iw.ply in2m in1bL:0.5 inrwbL:0.5 ic30bL:0.5 iw1bL:0.5 iw2bL:0.5
$PY -I $S/in_ceilcov.py $W/C_in2_in1b_rw.ply $W/E_C_ic30.ply $W/H_C_ic30_iw.ply
$PY -I $S/in_clutter.py $W/C_in2_in1b_rw.ply $W/E_C_ic30.ply $W/H_C_ic30_iw.ply
$PY -I $S/in_eval.py in1rL $W/C_in2_in1b_rw.ply $W/E_C_ic30.ply $W/H_C_ic30_iw.ply --img $W/eval_robot_EH.jpg
# ic30bF: exports/ic30bL 의 자세에서 frac >= 0.5 이고 위 id 가 아닌 키프레임만 (keyframes 는 exports/ic30b 링크)
$PY -I $S/in_tsdf.py $W/I_v03.ply in2m in1bL:0.5 inrwbL:0.5 ic30bF
$PY -I $S/mesh_croptop.py $W/I_v03.ply $W/I_v03c.ply        # 복도 구간 3.27 m 위(조명 depth 튐) 잘라냄
