# 실내 복도 맵 v0.2 패키지: 주 묶음 in2m + 보조 in1(in1bL) · rightwall(inrwbL) + 검증 in1rL + 합친 메쉬(구성 C).
# 이미 있는 폴더는 덮어쓰지 않는다. --note 의 수치는 05 단계(in_width.py, in_wallh.py) 출력에서 옮긴 것.
set -e
S=~/b_slam/scripts; W=~/b_slam/build_indoor; O=~/b_slam/exports/indoor; DL=~/b_slam/deliver; PY=~/tsdf_env/bin/python
$PY -I $S/in_view.py $W/C_in2_in1b_rw.ply in1rL $W/v02
$PY -I $S/build_indoor_pkg.py in2m $W/C_in2_in1b_rw.ply $DL/corridor_v0.2 ~/b_slam/docs/deliver_indoor/README_corridor_v02.md \
  --set in1:in1bL:0.5 --set rightwall:inrwbL:0.5 --val in1rL --crop -0.08 \
  --note "복도 폭은 메쉬 구성에 따라 다름: 주 묶음만 2.54 m, + in1 2.55 m, + rightwall(이 메쉬) 2.60 m. rightwall 묶음의 벽면이 주 묶음 벽면보다 양쪽으로 2~3 cm 바깥에 놓임. 5 m 구간별로는 2.50~2.63 m (in_width.py)" \
  --note "벽 꼭대기 높이(0.5 m 칸마다 98% 값, 그 중앙): y 작은 쪽 벽 1.89 m, y 큰 쪽 벽 1.61 m. 주 묶음만이면 1.25 m / 1.07 m (in_wallh.py)" \
  --qa $W/v02_top.png --qa $W/v02_kf.jpg --qa $W/eval_robot.jpg --qa $O/in2_manhattan.png --qa $O/in1b_localize.png --qa $O/inrwb_localize.png --qa $O/in1r_localize.png
cd $DL/corridor_v0.2 && sha256sum -c --quiet checksums.sha256 && echo "체크섬 OK"
# C 입력 점검: main 의 recon 을 임시 폴더에 풀고 README 의 설정(주 묶음 + extra_inputs 2개 + geometry.mesh)으로 check_input.py 실행 → 오류 0 · 경고 0
# 압축: deliver 폴더에서 corridor_v0.2 를 통째로 zip (python3 shutil.make_archive, base_dir=corridor_v0.2)
