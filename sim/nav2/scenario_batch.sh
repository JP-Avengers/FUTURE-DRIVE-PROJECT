#!/bin/bash
# 시나리오 일괄 — eval/scenarios/*.yaml 을 하나씩: 레이어(장애물 · 출발 자세)를 만들고, 씬 + Nav2(AMCL)를 새로 띄워 nav2_route.py --scenario 로 한 번.
#   sim/nav2/scenario_batch.sh eval/scenarios/parking_r3_straight_south.yaml eval/scenarios/parking_r3_obs_cones.yaml:ccw ...
# 바퀴(장애물) 시나리오는 방향을 "yaml:cw" · "yaml:ccw" 로 붙임. 인자가 없으면 구간 시나리오(straight · curve) 전부.
# 결과는 FD_OUT(기본 logs/nav2/scenario)/<이름>[_방향].json · route_*.log · launch_*.log, 끝에 SEG_DONE 한 줄.
# 구간 시나리오는 출발 자세(make_obstacles.py 의 START 줄)를 Nav2 launch 의 x0 · y0 · yaw0_deg(AMCL 초기 자세)로 넘김.
# 준비 실패(Nav2 lifecycle 3개 중 덜 뜸 · nav2_route 코드 3 = 지도 · 정답 위치 · AMCL 확인 실패)나 벽시계 15분 초과면 다시 띄움(최대 3번).
# 씬 · Nav2 는 항목마다 Ctrl+C 로 끄고 Fast DDS 공유 메모리를 정리함(강제로 끄면 다음 노드가 통신을 못 함, 10.13).
# FD_RECORD=1 이면 교사 데이터로 녹화 — Nav2 최종 명령을 /nav2/cmd_vel 로 돌리고 다리(sim/nav2/nav2_teacher.py)가 /cmd_vel · 라벨
# /teacher/cmd_vel · /route_command · /teacher/status 를 냄, 녹화는 sim/record_teacher.sh(운전자 nav2, QC 포함). 저장 FD_TEACHER_DIR
# (기본 data/teacher/nav2_v1)/<시각>_nav2_<방향>/ 에 teacher/(다리 · nav2_route 로그 · 요약 · 시나리오)와 meta.yaml teacher 칸.
# 준비 실패로 다시 띄운 회차의 녹화는 지움. FD_NOISE = 다리 조향 잡음 비율(기본 0 = 끔).
cd ~/Capstone_Design_1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
D=${FD_OUT:-logs/nav2/scenario}; mkdir -p $D
REC=${FD_RECORD:-0}; TDIR=${FD_TEACHER_DIR:-$PWD/data/teacher/nav2_v1}
NAVARG=(); [ "$REC" = 1 ] && NAVARG=(cmd_vel_out:=nav2/cmd_vel)
SEED=0
ITEMS=("$@"); [ ${#ITEMS[@]} -eq 0 ] && ITEMS=(eval/scenarios/parking_r3_straight_*.yaml eval/scenarios/parking_r3_curve_*.yaml)
start_scene() {   # $1 레이어 · $2 로그 이름 · $3 launch 시작 자세 인자
  tmux new-session -d -s fdsim "cd ~/Capstone_Design_1 && FD_LAYERS=$1 FD_LIDAR=1 FD_DOMAIN=43 FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/run_proxy.sh headless"
  sleep 5
  for i in $(seq 1 40); do sleep 3; L=$(ls -t logs/run_proxy/headless_*.log | head -1); grep -q "PROXY_TIMELINE playing" "$L" 2>/dev/null && break; done
  setsid ros2 launch sim/nav2/nav2.launch.py map:=$PWD/data/nav2/parking_v0.4b_r3/map.yaml localization:=amcl $3 "${NAVARG[@]}" > $D/launch_$2.log 2>&1 &
  LP=$!
  for i in $(seq 1 60); do n=$(grep -c "Managed nodes are active" $D/launch_$2.log 2>/dev/null); [ "${n:-0}" -ge 3 ] && break; sleep 1; done
  sleep 3
  # 위치 추정 · 지도 · 주행 lifecycle 셋이 다 떠야 함 — 상태 전환 응답이 통신 지연으로 빠지면 Nav2 활성 대기에서 멈춤(10.14) → 바로 다시 띄움
  [ "${n:-0}" -ge 3 ] || { echo "  Nav2 활성화 안 됨(lifecycle ${n:-0}/3) — 다시 띄움"; return 1; }
}
rec_start() {   # $1 다리가 따를 기준선 · $2 방향 · $3 이름 · $4 시드 — 다리 + 녹화기. 녹화기가 안 뜨면 1
  setsid python3 -u sim/nav2/nav2_teacher.py --route $1 --scenario $f --noise-duty ${FD_NOISE:-0} --seed $4 \
    --summary $D/teacher_$3.json > $D/teacher_$3.log 2>&1 &
  BP=$!
  sleep 3
  tmux new-session -d -s fdrec "cd ~/Capstone_Design_1 && FD_DOMAIN=43 FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda FD_TEACHER_DIR=$TDIR sim/record_teacher.sh nav2 $2 $3 > $D/rec_$3.log 2>&1"
  for i in $(seq 1 90); do grep -q "Recording\.\.\." $D/rec_$3.log 2>/dev/null && break; tmux has-session -t "=fdrec" 2>/dev/null || break; sleep 0.5; done
  RO=$(sed -n 's/^REC_START \([^ ]*\) .*/\1/p' $D/rec_$3.log | head -1)
  grep -q "Recording\.\.\." $D/rec_$3.log 2>/dev/null || { echo "  녹화가 안 뜸: $(tail -2 $D/rec_$3.log | tr '\n' ' ')"; return 1; }
  sleep 3   # 녹화기가 카메라 · odom 을 구독할 시간
}
rec_stop() {   # $1 nav2_route 코드 · $2 이름 · $3 기준선 · $4 방향 — 녹화를 닫고(QC) 다리를 끔. 준비 실패(3 · 124)면 녹화를 지움
  sleep 2
  tmux send-keys -t "=fdrec:" C-c
  for i in $(seq 1 600); do tmux has-session -t "=fdrec" 2>/dev/null || break; sleep 1; done
  kill -TERM $BP 2>/dev/null; wait $BP 2>/dev/null
  [ -n "$RO" ] || return
  if [ $1 -eq 3 ] || [ $1 -eq 124 ]; then rm -rf "$RO"; echo "  녹화 지움(준비 실패)"; return; fi
  mkdir -p $RO/teacher
  cp $D/teacher_$2.log $D/teacher_$2.json $D/route_$2.log $D/$2.json $D/rec_$2.log $f $3 $RO/teacher/ 2>/dev/null
  PASS=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('pass'))" $D/$2.json 2>/dev/null)
  { echo "teacher:"
    echo "  node: sim/nav2/nav2_teacher.py  # Nav2(+ AMCL) 주행 — sim/nav2/nav2_route.py 가 기준선 경유점으로 몲"
    echo "  scenario: $f"
    echo "  route: $3"
    echo "  direction: $4"
    echo "  noise_duty: ${FD_NOISE:-0}"
    echo "  rc: $1"
    echo "  pass: $PASS"
    echo "  label_topic: /teacher/cmd_vel"
    echo "  summary: teacher/teacher_$2.json · teacher/$2.json"
  } >> $RO/meta.yaml
  QC=$(sed -n 's/^QC_RESULT \([A-Z]*\) .*/\1/p' $RO/qc.txt 2>/dev/null | head -1)
  echo "  REC_ITEM $RO qc=${QC:-?} pass=$PASS"
}
stop_scene() {   # lifecycle_manager 가 INT · TERM 을 무시하고 남아 다음 Nav2 와 이름이 겹친 일이 있어(10.14) 마지막에 KILL
  kill -INT -$LP 2>/dev/null; sleep 8; kill -TERM -$LP 2>/dev/null; sleep 2; kill -KILL -$LP 2>/dev/null
  tmux send-keys -t "=fdsim:" C-c; sleep 12; tmux kill-session -t "=fdsim" 2>/dev/null
  fastdds shm clean > /dev/null 2>&1
}
for it in "${ITEMS[@]}"; do
  f=${it%%:*}; dir=""; [ "$f" != "$it" ] && dir=${it##*:}
  n=$(basename $f .yaml); tag=$n${dir:+_$dir}; LAYER=sim/scenes/parking/obstacles/$n.usda
  OUT=$(envs/isaacsim_env/bin/python sim/make_obstacles.py $f $LAYER 2>&1)
  echo "$OUT" | grep -E "START|SAVED" | sed "s/^/  /"
  # START s=16.0 → (-7.083, -20.346, 0.071) 방향 167.0° → x0:=-7.083 y0:=-20.346 yaw0_deg:=167.0 (없으면 launch 기본 = r3 출발점)
  POSE=$(echo "$OUT" | sed -nE 's/^START .*\(([-0-9.]+), ([-0-9.]+), [-0-9.]+\) [^-0-9]*([-0-9.]+).*/x0:=\1 y0:=\2 yaw0_deg:=\3/p')
  ROUTE_ARG=(); [ -n "$dir" ] && ROUTE_ARG=(--route eval/routes/parking_r3_$dir.csv)
  # 다리 · 녹화용 — Nav2 가 따르는 기준선(바퀴 = 방향별 CSV, 구간 = 시나리오의 route)과 방향
  RT=${ROUTE_ARG[1]:-$(sed -n 's/^route: *\([^ ]*\).*/\1/p' $f)}; DIRN=${dir:-$(sed -n 's/^direction: *\([a-z]*\).*/\1/p' $f)}
  SEED=$((SEED + 1))
  for try in 1 2 3; do
    echo "SCENARIO $tag 시도 $try $(date +%T) ${POSE}"
    start_scene $LAYER ${tag}_$try "$POSE" || { stop_scene; continue; }
    RO=; rc=0
    if [ "$REC" = 1 ]; then rec_start $RT $DIRN $tag $SEED || rc=3; fi
    if [ $rc -eq 0 ]; then
      timeout -k 20 900 python3 -u sim/nav2/nav2_route.py "${ROUTE_ARG[@]}" --scenario $f --timeout 300 --out $D/$tag.json > $D/route_$tag.log 2>&1
      rc=$?
    fi
    grep -E "NAV2_ROUTE_(DONE|AMCL|NO_)" $D/route_$tag.log | sed "s/^/  /" | cut -c1-400
    [ $rc -eq 124 ] && echo "  시간 초과(벽시계 15분)"
    [ "$REC" = 1 ] && rec_stop $rc $tag $RT $DIRN
    stop_scene
    [ $rc -ne 3 ] && [ $rc -ne 124 ] && break
  done
done
echo "SEG_DONE $(date +%T)"
