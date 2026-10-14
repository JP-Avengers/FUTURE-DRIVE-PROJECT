#!/bin/bash
# 시나리오 일괄 — eval/scenarios/*.yaml 을 하나씩: 레이어(장애물 · 출발 자세)를 만들고, 씬 + Nav2(AMCL)를 새로 띄워 nav2_route.py --scenario 로 한 번.
#   sim/nav2/scenario_batch.sh eval/scenarios/parking_r3_straight_south.yaml eval/scenarios/parking_r3_obs_cones.yaml:ccw ...
# 바퀴(장애물) 시나리오는 방향을 "yaml:cw" · "yaml:ccw" 로 붙임. 인자가 없으면 구간 시나리오(straight · curve) 전부.
# 결과는 FD_OUT(기본 logs/nav2/scenario)/<이름>[_방향].json · route_*.log · launch_*.log, 끝에 SEG_DONE 한 줄.
# 구간 시나리오는 출발 자세(make_obstacles.py 의 START 줄)를 Nav2 launch 의 x0 · y0 · yaw0_deg(AMCL 초기 자세)로 넘김.
# 준비 실패(nav2_route 코드 3 = 지도 · 정답 위치 · AMCL 확인 실패)나 벽시계 15분 초과면 다시 띄워 한 번 더.
# 씬 · Nav2 는 항목마다 Ctrl+C 로 끄고 Fast DDS 공유 메모리를 정리함(강제로 끄면 다음 노드가 통신을 못 함, 10.13).
cd ~/Capstone_Design_1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
D=${FD_OUT:-logs/nav2/scenario}; mkdir -p $D
ITEMS=("$@"); [ ${#ITEMS[@]} -eq 0 ] && ITEMS=(eval/scenarios/parking_r3_straight_*.yaml eval/scenarios/parking_r3_curve_*.yaml)
start_scene() {   # $1 레이어 · $2 로그 이름 · $3 launch 시작 자세 인자
  tmux new-session -d -s fdsim "cd ~/Capstone_Design_1 && FD_LAYERS=$1 FD_LIDAR=1 FD_DOMAIN=43 FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/run_proxy.sh headless"
  sleep 5
  for i in $(seq 1 40); do sleep 3; L=$(ls -t logs/run_proxy/headless_*.log | head -1); grep -q "PROXY_TIMELINE playing" "$L" 2>/dev/null && break; done
  setsid ros2 launch sim/nav2/nav2.launch.py map:=$PWD/data/nav2/parking_v0.4b_r3/map.yaml localization:=amcl $3 > $D/launch_$2.log 2>&1 &
  LP=$!
  for i in $(seq 1 60); do [ $(grep -c "Managed nodes are active" $D/launch_$2.log) -ge 3 ] && break; sleep 1; done
  sleep 3
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
  for try in 1 2; do
    echo "SCENARIO $tag 시도 $try $(date +%T) ${POSE}"
    start_scene $LAYER ${tag}_$try "$POSE"
    timeout -k 20 900 python3 -u sim/nav2/nav2_route.py "${ROUTE_ARG[@]}" --scenario $f --timeout 300 --out $D/$tag.json > $D/route_$tag.log 2>&1
    rc=$?
    grep -E "NAV2_ROUTE_(DONE|AMCL|NO_)" $D/route_$tag.log | sed "s/^/  /" | cut -c1-400
    [ $rc -eq 124 ] && echo "  시간 초과(벽시계 15분)"
    stop_scene
    [ $rc -ne 3 ] && [ $rc -ne 124 ] && break
  done
done
echo "SEG_DONE $(date +%T)"
