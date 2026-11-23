#!/bin/bash
# 시나리오 일괄 — eval/scenarios/*.yaml 을 하나씩: 레이어(장애물 · 출발 자세)를 만들고, 씬 + Nav2(AMCL)를 새로 띄워 nav2_route.py --scenario 로 한 번.
#   sim/nav2/scenario_batch.sh eval/scenarios/parking_r3_straight_south.yaml eval/scenarios/parking_r3_obs_cones.yaml:ccw ...
# 바퀴(장애물) 시나리오는 방향을 "yaml:cw" · "yaml:ccw" 로 붙임. 인자가 없으면 구간 시나리오(straight · curve) 전부.
# 결과는 FD_OUT(기본 logs/nav2/scenario)/<이름>[_방향].json · route_*.log · launch_*.log · layers/<이름>.usda, 끝에 SEG_DONE 한 줄.
# 장소에 딸린 값(씬 · 지도 · 바퀴의 방향별 기준선 · 바닥 메쉬)은 시나리오 yaml 에서 찾음 — 규칙은 eval/scenario_site.py(11.17).
# FD_DRY=1 이면 씬을 띄우지 않고 항목마다 찾은 씬 · 지도 · 기준선 · 레이어만 DRY 줄로 찍음(새 장소를 돌리기 전 확인용, GPU 안 씀).
# 구간 시나리오는 출발 자세(make_obstacles.py 의 START 줄)를 Nav2 launch 의 x0 · y0 · yaw0_deg(AMCL 초기 자세)로 넘김.
# 준비 실패(Nav2 lifecycle 3개 중 덜 뜸 · nav2_route 코드 3 = 지도 · 정답 위치 · AMCL 확인 실패)나 벽시계 15분 초과면 다시 띄움(최대 3번).
# 씬 · Nav2 는 항목마다 Ctrl+C 로 끄고 Fast DDS 공유 메모리를 정리함(강제로 끄면 다음 노드가 통신을 못 함, 10.13).
# FD_RECORD=1 이면 교사 데이터로 녹화 — Nav2 최종 명령을 /nav2/cmd_vel 로 돌리고 다리(sim/nav2/nav2_teacher.py)가 /cmd_vel · 라벨
# /teacher/cmd_vel · /route_command · /teacher/status 를 냄, 녹화는 sim/record_teacher.sh(운전자 nav2, QC 포함). 저장 FD_TEACHER_DIR
# (기본 data/teacher/nav2_v1)/<시각>_nav2_<방향>/ 에 teacher/(다리 · nav2_route 로그 · 요약 · 시나리오)와 meta.yaml teacher 칸.
# 준비 실패로 다시 띄운 회차의 녹화는 지움. FD_NOISE = 다리 조향 잡음 비율(기본 0 = 끔).
# 작업 폴더 · 도메인 · tmux 세션 이름은 CD1 · FD_DOMAIN · FD_SIM_SESSION · FD_REC_SESSION 으로 바꿀 수 있음(기본값은 예전 그대로).
# 다른 작업 폴더(git worktree)에서 돌릴 때 본 작업의 fdsim · fdrec 세션과 도메인 43 을 건드리지 않게 하려는 것(11.16).
export CD1=${CD1:-$HOME/Capstone_Design_1}; cd $CD1
source /opt/ros/jazzy/setup.bash
DOM=${FD_DOMAIN:-43}; SIMS=${FD_SIM_SESSION:-fdsim}; RECS=${FD_REC_SESSION:-fdrec}
export ROS_DOMAIN_ID=$DOM RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
D=${FD_OUT:-logs/nav2/scenario}; mkdir -p $D/layers
REC=${FD_RECORD:-0}; TDIR=${FD_TEACHER_DIR:-$PWD/data/teacher/nav2_v1}
NAVARG=(); [ "$REC" = 1 ] && NAVARG=(cmd_vel_out:=nav2/cmd_vel)
SEED=0
# 씬 · 지도 — 항목마다 시나리오 yaml 의 scene 과 그 씬의 지도(data/nav2/<씬 파일 이름>/map.yaml)를 씀(11.17 — 예전엔 시나리오와 상관없이
# v0.8 r3 고정이라 다른 장소 · 다른 판은 아래 두 변수를 꼭 줘야 했음). FD_SCENE 을 주면 모든 항목을 그 씬 · 그 씬의 지도 · 그 씬의 바닥 메쉬로,
# FD_NAV2_MAP 을 주면 지도만 그 파일로(위치 추정 지도를 바꿔 볼 때). 예) FD_SCENE=sim/scenes/parking/parking_v0.9.usda
# FD_NAV2_LOC=gt — 정답 위치(AMCL 없음, nav2_route.py 의 AMCL 출발 확인도 건너뜀). 교사 녹화용(평가는 amcl)
export FD_NAV2_LOC=${FD_NAV2_LOC:-amcl}
# FD_NAV2_ARGS — nav2.launch.py 에 더 넘길 인자(예 "scan_min_h:=-0.20 scan_range_max:=15.0", 11.11)
ITEMS=("$@"); [ ${#ITEMS[@]} -eq 0 ] && ITEMS=(eval/scenarios/parking_r3_straight_*.yaml eval/scenarios/parking_r3_curve_*.yaml)
start_scene() {   # $1 레이어 · $2 로그 이름 · $3 launch 시작 자세 인자
  tmux new-session -d -s $SIMS "cd $CD1 && CD1=$CD1 FD_LAYERS=$1 FD_LIDAR=1 FD_DOMAIN=$DOM FD_SCENE=$SCENE sim/run_proxy.sh headless"
  sleep 5
  for i in $(seq 1 40); do sleep 3; L=$(ls -t logs/run_proxy/headless_*.log | head -1); grep -q "PROXY_TIMELINE playing" "$L" 2>/dev/null && break; done
  setsid ros2 launch sim/nav2/nav2.launch.py map:=$NAVMAP localization:=${FD_NAV2_LOC:-amcl} $3 ${FD_NAV2_ARGS:-} "${NAVARG[@]}" > $D/launch_$2.log 2>&1 &
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
  tmux new-session -d -s $RECS "cd $CD1 && CD1=$CD1 FD_DOMAIN=$DOM FD_SCENE=$SCENE FD_TEACHER_DIR=$TDIR sim/record_teacher.sh nav2 $2 $3 > $D/rec_$3.log 2>&1"
  for i in $(seq 1 90); do grep -q "Recording\.\.\." $D/rec_$3.log 2>/dev/null && break; tmux has-session -t "=$RECS" 2>/dev/null || break; sleep 0.5; done
  RO=$(sed -n 's/^REC_START \([^ ]*\) .*/\1/p' $D/rec_$3.log | head -1)
  grep -q "Recording\.\.\." $D/rec_$3.log 2>/dev/null || { echo "  녹화가 안 뜸: $(tail -2 $D/rec_$3.log | tr '\n' ' ')"; return 1; }
  sleep 3   # 녹화기가 카메라 · odom 을 구독할 시간
}
rec_stop() {   # $1 nav2_route 코드 · $2 이름 · $3 기준선 · $4 방향 — 녹화를 닫고(QC) 다리를 끔. 준비 실패(3 · 124)면 녹화를 지움
  sleep 2
  tmux send-keys -t "=$RECS:" C-c
  for i in $(seq 1 600); do tmux has-session -t "=$RECS" 2>/dev/null || break; sleep 1; done
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
  tmux send-keys -t "=$SIMS:" C-c; sleep 12; tmux kill-session -t "=$SIMS" 2>/dev/null
  fastdds shm clean > /dev/null 2>&1
}
for it in "${ITEMS[@]}"; do
  f=${it%%:*}; dir=""; [ "$f" != "$it" ] && dir=${it##*:}
  # 레이어는 결과 폴더($D/layers)에 — 예전 자리 sim/scenes/parking/obstacles/ 는 추적되는 파일이라 돌릴 때마다 작업 트리가 더러워졌고,
  # 다른 장소의 레이어도 주차장 폴더에 쌓였음(11.17, eval/xm_batch.sh 와 같게)
  n=$(basename $f .yaml); tag=$n${dir:+_$dir}; LAYER=$D/layers/$n.usda
  SCENE=${FD_SCENE:-$(python3 eval/scenario_site.py $f scene)}
  NM=$(python3 eval/scenario_site.py $f nav2_map ${FD_SCENE:+--scene $FD_SCENE}); case $NM in /*) ;; *) NM=$PWD/$NM ;; esac
  NAVMAP=${FD_NAV2_MAP:-$NM}
  if [ -z "$SCENE" ] || [ ! -f "$NAVMAP" ]; then
    echo "SCENARIO $n — 씬 · 지도를 못 찾아 건너뜀: scene=${SCENE:-없음} map=$NAVMAP (지도는 sim/nav2/make_map.py 로 만들거나 FD_NAV2_MAP 으로 지정)"; continue
  fi
  OUT=$(envs/isaacsim_env/bin/python sim/make_obstacles.py $f $LAYER ${FD_SCENE:+--scene $FD_SCENE} 2>&1)
  echo "$OUT" | grep -E "START|SAVED" | sed "s/^/  /"
  [ -f "$LAYER" ] || { echo "SCENARIO $n — 레이어를 못 만들어 건너뜀: $(echo "$OUT" | tail -2 | tr '\n' ' ')"; continue; }
  # START s=16.0 → (-7.083, -20.346, 0.071) 방향 167.0° → x0:=-7.083 y0:=-20.346 yaw0_deg:=167.0 (없으면 launch 기본 = r3 출발점)
  POSE=$(echo "$OUT" | sed -nE 's/^START .*\(([-0-9.]+), ([-0-9.]+), [-0-9.]+\) [^-0-9]*([-0-9.]+).*/x0:=\1 y0:=\2 yaw0_deg:=\3/p')
  # 바퀴의 방향별 기준선 — 시나리오 route 의 이름 끝 _cw · _ccw 를 그 방향으로(또는 시나리오의 routes.<방향>). 예전엔 주차장 r3 파일로 고정이라
  # 다른 장소의 한 바퀴 시나리오를 못 돌렸음(11.17)
  ROUTE_ARG=()
  if [ -n "$dir" ]; then
    RDIR=$(python3 eval/scenario_site.py $f route --direction $dir)
    [ -n "$RDIR" ] && [ -f "$RDIR" ] || { echo "SCENARIO $tag — $dir 방향 기준선을 못 찾아 건너뜀: ${RDIR:-없음}"; continue; }
    ROUTE_ARG=(--route $RDIR)
  fi
  # 다리 · 녹화용 — Nav2 가 따르는 기준선(바퀴 = 방향별 CSV, 구간 = 시나리오의 route)과 방향
  RT=${ROUTE_ARG[1]:-$(sed -n 's/^route: *\([^ ]*\).*/\1/p' $f)}; DIRN=${dir:-$(sed -n 's/^direction: *\([a-z]*\).*/\1/p' $f)}
  SEED=$((SEED + 1))
  if [ "${FD_DRY:-0}" = 1 ]; then echo "DRY $tag scene=$SCENE map=$NAVMAP route=${ROUTE_ARG[1]:-$RT} dir=${DIRN:-없음} layer=$LAYER start=[${POSE}]"; continue; fi
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
