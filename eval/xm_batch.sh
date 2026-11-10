#!/bin/bash
# X-Mobility 시나리오 일괄 (E 초안 · D 채택/폐기) — sim/nav2/scenario_batch.sh 와 같은 순서로 항목마다: 레이어(장애물 · 출발 자세)를 만들고
# 씬을 새로 띄워 eval/xm_route.py 한 번(추론 노드 fd_xm_infer 는 xm_route.py 가 띄우고 끔). 판정은 nav2_route.py 와 같은 식.
#   FD_CKPT=data/checkpoints/xm_ft_v0.1_aw30/checkpoints/xm_ft_v0.1_aw30_e8.ckpt eval/xm_batch.sh eval/scenarios/parking_r3_straight_south.yaml eval/scenarios/parking_r3_obs_cones.yaml:ccw ...
#   ckpt 는 best 파일(동결 후보 — 이름 · sha 는 docs/interfaces/e2d_model.md 버전 칸 · MANIFEST). last.ckpt 는 마지막 에폭이라 동결 모델이 아님.
# 바퀴(장애물) 시나리오는 방향을 "yaml:cw" · "yaml:ccw" 로 붙임(scenario_batch.sh:3-4 와 같음). 인자가 없으면 구간 시나리오 전부.
# 환경변수
#   FD_CKPT(필수) · FD_METHOD(xmobility_ft | xmobility_zeroshot) · FD_MODEL_VERSION(비면 ckpt 경로에서 xm_ft_v0.1[_aw30 …]) · FD_SEEDS("1" — 항목마다 시드 목록, 반복 번호)
#   FD_NODE_PYTHON(노드 python, 기본 ~/xmobility_test/venv/bin/python — D 11.10 노드 시험에서 시스템 Jazzy rclpy 와 같이 import 됨. ml_env 엔 gin 없음)
#   FD_ANALYZE_PYTHON(끝의 analyze_runs.py 를 돌릴 python, 기본 노드 python — 시스템 python3 엔 pandas 가 없음) · XM_REPO(노드와 같은 기본 ~/xmobility_test/X-Mobility)
#   FD_OUT(data/eval/xm_<월일_시분>) · FD_CSV($FD_OUT/results.csv) · FD_SCENE(비면 시나리오 yaml 의 scene — scenario_batch.sh 는 r3 고정)
#   FD_LIDAR(1 — scenario_batch.sh:22 와 같게 켬, RTF 를 Nav2 회와 맞추려고. 모델은 LiDAR 를 안 씀) · FD_XM_CCW_START(scene | align — 아래)
#   FD_XM_ARGS(xm_route.py 에 더 넘길 인자, 예 "--intervene off --node-param device:=cuda")
#   FD_DAGGER(0 | 1, 11.10 E) — 1 이면 회마다 씬을 띄운 뒤 라벨 교사(sim/teacher.py --label-only, /cmd_vel 안 냄)와 녹화
#     (sim/auto_teacher.sh → record_teacher.sh)를 먼저 켜고, 모델이 모는 동안 그 자리 교사 라벨을 남김(DAgger, e2e/PLAN.md).
#     xm_route.py 가 끝나면(바퀴 끝 · 2 m 개입 · 시간 초과) 교사에 SIGTERM → auto_teacher 가 녹화를 닫음(AUTO_DONE).
#     녹화 = FD_DAGGER_DIR(기본 data/teacher/dagger_v1)/<묶음>, 메모 = <시나리오>_<방향>_dagger. 내보내기는
#     sim/export_xmobility.py <묶음> <출력> --scenario r3_dagger --split train --exec-cmd(실행 명령 = 모델 명령).
#     장애물 없는 바퀴(eval/scenarios/parking_r3_lap.yaml:cw|ccw)에서만 — 장애물 장면의 자동 교사 라벨은 '장애물로 직진'이라 틀림.
# 반시계 바퀴: 씬은 시계 방향(−96°)을 보고 뜸 — Nav2 회와 같은 조건(scene, 기본). align 이면 로봇 방향을 parking_r3_ccw.yaml 의 start(84°)로
# 덮는 레이어를 하나 더 얹음(Nav2 회와 조건이 달라짐 — XM_HARNESS.md 질문 1).
# 결과: $FD_OUT/runs/<이름>_s<시드>[_a<시도>].json(+ .node.log · .xm_infer.csv · .meta.json) · route_*.log · layers/ · results.csv, 끝에 XM_BATCH_DONE.
# 준비 실패(xm_route 코드 3 = 정답 위치 · /cmd_vel 발행자 · 출발 자세 · 노드 기동 · 시뮬 멈춤)나 벽시계 15분 초과(124)면 다시 띄움(최대 3번,
# scenario_batch.sh:83-97). 준비 실패한 시도는 CSV 행을 안 씀(results_schema.md §6 질문 3).
# 씬은 항목마다 Ctrl+C 로 끄고 Fast DDS 공유 메모리를 정리(scenario_batch.sh:67-71). 남은 노드는 pid 파일로 끔(GPU 가 비어야 run_proxy.sh 가 뜸, :55-57).
export CD1=${CD1:-$HOME/Capstone_Design_1}
cd $CD1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # scenario_batch.sh:15
[ -n "$FD_CKPT" ] || { echo "FD_CKPT(파인튜닝 .ckpt) 가 필요"; exit 2; }
[ -f "$FD_CKPT" ] || { echo "ckpt 없음: $FD_CKPT"; exit 2; }
# 노드 python 사전 확인 — 씬을 띄우기 전에. 모자라면 항목마다 노드 기동 실패(코드 3) → 씬 재시작 ×3 만 하고 결과 행이 없음.
# 기본 = ~/xmobility_test/venv(Python 3.12 · torch 2.5.1+cu124, D 11.10 노드 시험 3/3 통과). ml_env(docs/server207.md:27)엔 gin · pytorch_lightning · transformers 가 없음
NODE_PY=${FD_NODE_PYTHON:-$HOME/xmobility_test/venv/bin/python}
XM_REPO_DIR=${XM_REPO:-$HOME/xmobility_test/X-Mobility}
[ -x "$NODE_PY" ] || { echo "노드 python 없음: $NODE_PY (FD_NODE_PYTHON 으로 지정)"; exit 2; }
[ -f "$XM_REPO_DIR/train.py" ] || { echo "X-Mobility 레포 없음: $XM_REPO_DIR (XM_REPO 로 지정 — 노드 xm_repo 기본값과 같음)"; exit 2; }
PYTHONPATH=$CD1/ros2_ws/src/fd_xm_infer:$XM_REPO_DIR${PYTHONPATH:+:$PYTHONPATH} HF_HUB_OFFLINE=1 "$NODE_PY" - <<'PY' || { echo "노드 python 준비 안 됨($NODE_PY) — 위 import 오류 참고. FD_NODE_PYTHON 으로 X-Mobility requirements 가 있는 python 지정"; exit 2; }
import rclpy, gin, torch, pytorch_lightning, transformers, PIL, yaml   # node.py:34-47 · 120-125 가 import 하는 것
from fd_xm_infer import core, node                                       # ROS 메시지 · tf2_msgs · std_srvs 포함
from model.trainer import XMobilityTrainer                               # X-Mobility 레포(XM_REPO) — 나머지 의존성은 여기서 걸림
print("노드 python 확인:", torch.__version__, "cuda", torch.cuda.is_available())
PY
D=${FD_OUT:-data/eval/xm_$(date +%m%d_%H%M)}; mkdir -p $D/runs $D/layers
CSV=${FD_CSV:-$D/results.csv}
METHOD=${FD_METHOD:-xmobility_ft}; SEEDS=${FD_SEEDS:-1}; LIDAR=${FD_LIDAR:-1}; CCW=${FD_XM_CCW_START:-scene}
read -r -a XARGS <<< "${FD_XM_ARGS:-}"
MV=(); [ -n "$FD_MODEL_VERSION" ] && MV=(--model-version "$FD_MODEL_VERSION")
ITEMS=("$@"); [ ${#ITEMS[@]} -eq 0 ] && ITEMS=(eval/scenarios/parking_r3_straight_*.yaml eval/scenarios/parking_r3_curve_*.yaml)   # scenario_batch.sh:20
REV=$(git rev-parse --short=12 HEAD)$(git diff --quiet HEAD || echo -dirty)
SHA=$(sha256sum "$FD_CKPT" | cut -c1-12)   # e2d_model.md 버전 칸 · MANIFEST 의 sha 와 대조(후보들은 Kaggle 파일 이름이 같아 sha 로만 구분)
echo "XM_BATCH_START rev=$REV method=$METHOD ckpt=$FD_CKPT sha=$SHA node_python=$NODE_PY seeds=[$SEEDS] out=$D csv=$CSV lidar=$LIDAR ccw_start=$CCW 항목 ${#ITEMS[@]}"
case $REV in *-dirty) echo "  주의: 작업 트리가 더러움 — 결과 CSV repo_rev 에 -dirty 가 붙음(eval/README.md: 60회는 커밋 하나로)";; esac
start_scene() {   # $1 레이어(콜론 구분) · $2 씬 — scenario_batch.sh:21-24 에서 Nav2 를 뺀 것. 실패면 1
  tmux new-session -d -s fdsim "cd $CD1 && FD_LAYERS=$1 FD_LIDAR=$LIDAR FD_DOMAIN=43 FD_SCENE=$2 sim/run_proxy.sh headless"
  # 준비 = 카메라 영상이 실제로 옴(sim/auto_batch.sh:28-35 · sim/dagger_batch.sh 와 같음, D 11.10 지적). 예전처럼 최신 headless 로그의
  # "PROXY_TIMELINE playing" 을 보면 직전 씬 로그가 그 줄을 이미 담고 있어 새 씬이 뜨기 전에 통과함(녹화기가 /clock 을 못 받음)
  for i in $(seq 1 60); do
    timeout 5 ros2 topic echo --once /front_camera/image_raw --no-arr > /dev/null 2>&1 && { sleep 5; return 0; }
    tmux has-session -t "=fdsim" 2>/dev/null || break
    sleep 5
  done
  echo "  씬이 안 뜸(카메라 영상 없음)"; return 1
}
kill_node() {   # $1 pid 파일 — xm_route.py 가 timeout 으로 죽어 노드가 남았을 때(노드는 자기 세션 — 그룹째)
  [ -f "$1" ] || return
  NP=$(cat "$1"); kill -INT -$NP 2>/dev/null; sleep 5; kill -TERM -$NP 2>/dev/null; sleep 2; kill -KILL -$NP 2>/dev/null
  rm -f "$1"; echo "  남은 노드 pid $NP 끔"
}
stop_scene() {   # scenario_batch.sh:69-70
  tmux send-keys -t "=fdsim:" C-c; sleep 12; tmux kill-session -t "=fdsim" 2>/dev/null
  fastdds shm clean > /dev/null 2>&1
}
for it in "${ITEMS[@]}"; do
  f=${it%%:*}; dir=""; [ "$f" != "$it" ] && dir=${it##*:}
  n=$(basename $f .yaml); tag=$n${dir:+_$dir}
  # 레이어는 레포 밖($D/layers)에 — sim/scenes/parking/obstacles/ 에 다시 쓰면 작업 트리가 더러워짐(MANIFEST.md:40 "19개 -dirty 는 … start_json 재생성뿐")
  LAYER=$D/layers/$n.usda
  OUT=$(envs/isaacsim_env/bin/python sim/make_obstacles.py $f $LAYER 2>&1)                   # scenario_batch.sh:75
  echo "$OUT" | grep -E "START|SAVED" | sed "s/^/  /"
  [ -f "$LAYER" ] || { echo "  레이어를 못 만듦 — 건너뜀: $(echo "$OUT" | tail -2 | tr '\n' ' ')"; continue; }
  SCENE=${FD_SCENE:-$(sed -n 's/^scene: *\([^ ]*\).*/\1/p' $f)}
  LAYERS=$LAYER; ARGS=(--scenario $f --method $METHOD --ckpt $FD_CKPT --csv $CSV --scene $SCENE --node-python "$NODE_PY" "${MV[@]}")
  [ -n "$dir" ] && ARGS+=(--direction $dir)
  # START s=16.0 → (-7.083, -20.346, 0.071) 방향 167.0° — 구간이면 레이어가 놓은 출발 자세를 그대로 노드에(scenario_batch.sh:77-78 과 같은 읽기)
  POSE=$(echo "$OUT" | sed -nE 's/^START .*\(([-0-9.]+), ([-0-9.]+), [-0-9.]+\) [^-0-9]*([-0-9.]+).*/\1 \2 \3/p')
  [ -n "$POSE" ] && ARGS+=(--start-pose $POSE)
  if [ "$CCW" = align ] && [ "$dir" = ccw ] && [ -z "$POSE" ]; then
    YAW=$(python3 -c "import sys, yaml; print(float(yaml.safe_load(open(sys.argv[1]))['start']['yaw_deg']))" eval/routes/parking_r3_ccw.yaml)
    ALIGN=$D/layers/${n}_ccw_align.usda
    printf '#usda 1.0\n(\n    doc = "eval/xm_batch.sh FD_XM_CCW_START=align — 반시계 바퀴 출발 방향(eval/routes/parking_r3_ccw.yaml start)"\n)\n\nover "World"\n{\n    over "Nova_Carter_ROS"\n    {\n        float xformOp:rotateZ = %s\n    }\n}\n' "$YAW" > $ALIGN
    LAYERS=$LAYER:$ALIGN; ARGS+=(--start-yaw-deg $YAW)
    echo "  반시계 출발 방향 $YAW° 레이어 $ALIGN"
  fi
  for seed in $SEEDS; do
    for try in 1 2 3; do
      name=${tag}_s$seed$([ $try -gt 1 ] && echo _a$try); J=$D/runs/$name.json
      echo "SCENARIO $tag seed $seed 시도 $try $(date +%T) ${POSE}"
      if ! start_scene $LAYERS $SCENE; then stop_scene; continue; fi   # 준비 실패와 같게 다시 띄움(CSV 행 없음)
      if [ "${FD_DAGGER:-0}" = 1 ]; then   # 라벨 교사 + 녹화를 모델보다 먼저(auto_teacher 는 /cmd_vel 발행자가 있으면 거부)
        ROUTE_CSV=$(sed -n 's/^route: *\([^ #]*\).*/\1/p' $f)
        tmux new-session -d -s fddag "cd $CD1 && FD_DOMAIN=43 FD_SCENE=$SCENE FD_TEACHER_DIR=${FD_DAGGER_DIR:-data/teacher/dagger_v1} sim/auto_teacher.sh csv:$ROUTE_CSV ${dir:-cw} 1 ${tag}_dagger -- --label-only > $D/dagger_$name.log 2>&1"
        for i in $(seq 1 90); do grep -q "^AUTO_REC" $D/dagger_$name.log 2>/dev/null && break; sleep 1; done
        grep -q "^AUTO_REC" $D/dagger_$name.log 2>/dev/null && echo "  DAgger $(grep '^AUTO_REC' $D/dagger_$name.log)"           || echo "  DAgger 녹화가 안 뜸 — $D/dagger_$name.log 확인(모델 회는 그대로 돌림)"
      fi
      timeout -k 20 900 python3 -u eval/xm_route.py "${ARGS[@]}" --seed $seed --attempt $try --out $J "${XARGS[@]}" > $D/route_$name.log 2>&1   # scenario_batch.sh:89
      rc=$?
      if [ "${FD_DAGGER:-0}" = 1 ]; then   # 교사 끄기(SIGTERM → 요약 · TEACHER_EXIT) → auto_teacher 가 녹화 닫고 QC(긴 녹화는 몇 분)
        pkill -TERM -f "sim/teacher.py" 2>/dev/null
        for i in $(seq 1 600); do grep -q "^AUTO_DONE" $D/dagger_$name.log 2>/dev/null && break; tmux has-session -t "=fddag" 2>/dev/null || break; sleep 1; done
        grep -E "^(AUTO_DONE|TEACHER_EXIT)" $D/dagger_$name.log | sed "s/^/  DAgger /"
        tmux kill-session -t "=fddag" 2>/dev/null
      fi
      grep -E "XM_ROUTE_(DONE|END|START_CHECK|READY|NO_|CMD_BUSY|SIM_STALL|CSV_FAIL|WARN)" $D/route_$name.log | sed "s/^/  /" | cut -c1-400
      [ $rc -eq 124 ] && echo "  시간 초과(벽시계 15분)"
      kill_node $J.node.pid
      stop_scene
      [ $rc -ne 3 ] && [ $rc -ne 124 ] && break
    done
  done
done
echo "XM_BATCH_DONE $(date +%T) csv=$CSV 행 $(($(wc -l < $CSV 2>/dev/null || echo 1) - 1))"
# 스키마 검사(eval/README.md "60회 결과 분석") — 오류면 ANALYZE_RUNS_FAIL. pandas 가 있는 python 으로(시스템 python3 엔 없음 — D 11.10:
# 예전엔 import 오류를 grep 이 걸러 아무것도 안 찍혔음). 결과 줄이 없으면 끝 3줄을 그대로 보임
if [ -f "$CSV" ]; then
  APY=${FD_ANALYZE_PYTHON:-$NODE_PY}
  AOUT=$("$APY" -I eval/analyze_runs.py $CSV --out $D/analysis --no-plots 2>&1); arc=$?
  echo "$AOUT" | grep -E "^(경고|오류|ANALYZE_RUNS)" | sed "s/^/  /"
  echo "$AOUT" | grep -qE "^ANALYZE_RUNS" || { echo "  ANALYZE_RUNS 줄 없음(코드 $arc, $APY) — 끝 3줄:"; echo "$AOUT" | tail -3 | sed "s/^/    /"; }
fi
