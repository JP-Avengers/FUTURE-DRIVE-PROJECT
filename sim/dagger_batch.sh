#!/bin/bash
# DAgger 녹화 일괄 (11.10, D) — 학습 모델이 몰고(eval/xm_route.py → 노드 fd_xm_infer 가 /cmd_vel), 자동 교사는 라벨만(sim/teacher.py --label-only →
# /teacher/cmd_vel · /teacher/status), 녹화기(sim/record_teacher.sh)가 둘 다 남긴다. 순서는 e2e/PLAN.md "DAgger" 표 그대로.
#   FD_CKPT=<ckpt> sim/dagger_batch.sh cw ccw cw ccw cw ccw        (tmux 안에서 — 씬 · 녹화는 각자 tmux 세션)
# 항목 하나 = 방향 하나 = 한 바퀴. 씬은 항목마다 새로 띄움(eval/xm_batch.sh 와 같은 start/stop — 로봇을 출발점에 되돌릴 수단이 없어서).
# 시나리오 = eval/scenarios/parking_r3_lap.yaml(장애물 없음). 모델이 2 m 넘게 벗어나면 하네스가 끝내고(--intervene stop), 그때까지가 녹화됨 —
# 되돌아오는 구간이 DAgger 의 핵심 데이터(PLAN.md).
# 환경변수: FD_CKPT(필수) · FD_NODE_PYTHON(기본 ~/xmobility_test/venv/bin/python — envs/ml_env 엔 gin 없음, 11.10) · FD_TEACHER_DIR(기본 data/teacher/dagger_v1)
#          FD_OUT(하네스 결과, 기본 data/eval/dagger_<월일_시분>) · FD_XM_ARGS(xm_route.py 에 더 넘길 인자)
# 끝 줄: DAGGER_DONE ok=<n> fail=<n> — 묶음별 결과는 DAGGER_ITEM 줄 · 녹화 폴더 meta.yaml(teacher · dagger 칸) · qc.txt.
export CD1=${CD1:-$HOME/Capstone_Design_1}
cd $CD1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FD_DOMAIN=43
[ -f "${FD_CKPT:-}" ] || { echo "FD_CKPT(.ckpt) 가 필요"; exit 2; }
NODE_PY=${FD_NODE_PYTHON:-$HOME/xmobility_test/venv/bin/python}
export FD_TEACHER_DIR=${FD_TEACHER_DIR:-data/teacher/dagger_v1}
D=${FD_OUT:-data/eval/dagger_$(date +%m%d_%H%M)}; mkdir -p $D/runs $D/layers
SCN=eval/scenarios/parking_r3_lap.yaml
SCENE=$(sed -n 's/^scene: *\([^ ]*\).*/\1/p' $SCN)
export FD_SCENE=$SCENE
read -r -a XARGS <<< "${FD_XM_ARGS:-}"
SHA=$(sha256sum "$FD_CKPT" | cut -c1-12)
ITEMS=("$@"); [ ${#ITEMS[@]} -eq 0 ] && ITEMS=(cw ccw cw ccw cw ccw)
echo "DAGGER_START ckpt=$FD_CKPT sha=$SHA scene=$SCENE teacher_dir=$FD_TEACHER_DIR out=$D 항목 ${#ITEMS[@]}"

LAYER=$D/layers/parking_r3_lap.usda
envs/isaacsim_env/bin/python sim/make_obstacles.py $SCN $LAYER > $D/layers/make.log 2>&1
[ -f "$LAYER" ] || { echo "레이어를 못 만듦: $(tail -2 $D/layers/make.log | tr '\n' ' ')"; exit 3; }

start_scene() {   # eval/xm_batch.sh 와 같음
  tmux new-session -d -s fdsim "cd $CD1 && FD_LAYERS=$LAYER FD_LIDAR=1 FD_DOMAIN=43 FD_SCENE=$SCENE sim/run_proxy.sh headless"
  # 준비 = 카메라 영상이 실제로 옴(sim/auto_batch.sh 와 같음). eval/xm_batch.sh 처럼 최신 headless 로그를 보면 직전 씬의 로그가
  # playing 을 이미 담고 있어 새 씬이 뜨기 전에 통과함(11.10 첫 시도: 녹화기가 /clock 을 못 받음)
  for i in $(seq 1 60); do
    timeout 5 ros2 topic echo --once /front_camera/image_raw --no-arr > /dev/null 2>&1 && { sleep 5; return 0; }
    tmux has-session -t fdsim 2>/dev/null || break
    sleep 5
  done
  echo "  씬이 안 뜸"; return 1
}
stop_scene() {
  tmux send-keys -t "=fdsim:" C-c; sleep 12; tmux kill-session -t "=fdsim" 2>/dev/null
  fastdds shm clean > /dev/null 2>&1
}
kill_node() {
  [ -f "$1" ] || return
  NP=$(cat "$1"); kill -INT -$NP 2>/dev/null; sleep 5; kill -TERM -$NP 2>/dev/null; sleep 2; kill -KILL -$NP 2>/dev/null; rm -f "$1"
}
ok=0; fail=0; k=0
for dir in "${ITEMS[@]}"; do
  k=$((k + 1)); name=lap_${dir}_$k; J=$D/runs/$name.json
  echo "DAGGER_ITEM_START $k $dir $(date +%T)"
  start_scene || { stop_scene; fail=$((fail + 1)); continue; }
  # 녹화기 — auto_teacher.sh 와 같은 방식(운전자 칸 = dagger)
  REC=$D/runs/$name.rec.log
  tmux new-session -d -s fdrec_dagger "exec env FD_DOMAIN=43 FD_SCENE=$SCENE FD_TEACHER_DIR=$FD_TEACHER_DIR $CD1/sim/record_teacher.sh dagger $dir dagger_v0.1 > $REC 2>&1"
  for _ in $(seq 1 90); do grep -q "Recording\.\.\." $REC 2>/dev/null && break; sleep 0.5; done
  OUT=$(sed -n 's/^REC_START \([^ ]*\) .*/\1/p' $REC | head -1)
  if ! grep -q "Recording\.\.\." $REC 2>/dev/null; then
    echo "  녹화가 안 뜸 — 건너뜀: $(tail -2 $REC | tr '\n' ' ')"; tmux kill-session -t fdrec_dagger 2>/dev/null; stop_scene; fail=$((fail + 1)); continue
  fi
  sleep 3
  # 교사(라벨만) — /cmd_vel 을 안 내므로 xm_route.py 의 발행자 0 확인과 부딪히지 않음
  /usr/bin/python3 -u sim/teacher.py --route csv:eval/routes/parking_r3_cw.csv --dir $dir --laps 1 --label-only --noise-duty 0 \
    --summary $D/runs/$name.teacher.json > $D/runs/$name.teacher.log 2>&1 &
  TPID=$!
  sleep 3
  timeout -k 20 900 python3 -u eval/xm_route.py --scenario $SCN --direction $dir --ckpt $FD_CKPT --method xmobility_ft \
    --scene $SCENE --node-python "$NODE_PY" --seed $k --out $J --csv $D/results.csv "${XARGS[@]}" > $D/route_$name.log 2>&1
  rc=$?
  grep -E "XM_ROUTE_(DONE|END|START_CHECK|READY)" $D/route_$name.log | sed "s/^/  /" | cut -c1-300
  kill_node $J.node.pid
  kill -INT $TPID 2>/dev/null; sleep 3; kill -TERM $TPID 2>/dev/null; wait $TPID 2>/dev/null
  sleep 2
  tmux send-keys -t fdrec_dagger C-c
  for _ in $(seq 1 600); do tmux has-session -t fdrec_dagger 2>/dev/null || break; sleep 1; done
  tmux kill-session -t fdrec_dagger 2>/dev/null
  if [ -n "$OUT" ] && [ -d "$OUT" ]; then
    mkdir -p $OUT/teacher; cp $D/runs/$name.teacher.log $D/runs/$name.teacher.json $REC $D/route_$name.log $OUT/teacher/ 2>/dev/null
    cp $J $OUT/teacher/xm_route.json 2>/dev/null; cp eval/routes/parking_r3_cw.csv $OUT/teacher/ 2>/dev/null
    { echo "teacher:"; echo "  node: sim/teacher.py --label-only"; echo "  route: \"csv:eval/routes/parking_r3_cw.csv\""
      echo "  direction: $dir"; echo "  laps: 1"; echo "  extra_args: \"--label-only --noise-duty 0\""; echo "  rc: 0"
      echo "  label_topic: /teacher/cmd_vel"; echo "  summary: teacher/$name.teacher.json"
      echo "dagger:"; echo "  driver: eval/xm_route.py (fd_xm_infer)"; echo "  ckpt: $FD_CKPT"; echo "  ckpt_sha12: $SHA"
      echo "  route_rc: $rc"; echo "  result: \"$(sed -n 's/.*XM_ROUTE_DONE result=\([A-Z_]*\).*/\1/p' $D/route_$name.log | head -1)\""; } >> $OUT/meta.yaml
  fi
  QC=$(sed -n 's/^QC_RESULT \([A-Z]*\) .*/\1/p' $OUT/qc.txt 2>/dev/null | head -1)
  stop_scene
  if [ $rc -eq 0 ] && [ "${QC:-}" = OK ]; then ok=$((ok + 1)); else fail=$((fail + 1)); fi
  echo "DAGGER_ITEM $k $dir rc=$rc qc=${QC:-?} rec=$OUT"
done
echo "DAGGER_DONE ok=$ok fail=$fail $(date +%T)"
