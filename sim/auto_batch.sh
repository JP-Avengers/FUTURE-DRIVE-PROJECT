#!/usr/bin/env bash
# 자동 교사 무인 수집 — 헤드리스 씬을 띄우고, 작업 파일의 줄마다 sim/auto_teacher.sh 를 차례로 돌린 뒤 씬을 끈다(밤새 수집용).
#
#   FD_SCENE=<씬 루트> sim/auto_batch.sh <작업 파일>        (tmux 안에서 — 씬 · 녹화도 각자 tmux 세션으로 뜬다)
#
# 작업 파일 한 줄 = auto_teacher.sh 인자 그대로. 빈 줄 · # 주석은 건너뜀. 씬이 다른 작업은 파일을 나눈다(씬은 한 번만 띄움).
#   proxy cw 3 잡음30 -- --seed 1
#   proxy ccw 3 잡음30 -- --seed 2 --v-range 0.4,0.7
# 바퀴 끝마다 출발점에 멈추므로 작업을 이어도 같은 자리에서 시작한다. 한 작업이 실패하면(이탈 · 정체) 로봇이 경로 밖에 있을 수 있어
# 남은 작업은 씬을 다시 띄워 이어 간다(FD_BATCH_RESTART=0 이면 멈춤). 씬이 죽으면(/clock 없음) 다시 띄운다.
# 환경변수: FD_DOMAIN(기본 43 — 스트리밍 세션(42)과 섞이지 않게) · FD_SCENE · FD_TEACHER_DIR · FD_REC_SPLIT · FD_BATCH_RESTART
# 끝 줄: BATCH_DONE ok=<n> fail=<n> — 묶음별 결과는 BATCH_JOB 줄과 각 녹화 폴더의 qc.txt · meta.yaml.
set -uo pipefail
CD1=${CD1:-$HOME/Capstone_Design_1}
JOBS=${1:-}
[ -f "$JOBS" ] || { sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
export FD_DOMAIN=${FD_DOMAIN:-43}
SIM=fdsim_batch_$$
LOG=$CD1/logs/auto_batch/$(date +%m%d_%H%M%S)
mkdir -p "$LOG"
cp "$JOBS" "$LOG/jobs.txt"
set +u; source /opt/ros/jazzy/setup.bash; set -u
export ROS_DOMAIN_ID=$FD_DOMAIN RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

start_scene() {
  local k=$1
  printf -v cmd 'exec env FD_DOMAIN=%q FD_SCENE=%q %q headless 0 > %q 2>&1' "$FD_DOMAIN" "${FD_SCENE:-}" "$CD1/sim/run_proxy.sh" "$LOG/sim_$k.log"
  tmux new-session -d -s "$SIM" "$cmd"
  for _ in $(seq 1 60); do
    if timeout 5 ros2 topic echo --once /front_camera/image_raw --no-arr >/dev/null 2>&1; then echo "BATCH_SCENE_UP $k"; sleep 5; return 0; fi
    tmux has-session -t "$SIM" 2>/dev/null || break
    sleep 5
  done
  echo "BATCH_SCENE_FAIL $k — $LOG/sim_$k.log"; return 1
}
stop_scene() {
  tmux has-session -t "$SIM" 2>/dev/null || return 0
  tmux send-keys -t "$SIM" C-c
  for _ in $(seq 1 60); do tmux has-session -t "$SIM" 2>/dev/null || return 0; sleep 2; done
  tmux kill-session -t "$SIM" 2>/dev/null || true
}
scene_alive() {   # ros2 CLI 발견이 부하로 늦어 한 번 실패해도 씬은 살아 있을 수 있음(10.12 시험) — 세션 + /clock 3번까지
  tmux has-session -t "$SIM" 2>/dev/null || return 1
  for _ in 1 2 3; do timeout 15 ros2 topic echo --once /clock < /dev/null > /dev/null 2>&1 && return 0; sleep 2; done
  return 1
}
trap 'stop_scene; echo "BATCH_ABORT"; exit 130' INT TERM

echo "BATCH_START jobs=$JOBS scene=${FD_SCENE:-proxy} domain=$FD_DOMAIN log=$LOG"
k=0; ok=0; fail=0; restart=0
start_scene $k || exit 3
while IFS= read -r line <&3 || [ -n "$line" ]; do   # 작업 파일은 fd 3 — 안쪽 명령이 표준입력을 먹지 않게
  line=${line%%#*}
  [ -z "${line//[[:space:]]/}" ] && continue
  if [ $restart = 1 ] || ! scene_alive; then
    [ $restart = 1 ] || echo "BATCH_SCENE_LOST — 다시 띄움"
    stop_scene; k=$((k + 1)); start_scene $k || exit 3; restart=0
  fi
  eval "set -- $line"
  "$CD1/sim/auto_teacher.sh" "$@" < /dev/null > "$LOG/job_$((ok + fail)).log" 2>&1
  rc=$?
  res=$(tail -1 "$LOG/job_$((ok + fail)).log")
  echo "BATCH_JOB $((ok + fail)) rc=$rc :: $line :: $res"
  if [ $rc = 0 ]; then ok=$((ok + 1)); else
    fail=$((fail + 1))
    if [ "${FD_BATCH_RESTART:-1}" = 1 ]; then restart=1; else break; fi   # 다음 작업 앞에서 씬을 다시 띄움
  fi
done 3< "$JOBS"
stop_scene
echo "BATCH_DONE ok=$ok fail=$fail log=$LOG"
