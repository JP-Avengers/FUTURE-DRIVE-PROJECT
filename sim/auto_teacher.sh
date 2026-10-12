#!/usr/bin/env bash
# 자동 교사 녹화 한 묶음 — 씬(sim/run_proxy.sh stream|headless)이 Play 중일 때 녹화기(record_teacher.sh)와 자동 교사(teacher.py)를 함께 돌린다.
#
#   sim/auto_teacher.sh <경로> <cw|ccw> <바퀴> [메모] [-- teacher.py 추가 인자]
#   예) sim/auto_teacher.sh proxy cw 3 잡음30 -- --seed 1
#       FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/auto_teacher.sh csv:data/usd/parking_v0.4b_r3/route_bonnet_cw.csv ccw 2
#
# 녹화는 tmux 세션에서 돌고, 교사가 끝나면 Ctrl+C 로 닫는다(record_teacher.sh 가 qc_teacher.py 를 돌림). 녹화 폴더에 교사 로그 · 요약
# (teacher/)을 넣고 meta.yaml 에 teacher 칸을 붙인다. 저장 위치 = record_teacher.sh 와 같음(FD_TEACHER_DIR, 기본 data/teacher/raw_v1).
# 환경변수: FD_DOMAIN(씬과 같게, 기본 42) · FD_SCENE(씬 루트 — 시작 자세 · meta) · FD_TEACHER_DIR · FD_REC_SPLIT
# 끝 줄: AUTO_DONE <폴더> teacher_rc=<n> qc=<OK|NG|?> — 교사가 실패해도 녹화는 닫고 남긴다(meta 의 teacher.rc 로 거름).
set -euo pipefail
CD1=${CD1:-$HOME/Capstone_Design_1}
POS=()
while [ $# -gt 0 ] && [ "$1" != "--" ]; do POS+=("$1"); shift; done
[ "${1:-}" = "--" ] && shift
EXTRA=("$@")
ROUTE=${POS[0]:-}; DIR=${POS[1]:-}; LAPS=${POS[2]:-}; NOTE=${POS[3]:-auto}
if [ -z "$ROUTE" ] || { [ "$DIR" != cw ] && [ "$DIR" != ccw ]; } || ! [[ "$LAPS" =~ ^[1-9][0-9]*$ ]]; then
  sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=${FD_DOMAIN:-42} RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FD_DOMAIN=$ROS_DOMAIN_ID

clock_ok=0
for _ in 1 2 3; do timeout 15 ros2 topic echo --once /clock >/dev/null 2>&1 && { clock_ok=1; break; }; sleep 2; done
if [ $clock_ok = 0 ]; then echo "/clock 이 안 옴 — 씬이 Play 중인지, 도메인($ROS_DOMAIN_ID)이 씬과 같은지 확인"; exit 3; fi
# 씬이 막 떴을 때는 /cmd_vel 이 아직 안 보여 info 가 실패할 수 있음(10.12 첫 시험에서 set -e 로 조용히 끝남) — 실패는 발행자 0 으로 본다
pub=$(ros2 topic info /cmd_vel --no-daemon --spin-time 3 2>/dev/null | sed -n 's/^Publisher count: *//p' || true)
if [ "${pub:-0}" != 0 ]; then echo "/cmd_vel 발행자가 이미 ${pub}개 — teleop · pursuit 를 먼저 끄세요(발행자 1개 규칙)"; exit 3; fi

TMP=$(mktemp -d "${TMPDIR:-/tmp}/fd_auto.XXXXXX")
SES=fdrec_auto_$$
printf -v REC_CMD 'exec env FD_DOMAIN=%q FD_SCENE=%q FD_TEACHER_DIR=%q FD_REC_SPLIT=%q %q auto %q %q > %q 2>&1' \
  "$ROS_DOMAIN_ID" "${FD_SCENE:-}" "${FD_TEACHER_DIR:-}" "${FD_REC_SPLIT:-}" "$CD1/sim/record_teacher.sh" "$DIR" "$NOTE" "$TMP/rec.log"
OUT=
for try in 1 2; do   # 녹화기 점검(ros2 CLI 발견)이 부하에 따라 늦을 수 있어 한 번 더
  tmux new-session -d -s "$SES" "$REC_CMD"
  for _ in $(seq 1 90); do
    grep -q "Recording\.\.\." "$TMP/rec.log" 2>/dev/null && break
    tmux has-session -t "$SES" 2>/dev/null || break
    sleep 0.5
  done
  OUT=$(sed -n 's/^REC_START \([^ ]*\) .*/\1/p' "$TMP/rec.log" 2>/dev/null | head -1)
  grep -q "Recording\.\.\." "$TMP/rec.log" 2>/dev/null && break
  echo "녹화가 안 뜸(시도 $try): $(tail -3 "$TMP/rec.log" 2>/dev/null | tr '\n' ' ')"
  tmux kill-session -t "$SES" 2>/dev/null || true
  [ -n "$OUT" ] && rm -rf "$OUT"
  OUT=
  sleep 3
done
[ -n "$OUT" ] || exit 4
sleep 3   # 녹화기가 카메라 · odom 을 구독할 시간
echo "AUTO_REC $OUT"

set +e
/usr/bin/python3 "$CD1/sim/teacher.py" --route "$ROUTE" --dir "$DIR" --laps "$LAPS" --summary "$TMP/summary.json" "${EXTRA[@]}" \
  2>&1 | tee "$TMP/teacher.log"
TRC=${PIPESTATUS[0]}
set -e
sleep 2
tmux send-keys -t "$SES" C-c
for _ in $(seq 1 600); do tmux has-session -t "$SES" 2>/dev/null || break; sleep 1; done   # 긴 녹화는 QC 에 몇 분
tmux kill-session -t "$SES" 2>/dev/null || true

mkdir -p "$OUT/teacher"
cp "$TMP/teacher.log" "$TMP/rec.log" "$OUT/teacher/" 2>/dev/null || true
[ -f "$TMP/summary.json" ] && cp "$TMP/summary.json" "$OUT/teacher/"
case "$ROUTE" in csv:*) f=${ROUTE#csv:}; f=${f%@*}; [ -f "$CD1/$f" ] && f="$CD1/$f"; cp "$f" "$OUT/teacher/" 2>/dev/null || true ;; esac
if [ -f "$OUT/meta.yaml" ]; then
  {
    echo "teacher:"
    echo "  node: sim/teacher.py"
    echo "  route: \"$ROUTE\""
    echo "  direction: $DIR"
    echo "  laps: $LAPS"
    printf '  extra_args: "%s"\n' "${EXTRA[*]:-}"
    echo "  rc: $TRC"
    echo "  label_topic: /teacher/cmd_vel"
    echo "  summary: teacher/summary.json"
  } >> "$OUT/meta.yaml"
fi
QC=$(sed -n 's/^QC_RESULT \([A-Z]*\) .*/\1/p' "$OUT/qc.txt" 2>/dev/null | head -1)
rm -rf "$TMP"
echo "AUTO_DONE $OUT teacher_rc=$TRC qc=${QC:-?}"
[ "$TRC" = 0 ] && [ "${QC:-}" = OK ]
