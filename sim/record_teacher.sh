#!/usr/bin/env bash
# 교사 데이터 원본 녹화 — 규격서(docs/interfaces/topics.md) 토픽 전부를 rosbag2 MCAP 으로, 시뮬 시간 기준.
# 씬(sim/run_proxy.sh stream|headless)이 Play 된 뒤 다른 터미널(tmux)에서 띄운다. 조작(teleop)과는 따로 돈다 — 이 녹화는 구독만 한다.
#
#   sim/record_teacher.sh <운전자> <cw|ccw> [메모]      예) sim/record_teacher.sh D cw 정상주행
#   끄기: Ctrl+C — 녹화를 닫고 sim/qc_teacher.py 로 바로 점검한다(결과는 녹화 폴더의 qc.txt).
#
# 규칙: Stop → Play 하면 시뮬 시간이 0 으로 돌아간다(10.1) → Stop 전에 이 녹화를 끄고, Play 뒤 새로 띄운다.
# 저장: data/teacher/raw_v1/<월일_시분초>_<운전자>_<방향>/ (MCAP · meta.yaml · qc.txt). Git 에 안 올림 → MANIFEST 에 묶음 단위로 기록.
# 10 fps 샘플링 · 학습용 변환(HDF5)은 하지 않는다 — 원본을 그대로 남겨 E 가 정하는 방식으로 나중에 뽑는다.
# 환경변수: FD_DOMAIN(기본 42 — 씬과 같아야 함) · FD_REC_COMPRESS=0 압축 끔(기본 zstd_fast) · FD_TEACHER_DIR 저장 위치
#          FD_SCENE(씬 루트, meta 에 남김 — run_proxy.sh 와 같은 값) · FD_REC_SPLIT=초 파일 나누기(긴 무인 녹화용)
# 자동 교사(sim/teacher.py)의 라벨 /teacher/cmd_vel · 상태 /teacher/status 도 함께 남긴다 — 보통은 sim/auto_teacher.sh 가 이 녹화를 띄운다.
set -euo pipefail
CD1=${CD1:-$HOME/Capstone_Design_1}
OP=${1:-}; DIR=${2:-}; NOTE=${3:-}
if [ -z "$OP" ] || { [ "$DIR" != cw ] && [ "$DIR" != ccw ]; }; then sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 2; fi

set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=${FD_DOMAIN:-42} RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# 규격서 2장 토픽. /route_command 는 수집 때 아무도 내지 않는 안(경로 파일 + 자세로 나중에 붙임)이지만, 누가 내면 남도록 넣어 둔다
TOPICS=(/front_camera/image_raw /front_camera/camera_info /chassis/odom /chassis/imu /tf /clock /cmd_vel /route_command
        /teacher/cmd_vel /teacher/status)

# 씬이 Play 중이어야 시뮬 시간으로 기록된다
if ! timeout 15 ros2 topic echo --once /clock >/dev/null 2>&1; then
  echo "/clock 이 안 옴 — 씬이 Play 중인지, 도메인($ROS_DOMAIN_ID)이 씬과 같은지 확인"; exit 3
fi
# info 가 실패하면(씬이 막 떴거나 부하로 발견이 늦음) set -e 때문에 아무 말 없이 끝났음(10.12 자동 교사 시험) — 실패는 0 으로 본다
pub=$(ros2 topic info /cmd_vel --no-daemon --spin-time 3 2>/dev/null | sed -n 's/^Publisher count: *//p' || true)
if [ "${pub:-0}" -gt 1 ]; then echo "/cmd_vel 발행자가 ${pub}개 — 발행자는 항상 1개(규격서)"; exit 3; fi

ROOT=${FD_TEACHER_DIR:-$CD1/data/teacher/raw_v1}
NAME=$(date +%m%d_%H%M%S)_${OP}_${DIR}
OUT=$ROOT/$NAME
mkdir -p "$ROOT"
STORAGE=(-s mcap)
if [ "${FD_REC_COMPRESS:-1}" = 1 ]; then STORAGE+=(--storage-preset-profile zstd_fast); fi
if [ -n "${FD_REC_SPLIT:-}" ]; then STORAGE+=(--max-bag-duration "$FD_REC_SPLIT"); fi
SCENE=$(realpath "${FD_SCENE:-$CD1/sim/scenes/proxy/proxy_parking.usda}")

echo "REC_START $OUT domain=$ROS_DOMAIN_ID compress=${FD_REC_COMPRESS:-1} — Ctrl+C 로 끝"
# Ctrl+C 는 ros2 bag 이 받아 파일을 닫는다. 이 셸은 그 뒤 점검을 하려고 빈 처리기만 둔다
trap 'echo "Ctrl+C — 녹화 닫는 중"' INT
set +e
# --use-sim-time: /cmd_vel(Twist)에는 헤더가 없어 수신 시각을 시뮬 시간으로 남겨야 한다(9.29)
# --include-unpublished-topics: 발행자가 아직 없는 /cmd_vel 도 처음부터 구독해 첫 명령을 놓치지 않는다
ros2 bag record "${STORAGE[@]}" --use-sim-time --include-unpublished-topics -o "$OUT" --topics "${TOPICS[@]}"
rc=$?
set -e
trap - INT
if [ ! -f "$OUT/metadata.yaml" ]; then echo "REC_FAIL rc=$rc — $OUT 에 metadata.yaml 이 없음"; exit 4; fi

dirty=$(git -C "$CD1" diff --quiet HEAD -- sim docs 2>/dev/null || echo "-dirty")
cat > "$OUT/meta.yaml" <<EOF
operator: "$OP"
direction: $DIR
note: "$NOTE"
recorded_at: "$(date -Iseconds)"
repo_commit: "$(git -C "$CD1" rev-parse --short HEAD)$dirty"
scene_params_sha256_12: "$(sha256sum "$CD1/sim/scenes/proxy/params.yaml" | cut -c1-12)"
scene: "${SCENE#$CD1/}"
scene_root_sha256_12: "$(sha256sum "$SCENE" | cut -c1-12)"
ros_domain_id: $ROS_DOMAIN_ID
compress: "${FD_REC_COMPRESS:-1}"
EOF
/usr/bin/python3 "$CD1/sim/qc_teacher.py" "$OUT" | tee "$OUT/qc.txt"
echo "REC_DONE $OUT"
