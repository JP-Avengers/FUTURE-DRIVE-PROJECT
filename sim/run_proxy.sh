#!/usr/bin/env bash
# Proxy 씬 실행 — 같은 설정으로 뜬다: GPU 0 고정 · 시스템 Jazzy 먼저 source · ROS_DOMAIN_ID=42 ·
# 3D LiDAR 끔(규격서 "수집·평가 설정에서는 끔") · 씬 열고 Play. 씬 파일은 바꾸지 않는다(세션 레이어만).
#
#   sim/run_proxy.sh stream [IP]     스트리밍 — 1차 수집. tmux 안에서, 화면은 Mac 의 스트리밍 클라이언트(기본 IP = 서버 Tailscale)
#   sim/run_proxy.sh headless [초]   헤드리스 — tmux 안에서 (초를 안 주면 Ctrl+C 까지)
#   sim/run_proxy.sh check [초]      점검 — 씬이 Play 된 뒤 다른 터미널에서 (토픽 · /cmd_vel · RTF · 주기, 기본 10초)
#   sim/run_proxy.sh teleop [옵션]   키보드 조작(가감속 램프) — /cmd_vel 발행자가 0 일 때만 sim/teleop_ramp.py 를 띄운다
#   sim/run_proxy.sh monitor         이 서버에서는 못 씀(10.1) — 화면이 GPU 1 에만 나와 GPU 0 렌더 창을 못 그린다
#
# 끄기: 띄운 터미널에서 Ctrl+C (tmux 는 tmux send-keys -t <세션> C-c). 끝에 [Error] 줄 수와 GPU 상태를 찍는다.
# 환경변수: FD_LIDAR=1 LiDAR 켬 · FD_DOMAIN=43 시험용 도메인 · FD_VIEW_RES=1280x720 뷰포트 해상도(stream) · FD_TELEOP=stock 예전 조작(teleop)
# 로그: logs/run_proxy/<모드>_<월일_시분초>.log
set -euo pipefail
CD1=${CD1:-$HOME/Capstone_Design_1}
MODE=${1:-}
shift || true
usage() { sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }

# ROS 2 Bridge 는 Isaac Sim 기동 전에 시스템 Jazzy 를 같은 셸에서 source 해야 잡힌다(9.27).
# setup.bash 가 미정의 변수를 참조하므로 source 동안만 -u 를 끈다
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=${FD_DOMAIN:-42}   # .bashrc 값과 상관없이 고정. FD_DOMAIN 은 시험 격리용
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1
export FD_PROXY_MODE=$MODE FD_SCENE=$CD1/sim/scenes/proxy/proxy_parking.usda

case $MODE in
  check)
    exec /usr/bin/python3 "$CD1/sim/check_proxy_topics.py" "${1:-10}" ;;
  teleop)
    # 기본은 가감속 램프 조작(sim/teleop_ramp.py, 10.5 결정) — 키는 목표만 정하고 /cmd_vel 은 램프를 거쳐 연속으로 바뀌어 (v, ω) 라벨이 계단이 되지 않는다.
    # 인자는 그대로 넘긴다(예: sim/run_proxy.sh teleop --v-max 0.5). FD_TELEOP=stock 이면 예전 teleop_twist_keyboard
    # 램프 노드는 /cmd_vel 발행자를 스스로 확인하므로 아래 점검(약 3 s)을 건너뛴다 — 점검하는 동안 누른 키는 버려졌다(10.5 확인)
    if [ "${FD_TELEOP:-ramp}" != "stock" ]; then exec /usr/bin/python3 "$CD1/sim/teleop_ramp.py" "$@"; fi
    info=$(ros2 topic info /cmd_vel --no-daemon --spin-time 3 2>&1) || { echo "/cmd_vel 이 없음 — 씬이 Play 중인지, 도메인($ROS_DOMAIN_ID)이 같은지 확인"; exit 3; }
    pub=$(echo "$info" | sed -n 's/^Publisher count: *//p')
    if [ "${pub:-0}" != "0" ]; then echo "/cmd_vel 발행자가 이미 ${pub}개 — 발행자는 항상 1개(규격서). 먼저 그쪽을 끄세요"; exit 3; fi
    # speed · turn 은 시작값. 키 q/z(둘 다) · w/x(속도) · e/c(회전)로 10 %씩 바뀐다 — 상한(v 0.8 · ω 1.0)을 넘기지 말 것
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.5 -p turn:=0.5 ;;
  monitor)
    # 10.1 확인: 데스크톱 화면은 GPU 1 에만 나오는데(xorg.conf), 렌더를 GPU 0 에 고정한 창은 화면에 못 그린다
    # (createSwapchain failed → [Error] 700여 줄). GPU 1 렌더 · xorg 변경은 공용 서버 규칙상 하지 않는다. 예전 실행 방법은 918ca13
    echo "monitor 모드는 이 서버에서 쓸 수 없습니다 — 화면은 GPU 1 에만 나오는데 렌더는 GPU 0 고정이라 창을 못 그립니다(10.1 확인)."
    echo "대신: tmux 안에서 sim/run_proxy.sh stream → Mac 의 스트리밍 클라이언트로 보고, 다른 ssh 터미널에서 sim/run_proxy.sh teleop"
    exit 2 ;;
  stream|headless) ;;
  *) usage ;;
esac

# 공용 서버 — GPU 에 계산 프로세스가 하나라도 있으면 띄우지 않는다(남의 작업 · 이미 떠 있는 Isaac Sim)
busy=$(nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader)
if [ -n "$busy" ]; then echo "GPU 사용 중 — 띄우지 않음:"; echo "$busy"; exit 3; fi

ISAAC=$CD1/envs/isaacsim_env/bin
# GPU 0 고정 — 렌더 · 물리 둘 다. CUDA_VISIBLE_DEVICES 는 쓰지 않는다(종료 시 abort, 9.27). 옵션 이름 오타는 조용히 무시되니 주의
KIT_GPU=(--/renderer/activeGpu=0 --/renderer/multiGpu/enabled=false --/physics/cudaDevice=0)
RUNDIR=$CD1
case $MODE in
  stream)
    IP=${1:-$(tailscale ip -4 | head -1)}
    # 포트: TCP 49100(시그널링) · UDP 47998(미디어). 스트림에 인증이 없으니 공개망에 열지 말 것
    CMD=("$ISAAC/isaacsim" isaacsim.exp.full.streaming --no-window "${KIT_GPU[@]}"
         --/exts/omni.kit.livestream.app/primaryStream/publicIp="$IP"
         --/exts/omni.kit.livestream.app/primaryStream/signalPort=49100
         --/exts/omni.kit.livestream.app/primaryStream/streamPort=47998
         --exec "$CD1/sim/proxy_play.py")
    RUNDIR=$CD1/logs/etli ;;   # NvStreamer 가 실행 폴더에 *.etli 트레이스를 쌓는다(시간당 ~140MB)
  headless)
    export FD_DURATION=${1:-0}
    CMD=("$ISAAC/python" "$CD1/sim/proxy_play.py") ;;
esac

mkdir -p "$CD1/logs/run_proxy" "$RUNDIR"
LOG=$CD1/logs/run_proxy/${MODE}_$(date +%m%d_%H%M%S).log
echo "RUN_PROXY mode=$MODE domain=$ROS_DOMAIN_ID lidar=${FD_LIDAR:-0} view_res=${FD_VIEW_RES:-기본} log=$LOG"
cd "$RUNDIR"
# Ctrl+C 는 Isaac Sim 이 받아 스스로 끝난다. 이 셸은 그 뒤에 요약을 찍으려고 빈 처리기만 둔다
# (trap '' 로 무시하면 자식도 무시한 채 시작해 Ctrl+C 가 안 먹는다)
trap 'echo "Ctrl+C — Isaac Sim 종료 대기"' INT
set +e
"${CMD[@]}" 2>&1 | tee -i "$LOG"
rc=${PIPESTATUS[0]}
set -e
echo "RUN_PROXY_EXIT=$rc [Error]=$(grep -c '\[Error\]' "$LOG" || true) log=$LOG"
echo "GPU 계산 프로세스: $(nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader | tr '\n' ' ')"
exit "$rc"
