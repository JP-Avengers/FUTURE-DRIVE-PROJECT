#!/usr/bin/env bash
# Isaac Sim WebRTC 스트리밍 기동 (server207, GPU 0 고정, Tailscale 경유)
# 사용: sim/run_streaming.sh [서버 Tailscale IP]   — 클라이언트 Server 칸에 같은 IP
# 포트: TCP 49100(시그널링) · UDP 47998(미디어). 스트림에 인증이 없으니 공개망에 열지 말 것.
set -euo pipefail
CD1=${CD1:-$HOME/Capstone_Design_1}
IP=${1:-$(tailscale ip -4 | head -1)}
export OMNI_KIT_ACCEPT_EULA=YES
# ROS 2 Bridge 는 기동 전에 시스템 Jazzy 를 source 해야 잡힌다 — 안 하면 내장 라이브러리로 넘어가다
# LD_LIBRARY_PATH 가 없어 'ROS2 Bridge startup failed'. 비대화형 셸은 .bashrc 를 끝까지 읽지 않으니 여기서 직접.
# setup.bash 가 미정의 변수를 참조하므로 source 동안만 -u 를 끈다
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
# NvStreamer 가 실행 폴더에 *.etli 트레이스 로그를 약 3분마다 쌓는다(시간당 ~140MB) → logs/ 에서 실행
mkdir -p "$CD1/logs/etli" && cd "$CD1/logs/etli"
exec "$CD1/envs/isaacsim_env/bin/isaacsim" isaacsim.exp.full.streaming --no-window \
  --/renderer/activeGpu=0 --/renderer/multiGpu/enabled=false \
  --/exts/omni.kit.livestream.app/primaryStream/publicIp="$IP" \
  --/exts/omni.kit.livestream.app/primaryStream/signalPort=49100 \
  --/exts/omni.kit.livestream.app/primaryStream/streamPort=47998
