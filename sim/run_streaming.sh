#!/usr/bin/env bash
# Isaac Sim WebRTC 스트리밍 기동 (server207, GPU 0 고정, Tailscale 경유)
# 사용: sim/run_streaming.sh [서버 Tailscale IP]   — 클라이언트 Server 칸에 같은 IP
# 포트: TCP 49100(시그널링) · UDP 47998(미디어). 스트림에 인증이 없으니 공개망에 열지 말 것.
set -euo pipefail
CD1=${CD1:-$HOME/Capstone_Design_1}
IP=${1:-$(tailscale ip -4 | head -1)}
export OMNI_KIT_ACCEPT_EULA=YES
exec "$CD1/envs/isaacsim_env/bin/isaacsim" isaacsim.exp.full.streaming --no-window \
  --/renderer/activeGpu=0 --/renderer/multiGpu/enabled=false \
  --/exts/omni.kit.livestream.app/primaryStream/publicIp="$IP" \
  --/exts/omni.kit.livestream.app/primaryStream/signalPort=49100 \
  --/exts/omni.kit.livestream.app/primaryStream/streamPort=47998
