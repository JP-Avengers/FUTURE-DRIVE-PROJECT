#!/usr/bin/env bash
# 서버 · 터널이 켜져 있는지, 접속 주소 · 접속 코드, 작업 폴더 용량을 보여 줌.
#
#     bash recon/scan2sim/deploy/status.sh
set -uo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/common.sh"

if alive "$S2S_RUN/server.pid"; then
    if "$PY" -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:$S2S_PORT/', timeout=3)" 2>/dev/null; then
        say "서버   켜짐 · 응답함   http://127.0.0.1:$S2S_PORT"
    else
        say "서버   켜짐 · 응답 없음 (다시 켜지는 중일 수 있음 — $S2S_RUN/server.log)"
    fi
else
    say "서버   꺼짐"
fi

if alive "$S2S_RUN/tunnel.pid"; then
    U="$(tunnel_url)"
    say "터널   켜짐   ${U:-주소가 아직 안 나옴 — $S2S_RUN/tunnel.log}"
else
    say "터널   꺼짐"
fi

[ -s "$S2S_RUN/code" ] && say "접속 코드   $(cat "$S2S_RUN/code")"
if [ -d "$S2S_JOBS" ]; then
    say "작업 폴더   $S2S_JOBS — $(find "$S2S_JOBS" -mindepth 1 -maxdepth 1 -type d | wc -l)건 · $(du -sh "$S2S_JOBS" 2>/dev/null | cut -f1) · 디스크 여유 $(df -h --output=avail "$S2S_JOBS" | tail -1 | tr -d ' ')"
fi
