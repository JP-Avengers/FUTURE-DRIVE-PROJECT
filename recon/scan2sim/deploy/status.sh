#!/usr/bin/env bash
# 서버 · 터널이 켜져 있는지, 접속 주소 · 접속 코드 · 링크와 QR, 줄 · 작업 폴더 용량을 보여 줌.
#
#     bash recon/scan2sim/deploy/status.sh
#     S2S_QR=0 bash recon/scan2sim/deploy/status.sh     # QR 없이
#
# 링크와 QR 에는 접속 코드가 들어 있음(…/?code=<코드>) — 공개 자료 · 저장소에 남기지 말 것. QR 그림은 ~/scan2sim_run/qr.png 에도 남김.
set -uo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/common.sh"

CODE=""; U=""
if [ -s "$S2S_RUN/code" ]; then CODE="$(cat "$S2S_RUN/code")"; fi

if alive "$S2S_RUN/server.pid"; then
    H="$("$PY" - "$S2S_PORT" "$CODE" <<'EOF' 2>/dev/null
import json, sys, urllib.request
r = urllib.request.Request(f"http://127.0.0.1:{sys.argv[1]}/api/health", headers={"X-Code": sys.argv[2]})
h = json.load(urllib.request.urlopen(r, timeout=3))
a = h.get("addr_header")
print(f"기다리는 작업 {h['queue_length']}건 · {'처리 중' if h['running'] else '쉬는 중'} · 방문자 주소 " + (f"{a} 로 구분" if a else "구분 안 됨(터널로 온 요청이 아직 없거나 터널이 주소를 안 알려 줌)"))
EOF
)"
    if [ -n "$H" ]; then
        say "서버   켜짐 · 응답함   http://127.0.0.1:$S2S_PORT   $H"
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

if [ -n "$CODE" ]; then say "접속 코드   $CODE"; fi
if [ -n "$U" ] && [ -n "$CODE" ]; then
    L="$U/?code=$("$PY" -c 'import sys, urllib.parse; print(urllib.parse.quote(sys.argv[1], safe=""))' "$CODE")"
    say "링크   $L"
    if [ "$S2S_QR" != "0" ]; then
        "$PY" - "$L" "$S2S_RUN/qr.png" <<'EOF' 2>/dev/null || say "QR 을 그리지 못함 — install.sh 를 다시 돌릴 것 (segno)"
import sys, segno
q = segno.make(sys.argv[1], error="m")
q.save(sys.argv[2], scale=12, border=4)
q.terminal(compact=True, border=2)
print("QR 그림  " + sys.argv[2])
EOF
    fi
fi

if [ -d "$S2S_SAMPLES" ]; then
    say "예시 파일   $S2S_SAMPLES — $(find "$S2S_SAMPLES" -maxdepth 1 -type f \( -name '*.zip' -o -name '*.glb' \) | wc -l)개"
else
    say "예시 파일   없음 ($S2S_SAMPLES 폴더를 만들고 .zip · .glb 를 넣은 뒤 다시 켜면 화면에 나옴)"
fi
if [ -d "$S2S_JOBS" ]; then
    say "작업 폴더   $S2S_JOBS — $(find "$S2S_JOBS" -mindepth 1 -maxdepth 1 -type d | wc -l)건 · $(du -sh "$S2S_JOBS" 2>/dev/null | cut -f1) · 디스크 여유 $(free_gb) GB (기준 $S2S_MIN_FREE_GB GB)"
fi
