#!/usr/bin/env bash
# start.sh 로 켠 서버와 터널을 끔. 처리 중인 작업이 있으면 그 작업은 "서버가 다시 켜져서 끊김"으로 남음.
#
#     bash recon/scan2sim/deploy/stop.sh
set -uo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/common.sh"

for n in tunnel server; do
    f="$S2S_RUN/$n.pid"
    if alive "$f"; then
        p="$(cat "$f")"
        kill -TERM -- "-$p" 2>/dev/null || kill -TERM "$p" 2>/dev/null     # setsid 로 켰으므로 묶음째 끔
        for _ in $(seq 20); do kill -0 "$p" 2>/dev/null || break; sleep 0.25; done
        kill -KILL -- "-$p" 2>/dev/null
        say "$n 끔"
    else
        say "$n 은 켜져 있지 않음"
    fi
    rm -f "$f"
done
