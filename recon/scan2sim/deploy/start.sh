#!/usr/bin/env bash
# scan2sim 서버와 Cloudflare 임시 터널을 뒤에서 켜고, 접속 주소와 접속 코드를 보여 줌.
#
#     bash recon/scan2sim/deploy/start.sh              # 서버 + 터널
#     S2S_TUNNEL=0 bash recon/scan2sim/deploy/start.sh # 이 컴퓨터에서만 (http://127.0.0.1:8000)
#
# - 서버가 죽으면 2초 뒤 다시 켬. 터널도 다시 켜지만 그때는 주소가 바뀜 → status.sh 로 새 주소를 볼 것.
# - 접속 코드는 처음 켤 때 6자리로 만들어 ~/scan2sim_run/code 에 두고 계속 씀. 직접 정하려면 S2S_CODE=<코드>.
# - ~/scan2sim_samples 폴더가 있으면 그 안의 .zip · .glb 가 화면의 예시 파일이 됨(S2S_SAMPLES). 그 밖의 값은 common.sh.
#   링크를 아는 누구나 파일을 올려 이 컴퓨터에서 처리를 돌릴 수 있으므로 터널을 켤 때는 코드를 끄지 않음.
# - 터미널을 닫아도 계속 돎. 끄기: stop.sh
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/common.sh"

[ -x "$PY" ] || fail "가상 환경이 없음 — 먼저 bash $DEPLOY_DIR/install.sh"
mkdir -p "$S2S_RUN" "$S2S_JOBS"
if alive "$S2S_RUN/server.pid"; then fail "이미 켜져 있음 — bash $DEPLOY_DIR/status.sh 로 주소를 보거나 stop.sh 로 끈 뒤 다시"; fi

if [ -n "${S2S_CODE:-}" ]; then printf '%s\n' "$S2S_CODE" >"$S2S_RUN/code"; fi
[ -s "$S2S_RUN/code" ] || printf '%06d\n' $(( (RANDOM * 32768 + RANDOM) % 1000000 )) >"$S2S_RUN/code"
chmod 600 "$S2S_RUN/code"
CODE="$(cat "$S2S_RUN/code")"

# 처리 중에 노트북이 잠들지 않게 (되는 컴퓨터에서만. 덮개를 닫으면 잠드는 것은 설정에서 따로 꺼야 함)
KEEP=()
if command -v systemd-inhibit >/dev/null 2>&1 && systemd-inhibit --what=idle:sleep --who=scan2sim --why="scan2sim 서버" true 2>/dev/null; then
    KEEP=(systemd-inhibit --what=idle:sleep --who=scan2sim --why="scan2sim 서버")
fi

# 서버에 넘길 선택 사항 (common.sh). 예시 파일 폴더는 있을 때만, 나머지 셋은 줄 때만 넘김
EXTRA=(--min-free-gb "$S2S_MIN_FREE_GB")
if [ -d "$S2S_SAMPLES" ]; then EXTRA+=(--samples "$S2S_SAMPLES"); fi
if [ -n "${S2S_TIMEOUT_MIN:-}" ]; then EXTRA+=(--timeout-min "$S2S_TIMEOUT_MIN"); fi
if [ -n "${S2S_MAX_WAIT:-}" ]; then EXTRA+=(--max-wait "$S2S_MAX_WAIT"); fi
if [ -n "${S2S_TTL_HOURS:-}" ]; then EXTRA+=(--ttl-hours "$S2S_TTL_HOURS"); fi
if [ -n "${S2S_KEEP:-}" ]; then EXTRA+=(--keep "$S2S_KEEP"); fi
if [ -n "${S2S_CODE_TRIES:-}" ]; then EXTRA+=(--code-tries "$S2S_CODE_TRIES"); fi
# 끝난 작업을 언제 지우는지 — 서버는 켜질 때도 지우므로, 지우면 안 되는 결과가 있으면 여기서 알 수 있게 함
[ -n "$S2S_FROM_FILE" ] && say "설정 파일   $S2S_RUN/settings.env → $S2S_FROM_FILE"
T="${S2S_TTL_HOURS:-24}"; K="${S2S_KEEP:-5}"
if [ "${T%.*}" -gt 0 ] 2>/dev/null; then KEEP_DIR="끝난 지 ${T}시간 넘은 작업 폴더를 통째로 지움 — **켜질 때도 지움**"; else KEEP_DIR="작업 폴더를 지우지 않음"; fi
if [ "$K" -ge 0 ] 2>/dev/null; then KEEP_IN="올린 파일은 최근 ${K}건만 둠"; else KEEP_IN="올린 파일을 지우지 않음"; fi
say "작업 보관   $KEEP_DIR · $KEEP_IN"
if [ "${T%.*}" -gt 0 ] 2>/dev/null; then say "            남겨 둘 결과가 있으면 지금 끄고($DEPLOY_DIR/stop.sh) $S2S_RUN/settings.env 에 S2S_TTL_HOURS=0 · S2S_KEEP=-1 을 적은 뒤 다시 켤 것 — 켜지고 나면 이미 지워졌을 수 있음"; fi
if [ "$(free_gb)" -lt "${S2S_MIN_FREE_GB%.*}" ]; then
    say "주의: 디스크 여유가 $(free_gb) GB 로 기준($S2S_MIN_FREE_GB GB)보다 적어 서버가 새 작업을 받지 않음 — 자리를 비우거나 S2S_MIN_FREE_GB 를 낮춰 다시 켤 것"
fi

: >"$S2S_RUN/server.log"
cd "$RECON_DIR"
PYTHONDONTWRITEBYTECODE=1 setsid nohup "${KEEP[@]}" bash -c '
    while :; do
        echo "[$(date "+%F %T")] 서버 켬"
        "$0" -u -m scan2sim.web.server --host 127.0.0.1 --port "$1" --jobs "$2" --code "$3" "${@:4}"
        echo "[$(date "+%F %T")] 서버가 꺼짐 (종료 코드 $?) — 2초 뒤 다시 켬"
        sleep 2
    done' "$PY" "$S2S_PORT" "$S2S_JOBS" "$CODE" "${EXTRA[@]}" >>"$S2S_RUN/server.log" 2>&1 </dev/null &
echo $! >"$S2S_RUN/server.pid"

for _ in $(seq 60); do
    "$PY" -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:$S2S_PORT/', timeout=2)" 2>/dev/null && break
    sleep 0.5
done
"$PY" -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:$S2S_PORT/', timeout=2)" 2>/dev/null \
    || { tail -20 "$S2S_RUN/server.log" >&2; bash "$DEPLOY_DIR/stop.sh" >/dev/null; fail "서버가 뜨지 않음 (위 로그). 포트 $S2S_PORT 를 다른 것이 쓰고 있으면 S2S_PORT 를 바꿀 것"; }

if [ "$S2S_TUNNEL" != "0" ]; then
    [ -x "$CLOUDFLARED" ] || { bash "$DEPLOY_DIR/stop.sh" >/dev/null; fail "cloudflared 가 없음 — install.sh 를 다시 돌릴 것"; }
    : >"$S2S_RUN/tunnel.log"
    setsid nohup bash -c '
        while :; do
            echo "[$(date "+%F %T")] 터널 켬"
            "$0" tunnel --no-autoupdate --url "http://127.0.0.1:$1"
            echo "[$(date "+%F %T")] 터널이 꺼짐 (종료 코드 $?) — 5초 뒤 다시 켬 (주소가 바뀜)"
            sleep 5
        done' "$CLOUDFLARED" "$S2S_PORT" >>"$S2S_RUN/tunnel.log" 2>&1 </dev/null &
    echo $! >"$S2S_RUN/tunnel.pid"
    for _ in $(seq 60); do [ -n "$(tunnel_url)" ] && break; sleep 0.5; done
    [ -n "$(tunnel_url)" ] || { tail -20 "$S2S_RUN/tunnel.log" >&2; say "터널 주소가 30초 안에 나오지 않음 (위 로그). 서버는 켜져 있음 — 잠시 뒤 status.sh 로 다시 볼 것"; }
fi

bash "$DEPLOY_DIR/status.sh"
