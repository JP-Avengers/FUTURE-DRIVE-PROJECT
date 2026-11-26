#!/usr/bin/env bash
# 컴퓨터를 다시 켠 뒤 서버와 터널이 알아서 켜지게 함 — 사용자 crontab 에 @reboot 한 줄을 넣고 뺌 (sudo 없이).
#
#     S2S_TTL_HOURS=0 S2S_KEEP=-1 bash recon/scan2sim/deploy/autostart.sh on    # 넣기. 이때 준 S2S_* 값이 그 줄에 같이 적힘
#     bash recon/scan2sim/deploy/autostart.sh status                           # 들어 있는 줄을 보여 줌
#     bash recon/scan2sim/deploy/autostart.sh off                              # 빼기
#
# - 다시 켜지면 30초 기다린 뒤(네트워크가 붙을 시간) start.sh 를 부름. 기록은 ~/scan2sim_run/boot.log (접속 코드가 찍히므로 본인만 읽게 둠).
# - 접속 코드는 ~/scan2sim_run/code 에 남아 있어 그대로임. **터널 주소는 켤 때마다 바뀜** — 다시 켜진 뒤 status.sh 로 새 주소를 봐야 함.
# - S2S_CODE 는 그 줄에 적지 않음(crontab 에 코드를 남기지 않으려고). 로그인하지 않아도 켜짐(cron 이 돌 때).
# - 컴퓨터가 꺼질 때 돌던 작업은 다시 켜지면 줄에 한 번 다시 들어감(server.py recover).
set -uo pipefail
GIVEN="$(env | grep -E '^S2S_[A-Z_]+=' | grep -vE '^S2S_(CODE|CRONTAB)=' | sort || true)"     # common.sh 가 기본값을 채우기 전에 — 사용자가 준 것만
. "$(dirname "${BASH_SOURCE[0]}")/common.sh"
CRONTAB="${S2S_CRONTAB:-crontab}"
TAG="# scan2sim-autostart"
current() { "$CRONTAB" -l 2>/dev/null | grep -F "$TAG" || true; }
others()  { "$CRONTAB" -l 2>/dev/null | grep -vF "$TAG" || true; }

case "${1:-}" in
on)
    command -v "$CRONTAB" >/dev/null 2>&1 || fail "crontab 이 없음 — cron 을 설치하거나(sudo 필요) 다시 켠 뒤 start.sh 를 직접 부를 것"
    [ -x "$PY" ] || fail "설치가 안 돼 있음 — 먼저 bash $DEPLOY_DIR/install.sh"
    VARS=""
    while IFS= read -r kv; do
        [ -n "$kv" ] && VARS+="$(printf '%s=%q ' "${kv%%=*}" "${kv#*=}")"
    done <<<"$GIVEN"
    Q() { printf '%q' "$1"; }
    # 다시 켜진 직후에는 예전 프로세스 번호 파일이 가리키는 것이 살아 있을 수 없음 — 지우고 켬(그 번호를 다른 프로그램이 받았을 때 "이미 켜져 있음"으로 멈추지 않게)
    LINE="@reboot sleep 30; mkdir -p $(Q "$S2S_RUN"); rm -f $(Q "$S2S_RUN/server.pid") $(Q "$S2S_RUN/tunnel.pid"); touch $(Q "$S2S_RUN/boot.log"); chmod 600 $(Q "$S2S_RUN/boot.log"); ${VARS}bash $(Q "$DEPLOY_DIR/start.sh") >>$(Q "$S2S_RUN/boot.log") 2>&1 $TAG"
    case "$LINE" in *%*) fail "값에 % 가 들어 있어 crontab 에 적을 수 없음" ;; esac
    REST="$(others)"                                            # 먼저 다 읽어 둠 — 읽는 것과 쓰는 것이 겹치지 않게
    { [ -n "$REST" ] && printf '%s\n' "$REST"; printf '%s\n' "$LINE"; } | "$CRONTAB" - || fail "crontab 에 쓰지 못함"
    say "다시 켜지면 알아서 켜지게 함. 넣은 줄:"
    current
    say "주의: 터널 주소는 다시 켜질 때마다 바뀜 — bash $DEPLOY_DIR/status.sh 로 새 주소를 볼 것"
    ;;
off)
    if [ -z "$(current)" ]; then say "넣어 둔 줄이 없음"; exit 0; fi
    REST="$(others)"
    if [ -n "$REST" ]; then printf '%s\n' "$REST" | "$CRONTAB" - || fail "crontab 에 쓰지 못함"
    else "$CRONTAB" -r 2>/dev/null || printf '' | "$CRONTAB" - || fail "crontab 을 비우지 못함"; fi
    say "뺌 — 다시 켜져도 알아서 켜지지 않음"
    ;;
status)
    if [ -n "$(current)" ]; then say "넣어 둠:"; current; else say "넣어 두지 않음 — 다시 켜지면 start.sh 를 직접 불러야 함"; fi
    ;;
*)
    fail "쓰는 법: bash $DEPLOY_DIR/autostart.sh on|off|status"
    ;;
esac
