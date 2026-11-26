# scan2sim 배포 스크립트들이 함께 쓰는 값 — 직접 실행하지 않고 다른 스크립트가 불러 씀.
# 바꾸고 싶으면 실행할 때 환경 변수로 줌. 예: S2S_PORT=8010 bash start.sh

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RECON_DIR="$(cd "$DEPLOY_DIR/../.." && pwd)"                 # <레포>/recon

S2S_ENV="${S2S_ENV:-$HOME/scan2sim_env}"                      # 파이썬 가상 환경
S2S_JOBS="${S2S_JOBS:-$HOME/scan2sim_jobs}"                   # 올린 파일과 결과 (레포 밖)
S2S_RUN="${S2S_RUN:-$HOME/scan2sim_run}"                      # 로그 · 프로세스 번호 · 접속 코드
S2S_BIN="${S2S_BIN:-$HOME/.local/bin}"                        # cloudflared 를 두는 곳
S2S_PORT="${S2S_PORT:-8000}"
S2S_TUNNEL="${S2S_TUNNEL:-1}"                                 # 0 이면 터널 없이 이 컴퓨터에서만
S2S_SAMPLES="${S2S_SAMPLES:-$HOME/scan2sim_samples}"          # 예시 파일(.zip · .glb) 폴더 — 있을 때만 서버에 넘김
S2S_MIN_FREE_GB="${S2S_MIN_FREE_GB:-20}"                      # 디스크 여유가 이보다 적으면 새 작업을 거절(507)
S2S_QR="${S2S_QR:-1}"                                         # 0 이면 status.sh 가 QR 을 그리지 않음
# 줄 때만 서버에 넘기는 것(안 주면 서버 기본값): S2S_TIMEOUT_MIN(20) · S2S_MAX_WAIT(5) · S2S_TTL_HOURS(24) · S2S_KEEP(5)

PY="$S2S_ENV/bin/python"
CLOUDFLARED="$S2S_BIN/cloudflared"
command -v cloudflared >/dev/null 2>&1 && [ ! -x "$CLOUDFLARED" ] && CLOUDFLARED="$(command -v cloudflared)"

say()  { printf '%s\n' "$*"; }
fail() { printf '실패: %s\n' "$*" >&2; exit 1; }

# 프로세스 번호 파일에 적힌 프로세스 묶음이 살아 있는가
alive() { [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null; }

# 작업 폴더가 있는 디스크의 여유 (GB, 내림)
free_gb() { echo $(( $(df -Pk "$S2S_JOBS" | awk 'NR==2 {print $4}') / 1048576 )); }

# 터널 로그에서 가장 최근 주소
tunnel_url() { grep -o 'https://[a-z0-9-]*\.trycloudflare\.com' "$S2S_RUN/tunnel.log" 2>/dev/null | tail -1; }
