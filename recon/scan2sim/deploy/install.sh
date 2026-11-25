#!/usr/bin/env bash
# scan2sim 을 새 리눅스 컴퓨터(팀 ThinkPad)에 설치함. 여러 번 돌려도 됨 — 이미 된 것은 건너뜀.
#
#     bash recon/scan2sim/deploy/install.sh
#
# 하는 일: ① 파이썬 3.12 확인 ② 가상 환경 ~/scan2sim_env 만들고 recon/requirements.txt 설치
#          ③ 불러오기 확인 ④ cloudflared 내려받기(~/.local/bin) ⑤ 서버를 잠깐 켜서 응답 확인
# sudo 를 쓰지 않음. 파이썬 3.12 나 venv 가 없으면 필요한 명령만 알려 주고 멈춤.
# S2S_TUNNEL=0 을 주면 ④ 를 건너뜀.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/common.sh"

# 엔진이 recon/ 밖의 파일도 씀 (Nav2 지도 = sim/nav2/make_map.py). recon/ 만 복사해 오면 지도 · 진단 단계가 안 됨 — 저장소를 통째로 받을 것
[ -f "$RECON_DIR/../sim/nav2/make_map.py" ] || fail "sim/nav2/make_map.py 가 없음 — recon/ 만 복사하지 말고 저장소 전체를 받을 것 (git clone)"

say "== 1/5 파이썬 3.12"
PY312=""
for c in python3.12 python3; do
    if command -v "$c" >/dev/null 2>&1 && [ "$("$c" -c 'import sys; print("%d.%d" % sys.version_info[:2])')" = "3.12" ]; then PY312="$c"; break; fi
done
[ -n "$PY312" ] || fail "파이썬 3.12 가 없음. 우분투라면: sudo apt install python3.12 python3.12-venv  (requirements.txt 의 버전이 3.12 기준임)"
say "   $(command -v "$PY312") — $("$PY312" --version)"

say "== 2/5 가상 환경 $S2S_ENV"
if [ ! -x "$PY" ]; then
    "$PY312" -m venv "$S2S_ENV" || fail "venv 를 만들지 못함. 우분투라면: sudo apt install python3.12-venv"
fi
"$PY" -m pip install -q --upgrade pip
"$PY" -m pip install -q -r "$RECON_DIR/requirements.txt"
say "   설치 끝"

say "== 3/5 불러오기 확인"
( cd "$RECON_DIR" && PYTHONDONTWRITEBYTECODE=1 "$PY" - <<'EOF'
import importlib
from importlib.metadata import version
mods = dict(open3d="open3d", pxr="usd-core", numpy="numpy", scipy="scipy", trimesh="trimesh", cv2="opencv-python-headless", skimage="scikit-image",
            flask="Flask", PIL="pillow", waitress="waitress", segno="segno", yaml="PyYAML", pandas="pandas", matplotlib="matplotlib")
for m, pkg in mods.items():
    importlib.import_module(m)
    print(f"   {m} {version(pkg)}")
import cv2
assert "FFMPEG:                      YES" in cv2.getBuildInformation(), "OpenCV 에 동영상 디코더(FFMPEG)가 없음 — rgb.mp4 를 못 읽음"
import scan2sim.web.server, scan2sim.stray                      # 엔진과 서버가 불러와지는지
print("   scan2sim 불러오기 OK · OpenCV 동영상 디코더 있음")
EOF
) || fail "불러오기 확인에서 멈춤 (위 메시지)"

say "== 4/5 cloudflared"
if [ "$S2S_TUNNEL" = "0" ]; then
    say "   건너뜀 (S2S_TUNNEL=0)"
elif [ -x "$CLOUDFLARED" ]; then
    say "   이미 있음: $CLOUDFLARED — $("$CLOUDFLARED" --version 2>&1 | head -1)"
else
    case "$(uname -m)" in x86_64) A=amd64 ;; aarch64) A=arm64 ;; *) fail "지원하지 않는 CPU 종류: $(uname -m)" ;; esac
    mkdir -p "$S2S_BIN"
    U="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-$A"
    say "   내려받기: $U"
    if command -v curl >/dev/null 2>&1; then curl -fsSL -o "$CLOUDFLARED.part" "$U"; else wget -q -O "$CLOUDFLARED.part" "$U"; fi
    chmod +x "$CLOUDFLARED.part" && mv "$CLOUDFLARED.part" "$CLOUDFLARED"
    say "   $CLOUDFLARED — $("$CLOUDFLARED" --version 2>&1 | head -1)"
fi

say "== 5/5 서버 응답 확인"
T="$(mktemp -d)"; P=$((S2S_PORT + 1700))
( cd "$RECON_DIR" && PYTHONDONTWRITEBYTECODE=1 exec "$PY" -m scan2sim.web.server --port "$P" --jobs "$T/jobs" --code check ) >"$T/server.log" 2>&1 &
SP=$!
trap 'kill $SP 2>/dev/null || true; rm -rf "$T"' EXIT
OK=0
for _ in $(seq 40); do
    if "$PY" - "$P" <<'EOF' 2>/dev/null
import json, sys, urllib.error, urllib.request
b = f"http://127.0.0.1:{sys.argv[1]}"
assert b"<html" in urllib.request.urlopen(b + "/", timeout=3).read().lower()            # 화면
def post(code):
    r = urllib.request.Request(b + "/api/jobs", json.dumps(dict(name="a.zip", size=10)).encode(), {"Content-Type": "application/json", "X-Code": code})
    return json.load(urllib.request.urlopen(r, timeout=3))
try:
    post("wrong"); raise SystemExit("접속 코드가 틀려도 받아 줌")
except urllib.error.HTTPError as e:
    assert e.code == 401
assert len(post("check")["id"]) == 12                                                   # 올릴 자리 만들기
EOF
    then OK=1; break; fi
    kill -0 $SP 2>/dev/null || break
    sleep 0.5
done
[ "$OK" = 1 ] || { cat "$T/server.log" >&2; fail "서버가 응답하지 않음 (위 로그)"; }
say "   화면 · 접속 코드 · 올릴 자리 만들기 OK"

say ""
say "설치 끝. 켜기: bash $DEPLOY_DIR/start.sh"
