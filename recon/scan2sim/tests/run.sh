#!/usr/bin/env bash
# scan2sim 자동 시험 — 커밋 전 · 매일 통합 때 돌린다.
#
#   cd recon
#   bash scan2sim/tests/run.sh            # 전체 (약 5~8분): 가상의 방 GLB · 가상 Stray 촬영본, 같은 입력 두 번, 복원 정확도, Stray 경계 사례,
#                                         #   되돌아온 자리 어긋남
#   bash scan2sim/tests/run.sh --quick    # GLB + Stray 경계 사례 (약 1분)
#
# 시험 자료는 samples/ 의 스크립트로 저장소 밖 임시 폴더에 만든다(SCAN2SIM_TEST_DIR 로 바꿀 수 있음, 있으면 다시 만들지 않음).
# 파이썬은 지금 쓰는 python (다른 것을 쓰려면 PY=… bash …).
# 끝 줄: TESTS_OK 또는 TESTS_FAIL (실패한 시험 이름). 실패면 종료 코드 1.
set -u
cd "$(dirname "$0")/../.."                       # recon/
PY=${PY:-python}
QUICK=0; [ "${1:-}" = "--quick" ] && QUICK=1
D=${SCAN2SIM_TEST_DIR:-${TMPDIR:-/tmp}/scan2sim_tests}
mkdir -p "$D"
FAIL=()
t0=$(date +%s)

step() { echo; echo "== $1"; }
run() {                                          # run <이름> <명령…>
    local name=$1; shift
    if "$@"; then echo "  → $name 통과"; else echo "  → $name 실패"; FAIL+=("$name"); fi
}

step "시험 자료 (가상의 방)"
[ -f "$D/room/room.glb" ] || $PY scan2sim/samples/make_room.py --out "$D/room" --seed 0 || FAIL+=("make_room")
[ "$QUICK" = 1 ] || [ -f "$D/room/synth_room_s0.zip" ] || $PY scan2sim/samples/make_stray.py --room "$D/room" --out "$D/room" --seed 0 || FAIL+=("make_stray")

step "GLB: 서버로 올려 끝까지 + 넣어 둔 결함 3개가 넣은 넓이에서 0.25 m 안에 잡히는지"
run e2e_glb $PY -m scan2sim.tests.e2e "$D/room/room.glb" --truth "$D/room/room_truth.json" --json "$D/e2e_glb.json"

step "GLB: 같은 입력 두 번"
run repeat_glb $PY -m scan2sim.tests.repeat "$D/room/room.glb" --json "$D/repeat_glb.json"

step "Stray 경계 사례: 깨진 입력 · zip 이름 · frame 번호 짝짓기 · 멈춘 변환 · 깨진 장 · 읽기 전용 맵 · 보정 sha256 · --tilt (6 초 가상 촬영본, 서버 없이)"
# 6 초 가상 촬영본은 처음 한 번만 만듦 ($D/edge, 약 20초) — 시험은 약 15초
run stray_edge $PY -m scan2sim.tests.stray_edge --fixtures "$D/edge" --work "$D/stray_edge" --json "$D/stray_edge.json"

if [ "$QUICK" = 0 ]; then
    step "Stray: 서버로 올려 끝까지 + 넣어 둔 결함 3개가 넣은 넓이에서 0.25 m 안에 잡히는지"
    # 11/24 C 의 0534d95 부터 Stray 도 GLB 와 같은 길이라 바닥 결함이 남음 (integration_log R8)
    run e2e_stray $PY -m scan2sim.tests.e2e "$D/room/synth_room_s0.zip" --truth "$D/room/stray_truth.json" --json "$D/e2e_stray.json"

    step "Stray: 같은 입력 두 번 + 복원 정확도(안 쓴 프레임 깊이 ↔ 맵, 중앙 3 cm 이하)"
    run repeat_stray $PY -m scan2sim.tests.repeat "$D/room/synth_room_s0.zip" --work "$D/repeat_stray" --json "$D/repeat_stray.json"
    run quality_stray $PY -c "
import sys
from scan2sim import stray_quality as q
r = q.measure(sys.argv[1], write=False)
print('  ', r['summary'])
sys.exit(0 if r['verdict'] == 'pass' else 1)
" "$D/repeat_stray/run0"

    step "Stray: 되돌아온 자리 어긋남 — 두 바퀴 가상 촬영본에 알려진 틀어짐을 넣고 되찾는지 · 결과 폴더 꼴 · 다시 지나간 곳 없음 (서버 없이)"
    # 50 초 · 10 장/초 가상 촬영본은 처음 한 번만 만듦 ($D/revisit, 약 40초) — 시험은 약 10초
    run revisit $PY -m scan2sim.tests.revisit --fixtures "$D/revisit" --room "$D/room" --work "$D/revisit_work" --json "$D/revisit.json"
fi

echo
echo "걸린 시간 $(( $(date +%s) - t0 ))초 · 결과 $D"
if [ ${#FAIL[@]} -eq 0 ]; then echo "TESTS_OK"; else echo "TESTS_FAIL ${FAIL[*]}"; exit 1; fi
