#!/usr/bin/env bash
# scan2sim 자동 시험 — 커밋 전 · 매일 통합 때 돌린다.
#
#   cd recon
#   bash scan2sim/tests/run.sh            # 전체 (약 5~8분): 가상의 방 GLB · 가상 Stray 촬영본, 같은 입력 두 번, 복원 정확도
#   bash scan2sim/tests/run.sh --quick    # GLB 만 (약 1분)
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

step "GLB: 서버로 올려 끝까지 + 넣어 둔 결함 3개가 제자리(0.5 m 안)에 잡히는지"
run e2e_glb $PY -m scan2sim.tests.e2e "$D/room/room.glb" --truth "$D/room/room_truth.json" --json "$D/e2e_glb.json"

step "GLB: 같은 입력 두 번"
run repeat_glb $PY -m scan2sim.tests.repeat "$D/room/room.glb" --json "$D/repeat_glb.json"

if [ "$QUICK" = 0 ]; then
    step "Stray: 서버로 올려 끝까지"
    # 결함 위치는 견주지 않음 — Stray(맵 폴더) 경로는 바닥을 새로 깔아 넣어 둔 바닥 결함이 진단에 남지 않음 (docs/integration_log.md R8)
    run e2e_stray $PY -m scan2sim.tests.e2e "$D/room/synth_room_s0.zip" --json "$D/e2e_stray.json"

    step "Stray: 같은 입력 두 번 + 복원 정확도(안 쓴 프레임 깊이 ↔ 맵, 중앙 3 cm 이하)"
    run repeat_stray $PY -m scan2sim.tests.repeat "$D/room/synth_room_s0.zip" --work "$D/repeat_stray" --json "$D/repeat_stray.json"
    run quality_stray $PY -c "
import sys
from scan2sim import stray_quality as q
r = q.measure(sys.argv[1], write=False)
print('  ', r['summary'])
sys.exit(0 if r['verdict'] == 'pass' else 1)
" "$D/repeat_stray/run0"
fi

echo
echo "걸린 시간 $(( $(date +%s) - t0 ))초 · 결과 $D"
if [ ${#FAIL[@]} -eq 0 ]; then echo "TESTS_OK"; else echo "TESTS_FAIL ${FAIL[*]}"; exit 1; fi
