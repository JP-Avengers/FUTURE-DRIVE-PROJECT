#!/usr/bin/env bash
# 교사 데이터 → X-Mobility 학습 형식 (sim/export_xmobility.py, 묶음마다 하나씩 병렬).
# 원본 폴더 = FD_XM_SRC(기본 "raw_v1 nav2_v1" = xm_v1). xm_v2(11.05) = FD_XM_SRC="raw_v1 nav2_v1 nav2_v2".
# 나눔(묶음 단위): 규칙 파일 FD_XM_SPLIT(기본 sim/export_split_parking_r3.txt)에서 읽음 — 묶음 폴더 이름이나 meta.yaml 안의 문자열로 val · test 를
#                  정하고 나머지는 train(11.17 — 그 전엔 주차장 r3 의 묶음 이름 · rand_tr_18 · 19 가 이 스크립트에 박혀 있었음. 규칙 내용은 그대로).
# 추가 인자: FD_XM_ARGS(기본 없음) — export_xmobility.py 에 그대로 붙임. 예 FD_XM_ARGS="--exec-cmd"(실행 명령 열, 11.10 E · D 동의)
# 사용: [FD_XM_SRC="…"] [FD_XM_ARGS="…"] [FD_XM_SPLIT=규칙 파일] sim/export_xmobility.sh [출력 폴더=data/teacher/xm_v1] [병렬=8]
#       FD_XM_LIST=1 이면 변환하지 않고 "묶음 split" 목록만 찍고 끝(나눔 확인용)
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=${1:-data/teacher/xm_v1}
JOBS=${2:-8}
PY=${FD_XM_PY:-$HOME/xmobility_test/venv/bin/python}   # pandas · pyarrow 있는 python 3.12
RULES=${FD_XM_SPLIT:-sim/export_split_parking_r3.txt}
[ -f "$RULES" ] || { echo "나눔 규칙 파일 없음: $RULES (FD_XM_SPLIT)"; exit 1; }

split_of() {   # $1 묶음 폴더 → train | val | test. 규칙을 위에서부터 보고 뒤에 걸린 것이 이김
  local b=$1 n sp=train kind key val
  n=$(basename "$b")
  while read -r kind key val; do
    case $kind in
      name) if [ "$n" = "$key" ]; then sp=$val; fi ;;
      meta) if grep -q -- "$key" "$b/meta.yaml" 2>/dev/null; then sp=$val; fi ;;
    esac
  done < <(grep -vE '^[[:space:]]*(#|$)' "$RULES")
  echo "$sp"
}
list_bundles() {   # "묶음 폴더 split" 한 줄씩
  local src b
  for src in ${FD_XM_SRC:-raw_v1 nav2_v1}; do ls -d data/teacher/$src/*/; done | while read -r b; do
    echo "$b $(split_of "$b")"
  done
}
if [ "${FD_XM_LIST:-0}" = 1 ]; then list_bundles; exit 0; fi
set +u; source /opt/ros/jazzy/setup.bash; set -u   # ROS setup 은 정의 안 된 변수를 읽음
[ -e "$OUT" ] && { echo "이미 있음: $OUT (지우거나 다른 이름으로)"; exit 1; }
mkdir -p "$OUT"
list_bundles | xargs -P "$JOBS" -L 1 bash -c '"$0" sim/export_xmobility.py "$1" "'"$OUT"'" --split "$2" '"${FD_XM_ARGS:-}" "$PY" \
  2>&1 | tee "$OUT/export.log"
"$PY" - "$OUT" <<'EOF'
import glob, json, os, sys
out = sys.argv[1]
rows = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(out, "index", "*.json")))]
tot = {}
for r in rows:
    k = (r["split"], r["scenario"])
    tot[k] = [tot.get(k, [0, 0, 0])[i] + v for i, v in enumerate((1, r["runs"], r["samples"]))]
print(f"묶음 {len(rows)} · 샘플 {sum(r['samples'] for r in rows)}")
for (sp, sc), (nb, nr, ns) in sorted(tot.items()):
    print(f"  {sp:5s} {sc:12s} 묶음 {nb:2d} · run {nr:3d} · 샘플 {ns:6d} ({ns / 5 / 60:.1f}분)")
EOF
