#!/usr/bin/env bash
# 교사 데이터 → X-Mobility 학습 형식 (sim/export_xmobility.py, 묶음마다 하나씩 병렬).
# 원본 폴더 = FD_XM_SRC(기본 "raw_v1 nav2_v1" = xm_v1). xm_v2(11.05) = FD_XM_SRC="raw_v1 nav2_v1 nav2_v2".
# 나눔(묶음 단위): raw_v1 은 마지막 반시계 묶음 val · 마지막 시계 묶음 test,
#                  nav2_v1 은 콘(반시계) val · 슬라럼(시계) test — 같은 시나리오 반대 방향은 train 에 남김. 나머지 train.
#                  nav2_v2(무작위 장애물 배치)는 rand_tr_18 양방향 val · rand_tr_19 양방향 test.
# 사용: [FD_XM_SRC="…"] sim/export_xmobility.sh [출력 폴더=data/teacher/xm_v1] [병렬=8]
set -euo pipefail
cd "$(dirname "$0")/.."
OUT=${1:-data/teacher/xm_v1}
JOBS=${2:-8}
PY=${FD_XM_PY:-$HOME/xmobility_test/venv/bin/python}   # pandas · pyarrow 있는 python 3.12
set +u; source /opt/ros/jazzy/setup.bash; set -u   # ROS setup 은 정의 안 된 변수를 읽음

declare -A SPLIT=(
  [1013_121103_auto_ccw]=val [1013_122008_auto_cw]=test
  [1014_123928_nav2_ccw]=val [1014_115111_nav2_cw]=test
)
[ -e "$OUT" ] && { echo "이미 있음: $OUT (지우거나 다른 이름으로)"; exit 1; }
mkdir -p "$OUT"
for src in ${FD_XM_SRC:-raw_v1 nav2_v1}; do ls -d data/teacher/$src/*/; done | while read -r b; do
  n=$(basename "$b"); sp=${SPLIT[$n]:-train}
  grep -q "rand_tr_18" "$b/meta.yaml" 2>/dev/null && sp=val
  grep -q "rand_tr_19" "$b/meta.yaml" 2>/dev/null && sp=test
  echo "$b $sp"
done | xargs -P "$JOBS" -L 1 bash -c '"$0" sim/export_xmobility.py "$1" "'"$OUT"'" --split "$2"' "$PY" \
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
