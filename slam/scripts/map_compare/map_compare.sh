#!/bin/bash
# 맵 비교 한 번에: (1) in1r 강체 배치 자세(없을 때만) (2) 남의 메시(Polycam 등)를 건물 좌표로 맞춤 (3) v0.2 · v0.3 · 남의 메시를 같은 잣대로 → <out>/compare_table.md
# 사용: map_compare.sh <out_dir> [<foreign.glb|ply|obj> [이름] [mc_register.py 옵션 ...]]
#   예: map_compare.sh ~/b_slam/agents/compare/polycam ~/data/polycam/corridor.glb polycam
#       map_compare.sh ~/b_slam/agents/compare/polycam ~/data/polycam/corridor.glb polycam --apply-scale   (크기 비까지 적용)
#       map_compare.sh ~/b_slam/agents/compare/polycam ~/data/polycam/corridor.glb polycam --up +y         (위 축 자동 판정이 틀릴 때)
#   기준 줄 바꾸기: BASE="v0.3=$HOME/b_slam/deliver/corridor_v0.3/geometry_tsdf_2cm.ply" map_compare.sh ...
#   무거운 단계(맞춤 · 잣대)는 ~/b_slam/.heavy.lock 을 잡고 돈다. 같은 메시 결과는 ~/b_slam/agents/compare/mc_cache 에 남아 다시 안 잰다.
set -e
D=$(cd "$(dirname "$0")" && pwd); PY="$HOME/tsdf_env/bin/python -I"; LOCK=$HOME/b_slam/.heavy.lock
OUT=$1; shift; mkdir -p "$OUT"
RIG=${RIG:-$HOME/b_slam/agents/compare/rigidval}
[ -f "$RIG/in1r_rigid4.txt" ] || $PY "$D/mc_rigidval.py" in1r in1rL "$RIG"
BASE=${BASE:-"v0.2=$HOME/b_slam/deliver/corridor_v0.2/geometry_tsdf_2cm.ply v0.3=$HOME/b_slam/deliver/corridor_v0.3/geometry_tsdf_2cm.ply"}
EXTRA=()
if [ -n "$1" ]; then
  F=$1; shift; N=polycam
  if [ -n "$1" ] && [[ "$1" != --* ]]; then N=$1; shift; fi
  flock "$LOCK" $PY "$D/mc_register.py" "$F" "$OUT" --name "$N" "$@" 2>&1 | grep -v 'Open3D WARNING'
  EXTRA=("$N=$OUT/${N}_reg.ply" --reg "$N=$OUT/${N}_reg.json")
fi
flock "$LOCK" $PY "$D/map_compare.py" "$OUT" $BASE "${EXTRA[@]}" --rigid "$RIG"
echo "표: $OUT/compare_table.md  그림: $OUT/{topviews.png,robot_views.jpg,rigidval_bins.png}"
