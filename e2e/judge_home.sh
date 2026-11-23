#!/usr/bin/env bash
# 집 PC run 이 Kaggle(fd-xm-home-<run>)에 올라오면 노트북에서 한 줄로 받기 → 판정.
#   bash judge_home.sh xm_ft_v0.1_aw30_pad05_e10      # E1 → 결정 1(judge_d1.py, v1.0 test, aw30 기준)
#   bash judge_home.sh xm_ft_v1.4_aw30_exec_e10       # E4 → 장애물 진단(obstacle_diag, v1.4 test) + v1.4 test 지표 vs aw30
# ckpt(1 GB)는 안 받음 — 결과 파일만. 출력: ~/Downloads/fd_home/<run>/ 와 그 안의 judge.md
set -euo pipefail
RUN=${1:?run 이름(예 xm_ft_v1.4_aw30_exec_e10)}
REPO=${REPO:-$(cd "$(dirname "$0")/.." && pwd)}
DL=${DL:-~/Downloads}
# 경로는 환경 변수로 바꿀 수 있음. 집 PC: BASE=<fd-xm-baselines 푼 폴더> V14=/c/fd/data/xm_v1_4 bash e2e/judge_home.sh <run>
#   (BASE 를 주면 aw30 기준선을 그 안의 v10_eval_aw30_test · v14_eval_aw30_test 에서 읽음, Kaggle henryseo711/fd-xm-baselines)
if [ -n "${BASE:-}" ]; then
  AW30_V10=${AW30_V10:-$BASE/v10_eval_aw30_test/frames.csv}; AW30_V14=${AW30_V14:-$BASE/v14_eval_aw30_test}
fi
AW30_V10=${AW30_V10:-$DL/xm_ft_aw30_kaggle/xm_ft_v0.1_aw30_e10/eval_test/frames.csv}   # aw30 v1.0 test(결정 1 기준)
AW30_V14=${AW30_V14:-$DL/xm_eval_kaggle_v7_xm_v3/eval_aw30_test}                       # aw30 v1.4 test(장애물 0.18 · w_mae 0.082)
V14=${V14:-$DL/fd_d2/up/xm_v1_4}                                                       # obstacle_diag --dataset(index/ 있는 곳)
AW30_V15=${AW30_V15:-$DL/xm_eval_kaggle_v15_base/eval_aw30_test}                       # aw30 v1.5 test(build_v15.sh 평가 단계 출력)
V15=${V15:-$DL/fd_d3/up/xm_v1_5}
#   v1.5 run(E6): bash judge_home.sh xm_ft_v1.5_aw30_exec_e6 — 같은 판정을 v1.5 test 로
export PYTHONIOENCODING=utf-8

# home_after_train.ps1 와 같은 slug 규칙
SLUG=$(python -c "import re,sys;s='fd-xm-home-'+re.sub('[^a-z0-9]+','-',sys.argv[1].lower());s=s.strip('-');print(s[:50].strip('-'))" "$RUN")
OUT=$DL/fd_home/$RUN; mkdir -p "$OUT"
echo "받기: henryseo711/$SLUG → $OUT"
if [ -n "${LOCAL_RUN:-}" ]; then   # 집 PC: 학습 출력 폴더에서 바로(home_after_train.ps1 이 만든 eval_test 포함)
  cp "$LOCAL_RUN/run_meta.json" "$OUT/run_meta.json"
  cp "$LOCAL_RUN/eval_test/frames.csv" "$OUT/eval_test_frames.csv"
  cp "$LOCAL_RUN/eval_test/summary.json" "$OUT/eval_test_summary.json"
  SKIP_DL=1
fi
if [ "${SKIP_DL:-0}" != 1 ]; then   # SKIP_DL=1: 이미 받은 $OUT 로 판정만
python -m kaggle datasets files "henryseo711/$SLUG" || { echo "데이터셋이 아직 없음 — 집 PC 에서 home_after_train.ps1 를 돌렸는지 확인"; exit 1; }
for f in run_meta.json eval_test_summary.json eval_test_frames.csv curves.md report_tables.md; do
  python -m kaggle datasets download "henryseo711/$SLUG" -f "$f" -p "$OUT" --force -q || echo "  (없음: $f)"
  if [ -f "$OUT/$f.zip" ]; then (cd "$OUT" && python -c "import zipfile,sys;zipfile.ZipFile(sys.argv[1]).extractall('.')" "$f.zip" && rm "$f.zip"); fi
done
fi
FR=$OUT/eval_test_frames.csv; [ -f "$FR" ] || { echo "eval_test_frames.csv 없음 — 집 PC 평가가 안 됐음"; exit 1; }

python - "$OUT/run_meta.json" <<'EOF'
import json, sys
m = json.load(open(sys.argv[1], encoding="utf-8"))
keys = ["best_model_path", "best_score", "global_step", "steps_per_epoch", "step_time_s", "seed",
        "prev_action_dropout", "prev_action_source", "exec_stats"]
print("run_meta:", json.dumps({k: m.get(k) for k in keys}, ensure_ascii=False, default=str))
EOF

case "$RUN" in
  *v0.1*pad*)   # ---- E1: 결정 1
    python - "$OUT/run_meta.json" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1], encoding="utf-8")).get("prev_action_dropout")
assert d == 0.5, f"prev_action_dropout = {d} (0.5 여야 E1)"
EOF
    python "$REPO/e2e/judge_d1.py" --base aw30="$AW30_V10" E1="$FR" -o "$OUT/judge.md"
    ;;
  *v1.4*exec*|*v1.5*exec*)   # ---- E4 · E5: 장애물 진단 + v1.4 test 지표
    python - "$OUT/run_meta.json" <<'EOF'
import json, sys
m = json.load(open(sys.argv[1], encoding="utf-8"))
assert m.get("prev_action_source") == "exec", f"prev_action_source = {m.get('prev_action_source')}"
es = m.get("exec_stats") or {}
assert es.get("exec_ne_teacher_rows", 0) > 0, f"실행 ≠ 교사 행 0 — 옵션이 효과 없음: {es}"
EOF
    BASE=$AW30_V14; DSET=$V14; TV="v1.4"
    case "$RUN" in *v1.5*) BASE=$AW30_V15; DSET=$V15; TV="v1.5"
      [ -f "$BASE/summary.json" ] || { echo "aw30 v1.5 기준선 없음($BASE) — 먼저 build_v15.sh 의 평가 단계"; exit 1; };; esac
    python "$REPO/e2e/obstacle_diag.py" --dataset "$DSET" aw30="$BASE/frames.csv" "${RUN}=$FR" -o "$OUT/obstacle_diag.md"
    python - "$BASE/summary.json" "$OUT/eval_test_summary.json" "$RUN" "$OUT/judge.md" "$TV" <<'EOF'
import json, sys
a, b = (json.load(open(p, encoding="utf-8")) for p in sys.argv[1:3])
run, out, tv = sys.argv[3], sys.argv[4], sys.argv[5]
def g(s, *path):
    for k in path:
        s = (s or {}).get(k)
    return s
rows = [("전체 w_mae", ("overall", "w_mae"), "↓"), ("전체 ω 부호", ("overall", "w_sign_agree"), "↑"),
        ("자동 교사 w_mae", ("by_scenario", "r3_auto", "w_mae"), "↓ 유지"), ("Nav2 장애물 w_mae", ("by_scenario", "r3_nav2_obs", "w_mae"), "↓"),
        ("Nav2 장애물 ω 부호", ("by_scenario", "r3_nav2_obs", "w_sign_agree"), "↑"), ("장애물 1.5 m 안 w_mae", ("near_obstacle", "w_mae"), "↓"),
        ("경로 ADE m", ("overall", "ade_m"), "↓"),
        ("v0.8 r3 자동 w_mae", ("by_scenario", "v08r3_auto", "w_mae"), "↓ (v1.5 만)"),
        ("v0.8 r3 자동 ω 부호", ("by_scenario", "v08r3_auto", "w_sign_agree"), "↑ (v1.5 만)")]
L = [f"# {run} vs aw30 — {tv} test 기본 평가(직전 예측 입력)\n", f"| 지표 | aw30 | {run} | 차 | 좋은 쪽 |", "|---|---|---|---|---|"]
for name, path, good in rows:
    x, y = g(a, *path), g(b, *path)
    L.append(f"| {name} | {x:.4f} | {y:.4f} | {y - x:+.4f} | {good} |" if None not in (x, y) else f"| {name} | {x} | {y} | - | {good} |")
L.append(f"\n프레임 수 aw30 {g(a, 'overall', 'n')} · {run} {g(b, 'overall', 'n')} (같아야 같은 test)")
L.append("\n장애물 진단 = obstacle_diag.md — 기준: '회피 중 · 중앙' 크기 비 aw30 0.18(test). 오르면 되먹임이 줄어든 것.")
open(out, "w", encoding="utf-8").write("\n".join(L) + "\n")
print("\n".join(L))
EOF
    ;;
  *) echo "모르는 run 이름 패턴 — E1(v0.1…pad) 또는 E4 · E5 · E6(v1.4 · v1.5 …exec) 만"; exit 2;;
esac
echo "결과: $OUT"
