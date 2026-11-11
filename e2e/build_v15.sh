#!/usr/bin/env bash
# v1.5 한 줄 (노트북 · Git Bash): D 의 v0.8 r3 zip 들 → v1.5(= v1.4 + v0.8 r3 자동 교사 45바퀴 + v0.8 r3 Nav2 장애물 27회) → Kaggle dataset-08-xm-v1-5
#   → 기준선 평가 커널(aw30 · 제로샷 × v1.5 test · val) push → 끝나면 결과를 받아 v08r3_* 칸 요약.
#   bash e2e/build_v15.sh ~/Downloads/<xm_v08r3_v1 zip> ~/Downloads/<xm_v08r3_nav2_v1 zip>   # 순서 상관없음, 하나만 와도 됨
#   NO_EVAL=1 bash e2e/build_v15.sh <zip> ...           # 데이터셋까지만(평가 커널은 안 돌림 — 할당량 아낄 때)
# 단계마다 STEP 줄, 실패면 FAIL 줄로 멈춤. 다시 돌리면 끝난 단계는 건너뜀(zip 구성이 바뀌면 ~/Downloads/fd_d3 를 지우고)
# 기준 수치(MANIFEST 11.11): xm_v08r3_v1 = 45파일 · train 28,548 · val 4,001 · test 4,157 · sha12 418ae8d2dc1b
#                           xm_v08r3_nav2_v1 = 27파일 · train 15,551 · val 1,290 · test 643 · sha12 20810dc21e85
set -uo pipefail
[ $# -ge 1 ] || { echo "D 의 v0.8 r3 zip 경로(1–2개)"; exit 2; }
REPO=$(cd "$(dirname "$0")/.." && pwd)
DL=${DL:-$HOME/Downloads}; W=${W:-$DL/fd_d3}; UP=$W/up; mkdir -p "$W" "$UP"   # 집 PC: W=/c/fd/data/fd_d3 V14=/c/fd/data/xm_v1_4
V14=$(cygpath -m "${V14:-$DL/fd_d2/up/xm_v1_4}")
DS=henryseo711/dataset-08-xm-v1-5
KDIR=$DL/kaggle_runs_11_14/eval_v15_base
OUT=$DL/xm_eval_kaggle_v15_base
export PYTHONIOENCODING=utf-8
fail() { echo "FAIL $*"; exit 1; }
cd "$REPO" || fail "레포 없음"
[ -f "$V14/RESPLIT.json" ] || fail "v1.4 없음: $V14"

# 1. 풀기 — zip 마다 폴더 하나, 시나리오 폴더 이름으로 무엇인지 확인
ADDS=(); WANT_TRAIN=68441; k=0
for ZIP in "$@"; do
  k=$((k + 1)); U=$W/add$k
  if [ ! -d "$U" ]; then
    rm -rf "$U.tmp"; mkdir -p "$U.tmp"
    python -c "import zipfile,sys;zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$ZIP" "$U.tmp" || fail "unzip $ZIP"
    mv "$U.tmp" "$U"
  fi
  t=$(find "$U" -maxdepth 4 -type d -name train | head -1); [ -n "$t" ] || fail "train 폴더 없음: $ZIP"
  R=$(cygpath -m "$(dirname "$t")")
  n=$(python - "$R" <<'EOF'
import glob, os, sys
import pyarrow.parquet as pq
r = sys.argv[1]
want = {"v08r3_auto": 45, "v08r3_nav2_obs": 27}      # MANIFEST 11.11
sc = sorted({os.path.basename(os.path.dirname(p)) for p in glob.glob(f"{r}/*/*/*.pqt")})
nf = len(glob.glob(f"{r}/*/*/*.pqt"))
assert len(sc) == 1 and sc[0] in want, f"모르는 시나리오 {sc} — v08r3_auto 또는 v08r3_nav2_obs 여야 함"
assert nf == want[sc[0]], f"{sc[0]}: {nf}파일(MANIFEST {want[sc[0]]})"
tr = sum(pq.ParquetFile(f).metadata.num_rows for f in glob.glob(f"{r}/train/*/*.pqt"))
print(sc[0], nf, "파일 · train", tr, "행", file=sys.stderr)
print(tr)
EOF
) || fail "D 데이터가 MANIFEST 와 다름: $ZIP"
  ADDS+=(--add "$R"); WANT_TRAIN=$((WANT_TRAIN + n))
done
echo "STEP unzip ok ${#ADDS[@]} 인자 · 기대 train $WANT_TRAIN 행"

# 2. v1.5 만들기 · 점검
DST=$(cygpath -m "$UP")/xm_v1_5
if [ ! -f "$DST/RESPLIT.json" ]; then
  rm -rf "$DST"
  python e2e/resplit_xm.py "$V14" "$DST" "${ADDS[@]}" --allow-new-scenario --check > "$W/resplit.log" 2>&1
  rc=$?; tail -30 "$W/resplit.log"
  [ $rc -eq 0 ] || fail "resplit rc=$rc ($W/resplit.log)"
fi
python - "$DST" "$WANT_TRAIN" <<'EOF' || fail "v1.5 점검"
import glob, sys
import pyarrow.parquet as pq
r, want = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 68441 + 28548 + 15551
rows, lack = {}, []
for s in ("train", "val", "test"):
    rows[s] = 0
    for f in glob.glob(f"{r}/{s}/*/*.pqt"):
        m = pq.ParquetFile(f)
        rows[s] += m.metadata.num_rows
        if "exec_command" not in m.schema_arrow.names:
            lack.append(f)
print("행 수", rows, "· exec_command 없는 파일", len(lack))
assert not lack, f"exec_command 없는 파일 {len(lack)}개 — --prev-action-source exec 학습이 멈춤: {lack[:3]}"
assert rows["train"] == want, f"train {rows['train']} ≠ {want}"
EOF
echo "STEP v1.5 ok $(du -sh "$DST" | cut -f1)"

# 3. Kaggle 데이터셋
cat > "$UP/dataset-metadata.json" <<EOF
{"title": "dataset-08-xm-v1-5", "id": "$DS", "licenses": [{"name": "CC0-1.0"}]}
EOF
if ! python -m kaggle datasets status $DS 2>/dev/null | grep -qi ready; then
  python -m kaggle datasets create -p "$(cygpath -w "$UP")" -r tar > "$W/upload.log" 2>&1 || { tail -5 "$W/upload.log"; fail "dataset create"; }
  tail -3 "$W/upload.log"
fi
for i in $(seq 1 90); do
  st=$(python -m kaggle datasets status $DS 2>&1 | tail -1)
  echo "$st" | grep -qi ready && break
  echo "$st" | grep -qiE "error|fail" && fail "dataset status: $st"
  sleep 20
done
echo "STEP dataset ready: $st"
echo "MANIFEST 줄 초안은 $W/resplit.log 끝에 있음"
[ "${NO_EVAL:-0}" = 1 ] && { echo "DONE(데이터셋까지) — 학습: python e2e/gen_kaggle_11_14.py 뒤 v15_* 폴더 push"; exit 0; }

# 4. 기준선 평가 커널(aw30 · 제로샷 × test · val) — 레포 kaggle_eval.ipynb 에서 칸 1 만 바꿈
mkdir -p "$KDIR"
python - "$KDIR" "$DS" <<'EOF' || fail "평가 노트북 만들기"
import json, re, sys
kdir, ds = sys.argv[1:3]
nb = json.load(open("e2e/kaggle_eval.ipynb", encoding="utf-8"))
c = [c for c in nb["cells"] if c["cell_type"] == "code" and "CKPTS = {" in "".join(c["source"])]
assert len(c) == 1
s = "".join(c[0]["source"])
new = {
    r"^CKPTS = \{.*?^\}\n": 'CKPTS = {                                  # 11.11 v1.5 기준선(v08r3_auto 칸이 처음 생김)\n'
                            '    "aw30": "xm_ft_v0.1_aw30_e8.ckpt",     # 4fdf249cda0a\n'
                            '    "zs": "x_mobility-nav2-semantic_action_path.ckpt",   # 공개 ckpt(fd-xm-pretrained)\n}\n',
    r"^SOUPS = \{.*?^\}\n": "SOUPS = {}\n",
    r"^EVAL_TAGS = .*$": 'EVAL_TAGS = ["aw30", "zs"]',
    r"^SPLITS = .*$": 'SPLITS = ["test", "val"]',
}
for pat, rep in new.items():
    p = re.compile(pat, re.M | (re.S if "{" in pat else 0))   # 블록(CKPTS · SOUPS)만 여러 줄, 한 줄 패턴은 줄 끝까지
    assert len(p.findall(s)) == 1, pat
    s = p.sub(lambda m: rep, s, count=1)
c[0]["source"] = s.splitlines(keepends=True)
for i, cc in enumerate(nb["cells"]):
    if cc["cell_type"] == "code":
        compile("".join(cc["source"]), f"cell{i}", "exec")
json.dump(nb, open(f"{kdir}/fd-xm-eval.ipynb", "w", encoding="utf-8", newline="\n"), ensure_ascii=False, indent=1)
meta = {"id": "henryseo711/fd-xm-eval", "title": "fd-xm-eval", "code_file": "fd-xm-eval.ipynb", "language": "python",
        "kernel_type": "notebook", "is_private": True, "enable_gpu": True, "enable_tpu": False, "enable_internet": True,
        "keywords": ["gpu"], "dataset_sources": [ds, "henryseo711/fd-xm-ckpts", "henryseo711/fd-xm-pretrained"],
        "kernel_sources": [], "competition_sources": [], "model_sources": [], "machine_shape": "NvidiaTeslaT4"}
json.dump(meta, open(f"{kdir}/kernel-metadata.json", "w", encoding="utf-8"), indent=2)
print(s)
EOF
python -m kaggle kernels push -p "$(cygpath -w "$KDIR")" 2>&1 | tail -3
echo "STEP eval kernel pushed"
for i in $(seq 1 180); do
  st=$(python -m kaggle kernels status henryseo711/fd-xm-eval 2>&1 | tail -1)
  echo "$st" | grep -qiE "complete|error|cancel" && break
  sleep 30
done
echo "STEP kernel status: $st"
mkdir -p "$OUT" && python -m kaggle kernels output henryseo711/fd-xm-eval -p "$(cygpath -w "$OUT")" > "$W/output.log" 2>&1
python - "$OUT" <<'EOF'
import json, os, sys
o = sys.argv[1]
print("| 태그 | split | 전체 w_mae | r3_auto w_mae | r3_nav2_obs w_mae · 부호 | v08r3_auto w_mae · 부호 | v08r3_nav2_obs w_mae · 부호 |")
print("|---|---|---|---|---|---|---|")
for t in ("aw30", "zs"):
    for sp in ("test", "val"):
        f = f"{o}/eval_{t}_{sp}/summary.json"
        if not os.path.exists(f):
            print(f"| {t} | {sp} | 없음 | | | |"); continue
        s = json.load(open(f, encoding="utf-8")); b = s["by_scenario"]
        g = lambda k, m: (b.get(k) or {}).get(m)
        fmt = lambda x: "-" if x is None else f"{x:.3f}"
        print(f"| {t} | {sp} | {s['overall']['w_mae']:.3f} | {fmt(g('r3_auto','w_mae'))} | "
              f"{fmt(g('r3_nav2_obs','w_mae'))} · {fmt(g('r3_nav2_obs','w_sign_agree'))} | "
              f"{fmt(g('v08r3_auto','w_mae'))} · {fmt(g('v08r3_auto','w_sign_agree'))} | "
              f"{fmt(g('v08r3_nav2_obs','w_mae'))} · {fmt(g('v08r3_nav2_obs','w_sign_agree'))} |")
EOF
echo "DONE out=$OUT"
