"""같은 입력에 같은 결과가 나오는지 — 엔진을 여러 번 돌려 결함 목록과 주요 수치를 견준다.

    cd recon
    python -m scan2sim.tests.repeat <입력> [--runs 2] [--tol 0.1] [--work <폴더>] [--json 결과.json]

견주는 것 (첫 회와 나머지 각 회)
  - 단계별 판정
  - diagnose 의 수치: 바닥 한 겹 % · 두 겹 표본 수 · 바닥 아래 면 · 보이는 바닥 없음 %. 보정했으면 correct 의 보정 뒤 수치도
  - 결함 목록(보정 전 · 뒤): 종류별 개수, 그리고 같은 종류끼리 거리 순으로 1:1 짝지어 --tol m 안인지
    (결함 목록은 엔진이 30개에서 자르므로, 후보가 많은 장면은 끝자리 결함이 바뀌어 다르게 나올 수 있음)
종료 코드: 0 = 같음 · 1 = 다름 · 2 = 엔진이 끝까지 못 감(report 단계 없음)
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
from collections import Counter

RECON = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
METRICS = ("floor_single_pct", "floor_double", "floor_under", "visible_floor_missing_pct")


def run_once(inp, out):
    t = time.time()
    with open(out + ".txt", "w", encoding="utf-8") as log:
        rc = subprocess.run([sys.executable, "-m", "scan2sim", inp, "--out", out], cwd=RECON, stdout=log, stderr=subprocess.STDOUT,
                            env=dict(os.environ, PYTHONIOENCODING="utf-8")).returncode
    res = dict(rc=rc, seconds=round(time.time() - t, 1), stages={}, defects={}, metrics={}, after={})
    sd = os.path.join(out, "stages")
    for f in sorted(os.listdir(sd)) if os.path.isdir(sd) else []:
        try:
            d = json.load(open(os.path.join(sd, f), encoding="utf-8"))
        except Exception:
            continue
        res["stages"][d["stage"]] = d["status"]
        if d["stage"] == "diagnose":
            res["metrics"] = {k: d["metrics"].get(k) for k in METRICS if k in d["metrics"]}
        if d["stage"] == "correct" and isinstance(d["metrics"].get("after"), dict):
            res["after"] = {k: d["metrics"]["after"].get(k) for k in METRICS if k in d["metrics"]["after"]}
    for ph in ("before", "after"):
        p = os.path.join(out, f"defects_{ph}.json")
        if os.path.exists(p):
            res["defects"][ph] = [dict(type=d["type"], xy=d["center_scene"][:2], samples=d["samples"]) for d in json.load(open(p, encoding="utf-8"))["defects"]]
    return res


def match(A, B, tol):
    """같은 종류끼리 거리 순으로 1:1 짝지음 → (A 에서 짝 없는 것, B 에서 짝 없는 것)"""
    pairs = sorted((math.dist(a["xy"], b["xy"]), i, k) for i, a in enumerate(A) for k, b in enumerate(B) if a["type"] == b["type"])
    ua, ub = set(), set()
    for d, i, k in pairs:
        if d <= tol and i not in ua and k not in ub:
            ua.add(i), ub.add(k)
    return [a for i, a in enumerate(A) if i not in ua], [b for k, b in enumerate(B) if k not in ub]


def compare(r0, r1, tol):
    diffs = []
    if r0["stages"] != r1["stages"]:
        diffs.append(f"단계 판정이 다름: {r0['stages']} / {r1['stages']}")
    for key, name in (("metrics", "diagnose"), ("after", "보정 뒤")):
        for k in METRICS:
            if r0[key].get(k) != r1[key].get(k):
                diffs.append(f"{name} {k}: {r0[key].get(k)} / {r1[key].get(k)}")
    for ph in sorted(set(r0["defects"]) | set(r1["defects"])):
        A, B = r0["defects"].get(ph), r1["defects"].get(ph)
        if A is None or B is None:
            diffs.append(f"결함 목록({ph})이 한쪽에만 있음")
            continue
        ca, cb = Counter(d["type"] for d in A), Counter(d["type"] for d in B)
        if ca != cb:
            diffs.append(f"{ph} 종류별 개수: {dict(ca)} / {dict(cb)}")
        only_a, only_b = match(A, B, tol)
        for tag, L in (("첫째에만", only_a), ("둘째에만", only_b)):
            for d in L:
                diffs.append(f"{ph} {tag}: {d['type']} ({d['xy'][0]:.2f}, {d['xy'][1]:.2f}) 표본 {d['samples']}")
    return diffs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("--runs", type=int, default=2)
    ap.add_argument("--tol", type=float, default=0.1)
    ap.add_argument("--work", help="출력 폴더를 둘 곳 (안 주면 임시 폴더를 만들고 끝나면 지움)")
    ap.add_argument("--json")
    a = ap.parse_args()
    if a.runs < 2:
        ap.error("--runs 는 2 이상")
    work = a.work or tempfile.mkdtemp(prefix="scan2sim_repeat_")
    os.makedirs(work, exist_ok=True)
    inp = os.path.abspath(a.input)
    print(f"repeat: {inp} × {a.runs}", flush=True)
    runs, diffs, rc = [], [], 0
    try:
        for k in range(a.runs):
            out = os.path.join(work, f"run{k}")
            shutil.rmtree(out, ignore_errors=True)
            r = run_once(inp, out)
            print(f"  {k + 1}회: 종료 코드 {r['rc']} · {r['seconds']}초 · " + " · ".join(f"{ph} {len(v)}곳" for ph, v in r["defects"].items()) + f" · {r['metrics']}", flush=True)
            runs.append(r)
            if "report" not in r["stages"]:                    # 엔진이 단계 기록 없이 죽음 (예외 · SystemExit)
                print(f"  엔진이 끝까지 못 감 — {out}.txt", flush=True)
                rc = 2
                break
        if rc == 0:
            diffs = [f"{k + 1}회 ↔ 1회: {d}" for k in range(1, len(runs)) for d in compare(runs[0], runs[k], a.tol)]
            for d in diffs:
                print("  다름 —", d, flush=True)
            print(f"repeat {'같음' if not diffs else '다름'} — {a.runs}회", flush=True)
            rc = 0 if not diffs else 1
    finally:
        if a.json:
            json.dump(dict(input=inp, runs=runs, diffs=diffs, same=rc == 0, exit=rc), open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        if not a.work:
            shutil.rmtree(work, ignore_errors=True)
    sys.exit(rc)


if __name__ == "__main__":
    main()
