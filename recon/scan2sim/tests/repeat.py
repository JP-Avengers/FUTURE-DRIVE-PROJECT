"""같은 입력에 같은 결과가 나오는지 — 엔진을 두 번 돌려 결함 목록과 주요 수치를 견준다.

    cd recon
    python -m scan2sim.tests.repeat <입력> [--runs 2] [--tol 0.1] [--work <폴더>] [--json 결과.json]

견주는 것: 결함 목록(보정 전 · 뒤) — 종류별 개수와, 결함마다 같은 종류 중 가장 가까운 것과의 거리(바닥 평면, --tol m 안이면 같음)
           단계별 판정 · diagnose 의 수치(바닥 한 겹 % · 두 겹 표본 수 · 바닥 아래 면 · 보이는 바닥 없음 %)
종료 코드: 0 = 같음 · 1 = 다름 · 2 = 엔진 실패
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

RECON = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
METRICS = ("floor_single_pct", "floor_double", "floor_under", "visible_floor_missing_pct")


def run_once(inp, out):
    t = time.time()
    with open(out + ".txt", "w", encoding="utf-8") as log:
        rc = subprocess.run([sys.executable, "-m", "scan2sim", inp, "--out", out], cwd=RECON, stdout=log, stderr=subprocess.STDOUT,
                            env=dict(os.environ, PYTHONIOENCODING="utf-8")).returncode
    res = dict(rc=rc, seconds=round(time.time() - t, 1), stages={}, defects={}, metrics={})
    sd = os.path.join(out, "stages")
    for f in sorted(os.listdir(sd)) if os.path.isdir(sd) else []:
        d = json.load(open(os.path.join(sd, f), encoding="utf-8"))
        res["stages"][d["stage"]] = d["status"]
        if d["stage"] == "diagnose":
            res["metrics"] = {k: d["metrics"].get(k) for k in METRICS if k in d["metrics"]}
    for ph in ("before", "after"):
        p = os.path.join(out, f"defects_{ph}.json")
        if os.path.exists(p):
            res["defects"][ph] = [dict(type=d["type"], xy=d["center_scene"][:2], samples=d["samples"]) for d in json.load(open(p, encoding="utf-8"))["defects"]]
    return res


def unmatched(A, B, tol):
    """A 의 결함 중 B 에 같은 종류가 tol 안에 없는 것"""
    return [a for a in A if not any(b["type"] == a["type"] and math.dist(a["xy"], b["xy"]) <= tol for b in B)]


def compare(r0, r1, tol):
    diffs = []
    if r0["stages"] != r1["stages"]:
        diffs.append(f"단계 판정이 다름: {r0['stages']} / {r1['stages']}")
    for k in METRICS:
        if r0["metrics"].get(k) != r1["metrics"].get(k):
            diffs.append(f"diagnose {k}: {r0['metrics'].get(k)} / {r1['metrics'].get(k)}")
    for ph in sorted(set(r0["defects"]) | set(r1["defects"])):
        A, B = r0["defects"].get(ph), r1["defects"].get(ph)
        if A is None or B is None:
            diffs.append(f"결함 목록({ph})이 한쪽에만 있음")
            continue
        for x, y, tag in ((A, B, "첫째에만"), (B, A, "둘째에만")):
            for d in unmatched(x, y, tol):
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
    work = a.work or tempfile.mkdtemp(prefix="scan2sim_repeat_")
    os.makedirs(work, exist_ok=True)
    inp = os.path.abspath(a.input)
    print(f"repeat: {inp} × {a.runs}", flush=True)
    runs = []
    for k in range(a.runs):
        out = os.path.join(work, f"run{k}")
        shutil.rmtree(out, ignore_errors=True)
        r = run_once(inp, out)
        print(f"  {k + 1}회: 종료 코드 {r['rc']} · {r['seconds']}초 · " + " · ".join(f"{ph} {len(v)}곳" for ph, v in r["defects"].items()) + f" · {r['metrics']}", flush=True)
        runs.append(r)
    if any(r["rc"] not in (0, 1) or not r["stages"] for r in runs):
        print("엔진이 끝까지 돌지 않음", flush=True)
        sys.exit(2)
    diffs = [f"{k + 1}회 ↔ 1회: {d}" for k in range(1, len(runs)) for d in compare(runs[0], runs[k], a.tol)]
    for d in diffs:
        print("  다름 —", d, flush=True)
    print(f"repeat {'같음' if not diffs else '다름'} — {a.runs}회", flush=True)
    if a.json:
        json.dump(dict(input=inp, runs=runs, diffs=diffs, same=not diffs), open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    if not a.work:
        shutil.rmtree(work, ignore_errors=True)
    sys.exit(0 if not diffs else 1)


if __name__ == "__main__":
    main()
