#!/usr/bin/env python3
"""폐루프 묶음 비교 — 회마다 추종 · 속도 · 최대 이탈 자리, 장애물마다 지나가기 요약을 묶음끼리 한 표로 (E, 11.15)

왜    analyze_runs.py 는 회 결과(통과 · 이탈 · 충돌)만, obstacle_pass.py 는 장애물 행만 냄. 11.14 무작위 배치에서 E6 · E7 을 가른 것은
      장애물 사이의 추종(장애물 밖 이탈)과 속도였는데(e2e/results/closed_loop_rand_ev_11_14.md) 그 계산은 레포 밖이었음 →
      그 표를 이 레포에서 다시 내고, 60회 결과에도 같은 식으로 바로 쓰려고.
묶음        LABEL=글롭 — 같은 LABEL 을 여러 번 주면 합침(예: 같은 모델의 두 날). 회 JSON = xm_route.py · nav2_route.py 출력(track 있음).
회마다      구간 = t_goal(없으면 track 끝 − lap_sim_s) ~ t_end_decision(없으면 track 끝)
  장애물 밖 이탈   그 구간 표본 중 모든 장애물에서 진행 방향 앞 10 m ~ 뒤 6 m 밖인 표본의 cte 평균 · 최대
  속도            명령 = track v_cmd 평균(xm 회만, Nav2 track 에는 없음) · 실제 = 위치 차분 평균
  최대 이탈 자리   cte 가 가장 큰 표본에서 가장 가까운 장애물까지 진행 방향 거리(+ = 아직 앞)
장애물마다   obstacle_pass.analyze_run 그대로. 늦은 꺾기 = 꺾기 시작 < 2 m(꺾기 시작이 잡힌 장애물 중).
묶음 요약    통과(Wilson 95 %) · 최대 이탈 중앙값 · 속도 명령 · 실제 속도(회마다 평균의 중앙값) · 장애물 밖 평균 이탈(회 평균의 평균) ·
            장애물 밖 최대 이탈 중앙값 · 종류(중앙 · 옆)별 지나감 · 나란할 때 옆 거리 중앙값 · p90 · 최대 · 꺾기 시작 중앙값(지나간 장애물) ·
            늦은 꺾기 수 · 최소 거리 중앙값(늦게 / 제때, 지나간 장애물). 묶음 쌍마다 통과율 Fisher 정확 검정 p.
  python3 eval/closed_loop_diag.py E6=<묶음>/runs/*_s1.json E7=<묶음>/runs/*_s1.json Nav2=<nav2 묶음>/*.json [-o 표.md] [--csv 회.csv]
필요: numpy · pyyaml · pandas(analyze_runs 의 통계 함수를 씀).
"""
import argparse
import csv
import glob
import itertools
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import obstacle_pass as OP  # noqa: E402
from analyze_runs import fisher_two_sided, wilson  # noqa: E402

ZONE_AHEAD, ZONE_BEHIND = 10.0, 6.0     # 장애물 앞 10 m ~ 뒤 6 m 는 '장애물 구간'
LATE_M = 2.0


def run_metrics(path, route):
    """회 하나 → (회 dict, 장애물 행 list). track 이 없거나 시나리오가 없으면 (None, [])."""
    j = json.load(open(path, encoding="utf-8"))
    cols = j.get("track_columns") or ["t_sim", "x", "y", "cte", "clearance"]
    tr = np.array(j.get("track") or [], dtype=float)
    if tr.ndim != 2 or len(tr) < 3:
        return None, []
    sc = OP.load_scenario(j, path)
    if sc is None:
        return None, []
    c = {k: i for i, k in enumerate(cols)}
    tg = j.get("t_goal")
    if tg is None:
        tg = tr[-1, 0] - float(j.get("lap_sim_s") or 0.0)
    te = j.get("t_end_decision")
    if te is None:
        te = tr[-1, 0]
    T = tr[(tr[:, 0] >= tg - 1e-6) & (tr[:, 0] <= te + 1e-6)]
    if len(T) < 2:
        return None, []
    direction = OP.run_direction(j, path)
    sign = -1.0 if direction == "ccw" else 1.0
    L = route[2][-1]
    S, hint = np.empty(len(T)), None
    for k in range(len(T)):
        S[k], _ = OP.project(route, T[k, c["x"]], T[k, c["y"]], hint)
        hint = S[k]
    out = np.ones(len(T), bool)
    dnear = np.full(len(T), np.nan)
    for o in sc.get("obstacles") or []:
        d = sign * OP.wrap(float(o["s"]) - S, L)
        out &= ~((d > -ZONE_BEHIND) & (d < ZONE_AHEAD))
        dnear = np.where(np.abs(d) < np.abs(np.nan_to_num(dnear, nan=1e9)), d, dnear)
    cte = T[:, c["cte"]]
    dt = np.diff(T[:, 0])
    spd = np.hypot(np.diff(T[:, c["x"]]), np.diff(T[:, c["y"]])) / np.maximum(dt, 1e-3)
    k_max = int(np.argmax(cte))
    fail = str(j.get("fail_reason") or "")
    res = str(j.get("result") or "")
    if j.get("pass"):
        label = "통과"
    elif res == "INTERVENED":
        label = "이탈"
    elif res == "CANCELED":
        label = "충돌" if "collision" in fail else "취소"
    elif "goal" in fail:
        label = "도착 오차"
    else:
        label = res or "실패"
    r = {"file": path, "scenario": sc.get("name", ""), "direction": direction, "seed": j.get("seed", ""),
         "result": label, "pass": bool(j.get("pass")), "max_cte": float(j.get("max_cte", np.max(cte))),
         "maxdev_d": float(dnear[k_max]) if np.isfinite(dnear[k_max]) else float("nan"),
         "out_mean": float(cte[out].mean()) if out.any() else float("nan"),
         "out_max": float(cte[out].max()) if out.any() else float("nan"),
         "v_cmd": float(np.mean(T[:, c["v_cmd"]])) if "v_cmd" in c else float("nan"),
         "v_gt": float(np.mean(spd)), "fail_reason": fail}
    rows, _ = OP.analyze_run(path, route)
    return r, rows


def f2(v, nd=2):
    return "—" if v is None or (isinstance(v, float) and not np.isfinite(v)) else f"{v:.{nd}f}"


def med(a):
    a = np.asarray([x for x in a if x is not None and np.isfinite(x)], float)
    return float(np.median(a)) if len(a) else float("nan")


def summarize(runs, obs):
    k, n = sum(r["pass"] for r in runs), len(runs)
    lo, hi = wilson(k, n)
    s = {"통과 (95 % 구간)": f"{k}/{n} ({f2(lo)}–{f2(hi)})",
         "최대 이탈 중앙값 m": f2(med([r["max_cte"] for r in runs])),
         "속도 명령 중앙값 m/s": f2(med([r["v_cmd"] for r in runs])),
         "실제 속도 중앙값 m/s": f2(med([r["v_gt"] for r in runs])),
         "장애물 밖 평균 이탈 m": f2(float(np.nanmean([r["out_mean"] for r in runs])) if any(np.isfinite(r["out_mean"]) for r in runs) else float("nan")),
         "장애물 밖 최대 이탈 중앙값 m": f2(med([r["out_max"] for r in runs]))}
    onset = [o for o in obs if o["turn_start_m"] is not None]
    late = [o for o in onset if o["turn_start_m"] < LATE_M]
    for kd in ("중앙", "옆"):
        g = [o for o in obs if o["kind"] == kd]
        P = [o for o in g if o["passed"]]
        off = [o["pass_offset_m"] for o in P if o["pass_offset_m"] is not None]
        ts = [o["turn_start_m"] for o in P if o["turn_start_m"] is not None]
        s[f"지나간 {kd} 장애물"] = f"{len(P)}/{len(g)}"
        s[f"{kd}: 나란할 때 옆 거리 중앙값 · p90 · 최대 m"] = (f"{np.median(off):.2f} · {np.percentile(off, 90):.2f} · {max(off):.2f}" if off else "—")
        s[f"{kd}: 꺾기 시작 중앙값 m"] = f2(med(ts), 1)
        s[f"{kd}: 늦은 꺾기(< {LATE_M:g} m)"] = f"{sum(o['kind'] == kd for o in late)}/{sum(o['kind'] == kd for o in onset)}"
    lp = [o["min_clear_m"] for o in late if o["passed"] and o["min_clear_m"] is not None]
    op = [o["min_clear_m"] for o in onset if o["turn_start_m"] >= LATE_M and o["passed"] and o["min_clear_m"] is not None]
    s["최소 거리 중앙값 m: 늦게 · 제때"] = f"{f2(med(lp))} · {f2(med(op))}"
    return s


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("groups", nargs="+", help="LABEL=글롭 (같은 LABEL 은 합침)")
    ap.add_argument("--route", default=os.path.join(OP.REPO, "eval", "routes", "parking_r3_cw.csv"))
    ap.add_argument("-o", "--out")
    ap.add_argument("--csv", help="회 행 CSV")
    a = ap.parse_args()
    route = OP.load_route(a.route)
    groups = {}
    for g in a.groups:
        if "=" not in g:
            sys.exit(f"LABEL=글롭 형식이 아님: {g}")
        lab, pat = g.split("=", 1)
        fs = [f for f in (glob.glob(pat) or [pat]) if f.endswith(".json") and not f.endswith(".meta.json")]
        groups.setdefault(lab, set()).update(fs)
    R, O = {}, {}
    for lab, fs in groups.items():
        for f in sorted(fs):
            try:
                r, rows = run_metrics(f, route)
            except Exception as e:                                      # 한 회가 깨져도 나머지는 봄
                print(f"[건너뜀] {f}: {e}", file=sys.stderr)
                continue
            if r is None:
                print(f"[건너뜀] track · 시나리오 없음: {f}", file=sys.stderr)
                continue
            r["group"] = lab
            R.setdefault(lab, []).append(r)
            O.setdefault(lab, []).extend(rows)
    if not R:
        sys.exit("회 없음 — track 이 있는 회 JSON 인지 확인")
    labs = list(R)
    S = {lab: summarize(R[lab], O.get(lab, [])) for lab in labs}
    out = ["# 폐루프 묶음 비교 (eval/closed_loop_diag.py)\n",
           "묶음: " + " · ".join(f"{lab} {len(R[lab])}회" for lab in labs) + ". 장애물 구간 = 진행 방향 앞 "
           f"{ZONE_AHEAD:g} m ~ 뒤 {ZONE_BEHIND:g} m. 장애물 행은 obstacle_pass.py(실패 회는 못 지나간 장애물이 빠짐).\n",
           "## 묶음 요약\n", "| | " + " | ".join(labs) + " |", "|---" * (len(labs) + 1) + "|"]
    for key in S[labs[0]]:
        out.append(f"| {key} | " + " | ".join(S[lab][key] for lab in labs) + " |")
    if len(labs) > 1:
        out += ["\n## 통과율 쌍 비교 (Fisher 정확 검정, 양측)\n", "| 묶음 | 통과 | 묶음 | 통과 | p |", "|---|---|---|---|---|"]
        for l1, l2 in itertools.combinations(labs, 2):
            k1, n1 = sum(r["pass"] for r in R[l1]), len(R[l1])
            k2, n2 = sum(r["pass"] for r in R[l2]), len(R[l2])
            out.append(f"| {l1} | {k1}/{n1} | {l2} | {k2}/{n2} | {fisher_two_sided(k1, n1 - k1, k2, n2 - k2):.4f} |")
    keys = ["group", "scenario", "direction", "seed", "result", "max_cte", "maxdev_d", "out_mean", "out_max", "v_cmd", "v_gt", "fail_reason", "file"]
    out += ["\n## 회\n", "| 묶음 | 시나리오 | 방향 | 시드 | 결과 | 최대 이탈 m | 최대 이탈 자리 m(+ 앞) | 장애물 밖 평균 · 최대 m | 속도 명령 · 실제 m/s |",
            "|---|---|---|---|---|---|---|---|---|"]
    for lab in labs:
        for r in sorted(R[lab], key=lambda r: (r["scenario"], r["direction"], str(r["seed"]))):
            res = r["result"] if r["pass"] else f"**{r['result']}**"
            out.append(f"| {lab} | {r['scenario']} | {r['direction']} | {r['seed']} | {res} | {f2(r['max_cte'])} | {f2(r['maxdev_d'], 1)} | "
                       f"{f2(r['out_mean'])} · {f2(r['out_max'])} | {f2(r['v_cmd'])} · {f2(r['v_gt'])} |")
    text = "\n".join(out) + "\n"
    print(text)
    if a.out:
        open(a.out, "w", encoding="utf-8").write(text)
    if a.csv:
        with open(a.csv, "w", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=keys)
            wr.writeheader()
            for lab in labs:
                for r in R[lab]:
                    wr.writerow({k: (round(r[k], 4) if isinstance(r[k], float) else r[k]) for k in keys})


if __name__ == "__main__":
    main()
