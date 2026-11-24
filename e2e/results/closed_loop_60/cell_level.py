#!/usr/bin/env python3
"""60회 결과를 칸(시나리오 × 방향) 단위로 읽기 — 읽는 법은 PLAN.md '60회 읽는 법(11.24)'에 결과 전 고정 (E, 11.24). 결과 폴더 전용(도구 아님).

왜    eval/analyze_runs.py 의 짝 비교(McNemar)는 같은 칸의 반복 3회를 서로 독립으로 본다. 11.17 demo 시계 두 회는 궤적이
      0.05 m 안으로 같았으므로 반복은 독립이 아닐 수 있다 → 칸마다 통과율을 내 두 방법을 칸 단위 부호 검정으로도 견준다.
내는 것
  1) 칸 × 방법 통과 k/n
  2) 방법 쌍마다: 짝(시나리오 · 방향 · 반복) McNemar 정확 검정 + 칸 단위 부호 검정(칸 통과율 높음 / 같음 / 낮음, 같음은 뺌)
  3) 미리 고정한 말(PLAN.md): 낫다 / 나아 보이나 칸 단위로는 확정 못 함 / 차이를 가르지 못함 / 낫지 않음
  4) 민감도(탐색): 파인튜닝 대 제로샷을 반시계 바퀴 칸(direction ccw 이고 kind 가 obstacle · lap)을 빼고 한 번 더
  5) 11.18 에 적은 예상(closed_loop_demo_11_17.md)을 칸마다 채점
쓰기  python3 e2e/results/closed_loop_60/cell_level.py <results.csv> [<results.csv> ...] [-o 표.md]
필요: 표준 라이브러리만. 검사(스키마 · pass = 기준 6개 AND)는 analyze_runs.py 가 먼저 한다 — 이 스크립트는 pass 열을 그대로 믿는다.
"""
import argparse
import csv
import sys
from collections import defaultdict
from math import comb

FT, ZS, NAV = "xmobility_ft", "xmobility_zeroshot", "nav2"
ALPHA = 0.05
# 11.18 에 적은 예상(파인튜닝 24회) — (시나리오 이름에서 parking_r3_ 를 뗀 것, 방향): (통과 수 아래, 위). '1/3 안팎' = 0–2, '실패할 가능성이 큼' = 0–1.
PREDICTED = {
    ("obs_demo", "cw"): (0, 0), ("obs_demo", "ccw"): (0, 2), ("obs_slalom", "cw"): (0, 1), ("obs_cones", "ccw"): (3, 3),
    ("straight_south", "cw"): (3, 3), ("curve_se_ccw", "ccw"): (3, 3), ("lap", "cw"): (3, 3), ("lap", "ccw"): (3, 3),
}
PREDICTED_TOTAL = (13, 17)


def truthy(x):
    return str(x).strip().lower() in ("true", "1")


def binom_two_sided(a, b):
    """부호 검정 · McNemar 정확 검정: 한쪽 a 번, 다른 쪽 b 번일 때 양측 p."""
    n = a + b
    if n == 0:
        return 1.0
    return min(1.0, 2.0 * sum(comb(n, i) for i in range(0, min(a, b) + 1)) / 2 ** n)


def load(paths):
    rows = []
    for p in paths:
        with open(p, encoding="utf-8-sig", newline="") as f:
            rows += list(csv.DictReader(f))
    return rows


def compare(rows, a, b, drop=None):
    """방법 a 대 b. 돌려주는 것: 짝 수 · (둘 다, a 만, b 만, 둘 다 실패) · McNemar p · 칸 (높음, 같음, 낮음) · 부호 p · 견준 칸 수."""
    keep = [r for r in rows if not (drop and drop(r))]
    by = defaultdict(dict)
    for r in keep:
        if r["method"] in (a, b):
            by[(r["scenario"], r["direction"], r["seed"])][r["method"]] = truthy(r["pass"])
    pairs = [v for v in by.values() if a in v and b in v]
    both = sum(v[a] and v[b] for v in pairs)
    only_a = sum(v[a] and not v[b] for v in pairs)
    only_b = sum(v[b] and not v[a] for v in pairs)
    neither = len(pairs) - both - only_a - only_b
    cell = defaultdict(lambda: defaultdict(list))
    for r in keep:
        if r["method"] in (a, b):
            cell[(r["scenario"], r["direction"])][r["method"]].append(truthy(r["pass"]))
    hi = eq = lo = 0
    for v in cell.values():
        if not v[a] or not v[b]:
            continue
        ra, rb = sum(v[a]) / len(v[a]), sum(v[b]) / len(v[b])
        hi, eq, lo = hi + (ra > rb), eq + (ra == rb), lo + (ra < rb)
    return {"pairs": len(pairs), "tab": (both, only_a, only_b, neither), "mcnemar": binom_two_sided(only_a, only_b),
            "cells": (hi, eq, lo), "sign": binom_two_sided(hi, lo), "ncell": hi + eq + lo}


def wording(c):
    """PLAN.md 에 결과 전 고정한 말. 짝 검정과 칸 부호가 같은 쪽을 가리킬 때만 '낫다'."""
    both, only_a, only_b, _ = c["tab"]
    hi, _, lo = c["cells"]
    if c["mcnemar"] < ALPHA and only_a > only_b:
        return "낫다" if hi > lo else "나아 보이나 칸 단위로는 확정 못 함"
    if c["mcnemar"] < ALPHA and only_b > only_a:
        return "낫지 않음(더 낮음)" if lo > hi else "낮아 보이나 칸 단위로는 확정 못 함"
    return "차이를 가르지 못함"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("-o", "--out")
    a = ap.parse_args()
    rows = load(a.csv)
    methods = [m for m in (FT, ZS, NAV) if any(r["method"] == m for r in rows)]
    out = ["# 60회 — 칸 단위 읽기 (`e2e/results/closed_loop_60/cell_level.py`)", "",
           "행 %d · 방법 %s. 읽는 법은 `PLAN.md` '60회 읽는 법(11.24)'." % (len(rows), " · ".join(methods)), ""]

    cells = defaultdict(lambda: defaultdict(list))
    for r in rows:
        cells[(r["scenario"], r["direction"])][r["method"]].append(truthy(r["pass"]))
    out += ["## 칸 × 방법 통과", "", "| 시나리오 | 방향 | " + " | ".join(methods) + " |", "|---|---|" + "---|" * len(methods)]
    for (sc, d) in sorted(cells):
        out.append("| %s | %s | %s |" % (sc.replace("parking_r3_", ""), d, " | ".join(
            ("%d/%d" % (sum(cells[(sc, d)][m]), len(cells[(sc, d)][m])) if cells[(sc, d)][m] else "—") for m in methods)))
    out.append("| **합계** | | %s |" % " | ".join("**%d/%d**" % (sum(truthy(r["pass"]) for r in rows if r["method"] == m),
                                                              sum(r["method"] == m for r in rows)) for m in methods))

    out += ["", "## 방법 비교 — 짝(시나리오 · 방향 · 반복)과 칸", "",
            "| 비교 | 짝 | 둘 다 · 앞만 · 뒤만 · 둘 다 실패 | McNemar p | 칸: 앞이 높음 · 같음 · 낮음 | 칸 부호 p | 고정한 말 |", "|---|---|---|---|---|---|---|"]
    ccw_lap = lambda r: r["direction"] == "ccw" and r.get("kind", "") in ("obstacle", "lap")  # noqa: E731
    plans = [("파인튜닝 대 제로샷", FT, ZS, None), ("파인튜닝 대 Nav2", FT, NAV, None),
             ("(탐색) 파인튜닝 대 제로샷, 반시계 바퀴 칸 뺌", FT, ZS, ccw_lap)]
    for label, x, y, drop in plans:
        if x not in methods or y not in methods:
            continue
        c = compare(rows, x, y, drop)
        out.append("| %s | %d | %d · %d · %d · %d | %.4f | %d · %d · %d (칸 %d) | %.4f | %s |" % (
            (label, c["pairs"]) + c["tab"] + (c["mcnemar"],) + c["cells"] + (c["ncell"], c["sign"],
                                                                        wording(c) if drop is None else "(주 결론 아님)")))
    out += ["", "- 칸 부호 검정은 칸 통과율이 같은 칸을 빼고 센다. 8칸이면 낼 수 있는 가장 작은 양측 p 는 0.0078(8칸 모두 한쪽), 5칸이면 0.0625.",
            "- McNemar 는 반복을 독립으로 본 값이라 낙관적이다. 두 검정이 엇갈리면 '확정 못 함'으로 적는다. 다중 비교 보정 없음(주 비교 둘)."]

    if FT in methods:
        out += ["", "## 11.18 에 적은 예상 채점 (파인튜닝)", "", "| 칸 | 예상 통과 | 실제 | |", "|---|---|---|---|"]
        hit = n_scored = 0
        for (sc, d), (lo, hi) in PREDICTED.items():
            v = cells.get(("parking_r3_" + sc, d), {}).get(FT)
            if not v:
                out.append("| %s %s | %s | 없음 | — |" % (sc, d, "%d" % lo if lo == hi else "%d–%d" % (lo, hi)))
                continue
            k, ok = sum(v), lo <= sum(v) <= hi
            hit, n_scored = hit + ok, n_scored + 1
            out.append("| %s %s | %s/3 | %d/%d | %s |" % (sc, d, "%d" % lo if lo == hi else "%d–%d" % (lo, hi), k, len(v), "맞음" if ok else "**틀림**"))
        tot = sum(truthy(r["pass"]) for r in rows if r["method"] == FT)
        out.append("| 합계 | %d–%d | %d | %s |" % (PREDICTED_TOTAL + (tot, "맞음" if PREDICTED_TOTAL[0] <= tot <= PREDICTED_TOTAL[1] else "**틀림**")))
        out.append("")
        out.append("칸 %d개 중 %d개 맞음." % (n_scored, hit))

    text = "\n".join(out) + "\n"
    if a.out:
        open(a.out, "w", encoding="utf-8", newline="\n").write(text)
        print("→", a.out)
    else:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
