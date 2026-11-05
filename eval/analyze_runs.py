#!/usr/bin/env python3
"""고정 시나리오 60회 결과 CSV 분석 — eval/results_schema.md(스키마 v0)대로 검사한 뒤 방법 × 시나리오(× 방향)별 지표를 낸다.

검사(하나라도 걸리면 표를 만들지 않고 종료 코드 1):
  열     필수 열 · 값 형식(bool · 실수 · 정수 · 시각) · 빈칸 불가 열의 빈칸 · 음수
  버전   schema_version = v0 · repo_rev 가 커밋 해시(16진 7~40자, 끝 -dirty 허용)이고 한 값인지
         (eval/README: 60회는 재실행 없음 — 커밋 하나로 고정). 여러 값이면 --allow-mixed-rev 일 때만 경고로 넘어감
  중복   run_id 겹침
  판정   pass == 기준 6개 AND(completed · max_dev_ok · no_collision · no_long_stop · goal_reached · distance_ratio_ok)
         — sim/nav2/nav2_route.py 의 pass 와 같은 정의(수행계획서 2.4) · direction 이 cw/ccw
경고(진행은 함): 기준 bool 이 수치 열 · 한도와 안 맞음 · interventions 와 max_cte 모순 · 실패인데 fail_reason 빈칸 ·
  eval/scenarios 에 없는 시나리오 · yaml 과 다른 방향 · 버전 · -dirty · 행 수 ≠ --expect-n.

지표(방법 전체 · 방법 × 종류 · 방법 × 시나리오 · 방법 × 시나리오 × 방향):
  N · 통과 · 성공률 + Wilson 95 % 구간 · max_cte 평균 · 중앙 · 최대 · 랩(lap_sim_s, 통과한 회만) 평균 · 중앙 ·
  개입 회당 평균 · 충돌 회 수 · 장애물 최소 거리 · fail_reason 코드 수, 그리고 기준별 실패 수.
방법 비교(--ref 기준 방법 대 나머지 — 시나리오마다 · 종류마다 · 전체):
  짝 없이 — 성공률 차 + Newcombe 95 % 구간 · Fisher 정확 검정 p.
  짝(같은 scenario · direction · seed) — 통과 McNemar 정확 p, max_cte 차 · 랩 차(둘 다 통과한 짝만)의 중앙 · 평균,
  부호 검정 정확 p · Wilcoxon 부호 순위 정확 p(0 차는 빼고, 동점은 평균 순위의 조건부 정확 분포). scipy 없이 직접 계산.
N 이 작아 구간이 넓다 — 10회 중 8회 통과의 Wilson 구간은 0.49 ~ 0.94, 짝 n 개의 부호 검정이 낼 수 있는 가장 작은 p 는 2 · 0.5ⁿ
(n = 5 면 0.0625 — 0.05 밑으로 못 감). 표의 p 는 여러 시나리오를 따로 본 값이라 다중 비교 보정 없음.

사용: python3 eval/analyze_runs.py <결과.csv> [<결과2.csv> ...] [--out <폴더>] [--ref nav2] [--expect-n 60] [--allow-mixed-rev] [--no-plots]
  → 화면 표 + <폴더>/summary.json · summary.md(보고서용 마크다운 표) · matplotlib 이 있으면 PNG 2장
필요: numpy · pandas(scipy · tabulate · pyarrow 불필요, python3 -I 로 돎). 분석 환경은 eval_env(eval/README).
"""
import argparse
import datetime
import json
import math
import os
import re
import sys
from collections import Counter

import numpy as np
import pandas as pd

SCHEMA_VERSION = "v0"
# 열 이름: (형식, 필수 열, 빈칸 허용) — eval/results_schema.md 2장과 같게
COLS = {
    "schema_version": ("str", True, False),
    "run_id": ("str", True, False),
    "timestamp": ("time", True, False),
    "repo_rev": ("str", True, False),
    "method": ("str", True, False),
    "model_version": ("str", True, False),
    "scene": ("str", True, False),
    "scene_version": ("str", True, False),
    "scenario": ("str", True, False),
    "scenario_version": ("str", True, False),
    "direction": ("str", True, False),
    "seed": ("int", True, False),
    "result": ("str", True, False),
    "pass": ("bool", True, False),
    "completed": ("bool", True, False),
    "max_dev_ok": ("bool", True, False),
    "no_collision": ("bool", True, False),
    "no_long_stop": ("bool", True, False),
    "goal_reached": ("bool", True, False),
    "distance_ratio_ok": ("bool", True, False),
    "fail_reason": ("str", True, True),
    "mean_cte": ("float", True, False),
    "max_cte": ("float", True, False),
    "p95_cte": ("float", True, False),
    "lap_sim_s": ("float", True, False),
    "max_stall_s": ("float", True, False),
    "collision": ("bool", True, False),
    "min_clearance": ("float", True, True),
    "interventions": ("int", True, False),
    "dist_m": ("float", True, False),
    "plan_m": ("float", True, False),
    "goal_err": ("float", True, False),
    "log_path": ("str", True, False),
    # 선택 열
    "kind": ("str", False, True),
    "localization": ("str", False, True),
    "nav2_map": ("str", False, True),
    "route": ("str", False, True),
    "over_2m_frac": ("float", False, True),
    "recoveries": ("int", False, True),
    "wall_s": ("float", False, True),
    "amcl_start_err": ("float", False, True),
    "initialpose_resent": ("int", False, True),
    "deviation_limit_m": ("float", False, True),
    "stall_limit_s": ("float", False, True),
    "goal_radius_m": ("float", False, True),
    "timeout_s": ("float", False, True),
    "rc": ("int", False, True),
    "attempt": ("int", False, True),
    "note": ("str", False, True),
}
CRITERIA = ["completed", "max_dev_ok", "no_collision", "no_long_stop", "goal_reached", "distance_ratio_ok"]
FAIL_CODES = ["not_completed", "timeout", "max_dev", "collision", "long_stop", "goal", "distance", "harness"]
METHODS_KNOWN = ["xmobility_ft", "xmobility_zeroshot", "nav2"]
KINDS = ["straight", "curve", "obstacle", "lap", "other"]
DEFAULT_LIM = {"deviation_limit_m": 2.0, "stall_limit_s": 10.0, "goal_radius_m": 1.0}   # 시나리오 yaml 기본값(수행계획서 2.4)
DIST_RATIO = 0.8                       # nav2_route.py — 달린 거리 ≥ 경로 길이 80 %
TRUE, FALSE = {"true", "1", "yes"}, {"false", "0", "no"}
REV_RE = re.compile(r"^[0-9a-f]{7,40}(-dirty)?$")
Z = 1.959963984540054                  # 정규분포 97.5 % 점 — 95 % 구간


class Report:
    def __init__(self):
        self.errors, self.warnings = [], []

    def err(self, msg):
        self.errors.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)


# ---------------------------------------------------------------- 통계 (scipy 없이)

def wilson(k, n):
    """이항 비율의 Wilson 점수 95 % 구간. n = 0 이면 (nan, nan)."""
    if n == 0:
        return float("nan"), float("nan")
    p, z2 = k / n, Z * Z
    den = 1 + z2 / n
    c = (p + z2 / (2 * n)) / den
    h = Z * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def newcombe(k1, n1, k2, n2):
    """두 독립 비율 차 p1 − p2 의 Newcombe(Wilson 결합, method 10) 95 % 구간."""
    if n1 == 0 or n2 == 0:
        return float("nan"), float("nan")
    p1, p2 = k1 / n1, k2 / n2
    l1, u1 = wilson(k1, n1)
    l2, u2 = wilson(k2, n2)
    d = p1 - p2
    return d - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2), d + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2)


def fisher_two_sided(a, b, c, d):
    """2×2 표 [[a, b], [c, d]] 의 Fisher 정확 검정 양측 p(확률이 관측 이하인 표의 합)."""
    r1, n, c1 = a + b, a + b + c + d, a + c
    if n == 0 or r1 in (0, n) or c1 in (0, n):
        return 1.0
    tot = math.comb(n, r1)
    pr = lambda x: math.comb(c1, x) * math.comb(n - c1, r1 - x) / tot   # noqa: E731
    p0 = pr(a)
    return min(1.0, sum(pr(x) for x in range(max(0, r1 - (n - c1)), min(r1, c1) + 1) if pr(x) <= p0 * (1 + 1e-9)))


def binom_two_sided(k, n):
    """Binomial(n, 0.5) 에서 k 의 양측 정확 p — 부호 검정 · McNemar 정확 검정. n = 0 이면 nan."""
    if n == 0:
        return float("nan")
    lo = sum(math.comb(n, i) for i in range(0, k + 1)) / 2 ** n
    hi = sum(math.comb(n, i) for i in range(k, n + 1)) / 2 ** n
    return min(1.0, 2 * min(lo, hi))


def sign_test(d):
    d = [x for x in d if abs(x) > 1e-12]
    return binom_two_sided(sum(x > 0 for x in d), len(d)), len(d)


def wilcoxon_exact(d):
    """Wilcoxon 부호 순위 양측 정확 p. 0 차는 빼고, |d| 동점은 평균 순위 — 순위를 2배 해 정수로 놓고 각 부호가 반반인
    조건부 분포를 동적 계획으로 셈(동점이 없으면 보통의 정확 분포와 같음). 남은 n = 0 이면 nan."""
    d = np.array([x for x in d if abs(x) > 1e-12], dtype=float)
    n = len(d)
    if n == 0:
        return float("nan")
    r2 = (pd.Series(np.abs(d)).rank(method="average").to_numpy() * 2).round().astype(int)
    w = int(r2[d > 0].sum())
    tot = int(r2.sum())
    cnt = [0] * (tot + 1)
    cnt[0] = 1
    for r in r2:
        for s_ in range(tot - r, -1, -1):
            if cnt[s_]:
                cnt[s_ + r] += cnt[s_]
    den = 2 ** n
    return min(1.0, 2 * min(sum(cnt[: w + 1]) / den, sum(cnt[w:]) / den))


# ---------------------------------------------------------------- 읽기 · 검사

def where(df, mask, k=5):
    rows = df.loc[mask, ["_src", "_line"]].head(k)
    more = int(mask.sum()) - len(rows)
    return ", ".join(f"{s}:{ln}" for s, ln in rows.itertuples(index=False)) + (f" 외 {more}" if more > 0 else "")


def load(paths, rep):
    frames = []
    for p in paths:
        try:
            df = pd.read_csv(p, dtype=str, keep_default_na=False, na_values=[])
        except Exception as e:   # noqa: BLE001 — 파일 자체를 못 읽음
            rep.err(f"{p}: 읽기 실패 — {e}")
            continue
        df.columns = [c.strip() for c in df.columns]
        miss = [c for c, (_, req, _) in COLS.items() if req and c not in df.columns]
        if miss:
            rep.err(f"{p}: 필수 열 없음 — {', '.join(miss)}")
        extra = [c for c in df.columns if c not in COLS]
        if extra:
            rep.warn(f"{p}: 스키마에 없는 열(무시) — {', '.join(extra)}")
        df["_src"] = os.path.basename(p)
        df["_line"] = np.arange(2, len(df) + 2)
        frames.append(df)
    if not frames or rep.errors:
        return None
    df = pd.concat(frames, ignore_index=True)
    if len(df) == 0:
        rep.err("행이 하나도 없음")
    return df


def convert(df, rep):
    """문자열 열을 형식대로 바꾸며 형식 오류를 모음. 빈칸은 None/NaN."""
    out = df[["_src", "_line"]].copy()
    for c, (typ, req, nullable) in COLS.items():
        s = df[c].astype(str).str.strip() if c in df.columns else pd.Series([""] * len(df), index=df.index)
        empty = s == ""
        if c in df.columns and not nullable and empty.any():
            rep.err(f"{c}: 빈칸 {int(empty.sum())}행 ({where(df, empty)})")
        if typ == "str":
            out[c] = s
        elif typ == "bool":
            low = s.str.lower()
            bad = ~empty & ~low.isin(TRUE | FALSE)
            if bad.any():
                rep.err(f"{c}: bool 아님 {int(bad.sum())}행 — 예 {low[bad].iloc[0]!r} ({where(df, bad)})")
            out[c] = low.map(lambda v: True if v in TRUE else (False if v in FALSE else None)).astype(object)
        elif typ in ("float", "int"):
            v = pd.to_numeric(s.where(~empty), errors="coerce").astype(float)
            bad = ~empty & (v.isna() | ~np.isfinite(v.fillna(0)))
            if typ == "int":
                bad |= ~empty & v.notna() & (v != v.round())
            if bad.any():
                rep.err(f"{c}: {'정수' if typ == 'int' else '실수'} 아님 {int(bad.sum())}행 — 예 {s[bad].iloc[0]!r} ({where(df, bad)})")
            neg = v < 0
            if neg.any():
                rep.err(f"{c}: 음수 {int(neg.sum())}행 ({where(df, neg)})")
            out[c] = v
        elif typ == "time":
            t = pd.to_datetime(s.where(~empty), errors="coerce", utc=True, format="ISO8601")
            bad = ~empty & t.isna()
            if bad.any():
                rep.err(f"{c}: ISO 8601 시각 아님 {int(bad.sum())}행 — 예 {s[bad].iloc[0]!r} ({where(df, bad)})")
            out[c] = t
    return out


def kind_of(name):
    if "_obs_" in name:
        return "obstacle"
    if "_straight_" in name or name.endswith("_straight"):
        return "straight"
    if "_curve_" in name:
        return "curve"
    if name.endswith("_lap"):
        return "lap"
    return "other"


def read_yaml_fields(path):
    """시나리오 yaml 에서 version · direction · directions 만 정규식으로(PyYAML 없이 — scenario_batch.sh 와 같은 방식)."""
    txt = open(path, encoding="utf-8").read()
    f = {}
    m = re.search(r"^version:\s*(\S+)", txt, re.M)
    if m:
        f["version"] = m.group(1)
    m = re.search(r"^direction:\s*([a-z]+)", txt, re.M)
    if m:
        f["directions"] = [m.group(1)]
    m = re.search(r"^directions:\s*\[([^\]]*)\]", txt, re.M)
    if m:
        f["directions"] = [x.strip() for x in m.group(1).split(",") if x.strip()]
    return f


def validate(df, a, rep):
    """형식 변환이 끝난 표의 내용 검사."""
    # 버전
    bad = df["schema_version"] != SCHEMA_VERSION
    if bad.any():
        rep.err(f"schema_version ≠ {SCHEMA_VERSION}: {sorted(set(df.loc[bad, 'schema_version']))} ({where(df, bad)})")
    bad = ~df["repo_rev"].str.match(REV_RE.pattern)
    if bad.any():
        rep.err(f"repo_rev 가 커밋 해시가 아님(16진 소문자 7~40자 [+ -dirty]): {sorted(set(df.loc[bad, 'repo_rev']))[:5]} ({where(df, bad)})")
    revs = df["repo_rev"].value_counts()
    if len(revs) > 1:
        msg = "repo_rev 가 여러 값 — " + " · ".join(f"{r} {n}행" for r, n in revs.items())
        (rep.warn if a.allow_mixed_rev else rep.err)(msg + ("" if a.allow_mixed_rev else " (60회는 커밋 하나여야 함 — 의도면 --allow-mixed-rev)"))
    dirty = df["repo_rev"].str.endswith("-dirty")
    if dirty.any():
        rep.warn(f"repo_rev 에 -dirty {int(dirty.sum())}행 — 커밋되지 않은 변경으로 돈 회 ({where(df, dirty)})")
    # 중복
    dup = df["run_id"].duplicated(keep=False) & (df["run_id"] != "")
    if dup.any():
        rep.err(f"run_id 중복 {df.loc[dup, 'run_id'].nunique()}개 — {sorted(set(df.loc[dup, 'run_id']))[:5]} ({where(df, dup)})")
    # 값 목록
    bad = ~df["direction"].isin(["cw", "ccw"]) & (df["direction"] != "")
    if bad.any():
        rep.err(f"direction 이 cw/ccw 아님: {sorted(set(df.loc[bad, 'direction']))} ({where(df, bad)})")
    unk = sorted(set(df["method"]) - set(METHODS_KNOWN) - {""})
    if unk:
        rep.warn(f"제안 목록({', '.join(METHODS_KNOWN)})에 없는 method: {unk}")
    if rep.errors:
        return
    # 판정 = 기준 6개 AND
    allc = df[CRITERIA].astype(bool).all(axis=1)
    bad = df["pass"].astype(bool) != allc
    if bad.any():
        rep.err(f"pass 가 기준 6개 AND 와 다름 {int(bad.sum())}행 ({where(df, bad)})")
    # 기준 bool ↔ 수치 열(한도 근처 ±tol 은 반올림 차이로 보고 넘어감)
    lim = {k: df[k].fillna(v) if k in df else pd.Series(v, index=df.index) for k, v in DEFAULT_LIM.items()}
    checks = [
        ("completed", df["result"] == "SUCCEEDED", None, "result == SUCCEEDED"),
        ("max_dev_ok", df["max_cte"] <= lim["deviation_limit_m"], (df["max_cte"] - lim["deviation_limit_m"]).abs() <= 1e-3,
         "max_cte ≤ deviation_limit_m"),
        ("no_collision", ~df["collision"].astype(bool), None, "not collision"),
        ("no_long_stop", df["max_stall_s"] < lim["stall_limit_s"], (df["max_stall_s"] - lim["stall_limit_s"]).abs() <= 0.05,
         "max_stall_s < stall_limit_s"),
        ("goal_reached", df["goal_err"] <= lim["goal_radius_m"], (df["goal_err"] - lim["goal_radius_m"]).abs() <= 1e-3,
         "goal_err ≤ goal_radius_m"),
        ("distance_ratio_ok", df["dist_m"] >= DIST_RATIO * df["plan_m"], (df["dist_m"] - DIST_RATIO * df["plan_m"]).abs() <= 0.1,
         f"dist_m ≥ {DIST_RATIO} · plan_m"),
    ]
    for col, expect, near, rule in checks:
        bad = df[col].astype(bool) != expect
        if near is not None:
            bad &= ~near
        if bad.any():
            rep.warn(f"{col} 가 수치와 안 맞음({rule}) {int(bad.sum())}행 ({where(df, bad)})")
    over = df["max_cte"] > lim["deviation_limit_m"] + 1e-3
    bad = over & (df["interventions"] == 0)
    if bad.any():
        rep.warn(f"max_cte 가 한도를 넘었는데 interventions = 0 — {int(bad.sum())}행 ({where(df, bad)})")
    bad = ~(df["max_cte"] > lim["deviation_limit_m"] - 1e-3) & (df["interventions"] > 0)
    if bad.any():
        rep.warn(f"max_cte 가 한도 안인데 interventions > 0 — {int(bad.sum())}행 ({where(df, bad)})")
    bad = (df["mean_cte"] > df["p95_cte"] + 1e-3) | (df["p95_cte"] > df["max_cte"] + 1e-3)
    if bad.any():
        rep.warn(f"mean_cte ≤ p95_cte ≤ max_cte 가 아님 {int(bad.sum())}행 ({where(df, bad)})")
    if "over_2m_frac" in df:
        bad = df["over_2m_frac"] > 1
        if bad.any():
            rep.err(f"over_2m_frac > 1 {int(bad.sum())}행 ({where(df, bad)})")
    # fail_reason
    failed = ~df["pass"].astype(bool)
    bad = failed & (df["fail_reason"] == "")
    if bad.any():
        rep.warn(f"실패인데 fail_reason 빈칸 {int(bad.sum())}행 ({where(df, bad)})")
    bad = ~failed & (df["fail_reason"] != "")
    if bad.any():
        rep.warn(f"통과인데 fail_reason 있음 {int(bad.sum())}행 ({where(df, bad)})")
    code = df["fail_reason"].map(fail_code)
    bad = (code != "") & ~code.isin(FAIL_CODES)
    if bad.any():
        rep.warn(f"모르는 fail_reason 코드 {sorted(set(code[bad]))} — 쓸 수 있는 코드 {FAIL_CODES} ({where(df, bad)})")
    # 장애물 시나리오인데 장애물 거리 없음
    bad = (df["kind"] == "obstacle") & df["min_clearance"].isna()
    if bad.any():
        rep.warn(f"장애물 시나리오인데 min_clearance 빈칸 {int(bad.sum())}행 ({where(df, bad)})")
    # 시나리오 파일과 대조
    for name, g in df.groupby("scenario", sort=True):
        path = os.path.join(a.scenarios_dir, f"{name}.yaml")
        if not os.path.isfile(path):
            rep.warn(f"시나리오 파일 없음: {path} ({len(g)}행)")
            continue
        y = read_yaml_fields(path)
        if "version" in y and (g["scenario_version"] != y["version"]).any():
            rep.warn(f"{name}: scenario_version {sorted(set(g['scenario_version']))} ≠ yaml {y['version']}")
        if "directions" in y and (~g["direction"].isin(y["directions"])).any():
            rep.warn(f"{name}: yaml 방향 {y['directions']} 밖의 direction {sorted(set(g['direction']) - set(y['directions']))}")
    if a.expect_n and len(df) != a.expect_n:
        rep.warn(f"행 수 {len(df)} ≠ --expect-n {a.expect_n}")


def fail_code(s):
    return s.split(":", 1)[0].strip() if s else ""


# ---------------------------------------------------------------- 요약

def nanstat(x, f):
    x = np.asarray(x, dtype=float)
    x = x[~np.isnan(x)]
    return float(f(x)) if len(x) else float("nan")


def group_stats(g):
    n, k = len(g), int(g["pass"].sum())
    lo, hi = wilson(k, n)
    lap = g.loc[g["pass"], "lap_sim_s"]
    codes = Counter(c for c in g["fail_reason"].map(fail_code) if c)
    return {
        "N": n, "n_pass": k, "rate": k / n if n else float("nan"), "ci_lo": lo, "ci_hi": hi,
        "max_cte_mean": nanstat(g["max_cte"], np.mean), "max_cte_median": nanstat(g["max_cte"], np.median),
        "max_cte_max": nanstat(g["max_cte"], np.max), "mean_cte_mean": nanstat(g["mean_cte"], np.mean),
        "p95_cte_median": nanstat(g["p95_cte"], np.median),
        "lap_pass_n": int(len(lap)), "lap_pass_mean": nanstat(lap, np.mean), "lap_pass_median": nanstat(lap, np.median),
        "interventions_total": int(g["interventions"].sum()), "interventions_per_run": float(g["interventions"].mean()),
        "collision_runs": int(g["collision"].sum()), "min_clearance_min": nanstat(g["min_clearance"], np.min),
        "max_stall_max": nanstat(g["max_stall_s"], np.max),
        "fail_codes": dict(sorted(codes.items())),
        "criteria_fail": {c: int((~g[c]).sum()) for c in CRITERIA},
    }


def table(df, keys):
    rows = []
    for kv, g in df.groupby(keys, sort=True):
        kv = kv if isinstance(kv, tuple) else (kv,)
        rows.append({**dict(zip(keys, kv)), **group_stats(g)})
    return rows


def compare(df, ref, pair_key, rep):
    """기준 방법(ref) 대 나머지 — 시나리오마다 · 종류마다 · 전체. 짝 없는 성공률 차와 짝 검정."""
    unpaired, paired = [], []
    others = [m for m in sorted(df["method"].unique()) if m != ref]
    scopes = [("scenario", s, df["scenario"] == s) for s in sorted(df["scenario"].unique())]
    scopes += [("kind", k, df["kind"] == k) for k in KINDS if (df["kind"] == k).any()]
    scopes += [("all", "전체", pd.Series(True, index=df.index))]
    dup_noted = set()
    for m in others:
        for level, name, mask in scopes:
            A, B = df[mask & (df["method"] == m)], df[mask & (df["method"] == ref)]
            if len(A) == 0 or len(B) == 0:
                continue
            k1, n1, k2, n2 = int(A["pass"].sum()), len(A), int(B["pass"].sum()), len(B)
            lo, hi = newcombe(k1, n1, k2, n2)
            unpaired.append({"method": m, "ref": ref, "level": level, "scope": name, "n": n1, "k": k1, "n_ref": n2, "k_ref": k2,
                             "rate": k1 / n1, "rate_ref": k2 / n2, "diff": k1 / n1 - k2 / n2, "diff_lo": lo, "diff_hi": hi,
                             "fisher_p": fisher_two_sided(k1, n1 - k1, k2, n2 - k2)})
            # 짝 — 열쇠가 방법 안에서 겹치면 그 열쇠는 뺌
            da, db = A[A.duplicated(pair_key, keep=False)], B[B.duplicated(pair_key, keep=False)]
            for x, d_ in ((m, da), (ref, db)):
                if len(d_) and (x, name) not in dup_noted and level == "all":
                    rep.warn(f"짝 열쇠 {pair_key} 가 {x} 안에서 겹침 {len(d_)}행 — 짝 비교에서 뺌")
                    dup_noted.add((x, name))
            A2, B2 = A.drop_duplicates(pair_key, keep=False), B.drop_duplicates(pair_key, keep=False)
            P = A2.merge(B2, on=pair_key, suffixes=("", "_ref"))
            if len(P) == 0:
                continue
            b = int((P["pass"] & ~P["pass_ref"]).sum())
            c = int((~P["pass"] & P["pass_ref"]).sum())
            dcte = (P["max_cte"] - P["max_cte_ref"]).to_list()
            both = P[P["pass"] & P["pass_ref"]]
            dlap = (both["lap_sim_s"] - both["lap_sim_s_ref"]).to_list()
            p_sc, n_sc = sign_test(dcte)
            p_sl, n_sl = sign_test(dlap)
            paired.append({
                "method": m, "ref": ref, "level": level, "scope": name, "n_pairs": len(P),
                "both_pass": len(both), "only_method_pass": b, "only_ref_pass": c, "mcnemar_p": binom_two_sided(b, b + c),
                "max_cte_diff_median": nanstat(dcte, np.median), "max_cte_diff_mean": nanstat(dcte, np.mean),
                "max_cte_method_lower": int(sum(x < 0 for x in dcte)), "max_cte_sign_p": p_sc,
                "max_cte_wilcoxon_p": wilcoxon_exact(dcte), "max_cte_min_p": 2 * 0.5 ** n_sc if n_sc else float("nan"),
                "lap_n": len(dlap), "lap_diff_median": nanstat(dlap, np.median), "lap_diff_mean": nanstat(dlap, np.mean),
                "lap_method_faster": int(sum(x < 0 for x in dlap)), "lap_sign_p": p_sl, "lap_wilcoxon_p": wilcoxon_exact(dlap),
                "lap_min_p": 2 * 0.5 ** n_sl if n_sl else float("nan"),
            })
    return unpaired, paired


# ---------------------------------------------------------------- 출력

def fmt(v, nd=2):
    if v is None:
        return "—"
    if isinstance(v, (bool, np.bool_)):
        return "예" if v else "아니오"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, float) and math.isnan(v):
        return "—"
    if isinstance(v, (float, np.floating)):
        return f"{v:.{nd}f}"
    return str(v)


def fmt_p(v):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "—"
    return "<0.001" if v < 0.001 else f"{v:.3f}"


def md_table(headers, rows):
    esc = lambda s: str(s).replace("|", "\\|").replace("\n", " ")   # noqa: E731
    out = ["| " + " | ".join(headers) + " |", "| " + " | ".join("---" for _ in headers) + " |"]
    out += ["| " + " | ".join(esc(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def codes_str(d):
    return " · ".join(f"{k} {v}" for k, v in d.items()) if d else "—"


def stat_cols(r, lap=True):
    cols = [fmt(r["N"]), f"{r['n_pass']}/{r['N']}", fmt(r["rate"]), f"{fmt(r['ci_lo'])}–{fmt(r['ci_hi'])}",
            fmt(r["max_cte_mean"]), fmt(r["max_cte_median"]), fmt(r["max_cte_max"])]
    if lap:
        cols += [fmt(r["lap_pass_mean"], 1), fmt(r["lap_pass_median"], 1)]
    cols += [fmt(r["interventions_per_run"]), fmt(r["collision_runs"]), fmt(r["min_clearance_min"]), codes_str(r["fail_codes"])]
    return cols


STAT_H = ["N", "통과", "성공률", "95 % 구간(Wilson)", "max_cte 평균", "중앙", "최대"]
STAT_T = ["개입/회", "충돌 회", "장애물 최소 거리", "실패 사유(fail_reason)"]
LAP_H = ["랩 평균(통과)", "랩 중앙(통과)"]


def build_md(S):
    L = [f"# 60회 결과 요약 — {', '.join(S['inputs'])}", ""]
    L += [f"- 생성: {S['generated']} · `eval/analyze_runs.py` · 스키마 {S['schema_version']}",
          f"- 행 수: {S['n_rows']} · repo_rev: {', '.join(f'`{r}`' for r in S['repo_rev'])} · 방법: {', '.join(S['methods'])} · 비교 기준: {S['ref'] or '—'}",
          "- 단위: 이탈(cte) m · 랩 s(시뮬). 이탈은 기준선(eval/routes)까지 거리, 판정은 수행계획서 2.4(완주 · 이탈 2.0 m · 충돌 없음 · "
          "10 s 넘는 정지 없음 · 목표 1.0 m · 거리 80 %).",
          f"- **N 이 작아 구간이 넓습니다.** 예: 성공률 구간 폭 — 전체 방법별 {S['ci_width_note']}. 시나리오 칸은 N 이 더 작아 더 넓습니다. "
          "p 값은 칸마다 따로 낸 값(다중 비교 보정 없음)이고, 짝 n 개 부호 검정의 가장 작은 p 는 2 · 0.5ⁿ 입니다.", ""]
    L += ["## 1. 방법별 전체", "", md_table(["방법"] + STAT_H + STAT_T, [[r["method"]] + stat_cols(r, lap=False) for r in S["by_method"]]), ""]
    L += ["## 2. 방법 × 종류", "", md_table(["방법", "종류"] + STAT_H + STAT_T,
                                            [[r["method"], r["kind"]] + stat_cols(r, lap=False) for r in S["by_method_kind"]]), ""]
    L += ["## 3. 방법 × 시나리오", "", md_table(["방법", "시나리오"] + STAT_H + LAP_H + STAT_T,
                                             [[r["method"], r["scenario"]] + stat_cols(r) for r in S["by_method_scenario"]]), ""]
    L += ["## 4. 방법 × 시나리오 × 방향", "", md_table(["방법", "시나리오", "방향"] + STAT_H + LAP_H + STAT_T,
                                                  [[r["method"], r["scenario"], r["direction"]] + stat_cols(r)
                                                   for r in S["by_method_scenario_direction"]]), ""]
    L += ["## 5. 기준별 실패 수 (한 회가 여러 기준에 걸릴 수 있음)", "",
          md_table(["방법", "종류", "N"] + CRITERIA, [[r["method"], r["kind"], r["N"]] + [r["criteria_fail"][c] for c in CRITERIA]
                                                    for r in S["by_method_kind"]] +
                   [[r["method"], "전체", r["N"]] + [r["criteria_fail"][c] for c in CRITERIA] for r in S["by_method"]]), ""]
    if S["ref"]:
        L += [f"## 6. 방법 비교 — 기준 {S['ref']}", "", "### 6.1 짝 없이 — 성공률 차(방법 − 기준)", "",
              md_table(["방법", "범위", "통과", "기준 통과", "차", "95 % 구간(Newcombe)", "Fisher p"],
                       [[u["method"], u["scope"], f"{u['k']}/{u['n']}", f"{u['k_ref']}/{u['n_ref']}", fmt(u["diff"]),
                         f"{fmt(u['diff_lo'])}–{fmt(u['diff_hi'])}", fmt_p(u["fisher_p"])] for u in S["comparison"]["unpaired"]]), ""]
        L += [f"### 6.2 짝 비교 — 같은 {' · '.join(S['pair_key'])}, 차 = 방법 − 기준(음수면 방법이 이탈이 작거나 빠름)", ""]
        if S["comparison"]["paired"]:
            L += [md_table(["방법", "범위", "짝", "둘 다 통과", "방법만", "기준만", "McNemar p", "max_cte 차 중앙", "평균", "방법이 작음",
                            "부호 p", "Wilcoxon p", "최소 가능 p"],
                           [[p["method"], p["scope"], p["n_pairs"], p["both_pass"], p["only_method_pass"], p["only_ref_pass"],
                             fmt_p(p["mcnemar_p"]), fmt(p["max_cte_diff_median"]), fmt(p["max_cte_diff_mean"]),
                             f"{p['max_cte_method_lower']}/{p['n_pairs']}", fmt_p(p["max_cte_sign_p"]), fmt_p(p["max_cte_wilcoxon_p"]),
                             fmt_p(p["max_cte_min_p"])] for p in S["comparison"]["paired"]]), "",
                  md_table(["방법", "범위", "둘 다 통과한 짝", "랩 차 중앙", "평균", "방법이 빠름", "부호 p", "Wilcoxon p", "최소 가능 p"],
                           [[p["method"], p["scope"], p["lap_n"], fmt(p["lap_diff_median"], 1), fmt(p["lap_diff_mean"], 1),
                             f"{p['lap_method_faster']}/{p['lap_n']}", fmt_p(p["lap_sign_p"]), fmt_p(p["lap_wilcoxon_p"]),
                             fmt_p(p["lap_min_p"])] for p in S["comparison"]["paired"]]), ""]
        else:
            L += ["짝이 없음 — 방법 사이에 같은 짝 열쇠의 회가 없습니다.", ""]
    L += ["## 7. 실패한 회", ""]
    if S["failed_runs"]:
        L += [md_table(["run_id", "방법", "시나리오", "방향", "seed", "result", "fail_reason", "max_cte", "기준 실패", "log_path"],
                       [[f["run_id"], f["method"], f["scenario"], f["direction"], f["seed"], f["result"], f["fail_reason"] or "—",
                         fmt(f["max_cte"]), " · ".join(f["criteria_failed"]), f["log_path"]] for f in S["failed_runs"]]), ""]
    else:
        L += ["없음", ""]
    L += ["## 경고", ""] + ([f"- {w}" for w in S["warnings"]] or ["없음"]) + [""]
    return "\n".join(L)


def clean(o):
    """JSON 으로 — NaN 은 null, numpy 값은 파이썬 값으로."""
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, (np.bool_, bool)):
        return bool(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating, float)):
        return None if math.isnan(o) else float(f"{float(o):.6g}")   # 유효 숫자 6자리 — 작은 p 가 0 으로 뭉개지지 않게
    return o


def plots(S, df, out, rep):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib 없음 — 그림은 건너뜀")
        return []
    files = []
    methods, scen = S["methods"], sorted(df["scenario"].unique())
    w = 0.8 / max(len(methods), 1)
    rows = {(r["method"], r["scenario"]): r for r in S["by_method_scenario"]}
    fig, ax = plt.subplots(figsize=(min(18, max(6, 0.5 * len(scen) * max(len(methods), 1))), 4.5))
    for i, m in enumerate(methods):
        xs, ys, el, eh = [], [], [], []
        for j, s_ in enumerate(scen):
            r = rows.get((m, s_))
            if r:
                xs.append(j + (i - (len(methods) - 1) / 2) * w)
                ys.append(r["rate"])
                el.append(r["rate"] - r["ci_lo"])
                eh.append(r["ci_hi"] - r["rate"])
        ax.bar(xs, ys, w, yerr=[el, eh], capsize=3, label=m)
    ax.set_xticks(range(len(scen)), [s_.replace("parking_r3_", "") for s_ in scen], rotation=40, ha="right")
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("success rate (Wilson 95% CI)")
    ax.legend()
    fig.tight_layout()
    p = os.path.join(out, "success_rate.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    files.append(p)
    fig, ax = plt.subplots(figsize=(min(18, max(6, 0.5 * len(scen) * max(len(methods), 1))), 4.5))
    for i, m in enumerate(methods):
        for j, s_ in enumerate(scen):
            v = df[(df["method"] == m) & (df["scenario"] == s_)]["max_cte"].to_numpy()
            ax.scatter(np.full(len(v), j + (i - (len(methods) - 1) / 2) * w), v, s=14, color=f"C{i}", label=m if j == 0 else None)
    ax.axhline(2.0, color="k", lw=0.8, ls="--")
    ax.set_xticks(range(len(scen)), [s_.replace("parking_r3_", "") for s_ in scen], rotation=40, ha="right")
    ax.set_ylabel("max_cte [m] (limit 2.0)")
    ax.legend()
    fig.tight_layout()
    p = os.path.join(out, "max_cte.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    files.append(p)
    return files


def main():
    ap = argparse.ArgumentParser(description="60회 결과 CSV 검사 · 분석 (eval/results_schema.md v0)")
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--out", help="summary.json · summary.md · 그림 폴더(기본: 첫 CSV 옆 analysis/)")
    ap.add_argument("--ref", default="nav2", help="비교 기준 방법(없으면 비교 생략)")
    ap.add_argument("--pair-key", default="scenario,direction,seed", help="짝 비교 열쇠(쉼표로)")
    ap.add_argument("--expect-n", type=int, default=0, help="기대 행 수(예: 60) — 다르면 경고")
    ap.add_argument("--allow-mixed-rev", action="store_true", help="repo_rev 가 여러 값이어도 경고만")
    ap.add_argument("--scenarios-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenarios"))
    ap.add_argument("--no-plots", action="store_true")
    a = ap.parse_args()

    rep = Report()
    raw = load(a.csv, rep)
    df = convert(raw, rep) if raw is not None and not rep.errors else None
    if df is not None and not rep.errors:
        df["kind"] = [k if k else kind_of(s) for k, s in zip(df["kind"], df["scenario"])]
        bad = ~df["kind"].isin(KINDS)
        if bad.any():
            rep.warn(f"kind 값이 {KINDS} 밖: {sorted(set(df.loc[bad, 'kind']))} — other 로 셈 ({where(df, bad)})")
            df.loc[bad, "kind"] = "other"
        validate(df, a, rep)
    for w in rep.warnings:
        print(f"경고: {w}")
    if rep.errors:
        for e in rep.errors:
            print(f"오류: {e}")
        print(f"ANALYZE_RUNS_FAIL 오류 {len(rep.errors)} · 경고 {len(rep.warnings)} — 스키마 eval/results_schema.md")
        sys.exit(1)

    for c in CRITERIA + ["pass", "collision"]:
        df[c] = df[c].astype(bool)
    for c in ("interventions", "seed"):
        df[c] = df[c].astype(int)
    pair_key = [k.strip() for k in a.pair_key.split(",") if k.strip()]
    methods = sorted(df["method"].unique())
    ref = a.ref if a.ref in methods and len(methods) > 1 else None
    if a.ref and not ref and len(methods) > 1:
        rep.warn(f"기준 방법 {a.ref!r} 가 데이터에 없음 — 방법 비교 생략(--ref 로 지정)")
    unpaired, paired = compare(df, ref, pair_key, rep) if ref else ([], [])
    by_method = table(df, ["method"])
    widths = [f"{r['method']} N {r['N']} → ±{(r['ci_hi'] - r['ci_lo']) / 2:.2f}" for r in by_method]
    failed = df[~df["pass"]].sort_values(["method", "scenario", "direction", "seed"])
    S = {
        "schema_version": SCHEMA_VERSION, "generated": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "inputs": [os.path.basename(p) for p in a.csv], "n_rows": len(df), "repo_rev": sorted(df["repo_rev"].unique()),
        "methods": methods, "ref": ref, "pair_key": pair_key, "ci_width_note": " · ".join(widths),
        "by_method": by_method, "by_method_kind": table(df, ["method", "kind"]),
        "by_method_scenario": table(df, ["method", "scenario"]),
        "by_method_scenario_direction": table(df, ["method", "scenario", "direction"]),
        "comparison": {"unpaired": unpaired, "paired": paired},
        "failed_runs": [{"run_id": r.run_id, "method": r.method, "scenario": r.scenario, "direction": r.direction, "seed": int(r.seed),
                         "result": r.result, "fail_reason": r.fail_reason, "max_cte": float(r.max_cte),
                         "criteria_failed": [c for c in CRITERIA if not getattr(r, c)], "log_path": r.log_path}
                        for r in failed.itertuples(index=False)],
        "warnings": rep.warnings,
    }
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.csv[0])), "analysis")
    os.makedirs(out, exist_ok=True)
    md = build_md(S)
    S["plots"] = [] if a.no_plots else plots(S, df, out, rep)
    with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(clean(S), f, ensure_ascii=False, indent=1)
    with open(os.path.join(out, "summary.md"), "w", encoding="utf-8") as f:
        f.write(md)
    print(md)
    print(f"ANALYZE_RUNS_OK 행 {len(df)} · 방법 {len(methods)} · 시나리오 {df['scenario'].nunique()} · 경고 {len(rep.warnings)} → {out}/summary.json · summary.md"
          + (f" · 그림 {len(S['plots'])}" if S["plots"] else ""))


if __name__ == "__main__":
    main()
