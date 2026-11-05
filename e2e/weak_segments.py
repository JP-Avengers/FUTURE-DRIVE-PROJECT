#!/usr/bin/env python3
"""약한 구간 찾기 — e2e/eval_offline.py 의 frames.csv(프레임별)를 기준선 위 s 구간으로 묶어, 모델 명령이 교사와 가장 많이 다른
곳을 찾는다. 실패구간 보정(추가 수집 · DAgger 대상 구간 고르기)과 제로샷 vs 파인튜닝 구간별 비교에 쓴다.
개루프 지표라(eval_offline.py 머리말) 폐루프에서 실제로 이탈 · 충돌하는 위치와 같다는 보장은 없음.

묶음: (시나리오 폴더, 방향, s 구간 --bin m). s 는 그 방향 기준선(eval/routes/<route>_<방향>.csv) 위, 출발점 0.
  방향  file 이름 토큰 cw · ccw(run_<묶음>_<NN>.pqt, 묶음 이름 끝이 _cw · _ccw) → 없으면 시나리오 이름 토큰
        → 없으면 지도 위치(map_x · map_y)를 시계 기준선에 투영한 s 가 파일 안에서 늘면 cw · 줄면 ccw → 다 안 되면 '?'.
        route_s 자체로는 방향을 못 가림: sim/export_xmobility.py 가 반시계 녹화는 뒤집은 선(같은 출발점) 위에서 s 를 재서
        두 방향 모두 주행하면 s 가 늘어남.
  s     route_s → 파일 안에서 한 값뿐이거나(가짜 데이터) NaN 이면 지도 위치를 그 방향 기준선에 투영 → 둘 다 없으면 그 프레임은 빠짐.
        둘 다 있으면 차이(원형) 중앙값을 방향별로 출력 — 1 m 넘으면 방향 · 기준선이 틀렸을 수 있음.
지표(구간마다, 유한값만 — 정의는 eval_offline.metrics_block 그대로): N · 지남(파일 안에서 그 구간에 연속으로 머문 덩어리 수
  = 통과 횟수) · ω MAE · v MAE · ω 부호 일치(|ω 교사| > 0.1 rad/s) · ADE · 교사 |cte| 평균 · 장애물 근처 비율
  (obstacle_dist < 1.5 m 인 프레임 / N, 그 구간에 obstacle_dist 값이 하나도 없으면 빈칸) · 꺾임(기준선 yaml commands 구간과 겹침).
순위: --rank(기본 ω MAE, 부호 일치는 낮은 쪽이 나쁨) 나쁜 순. N < --min-n 구간은 순위에서 뺌(CSV 에는 남음).
두 폴더(--a 제로샷 --b 파인튜닝): 구간별 b − a, 순위는 b 기준. 나빠진 구간 = 두 쪽 다 N ≥ --min-n 이고 Δ(rank 지표)가
  --worse 보다 나쁜 쪽으로 큰 구간. 두 평가의 프레임 집합(시나리오 · 파일 · idx)이 다르면 경고.
출력(-o, 기본 --b 아니면 --a 폴더): 화면 표 · weak_segments.csv(전 구간) · weak_segments.md · weak_segments.png(matplotlib 있을 때,
  방향별로 기준선을 구간 ω MAE 색으로, 두 폴더면 a · b · b−a, 시나리오는 합침 — --plot-scenario 로 하나만. 글자는 영어).

사용:
  python3 e2e/weak_segments.py --a logs/xm_eval_zeroshot
  python3 e2e/weak_segments.py --a logs/xm_eval_zeroshot --b logs/xm_eval_v0.1 -o logs/weak_v0.1 [--bin 2.0]
필요: numpy · pandas (그림은 matplotlib 선택). frames.csv 에 route_s 또는 map_x · map_y 열이 있어야 함(eval_offline.py 위치 열
  들어간 판). 없으면 구간을 못 나눠 멈춤 — 그 판으로 평가를 다시 돌릴 것.
"""
import argparse
import json
import math
import os
import re
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)            # python -I 로 돌려도 eval_offline 을 찾게
from eval_offline import NEAR_OBS_M, REPO, W_SIGN_THR, _pad, _w, finite_mean, metrics_block  # noqa: E402

DIRS = ("cw", "ccw")
NEED = ("scenario", "file", "idx", "v_teacher", "w_teacher", "v_pred", "w_pred")
OPTIONAL = ("route_s", "map_x", "map_y", "lap", "cte", "obstacle_dist", "ade_m")
KEYS = ["scenario", "direction", "bin"]
RANK = {"w_mae": ("ω MAE", "w MAE", False), "v_mae": ("v MAE", "v MAE", False),
        "w_sign_agree": ("ω 부호", "w sign agree", True), "ade_m": ("ADE m", "ADE m", False)}   # (표, 그림, 높을수록 좋음)
METRICS = ["v_mae", "w_mae", "w_sign_agree", "n_w_sign", "ade_m"]
MAP_DIR_MIN_M = 1.0                     # 지도 투영 s 가 파일 안에서 이만큼 넘게 늘거나 줄어야 방향으로 침
S_CHECK_WARN_M = 1.0


# ---------------------------------------------------------------- 기준선

class Route:
    """닫힌 기준선 — 점 (M, 2), 누적 길이 cum (M,), 경로 지시 구간(yaml commands)."""

    def __init__(self, xy, commands=(), name=""):
        xy = np.asarray(xy, np.float64)
        if np.hypot(*(xy[-1] - xy[0])) > 1e-6:
            xy = np.vstack([xy, xy[:1]])
        self.xy, self.name, self.commands = xy, name, list(commands)
        self.cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
        self.L = float(self.cum[-1])

    def at(self, s):
        s = np.mod(np.asarray(s, np.float64), self.L)
        return np.stack([np.interp(s, self.cum, self.xy[:, 0]), np.interp(s, self.cum, self.xy[:, 1])], axis=-1)

    def project(self, x, y, chunk=4096):
        """점들 → 가장 가까운 선분 위 s (N,) · 거리 (N,). NaN 점은 NaN."""
        x, y = np.asarray(x, np.float64), np.asarray(y, np.float64)
        s_out, d_out = np.full(len(x), np.nan), np.full(len(x), np.nan)
        a, d = self.xy[:-1], np.diff(self.xy, axis=0)
        l2 = np.maximum((d ** 2).sum(1), 1e-12)
        ok = np.flatnonzero(np.isfinite(x) & np.isfinite(y))
        for k in range(0, len(ok), chunk):
            i = ok[k:k + chunk]
            px, py = x[i, None] - a[:, 0], y[i, None] - a[:, 1]
            t = np.clip((px * d[:, 0] + py * d[:, 1]) / l2, 0.0, 1.0)
            d2 = (px - t * d[:, 0]) ** 2 + (py - t * d[:, 1]) ** 2
            j = d2.argmin(1)
            r = np.arange(len(i))
            s_out[i] = np.mod(self.cum[j] + t[r, j] * np.sqrt(l2[j]), self.L)
            d_out[i] = np.sqrt(d2[r, j])
        return s_out, d_out

    def corners(self, lo, hi):
        """[lo, hi) 와 겹치는 경로 지시 구간 이름 — 'R-97°@12.9'(from_s > to_s 면 바퀴 이음을 넘는 구간)."""
        out = []
        for c in self.commands:
            f, t = c["from_s"] % self.L, c["to_s"] % self.L
            parts = [(f, t)] if f <= t else [(f, self.L), (0.0, t)]
            if any(p0 < hi and lo < p1 for p0, p1 in parts):
                out.append(f"{c['cmd'][:1].upper()}{c['turn_deg']:+.0f}°@{c['apex_s']:g}")
        return ",".join(out)


def read_commands(path):
    """make_refline.py 가 만든 yaml 의 commands 줄('  - {from_s: …, to_s: …, cmd: …, turn_deg: …, apex_s: …}') — PyYAML 없이."""
    out = []
    if not os.path.isfile(path):
        return out
    with open(path, encoding="utf-8") as f:
        for line in f:
            m = re.match(r"\s*-\s*\{(.*)\}", line.split("#")[0])
            if not m:
                continue
            d = dict(kv.split(":", 1) for kv in m.group(1).split(",") if ":" in kv)
            d = {k.strip(): v.strip() for k, v in d.items()}
            if {"from_s", "to_s", "cmd"} <= set(d):
                out.append(dict(from_s=float(d["from_s"]), to_s=float(d["to_s"]), cmd=d["cmd"],
                                turn_deg=float(d.get("turn_deg", "nan")), apex_s=float(d.get("apex_s", "nan"))))
    return out


def read_route_csv(path):
    df = pd.read_csv(path)
    if "run" in df.columns:
        df = df[df["run"] == 0]           # sim/export_xmobility.py load_csv 와 같음
    return df[["x", "y"]].to_numpy(np.float64)


def load_routes(folder, name):
    """{cw: Route|None, ccw: Route|None}. ccw CSV 가 없으면 cw 를 뒤집음(같은 출발점, export_xmobility.py:162-163 과 같은 뜻)."""
    out = {}
    for d in DIRS:
        csv = os.path.join(folder, f"{name}_{d}.csv")
        out[d] = (Route(read_route_csv(csv), read_commands(os.path.join(folder, f"{name}_{d}.yaml")), f"{name}_{d}")
                  if os.path.isfile(csv) else None)
    if out["ccw"] is None and out["cw"] is not None:
        out["ccw"] = Route(out["cw"].xy[::-1], read_commands(os.path.join(folder, f"{name}_ccw.yaml")), f"{name}_ccw(cw 뒤집음)")
    return out


# ---------------------------------------------------------------- 방향 · s · 구간

def direction_from_name(name):
    tok = re.split(r"[^a-z0-9]+", str(name).lower())
    return "ccw" if "ccw" in tok else ("cw" if "cw" in tok else None)


def direction_from_map(x, y, route_cw):
    """시계 기준선에 투영한 s 의 프레임 간 변화(원형, 2 m 넘는 점프는 뺌) 합 > +1 m → cw, < −1 m → ccw."""
    if route_cw is None:
        return None
    s, _ = route_cw.project(x, y)
    s = s[np.isfinite(s)]
    if len(s) < 3:
        return None
    ds = np.mod(np.diff(s) + route_cw.L / 2, route_cw.L) - route_cw.L / 2
    tot = float(ds[np.abs(ds) < 2.0].sum())
    return "cw" if tot > MAP_DIR_MIN_M else ("ccw" if tot < -MAP_DIR_MIN_M else None)


def circ_diff(a, b, L):
    return np.abs(np.mod(a - b + L / 2, L) - L / 2)


def locate(frames, routes, width):
    """frames.csv 표 → direction · dir_src · s · s_src · bin · s_lo · s_hi · visit 열을 붙인 표 + 통계 dict. s 없는 행은 뺌."""
    df = frames.copy()
    df["direction"], df["dir_src"], df["s"], df["s_src"] = "?", "", np.nan, ""
    st = dict(frames=len(df), dir_src={}, s_src={}, dropped_no_s=0, s_check={}, constant_route_s_files=[])
    check = {d: [] for d in DIRS}
    for (scen, fn), g in df.groupby(["scenario", "file"], sort=False):
        i = g.index
        mx, my = g["map_x"].to_numpy(np.float64), g["map_y"].to_numpy(np.float64)
        map_ok = np.isfinite(mx) & np.isfinite(my)
        if map_ok.sum() >= 3 and np.ptp(mx[map_ok]) + np.ptp(my[map_ok]) < 1e-6:
            map_ok[:] = False                                  # 지도 위치가 한 점뿐(가짜 데이터의 0 등)
        d, src = direction_from_name(fn), "file"
        if d is None:
            d, src = direction_from_name(scen), "scenario"
        if d is None:
            d, src = direction_from_map(np.where(map_ok, mx, np.nan), my, routes.get("cw")), "map"
        if d is None:
            d, src = "?", "unknown"
        df.loc[i, "direction"], df.loc[i, "dir_src"] = d, src
        rs = g["route_s"].to_numpy(np.float64)
        fin = np.isfinite(rs)
        if fin.sum() >= 3 and np.ptp(rs[fin]) < 1e-6:
            st["constant_route_s_files"].append(f"{scen}/{fn}")
            fin[:] = False
        s, s_src = np.where(fin, rs, np.nan), np.where(fin, "route_s", "").astype(object)
        r = routes.get(d)
        if r is not None and map_ok.any():
            sp, _ = r.project(np.where(map_ok, mx, np.nan), my)
            both = fin & np.isfinite(sp)
            check[d].append(circ_diff(rs[both], sp[both], r.L))
            fill = ~fin & np.isfinite(sp)
            s[fill], s_src[fill] = sp[fill], "map"
        df.loc[i, "s"], df.loc[i, "s_src"] = s, s_src
    for d in DIRS:
        if check[d] and sum(len(c) for c in check[d]):
            c = np.concatenate(check[d])
            st["s_check"][d] = dict(n=int(len(c)), median_m=float(np.median(c)), p95_m=float(np.percentile(c, 95)))
    st["dir_src"] = {k: int(v) for k, v in df["dir_src"].value_counts().items()}
    keep = np.isfinite(df["s"].to_numpy(np.float64))
    st["s_src"] = {k: int(v) for k, v in df.loc[keep, "s_src"].value_counts().items()}
    st["dropped_no_s"] = int((~keep).sum())
    df = df[keep].copy()
    L = df["direction"].map(lambda d: routes[d].L if routes.get(d) is not None else np.nan).to_numpy(np.float64)
    s = df["s"].to_numpy(np.float64)
    s = np.where(np.isfinite(L), np.mod(s, np.where(np.isfinite(L), L, 1.0)), s)
    df["s"] = s
    df["bin"] = np.floor(s / width).astype(int)
    df["s_lo"] = df["bin"] * width
    df["s_hi"] = np.where(np.isfinite(L), np.minimum(df["s_lo"] + width, L), df["s_lo"] + width)
    df = df.sort_values(["scenario", "file", "idx"], kind="stable")
    key = df["scenario"].astype(str) + "/" + df["file"].astype(str)
    same = (key == key.shift()) & (df["bin"] == df["bin"].shift()) & (df["idx"] - df["idx"].shift() == 1)
    df["visit"] = ~same
    df["run_key"] = key
    return df, st


def bin_table(df, keys, routes, near_m=NEAR_OBS_M, w_thr=W_SIGN_THR):
    """구간별 지표 표."""
    rows = []
    for k, g in df.groupby(keys, sort=True):
        k = k if isinstance(k, tuple) else (k,)
        m = metrics_block(g["v_teacher"], g["w_teacher"], g["v_pred"], g["w_pred"], g["ade_m"], g["ade_m"], w_thr)
        od = pd.to_numeric(g["obstacle_dist"], errors="coerce")
        r = routes.get(g["direction"].iat[0])
        lo, hi = float(g["s_lo"].iat[0]), float(g["s_hi"].iat[0])
        rows.append(dict(zip(keys, k), s_lo=lo, s_hi=hi, corner=r.corners(lo, hi) if r is not None else "",
                         n=m["n"], n_visits=int(g["visit"].sum()), n_files=int(g["run_key"].nunique()),
                         v_mae=m["v_mae"], w_mae=m["w_mae"], w_sign_agree=m["w_sign_agree"], n_w_sign=m["n_w_sign"],
                         ade_m=m["ade_m"], cte_abs=finite_mean(np.abs(pd.to_numeric(g["cte"], errors="coerce"))),
                         near_obs_frac=float((od < near_m).sum() / len(g)) if od.notna().any() else None))
    cols = keys + ["s_lo", "s_hi", "corner", "n", "n_visits", "n_files"] + METRICS + ["cte_abs", "near_obs_frac"]
    return pd.DataFrame(rows, columns=cols).astype({c: "float64" for c in METRICS[:3] + ["ade_m", "cte_abs", "near_obs_frac"]})


def add_rank(t, metric, min_n, suffix=""):
    """N ≥ min_n 인 구간에 1부터 나쁜 순 순위(그 밖은 NaN)."""
    col, ncol = metric + suffix, "n" + suffix
    ok = (t[ncol] >= min_n) & t[col].notna()
    higher_good = RANK[metric][2]
    order = t[ok].sort_values([col, ncol], ascending=[higher_good, False], kind="stable").index
    t["rank"] = np.nan
    t.loc[order, "rank"] = np.arange(1, len(order) + 1)
    return t


def compare(ta, tb, metric, min_n, worse):
    """구간 표 둘 → 열 이름 _a · _b, 차 d_<지표> = b − a, worse 열. 교사 쪽 값(cte · 장애물 · 꺾임)은 b(없으면 a)."""
    side = ["n", "n_visits", "n_files"] + METRICS
    teach = ["s_lo", "s_hi", "corner", "cte_abs", "near_obs_frac"]
    m = ta[KEYS + teach + side].merge(tb[KEYS + teach + side], on=KEYS, how="outer", suffixes=("_a", "_b"), sort=True)
    for c in teach:
        m[c] = m[c + "_b"].where(m[c + "_b"].notna(), m[c + "_a"])
    for c in ("n", "n_visits", "n_files", "n_w_sign"):
        m[c + "_a"], m[c + "_b"] = m[c + "_a"].fillna(0).astype(int), m[c + "_b"].fillna(0).astype(int)
    for c in METRICS[:3] + ["ade_m"]:
        m["d_" + c] = m[c + "_b"] - m[c + "_a"]
    sgn = -1.0 if RANK[metric][2] else 1.0
    both = (m["n_a"] >= min_n) & (m["n_b"] >= min_n) & m["d_" + metric].notna()
    m["comparable"] = both
    m["worse"] = both & (sgn * m["d_" + metric] > worse)
    m["better"] = both & (sgn * m["d_" + metric] < -worse)
    cols = (KEYS + teach + [c + s for c in side for s in ("_a", "_b")] + ["d_" + c for c in METRICS[:3] + ["ade_m"]]
            + ["comparable", "worse", "better"])
    return add_rank(m[cols].copy(), metric, min_n, "_b")


# ---------------------------------------------------------------- 입력 · 출력

def load_eval(path):
    """eval_offline 출력 폴더(또는 frames.csv) → (frames, run_meta dict, 경로)."""
    fp = os.path.join(path, "frames.csv") if os.path.isdir(path) else path
    if not os.path.isfile(fp):
        sys.exit(f"frames.csv 없음: {fp}")
    df = pd.read_csv(fp)
    miss = [c for c in NEED if c not in df.columns]
    if miss:
        sys.exit(f"{fp}: 필수 열 없음 {miss} — e2e/eval_offline.py 의 frames.csv 인지 확인")
    absent = [c for c in OPTIONAL if c not in df.columns]
    for c in absent:
        df[c] = np.nan
    meta = {}
    mp = os.path.join(os.path.dirname(fp), "run_meta.json")
    if os.path.isfile(mp):
        try:
            with open(mp, encoding="utf-8") as f:
                meta = json.load(f)
        except (OSError, ValueError):
            meta = {}
    return df, dict(meta=meta, absent=absent, path=os.path.abspath(fp)), fp


def fmt(x, nd=3):
    return "-" if x is None or (isinstance(x, float) and not math.isfinite(x)) or pd.isna(x) else f"{x:.{nd}f}"


def md_table(head, rows):
    return "\n".join(["| " + " | ".join(head) + " |", "|" + " --- |" * len(head)] + ["| " + " | ".join(r) + " |" for r in rows])


def s_range(r):
    return f"{r['s_lo']:.1f}–{r['s_hi']:.1f}"


def describe_input(tag, info, st):
    m = info["meta"]
    bits = [f"`{info['path']}`", f"프레임 {st['frames']}"]
    if m:
        bits.append(f"split {m.get('split', '?')} · ckpt {m.get('checkpoint_sha12', '?')} · "
                    f"이전 명령 {'교사' if m.get('teacher_forcing') else '직전 예측'}")
    lines = [f"- {tag}: " + " · ".join(bits)]
    srcs = " · ".join(f"{k or '없음'} {v}" for k, v in st["s_src"].items())
    lines.append(f"  - s 출처: {srcs or '-'} · s 없어 뺀 프레임 {st['dropped_no_s']} · 방향 출처(프레임): "
                 + " · ".join(f"{k} {v}" for k, v in st["dir_src"].items()))
    for d, c in st["s_check"].items():
        warn = " — **1 m 넘음: 방향 · 기준선 확인**" if c["median_m"] > S_CHECK_WARN_M else ""
        lines.append(f"  - {d}: route_s vs 지도 위치 투영 차 중앙값 {c['median_m']:.3f} m · P95 {c['p95_m']:.3f} m (n {c['n']}){warn}")
    if st["constant_route_s_files"]:
        lines.append(f"  - route_s 가 한 값뿐인 파일 {len(st['constant_route_s_files'])}개(지도 투영으로 대신): "
                     + ", ".join(st["constant_route_s_files"][:5]))
    if info["absent"]:
        lines.append(f"  - frames.csv 에 없는 열(빈칸 처리): {', '.join(info['absent'])}")
    return lines


def report(t, a, label, metric, top, min_n, worse_thr, inputs, routes, two):
    mk = RANK[metric][0]
    L = " · ".join(f"{d} {routes[d].name} L {routes[d].L:.2f} m · 꺾임 {len(routes[d].commands)}" for d in DIRS if routes.get(d))
    lines = [f"# 약한 구간 — {label}", ""] + inputs + [
        f"- 구간: s 폭 {a.bin:g} m · 기준선 {L or '없음(꺾임 · 그림 · 지도 투영 없음)'}",
        f"- 순위: {mk} 나쁜 순, N ≥ {min_n} 구간만{' · b 기준' if two else ''}. ω 부호는 |ω 교사| > {W_SIGN_THR} rad/s 프레임, "
        f"장애물 근처 = obstacle_dist < {NEAR_OBS_M} m 비율 · 지남 = 통과 횟수", ""]
    ranked = t[t["rank"].notna()].sort_values("rank").head(top)
    sfx = "_b" if two else ""
    if two:
        head = ["순위", "시나리오", "방향", "s m", "꺾임", "N", "지남", "ω MAE a", "ω MAE b", "Δω", "v MAE b", "Δv",
                "ω 부호 b", "Δ부호", "교사 cte m", "장애물 근처"]
        row = lambda r: [f"{int(r['rank'])}" if pd.notna(r.get("rank")) else "-", str(r["scenario"]), r["direction"],  # noqa: E731
                         s_range(r), r["corner"] or "", str(r["n_b"]), str(r["n_visits_b"]), fmt(r["w_mae_a"]),
                         fmt(r["w_mae_b"]), fmt(r["d_w_mae"]), fmt(r["v_mae_b"]), fmt(r["d_v_mae"]),
                         fmt(r["w_sign_agree_b"], 2), fmt(r["d_w_sign_agree"], 2), fmt(r["cte_abs"]), fmt(r["near_obs_frac"], 2)]
    else:
        head = ["순위", "시나리오", "방향", "s m", "꺾임", "N", "지남", "ω MAE", "v MAE", "ω 부호", "(n)", "ADE m",
                "교사 cte m", "장애물 근처"]
        row = lambda r: [f"{int(r['rank'])}" if pd.notna(r.get("rank")) else "-", str(r["scenario"]), r["direction"],  # noqa: E731
                         s_range(r), r["corner"] or "", str(r["n"]), str(r["n_visits"]), fmt(r["w_mae"]), fmt(r["v_mae"]),
                         fmt(r["w_sign_agree"], 2), str(r["n_w_sign"]), fmt(r["ade_m"]), fmt(r["cte_abs"]),
                         fmt(r["near_obs_frac"], 2)]
    n_ok = int((t["n" + sfx] >= min_n).sum())
    lines += [f"## 나쁜 구간 상위 {min(top, len(ranked))} (전체 구간 {len(t)} · N ≥ {min_n} 인 구간 {n_ok})", "",
              md_table(head, [row(r) for _, r in ranked.iterrows()]) if len(ranked) else "(순위 낼 구간 없음)", ""]
    if two:
        comp = t[t["comparable"]]
        w = t[t["worse"]].sort_values("d_" + metric, ascending=RANK[metric][2])
        lines += [f"## 나빠진 구간 (b − a, Δ{mk} {'<' if RANK[metric][2] else '>'} "
                  f"{'-' if RANK[metric][2] else '+'}{worse_thr:g})", "",
                  f"비교 가능한 구간(두 쪽 다 N ≥ {min_n}) {len(comp)} · 나빠짐 {len(w)} · 좋아짐 {int(t['better'].sum())} · "
                  f"그 사이 {len(comp) - len(w) - int(t['better'].sum())}", ""]
        lines += [md_table(head, [row(r) for _, r in w.head(max(top, 20)).iterrows()]) if len(w) else "(없음)", ""]
    lines += ["## 읽을 때", "",
              "- 개루프 지표 — 입력은 교사가 실제로 지나간 상태라 모델 오차가 쌓이지 않음. 폐루프 실패 위치는 시뮬레이터에서 따로 확인.",
              "- 한 구간 안 프레임은 한 번 지날 때 이어진 것이라 서로 독립이 아님 — 지남(통과 횟수)이 적은 구간의 작은 차이는 믿지 말 것.",
              "- 교사 cte(기준선 이탈 절댓값 평균)가 큰 구간은 교사 잡음 · 복귀 동작이 많은 곳 — 오차가 모델보다 교사 쪽 특이 동작 때문일 수 있음.",
              "- s 는 방향마다 그 방향 기준선 위(출발점 0)라 cw 와 ccw 의 같은 s 는 다른 곳. 그림(weak_segments.png)에서 위치 확인."]
    return "\n".join(lines) + "\n"


def term(md):
    """md 표를 화면용으로 칸 맞춤(한글 폭 2)."""
    out, block = [], []

    def flush():
        if block:
            cells = [[c.strip() for c in ln.strip().strip("|").split("|")] for ln in block if not set(ln) <= set("|- ")]
            k = max(len(r) for r in cells)
            cells = [r + [""] * (k - len(r)) for r in cells]
            wd = [max(_w(r[i]) for r in cells) for i in range(k)]
            out.extend("  ".join(_pad(c, wd[i]) for i, c in enumerate(r)).rstrip() for r in cells)
            block.clear()
    for ln in md.splitlines():
        if ln.startswith("|"):
            block.append(ln)
        else:
            flush()
            out.append(ln)
    flush()
    return "\n".join(out)


def outward(r, s, off):
    """기준선 위 s 점을 닫힌 선 바깥쪽으로 off m 민 점 (N, 2). 반시계로 도는 선(넓이 > 0)이면 왼쪽 법선이 안쪽."""
    p, t = r.at(s), r.at(s + 0.1) - r.at(s - 0.1)
    left = np.stack([-t[:, 1], t[:, 0]], 1) / np.maximum(np.hypot(t[:, 0], t[:, 1]), 1e-9)[:, None]
    x, y = r.xy[:, 0], r.xy[:, 1]
    area = 0.5 * float(np.sum(x[:-1] * y[1:] - x[1:] * y[:-1]))
    return p + (-off if area > 0 else off) * left


def plot(path, tables, routes, min_n, title):
    """방향(행) × 표(열 — a, b, b−a) 그림. tables: [(이름, 표, 차이인가)] — 표는 direction · s_lo · s_hi · n · w_mae(또는 d_w_mae)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
    except ImportError:
        print("matplotlib 없음 — 그림 건너뜀")
        return None
    dirs = [d for d in DIRS if routes.get(d) is not None and any((t["direction"] == d).any() for _, t, _ in tables)]
    if not dirs:
        print("그릴 기준선 없음(방향 모름 또는 eval/routes CSV 없음) — 그림 건너뜀")
        return None
    vals = [t["w_mae"].to_numpy(np.float64) for _, t, diff in tables if not diff]
    vmax = np.nanmax(np.concatenate(vals)) if vals and np.isfinite(np.concatenate(vals)).any() else 1.0
    dvals = [t["d_w_mae"].to_numpy(np.float64) for _, t, diff in tables if diff]
    dmax = np.nanmax(np.abs(np.concatenate(dvals))) if dvals and np.isfinite(np.concatenate(dvals)).any() else 1.0
    vmax, dmax = max(float(vmax), 1e-6), max(float(dmax), 1e-6)
    fig, axes = plt.subplots(len(dirs), len(tables), figsize=(5.6 * len(tables), 5.0 * len(dirs)), squeeze=False)
    for i, d in enumerate(dirs):
        r = routes[d]
        for j, (name, t, diff) in enumerate(tables):
            ax = axes[i][j]
            ax.plot(r.xy[:, 0], r.xy[:, 1], color="0.85", lw=1, zorder=1)
            t = t[t["direction"] == d]
            col = "d_w_mae" if diff else "w_mae"
            ncol = [c for c in ("n_b", "n") if c in t.columns][0]
            good = t[(t[ncol] >= min_n) & t[col].notna()]
            if diff:
                good = good[good["n_a"] >= min_n]
            sparse = t.drop(index=good.index)
            for _, b in sparse.iterrows():
                xy = r.at(np.linspace(b["s_lo"], b["s_hi"], 6))
                ax.plot(xy[:, 0], xy[:, 1], color="0.6", lw=1.5, ls=":", zorder=2)
            if len(good):
                segs = [r.at(np.linspace(b["s_lo"], b["s_hi"], 6)) for _, b in good.iterrows()]
                lc = LineCollection(segs, cmap="coolwarm" if diff else "viridis", linewidths=7, zorder=3)
                lc.set_array(good[col].to_numpy(np.float64))
                if diff:
                    lc.set_clim(-dmax, dmax)
                else:
                    lc.set_clim(0.0, vmax)
                ax.add_collection(lc)
                fig.colorbar(lc, ax=ax, shrink=0.75, label=("w MAE b - a (rad/s)" if diff else "w MAE (rad/s)"))
            for c in r.commands:                                   # 꺾임 창은 선 바깥쪽 1.2 m 옆에 검은 선, 꼭짓점 x
                f, to = c["from_s"] % r.L, c["to_s"] % r.L
                ss = np.linspace(f, to if to >= f else to + r.L, 30)
                xy = outward(r, ss, 1.2)
                ax.plot(xy[:, 0], xy[:, 1], color="k", lw=1.5, zorder=4)
                pa = outward(r, np.array([c["apex_s"]]), 1.2)[0]
                ax.plot(*pa, marker="x", color="k", ms=6, zorder=5)
                ax.annotate(f"{c['cmd'][:1].upper()}{c['turn_deg']:+.0f} s={c['apex_s']:g}", pa,
                            fontsize=7, xytext=(3, 3), textcoords="offset points", zorder=6)
            p0, p1 = r.at(0.0), r.at(2.5)
            ax.annotate("", xy=p1, xytext=p0, arrowprops=dict(arrowstyle="->", color="red", lw=1.5), zorder=6)
            ax.annotate("s=0", p0, fontsize=7, color="red", xytext=(-18, -10), textcoords="offset points")
            ax.set_aspect("equal")
            ax.margins(0.12)
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_title(f"{d} | {name}", fontsize=10)
            ax.grid(alpha=0.2)
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def ascii(s):
    return str(s).encode("ascii", "replace").decode()


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="약한 구간 찾기 — eval_offline frames.csv 를 기준선 s 구간별로 (E)")
    ap.add_argument("--a", required=True, help="eval_offline 출력 폴더(또는 frames.csv) — 비교면 제로샷 쪽")
    ap.add_argument("--b", default=None, help="비교할 두 번째 폴더(파인튜닝 쪽). 차 = b − a")
    ap.add_argument("-o", "--out", default=None, help="출력 폴더(기본: --b, 없으면 --a 폴더)")
    ap.add_argument("--bin", type=float, default=2.0, help="s 구간 폭 m")
    ap.add_argument("--min-n", type=int, default=10, help="순위 · 비교에 넣을 구간 최소 프레임 수")
    ap.add_argument("--top", type=int, default=10)
    ap.add_argument("--rank", choices=list(RANK), default="w_mae")
    ap.add_argument("--worse", type=float, default=0.02, help="나빠짐 판정 문턱(rank 지표 단위, 기본 0.02)")
    ap.add_argument("--routes", default=os.path.join(REPO, "eval", "routes"), help="기준선 CSV · yaml 폴더")
    ap.add_argument("--route", default="parking_r3", help="기준선 이름 — <이름>_cw.csv · <이름>_ccw.csv(+ .yaml)")
    ap.add_argument("--plot-scenario", default=None, help="그림에 이 시나리오만(기본: 시나리오 합침)")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args(argv)
    if not a.bin > 0:
        ap.error("--bin 은 0 보다 커야 함")
    return a


def run(a, log=print):
    routes = load_routes(a.routes, a.route)
    if not any(routes.values()):
        log(f"기준선 없음: {a.routes}/{a.route}_{{cw,ccw}}.csv — route_s 만으로 구간을 나누고, 꺾임 · 지도 투영 · 그림은 건너뜀")
    evals = []
    for tag, p in (("a", a.a), ("b", a.b)):
        if p is None:
            continue
        fr, info, fp = load_eval(p)
        loc, st = locate(fr, routes, a.bin)
        if loc.empty:
            sys.exit(f"{fp}: 구간을 정할 프레임이 없음 — route_s · map_x/map_y 가 없거나 한 값뿐"
                     f"(없는 열: {', '.join(info['absent']) or '없음'}, route_s 한 값뿐인 파일 {len(st['constant_route_s_files'])}개). "
                     f"위치 열이 들어간 e2e/eval_offline.py 로 다시 평가할 것")
        evals.append((tag, fr, info, loc, st))
    two = len(evals) == 2
    out = os.path.abspath(a.out or (os.path.dirname(evals[-1][2]["path"])))
    os.makedirs(out, exist_ok=True)
    tabs = {tag: bin_table(loc, KEYS, routes) for tag, _, _, loc, _ in evals}
    inputs = sum((describe_input(tag, info, st) for tag, _, info, _, st in evals), [])
    if two:
        fa, fb = (set(zip(fr["scenario"].astype(str), fr["file"].astype(str), fr["idx"])) for _, fr, _, _, _ in evals)
        if fa != fb:
            inputs.append(f"- **경고: 두 평가의 프레임이 다름** — a 만 {len(fa - fb)} · b 만 {len(fb - fa)} · 공통 {len(fa & fb)} "
                          "(split · --max-files 확인). 구간 차에 데이터 차이가 섞임")
        ma, mb = evals[0][2]["meta"], evals[1][2]["meta"]
        for k in ("split", "teacher_forcing", "max_files"):
            if ma and mb and ma.get(k) != mb.get(k):
                inputs.append(f"- **경고: {k} 가 다름** — a {ma.get(k)} · b {mb.get(k)}")
        t = compare(tabs["a"], tabs["b"], a.rank, a.min_n, a.worse)
        label = f"{os.path.basename(os.path.dirname(evals[0][2]['path']))} → {os.path.basename(os.path.dirname(evals[1][2]['path']))}"
    else:
        t = add_rank(tabs["a"], a.rank, a.min_n)
        label = os.path.basename(os.path.dirname(evals[0][2]["path"]))
    t.to_csv(os.path.join(out, "weak_segments.csv"), index=False)
    md = report(t, a, label, a.rank, a.top, a.min_n, a.worse, inputs, routes, two)
    with open(os.path.join(out, "weak_segments.md"), "w", encoding="utf-8") as f:
        f.write(md)
    log(term(md))
    png = None
    sels = [loc if a.plot_scenario is None else loc[loc["scenario"] == a.plot_scenario] for _, _, _, loc, _ in evals]
    if not a.no_plot and not all(len(s) for s in sels):
        log(f"그림: --plot-scenario {a.plot_scenario} 프레임 없음 — 건너뜀")
    elif not a.no_plot:
        pooled = [bin_table(s, ["direction", "bin"], routes) for s in sels]
        tables = [(f"{tag}: {ascii(os.path.basename(os.path.dirname(info['path'])))} | w MAE", tb, False)
                  for (tag, _, info, _, _), tb in zip(evals, pooled)]
        if two:
            d = compare(pooled[0].assign(scenario="*"), pooled[1].assign(scenario="*"), "w_mae", a.min_n, a.worse)
            tables.append(("b - a | w MAE", d, True))
        scen = a.plot_scenario or "all scenarios pooled"
        png = plot(os.path.join(out, "weak_segments.png"), tables, routes, a.min_n,
                   ascii(f"offline eval, w MAE per {a.bin:g} m bin along reference line ({scen}); dotted = N < {a.min_n}; "
                         "black line beside track = route-command turn window, x = apex; red arrow = start s=0, driving direction"))
    log(f"[끝] {out}/weak_segments.csv · weak_segments.md" + (" · weak_segments.png" if png else ""))
    return t, dict(out=out, png=png, stats={tag: st for tag, _, _, _, st in evals})


def main():
    run(parse_args())


if __name__ == "__main__":
    main()
