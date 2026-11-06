"""오픈루프 적분 이탈 — frames.csv 의 예측 (v, ω) 와 교사 (v, ω) 를 같은 출발 자세에서 T 초 적분해 끝점 차이를 잰다 (E)

  왜    프레임별 MAE 는 '한쪽으로 조금씩 치우친 오차'가 쌓여 코스를 벗어나는 것을 못 본다(덜 꺾기 = 코너마다 바깥으로 밀림).
        폐루프 결과가 적을 때 모델 고르기(PLAN.md 규칙 2)에 쓰는 대리 지표. 폐루프 보장은 아님(영상은 교사 궤적의 것이라
        모델이 밀려난 자리에서 어떻게 되돌아오는지는 못 봄).
  방법  유니사이클 x += v cosθ dt · y += v sinθ dt · θ += ω dt, dt 0.2 s(sim/export_xmobility.py 5 Hz). 창 T 초(5·10 프레임),
        같은 파일 안에서만. 끝점 차이를 교사 끝 자세 기준 가로(lateral, 왼쪽 +) · 세로 · 방향(deg)으로 나눔.
  출력  시나리오 × 모델: |가로| 평균 · p95 · 부호 평균(한쪽으로 치우침) · |방향| 평균, 코너 창(교사 |ω| 평균 > 0.2) 따로.
        가로 이탈 큰 구간(방향 · route_s 5 m 칸) 상위 --top.
  --common  모든 입력에 있는 파일만(steer_diag 와 같음).

  python e2e/rollout_drift.py ZS=<zs>/frames.csv v0.1=<v01>/eval_test/frames.csv -o e2e/results/xm_ft_v0.1/rollout_drift.md
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

DT, WINDOWS_S, W_CORNER, BIN_M = 0.2, (1.0, 2.0), 0.2, 5.0


def integrate(v, w, H):
    """(n, H) 창마다 H 스텝 적분한 끝 자세 (x, y, θ) — 출발 (0, 0, 0). v·w 는 길이 n 의 1차원."""
    n = len(v) - H + 1
    if n <= 0:
        return None
    x = np.zeros(n); y = np.zeros(n); th = np.zeros(n)
    for k in range(H):
        vk, wk = v[k:k + n], w[k:k + n]
        x += vk * np.cos(th) * DT
        y += vk * np.sin(th) * DT
        th += wk * DT
    return x, y, th


def drift_file(g, H):
    g = g.sort_values("idx")
    p = integrate(g.v_pred.values, g.w_pred.values, H)
    t = integrate(g.v_teacher.values, g.w_teacher.values, H)
    if p is None:
        return None
    dx, dy = p[0] - t[0], p[1] - t[1]
    c, s = np.cos(t[2]), np.sin(t[2])
    lat, lon = -s * dx + c * dy, c * dx + s * dy
    dth = np.degrees((p[2] - t[2] + np.pi) % (2 * np.pi) - np.pi)
    n = len(lat)
    wmean = np.convolve(np.abs(g.w_teacher.values), np.ones(H) / H, mode="valid")[:n]
    if "teacher_noise" in g.columns:
        tn = np.abs(np.nan_to_num(pd.to_numeric(g.teacher_noise, errors="coerce").values.astype(np.float64)))
        noisy = np.convolve(tn, np.ones(H) / H, mode="valid")[:n] > 0
    else:
        noisy = np.zeros(n, bool)
    return pd.DataFrame(dict(file=g.file.values[:n], scenario=g.scenario.values[:n], route_s=g.route_s.values[:n],
                             lat=lat, lon=lon, dth=dth, corner=wmean > W_CORNER, noisy=noisy))


def summarize(d):
    a = d.lat.abs()
    return dict(n=len(d), lat_mean=a.mean(), lat_p95=a.quantile(0.95), lat_signed=d.lat.mean(), dth_mean=d.dth.abs().mean())


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="+", help="이름=frames.csv")
    ap.add_argument("--common", action="store_true", help="모든 입력에 공통인 파일만")
    ap.add_argument("--top", type=int, default=8, help="가로 이탈 큰 구간 수(마지막 모델 · 가장 긴 창)")
    ap.add_argument("-o", "--out", help="표를 이 .md 로도 저장")
    a = ap.parse_args()
    runs = []
    for r in a.runs:
        name, _, path = r.partition("=")
        if not path:
            sys.exit(f"이름=경로 형식이어야 함: {r}")
        runs.append((name, pd.read_csv(path)))
    lines = []
    if a.common:
        keep = set.intersection(*(set(d.file) for _, d in runs))
        drop = sorted(set().union(*(set(d.file) for _, d in runs)) - keep)
        runs = [(n, d[d.file.isin(keep)]) for n, d in runs]
        lines += [f"공통 파일 {len(keep)}개만 사용 · 뺀 파일: {', '.join(drop) or '없음'}", ""]
    scen = sorted(set().union(*(set(d.scenario) for _, d in runs)))
    lines += ["| 창 | 시나리오 | 모델 | 창 수 | 가로 이탈 평균 (m) | p95 | 부호 평균 | 방향 차 평균 (°) | 코너 창 가로 평균 | 코너 창 수 | 잡음 창 가로 평균 | 잡음 창 수 |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    last = None
    for T in WINDOWS_S:
        H = int(round(T / DT))
        for sc in scen + ["*"]:
            for name, d in runs:
                x = d if sc == "*" else d[d.scenario == sc]
                parts = [r for r in (drift_file(g, H) for _, g in x.groupby("file", sort=False)) if r is not None]
                if not parts:
                    continue
                dd = pd.concat(parts)
                m, c, nz = summarize(dd), dd[dd.corner], dd[dd.noisy]
                lines.append(f"| {T:g} s | {'전체' if sc == '*' else sc} | {name} | {m['n']} | {m['lat_mean']:.3f} | {m['lat_p95']:.3f} | "
                             f"{m['lat_signed']:+.3f} | {m['dth_mean']:.1f} | {c.lat.abs().mean() if len(c) else float('nan'):.3f} | {len(c)} | "
                             f"{nz.lat.abs().mean() if len(nz) else float('nan'):.3f} | {len(nz)} |")
                if sc == "*":
                    last = (name, T, dd)
    if last is not None:
        name, T, dd = last
        dd = dd.copy()
        dd["dir"] = np.where(dd.file.str.contains("_ccw"), "ccw", "cw")
        dd["bin"] = (np.floor(dd.route_s / BIN_M) * BIN_M).astype(int)
        g = dd.groupby(["scenario", "dir", "bin"]).lat.agg(n="count", mean_abs=lambda v: v.abs().mean(), signed="mean")
        g = g[g.n >= 10].sort_values("mean_abs", ascending=False).head(a.top)
        lines += ["", f"### {name} · {T:g} s 창 · 가로 이탈 큰 구간 상위 {len(g)} ({BIN_M:g} m 칸)", "",
                  "| 시나리오 | 방향 | s (m) | 창 수 | 가로 이탈 평균 | 부호 평균 |", "|---|---|---|---|---|---|"]
        for (sc, dr, b), r in g.iterrows():
            lines.append(f"| {sc} | {dr} | {b}–{b + int(BIN_M)} | {int(r.n)} | {r.mean_abs:.3f} | {r.signed:+.3f} |")
    lines += ["", f"유니사이클 적분 dt {DT} s · 가로 = 교사 끝 자세 기준 왼쪽 + · 코너 창 = 창 안 교사 |ω| 평균 > {W_CORNER} · 잡음 창 = 창 안에 teacher_noise≠0(교사가 일부러 벗어난 상태, 폐루프와 가장 가까움)"]
    txt = "\n".join(lines)
    print(txt)
    if a.out:
        os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
        with open(a.out, "w", encoding="utf-8") as fh:
            fh.write(txt + "\n")


if __name__ == "__main__":
    main()
