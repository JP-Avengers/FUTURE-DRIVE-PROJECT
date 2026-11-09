"""교사 구분 검사 — eval_offline 의 frames.csv 로 '같은 자리에서 모델이 두 교사를 구분해 다른 ω 를 내는가'를 잰다 (E)

  왜    REPORT 한계 13 · compare_11_7 ⑤: Nav2 녹화의 '덜 꺾기'는 두 교사가 같은 코너를 다르게 꺾는 자리에 몰려 있다.
        11.6 가설은 '모델이 같은 코너에서 두 교사를 평균한다' 였는데, v0.2(Nav2 ×3)가 자동 교사 코너 예측을 Nav2 쪽으로
        못 옮겨서(0.894 → 0.806) 모델이 입력(속도 · 직전 명령 · 영상)으로 교사를 이미 구분할 가능성이 생겼다. 이 스크립트가 그걸 잰다.
  데이터 두 교사가 같은 코스 · 같은 방향을 달린 test(v1.0 test = 자동 시계 5바퀴 + Nav2 슬라럼 시계 1바퀴). route_s 2 m 칸으로 묶어
        두 교사가 모두 --min-n 프레임 이상 있고 교사 평균 ω 차 |T_nav2 − T_auto| ≥ --dw 인 칸('교사가 다른 칸')만 본다.
  지표  추종비 r = (P_nav2 − P_auto) / (T_nav2 − T_auto)  (P = 그 칸 · 그 교사 프레임의 예측 평균, T = 교사 평균)
             r ≈ 1 교사마다 따라감(구분) · r ≈ 0 두 교사에 같은 값(평균 또는 한쪽만 배움)
        프레임 가중 평균(가중 = 칸의 두 교사 프레임 수 합)과 합산비(Σw·ΔP / Σw·ΔT) 둘 다 냄.
        같은 칸에서 교사별 예측 / 교사(합산), 교사별 칸 안 기울기(예측 ω 를 교사 ω 에 회귀).
        회귀: y = 예측 ω − 칸의 자동 교사 평균 을 속도 · 직전 ω(예측 = 평가 때 입력, 교사 = 학습 때 입력) · Nav2 표시에 회귀(계수 · R²).
        잡음 상태(teacher_noise == 0 / ≠ 0)로 나눠 같은 계산을 반복.
  입력  이름=frames.csv 여러 개(교사 값은 같은 test 여야 함 — 확인함). --dataset <root>/test 를 주면 parquet 행 수 · 파일 이름을 대조.

  python e2e/teacher_id.py ZS=<zs>/frames.csv v0.1=<v01>/frames.csv aw30=<aw>/frames.csv v0.2=<v02>/frames.csv \
      --dataset ~/Downloads/xm_v1_unzip/xm_v1/test -o e2e/results/teacher_id_11_7.md
"""
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd

NEAR = 3.0           # m — Nav2 프레임 중 장애물 3 m 안 비율(영상에 콘이 보이는지)
W_FIT = 0.1          # 칸 안 기울기용 교사 |ω| 문턱(steer_diag 와 같음)


def f(x, nd=3):
    return "—" if x is None or (isinstance(x, float) and not np.isfinite(x)) else f"{x:.{nd}f}"


def ols(X, y):
    """절편 포함 최소제곱 → (계수[절편, …], R²)."""
    A = np.column_stack([np.ones(len(y)), X])
    beta, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ beta
    tot = y - y.mean()
    return beta, 1.0 - (res @ res) / (tot @ tot) if (tot @ tot) > 0 else np.nan


def add_prev(d):
    """파일 안 idx 순으로 직전 프레임 ω(예측 · 교사). 첫 프레임은 0(eval_offline 은 run 시작마다 action 0)."""
    d = d.sort_values(["file", "idx"]).copy()
    g = d.groupby("file", sort=False)
    d["prev_w_pred"] = g.w_pred.shift(1).fillna(0.0)
    d["prev_w_teacher"] = g.w_teacher.shift(1).fillna(0.0)
    return d


def teacher_bins(d, auto, nav2, bin_m):
    """교사 쪽 칸 표(첫 run 기준 — 교사 값은 run 마다 같음)."""
    d = d.assign(s_bin=np.floor(d.route_s / bin_m) * bin_m)
    rows = []
    for s, g in d.groupby("s_bin"):
        a, n = g[g.scenario == auto], g[g.scenario == nav2]
        rows.append(dict(s=s, n_auto=len(a), n_nav2=len(n),
                         T_auto=a.w_teacher.mean(), T_nav2=n.w_teacher.mean(),
                         Tabs_auto=a.w_teacher.abs().mean(), Tabs_nav2=n.w_teacher.abs().mean(),
                         spd_auto=a.ego_speed.mean(), spd_nav2=n.ego_speed.mean(),
                         cte_auto=a.cte.abs().mean(), cte_nav2=n.cte.abs().mean(),
                         near_nav2=(n.obstacle_dist < NEAR).mean() if len(n) else np.nan,
                         noise_auto=(a.teacher_noise.abs() > 0).mean() if len(a) else np.nan,
                         noise_nav2=(n.teacher_noise.abs() > 0).mean() if len(n) else np.nan))
    t = pd.DataFrame(rows).set_index("s")
    t["dT"] = t.T_nav2 - t.T_auto
    return t


def select_bins(t, min_n, dw, fixed=None):
    if fixed is not None:
        t = t.loc[[s for s in fixed if s in t.index]]
        return t[(t.n_auto > 0) & (t.n_nav2 > 0)]
    ok = (t.n_auto >= min_n) & (t.n_nav2 >= min_n) & (t.dT.abs() >= dw)
    return t[ok]


def tracking(d, sel, auto, nav2, bin_m):
    """run 하나의 추종비 · 교사별 예측/교사 · 칸 안 기울기 · 칸별 값."""
    d = d.assign(s_bin=np.floor(d.route_s / bin_m) * bin_m)
    d = d[d.s_bin.isin(sel.index)]
    a, n = d[d.scenario == auto], d[d.scenario == nav2]
    per = []
    for s in sel.index:
        ga, gn = a[a.s_bin == s], n[n.s_bin == s]
        P_auto, P_nav2 = ga.w_pred.mean(), gn.w_pred.mean()
        dT = sel.loc[s, "dT"]
        per.append(dict(s=s, w=len(ga) + len(gn), P_auto=P_auto, P_nav2=P_nav2, dP=P_nav2 - P_auto,
                        r=(P_nav2 - P_auto) / dT))
    per = pd.DataFrame(per).set_index("s")
    w = per.w.to_numpy(float)
    dT = sel.dT.to_numpy(float)
    ok = np.isfinite(per.r.to_numpy(float))
    dP = per.dP.to_numpy(float)
    out = dict(per=per, n_auto=len(a), n_nav2=len(n),
               r_wmean=float((per.r[ok] * w[ok]).sum() / w[ok].sum()) if w[ok].sum() else np.nan,
               r_slope=float((w * dP * dT)[ok].sum() / (w * dT * dT)[ok].sum()) if ok.any() else np.nan,
               ratio_auto=a.w_pred.mean() / a.w_teacher.mean() if len(a) else np.nan,
               ratio_nav2=n.w_pred.mean() / n.w_teacher.mean() if len(n) else np.nan,
               mae_auto=(a.w_pred - a.w_teacher).abs().mean() if len(a) else np.nan,
               mae_nav2=(n.w_pred - n.w_teacher).abs().mean() if len(n) else np.nan)
    for k, g in (("slope_auto", a), ("slope_nav2", n)):
        t = g[g.w_teacher.abs() > W_FIT]
        out[k] = float(np.polyfit(t.w_teacher, t.w_pred, 1)[0]) if len(t) > 2 else np.nan
    # 회귀: y = 예측 ω − 칸의 자동 교사 평균
    T0 = d.s_bin.map(sel.T_auto)
    d = d.assign(y=d.w_pred - T0, nav2=(d.scenario == nav2).astype(float),
                 prev_t=d.prev_w_teacher - T0, prev_p=d.prev_w_pred - T0)
    d["d"] = d.nav2 * d.s_bin.map(sel.dT)
    reg = {}
    for name, cols in (("nav2", ["nav2"]), ("속도+prev_t+nav2", ["ego_speed", "prev_t", "nav2"]),
                       ("d", ["d"]), ("d+속도", ["d", "ego_speed"]), ("d+prev_t", ["d", "prev_t"]),
                       ("d+prev_p", ["d", "prev_p"]), ("d+속도+prev_t", ["d", "ego_speed", "prev_t"])):
        beta, r2 = ols(d[cols].to_numpy(float), d.y.to_numpy(float))
        reg[name] = (dict(zip(["절편"] + cols, beta)), r2)
    out["reg"] = reg
    out["frames"] = d
    return out


def speed_table(d, sel, auto, nav2, bin_m):
    d = d.assign(s_bin=np.floor(d.route_s / bin_m) * bin_m)
    d = d[d.s_bin.isin(sel.index)]
    rows = []
    for name, sc in (("자동", auto), ("Nav2", nav2)):
        g = d[d.scenario == sc]
        q = g.ego_speed.quantile([0.1, 0.5, 0.9])
        rows.append(f"| {name} | {len(g)} | {f(g.ego_speed.mean())} | {f(q[0.1])} · {f(q[0.5])} · {f(q[0.9])} "
                    f"| {f(g.prev_w_teacher.abs().mean())} | {f(g.cte.abs().mean())} |")
    c = np.corrcoef(d.ego_speed, (d.scenario == nav2).astype(float))[0, 1] if len(d) > 2 else np.nan
    rows.append("")
    rows.append(f"선택 칸 프레임에서 corr(속도, Nav2 표시) = {f(c, 2)} — 1 에 가까우면 회귀가 속도와 교사 표시를 못 가름")
    return rows


def analyse(runs, auto, nav2, bin_m, min_n, dw, fixed, title, lines):
    first = runs[0][1]
    if not len(first):
        lines.append(f"\n### {title}\n\n프레임 없음")
        return
    t = teacher_bins(first, auto, nav2, bin_m)
    sel = select_bins(t, min_n, dw, fixed)
    lines.append(f"\n### {title}")
    lines.append(f"\n프레임 자동 {int((first.scenario == auto).sum())} · Nav2 {int((first.scenario == nav2).sum())} · "
                 f"칸 {bin_m:g} m · 선택 칸 {len(sel)}개(두 교사 ≥ {min_n} 프레임 · |T_nav2 − T_auto| ≥ {dw})"
                 + (f" — 고정 칸 {list(sel.index.astype(int))}" if fixed is not None else "")
                 + f" · 선택 칸 프레임 자동 {int(sel.n_auto.sum())} · Nav2 {int(sel.n_nav2.sum())}")
    if not len(sel):
        return
    sel = sel.assign(typ=np.where(np.sign(sel.T_nav2) != np.sign(sel.T_auto), "부호 반대",
                                  np.where(sel.T_nav2.abs() > sel.T_auto.abs(), "같은 부호 · Nav2 더 셈", "같은 부호 · Nav2 더 약함")))
    lines.append("\n교사 쪽(칸별): T = 교사 평균 ω(rad/s) · 속도 m/s · |cte| m · 콘 3 m 안 = Nav2 프레임 중 obstacle_dist < 3 m 비율 · 잡음 = teacher_noise ≠ 0 비율\n")
    lines.append("| s | n 자동 · Nav2 | T 자동 | T Nav2 | ΔT | 속도 자동 · Nav2 | \\|cte\\| 자동 · Nav2 | 콘 3 m 안 | 잡음 자동 · Nav2 |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for s, r in sel.iterrows():
        lines.append(f"| {s:g} | {int(r.n_auto)} · {int(r.n_nav2)} | {r.T_auto:+.3f} | {r.T_nav2:+.3f} | {r.dT:+.3f} "
                     f"| {f(r.spd_auto, 2)} · {f(r.spd_nav2, 2)} | {f(r.cte_auto, 2)} · {f(r.cte_nav2, 2)} | {f(r.near_nav2, 2)} "
                     f"| {f(r.noise_auto, 2)} · {f(r.noise_nav2, 2)} |")
    res = {name: tracking(d, sel, auto, nav2, bin_m) for name, d in runs}
    lines.append("\n모델 쪽(선택 칸 합산): 추종비 r 가중 평균 · r 기울기(칸별 ΔP 를 ΔT 에 원점 통과 가중 회귀한 기울기 — ΔT 부호가 칸마다 달라 합산비 대신), "
                 "교사별 예측 / 교사(부호 포함 평균의 비), 칸 안 기울기(교사 |ω| > 0.1), ω MAE\n")
    lines.append("| 모델 | r 가중 평균 | r 기울기 | 예측/교사 자동 | 예측/교사 Nav2 | 기울기 자동 | 기울기 Nav2 | ω MAE 자동 · Nav2 |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for name, o in res.items():
        lines.append(f"| {name} | {f(o['r_wmean'], 2)} | {f(o['r_slope'], 2)} | {f(o['ratio_auto'])} | {f(o['ratio_nav2'])} "
                     f"| {f(o['slope_auto'], 2)} | {f(o['slope_nav2'], 2)} | {f(o['mae_auto'])} · {f(o['mae_nav2'])} |")
    lines.append("\n칸별 추종비 r (괄호 = 예측 평균 자동 → Nav2)\n")
    lines.append("| s | ΔT | " + " | ".join(n for n, _ in runs) + " |")
    lines.append("|---|---|" + "---|" * len(runs))
    for s in sel.index:
        cells = [f"{o['per'].loc[s, 'r']:+.2f} ({o['per'].loc[s, 'P_auto']:+.3f} → {o['per'].loc[s, 'P_nav2']:+.3f})" for o in res.values()]
        lines.append(f"| {s:g} | {sel.loc[s, 'dT']:+.3f} | " + " | ".join(cells) + " |")
    lines.append("\n칸 종류별 — r 가중 평균 · r 기울기 (예측/교사 자동 · Nav2). '같은 부호 · Nav2 더 셈' = Nav2 가 같은 방향으로 더 꺾는 코너(덜 꺾기 문제의 자리)\n")
    lines.append("| 종류 | 칸 | n 자동 · Nav2 | " + " | ".join(n for n, _ in runs) + " |")
    lines.append("|---|---|---|" + "---|" * len(runs))
    for t in ("같은 부호 · Nav2 더 셈", "부호 반대", "같은 부호 · Nav2 더 약함"):
        idx = sel.index[sel.typ == t]
        if not len(idx):
            continue
        cells = []
        for o in res.values():
            p = o["per"].loc[idx]
            w, r, dp, dt = (p.w.to_numpy(float), p.r.to_numpy(float), p.dP.to_numpy(float), sel.dT.loc[idx].to_numpy(float))
            ok = np.isfinite(r)
            fr = o["frames"][o["frames"].s_bin.isin(idx)]
            fa, fn = fr[fr.scenario == auto], fr[fr.scenario == nav2]
            cells.append(f"{f((r[ok] * w[ok]).sum() / w[ok].sum(), 2)} · {f((w * dp * dt)[ok].sum() / (w * dt * dt)[ok].sum(), 2)} "
                         f"({f(fa.w_pred.mean() / fa.w_teacher.mean(), 2)} · {f(fn.w_pred.mean() / fn.w_teacher.mean(), 2)})")
        lines.append(f"| {t} | {len(idx)} ({', '.join(f'{s:g}' for s in idx)}) | {int(sel.loc[idx, 'n_auto'].sum())} · {int(sel.loc[idx, 'n_nav2'].sum())} | "
                     + " | ".join(cells) + " |")
    lines.append("\n선택 칸의 교사별 입력 분포(속도 · 직전 교사 |ω| · |cte|)\n")
    lines.append("| 교사 | 프레임 | 속도 평균 | 속도 p10 · p50 · p90 | 직전 교사 \\|ω\\| 평균 | \\|cte\\| 평균 |")
    lines.append("|---|---|---|---|---|---|")
    lines += speed_table(first, sel, auto, nav2, bin_m)
    lines.append("\n회귀 y = 예측 ω − 칸의 자동 교사 평균 (선택 칸 프레임, 절편 포함): 계수 · R². nav2 = Nav2 표시(0/1, ΔT 부호를 모름), "
                 "d = nav2 × ΔT(칸) (Nav2 를 그대로 따르면 계수 1 = r), prev_t · prev_p = 직전 교사 ω · 직전 예측 ω − 칸의 자동 교사 평균\n")
    lines.append("| 모델 | 설명변수 | 계수 | R² |")
    lines.append("|---|---|---|---|")
    for name, o in res.items():
        for rn, (coef, r2) in o["reg"].items():
            cs = " · ".join(f"{k} {v:+.3f}" for k, v in coef.items() if k != "절편")
            lines.append(f"| {name} | {rn} | {cs} | {f(r2)} |")


def sanity(runs, dataset, lines):
    lines.append("\n## 확인 — 파일 · 프레임 수\n")
    first = runs[0][1]
    cnt = first.groupby(["scenario", "file"]).size()
    lines.append("| 시나리오 | 파일 | frames.csv | parquet |")
    lines.append("|---|---|---|---|")
    import pyarrow.parquet as pq
    pq_rows = {}
    if dataset:
        for p in sorted(glob.glob(os.path.join(dataset, "*", "*.pqt"))):
            pq_rows[os.path.basename(p)] = pq.ParquetFile(p).metadata.num_rows
    for (sc, fn), n in cnt.items():
        lines.append(f"| {sc} | `{fn}` | {n} | {pq_rows.get(fn, '—')} |")
    extra = sorted(set(pq_rows) - set(cnt.index.get_level_values(1)))
    if extra:
        lines.append(f"\nparquet 에만 있는 파일: {extra}")
    # 교사 값 · 입력이 run 사이에 같은지
    for name, d in runs[1:]:
        m = first.merge(d, on=["file", "idx"], suffixes=("", "_b"))
        lines.append(f"- {runs[0][0]} vs {name}: 공통 프레임 {len(m)} / {len(first)} · 교사 ω 차 최대 {(m.w_teacher - m.w_teacher_b).abs().max():.1e} "
                     f"· 속도 차 최대 {(m.ego_speed - m.ego_speed_b).abs().max():.1e} · route_s 차 최대 {(m.route_s - m.route_s_b).abs().max():.1e}")
    if dataset and pq_rows:
        # parquet 의 ego_speed · route_s · driving_command[5] 가 frames.csv 와 같은지(한 파일씩)
        worst = 0.0
        for fn in cnt.index.get_level_values(1):
            hits = glob.glob(os.path.join(dataset, "*", fn))
            if not hits:
                continue
            t = pq.read_table(hits[0], columns=["ego_speed", "route_s", "driving_command"]).to_pandas()
            g = first[first.file == fn].sort_values("idx")
            w = np.stack(t.driving_command.to_numpy())[:, 5]
            worst = max(worst, np.abs(t.ego_speed.to_numpy() - g.ego_speed.to_numpy()).max(),
                        np.abs(t.route_s.to_numpy() - g.route_s.to_numpy()).max(), np.abs(w - g.w_teacher.to_numpy()).max())
        lines.append(f"- parquet(ego_speed · route_s · driving_command[5]) vs frames.csv 최대 차 {worst:.1e}")


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="+", help="이름=frames.csv")
    ap.add_argument("--auto", default="r3_auto", help="자동 교사 시나리오 이름")
    ap.add_argument("--nav2", default="r3_nav2_obs", help="Nav2 시나리오 이름")
    ap.add_argument("--dir", default="cw", help="파일 이름의 방향 토막(_cw_ · _ccw_)만 사용")
    ap.add_argument("--bin", type=float, default=2.0, help="route_s 칸 m")
    ap.add_argument("--min-n", type=int, default=20, help="칸마다 교사별 최소 프레임")
    ap.add_argument("--dw", type=float, default=0.1, help="교사 평균 ω 차 문턱 rad/s")
    ap.add_argument("--min-v", type=float, default=0.1,
                    help="교사 v 가 이보다 작은(정지 · 제자리 회전) 프레임 제외 — Nav2 는 제자리 회전을 함(0 이면 안 뺌)")
    ap.add_argument("--bins", help="칸을 자동 선택하지 않고 고정(예 10,12,40,48,88)")
    ap.add_argument("--dataset", help="<root>/test — parquet 행 수 · 값 대조")
    ap.add_argument("-o", "--out", help="결과 .md")
    a = ap.parse_args()
    runs = []
    for r in a.runs:
        name, _, path = r.partition("=")
        if not path:
            sys.exit(f"이름=경로 형식이어야 함: {r}")
        d = pd.read_csv(path)
        d = d[d.file.str.contains(f"_{a.dir}_") & d.scenario.isin([a.auto, a.nav2])]
        runs.append((name, add_prev(d)))
    keep = set.intersection(*(set(d.file) for _, d in runs))
    runs = [(n, d[d.file.isin(keep)]) for n, d in runs]
    fixed = [float(x) for x in a.bins.split(",")] if a.bins else None

    lines = [f"# 교사 구분 검사 — {' · '.join(n for n, _ in runs)} ({a.dir}, 칸 {a.bin:g} m)"]
    sanity(runs, a.dataset, lines)
    if a.min_v > 0:
        d0 = runs[0][1]
        drop = d0[d0.v_teacher < a.min_v].groupby("scenario").size().to_dict()
        where = {sc: sorted(set((np.floor(g.route_s / a.bin) * a.bin).astype(int))) for sc, g in d0[d0.v_teacher < a.min_v].groupby("scenario")}
        lines.append(f"- 교사 v < {a.min_v} 프레임 제외(정지 · 제자리 회전): {drop} · 칸 {where}")
        runs = [(n, d[d.v_teacher >= a.min_v]) for n, d in runs]
    lines.append("\n## (a)(b) 전체 프레임")
    analyse(runs, a.auto, a.nav2, a.bin, a.min_n, a.dw, fixed, "전체", lines)
    lines.append("\n## (c) 잡음 상태별 (두 교사 모두 teacher_noise 로 나눔, 칸은 다시 선택)")
    for title, pick in (("잡음 없음 (teacher_noise == 0)", lambda d: d[d.teacher_noise.abs() == 0]),
                        ("잡음 중 (teacher_noise ≠ 0)", lambda d: d[d.teacher_noise.abs() > 0])):
        analyse([(n, pick(d)) for n, d in runs], a.auto, a.nav2, a.bin, a.min_n, a.dw, fixed, title, lines)
    text = "\n".join(lines) + "\n"
    print(text)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as fh:
            fh.write(text)


if __name__ == "__main__":
    main()
