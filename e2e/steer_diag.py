"""조향 진단 — eval_offline 의 frames.csv 로 '덜 꺾기 · 반대로 꺾기 · 지연'을 시나리오별로 비교 (E)

  왜    xm_ft v0.1 은 Nav2 장애물 구간에서 필요한 회전의 절반만 꺾음(REPORT_DRAFT 한계 13). v0.2(Nav2 ×3)가
        이걸 고쳤는지 같은 기준으로 재려고 만든 스크립트(v0.1 진단은 손으로 돌렸던 계산을 그대로 옮김).
  지표  기울기   교사 |ω| > 0.1 프레임에서 예측 ω 를 교사 ω 에 1차 회귀한 기울기(1 = 교사만큼, 0.5 = 절반만 꺾음)
        급회전   교사 |ω| > 0.3 프레임의 평균 |ω| 예측 / 교사, 부호 반대 비율
        지연     파일별로 예측을 k 프레임 늦춰 교사와 상관 — 최대인 k(0 이면 지연 없음)
  장애물  obstacle_dist 가 있는 시나리오는 '장애물 3 m 안 · 밖'으로도 나눔(급회전이 장애물 때문인지 코너 때문인지)
  공통  --common 이면 모든 입력에 있는 파일만 씀. test split 이 다른 결과(v1.0 vs v1.1)를 공정하게 비교할 때.

  python e2e/steer_diag.py ZS=<zs>/frames.csv v0.1=<v01>/eval_test/frames.csv v0.2=<v02>/eval_test/frames.csv --common
"""
import argparse
import sys

import numpy as np
import pandas as pd

W_FIT, W_BIG, MAX_LAG, NEAR = 0.1, 0.3, 5, 3.0


def best_lag(x):
    cs = []
    for k in range(MAX_LAG + 1):
        a, b = [], []
        for _, g in x.groupby("file", sort=False):
            g = g.sort_values("idx")
            if len(g) > k + 2:
                a.append(g.w_teacher.values[:len(g) - k])
                b.append(g.w_pred.values[k:])
        cs.append(np.corrcoef(np.concatenate(a), np.concatenate(b))[0, 1] if a else np.nan)
    return int(np.nanargmax(cs)) if not np.all(np.isnan(cs)) else -1


def diag(x):
    t, big = x[x.w_teacher.abs() > W_FIT], x[x.w_teacher.abs() > W_BIG]
    return dict(n=len(x),
                slope=np.polyfit(t.w_teacher, t.w_pred, 1)[0] if len(t) > 2 else np.nan,
                n_big=len(big),
                big_pred=big.w_pred.abs().mean() if len(big) else np.nan, big_teacher=big.w_teacher.abs().mean(),
                flip=(np.sign(big.w_pred) != np.sign(big.w_teacher)).mean() if len(big) else np.nan,
                lag=best_lag(x))


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="+", help="이름=frames.csv")
    ap.add_argument("--common", action="store_true", help="모든 입력에 공통인 파일만")
    ap.add_argument("-o", "--out", help="표를 이 .md 로도 저장")
    a = ap.parse_args()
    runs = []
    for r in a.runs:
        name, _, path = r.partition("=")
        if not path:
            sys.exit(f"이름=경로 형식이어야 함: {r}")
        runs.append((name, pd.read_csv(path)))
    note = ""
    if a.common:
        keep = set.intersection(*(set(d.file) for _, d in runs))
        drop = sorted(set().union(*(set(d.file) for _, d in runs)) - keep)
        runs = [(n, d[d.file.isin(keep)]) for n, d in runs]
        note = f"공통 파일 {len(keep)}개만 사용 · 뺀 파일: {', '.join(drop) or '없음'}"
    scen = sorted(set().union(*(set(d.scenario) for _, d in runs)))
    lines = ["| 시나리오 | 모델 | 프레임 | ω 기울기 | 급회전 수 | 급회전 ω 크기 예측 / 교사 | 부호 반대 | 지연(프레임) |",
             "|---|---|---|---|---|---|---|---|"]
    groups = []
    for sc in scen:
        groups.append((sc, lambda d, sc=sc: d[d.scenario == sc]))
        if any(d[d.scenario == sc].obstacle_dist.notna().any() for _, d in runs):
            groups.append((f"{sc} · 장애물 {NEAR} m 안", lambda d, sc=sc: d[(d.scenario == sc) & (d.obstacle_dist < NEAR)]))
            groups.append((f"{sc} · 장애물 {NEAR} m 밖", lambda d, sc=sc: d[(d.scenario == sc) & (d.obstacle_dist >= NEAR)]))
        if all("teacher_noise" in d.columns for _, d in runs) and any((d[d.scenario == sc].teacher_noise.abs() > 0).any() for _, d in runs):
            groups.append((f"{sc} · 잡음 중(이탈 상태)", lambda d, sc=sc: d[(d.scenario == sc) & (d.teacher_noise.abs() > 0)]))
    groups.append(("전체", lambda d: d))
    for label, pick in groups:
        for name, d in runs:
            x = pick(d)
            if len(x) < 3:
                continue
            m = diag(x)
            lines.append(f"| {label} | {name} | {m['n']} | {m['slope']:.2f} | {m['n_big']} | "
                         f"{m['big_pred']:.2f} / {m['big_teacher']:.2f} | {100 * m['flip']:.1f} % | {m['lag']} |")
    txt = "\n".join(([note, ""] if note else []) + lines
                    + ["", f"기울기: 교사 |ω|>{W_FIT} 프레임 1차 회귀 · 급회전: 교사 |ω|>{W_BIG} · 지연: 파일별 상관 최대 k(0–{MAX_LAG}) · 잡음 중 = teacher_noise≠0(교사가 일부러 벗어난 상태 — 폐루프와 가장 가까운 프레임, Codevilla 2018)"])
    print(txt)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as fh:
            fh.write(txt + "\n")


if __name__ == "__main__":
    main()
