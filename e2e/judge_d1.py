"""결정 1(직전 명령 드롭아웃 채택) 판정 — PLAN.md '11.9 계획'에 결과 보기 전 고정한 기준을 그대로 계산 (E)

  입력  같은 v1.0 test 의 frames.csv(eval_offline 기본 평가 = 직전 예측 입력). 기준 = aw30, 후보 = E1 · E2 …
  기준  (후보 − aw30)  ② Nav2 ω 기울기 ≥ +0.04  그리고  ③ Nav2 2 s 가로 이탈 ≤ −0.008 m
                       그리고  ① 자동 교사 기울기 ≥ −0.012  그리고  ③ 자동 교사 2 s 이탈 ≤ +0.0032 m
        여럿 통과 → Nav2 2 s 이탈이 더 작은 쪽. 흔들림(예측 ω 초당 부호 바뀜, |ω|>0.05)이 aw30 의 1.5배 초과면
        탈락은 아니고 '노드 ω 평활 플래그 준비' 표시.
  계산  기울기 = steer_diag.diag, 이탈 = rollout_drift.drift_file(2 s = 10 프레임) — 기존 표와 같은 값.
  신뢰  --boot N(기본 2000, 0 이면 끔): 파일 안을 --block 프레임(기본 50 = 10 s) 덩어리로 나눠 덩어리를 복원 추출하는
        짝지은 블록 부트스트랩 — 기준 · 후보에 같은 덩어리를 뽑아 차(후보 − 기준)의 95 % 구간. test 프레임 표본의
        흔들림만 보며 시드 차(규칙 3)는 따로다 — 판정은 위 기준 그대로, 구간은 참고(0 을 걸치면 '이 test 로는 불확실').

  python e2e/judge_d1.py --base aw30=<aw30 v1.0 test frames.csv> E1=<…/eval_test_frames.csv> [E2=…] [-o out.md]
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rollout_drift import DT, drift_file  # noqa: E402
from steer_diag import diag  # noqa: E402

TH = dict(nav2_slope=+0.04, nav2_drift=-0.008, auto_slope=-0.012, auto_drift=+0.0032, jitter_ratio=1.5)
H2 = int(round(2.0 / DT))


def flips_per_s(d, col):
    f = s = 0.0
    for _, g in d.groupby("file", sort=False):
        w = g.sort_values("idx")[col].values
        f += (np.diff(np.sign(w[np.abs(w) > 0.05])) != 0).sum()
        s += len(w) * DT
    return f / s if s else np.nan


def metrics(d):
    out = {}
    for key, sel in (("auto", d.scenario == "r3_auto"), ("nav2", d.scenario.str.startswith("r3_nav2"))):
        x = d[sel]
        parts = [r for r in (drift_file(g, H2) for _, g in x.groupby("file", sort=False)) if r is not None]
        out[f"{key}_slope"] = diag(x)["slope"]
        out[f"{key}_drift"] = pd.concat(parts).lat.abs().mean() if parts else np.nan
        out[f"{key}_mae"] = (x.w_pred - x.w_teacher).abs().mean()
        out[f"{key}_jitter"] = flips_per_s(x, "w_pred")
        out[f"{key}_jitter_teacher"] = flips_per_s(x, "w_teacher")
        out[f"{key}_n"] = len(x)
    return out


def block_stats(d, sel, block):
    """그룹 sel 의 파일별 덩어리 통계 — 기울기 회귀 합(n, Σx, Σy, Σxx, Σxy) · 2 s 이탈 합(n, Σ|lat|). 행 순서 = 파일 · 덩어리."""
    rows = []
    for _, g in d[sel].groupby("file", sort=True):
        g = g.sort_values("idx")
        x, y = g.w_teacher.values, g.w_pred.values
        dr = drift_file(g, H2)
        lat = np.abs(dr.lat.values) if dr is not None else np.zeros(0)
        for b0 in range(0, len(g), block):
            xs, ys = x[b0:b0 + block], y[b0:b0 + block]
            m = np.abs(xs) > 0.1
            xs, ys = xs[m], ys[m]
            la = lat[b0:b0 + block]
            rows.append((len(xs), xs.sum(), ys.sum(), (xs * xs).sum(), (xs * ys).sum(), len(la), la.sum()))
    return np.array(rows, dtype=np.float64)


def boot_delta(bd, cd, n_boot, block, seed=0):
    """짝지은 블록 부트스트랩 — 차(후보 − 기준)의 2.5 · 97.5 % 분위수. 키 = auto/nav2 _slope/_drift."""
    rng = np.random.default_rng(seed)
    out = {}
    for key, f in (("auto", lambda d: d.scenario == "r3_auto"), ("nav2", lambda d: d.scenario.str.startswith("r3_nav2"))):
        B, C = block_stats(bd, f(bd), block), block_stats(cd, f(cd), block)
        assert B.shape == C.shape and np.allclose(B[:, [0, 1, 3, 5]], C[:, [0, 1, 3, 5]]), "덩어리가 안 맞음"
        idx = rng.integers(0, len(B), size=(n_boot, len(B)))
        def slope(S):
            n, sx, sy, sxx, sxy = (S[..., k] for k in range(5))
            return (n * sxy - sx * sy) / (n * sxx - sx * sx)
        sb, sc = B[idx].sum(1), C[idx].sum(1)
        out[f"{key}_slope"] = np.percentile(slope(sc) - slope(sb), [2.5, 97.5])
        out[f"{key}_drift"] = np.percentile(sc[:, 6] / sc[:, 5] - sb[:, 6] / sb[:, 5], [2.5, 97.5])
        out[f"{key}_blocks"] = len(B)
    return out


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--base", required=True, help="이름=aw30 frames.csv")
    ap.add_argument("cands", nargs="+", help="이름=후보 frames.csv")
    ap.add_argument("--boot", type=int, default=2000, help="블록 부트스트랩 반복 수(0 = 끔)")
    ap.add_argument("--block", type=int, default=50, help="덩어리 프레임 수(50 = 10 s)")
    ap.add_argument("-o", "--out")
    a = ap.parse_args()
    runs = []
    for r in [a.base] + a.cands:
        name, _, path = r.partition("=")
        if not path:
            sys.exit(f"이름=경로 형식이어야 함: {r}")
        runs.append((name, pd.read_csv(path)))
    bname, bd = runs[0]
    key = lambda d: sorted(d.groupby("file").size().items())  # noqa: E731
    for n, d in runs[1:]:
        if key(d) != key(bd):
            sys.exit(f"{n}: 기준과 test 파일 · 프레임 수가 다름(같은 v1.0 test 가 아님) — 판정 안 함")
    m = {n: metrics(d) for n, d in runs}
    b = m[bname]
    L = [f"# 결정 1 판정 — 기준 {bname}, v1.0 test {len(bd)} 프레임 (기준은 PLAN.md '11.9 계획', 결과 보기 전 고정)", "",
         "| 지표 | " + " | ".join(n for n, _ in runs) + " |", "|---|" + "---|" * len(runs)]
    rows = [("① 자동 기울기", "auto_slope", "{:.3f}"), ("② Nav2 기울기", "nav2_slope", "{:.3f}"),
            ("③ 2 s 이탈 자동 (m)", "auto_drift", "{:.4f}"), ("③ 2 s 이탈 Nav2 (m)", "nav2_drift", "{:.4f}"),
            ("ω MAE 자동", "auto_mae", "{:.4f}"), ("ω MAE Nav2", "nav2_mae", "{:.4f}"),
            ("흔들림 자동 (부호 바뀜/s)", "auto_jitter", "{:.3f}"), ("흔들림 Nav2 (부호 바뀜/s)", "nav2_jitter", "{:.3f}")]
    for lab, k, f in rows:
        L.append(f"| {lab} | " + " | ".join(f.format(m[n][k]) for n, _ in runs) + " |")
    L += ["", f"교사 흔들림(참고): 자동 {b['auto_jitter_teacher']:.3f} · Nav2 {b['nav2_jitter_teacher']:.3f} 부호 바뀜/s", ""]
    passed = []
    for n, _ in runs[1:]:
        c = m[n]
        dl = {k: c[k] - b[k] for k in ("nav2_slope", "nav2_drift", "auto_slope", "auto_drift")}
        chk = [("② Nav2 기울기", dl["nav2_slope"], dl["nav2_slope"] >= TH["nav2_slope"], f"≥ {TH['nav2_slope']:+.3f}"),
               ("③ Nav2 이탈", dl["nav2_drift"], dl["nav2_drift"] <= TH["nav2_drift"], f"≤ {TH['nav2_drift']:+.4f}"),
               ("① 자동 기울기", dl["auto_slope"], dl["auto_slope"] >= TH["auto_slope"], f"≥ {TH['auto_slope']:+.3f}"),
               ("③ 자동 이탈", dl["auto_drift"], dl["auto_drift"] <= TH["auto_drift"], f"≤ {TH['auto_drift']:+.4f}")]
        ok = all(x[2] for x in chk)
        ci = boot_delta(bd, dict(runs)[n], a.boot, a.block) if a.boot > 0 else None
        jr = max(c["auto_jitter"] / b["auto_jitter"], c["nav2_jitter"] / b["nav2_jitter"])
        L.append(f"## {n}: {'통과' if ok else '불통과'}")
        ck = ["nav2_slope", "nav2_drift", "auto_slope", "auto_drift"]
        L += [f"- {lab}: {v:+.4f} (기준 {t}) {'✓' if p else '✗'}"
              + (f" · 95 % 구간 [{ci[k][0]:+.4f}, {ci[k][1]:+.4f}]" if ci else "") for (lab, v, p, t), k in zip(chk, ck)]
        L.append(f"- 흔들림 배수(자동 · Nav2 중 큰 쪽): {jr:.2f}× {'→ 노드 ω 평활 플래그 준비' if jr > TH['jitter_ratio'] else '(1.5× 이하, 문제 없음)'}")
        if ci:
            L.append(f"- 구간: 짝지은 블록 부트스트랩 {a.boot}회 · 덩어리 {a.block} 프레임(자동 {ci['auto_blocks']} · Nav2 {ci['nav2_blocks']}개) — test 표본 흔들림만, 시드 차는 규칙 3")
        L.append("")
        if ok:
            passed.append((c["nav2_drift"], n))
    L.append(f"**결론**: {'채택 = ' + min(passed)[1] + ' (통과 중 Nav2 2 s 이탈 최소)' if passed else '채택 없음 — aw30 · A 설정 유지'}")
    txt = "\n".join(L)
    print(txt)
    if a.out:
        os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
        with open(a.out, "w", encoding="utf-8") as fh:
            fh.write(txt + "\n")


if __name__ == "__main__":
    main()
