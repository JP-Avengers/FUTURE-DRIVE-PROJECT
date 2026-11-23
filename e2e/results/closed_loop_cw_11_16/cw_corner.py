"""시계 방향 실패 자리 — 장애물이 코너 출구에서 몇 m 뒤에 있었나 (E, 11.16). 결과 폴더 전용 분석(도구 아님).

쓰기 (레포 루트에서):
  python3 e2e/results/closed_loop_cw_11_16/cw_corner.py "E6=<11.13>/runs/*_s1.json" "E6=<11.14 대조>/runs/*_s1.json" \
      "E7=<11.14>/runs/*_s1.json" "Nav2=<nav2>/*.json" -o e2e/results/closed_loop_cw_11_16
  → encounters.csv(장애물 행 + 코너 출구 뒤 거리) · profile.csv(시계 s 40–66 의 옆 위치) · 표준 출력에 요약

- 코너 = 시계 기준선(eval/routes/parking_r3_cw.csv)에서 1 m 간격 방향 변화율 |κ| > 0.08 rad/m 인 연속 구간.
- 출구 뒤 거리 = 진행 방향으로 마지막 코너 끝(시계면 구간 위 끝, 반시계면 아래 끝)부터 장애물 s 까지.
- 장애물 행 · 꺾기 시작 · 최소 거리는 eval/obstacle_pass.py 그대로. 옆 위치는 시계 진행의 왼쪽 + (= 바퀴 바깥).
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(REPO, "eval"))
import obstacle_pass as OP  # noqa: E402

K_MIN = 0.08
PROFILE_S = list(range(40, 67, 2))


def corners(route):
    R, seg, s = route
    h = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    grid = np.arange(0.0, s[-1], 1.0)
    k = np.gradient(np.interp(grid, (s[:-1] + s[1:]) / 2, h), grid)
    out, cur = [], None
    for g, kk in zip(grid, k):
        if abs(kk) > K_MIN:
            cur = [g, g, 0.0] if cur is None else cur
            cur[1] = g
            cur[2] += kk
        elif cur is not None:
            out.append(tuple(cur))
            cur = None
    if cur is not None:
        out.append(tuple(cur))
    return out   # (s 시작, s 끝, 방향 변화 rad — 1 m 칸 합)


def after_exit(s_obs, direction, cs, L):
    best = None
    for a, b, dh in cs:
        d = (s_obs - b) % L if direction == "cw" else (a - s_obs) % L
        if best is None or d < best[0]:
            best = (d, f"{a:.0f}-{b:.0f}", dh)
    return best


def lateral_profile(path, route):
    j = json.load(open(path, encoding="utf-8"))
    cols = j.get("track_columns") or ["t_sim", "x", "y", "cte", "clearance"]
    c = {k: i for i, k in enumerate(cols)}
    tr = np.array(j.get("track") or [], dtype=float)
    S, LAT, hint = [], [], None
    for k in range(len(tr)):
        s_, l_ = OP.project(route, tr[k, c["x"]], tr[k, c["y"]], hint)
        hint = s_
        S.append(s_)
        LAT.append(l_)
    S, LAT = np.array(S), np.array(LAT)
    return [round(float(np.mean(LAT[np.abs(S - g) < 0.5])), 2) if (np.abs(S - g) < 0.5).any() else None for g in PROFILE_S]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("groups", nargs="+", help="이름=회 JSON glob (같은 이름은 합침)")
    ap.add_argument("--route", default=os.path.join(REPO, "eval", "routes", "parking_r3_cw.csv"))
    ap.add_argument("--near", type=float, default=4.5, help="'출구 바로 뒤' 기준 m")
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args()
    route = OP.load_route(a.route)
    L = route[2][-1]
    cs = corners(route)
    print("코너(시계 s · 방향 변화 rad):", " · ".join(f"{x:.0f}-{y:.0f} {dh:+.2f}" for x, y, dh in cs))
    enc, prof = [], []
    for g in a.groups:
        name, pat = g.split("=", 1)
        for p in sorted(glob.glob(pat)):
            if p.endswith(".meta.json"):
                continue
            rows, j = OP.analyze_run(p, route)
            sc = OP.load_scenario(j, p)
            if not rows or sc is None:
                continue
            so = {o["name"]: float(o["s"]) for o in sc["obstacles"]}
            fr = str(j.get("fail_reason") or "")
            res = str(j.get("result") or j.get("status") or "")
            for r in rows:
                d, cn, dh = after_exit(so[r["obstacle"]], r["direction"], cs, L)
                enc.append(dict(group=name, file=p.replace("\\", "/"), scenario=r["scenario"], direction=r["direction"], result=res,
                                fail_reason=fr, obstacle=r["obstacle"], s=round(so[r["obstacle"]], 2), lateral=r["lateral"], kind=r["kind"],
                                exit_m=round(d, 1), corner=cn, corner_dh=round(dh, 2), passed=r["passed"], side=r["side"],
                                pass_offset_m=r["pass_offset_m"], turn_start_m=r["turn_start_m"], min_clear_m=r["min_clear_m"]))
            prof.append(dict(group=name, file=p.replace("\\", "/"), scenario=sc.get("name", ""), direction=OP.run_direction(j, p), result=res,
                             **{f"s{g_}": v for g_, v in zip(PROFILE_S, lateral_profile(p, route))}))
    os.makedirs(a.out, exist_ok=True)
    for fn, rows in (("encounters.csv", enc), ("profile.csv", prof)):
        with open(os.path.join(a.out, fn), "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    print(f"\n장애물 행(지나갔거나 꺾기 시작이 있는 행) — '출구 바로 뒤' = 코너 출구 뒤 {a.near} m 안")
    print("묶음 방향 출구뒤 | 행 지나감 늦은꺾기(<2 m) 꺾기시작중앙 최소거리중앙")
    for name in dict.fromkeys(e["group"] for e in enc):
        for d in ("cw", "ccw"):
            for near in (True, False):
                x = [e for e in enc if e["group"] == name and e["direction"] == d and (e["exit_m"] < a.near) == near
                     and (e["passed"] or e["turn_start_m"] is not None)]
                if not x:
                    continue
                ts = [e["turn_start_m"] for e in x if e["turn_start_m"] is not None]
                cl = [e["min_clear_m"] for e in x if e["min_clear_m"] is not None]
                print(f"{name:5s} {d:3s} {'안' if near else '밖'} | {len(x):2d} {sum(e['passed'] for e in x):2d} {sum(t < 2 for t in ts):2d} "
                      f"{np.median(ts) if ts else float('nan'):5.2f} {np.median(cl) if cl else float('nan'):5.2f}")


if __name__ == "__main__":
    main()
