#!/usr/bin/env python3
"""폐루프 회 JSON(eval/xm_route.py · sim/nav2/nav2_route.py 의 track)에서 장애물마다 어떻게 지나갔는지 (E, 11.12)

왜    analyze_runs.py 는 회 단위(통과 · 이탈 · 충돌)만 봄. 오프라인 진단(e2e/obstacle_diag.py)의 남은 약점은 '중앙 장애물 앞에서
      피할 쪽을 정하는 순간'이라, 폐루프에서도 장애물 하나하나를 봐야 원인을 가를 수 있음 — 늦게 꺾나 · 좌우로 흔들리나 · 앞에서
      멈추나 · Nav2 와 같은 쪽으로 가나.
장애물 자리  시나리오 yaml 의 s · lateral(시계 기준선 길이 · 시계 진행의 왼쪽 +) → 세계 좌표는 sim/make_obstacles.py:38-43 과 같게.
로봇 자리    track 의 x · y 를 시계 기준선(eval/routes/parking_r3_cw.csv)에 사영 → s · 옆(왼쪽 +). 반시계 회는 s 가 줄어드는 쪽으로 감.
장애물마다(앞 거리 d = 진행 방향으로 장애물까지 남은 기준선 길이)
  지나감      d 가 + 에서 0 이하로 넘어간 표본이 있으면 지나감, 없으면 못 지나감(멈춤 · 개입 · 시간 초과)
  쪽          지나가는 순간 로봇 옆 − 장애물 옆의 부호 → 진행 방향 기준 왼 · 오. 기하 쪽 = 장애물 오프셋의 반대쪽
              (중앙 |lateral| ≤ 0.15 m 는 기하가 쪽을 약하게만 정함 — Nav2 도 68 %, e2e/results/side_vs_geometry_11_11)
  꺾기 시작   d ≤ 8 m 안에서 로봇 옆이 d = 8 m 때보다 0.15 m 넘게 벌어진 첫 d(Nav2 교사 중앙값 3.7 m)
  흔들림      d ∈ (0, 6] 에서 명령 ω(|ω| ≥ 0.05) 부호가 바뀐 횟수 — track 에 w_cmd 가 있을 때만(xm_route; Nav2 track 은 5칸)
  최소 거리   d ∈ (−1, 6] 의 clearance 최소(로봇 발자국 ↔ 장애물, 0 = 닿음)
  멈춤        d ∈ (0, 3] 에서 속도 < 0.05 m/s 로 보낸 시간(s, 위치 차분)
  python3 eval/obstacle_pass.py data/eval/<묶음>/runs/*.json [nav2 회 json ...] [-o 표.md] [--csv 행.csv]
필요: numpy · pyyaml. 출력: 장애물 행 표 + 방법 × (중앙 · 옆) 요약 + 같은 시나리오 · 방향의 방법 간 쪽 비교.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CENTER = 0.15
W_MIN, V_STOP = 0.05, 0.05


def load_route(path):
    R = np.array([(float(r["x"]), float(r["y"])) for r in csv.DictReader(open(path, encoding="utf-8")) if r.get("run", "0") == "0"])
    seg = np.diff(R, axis=0)
    s = np.concatenate([[0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
    return R, seg, s


def project(route, x, y, hint=None, win=6.0):
    """(s, 옆) — 옆은 시계 진행의 왼쪽 +. hint 가 있으면 그 ±win m 안의 구간만(교차 · 가까운 변 혼동 방지)."""
    R, seg, s = route
    L = s[-1]
    a0 = R[:-1]
    l2 = np.maximum((seg ** 2).sum(1), 1e-12)
    t = np.clip(((x - a0[:, 0]) * seg[:, 0] + (y - a0[:, 1]) * seg[:, 1]) / l2, 0, 1)
    px, py = a0[:, 0] + t * seg[:, 0], a0[:, 1] + t * seg[:, 1]
    dist = np.hypot(px - x, py - y)
    st = (s[:-1] + t * np.sqrt(l2)) % L
    if hint is not None:
        ok = np.abs((st - hint + L / 2) % L - L / 2) <= win
        if ok.any():
            dist = np.where(ok, dist, np.inf)
    i = int(np.argmin(dist))
    nx, ny = -seg[i, 1], seg[i, 0]
    side = (x - px[i]) * nx + (y - py[i]) * ny
    return float(st[i]), float(math.copysign(dist[i], side) if dist[i] > 0 else 0.0)


def wrap(d, L):
    return (d + L / 2) % L - L / 2


def analyze_run(path, route):
    import yaml
    j = json.load(open(path, encoding="utf-8"))
    cols = j.get("track_columns") or ["t_sim", "x", "y", "cte", "clearance"]
    tr = np.array(j.get("track") or [], dtype=float)
    if tr.ndim != 2 or len(tr) < 3:
        return [], j
    c = {k: i for i, k in enumerate(cols)}
    yml = str(j.get("scenario_yaml") or j.get("scenario") or "")
    base = os.path.basename(yml.replace("\\", "/"))
    if base and not base.endswith(".yaml"):
        base += ".yaml"
    cands = [yml, os.path.join(REPO, yml), os.path.join(REPO, "eval", "scenarios", base), os.path.join(REPO, "eval", "scenarios", "rand", base)]
    yp = next((p for p in cands if p and os.path.isfile(p)), None)   # 서버 절대 경로면 이 레포의 같은 이름으로
    if not yp:
        print(f"[건너뜀] 시나리오 yaml 없음: {yml} ({os.path.basename(path)})", file=sys.stderr)
        return [], j
    sc = yaml.safe_load(open(yp, encoding="utf-8"))
    obs = sc.get("obstacles") or []
    direction = j.get("direction")   # xm_route 회 JSON 에는 있음. nav2_route.py 는 route(기준선 CSV 경로) · 없으면 파일 이름(<이름>_ccw.json)
    if direction not in ("cw", "ccw"):
        direction = "ccw" if ("_ccw" in str(j.get("route", "")) or "_ccw" in os.path.basename(path)) else "cw"
    sign = -1.0 if direction == "ccw" else 1.0
    L = route[2][-1]
    t, x, y = tr[:, c["t_sim"]], tr[:, c["x"]], tr[:, c["y"]]
    S, LAT = np.empty(len(tr)), np.empty(len(tr))
    hint = None
    for k in range(len(tr)):
        S[k], LAT[k] = project(route, x[k], y[k], hint)
        hint = S[k]
    sp = np.r_[0.0, np.hypot(np.diff(x), np.diff(y)) / np.maximum(np.diff(t), 1e-3)]
    w = tr[:, c["w_cmd"]] if "w_cmd" in c else None
    clr = tr[:, c["clearance"]] if "clearance" in c else None
    method = j.get("method") or ("nav2" if "w_cmd" not in c else "?")
    rows = []
    for o in obs:
        so, lo = float(o["s"]), float(o.get("lateral", 0.0))
        d = sign * wrap(so - S, L)                                    # + = 앞에 있음
        near = np.abs(d) < 15.0
        passed_idx = [k for k in range(1, len(d)) if near[k] and d[k - 1] > 0 >= d[k] and abs(d[k - 1] - d[k]) < 2.0]
        k_pass = passed_idx[0] if passed_idx else None
        lo_tr = sign * lo                                             # 진행 방향 기준 왼쪽 +
        row = {"file": os.path.basename(path), "method": method, "model_version": j.get("model_version", ""),
               "scenario": sc.get("name", ""), "direction": direction, "seed": j.get("seed", ""), "obstacle": o.get("name", ""),
               "type": o.get("type", ""), "lateral": lo, "kind": "중앙" if abs(lo) <= CENTER else "옆",
               "geom_side": "-" if abs(lo) <= CENTER else ("오" if lo_tr > 0 else "왼"),
               "passed": k_pass is not None, "side": "", "turn_start_m": None, "w_flips": None, "min_clear_m": None,
               "stall_s": 0.0, "closest_d_m": round(float(np.min(np.where(near & (d > 0), d, np.inf))), 2) if (near & (d > 0)).any() else None}
        if k_pass is not None:
            rel = sign * (LAT[k_pass] - lo)                           # 진행 방향 기준 왼쪽 +
            row["side"] = "왼" if rel > 0 else "오"
        app = near & (d > 0) & (d <= 8.0)
        if app.any():
            ia = np.where(app)[0]
            base = LAT[ia[np.argmax(d[ia])]]
            dev = np.where(np.abs(LAT[ia] - base) > 0.15)[0]
            row["turn_start_m"] = round(float(d[ia[dev[0]]]), 2) if len(dev) else None
        win6 = near & (d > 0) & (d <= 6.0)
        if w is not None and win6.any():
            ws = w[win6]
            sg = np.sign(ws[np.abs(ws) >= W_MIN])
            row["w_flips"] = int(np.sum(sg[1:] != sg[:-1])) if len(sg) > 1 else 0
        if clr is not None:
            m = near & (d > -1.0) & (d <= 6.0) & (clr >= 0)
            if m.any():
                row["min_clear_m"] = round(float(np.min(clr[m])), 2)
        st = np.where(near & (d > 0) & (d <= 3.0) & (sp < V_STOP))[0]
        if len(st):
            dt = np.r_[np.diff(t), 0.0]
            row["stall_s"] = round(float(np.sum(dt[st])), 1)
        rows.append(row)
    return rows, j


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "예" if v else "**아니오**"
    if isinstance(v, float):
        return f"{v:g}"
    return str(v)


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="+", help="회 JSON(글롭 가능)")
    ap.add_argument("--route", default=os.path.join(REPO, "eval", "routes", "parking_r3_cw.csv"), help="장애물 s 의 기준(시계 기준선)")
    ap.add_argument("-o", "--out")
    ap.add_argument("--csv")
    a = ap.parse_args()
    files = sorted({f for p in a.runs for f in (glob.glob(p) or [p]) if f.endswith(".json")})
    route = load_route(a.route)
    rows = []
    for f in files:
        try:
            r, _ = analyze_run(f, route)
            rows += r
        except Exception as e:                                        # 한 회가 깨져도 나머지는 봄
            print(f"[건너뜀] {f}: {e}", file=sys.stderr)
    if not rows:
        sys.exit("장애물 행 없음 — track 이 있는 회 JSON 인지 확인")
    keys = ["method", "model_version", "scenario", "direction", "seed", "obstacle", "kind", "lateral", "geom_side", "passed", "side",
            "turn_start_m", "w_flips", "min_clear_m", "stall_s", "closest_d_m"]
    names = ["방법", "모델", "시나리오", "방향", "시드", "장애물", "종류", "옆(m)", "기하 쪽", "지나감", "쪽", "꺾기 시작 m",
             "ω 흔들림", "최소 거리 m", "멈춤 s", "가장 가까운 앞 거리 m"]
    out = ["# 폐루프 장애물별 지나가기 (eval/obstacle_pass.py)\n", f"회 {len(files)}개 · 장애물 행 {len(rows)}개. 쪽 = 진행 방향 기준.\n",
           "## 요약 (방법 × 중앙 · 옆)\n",
           "| 방법 | 종류 | 장애물 | 지나감 | 기하 쪽과 같음 | 꺾기 시작 중앙값 m | ω 흔들림 평균 | 최소 거리 중앙값 m | 멈춤 있음 |",
           "|---|---|---|---|---|---|---|---|---|"]
    for m in sorted({r["method"] for r in rows}):
        for kd in ("중앙", "옆"):
            g = [r for r in rows if r["method"] == m and r["kind"] == kd]
            if not g:
                continue
            ps = [r for r in g if r["passed"]]
            gs = [r for r in ps if r["geom_side"] != "-"]
            ts = [r["turn_start_m"] for r in g if r["turn_start_m"] is not None]
            wf = [r["w_flips"] for r in g if r["w_flips"] is not None]
            mc = [r["min_clear_m"] for r in g if r["min_clear_m"] is not None]
            out.append(f"| {m} | {kd} | {len(g)} | {len(ps)}/{len(g)} | "
                       f"{(str(sum(r['side'] == r['geom_side'] for r in gs)) + '/' + str(len(gs))) if gs else '-'} | "
                       f"{fmt(float(np.median(ts))) if ts else '-'} | {fmt(round(float(np.mean(wf)), 2)) if wf else '-'} | "
                       f"{fmt(float(np.median(mc))) if mc else '-'} | {sum(r['stall_s'] > 0 for r in g)} |")
    pairs = {}
    for r in rows:
        if r["passed"]:
            pairs.setdefault((r["scenario"], r["direction"], r["obstacle"]), {}).setdefault(r["method"], []).append(r["side"])
    multi = {k: v for k, v in pairs.items() if len(v) > 1}
    if multi:
        out += ["\n## 같은 장애물에서 방법 간 쪽 (지나간 회만)\n", "| 시나리오 | 방향 | 장애물 | " + " | ".join(sorted({m for v in multi.values() for m in v})) + " |"]
        ms = sorted({m for v in multi.values() for m in v})
        out.append("|---|---|---|" + "---|" * len(ms))
        for (scn, dr, ob), v in sorted(multi.items()):
            out.append(f"| {scn} | {dr} | {ob} | " + " | ".join("·".join(v.get(m, [])) or "-" for m in ms) + " |")
    out += ["\n## 장애물 행\n", "| " + " | ".join(names) + " |", "|" + "---|" * len(names)]
    for r in sorted(rows, key=lambda r: (r["scenario"], r["direction"], r["obstacle"], r["method"], str(r["seed"]))):
        out.append("| " + " | ".join(fmt(r[k]) for k in keys) + " |")
    text = "\n".join(out) + "\n"
    print(text)
    if a.out:
        open(a.out, "w", encoding="utf-8").write(text)
    if a.csv:
        with open(a.csv, "w", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=["file"] + keys)
            wr.writeheader()
            for r in rows:
                wr.writerow({k: r.get(k) for k in ["file"] + keys})


if __name__ == "__main__":
    main()
