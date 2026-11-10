"""장애물 회피 진단 — '중앙 장애물로 직진'이 좌우 다봉 평균 때문인지, 직전 명령 되먹임 때문인지 가른다 (E, 11.10)

  왜    D 폐루프(11.05–06): 파인튜닝 실패 = 경로 중앙 장애물로 거의 직진. Nav2 교사는 중앙 장애물을 좌우 반반으로 피함 →
        (가) 다봉 평균: 같은 장면에 좌 · 우 두 정답 → 회귀가 평균(직진)을 배움
        (나) 되먹임: 피하기 시작해도 자기 직전 명령(직진)을 이어감(11.9 교사 강제 입력 평가)
  회피 성분  a = ω − v · κ(기준선 곡률, 진행 방향 부호). Nav2 는 코너를 덜 도는 식으로도 피해서(r3 슬라럼 상자 1: ω 는 작아도
        기준선 왼쪽 0.95 m) ω 만 보면 회피가 안 보임 → 기준선 따라가기 몫을 뺀다. 예측도 같은 v · κ(교사 v)를 뺌.
  프레임  장애물이 로봇 앞 0.5–10 m · 옆 3 m 안이고 교사가 피하는 중(|a 교사| > 0.08)인 프레임만. 두 묶음:
        - 결정 순간 = 로봇이 아직 기준선 위에 반듯이(|옆| < 0.2 m · |방향 차| < 0.1 rad) — 직전 명령(교사 입력이어도)에
          방향 정보가 거의 없어 모델이 영상으로 방향을 골라야 함
        - 회피 중 = 이미 기준선에서 벗어남
        부호 일치(예측 a 가 교사 a 와 같은 방향 · |a 예측| > 0.03) · 크기 비(Σ 예측 a · 부호 / Σ |교사 a|)
  판독  교사 입력 평가(eval_offline --teacher-forcing)와 기본 평가를 같이 넣으면:
        - 중앙 장애물 결정 순간에 교사 입력으로도 부호 일치 ≈ 50 % · 크기 비가 작으면 → (가) 다봉 평균
        - 결정 순간은 맞는데 회피 중 크기 비가 기본 평가에서만 작으면 → (나) 되먹임
        중앙 · 옆 장애물을 나눠 봄: 옆 장애물은 피할 쪽이 정해져 있어 (가)가 없어야 함 — 대조군.
  장애물 자리 = 시나리오 yaml(s · lateral, sim/make_obstacles.py on_route 와 같은 계산), 녹화 → 시나리오는 데이터 index/<묶음>.json 의 note.
  중앙 = |lateral| ≤ 0.15 m(eval/scenarios/gen_rand_obstacles.py 의 '가운데' 정의). 장애물이 여럿 가까우면 가장 가까운 것.

  python e2e/obstacle_diag.py --dataset <xm 데이터 폴더(index/ 있는 곳)> aw30=<frames.csv> aw30_tf=<교사 입력 frames.csv> [-o out.md] [--csv 프레임별.csv]
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np
import pandas as pd
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AHEAD, NEAR_MIN, SIDE, A_MIN, CENTER, K_SMOOTH_M = 10.0, 0.5, 3.0, 0.08, 0.15, 1.0
CALM_Y, CALM_PSI, SIGN_MIN = 0.2, 0.1, 0.03
_CACHE = {}


def load_route(path):
    R = np.array([(float(r["s"]), float(r["x"]), float(r["y"])) for r in csv.DictReader(open(path)) if r.get("run", "0") == "0"])
    if len(R) > 2 and math.hypot(R[-1, 1] - R[0, 1], R[-1, 2] - R[0, 2]) < 1e-6:
        R = R[:-1]   # 끝점 = 시작점(parking_r3_cw.csv s 89.96 = 0) — 그대로 두면 닫는 구간 길이 0 → κ ≈ 1e6, 출발 근처 프레임이 합을 덮음(11.10 xm_v3)
    s, x, y = R[:, 0], R[:, 1], R[:, 2]
    xe, ye = np.append(x, x[0]), np.append(y, y[0])          # 닫힌 고리
    th = np.unwrap(np.arctan2(np.diff(ye), np.diff(xe)))
    ds = np.hypot(np.diff(xe), np.diff(ye))
    w = max(1, int(round(K_SMOOTH_M / max(float(np.median(ds)), 1e-3))))
    k = np.gradient(th) / np.maximum(ds, 1e-6)                # 시계 방향 진행 기준 곡률(왼쪽 회전 +)
    k = np.convolve(np.r_[k[-w:], k, k[:w]], np.ones(2 * w + 1) / (2 * w + 1), mode="same")[w:-w]
    return dict(s=s, x=x, y=y, th=np.arctan2(np.sin(th), np.cos(th)), kappa=k)


def scenario(note):
    name = note.rsplit("_", 1)[0] if note.endswith(("_cw", "_ccw")) else note
    if name in _CACHE:
        return _CACHE[name]
    hits = glob.glob(os.path.join(REPO, "eval", "scenarios", "**", name + ".yaml"), recursive=True)
    if not hits:
        _CACHE[name] = None
        return None
    sc = yaml.safe_load(open(hits[0], encoding="utf-8"))
    route = load_route(os.path.join(REPO, sc["route"]))
    R = np.c_[route["s"], route["x"], route["y"]]
    obs = []
    for o in sc.get("obstacles") or []:
        s, lat = float(o["s"]), float(o.get("lateral", 0.0))
        i = int(np.clip(np.searchsorted(R[:, 0], s), 1, len(R) - 1))
        (s0, x0, y0), (s1, x1, y1) = R[i - 1], R[i]
        f = (s - s0) / max(s1 - s0, 1e-9)
        x, y, th = x0 + f * (x1 - x0), y0 + f * (y1 - y0), math.atan2(y1 - y0, x1 - x0)
        obs.append(dict(name=o["name"], type=o.get("type", "?"), lateral=lat, center=abs(lat) <= CENTER,
                        x=x - lat * math.sin(th), y=y + lat * math.cos(th)))
    _CACHE[name] = (name, obs, route)
    return _CACHE[name]


def route_frame(g, route):
    """프레임마다 (기준선 따라가기 ω = v_교사 · κ, 옆 위치(진행 방향 왼쪽 +), 방향 차)."""
    d2 = (g.map_x.values[:, None] - route["x"][None]) ** 2 + (g.map_y.values[:, None] - route["y"][None]) ** 2
    j = np.argmin(d2, axis=1)
    dirn = np.sign(np.cos(g.map_yaw.values - route["th"][j]))      # 시계 방향 진행 +1 · 반시계 −1
    th = route["th"][j] + np.where(dirn < 0, np.pi, 0.0)
    y = -np.sin(th) * (g.map_x.values - route["x"][j]) + np.cos(th) * (g.map_y.values - route["y"][j])
    psi = np.arctan2(np.sin(g.map_yaw.values - th), np.cos(g.map_yaw.values - th))
    return g.v_teacher.values * route["kappa"][j] * dirn, y, psi


def frame_rows(frames, index):
    out = []
    for f, g in frames.groupby("file", sort=False):
        bundle = os.path.splitext(f)[0][4:].rsplit("_", 1)[0]
        info = index.get(bundle)
        sc = scenario(info["note"]) if info and info.get("note") else None
        if not sc or not sc[1]:
            continue
        g = g.sort_values("idx").reset_index(drop=True)
        wr, yr, psi = route_frame(g, sc[2])
        c, s = np.cos(g.map_yaw.values), np.sin(g.map_yaw.values)
        best = np.full(len(g), np.inf)
        pick = np.full(len(g), -1)
        for i, o in enumerate(sc[1]):
            dx, dy = o["x"] - g.map_x.values, o["y"] - g.map_y.values
            fwd, left = c * dx + s * dy, -s * dx + c * dy
            ok = (fwd >= NEAR_MIN) & (fwd <= AHEAD) & (np.abs(left) <= SIDE)
            d = np.where(ok, np.hypot(fwd, left), np.inf)
            upd = d < best
            best[upd], pick[upd] = d[upd], i
        at = g.w_teacher.values - wr
        ap = g.w_pred.values - wr
        m = (pick >= 0) & (np.abs(at) > A_MIN)
        if not m.any():
            continue
        out.append(pd.DataFrame(dict(
            file=f, idx=g.idx.values[m], scenario=sc[0], obstacle=[sc[1][p]["name"] for p in pick[m]],
            center=[sc[1][p]["center"] for p in pick[m]], dist=best[m],
            calm=(np.abs(yr[m]) < CALM_Y) & (np.abs(psi[m]) < CALM_PSI), a_teacher=at[m], a_pred=ap[m])))
    return pd.concat(out) if out else pd.DataFrame()


def summarize(x):
    sg = np.sign(x.a_teacher)
    ok = (np.sign(x.a_pred) == sg) & (np.abs(x.a_pred) > SIGN_MIN)
    ratio = (x.a_pred * sg).sum() / max(np.abs(x.a_teacher).sum(), 1e-9)
    return len(x), x.file.nunique(), ok.mean(), ratio


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="+", help="이름=frames.csv(eval_offline)")
    ap.add_argument("--dataset", required=True, help="xm 데이터 폴더 — index/<묶음>.json 의 note 로 시나리오를 찾음")
    ap.add_argument("-o", "--out")
    ap.add_argument("--csv", help="프레임별 행을 이 CSV 로도")
    a = ap.parse_args()
    index = {}
    for p in glob.glob(os.path.join(a.dataset, "index", "*.json")):
        d = json.load(open(p, encoding="utf-8"))
        index[d["bundle"]] = d
    L = [f"# 장애물 회피 진단 — {a.dataset}", "",
         f"회피 성분 a = ω − v·κ(기준선). 프레임 = 장애물 앞 {NEAR_MIN:g}–{AHEAD:g} m · 옆 {SIDE:g} m 안 · 교사 |a| > {A_MIN}. "
         f"결정 순간 = 로봇이 기준선 위(|옆| < {CALM_Y} m · |방향 차| < {CALM_PSI} rad), 회피 중 = 그 밖. 중앙 = |lateral| ≤ {CENTER} m. "
         "크기 비 1 = 교사만큼 피함, 0 = 직진, 음수 = 반대.", "",
         "| 모델 | 장애물 | 순간 | 프레임 (녹화) | 부호 일치 | 크기 비 |", "|---|---|---|---|---|---|"]
    allrows = []
    for r in a.runs:
        name, _, path = r.partition("=")
        df = frame_rows(pd.read_csv(path), index)
        if df.empty:
            L.append(f"| {name} | — | — | 0 | | |")
            continue
        df.insert(0, "model", name)
        allrows.append(df)
        for og, xo in (("중앙", df[df.center]), ("옆", df[~df.center])):
            for mg, x in (("결정 순간", xo[xo.calm]), ("회피 중", xo[~xo.calm])):
                if x.empty:
                    continue
                n, nf, ok, ratio = summarize(x)
                L.append(f"| {name} | {og} | {mg} | {n} ({nf}) | {ok * 100:.0f} % | {ratio:.2f} |")
    if allrows and a.csv:
        pd.concat(allrows).to_csv(a.csv, index=False)
    L += ["", "프레임별 열린 고리(교사 자리에서의 예측) — 폐루프 결과 아님. 프레임 수가 적은 칸(< 30)은 참고만."]
    txt = "\n".join(L)
    print(txt)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as fh:
            fh.write(txt + "\n")


if __name__ == "__main__":
    main()
