"""side_vs_geometry -- is the side Nav2 picks to pass a centre obstacle predictable from visible geometry? (teacher data only, 11.11)

Question: X-Mobility fine-tuned on Nav2 drives almost straight into centre obstacles. Hypothesis (a) multimodal averaging = Nav2
passes centre obstacles left/right ~50/50 and nothing in the image decides which -> regression averages to ~0. Here only the Nav2
teacher recordings are used: per (recording, obstacle) encounter, which side did Nav2 pass on, and does a simple geometric rule
predict it (sign of the obstacle's lateral offset, lap direction, robot offset / heading / obstacle bearing before the swerve)?

Reuses e2e/obstacle_diag.py: scenario(note) -> obstacle x/y (+ centre flag) and route; route_frame -> v*kappa, robot lateral y,
heading error psi. Only s and size are re-read from the same yaml (obstacle_diag does not keep them).

Signs (TRAVEL frame unless noted):
  D        lap direction +1 cw / -1 ccw (index json; checked against heading = dir_check)
  lat_t    obstacle offset from the route centreline, left of travel + (= yaml lateral * D; yaml lateral + = left of cw travel =
           away from the central island, so 'outer side' in travel frame = D)
  y, psi   robot lateral offset from the centreline (left +), heading error (obstacle_diag.route_frame)
  a        teacher avoidance component = w_teacher - v_teacher * kappa(route)   (obstacle_diag definition)
  ds       along-route distance robot -> obstacle (+ = obstacle ahead)
Encounter  contiguous frames with -3 m < ds <= 10 m that contain the abreast crossing (ds: + -> <= 0). passed = crossing exists.
Pass side  side_disp = sign(y - lat_t) at the abreast crossing (interpolated): +1 = robot passed LEFT of the obstacle  [truth]
           side_w    = sign of the teacher's integrated avoidance turn, anchored at abreast: back(q) = -sum_{q..abreast} a*dt
                       (= heading swing Nav2 still has to undo before it is abreast), taken at its largest |back| over the
                       frames with ds <= W (primary W = 4 m). A full avoidance integrates to ~0, so the sign lives in the
                       out-swing / turn-back lobe right before abreast; anchoring at abreast avoids depending on where the
                       swing started. Reference variants (all W in 4/6/8/10 m): side_wf = start-anchored peak of sum a*dt,
                       side_w2 = commanded displacement sum a*dt*(t_abreast - t).
Swerve onset (decision) = start of the last stretch before abreast in which the robot heads toward the pass side (psi*side > 0);
           'pre' = the frame just before it. NOTE psi_pre is ~0 or of the wrong sign by construction -> R_psi only at fixed ds.
States at fixed distance ds = 8/6/5/4 m (first frame with ds <= d; if the run starts closer -- robot starts at rest in front of
           the first cw obstacle -- the first frame of the run).
Rules (truth = side_disp)
  R_lat    pass opposite to the obstacle offset:                    -sign(lat_t)    (abstains when lat_t == 0)
  R_rel    pass on the side the robot already is (route frame):      sign(y - lat_t)
  R_bear   pass opposite to the obstacle bearing in the robot frame: -sign(left)     (what the camera sees: obstacle left of
           image centre -> go right; = sign of the straight-ahead miss distance)
  R_psi    pass toward the current heading:                           sign(psi)
  R_out    always pass on the outer side (away from the island):     D
  R_dir    majority side per lap direction, leave-one-recording-out (LORO)
  R_place  majority side per (route segment east/south/west_diag/north x direction), LORO / LOSO
  LR(...)  L2 logistic regression, LORO (and leave-one-scenario-out = LOSO, cw+ccw of one layout held out together)
Replicate  the same layout + direction + obstacle recorded in both groups (r3 11.05 vs v08r3 11.11): same-side rate r gives a
           ceiling q for any predictor that only sees the (identical) layout, q^2 + (1-q)^2 = r.
CI = exact Clopper-Pearson 95 %, p = exact two-sided binomial test against 50 %.

  python e2e/side_vs_geometry.py --group r3=<xm_v1_4 dir> --group v08r3=<xm_v08r3_nav2_v1 dir> -o e2e/results/side_vs_geometry_11_11
  -> encounters.csv (one row per encounter) · summary.md (all tables, English/Korean mixed) · traces.png · sanity.csv
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np
import pandas as pd
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.dont_write_bytecode = True      # read-only on the repo: importing obstacle_diag must not write e2e/__pycache__
sys.path.insert(0, os.path.join(REPO, "e2e"))
import obstacle_diag as od  # noqa: E402

# (group name, data dir) from --group. 11.11: r3 = xm_v1_4 (10.14 + 11.05 Nav2, AMCL loc, teacher noise, scene v0.4b r3),
# v08r3 = xm_v08r3_nav2_v1 (11.11 Nav2, gt loc, no noise, scene v0.8 r3 = xm_v1_5 v08r3_nav2_obs)
GROUPS = []
V_IDX, W_IDX = 0, 5                     # driving_command linear x / angular z (eval_offline.py)
AHEAD, BEHIND, NEAR_MIN, A_MIN = od.AHEAD, 3.0, od.NEAR_MIN, od.A_MIN
W_INT = 4.0
W_LIST = (4.0, 6.0, 8.0, 10.0)
D_LIST = (8.0, 6.0, 5.0, 4.0)
D_MAIN = 6.0
AMBIG = 0.25          # |y - lat_t| at abreast below this = drove (almost) through the obstacle line -> ambiguous


# ----------------------------------------------------------------------------------------------------------- statistics
def _pmf(i, n, p):
    return math.comb(n, i) * p ** i * (1 - p) ** (n - i)


def _cdf(k, n, p):
    return sum(_pmf(i, n, p) for i in range(0, k + 1))


def clopper_pearson(k, n, alpha=0.05):
    if n == 0:
        return float("nan"), float("nan")

    def bisect(f):
        lo, hi = 0.0, 1.0
        for _ in range(60):
            mid = (lo + hi) / 2
            lo, hi = (lo, mid) if f(mid) else (mid, hi)
        return (lo + hi) / 2
    lo = 0.0 if k == 0 else bisect(lambda p: 1 - _cdf(k - 1, n, p) >= alpha / 2)
    hi = 1.0 if k == n else bisect(lambda p: _cdf(k, n, p) <= alpha / 2)
    return lo, hi


def binom_p(k, n, p=0.5):
    if n == 0:
        return float("nan")
    pk = _pmf(k, n, p)
    return min(1.0, sum(_pmf(i, n, p) for i in range(n + 1) if _pmf(i, n, p) <= pk * (1 + 1e-9)))


def fk(k, n):
    if n == 0:
        return "0/0"
    lo, hi = clopper_pearson(k, n)
    return f"{k}/{n} = {100 * k / n:.0f} % [{100 * lo:.0f}–{100 * hi:.0f}]"


def logreg_fit(X, y, l2=1.0, iters=100):
    """L2 logistic regression by IRLS; y in {0,1}; X standardized; intercept unpenalized."""
    Xb = np.c_[np.ones(len(X)), X]
    w = np.zeros(Xb.shape[1])
    R = np.eye(Xb.shape[1]) * l2
    R[0, 0] = 1e-6
    for _ in range(iters):
        p = 1 / (1 + np.exp(-Xb @ w))
        step = np.linalg.solve(Xb.T @ (Xb * (p * (1 - p))[:, None]) + R, Xb.T @ (p - y) + R @ w)
        w -= step
        if np.abs(step).max() < 1e-9:
            break
    return w


def held_out_logreg(df, feats, hold, l2=1.0):
    """predicted side (+1/-1) for each row with its 'hold' group (recording or scenario) left out of the fit."""
    y = (df.side_disp.values > 0).astype(float)
    X = df[feats].values.astype(float)
    g = df[hold].values
    pred = np.zeros(len(df))
    for r in np.unique(g):
        tr, te = g != r, g == r
        if len(np.unique(y[tr])) < 2:
            pred[te] = 1.0 if y[tr].mean() >= 0.5 else -1.0
            continue
        mu, sd = X[tr].mean(0), X[tr].std(0) + 1e-9
        w = logreg_fit((X[tr] - mu) / sd, y[tr], l2)
        p = 1 / (1 + np.exp(-np.c_[np.ones(te.sum()), (X[te] - mu) / sd] @ w))
        pred[te] = np.where(p >= 0.5, 1.0, -1.0)
    return pred


def held_out_majority(df, key, hold):
    g, kv, y = df[hold].values, df[key].values, df.side_disp.values
    pred = np.zeros(len(df))
    for r in np.unique(g):
        for v in np.unique(kv[g == r]):
            tr = (g != r) & (kv == v)
            pred[(g == r) & (kv == v)] = 1.0 if (y[tr] > 0).sum() >= (y[tr] < 0).sum() else -1.0
    return pred


def best_threshold(v, y):
    """in-sample best 1-D rule pass = sgn * sign(v - thr) -- optimistic upper bound."""
    best = (0, 0.0, 1)
    for thr in np.unique(np.r_[v, 0.0]):
        for t in (thr - 1e-9, thr + 1e-9):
            for sgn in (1, -1):
                k = int((np.sign(sgn * (v - t)) == y).sum())
                if k > best[0]:
                    best = (k, t, sgn)
    return best


# ------------------------------------------------------------------------------------------------------------- geometry
def route_len(route):
    return float(route["s"][-1] + math.hypot(route["x"][0] - route["x"][-1], route["y"][0] - route["y"][-1]))


def nearest_idx(x, y, route):
    return np.argmin((x[:, None] - route["x"][None]) ** 2 + (y[:, None] - route["y"][None]) ** 2, axis=1)


def project_s(x, y, route):
    """cw arclength of the projection on the route polyline (nearest vertex + tangent offset)."""
    j = nearest_idx(x, y, route)
    th = route["th"][j]
    return route["s"][j] + np.cos(th) * (x - route["x"][j]) + np.sin(th) * (y - route["y"][j])


def yaml_obstacles(name):
    hits = glob.glob(os.path.join(REPO, "eval", "scenarios", "**", name + ".yaml"), recursive=True)
    sc = yaml.safe_load(open(hits[0], encoding="utf-8"))
    return {o["name"]: o for o in sc.get("obstacles") or []}, sc.get("scene")


def runs_of(mask):
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            out.append((i, j))
            i = j
        else:
            i += 1
    return out


def load_recording(dataset, info):
    parts = []
    for fi in info["files"]:
        p = os.path.join(dataset, fi["file"])
        x = pd.read_parquet(p, columns=["acqtime", "driving_command", "map_pose", "teacher_noise", "obstacle_dist"])
        dc, mp = np.stack(x.driving_command.values), np.stack(x.map_pose.values)
        parts.append(pd.DataFrame(dict(file=os.path.basename(p), idx=np.arange(len(x)), t=x.acqtime.values,
                                       map_x=mp[:, 0], map_y=mp[:, 1], map_yaw=mp[:, 2],
                                       v_teacher=dc[:, V_IDX], w_teacher=dc[:, W_IDX],
                                       noise=pd.to_numeric(x.teacher_noise, errors="coerce").fillna(0).values,
                                       obstacle_dist=pd.to_numeric(x.obstacle_dist, errors="coerce").values)))
    return pd.concat(parts, ignore_index=True)


def encounters_for(group, info, g, traces):
    name, obs, route = od.scenario(info["note"])
    yobs, scene = yaml_obstacles(name)
    L = route_len(route)
    wr, yr, psi = od.route_frame(g, route)
    a = g.w_teacher.values - wr
    t = g.t.values
    dt = np.clip(np.diff(t, prepend=t[0] - 0.2), 0.0, 0.5)
    X, Y = g.map_x.values, g.map_y.values
    s_r = project_s(X, Y, route)
    D = 1 if info.get("direction") == "cw" else -1
    dir_check = float(np.mean(np.sign(np.cos(g.map_yaw.values - route["th"][nearest_idx(X, Y, route)])) == D))
    c, s = np.cos(g.map_yaw.values), np.sin(g.map_yaw.values)
    s_all = [float(yobs[o["name"]]["s"]) for o in obs]
    rows = []
    for oi, o in enumerate(obs):
        s_o, lat_y = s_all[oi], float(o["lateral"])
        lat_t = lat_y * D
        ds = ((s_o - s_r) * D + L / 2) % L - L / 2
        dx, dy = o["x"] - X, o["y"] - Y
        fwd, left = c * dx + s * dy, -s * dx + c * dy
        cands = [(b, e, [k for k in range(b + 1, e) if ds[k - 1] > 0 >= ds[k]]) for b, e in runs_of((ds > -BEHIND) & (ds <= AHEAD))]
        r = dict(group=group, split=info["split"], recording=info["bundle"], scenario=name,
                 scene=os.path.basename(scene or ""), direction=info.get("direction"), D=D, dir_check=round(dir_check, 3),
                 obstacle=o["name"], type=o["type"], s_obs=s_o, lateral_yaml=lat_y, lateral_travel=lat_t, abs_lat=abs(lat_y),
                 center=bool(o["center"]), size=json.dumps(yobs[o["name"]].get("size")),
                 segment=("east" if s_o < 12 else "south" if s_o < 45 else "west_diag" if s_o < 62 else "north"),
                 paired=any(abs(((s_o - s2 + L / 2) % L) - L / 2) < 2.0 for k2, s2 in enumerate(s_all) if k2 != oi))
        hits = [cd for cd in cands if cd[2]]
        if not hits:
            r.update(passed=False, n_cross=0)
            rows.append(r)
            continue
        b, e, cross = hits[0]
        k = cross[0]
        f = ds[k - 1] / (ds[k - 1] - ds[k])
        y_ab = yr[k - 1] + f * (yr[k] - yr[k - 1])
        rel = yr - lat_t
        side = float(np.sign(y_ab - lat_t))
        r.update(passed=True, n_cross=sum(len(cd[2]) for cd in cands), start_inside=bool(b == 0), ds_start=float(ds[b]),
                 y_abreast=float(y_ab), rel_abreast=float(y_ab - lat_t), side_disp=side,
                 side_robotframe=float(-np.sign(left[k])), min_centre_dist=float(np.hypot(dx, dy)[b:e].min()),
                 noise_frac=float(np.mean(g.noise.values[b:k + 1] != 0)),
                 od_min_rec=float(np.nanmin(g.obstacle_dist.values[b:e])) if np.isfinite(g.obstacle_dist.values[b:e]).any() else np.nan)
        r["ambiguous"] = bool(abs(r["rel_abreast"]) < AMBIG)
        r["pass_outer"] = side * D
        # teacher-command based side over windows starting at ds = W
        for W in W_LIST:
            q0 = next((q for q in range(b, k + 1) if ds[q] <= W), b)
            ad = a[q0:k + 1] * dt[q0:k + 1]
            cum = np.cumsum(ad)
            back = -np.cumsum(ad[::-1])[::-1]
            tag = "" if W == W_INT else f"_{int(W)}"
            r["side_w" + tag] = float(np.sign(back[int(np.argmax(np.abs(back)))]))
            r["side_wf" + tag] = float(np.sign(cum[int(np.argmax(np.abs(cum)))]))
            r["side_w2" + tag] = float(np.sign(np.sum(ad * (t[k] - t[q0:k + 1]))))
            if W == W_INT:
                r["w_swing"] = float(back[int(np.argmax(np.abs(back)))])
        # swerve onset: going back from abreast, last frame where the robot did not head toward the pass side
        q = k - 1
        while q >= b and psi[q] * side > 0:
            q -= 1
        pre = max(q, b)
        r.update(onset_at_start=bool(q < b), ds_onset=float(ds[min(pre + 1, k)]), ds_pre=float(ds[pre]),
                 y_pre=float(yr[pre]), psi_pre=float(psi[pre]), rel_pre=float(rel[pre]), left_pre=float(left[pre]),
                 calm_pre=bool(abs(yr[pre]) < od.CALM_Y and abs(psi[pre]) < od.CALM_PSI))
        for d in D_LIST:
            q = next((q for q in range(b, k + 1) if ds[q] <= d), b)
            tag = f"_{int(d)}"
            r.update({"ds" + tag: float(ds[q]), "y" + tag: float(yr[q]), "psi" + tag: float(psi[q]), "rel" + tag: float(rel[q]),
                      "left" + tag: float(left[q]), "bear_deg" + tag: float(np.degrees(np.arctan2(left[q], fwd[q])))})
        rows.append(r)
        if o["center"]:
            m = (ds > -BEHIND) & (ds < 25)
            traces.append(dict(group=group, side=side, ds=ds[m], rel=rel[m], psi=psi[m]))
    sanity = dict(group=group, recording=info["bundle"], dir_check=dir_check,
                  od_minus_centre_med=float(np.nanmedian(np.min([np.hypot(o["x"] - X, o["y"] - Y) for o in obs], 0)
                                                         - g.obstacle_dist.values)))
    return rows, sanity


def collect():
    rows, sanity, traces = [], [], []
    for group, root in GROUPS:
        for p in sorted(glob.glob(os.path.join(root, "index", "*.json"))):
            info = json.load(open(p, encoding="utf-8"))
            if info.get("operator") != "nav2" or not info.get("note"):
                continue
            sc = od.scenario(info["note"])
            if not sc or not sc[1]:
                continue
            r, sn = encounters_for(group, info, load_recording(root, info), traces)
            rows += r
            sanity.append(sn)
    return pd.DataFrame(rows), pd.DataFrame(sanity), traces


# ------------------------------------------------------------------------------------------------------------ reporting
def agree(x, col):
    ok = x[col] != 0
    return fk(int((x[col][ok] == x.side_disp[ok]).sum()), int(ok.sum()))


def rule_eval(x, pred):
    pred = np.asarray(pred, float)
    ok = pred != 0
    k, n = int((np.sign(pred[ok]) == x.side_disp.values[ok]).sum()), int(ok.sum())
    return k, n


def rule_line(name, x, pred, note=""):
    k, n = rule_eval(x, pred)
    return f"| {name} | {fk(k, n)} | {binom_p(k, n):.3g} | {len(x) - n} | {note} |"


def lat_crosstab(x):
    L = ["| 장애물 위치 (진행 기준) | 왼쪽으로 지나감 | 오른쪽으로 지나감 | 반대쪽으로 지나감 [95 % CI] |", "|---|---|---|---|"]
    for lab, m in (("왼쪽 (lat > 0)", x.lateral_travel > 0), ("정중앙 (lat = 0)", x.lateral_travel == 0),
                   ("오른쪽 (lat < 0)", x.lateral_travel < 0)):
        xx = x[m]
        nl, nr = int((xx.side_disp > 0).sum()), int((xx.side_disp < 0).sum())
        opp = "—" if lab.startswith("정중앙") else fk(int((xx.side_disp == -np.sign(xx.lateral_travel)).sum()), len(xx))
        L.append(f"| {lab} | {nl} | {nr} | {opp} |")
    nz = x[x.lateral_travel != 0]
    k = int((nz.side_disp == -np.sign(nz.lateral_travel)).sum())
    L.append(f"| **계 (lat ≠ 0)** | | | **{fk(k, len(nz))}**, p = {binom_p(k, len(nz)):.3g} |")
    return L


def bin_table(x, edges, col="abs_lat", label="abs(lateral) m", rule=None):
    """rule(xx) -> predicted side; default = opposite to lat_t (rows with lat_t == 0 abstain)."""
    L = [f"| {label} | 조우 | 규칙 맞음 [95 % CI] | 바깥쪽(화단 반대)으로 |", "|---|---|---|---|"]
    for lo, hi, lab in edges:
        m = (x[col] == 0) if hi == 0 else ((x[col] > lo) & (x[col] <= hi + 1e-9))
        xx = x[m]
        if xx.empty:
            continue
        pred = (-np.sign(xx.lateral_travel)) if rule is None else rule(xx)
        k, n = rule_eval(xx, pred)
        L.append(f"| {lab} | {len(xx)} | {fk(k, n) if n else '— (판정 못함)'} | {fk(int((xx.pass_outer > 0).sum()), len(xx))} |")
    return L


def rules_block(x, title):
    d = int(D_MAIN)
    L = [f"#### {title} — 조우 {len(x)} (녹화 {x.recording.nunique()} · 배치 {x.scenario.nunique()})", "",
         "| 규칙 | 맞음/판정 [95 % CI] | p (= 50 %) | 판정 못함 | 비고 |", "|---|---|---|---|---|"]
    L.append(rule_line("R_lat: lat 반대쪽", x, -np.sign(x.lateral_travel), "고정 규칙"))
    for dd in D_LIST:
        L.append(rule_line(f"R_rel @ {dd:g} m: 로봇이 이미 있는 쪽", x, np.sign(x[f"rel_{int(dd)}"]), "고정 규칙"))
    for dd in D_LIST:
        L.append(rule_line(f"R_bear @ {dd:g} m: 화면 속 장애물 반대쪽", x, -np.sign(x[f"left_{int(dd)}"]), "고정 규칙"))
    L.append(rule_line(f"R_psi @ {d} m: 지금 향한 쪽", x, np.sign(x[f"psi_{d}"]), "고정 규칙"))
    L.append(rule_line("R_rel @ 꺾기 직전(pre)", x, np.sign(x.rel_pre), "고정 규칙, pre 는 결과로 정한 시점(참고)"))
    L.append(rule_line("R_out: 늘 바깥쪽(화단 반대)", x, x.D.values.astype(float), "고정 규칙"))
    L.append(rule_line("R_dir: 방향별 다수 (LORO)", x, held_out_majority(x, "direction", "recording"), "적합"))
    L.append(rule_line("R_place: 구간×방향별 다수 (LORO)", x, held_out_majority(x, "place", "recording"), "적합"))
    L.append(rule_line("R_place: 구간×방향별 다수 (LOSO)", x, held_out_majority(x, "place", "scenario"), "적합"))
    pl = [c for c in x.columns if c.startswith("pl_")]
    for feats, nm in ((["lateral_travel"], "LR(lat)"), ([f"rel_{d}", f"psi_{d}"], f"LR(rel, psi @ {d} m)"),
                      (["lateral_travel", f"y_{d}", f"psi_{d}", "D"], f"LR(lat, y, psi @ {d} m, D)"),
                      (["lateral_travel"] + pl, "LR(lat, 구간×방향)"),
                      (["lateral_travel", f"y_{d}", f"psi_{d}"] + pl, f"LR(lat, y, psi @ {d} m, 구간×방향)")):
        for hold, hn in (("recording", "LORO"), ("scenario", "LOSO")):
            L.append(rule_line(f"{nm} {hn}", x, held_out_logreg(x, feats, hold), "적합"))
    k, thr, sgn = best_threshold(x[f"rel_{d}"].values, x.side_disp.values)
    L.append(f"| (상한) rel @ {d} m 최적 문턱 {thr:+.2f} m, 표본 안 | {fk(k, len(x))} | | 0 | 낙관적 |")
    k, thr, sgn = best_threshold(x["lateral_travel"].values, x.side_disp.values)
    L.append(f"| (상한) lat 최적 문턱 {thr:+.3f} m, 표본 안 | {fk(k, len(x))} | | 0 | 낙관적 |")
    return L


def make_figure(traces, enc, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 2, figsize=(14, 9))
    for gi, (group, _) in enumerate(GROUPS):
        for tr in traces:
            if tr["group"] != group:
                continue
            col = "tab:blue" if tr["side"] > 0 else "tab:red"
            dsv, rv = tr["ds"].astype(float), tr["rel"].astype(float)
            cut = np.r_[False, np.abs(np.diff(dsv)) > 1.0]          # separate passes (end-of-lap re-approach)
            dsv, rv = np.insert(dsv, np.where(cut)[0], np.nan), np.insert(rv, np.where(cut)[0], np.nan)
            axs[0, gi].plot(dsv, rv, "-", lw=0.7, color=col, alpha=0.55)
        axs[0, gi].set_title(f"{group} centre obstacles: robot lateral minus obstacle lateral (m)\nblue = passed left, red = passed right")
        axs[0, gi].set_xlabel("ds = along-route distance to obstacle (m)")
        axs[0, gi].invert_xaxis()
        axs[0, gi].grid(alpha=0.3)
        axs[0, gi].set_ylim(-2.2, 2.2)
        x = enc[(enc.group == group) & enc.center & enc.passed]
        for side, col in ((1, "tab:blue"), (-1, "tab:red")):
            xx = x[x.side_disp == side]
            axs[1, gi].scatter(xx.lateral_travel + np.random.default_rng(0).uniform(-0.004, 0.004, len(xx)), xx[f"rel_{int(D_MAIN)}"],
                               c=col, s=22, label="passed left" if side > 0 else "passed right")
        axs[1, gi].axhline(0, color="k", lw=0.5)
        axs[1, gi].axvline(0, color="k", lw=0.5)
        axs[1, gi].set_xlabel("obstacle lateral offset lat_t (m, left of travel +)")
        axs[1, gi].set_ylabel(f"robot - obstacle lateral at ds = {D_MAIN:g} m (m)")
        axs[1, gi].set_title(f"{group}: side vs obstacle offset and robot offset @ {D_MAIN:g} m")
        axs[1, gi].legend(fontsize=8)
        axs[1, gi].grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(path, dpi=80)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--group", action="append", required=True, metavar="NAME=DIR",
                    help="group name=data dir (train/val/test and index directly below); repeat")
    ap.add_argument("-o", "--outdir", default=os.path.join(REPO, "e2e", "results", "side_vs_geometry_11_11"))
    a = ap.parse_args()
    GROUPS[:] = [tuple(g.split("=", 1)) for g in a.group]
    os.makedirs(a.outdir, exist_ok=True)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    df, sanity, traces = collect()
    df.to_csv(os.path.join(a.outdir, "encounters.csv"), index=False, float_format="%.4f")
    sanity.to_csv(os.path.join(a.outdir, "sanity.csv"), index=False, float_format="%.4f")
    make_figure(traces, df, os.path.join(a.outdir, "traces.png"))

    P = df[df.passed]
    L = ["# side_vs_geometry — 자동 요약 (side_vs_geometry.py)", ""]
    # counts
    L += ["## 조우 수", "", "| 묶음 | split | 녹화 | 중앙 조우 (통과) | 옆 조우 (통과) | 중앙 lat=0 | 중앙 · 출발 직후(정지 상태) |",
          "|---|---|---|---|---|---|---|"]
    for (gname, sp), x in df.groupby(["group", "split"], sort=False):
        c, sd = x[x.center], x[~x.center]
        L.append(f"| {gname} | {sp} | {x.recording.nunique()} | {len(c)} ({int(c.passed.sum())}) | {len(sd)} ({int(sd.passed.sum())}) | "
                 f"{int((c.lateral_yaml == 0).sum())} | {int(c.get('start_inside', pd.Series(False, index=c.index)).fillna(False).sum())} |")
    L += ["", f"모호(abreast 에서 |y − lat| < {AMBIG} m): {int(P.ambiguous.sum())} · 지나가지 못함: {int((~df.passed).sum())} · "
          f"중심 거리 최소 {P.min_centre_dist.min():.2f} m", ""]
    # definitions agreement
    L += ["## 지나간 쪽 정의끼리 일치 (기준 = side_disp, abreast 에서 로봇이 장애물의 어느 쪽)", "",
          f"| 묶음 | 장애물 | side_w (∫a, abreast 기준, {W_INT:g} m~) | side_wf (∫a, 창 시작 기준) | side_w2 (Σa·dt·Δt) | 로봇 좌표계 부호 |",
          "|---|---|---|---|---|---|"]
    for gname in ("r3", "v08r3", "all"):
        for cn, cm in (("중앙", True), ("옆", False)):
            x = P[(P.center == cm) & ((P.group == gname) | (gname == "all"))]
            L.append(f"| {gname} | {cn} | {agree(x, 'side_w')} | {agree(x, 'side_wf')} | {agree(x, 'side_w2')} | {agree(x, 'side_robotframe')} |")
    L += ["", "| 적분 창 W (전체 268) | side_w (abreast 기준) | side_wf (시작 기준) | side_w2 |", "|---|---|---|---|"]
    for W in W_LIST:
        tg = "" if W == W_INT else f"_{int(W)}"
        L.append(f"| {W:g} m | {agree(P, 'side_w' + tg)} | {agree(P, 'side_wf' + tg)} | {agree(P, 'side_w2' + tg)} |")
    dis = P[P.side_w != P.side_disp]
    L += ["", f"side_w 불일치 {len(dis)}건: |swing| 중앙값 {dis.w_swing.abs().median():.2f} rad (일치 {P[P.side_w == P.side_disp].w_swing.abs().median():.2f}) · "
          f"r3(교사 잡음 · AMCL) {int((dis.group == 'r3').sum())} / v08r3 {int((dis.group == 'v08r3').sum())}", ""]
    # crosstabs
    P = P.assign(place=P.segment + "_" + P.direction)
    P = pd.concat([P, pd.get_dummies(P.place, prefix="pl").astype(float)], axis=1)
    C = P[P.center & ~P.paired]
    S = P[~P.center & ~P.paired]
    for gname in ("r3", "v08r3", "all"):
        c = C if gname == "all" else C[C.group == gname]
        s_ = S if gname == "all" else S[S.group == gname]
        L += [f"## {gname}: 중앙 장애물 (|lat| ≤ 0.15, 쌍 콘 제외) — 지나간 쪽 vs sign(lat)", ""] + lat_crosstab(c) + [""]
        L += bin_table(c, [(0, 0, "= 0"), (0, 0.05, "(0, 0.05]"), (0.05, 0.10, "(0.05, 0.10]"), (0.10, 0.15, "(0.10, 0.15]")]) + [""]
        L += [f"### {gname}: 옆 장애물 (대조군)", ""] + lat_crosstab(s_) + [""]
        L += bin_table(s_, [(0.15, 0.40, "(0.15, 0.40]"), (0.40, 0.55, "(0.40, 0.55]"), (0.55, 0.70, "(0.55, 0.70]")]) + [""]
    # direction / outer
    L += ["## 방향 · 바깥쪽 (중앙, 쌍 제외)", "", "| 묶음 | 방향 | 조우 | 왼쪽 | 바깥쪽(화단 반대) |", "|---|---|---|---|---|"]
    for (gname, dname), x in C.groupby(["group", "direction"]):
        L.append(f"| {gname} | {dname} | {len(x)} | {fk(int((x.side_disp > 0).sum()), len(x))} | {fk(int((x.pass_outer > 0).sum()), len(x))} |")
    L += ["", "## 구간별 (중앙, 쌍 제외) — lat 반대쪽 규칙", "", "| 구간 | 조우 | lat 반대쪽 | R_rel @ 6 m |", "|---|---|---|---|"]
    for seg, x in C.groupby("segment"):
        k1, n1 = rule_eval(x, -np.sign(x.lateral_travel))
        k2, n2 = rule_eval(x, np.sign(x.rel_6))
        L.append(f"| {seg} | {len(x)} | {fk(k1, n1)} | {fk(k2, n2)} |")
    # rules
    L += ["", "## 규칙 / 모델 (중앙, 쌍 제외)", ""]
    for gname in ("r3", "v08r3", "all"):
        x = (C if gname == "all" else C[C.group == gname]).reset_index(drop=True)
        L += rules_block(x, gname) + [""]
    rest = C[C.start_inside.fillna(False).astype(bool)]
    mov = C[~C.start_inside.fillna(False).astype(bool)].reset_index(drop=True)
    L += ["## 출발 직후(정지 · 기준선 위에서 결정) vs 달리는 중", "",
          "| 조건 | 조우 | R_lat (lat 반대쪽) | R_rel @ 6 m | R_bear @ 6 m |", "|---|---|---|---|---|"]
    for nm, x in (("출발 직후 (첫 장애물, cw)", rest), ("달리는 중", mov)):
        L.append(f"| {nm} | {len(x)} | {fk(*rule_eval(x, -np.sign(x.lateral_travel)))} | {fk(*rule_eval(x, np.sign(x.rel_6)))} | "
                 f"{fk(*rule_eval(x, -np.sign(x.left_6)))} |")
    L += ["", "### 달리는 중: |rel @ 6 m| (로봇 − 장애물 옆 거리) 구간별 R_rel", ""]
    L += bin_table(mov.assign(arel=mov.rel_6.abs()), [(0, 0.1, "(0, 0.1]"), (0.1, 0.2, "(0.1, 0.2]"), (0.2, 0.3, "(0.2, 0.3]"),
                                                       (0.3, 0.5, "(0.3, 0.5]"), (0.5, 3.0, "> 0.5")],
                   col="arel", label="|rel @ 6 m| m", rule=lambda xx: np.sign(xx.rel_6))
    L += ["", "### 달리는 중: |left @ 6 m| (화면 속 장애물이 정면에서 벗어난 옆 거리) 구간별 R_bear", ""]
    L += bin_table(mov.assign(al=mov.left_6.abs()), [(0, 0.1, "(0, 0.1]"), (0.1, 0.2, "(0.1, 0.2]"), (0.2, 0.3, "(0.2, 0.3]"),
                                                      (0.3, 0.5, "(0.3, 0.5]"), (0.5, 3.0, "> 0.5")],
                   col="al", label="|left @ 6 m| m", rule=lambda xx: -np.sign(xx.left_6))
    # replicate: the same layout + direction + obstacle driven twice (r3 11.05 vs v08r3 11.11)
    key = ["scenario", "direction", "obstacle"]
    j = P[P.group == "r3"].set_index(key)[["side_disp", "center", "paired", "rel_6", "lateral_travel", "segment"]].join(
        P[P.group == "v08r3"].set_index(key)[["side_disp", "rel_6"]], rsuffix="_b", how="inner")
    L += ["", "## 같은 배치 · 같은 방향 · 같은 장애물을 두 번 녹화 (r3 11.05 vs v08r3 11.11) — Nav2 선택의 재현성", "",
          "| 장애물 | 쌍 | 같은 쪽 [95 % CI] | 이로부터 본 '기하로 맞힐 수 있는 상한' q (q² + (1−q)² = 재현율) |", "|---|---|---|---|"]
    def ceil_q(r):
        return (1 + math.sqrt(max(2 * r - 1, 0.0))) / 2
    for cn, cm in (("중앙", True), ("옆", False)):
        x = j[(j.center == cm) & ~j.paired]
        k, n = int((x.side_disp == x.side_disp_b).sum()), len(x)
        lo, hi = clopper_pearson(k, n)
        L.append(f"| {cn} | {n} | {fk(k, n)} | {100 * ceil_q(k / n):.0f} % [{100 * ceil_q(lo):.0f}–{100 * ceil_q(hi):.0f}] |")
    x = j[j.center & ~j.paired]
    flips = x[x.side_disp != x.side_disp_b]
    L += ["", f"중앙에서 갈린 {len(flips)}건 중 6 m 앞 로봇 옆 위치(rel) 부호가 두 녹화에서 같은 것 {int((np.sign(flips.rel_6) == np.sign(flips.rel_6_b)).sum())}건 "
          "(로봇 자리도 같은 쪽인데 다른 쪽으로 지나감)"]
    L += ["", "## 같은 배치 cw · ccw — 물리적으로 같은 쪽(바깥/화단 쪽)으로 지나갔나", "", "| 묶음 | 장애물 | 쌍 | 같은 쪽 |", "|---|---|---|---|"]
    for gname in ("r3", "v08r3"):
        x = P[(P.group == gname) & ~P.paired]
        jj = x[x.direction == "cw"].set_index(["scenario", "obstacle"])[["pass_outer", "center"]].join(
            x[x.direction == "ccw"].set_index(["scenario", "obstacle"])[["pass_outer"]], rsuffix="_ccw", how="inner")
        for cn, cm in (("중앙", True), ("옆", False)):
            y = jj[jj.center == cm]
            L.append(f"| {gname} | {cn} | {len(y)} | {fk(int((y.pass_outer == y.pass_outer_ccw).sum()), len(y))} |")
    ex = S[S.side_disp == np.sign(S.lateral_travel)]
    L += ["", f"## 옆 장애물인데 장애물 쪽으로 지나간 {len(ex)}건", "", "| 묶음 | 녹화 | 배치 | 장애물 | s | 구간 | lat(yaml) | rel @ 6 m |",
          "|---|---|---|---|---|---|---|---|"]
    for _, r in ex.iterrows():
        L.append(f"| {r.group} | {r.recording} | {r.scenario} | {r.obstacle} | {r.s_obs:.1f} | {r.segment} | {r.lateral_yaml:+.2f} | {r.rel_6:+.2f} |")
    L += ["", "구간별 옆 장애물 lat 반대쪽: " + " · ".join(
        f"{seg} {fk(*rule_eval(x, -np.sign(x.lateral_travel)))}" for seg, x in S.groupby("segment"))]
    L += ["", "### abs(lateral) 구간별 lat 반대쪽 — 중앙 + 옆, 서쪽 사선(s 53–58) 제외 / 포함", "",
          "| abs(lateral) m | 서쪽 사선 제외 | 전체 |", "|---|---|---|"]
    CS = pd.concat([C, S])
    for lo, hi, lab in ((0, 0.05, "(0, 0.05]"), (0.05, 0.10, "(0.05, 0.10]"), (0.10, 0.15, "(0.10, 0.15]"),
                        (0.15, 0.40, "[0.30, 0.40]"), (0.40, 0.55, "(0.40, 0.55]"), (0.55, 0.70, "(0.55, 0.70]")):
        x = CS[(CS.abs_lat > lo) & (CS.abs_lat <= hi + 1e-9)]
        xo = x[x.segment != "west_diag"]
        L.append(f"| {lab} | {fk(*rule_eval(xo, -np.sign(xo.lateral_travel)))} | {fk(*rule_eval(x, -np.sign(x.lateral_travel)))} |")
    L += ["", "## 구간 × 방향별 중앙 장애물 — 바깥쪽(화단 반대)으로 지나간 비율", "", "| 구간 | 방향 | 조우 | 바깥쪽 |", "|---|---|---|---|"]
    for (seg, dname), x in C.groupby(["segment", "direction"]):
        L.append(f"| {seg} | {dname} | {len(x)} | {fk(int((x.pass_outer > 0).sum()), len(x))} |")
    L += ["", f"꺾기 시작 거리 ds_onset (중앙): 중앙값 {C.ds_onset.median():.1f} m, 사분위 {C.ds_onset.quantile(.25):.1f}–{C.ds_onset.quantile(.75):.1f} m, "
          f"창 시작부터 이미 꺾는 중 {int(C.onset_at_start.sum())}/{len(C)}",
          f"옆 장애물 (쌍 제외) R_rel @ 6 m: {fk(*rule_eval(S, np.sign(S.rel_6)))} · R_lat: {fk(*rule_eval(S, -np.sign(S.lateral_travel)))}"]
    txt = "\n".join(L)
    with open(os.path.join(a.outdir, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")
    print(txt)


if __name__ == "__main__":
    main()
