"""폐루프 회 JSON 의 궤적 그림 — 펼친 그림(기준선 길이 s × 옆 위치) · 위에서 본 그림 (E, 11.18). 결과 폴더 전용(도구 아님).

쓰기 (레포 루트에서, 한 그림에는 시나리오 하나 · 방향 하나):
  python3 e2e/results/closed_loop_demo_11_17/traj_fig.py "E6=<묶음>/runs/*_cw_s?.json" "Nav2=<nav2>/parking_r3_obs_demo_cw.json" \
      --strip e2e/results/closed_loop_demo_11_17/demo_cw_strip.png --map e2e/results/closed_loop_demo_11_17/demo_cw_map.png \
      [--windows 0:40,55:85] [--title "..."]

- 펼친 그림: 가로 = 시계 기준선 길이 s(m), 세로 = 옆 위치(m, 시계 진행의 왼쪽 + = 화단 반대쪽). 반시계 회는 가로축을 뒤집어
  늘 왼쪽 → 오른쪽이 진행 방향. 붉은 점선 = 이탈 한도(시나리오 deviation_limit_m), 회색 띠 = 코너(1 m 간격 방향 변화율 > 0.08 rad/m,
  cw_corner.py 와 같은 식 — 코너 안에서는 사영 때문에 옆 위치가 실제보다 크게 보일 수 있음). 장애물은 시나리오 yaml 의 크기대로.
- 궤적: 회 JSON 의 track 을 판정 구간(judged_samples 가 있으면 그 앞까지)만. 사영은 eval/obstacle_pass.py 그대로.
  실패 회는 끝 자리에 × 와 fail_reason(없으면 result).
- 위에서 본 그림: 세계 좌표 x · y, 기준선 · 한도 띠 · 장애물 · 궤적. 씬 메쉬는 쓰지 않음(벽 · 주차 칸은 안 그림).
필요: numpy · pyyaml · matplotlib.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(REPO, "eval"))
import obstacle_pass as OP  # noqa: E402

K_MIN = 0.08
COLORS = ["#d62728", "#1f77b4", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf"]


def corners(route):
    R, seg, s = route
    h = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    grid = np.arange(0.0, s[-1], 1.0)
    k = np.gradient(np.interp(grid, (s[:-1] + s[1:]) / 2, h), grid)
    out, cur = [], None
    for g, kk in zip(grid, k):
        if abs(kk) > K_MIN:
            cur = [g, g] if cur is None else cur
            cur[1] = g
        elif cur is not None:
            out.append(tuple(cur))
            cur = None
    if cur is not None:
        out.append(tuple(cur))
    return out


def on_route(route, s, lateral):
    """(x, y, 방향 rad) — sim/make_obstacles.py 의 on_route 와 같게."""
    R, seg, S = route
    i = int(np.clip(np.searchsorted(S, s), 1, len(R) - 1))
    f = (s - S[i - 1]) / max(S[i] - S[i - 1], 1e-9)
    x, y = R[i - 1] + f * (R[i] - R[i - 1])
    th = np.arctan2(R[i, 1] - R[i - 1, 1], R[i, 0] - R[i - 1, 0])
    return x - lateral * np.sin(th), y + lateral * np.cos(th), th


def load_run(path, route):
    j = json.load(open(path, encoding="utf-8"))
    cols = j.get("track_columns") or ["t_sim", "x", "y", "cte", "clearance"]
    c = {k: i for i, k in enumerate(cols)}
    tr = np.array(j.get("track") or [], dtype=float)
    if tr.ndim != 2 or len(tr) < 3:
        return None
    n = j.get("judged_samples")
    if isinstance(n, int) and 3 <= n <= len(tr):
        tr = tr[:n]
    x, y = tr[:, c["x"]], tr[:, c["y"]]
    S, LAT, hint = np.empty(len(tr)), np.empty(len(tr)), None
    for k in range(len(tr)):
        S[k], LAT[k] = OP.project(route, x[k], y[k], hint)
        hint = S[k]
    ok = bool(j.get("pass")) if "pass" in j else str(j.get("result", "")).upper() == "SUCCEEDED"
    why = "" if ok else str(j.get("fail_reason") or j.get("end_detail") or j.get("result") or "")
    return {"path": path, "j": j, "x": x, "y": y, "s": S, "lat": LAT, "pass": ok, "why": why,
            "direction": OP.run_direction(j, path), "seed": j.get("seed", ""), "sc": OP.load_scenario(j, path)}


def set_font():
    import matplotlib
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    ko = next((n for n in ("Malgun Gothic", "NanumGothic", "Noto Sans CJK KR", "Noto Sans KR", "AppleGothic") if n in have), None)
    if ko:
        matplotlib.rcParams["font.family"] = [ko, "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    return ko is not None


def draw_obstacle_strip(ax, o):
    from matplotlib.patches import Ellipse, Rectangle
    s, lat = float(o["s"]), float(o.get("lateral", 0.0))
    col = tuple(o.get("color", [0.4, 0.4, 0.4]))
    if o.get("type", "box") == "box":
        sx, sy = float(o["size"][0]), float(o["size"][1])
        ax.add_patch(Rectangle((s - sx / 2, lat - sy / 2), sx, sy, fc=col, ec="k", lw=0.8, zorder=4))
    else:
        d = float(o["size"][0])
        ax.add_patch(Ellipse((s, lat), d, d, fc=col, ec="k", lw=0.8, zorder=4))


def draw_obstacle_map(ax, o, route):
    from matplotlib.patches import Circle, Polygon
    x, y, th = on_route(route, float(o["s"]), float(o.get("lateral", 0.0)))
    col = tuple(o.get("color", [0.4, 0.4, 0.4]))
    if o.get("type", "box") == "box":
        sx, sy = float(o["size"][0]) / 2, float(o["size"][1]) / 2
        a = th + np.radians(float(o.get("yaw_deg", 0.0)))
        ca, sa = np.cos(a), np.sin(a)
        pts = [(x + dx * ca - dy * sa, y + dx * sa + dy * ca) for dx, dy in ((-sx, -sy), (sx, -sy), (sx, sy), (-sx, sy))]
        ax.add_patch(Polygon(pts, closed=True, fc=col, ec="k", lw=0.8, zorder=4))
    else:
        ax.add_patch(Circle((x, y), float(o["size"][0]) / 2, fc=col, ec="k", lw=0.8, zorder=4))


def group_label(name, runs, ko):
    n, k = len(runs), sum(r["pass"] for r in runs)
    return f"{name} — {n}회 중 {k}회 통과" if ko else f"{name} — {k}/{n} pass"


def strip(groups, route, sc, direction, windows, out, title, ko):
    import matplotlib.pyplot as plt
    L = route[2][-1]
    limit = float(sc.get("deviation_limit_m", 2.0))
    cs = corners(route)
    fig, axes = plt.subplots(len(windows), 1, figsize=(12, 3.3 * len(windows) + 0.5), squeeze=False)
    for ax, (a, b) in zip(axes[:, 0], windows):
        for c0, c1 in cs:
            ax.axvspan(c0 - 0.5, c1 + 0.5, color="0.90", lw=0, zorder=0)
        ax.axhline(0, color="0.55", lw=0.8, zorder=1)
        for sg in (1, -1):
            ax.axhline(sg * limit, color="#c00000", lw=1.0, ls="--", zorder=1)
        for o in sc.get("obstacles") or []:
            if a - 1 <= float(o["s"]) <= b + 1:
                draw_obstacle_strip(ax, o)
                ax.annotate(o.get("name", ""), (float(o["s"]), -limit - 0.12), ha="center", va="top", fontsize=8, color="0.25",
                            annotation_clip=True)
        for gi, (name, runs) in enumerate(groups):
            col = COLORS[gi % len(COLORS)]
            for ri, r in enumerate(runs):
                s, lat = r["s"], r["lat"]
                brk = np.where(np.abs(np.diff(s)) > L / 2)[0] + 1
                for pi, (ss, ll) in enumerate(zip(np.split(s, brk), np.split(lat, brk))):
                    ax.plot(ss, ll, color=col, lw=1.5, alpha=0.85, zorder=3,
                            label=group_label(name, runs, ko) if (ri == 0 and pi == 0) else None)
                if not r["pass"] and a <= s[-1] <= b:
                    ax.plot(s[-1], lat[-1], marker="x", ms=9, mew=2.2, color=col, zorder=5)
        ax.set_xlim((b, a) if direction == "ccw" else (a, b))
        ax.set_ylim(-limit - 0.75, limit + 0.75)
        ax.set_ylabel("옆 위치 (m)\n+ 화단 반대쪽 · − 화단 쪽" if ko else "lateral (m)\n+ away from island")
        ax.grid(True, axis="x", color="0.92", lw=0.6)
        ax.text(0.995, 0.96, "진행 →" if ko else "travel →", transform=ax.transAxes, ha="right", va="top", fontsize=9, color="0.3")
    axes[-1, 0].set_xlabel(("시계 기준선 길이 s (m)" if ko else "station s along cw route (m)")
                           + ("  · 반시계 회라 축을 뒤집음" if (ko and direction == "ccw") else ""))
    h, lab = axes[0, 0].get_legend_handles_labels()
    from matplotlib.lines import Line2D
    h += [Line2D([], [], color="#c00000", ls="--", lw=1.0), Line2D([], [], color="k", marker="x", ls="", ms=8, mew=2)]
    lab += [f"이탈 한도 ±{limit:g} m" if ko else f"deviation limit ±{limit:g} m", "실패 끝 자리" if ko else "failed run end"]
    axes[0, 0].legend(h, lab, loc="upper left", fontsize=8.5, ncol=len(h), framealpha=0.9)
    if title:
        fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def topview(groups, route, sc, out, title, ko):
    import matplotlib.pyplot as plt
    R, seg, S = route
    limit = float(sc.get("deviation_limit_m", 2.0))
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(R[:, 0], R[:, 1], color="0.45", lw=1.0, zorder=1, label="기준선" if ko else "reference route")
    n = np.c_[-seg[:, 1], seg[:, 0]] / np.maximum(np.hypot(seg[:, 0], seg[:, 1]), 1e-9)[:, None]
    mid = (R[:-1] + R[1:]) / 2
    for sg in (1, -1):
        e = mid + sg * limit * n
        ax.plot(e[:, 0], e[:, 1], color="#c00000", lw=0.8, ls="--", zorder=1,
                label=(f"이탈 한도 ±{limit:g} m" if ko else f"deviation limit ±{limit:g} m") if sg == 1 else None)
    for o in sc.get("obstacles") or []:
        draw_obstacle_map(ax, o, route)
    for gi, (name, runs) in enumerate(groups):
        col = COLORS[gi % len(COLORS)]
        for ri, r in enumerate(runs):
            ax.plot(r["x"], r["y"], color=col, lw=1.5, alpha=0.85, zorder=3, label=group_label(name, runs, ko) if ri == 0 else None)
            if not r["pass"]:
                ax.plot(r["x"][-1], r["y"][-1], marker="x", ms=9, mew=2.2, color=col, zorder=5)
    ax.plot(R[0, 0], R[0, 1], marker="o", ms=7, mfc="w", mec="k", zorder=6, label="출발" if ko else "start")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, color="0.93", lw=0.6)
    ax.legend(loc="center", fontsize=8.5, framealpha=0.9)
    if title:
        ax.set_title(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("groups", nargs="+", help="이름=회 JSON glob (같은 이름은 합침)")
    ap.add_argument("--route", default=os.path.join(REPO, "eval", "routes", "parking_r3_cw.csv"))
    ap.add_argument("--windows", default=None, help="펼친 그림의 s 구간들 'a:b,c:d' (기본: 한 바퀴 전체)")
    ap.add_argument("--strip", default=None, help="펼친 그림 PNG")
    ap.add_argument("--map", default=None, help="위에서 본 그림 PNG")
    ap.add_argument("--title", default="")
    a = ap.parse_args()
    if not (a.strip or a.map):
        ap.error("--strip · --map 중 하나는 있어야 함")
    import matplotlib
    matplotlib.use("Agg")
    ko = set_font()
    route = OP.load_route(a.route)
    L = route[2][-1]
    groups = {}
    for g in a.groups:
        name, pat = g.split("=", 1)
        files = sorted(f for f in glob.glob(pat) if not f.endswith(".meta.json"))
        if not files:
            sys.exit(f"회 JSON 없음: {pat}")
        for f in files:
            r = load_run(f, route)
            if r is not None and r["sc"] is not None:
                groups.setdefault(name, []).append(r)
    runs = [r for rs in groups.values() for r in rs]
    names = {r["sc"].get("name") for r in runs}
    dirs = {r["direction"] for r in runs}
    if len(names) != 1 or len(dirs) != 1:
        sys.exit(f"한 그림에는 시나리오 하나 · 방향 하나만: 시나리오 {sorted(names)} · 방향 {sorted(dirs)}")
    sc, direction = runs[0]["sc"], dirs.pop()
    windows = [tuple(float(v) for v in w.split(":")) for w in a.windows.split(",")] if a.windows else [(0.0, float(L))]
    gl = list(groups.items())
    for name, rs in gl:
        for r in rs:
            print(f"{name}\t{os.path.basename(r['path'])}\t{direction}\t{'통과' if r['pass'] else '실패'}\t끝 s {r['s'][-1]:.1f} 옆 {r['lat'][-1]:+.2f}"
                  f"\t최대 |옆| {np.max(np.abs(r['lat'])):.2f}\t{r['why']}")
    if a.strip:
        strip(gl, route, sc, direction, windows, a.strip, a.title, ko)
        print("→", a.strip)
    if a.map:
        topview(gl, route, sc, a.map, a.title, ko)
        print("→", a.map)


if __name__ == "__main__":
    main()
