#!/usr/bin/env python3
"""기준선 · 출발점 생성 — 찍은 점(또는 촬영 궤적)을 잇는 일반 경로 (11.18, D). 시스템 python3(PyYAML · numpy)로:

    python3 eval/routes/make_route.py eval/routes/<이름>_route.yaml [--check-only] [--scenario] [--force]

eval/routes/make_refline.py 는 볼록한 섬 둘레만 만든다. 이 도구는 열린 경로(직선 · 꺾인 길)와 닫힌 경로(오목해도 됨)를 만든다.
입력 <이름>_route.yaml:
  name · scene(씬 루트) · kind(open | loop) · step(기본 0.25)
  points: [[x, y], …]            지도 위에 찍은 점(씬 좌표, m). loop 는 닫는 점을 다시 적지 않음
  corner_radius: 1.0             꺾이는 점을 이 반경의 원호로 둥글림(0 = 꺾은선 그대로). 변이 짧으면 그 모서리만 반경을 줄임
  capture: {csv: <x,y 열이 있는 CSV>, cut: farthest | none, trim_start_m: 0, trim_end_m: 0, smooth_m: 1.0, simplify_m: 0.3}
                                 points 대신 — 촬영 궤적(recon 의 preview_route.csv · route_capture.csv)을 다듬어 점으로 씀
  nav2_map: data/nav2/<씬 이름>/map.yaml   있으면 지도 위 여유를 잼(없으면 씬 이름으로 찾고, 그것도 없으면 건너뜀)
  clearance: {warn_m: 0.66, fail_m: 0.25}  여유 기준(로봇 외접 반경 · 반폭 — 맵 품질 판정과 같은 값)
  names: {a: fwd, b: back}       open 의 출력 이름 꼬리(기본 fwd · back). loop 는 도는 방향으로 cw · ccw 가 정해짐
출력(입력 파일 옆, 고치지 말고 다시 생성):
  <name>_<꼬리>.csv   step 간격 점 run,s,x,y — loop 는 닫는 점 포함(make_refline.py 와 같은 형식)
  <name>_<꼬리>.yaml  길이 · 출발 자세 · 경로 지시 구간 · 지도 위 여유
  --scenario 면 확인용 시나리오도 씀: open 은 양 끝을 띄운 구간 시나리오 둘, loop 는 한 바퀴 시나리오 하나(있는 파일은 --force 없이는 안 덮음)
끝 줄: ROUTE_OK | ROUTE_WARN | ROUTE_FAIL — 여유가 기준 아래거나 경로가 지도의 빈 칸 밖이면 WARN · FAIL(FAIL 이면 종료 코드 3).
"""
import argparse
import csv
import hashlib
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "sim"))
import teacher_core as tc  # noqa: E402

ARC_STEP = 0.05   # 원호를 나누는 간격(m) — make_refline.py 와 같음


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rel(path):
    """레포 루트 기준 경로로 열 수 있게."""
    return path if os.path.isabs(path) or os.path.exists(path) else os.path.join(REPO, path)


# ── 점 → 꺾은선 ──────────────────────────────────────────────────────

def dedup(pts, eps=1e-6):
    out = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
            out.append((float(p[0]), float(p[1])))
    return out


def fillet(pts, r, closed):
    """꺾이는 점을 반경 r 원호로 둥글린 꺾은선. 닫힌 경로는 첫 꼭짓점의 원호가 끝나는 곳에서 시작한다.
    (점들, 모서리마다 실제로 쓴 반경, 출발 방향 rad)"""
    n = len(pts)
    idx = list(range(n)) if closed else list(range(1, n - 1))
    corner = {}
    for i in idx:
        a, v, b = pts[i - 1], pts[i], pts[(i + 1) % n]
        l_in, l_out = math.hypot(v[0] - a[0], v[1] - a[1]), math.hypot(b[0] - v[0], b[1] - v[1])
        u_in = ((v[0] - a[0]) / l_in, (v[1] - a[1]) / l_in)
        u_out = ((b[0] - v[0]) / l_out, (b[1] - v[1]) / l_out)
        turn = wrap(math.atan2(u_out[1], u_out[0]) - math.atan2(u_in[1], u_in[0]))
        if r <= 0 or abs(turn) < 1e-9:
            continue
        if abs(abs(turn) - math.pi) < 1e-6:
            raise SystemExit(f"점 {i} 에서 왔던 길로 되돌아감(180°) — 왕복은 열린 경로 하나로 만들고 반대 방향 파일을 쓸 것")
        tl = r * math.tan(abs(turn) / 2)
        room = 0.5 * min(l_in, l_out) if (closed or (0 < i - 1 and i + 1 < n - 1)) else min(
            l_in if i - 1 == 0 else 0.5 * l_in, l_out if i + 1 == n - 1 else 0.5 * l_out)
        rr = r if tl <= room else room / math.tan(abs(turn) / 2)
        tl = min(tl, room)
        sgn = 1.0 if turn > 0 else -1.0
        t_in = (v[0] - u_in[0] * tl, v[1] - u_in[1] * tl)
        t_out = (v[0] + u_out[0] * tl, v[1] + u_out[1] * tl)
        c = (t_in[0] - sgn * u_in[1] * rr, t_in[1] + sgn * u_in[0] * rr)        # 도는 쪽으로 rr
        a0 = math.atan2(t_in[1] - c[1], t_in[0] - c[0])
        k = max(2, int(abs(turn) * rr / ARC_STEP))
        arc = [(c[0] + rr * math.cos(a0 + turn * j / k), c[1] + rr * math.sin(a0 + turn * j / k)) for j in range(1, k)]
        corner[i] = (t_in, arc, t_out, rr)
    out = []
    if closed:
        for i in range(n):                          # 변 i = 꼭짓점 i → i+1
            j = (i + 1) % n
            out.append(corner[i][2] if i in corner else pts[i])
            if j in corner:
                out.append(corner[j][0])
                out.extend(corner[j][1])
        first = corner[0][2] if 0 in corner else pts[0]
        nxt = corner[1][0] if 1 in corner else pts[1 % n]
    else:
        out.append(pts[0])
        for i in range(1, n - 1):
            if i in corner:
                out.append(corner[i][0]); out.extend(corner[i][1]); out.append(corner[i][2])
            else:
                out.append(pts[i])
        out.append(pts[-1])
        first, nxt = out[0], out[1]
    return dedup(out), {i: round(c[3], 3) for i, c in corner.items()}, math.atan2(nxt[1] - first[1], nxt[0] - first[0])


def resample(pts, step, closed):
    """step 에 가장 가까운 고른 간격으로 다시 찍음. 닫힌 경로는 닫는 점을 빼고 돌려준다(make_refline.py resample 과 같은 계산)."""
    line = pts + [pts[0]] if closed else pts
    cum = [0.0]
    for (ax, ay), (bx, by) in zip(line, line[1:]):
        cum.append(cum[-1] + math.hypot(bx - ax, by - ay))
    L = cum[-1]
    n = max(1, int(round(L / step)))
    out, j = [], 0
    for k in range(n if closed else n + 1):
        s = L * k / n
        while j < len(line) - 2 and cum[j + 1] < s:
            j += 1
        seg = cum[j + 1] - cum[j]
        t = 0.0 if seg < 1e-12 else (s - cum[j]) / seg
        out.append((line[j][0] + t * (line[j + 1][0] - line[j][0]), line[j][1] + t * (line[j + 1][1] - line[j][1])))
    return out, L, n


# ── 촬영 궤적 → 점 ───────────────────────────────────────────────────

def simplify(pts, tol):
    """Douglas–Peucker — 선에서 tol 안에 있는 점을 뺌."""
    if len(pts) < 3 or tol <= 0:
        return pts
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        (ax, ay), (bx, by) = pts[a], pts[b]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        best, bi = 0.0, None
        for i in range(a + 1, b):
            t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((pts[i][0] - ax) * dx + (pts[i][1] - ay) * dy) / l2))
            d = math.hypot(pts[i][0] - ax - t * dx, pts[i][1] - ay - t * dy)
            if d > best:
                best, bi = d, i
        if bi is not None and best > tol:
            keep[bi] = True
            stack += [(a, bi), (bi, b)]
    return [p for p, k in zip(pts, keep) if k]


def from_capture(cfg, step):
    """촬영 궤적 CSV(x, y 열) → 다듬은 점. (점들, 한 일을 적은 줄)"""
    rows = list(csv.DictReader(open(rel(cfg["csv"]), encoding="utf-8")))
    raw = dedup([(float(r["x"]), float(r["y"])) for r in rows], eps=0.01)
    note = [f"촬영 궤적 {len(rows)}점"]
    cut = cfg.get("cut", "farthest")
    if cut == "farthest":                           # 갔다가 돌아온 촬영 — 출발점에서 가장 먼 곳까지만
        k = max(range(len(raw)), key=lambda i: math.hypot(raw[i][0] - raw[0][0], raw[i][1] - raw[0][1]))
        if k < len(raw) - 1:
            note.append(f"가장 먼 점({raw[k][0]:.2f}, {raw[k][1]:.2f})에서 자름 — 뒤 {len(raw) - 1 - k}점 버림")
        raw = raw[:k + 1]
    line, L, _ = resample(raw, step, False)
    a, b = float(cfg.get("trim_start_m", 0.0)), float(cfg.get("trim_end_m", 0.0))
    i0, i1 = int(round(a / step)), len(line) - int(round(b / step))
    line = line[i0:i1]
    if len(line) < 2:
        raise SystemExit("capture: 자르고 나니 점이 2개 미만")
    k = int(round(float(cfg.get("smooth_m", 1.0)) / step))
    if k >= 1:                                       # 이동 평균(양 끝은 창을 줄임 — 끝점은 그대로)
        sm = []
        for i in range(len(line)):
            h = min(k, i, len(line) - 1 - i)
            sm.append((sum(p[0] for p in line[i - h:i + h + 1]) / (2 * h + 1), sum(p[1] for p in line[i - h:i + h + 1]) / (2 * h + 1)))
        line = sm
    pts = simplify(line, float(cfg.get("simplify_m", 0.3)))
    note.append(f"길이 {L:.1f} m → 앞 {a:g} · 뒤 {b:g} m 자르고 {len(pts)}점으로 줄임")
    return pts, " · ".join(note)


# ── 경로 지시(열린 경로) ─────────────────────────────────────────────

def turn_windows_open(pts, L, thresh_deg=40.0, half=3.0, before=5.0, after=2.0):
    """sim/teacher_core.py turn_windows 와 같은 규칙을 열린 경로에 — 끝을 넘어 이어 보지 않는다.
    [(시작 s, 끝 s, 지시, 꺾인 각 deg, 꼭짓점 s)]"""
    n = len(pts) - 1
    if n < 2:
        return []
    step = L / n
    yaw = [math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0]) for i in range(n)]
    k = max(1, int(round(half / step)))
    delta = [wrap(yaw[min(i + k, n - 1)] - yaw[max(i - k, 0)]) for i in range(n)]
    th = math.radians(thresh_deg)
    runs, cur = [], None
    for i in range(n):
        if abs(delta[i]) >= th:
            cur = [i] if cur is None else cur + [i]
        elif cur is not None:
            runs.append(cur); cur = None
    if cur is not None:
        runs.append(cur)
    out = []
    for run in runs:
        dmax = max(abs(delta[i]) for i in run)
        plat = [i for i in run if abs(delta[i]) >= 0.95 * dmax]
        apex = (plat[0] + plat[-1] + 1) / 2 * step
        sgn = delta[max(run, key=lambda i: abs(delta[i]))]
        out.append((max(0.0, apex - before), min(L, apex + after), tc.LEFT if sgn > 0 else tc.RIGHT, math.degrees(sgn), apex))
    return out


# ── 지도 위 여유 ─────────────────────────────────────────────────────

class NavMap:
    """Nav2 지도(map.yaml + PGM). 빈 칸 254 · 장애물 0 · 미지 205."""

    def __init__(self, yaml_path):
        meta = yaml.safe_load(open(yaml_path, encoding="utf-8"))
        self.res = float(meta["resolution"])
        self.ox, self.oy = (float(v) for v in meta["origin"][:2])
        with open(os.path.join(os.path.dirname(yaml_path), meta["image"]), "rb") as f:
            if f.readline().strip() != b"P5":
                raise SystemExit(f"{yaml_path}: PGM(P5)이 아님")
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()
            self.w, self.h = map(int, line.split())
            f.readline()
            self.im = np.frombuffer(f.read(), np.uint8).reshape(self.h, self.w)
        self.free = self.im == 254

    def clearance(self, xy, reach=5.0):
        """점마다 (빈 칸 위인지, 가장 가까운 빈 칸 아닌 칸까지 m — reach 넘으면 reach). 지도 밖은 빈 칸이 아닌 것으로 본다."""
        k = int(math.ceil(reach / self.res))
        pad = np.zeros((self.h + 2 * k, self.w + 2 * k), bool)
        pad[k:k + self.h, k:k + self.w] = self.free
        dy, dx = np.mgrid[-k:k + 1, -k:k + 1]
        dist = np.hypot(dx, dy) * self.res
        on, clr = [], []
        for x, y in xy:
            c = int(math.floor((x - self.ox) / self.res)); r = self.h - 1 - int(math.floor((y - self.oy) / self.res))
            if not (0 <= c < self.w and 0 <= r < self.h):
                on.append(False); clr.append(0.0); continue
            win = pad[r:r + 2 * k + 1, c:c + 2 * k + 1]
            on.append(bool(win[k, k]))
            blocked = dist[~win]
            clr.append(float(min(reach, blocked.min())) if blocked.size else reach)
        return np.array(on), np.array(clr)


def check_clearance(nav, pts, L, warn, fail, closed):
    xy = np.array(pts)
    on, clr = nav.clearance(xy)
    s = np.linspace(0.0, L, len(xy), endpoint=not closed)
    i = int(np.argmin(clr))
    level = "fail" if (not on.all() or clr.min() < fail) else "warn" if clr.min() < warn else "ok"
    return dict(level=level, off_free_pct=round(float(100.0 * (~on).mean()), 2), min_m=round(float(clr.min()), 2),
                min_at=dict(s=round(float(s[i]), 1), x=round(float(xy[i, 0]), 2), y=round(float(xy[i, 1]), 2)),
                p5_m=round(float(np.percentile(clr, 5)), 2), median_m=round(float(np.median(clr)), 2),
                below_warn_m=round(float((clr < warn).mean() * L), 1))


# ── 쓰기 ─────────────────────────────────────────────────────────────

def write_csv(path, pts, L, closed):
    n = len(pts) if closed else len(pts) - 1
    with open(path, "w") as f:
        f.write("run,s,x,y\n")
        for i, (x, y) in enumerate(pts + [pts[0]] if closed else pts):
            f.write(f"0,{i * L / n:.3f},{x:.3f},{y:.3f}\n")


def write_yaml(path, name, tag, cfg, src, sha, rule, csv_name, step, L, pts, yaw, closed, wins, clr, map_path):
    with open(path, "w") as f:
        f.write(f"# 생성 파일 — 고치지 말고 {os.path.basename(src)} 를 고친 뒤 eval/routes/make_route.py 로 다시 만든다\n")
        f.write(f"name: {name}_{tag}\nversion: {cfg.get('version', 'v0-draft')}\nscene: {cfg.get('scene', '')}\n")
        f.write(f"direction: {tag}\nkind: {'loop' if closed else 'open'}\nsource: {os.path.basename(src)} (sha256 {sha})\n")
        f.write(f"rule: {rule}\n")
        f.write(f"ref_path_csv: {csv_name}   # {step:g} m 간격 run,s,x,y{' (닫는 점 포함)' if closed else ''}\n")
        f.write(f"length_m: {L:.2f}\n")
        f.write(f"start: {{x: {pts[0][0]:.3f}, y: {pts[0][1]:.3f}, yaw_deg: {yaw:.1f}, s: 0.0}}   # 씬 시작 자세와 같게\n")
        if not closed:
            f.write(f"end: {{x: {pts[-1][0]:.3f}, y: {pts[-1][1]:.3f}, s: {L:.2f}}}\n")
        if clr:
            f.write(f"clearance: {{map: {map_path}, level: {clr['level']}, min_m: {clr['min_m']}, min_at_s: {clr['min_at']['s']}, "
                    f"p5_m: {clr['p5_m']}, median_m: {clr['median_m']}, off_free_pct: {clr['off_free_pct']}}}   # 지도의 빈 칸 아닌 칸까지 거리\n")
        f.write("commands:   # /route_command — 40° 넘는 꺾임 꼭짓점 앞 5 m ~ 뒤 2 m(sim/teacher_core.py turn_windows 규칙), 그 밖은 straight"
                + (". from_s > to_s 면 바퀴 이음을 넘는 구간" if closed else "") + "\n")
        if wins:
            for a, b, c, deg, apex in sorted(wins, key=lambda w: w[4]):
                f.write(f"  - {{from_s: {a:.1f}, to_s: {b:.1f}, cmd: {'left' if c == tc.LEFT else 'right'}, turn_deg: {deg:.0f}, apex_s: {apex:.1f}}}\n")
        else:
            f.write("  []\n")


def write_scenarios(cfg, name, outs, closed, L, force):
    """확인용 시나리오 — 형식은 eval/scenarios/corridor_v02_straight_fwd.yaml(구간) · parking_r3_lap.yaml(한 바퀴)."""
    sc = cfg.get("scenario") or {}
    d = os.path.join(REPO, "eval", "scenarios")
    made = []
    if closed:
        first = next(o for o in outs if o["tag"] == "cw")   # 한 바퀴 시나리오의 route 는 시계 방향 파일(반시계는 이름 끝을 바꿔 찾음 — eval/scenario_site.py)
        items = [(f"{name}_lap", first, None)]
    else:
        items = [(f"{name}_straight_{o['tag']}" if not o["wins"] else f"{name}_path_{o['tag']}", o, o["tag"]) for o in outs]
    for sname, o, _ in items:
        p = os.path.join(d, sname + ".yaml")
        if os.path.exists(p) and not force:
            print(f"  시나리오 그대로 둠(이미 있음, --force 로 덮어씀): eval/scenarios/{sname}.yaml")
            continue
        margin = float(sc.get("margin_m", 2.0))
        route = os.path.relpath(o["csv"], REPO).replace(os.sep, "/")
        with open(p, "w") as f:
            f.write(f"# {cfg.get('title', name)} — {'한 바퀴' if closed else '구간'} {sname} v0 (eval/routes/make_route.py --scenario). 새 경로 확인용 — 평가 시나리오 아님.\n")
            f.write(f"name: {sname}\nversion: v0\nscene: {cfg['scene']}\nroute: {route}\n")
            if closed:
                f.write("directions: [cw, ccw]\n")
                f.write(f"start: {{x: {o['pts'][0][0]:.3f}, y: {o['pts'][0][1]:.3f}, yaw_deg: {o['yaw']:.1f}}}   # 씬 시작 자세 = 기준선 출발점\n")
                f.write("goal: {lap: 1, radius_m: 1.0}\n")
            else:
                cmds = sorted({"left" if w[2] == tc.LEFT else "right" for w in o["wins"]}) or []
                f.write("direction: cw\nstart: {s: 0.0}\n")
                f.write(f"goal: {{s: {max(L - margin, 0.5 * L):.1f}, radius_m: 1.0}}\n")
                f.write(f"commands: [{', '.join(['straight'] + cmds)}]\n")
            f.write("obstacles: []\ndeviation_limit_m: 2.0\nstall_limit_s: 10\n")
            f.write(f"timeout_s: {int(sc.get('timeout_s', max(120, round(L * 5 / 10) * 10)))}\n")
        made.append(sname)
        print(f"  시나리오 씀: eval/scenarios/{sname}.yaml")
    return made


def main():
    ap = argparse.ArgumentParser(description="찍은 점 · 촬영 궤적 → 기준선 CSV · yaml, 지도 위 여유 확인")
    ap.add_argument("config")
    ap.add_argument("--check-only", action="store_true", help="파일을 쓰지 않고 길이 · 출발 자세 · 여유만 찍음")
    ap.add_argument("--scenario", action="store_true", help="확인용 시나리오 yaml 도 씀(eval/scenarios/)")
    ap.add_argument("--force", action="store_true", help="--scenario 가 이미 있는 시나리오를 덮어씀")
    a = ap.parse_args()
    raw = open(a.config, "rb").read()
    cfg = yaml.safe_load(raw)
    name, step = cfg["name"], float(cfg.get("step", 0.25))
    kind = cfg.get("kind", "open")
    if kind not in ("open", "loop"):
        raise SystemExit("kind 는 open 또는 loop")
    closed = kind == "loop"
    r = float(cfg.get("corner_radius", 0.0))
    if cfg.get("capture"):
        if closed:
            raise SystemExit("capture 는 열린 경로(kind: open)에서만")
        src_pts, how = from_capture(cfg["capture"], step)
        print(f"capture: {how}")
        rule = f"촬영 궤적({os.path.basename(cfg['capture']['csv'])})을 다듬은 선 — 꺾이는 점은 반경 {r:g} m 원호"
    else:
        src_pts = dedup([(float(p[0]), float(p[1])) for p in cfg["points"]])
        rule = f"찍은 점 {len(src_pts)}개를 이은 선 — 꺾이는 점은 반경 {r:g} m 원호" if r > 0 else f"찍은 점 {len(src_pts)}개를 이은 꺾은선"
    if closed and len(src_pts) > 1 and math.hypot(src_pts[0][0] - src_pts[-1][0], src_pts[0][1] - src_pts[-1][1]) < 1e-6:
        src_pts = src_pts[:-1]
    if len(src_pts) < (3 if closed else 2):
        raise SystemExit(f"점이 너무 적음: {len(src_pts)}개({'loop 는 3개' if closed else 'open 은 2개'} 이상)")
    line, radii, yaw0 = fillet(src_pts, r, closed)
    shrunk = {i: v for i, v in radii.items() if v < r - 1e-9}
    pts, L, n = resample(line, step, closed)
    if closed:
        area = 0.5 * sum(src_pts[i][0] * src_pts[(i + 1) % len(src_pts)][1] - src_pts[(i + 1) % len(src_pts)][0] * src_pts[i][1] for i in range(len(src_pts)))
        t0, t1 = ("ccw", "cw") if area > 0 else ("cw", "ccw")
        rev = [pts[0]] + pts[1:][::-1]
        yaw_rev = (math.degrees(yaw0) + 360) % 360 - 180   # 출발점의 접선을 뒤집은 방향(make_refline.py 와 같음 — 뒤쪽은 바로 원호라 다음 점 방향으로 재면 틀림)
    else:
        nm = cfg.get("names") or {}
        t0, t1 = nm.get("a", "fwd"), nm.get("b", "back")
        rev = pts[::-1]
        yaw_rev = math.degrees(wrap(math.atan2(rev[1][1] - rev[0][1], rev[1][0] - rev[0][0])))
    sets = [(t0, pts, math.degrees(yaw0), True), (t1, rev, yaw_rev, False)]

    map_path = cfg.get("nav2_map") or (os.path.join("data/nav2", os.path.splitext(os.path.basename(cfg.get("scene", "")))[0], "map.yaml") if cfg.get("scene") else None)
    nav = NavMap(rel(map_path)) if map_path and os.path.exists(rel(map_path)) else None
    cl = cfg.get("clearance") or {}
    warn, fail = float(cl.get("warn_m", 0.66)), float(cl.get("fail_m", 0.25))
    clr = check_clearance(nav, pts, L, warn, fail, closed) if nav else None

    sha = hashlib.sha256(raw).hexdigest()[:12]
    base = os.path.join(os.path.dirname(os.path.abspath(a.config)), name)
    print(f"{name}: {'닫힌' if closed else '열린'} 경로 · 길이 {L:.2f} m · 점 {len(pts)} · 간격 {L / n:.4f} m · 입력 점 {len(src_pts)}"
          + (f" · 반경을 줄인 모서리 {shrunk}" if shrunk else ""))
    outs = []
    for tag, p, yaw, given in sets:
        if closed:
            wins = tc.turn_windows(tc.Route(p, step=0))
        else:
            wins = turn_windows_open(p, L)
        o = dict(tag=tag, pts=p, yaw=yaw, wins=wins, given=given, csv=f"{base}_{tag}.csv")
        outs.append(o)
        if not a.check_only:
            write_csv(o["csv"], p, L, closed)
            write_yaml(f"{base}_{tag}.yaml", name, tag, cfg, a.config, sha, rule, f"{os.path.basename(base)}_{tag}.csv", step, L, p, yaw, closed, wins, clr, map_path)
        print(f"  {tag}: 출발 ({p[0][0]:.3f}, {p[0][1]:.3f}) yaw {yaw:.1f}° · 회전 {len(wins)}" + ("" if a.check_only else f" — {base}_{tag}.csv · .yaml"))

    level = "ok"
    scene = cfg.get("scene")
    if scene and os.path.exists(rel(scene)):
        sp = tc.start_pose_from_usda(rel(scene))
        if sp:
            best = min(outs, key=lambda o: math.hypot(o["pts"][0][0] - sp[0], o["pts"][0][1] - sp[1]) + abs(wrap(math.radians(o["yaw"]) - sp[2])))
            d = math.hypot(best["pts"][0][0] - sp[0], best["pts"][0][1] - sp[1]); dyaw = abs(math.degrees(wrap(math.radians(best["yaw"]) - sp[2])))
            same = d <= 0.05 and dyaw <= 1.0
            print(f"  씬 시작 자세 ({sp[0]:.3f}, {sp[1]:.3f}) yaw {math.degrees(sp[2]):.1f}° — {best['tag']} 출발점과 {d:.2f} m · {dyaw:.1f}° 차이"
                  + ("" if same else " → 한 바퀴 시나리오는 씬 시작 자세를 출발점에 맞춰야 함" if closed else " (구간 시나리오는 레이어가 출발 자세를 덮어쓰므로 달라도 됨)" if not same else ""))
            if closed and not same:
                level = "warn"
        else:
            print("  씬 루트에서 로봇 시작 자세를 못 읽음")
    if clr:
        print(f"  지도 위 여유({map_path}): 최소 {clr['min_m']} m (s {clr['min_at']['s']}, {clr['min_at']['x']}, {clr['min_at']['y']}) · 5 % {clr['p5_m']} m · 중앙 {clr['median_m']} m"
              f" · {warn:g} m 아래인 길이 {clr['below_warn_m']} m · 빈 칸 밖 {clr['off_free_pct']} % → {clr['level']}")
        level = "fail" if clr["level"] == "fail" else "warn" if "warn" in (clr["level"], level) else level
    else:
        print(f"  지도 위 여유: 재지 않음 — 지도 없음({map_path or 'nav2_map · scene 이 없음'})")
    if a.scenario and not a.check_only:
        write_scenarios(cfg, name, outs, closed, L, a.force)
    print({"ok": "ROUTE_OK", "warn": "ROUTE_WARN", "fail": "ROUTE_FAIL"}[level])
    return 3 if level == "fail" else 0


if __name__ == "__main__":
    sys.exit(main())
