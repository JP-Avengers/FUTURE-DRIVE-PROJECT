#!/usr/bin/env python3
"""장소 하나를 명령 하나로 들임 — 입력(RGB-D + 자세 폴더 · 맵 패키지) → 맵 → Nav2 지도 · 경로 · 시나리오 · 씬 루트 → 맵 판정 (11.19, D).

    python3 sim/add_site.py <입력> --site <장소> --ver <판> [--route <경로>] [--keepout <yaml>] [--group <폴더>] [--no-install]

지금까지 맵을 받을 때마다 손으로 짠 반입 스크립트(11.14 복도 59줄 · 11.16 네 맵 28줄)가 하던 일을 한 곳에 모으고, 앞에 맵 만들기를 붙였다.
<입력> 은 넷 중 하나이고 모양을 보고 알아낸다:
  맵 패키지 zip · 폴더      usd/<장소>_mesh.usdc 가 든 것(C 의 recon/pipeline/package.py 결과. run.py 의 출력 폴더면 usd 와 점검 결과만 가져감)
  RGB-D + 자세 폴더         keyframes/{rgb,depth,calib} + 자세 파일(rtabmap_keyframes)  또는  rgb.txt · depth.txt + 자세 파일(tum)
  recon 설정 yaml           recon/configs/<장소>.yaml 을 직접 줄 때
RGB-D 폴더면 실내 틀(recon/configs/templates/indoor_rtabmap.yaml)을 물려받는 장소 설정을 써서 recon/pipeline/run.py 를 돌린다.
이름 규칙(eval/scenario_site.py 와 같음): <이름> = <장소>_<판> — 맵 data/usd/<이름>/ · 지도 data/nav2/<이름>/ · 씬 루트 sim/scenes/<장소>/<이름>.usda.

--route 는 둘 중 하나:
  <이름>_route.yaml   찍은 점으로 기준선을 만듦(eval/routes/make_route.py — 사람이 주는 것은 이 파일뿐). 확인용 시나리오도 같이 만듦
  이미 있는 기준선 .csv   새로 만들지 않고 맵 판정에만 씀(주차장의 parking_r3_cw.csv 처럼 좌표가 같은 새 판)
없으면 지도의 가장 넓은 자리를 출발점으로 잡는다(주행 경로는 없음).

하는 일(단계 이름은 결과 표의 첫 칸):
  input   입력 모양 판별                    recon   (RGB-D 일 때) 장소 설정 쓰기 → run.py
  usd     패키지 풀기 · 메쉬 해시             nav2    sim/nav2/make_map.py, 패키지의 참고 지도(nav2_reference/)와 대조
  route   make_route.py --scenario          scene   패키지의 씬 루트 제안, 없으면 출발 자세로 새로 씀
  gate    맵 판정(환경 변수 FD_GATE_DIR 의 map_gate.py · gate_verdict.py — 없으면 건너뜀)
  place   만든 것을 제자리에 놓음            layer   시나리오마다 sim/make_obstacles.py 로 레이어가 만들어지는지
모든 결과는 먼저 <작업 폴더>/tree/ 에 레포와 같은 자리로 만들고, 그다음 제자리(--dest, 기본 이 레포)에 놓는다.
  제자리에 없는 것만 놓는다. 이미 있으면 덮어쓰지 않고 같은지만 대조한다(MANIFEST 규칙 — 덮어쓰지 말고 판을 올림).
  그래서 이미 들인 맵에 다시 돌리면 "예전 반입과 같은가"를 확인하는 명령이 된다. --no-install 이면 놓지 않고 대조만 한다.
작업 폴더(기본 <dest>/logs/add_site/<이름>/): site_report.md · site_report.json · manifest_rows.md(MANIFEST 행 초안 — 직접 옮겨 적음) · 단계별 로그.
끝 줄: SITE_OK | SITE_WARN | SITE_FAIL (FAIL 이면 종료 코드 3). 시뮬은 띄우지 않는다 — 주행 점검 명령은 site_report.md 끝에 적는다.

파이썬: 이 파일은 시스템 python3(PyYAML). pxr 이 필요한 단계는 FD_USD_PY(기본 envs/isaacsim_env/bin/python),
recon 과 맵 판정은 FD_RECON_PY(기본 envs/recon_env/bin/python)로 돌린다.
"""
import argparse
import glob
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import time
import zipfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POSE_NAMES = ("trajectory_camera_optical.txt", "poses.txt", "groundtruth.txt", "trajectory.txt")


# ── pxr 이 필요한 계산(FD_USD_PY 로 이 파일을 다시 부름) ─────────────────

def _helper(argv):
    import numpy as np
    import yaml
    from pxr import Usd, UsdGeom
    mode, mesh = argv[0], argv[1]
    st = Usd.Stage.Open(mesh)
    coll = next(p for p in Usd.PrimRange(st.GetDefaultPrim()) if p.GetName() == "Collision")
    m = UsdGeom.Mesh(coll)
    P = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
    F = np.array(m.GetFaceVertexIndicesAttr().Get(), dtype=np.int64).reshape(-1, 3)
    M = np.array(UsdGeom.Xformable(coll).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=np.float64)
    P = (np.hstack([P, np.ones((len(P), 1))]) @ M)[:, :3]
    A, B, C = P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]
    N = np.cross(B - A, C - A)
    up = N[:, 2] / np.maximum(np.linalg.norm(N, axis=1), 1e-30) > 0.9
    cen = (A + B + C) / 3

    def floor(x, y):
        near = up & (np.hypot(cen[:, 0] - x, cen[:, 1] - y) < 0.4)
        return (float(np.median(cen[near, 2])) if near.any() else 0.0), int(near.sum())

    if mode == "floor":
        x, y = float(argv[2]), float(argv[3])
        z, n = floor(x, y)
        print("HELPER " + json.dumps(dict(x=x, y=y, floor_z=round(z, 4), floor_tris_near=n)))
        return
    # pick: 빈 칸 아닌 칸에서 가장 먼 빈 칸, 방향은 가장 긴 빈 직선(사이드 2a 의 pick_start.py 와 같은 규칙)
    from scipy import ndimage
    map_dir = argv[2]
    meta = yaml.safe_load(open(os.path.join(map_dir, "map.yaml"), encoding="utf-8"))
    res = float(meta["resolution"])
    ox, oy = (float(v) for v in meta["origin"][:2])
    with open(os.path.join(map_dir, meta["image"]), "rb") as f:
        assert f.readline().strip() == b"P5"
        w, h = map(int, f.readline().split())
        f.readline()
        im = np.frombuffer(f.read(), np.uint8).reshape(h, w)
    free = im == 254
    dist = ndimage.distance_transform_edt(free) * res
    r, c = (int(v) for v in np.unravel_index(np.argmax(dist), dist.shape))
    x, y = ox + (c + 0.5) * res, oy + (h - 1 - r + 0.5) * res
    best, yaw = 0.0, 0.0
    for deg in range(0, 360, 5):
        dx, dy = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        n = 0
        while True:
            cc, rr = int(round(c + dx * (n + 1))), int(round(r - dy * (n + 1)))
            if not (0 <= cc < w and 0 <= rr < h) or not free[rr, cc]:
                break
            n += 1
        if n * res > best:
            best, yaw = n * res, float(deg)
    z, n = floor(x, y)
    foot = math.hypot(0.607, 0.25)
    print("HELPER " + json.dumps(dict(x=round(x, 3), y=round(y, 3), yaw_deg=yaw, floor_z=round(z, 4), floor_tris_near=n,
                                      clearance_m=round(float(dist[r, c]), 3), longest_free_ray_m=round(best, 2),
                                      free_area_m2=round(float(free.sum()) * res * res, 2),
                                      area_clear_ge_footprint_m2=round(float((dist >= foot).sum()) * res * res, 2))))


# ── 작은 도구 ────────────────────────────────────────────────────────

def sha12(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()[:12]


def du_h(path):
    n = 0
    for d, _, fs in os.walk(path):
        for f in fs:
            p = os.path.join(d, f)
            if not os.path.islink(p):
                n += os.path.getsize(p)
    for unit, k in (("G", 1 << 30), ("M", 1 << 20), ("K", 1 << 10)):
        if n >= k:
            return f"{n / k:.1f}{unit}" if n / k < 10 else f"{n / k:.0f}{unit}"
    return f"{n}B"


def yaml_data(path):
    import yaml
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def same_file(a, b):
    """'same' 바이트 단위로 같음 · 'same_data' yaml 내용은 같고 주석 · 줄바꿈만 다름 · 'diff'."""
    with open(a, "rb") as fa, open(b, "rb") as fb:
        if fa.read() == fb.read():
            return "same"
    if a.endswith((".yaml", ".yml")):
        try:
            if yaml_data(a) == yaml_data(b):
                return "same_data"
        except Exception:
            pass
    return "diff"


class Run:
    def __init__(self, work):
        self.work, self.rows, self.t0 = work, [], time.time()

    def add(self, step, level, msg):
        self.rows.append(dict(step=step, level=level, msg=msg))
        print(f"[{step}] {dict(ok='OK', warn='WARN', fail='FAIL', skip='SKIP')[level]} — {msg}", flush=True)

    def sh(self, cmd, log, cwd=None, env=None, nice=False):
        e = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
        e.update(env or {})
        if nice:
            cmd = ["nice", "-n", "19"] + cmd
        lp = os.path.join(self.work, log)
        with open(lp, "a", encoding="utf-8") as fh:
            fh.write(f"\n===== {time.strftime('%F %T')} (cwd {cwd or os.getcwd()}) {' '.join(cmd)}\n")
            fh.flush()
            p = subprocess.run(cmd, cwd=cwd, env=e, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
            fh.write(p.stdout)
        return p.returncode, p.stdout

    def level(self):
        ls = {r["level"] for r in self.rows}
        return "fail" if "fail" in ls else "warn" if "warn" in ls else "ok"


class Stop(Exception):
    pass


# ── 입력 ─────────────────────────────────────────────────────────────

def find_mesh(usd_dir):
    c = [p for p in sorted(glob.glob(os.path.join(usd_dir, "*_mesh.usdc"))) if not os.path.islink(p)]
    return c[0] if c else None


def detect(inp, a):
    if os.path.isfile(inp):
        if inp.endswith(".zip"):
            return "zip"
        if inp.endswith((".yaml", ".yml")):
            return "recon_cfg"
        raise Stop(f"입력 파일을 알 수 없음(zip · yaml 만): {inp}")
    if not os.path.isdir(inp):
        raise Stop(f"입력이 없음: {inp}")
    if find_mesh(os.path.join(inp, "usd")):
        return "recon_out" if os.path.isfile(os.path.join(inp, "report.json")) else "package"   # run.py 출력 폴더는 중간 파일이 많아 usd · 점검 결과만 가져감
    if os.path.isdir(os.path.join(inp, a.keyframes, "rgb")) and os.path.isdir(os.path.join(inp, a.keyframes, "depth")):
        return "rtabmap_keyframes"
    if os.path.isfile(os.path.join(inp, "rgb.txt")) and os.path.isfile(os.path.join(inp, "depth.txt")):
        return "tum"
    raise Stop("입력 모양을 알 수 없음 — usd/*_mesh.usdc(패키지) · keyframes/rgb + depth(rtabmap_keyframes) · rgb.txt + depth.txt(tum) 중 하나여야 함")


def pose_file(inp, a, cols):
    names = [a.poses] if a.poses else POSE_NAMES
    for n in names:
        p = os.path.join(inp, n)
        if os.path.isfile(p):
            with open(p, encoding="utf-8") as f:
                rows = [l.split() for l in f if l.strip() and not l.startswith("#")]
            bad = [i for i, r in enumerate(rows) if len(r) != cols]
            if not rows or bad:
                raise Stop(f"자세 파일 {n}: 칸 수가 {cols} 이 아닌 줄이 {len(bad)}개(전체 {len(rows)}줄) — "
                           + ("시각 tx ty tz qx qy qz qw id" if cols == 9 else "시각 tx ty tz qx qy qz qw"))
            return n, len(rows)
    raise Stop(f"자세 파일을 못 찾음({', '.join(names)}) — --poses 로 이름을 줄 것")


def write_recon_cfg(kind, inp, a, R, out):
    import yaml
    cfg = dict(base=os.path.join(REPO, "recon", "configs", "templates", "indoor_rtabmap.yaml"), name=a.site, out=out)
    if kind == "rtabmap_keyframes":
        pf, n = pose_file(inp, a, 9)
        cal = len(glob.glob(os.path.join(inp, a.keyframes, "calib", "*.yaml")))
        if not cal:
            raise Stop(f"{a.keyframes}/calib/<id>.yaml 이 없음 — 키프레임마다 내부 파라미터 파일이 있어야 함")
        cfg["input"] = dict(format="rtabmap_keyframes", root=inp, keyframes=a.keyframes, poses=pf, depth_scale=a.depth_scale)
        note = f"rtabmap_keyframes · 자세 {pf} {n}줄 · calib {cal}개"
    else:
        pf, n = pose_file(inp, a, 8)
        if not a.intrinsics:
            raise Stop("tum 형식은 내부 파라미터를 따로 줘야 함: --intrinsics 가로,세로,fx,fy,cx,cy")
        v = [float(x) for x in a.intrinsics.split(",")]
        if len(v) != 6:
            raise Stop("--intrinsics 는 값 6개: 가로,세로,fx,fy,cx,cy")
        cfg["input"] = dict(format="tum", root=inp, poses=pf, depth_scale=a.depth_scale,
                            intrinsics=dict(width=int(v[0]), height=int(v[1]), fx=v[2], fy=v[3], cx=v[4], cy=v[5]))
        note = f"tum · 자세 {pf} {n}줄 · 내부 파라미터 {a.intrinsics}"
    cfg["colorize"] = None                      # 받은 형상 메쉬가 없으면 TSDF 메쉬에 이미 색이 있음(lounge_indoor.yaml 과 같음)
    if a.frame_step != 1:
        cfg["tsdf"] = dict(frame_step=a.frame_step)
    if a.up == "camera":                        # 자세에 중력 방향이 없을 때(바닥을 카메라 방향으로 세움) — 좌표도 새로 잡음
        cfg["orient"] = dict(up_hint="camera", origin="min_corner")
    p = os.path.join(R.work, "recon_cfg.yaml")
    with open(p, "w", encoding="utf-8") as f:
        f.write(f"# sim/add_site.py 가 쓴 장소 설정({time.strftime('%F %T')}) — 입력 {inp}\n")
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
    return p, note


def run_recon(cfg_path, out, a, R):
    summ = os.path.join(out, "run_summary.json")
    if os.path.isfile(summ) and not a.rebuild and json.load(open(summ, encoding="utf-8")).get("usd_ok"):
        R.add("recon", "ok", f"이미 만든 결과를 씀(--rebuild 로 다시): {out}")
        return
    py = a.recon_py
    if not os.path.exists(py):
        raise Stop(f"recon 파이썬이 없음: {py} (FD_RECON_PY)")
    t = time.time()
    rc, o = R.sh([py, os.path.join(REPO, "recon", "pipeline", "run.py"), "--config", cfg_path, "--out", out, "--no-preview"], "recon.log", nice=True)
    if rc:
        tail = " / ".join(l.strip() for l in o.strip().splitlines()[-3:])
        raise Stop(f"recon/pipeline/run.py 실패(코드 {rc}) — {tail[:300]} · 로그 {R.work}/recon.log · 입력 점검 {out}/check_input.json")
    s = json.load(open(summ, encoding="utf-8"))
    vis = (s.get("visibility") or {}).get("holes_visible_pct")
    col = (s.get("collider_err") or {}).get("p95_cm")
    R.add("recon", "ok" if s.get("usd_ok") else "fail",
          f"{(time.time() - t) / 60:.1f}분 · 단계 {' · '.join(f'{k} {v:.0f}s' for k, v in s.get('steps', []))} · USD 검증 {'통과' if s.get('usd_ok') else '문제'}"
          f" · 충돌 메쉬 오차 95 % {col} cm · 주행 중 보이는 구멍 {vis} %")


def package_from_recon(out, dst):
    os.makedirs(dst)
    shutil.copytree(os.path.join(out, "usd"), os.path.join(dst, "usd"), symlinks=True)
    os.makedirs(os.path.join(dst, "check"))
    for src, name in (("transform.json", "transform.json"), ("check_input.json", "check/check_input.json"), ("report.json", "check/report.json"),
                      ("run_summary.json", "check/run_summary.json"), ("visibility/summary.json", "check/visibility_summary.json")):
        if os.path.isfile(os.path.join(out, src)):
            shutil.copy(os.path.join(out, src), os.path.join(dst, name))


# ── 씬 루트 ──────────────────────────────────────────────────────────

def scene_root_text(stem, scene_usda, st, why):
    return f'''#usda 1.0
(
    customLayerData = {{
        string generated_by = "sim/add_site.py"
        string lights = "돔 600 · 해 0 (실내 기본값 — 복도 v0.2 루트와 같음)"
        string map = "data/usd/{stem}/"
        string start_pose = "{why} ({st['x']:.3f}, {st['y']:.3f}) · 방향 {st['yaw_deg']:.1f}°. 바닥 {st['floor_z']:.3f} + 0.022"
    }}
    defaultPrim = "World"
    doc = "씬 루트 — sim/add_site.py 가 복도 v0.2 루트의 구성(센서 · ROS · 로봇 · 맵 · 월드)으로 씀"
    metersPerUnit = 1
    subLayers = [
        @../proxy/sensors.usda@,
        @../proxy/ros.usda@,
        @../proxy/robot.usda@,
        @../../../data/usd/{stem}/usd/{scene_usda}@,
        @../proxy/world.usda@
    ]
    timeCodesPerSecond = 60
    upAxis = "Z"
)

over "World"
{{
    over "Nova_Carter_ROS"
    {{
        float xformOp:rotateZ = {st['yaw_deg']:.1f}
        double3 xformOp:translate = ({st['x']:.3f}, {st['y']:.3f}, {st['floor_z'] + 0.022:.3f})
    }}

    over "Lights"
    {{
        over "Dome"
        {{
            float inputs:intensity = 600
        }}

        over "Sun"
        {{
            float inputs:intensity = 0
        }}
    }}
}}
'''


def helper(R, a, *args):
    rc, o = R.sh([a.usd_py, os.path.abspath(__file__), "--_helper"] + [str(x) for x in args], "helper.log")
    line = next((l for l in o.splitlines() if l.startswith("HELPER ")), None)
    if rc or not line:
        raise Stop(f"출발 자세 계산 실패(코드 {rc}) — 로그 {R.work}/helper.log")
    return json.loads(line[7:])


# ── 본체 ─────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="입력(RGB-D + 자세 · 맵 패키지) → 맵 · Nav2 지도 · 경로 · 시나리오 · 씬 루트 · 맵 판정")
    ap.add_argument("input")
    ap.add_argument("--site", required=True, help="장소 이름(영문 — sim/scenes/<장소>/, 메쉬 이름 <장소>_mesh.usdc)")
    ap.add_argument("--ver", required=True, help="판(v0.2 · v0.8_r3 …) — <이름> = <장소>_<판>")
    ap.add_argument("--route", help="<이름>_route.yaml(기준선을 만듦) 또는 이미 있는 기준선 .csv")
    ap.add_argument("--dirs", default="cw,ccw", help="--route 가 이미 있는 .csv 일 때 맵 판정에 쓸 두 방향 이름(열린 경로면 fwd,back)")
    ap.add_argument("--group", help="씬 루트를 둘 sim/scenes/ 아래 폴더(기본: 장소 이름) — parking_polycam_v1 을 sim/scenes/parking/ 에 둘 때처럼")
    ap.add_argument("--keepout", help="sim/nav2/make_map.py --keepout 에 넘길 yaml(가운데 섬을 채운 경로 계획 지도도 만듦)")
    ap.add_argument("--dest", default=REPO, help="제자리(기본: 이 레포)")
    ap.add_argument("--work", help="작업 폴더(기본: <dest>/logs/add_site/<이름>)")
    ap.add_argument("--no-install", action="store_true", help="제자리에 놓지 않고 만들어 보고 대조만")
    ap.add_argument("--rebuild", action="store_true", help="recon 결과가 있어도 다시 만듦")
    ap.add_argument("--keyframes", default="keyframes", help="rtabmap_keyframes: 키프레임 폴더 이름")
    ap.add_argument("--poses", help="자세 파일 이름(입력 폴더 기준)")
    ap.add_argument("--intrinsics", help="tum: 가로,세로,fx,fy,cx,cy")
    ap.add_argument("--depth-scale", type=float, default=1000.0, help="깊이 PNG 값 / 이 값 = m")
    ap.add_argument("--frame-step", type=int, default=1, help="몇 장마다 한 장을 쓸지(동영상 프레임이면 5)")
    ap.add_argument("--up", choices=("z", "camera"), default="z", help="자세가 중력 기준 Z-up 이면 z, 중력 정보가 없으면 camera")
    ap.add_argument("--usd-py", default=os.environ.get("FD_USD_PY") or os.path.join(REPO, "envs", "isaacsim_env", "bin", "python"))
    ap.add_argument("--recon-py", default=os.environ.get("FD_RECON_PY") or os.path.join(REPO, "envs", "recon_env", "bin", "python"))
    ap.add_argument("--gate-dir", default=os.environ.get("FD_GATE_DIR"), help="맵 판정 도구 폴더(map_gate.py · gate_verdict.py)")
    a = ap.parse_args()

    stem = f"{a.site}_{a.ver}"
    dest = os.path.abspath(os.path.expanduser(a.dest))
    work = os.path.abspath(os.path.expanduser(a.work or os.path.join(dest, "logs", "add_site", stem)))
    T = os.path.join(work, "tree")
    os.makedirs(work, exist_ok=True)
    shutil.rmtree(T, ignore_errors=True)
    for f in glob.glob(os.path.join(work, "*.log")) + glob.glob(os.path.join(work, "layer_*.usda")):
        os.remove(f)
    shutil.rmtree(os.path.join(work, "gate"), ignore_errors=True)
    os.makedirs(T)
    R = Run(work)
    inp = os.path.abspath(os.path.expanduser(a.input))
    rel_usd, rel_nav, rel_scene = f"data/usd/{stem}", f"data/nav2/{stem}", f"sim/scenes/{a.group or a.site}/{stem}.usda"
    info = dict(name=stem, site=a.site, ver=a.ver, input=inp, dest=dest, work=work, started=time.strftime("%F %T"), artifacts=[])
    print(f"add_site: {stem} ← {inp}\n  제자리 {dest}{' (놓지 않음 — 대조만)' if a.no_install else ''} · 작업 폴더 {work}", flush=True)
    try:
        # ── input · recon · usd
        kind = detect(inp, a)
        info["input_kind"] = kind
        pkg_t = os.path.join(T, rel_usd)
        os.makedirs(os.path.dirname(pkg_t))
        existing = os.path.join(dest, rel_usd)
        if kind == "zip":
            R.add("input", "ok", f"맵 패키지 zip · sha {sha12(inp)}")
            info["zip_sha12"] = sha12(inp)
            unz = os.path.join(work, "unz")
            shutil.rmtree(unz, ignore_errors=True)
            with zipfile.ZipFile(inp) as z:
                z.extractall(unz)
            tops = os.listdir(unz)
            src = os.path.join(unz, tops[0]) if len(tops) == 1 and os.path.isdir(os.path.join(unz, tops[0])) else unz
            shutil.move(src, pkg_t)
            shutil.rmtree(unz, ignore_errors=True)
        elif kind == "package":
            R.add("input", "ok", "맵 패키지 폴더")
            if os.path.exists(existing) and os.path.realpath(existing) == os.path.realpath(inp):
                os.symlink(os.path.realpath(inp), pkg_t)          # 이미 제자리에 있는 패키지 — 복사하지 않고 읽기만
            else:
                shutil.copytree(inp, pkg_t, symlinks=True)
        elif kind == "recon_out":
            R.add("input", "ok", "recon/pipeline/run.py 출력 폴더 — usd 와 점검 결과만 가져감")
            package_from_recon(inp, pkg_t)
        else:
            out = os.path.join(work, "recon_out")
            if kind == "recon_cfg":
                cfg_path, note = inp, "recon 설정 yaml"
            else:
                cfg_path, note = write_recon_cfg(kind, inp, a, R, out)
            R.add("input", "ok", f"RGB-D + 자세 — {note} → 장소 설정 {cfg_path}")
            run_recon(cfg_path, out, a, R)
            package_from_recon(out, pkg_t)
        mesh = find_mesh(os.path.join(pkg_t, "usd"))
        if not mesh:
            raise Stop(f"패키지에 usd/*_mesh.usdc 가 없음: {pkg_t}/usd")
        scene_usdas = sorted(glob.glob(os.path.join(pkg_t, "usd", "*_scene.usda")))
        if len(scene_usdas) != 1:
            raise Stop(f"패키지의 usd/*_scene.usda 가 하나가 아님: {[os.path.basename(s) for s in scene_usdas]}")
        mesh_name, mesh_sha = os.path.basename(mesh), sha12(mesh)
        info.update(mesh=f"{rel_usd}/usd/{mesh_name}", mesh_sha12=mesh_sha, usd_size=du_h(os.path.realpath(pkg_t)))
        note = ""
        if mesh_name != f"{a.site}_mesh.usdc":
            note = f" · 메쉬 이름이 {a.site}_mesh.usdc 가 아님(찾는 데는 문제없음)"
        R.add("usd", "ok", f"{rel_usd}/ {info['usd_size']} · 메쉬 {mesh_name} sha {mesh_sha}{note}")

        # ── nav2
        if not os.path.exists(a.usd_py):
            raise Stop(f"pxr 파이썬이 없음: {a.usd_py} (FD_USD_PY)")
        cmd = [a.usd_py, os.path.join(REPO, "sim", "nav2", "make_map.py"), f"{rel_usd}/usd/{mesh_name}", rel_nav, "--route", "none"]
        if a.keepout:
            cmd += ["--keepout", os.path.abspath(a.keepout)]
        rc, o = R.sh(cmd, "make_map.log", cwd=T)
        nav_t = os.path.join(T, rel_nav)
        if rc or not os.path.isfile(os.path.join(nav_t, "map.pgm")):
            raise Stop(f"sim/nav2/make_map.py 실패(코드 {rc}) — 로그 {work}/make_map.log")
        maps = ["map.pgm"] + (["map_keepout.pgm"] if os.path.isfile(os.path.join(nav_t, "map_keepout.pgm")) else [])
        info["map_sha12"] = {m: sha12(os.path.join(nav_t, m)) for m in maps}
        my = yaml_data(os.path.join(nav_t, "map.yaml"))
        with open(os.path.join(nav_t, "map.pgm"), "rb") as f:
            f.readline()
            mw, mh = map(int, f.readline().split())
        info["map"] = dict(w=mw, h=mh, res=my["resolution"], origin=my["origin"][:2])
        ref = os.path.join(pkg_t, "nav2_reference")
        refnote, lvl = "", "ok"
        if os.path.isdir(ref):
            cmpd = {m: (sha12(os.path.join(ref, m)) == info["map_sha12"][m]) for m in maps if os.path.isfile(os.path.join(ref, m))}
            info["map_vs_reference"] = cmpd
            if cmpd:
                refnote = " · 패키지의 참고 지도와 " + ("바이트 단위로 같음" if all(cmpd.values()) else f"다름 {[m for m, v in cmpd.items() if not v]}")
                lvl = "ok" if all(cmpd.values()) else "warn"
        R.add("nav2", lvl, f"{rel_nav}/ {mw}×{mh} 칸 · {my['resolution']} m · 원점 ({my['origin'][0]}, {my['origin'][1]}) · "
              + " · ".join(f"{m} sha {s}" for m, s in info["map_sha12"].items()) + refnote)

        # ── route
        start, start_why, gate_route, gate_dirs, scenarios = None, "", None, None, []
        if a.route and a.route.endswith((".yaml", ".yml")):
            rcfg = yaml_data(a.route)
            closed = rcfg.get("kind", "open") == "loop"
            nm = rcfg.get("names") or {}
            tags = ("cw", "ccw") if closed else (nm.get("a", "fwd"), nm.get("b", "back"))
            rc, o = R.sh([sys.executable, os.path.join(REPO, "eval", "routes", "make_route.py"), os.path.abspath(a.route), "--root", T,
                          "--nav2-map", os.path.join(nav_t, "map.yaml"), "--scenario"], "make_route.log")
            last = o.strip().splitlines()[-1] if o.strip() else ""
            if last not in ("ROUTE_OK", "ROUTE_WARN", "ROUTE_FAIL"):
                raise Stop(f"eval/routes/make_route.py 가 끝까지 못 감(코드 {rc}) — 로그 {work}/make_route.log")
            head = next((l.strip() for l in o.splitlines() if "경로 · 길이" in l), "")
            clr = next((l.strip() for l in o.splitlines() if l.strip().startswith("지도 위 여유")), "")
            ry = yaml_data(os.path.join(T, "eval", "routes", f"{rcfg['name']}_{tags[0]}.yaml"))
            start = dict(x=float(ry["start"]["x"]), y=float(ry["start"]["y"]), yaw_deg=float(ry["start"]["yaw_deg"]))
            start_why = f"기준선 {rcfg['name']}_{tags[0]} 의 출발점"
            gate_route, gate_dirs = os.path.join(T, "eval", "routes", f"{rcfg['name']}_{tags[0]}.csv"), ",".join(tags)
            scenarios = sorted(os.path.basename(p) for p in glob.glob(os.path.join(T, "eval", "scenarios", "*.yaml")))
            info["route"] = dict(config=os.path.abspath(a.route), name=rcfg["name"], kind="loop" if closed else "open", tags=list(tags),
                                 length_m=ry.get("length_m"), clearance=ry.get("clearance"), result=last, scenarios=scenarios)
            warn = ""
            if rcfg.get("scene") != rel_scene:
                warn = f" · 경로 파일의 scene({rcfg.get('scene')})이 이 장소의 씬 루트({rel_scene})와 다름"
            lv = "fail" if last == "ROUTE_FAIL" else "warn" if (last == "ROUTE_WARN" or warn) else "ok"
            R.add("route", lv, f"{head} · {clr} · 시나리오 {', '.join(scenarios) or '없음'} → {last}{warn}")
        elif a.route:
            gate_route = os.path.abspath(a.route)
            if not os.path.isfile(gate_route):
                raise Stop(f"기준선이 없음: {a.route}")
            gate_dirs = a.dirs
            info["route"] = dict(existing=a.route)
            R.add("route", "skip", f"이미 있는 기준선을 씀(새로 만들지 않음): {a.route}")
        else:
            R.add("route", "skip", "경로를 안 줌 — 기준선 · 시나리오를 만들지 않음(지도에서 출발점만 잡음)")

        # ── scene
        scene_t = os.path.join(T, rel_scene)
        os.makedirs(os.path.dirname(scene_t))
        sug = os.path.join(pkg_t, "scene_suggestion", f"{stem}.usda")
        if os.path.isfile(sug):
            shutil.copy(sug, scene_t)
            info["scene"] = dict(source="package scene_suggestion")
            R.add("scene", "ok", f"{rel_scene} — 패키지의 씬 루트 제안 그대로")
        else:
            if start:
                h = helper(R, a, "floor", os.path.realpath(mesh), start["x"], start["y"])
                start["floor_z"] = h["floor_z"]
                extra = f"바닥 삼각형 {h['floor_tris_near']}개"
                lv = "ok" if h["floor_tris_near"] else "warn"
            else:
                h = helper(R, a, "pick", os.path.realpath(mesh), nav_t)
                start = dict(x=h["x"], y=h["y"], yaw_deg=h["yaw_deg"], floor_z=h["floor_z"])
                start_why = "지도에서 벽 · 미지 칸에서 가장 먼 빈 칸"
                extra = f"여유 {h['clearance_m']} m · 가장 긴 빈 직선 {h['longest_free_ray_m']} m · 제자리 회전 가능 넓이 {h['area_clear_ge_footprint_m2']} m²"
                lv = "ok" if h["clearance_m"] >= math.hypot(0.607, 0.25) else "warn"
                info["pick"] = h
            with open(scene_t, "w", encoding="utf-8") as f:
                f.write(scene_root_text(stem, os.path.basename(scene_usdas[0]), start, start_why))
            info["scene"] = dict(source="generated", start=start, why=start_why)
            R.add("scene", lv, f"{rel_scene} — 새로 씀: {start_why} ({start['x']:.3f}, {start['y']:.3f}) {start['yaw_deg']:.1f}° · 바닥 {start['floor_z']:.3f} m · {extra}")

        # ── gate
        gd = a.gate_dir and os.path.expanduser(a.gate_dir)
        if not gd or not os.path.isfile(os.path.join(gd, "map_gate.py")):
            R.add("gate", "skip", "맵 판정 도구가 없음(FD_GATE_DIR) — 건너뜀")
        else:
            gout = os.path.join(work, "gate")
            cmd = [a.recon_py, os.path.join(gd, "map_gate.py"), "--name", stem, "--mesh", os.path.realpath(mesh),
                   "--map", os.path.join(nav_t, "map.yaml"), "--out", os.path.join(gout, stem)]
            if gate_route:
                cmd += ["--route", gate_route, "--dirs", gate_dirs]
            else:
                st = info["scene"].get("start") or start
                if not st:
                    import re
                    txt = open(scene_t, encoding="utf-8").read()
                    mt = re.search(r"xformOp:translate = \(([-0-9.e]+), ([-0-9.e]+),", txt)
                    mr = re.search(r"xformOp:rotateZ = ([-0-9.e]+)", txt)
                    st = dict(x=float(mt.group(1)), y=float(mt.group(2)), yaw_deg=float(mr.group(1)) if mr else 0.0)
                cmd += ["--start", f"{st['x']},{st['y']},{st['yaw_deg']}"]
            env = dict(FD_GATE_THREADS=os.environ.get("FD_GATE_THREADS", "4"), OMP_NUM_THREADS=os.environ.get("FD_GATE_THREADS", "4"))
            rc, o = R.sh(cmd, "gate.log", env=env, nice=True)
            rc2, o2 = R.sh([a.recon_py, os.path.join(gd, "gate_verdict.py"), stem], "gate.log", env=dict(env, FD_GATE_OUT=gout))
            vp = os.path.join(gout, stem, "verdict.json")
            if rc or rc2 or not os.path.isfile(vp):
                R.add("gate", "warn", f"맵 판정이 끝까지 못 감(코드 {rc} · {rc2}) — 로그 {work}/gate.log")
            else:
                v = json.load(open(vp, encoding="utf-8"))
                bad = [i for i in v.get("items", []) if i.get("level") != "통과"]
                info["gate"] = dict(verdict=v["verdict"], items=bad)
                R.add("gate", {"통과": "ok", "주의": "warn"}.get(v["verdict"], "fail"),
                      f"{v['verdict']}" + (" — " + " / ".join(f"{i.get('level')}: {i.get('what')} {i.get('value')}" for i in bad[:6]) if bad else ""))

        # ── place
        arts = [("dir", rel_usd), ("dir", rel_nav), ("file", rel_scene)]
        arts += [("file", os.path.relpath(p, T)) for p in sorted(glob.glob(os.path.join(T, "eval", "routes", "*")) + glob.glob(os.path.join(T, "eval", "scenarios", "*")))]
        counts = {}
        route_failed = (info.get("route") or {}).get("result") == "ROUTE_FAIL"
        for typ, rel in arts:
            src, dst = os.path.join(T, rel), os.path.join(dest, rel)
            if not os.path.lexists(dst):
                if a.no_install or route_failed:
                    st_ = "new_not_placed"
                else:
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    if os.path.islink(src):
                        raise Stop(f"놓을 것이 링크임(있을 수 없음): {src}")
                    if typ == "dir":
                        shutil.copytree(src, dst, symlinks=True)
                    else:
                        shutil.copy(src, dst)
                    st_ = "placed"
                detail = ""
            elif typ == "file":
                st_ = same_file(src, dst)
                detail = ""
            elif rel == rel_usd:
                dm = find_mesh(os.path.join(dst, "usd"))
                same = dm and sha12(dm) == mesh_sha
                st_, detail = ("same" if same else "diff"), f"메쉬 sha {sha12(dm) if dm else '없음'}"
            else:
                res = {}
                for fn in sorted(os.listdir(src)):
                    if fn.endswith(".png"):
                        continue                                   # 미리보기 그림은 겹쳐 그린 기준선에 따라 달라짐 — 대조하지 않음
                    res[fn] = same_file(os.path.join(src, fn), os.path.join(dst, fn)) if os.path.isfile(os.path.join(dst, fn)) else "missing"
                st_ = "same" if all(v in ("same", "same_data") for v in res.values()) else "diff"
                detail = " · ".join(f"{k} {v}" for k, v in res.items())
            counts[st_] = counts.get(st_, 0) + 1
            info["artifacts"].append(dict(path=rel, status=st_, detail=detail))
        word = dict(placed="새로 놓음", new_not_placed="새 것(놓지 않음)", same="있는 것과 같음", same_data="있는 것과 내용 같음(주석만 다름)", diff="있는 것과 다름(덮어쓰지 않음)")
        R.add("place", "warn" if counts.get("diff") else "ok", ("경로가 실패라 아무것도 놓지 않음(경로를 고쳐 다시 돌릴 것) — " if route_failed and not a.no_install else "") + " · ".join(f"{word[k]} {v}" for k, v in counts.items())
              + "".join(f" · 다름: {x['path']}{' (' + x['detail'] + ')' if x['detail'] else ''}" for x in info["artifacts"] if x["status"] == "diff"))

        # ── layer
        todo = [s for s in scenarios if os.path.isfile(os.path.join(dest, "eval", "scenarios", s))]
        if not scenarios:
            R.add("layer", "skip", "시나리오가 없음")
        elif os.path.realpath(dest) != os.path.realpath(REPO) or not todo or not os.path.isfile(os.path.join(dest, rel_scene)):
            R.add("layer", "skip", "제자리에 놓지 않았거나 제자리가 이 레포가 아니라서 레이어 시험을 건너뜀")
        else:
            okn, lines = 0, []
            for s in todo:
                rc, o = R.sh([a.usd_py, os.path.join(REPO, "sim", "make_obstacles.py"), f"eval/scenarios/{s}", os.path.join(work, f"layer_{s[:-5]}.usda")],
                             "layer.log", cwd=dest, env=dict(CD1=dest))
                stl = next((l.strip() for l in o.splitlines() if l.startswith("START")), "")
                okn += int(rc == 0 and "SAVED" in o)
                lines.append(f"{s[:-5]}: {stl or '출발 줄 없음'}")
            info["layer"] = lines
            R.add("layer", "ok" if okn == len(todo) else "fail", f"{okn}/{len(todo)} 만들어짐 — " + " / ".join(lines))
    except Stop as e:
        R.add("stop", "fail", str(e))

    # ── 기록
    level = R.level()
    info.update(rows=R.rows, level=level, seconds=round(time.time() - R.t0, 1))
    json.dump(info, open(os.path.join(work, "site_report.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    day = time.strftime("%m.%d").lstrip("0").replace(".0", ".")
    if info.get("mesh_sha12"):
        src_note = {"zip": f"맵 패키지 zip {os.path.basename(inp)} {info.get('zip_sha12', '')}", "package": f"맵 패키지 폴더 {os.path.basename(inp)}",
                    "recon_out": f"recon/pipeline/run.py 출력 {inp}", "recon_cfg": f"recon/pipeline/run.py --config {inp}"}.get(
            info.get("input_kind"), f"RGB-D + 자세({info.get('input_kind')}) {inp} → recon/pipeline/run.py(실내 틀)")
        with open(os.path.join(work, "manifest_rows.md"), "w", encoding="utf-8") as f:
            f.write("<!-- MANIFEST.md 행 초안 — sim/add_site.py. 출처 · 비고를 고쳐서 직접 옮겨 적을 것 -->\n")
            f.write(f"| {day} | `usd` | `{a.site}` | {a.ver} | `{rel_usd}/` | {info.get('usd_size')} | {info['mesh_sha12']} | `{src_note}, sha 는 usd/{os.path.basename(info['mesh'])}` | D | `씬 루트 {rel_scene}({'패키지 제안 그대로' if (info.get('scene') or {}).get('source') != 'generated' else 'add_site 가 씀'}). sim/add_site.py 로 들임` |\n")
            if info.get("map_sha12"):
                mp = info["map"]
                f.write(f"| {day} | `nav2` | `{stem}` | v1 | `{rel_nav}/` | — | {info['map_sha12']['map.pgm']} | `sim/nav2/make_map.py {info['mesh']} {rel_nav}{' --keepout ' + a.keepout if a.keepout else ''}(Collision, 0.12~1.0 m)` | D | `sha 는 map.pgm. {mp['w']}×{mp['h']} 칸 · {mp['res']} m · 원점 ({mp['origin'][0]}, {mp['origin'][1]})` |\n")
    with open(os.path.join(work, "site_report.md"), "w", encoding="utf-8") as f:
        f.write(f"# {stem} — sim/add_site.py ({info['started']}, {info['seconds']} s)\n\n입력 `{inp}` · 제자리 `{dest}`{' (놓지 않음)' if a.no_install else ''}\n\n")
        f.write("| 단계 | 결과 | 내용 |\n| --- | --- | --- |\n")
        for r in R.rows:
            f.write(f"| {r['step']} | {r['level']} | {r['msg'].replace('|', '/')} |\n")
        if info["artifacts"]:
            f.write("\n| 놓는 것 | 상태 | |\n| --- | --- | --- |\n")
            for x in info["artifacts"]:
                f.write(f"| `{x['path']}` | {x['status']} | {x['detail']} |\n")
        scs = (info.get("route") or {}).get("scenarios") or []
        f.write("\n## 다음 — 주행 점검(시뮬, 본 작업이 안 도는 시간에)\n\n```bash\n")
        for s in scs:
            f.write(f"FD_DRY=1 FD_OUT=logs/nav2/{stem}_dry sim/nav2/scenario_batch.sh eval/scenarios/{s}   # 씬을 안 띄우고 찾은 씬 · 지도 · 기준선만\n")
            f.write(f"FD_OUT=logs/nav2/{stem} sim/nav2/scenario_batch.sh eval/scenarios/{s}                  # Nav2 + AMCL 한 회\n")
        if not scs:
            f.write(f"# 시나리오가 없음 — 경로(--route <이름>_route.yaml)를 주고 다시 돌리면 생김. 씬만 열어 볼 때: FD_SCENE={rel_scene} sim/run_proxy.sh headless 110\n")
        f.write("```\n")
    print(f"기록: {work}/site_report.md · site_report.json" + (" · manifest_rows.md" if info.get("mesh_sha12") else ""))
    print({"ok": "SITE_OK", "warn": "SITE_WARN", "fail": "SITE_FAIL"}[level])
    return 3 if level == "fail" else 0


if __name__ == "__main__":
    if sys.argv[1:2] == ["--_helper"]:
        _helper(sys.argv[2:])
        sys.exit(0)
    sys.exit(main())
