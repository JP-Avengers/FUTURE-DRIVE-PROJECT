"""Nav2 장애물 회피 시연 영상 — 실제 주차장 r3 씬에 시나리오 장애물을 얹고 Nav2(+ AMCL)가 기준선을 도는 모습을
3인칭 카메라 · 로봇 앞 카메라 · 미니맵(기준선 · 장애물 · Nav2 경로 계획 · 지나온 길)으로 한 화면(1920×1080)에 담아 mp4 로 저장한다.
X-Mobility 시연 영상(서버 ~/xmobility_test/tools/record_demo_xm_parking.py)과 같은 화면 구성.

두 단계 — 주행하면서 바로 그리면 렌더 · 합성 때문에 시뮬이 실시간의 0.35 ~ 0.6배로 느려지고, Nav2 의 일부(행동 트리 재계획 ·
제어 루프 · 속도 다듬기)는 실제 시간으로 돌아 동작이 달라짐(10.14: 수집 때 넉넉히 피한 상자에 부딪히거나 스침). 그래서
  FD_DEMO_MODE=drive  — 영상 렌더 없이 수집 때와 같은 조건으로 주행하며, 0.1 s(시뮬)마다 로봇 자세(chassis_link 월드 자세)와
                        그때의 다리 /teacher/status · Nav2 /plan 번호를 기록(<출력>_logs/drive.jsonl · plans.jsonl · route.log)
  FD_DEMO_MODE=replay — 물리 없이(타임라인 정지) 기록한 자세에 로봇을 다시 놓으며 3인칭 · 앞 카메라를 렌더해 합성
씬은 이 스크립트가 직접 연다(헤드리스, 3D LiDAR 켬, 장애물 레이어를 세션 레이어에 얹음 — sim/proxy_play.py 와 같은 방식).
drive 의 Nav2 · 다리 · 주행 스크립트는 하위 프로세스(시스템 Jazzy, 도메인 43) — 교사 녹화와 같은 연결:
  nav2.launch.py cmd_vel_out:=nav2/cmd_vel → 다리 sim/nav2/nav2_teacher.py → /cmd_vel, 주행은 sim/nav2/nav2_route.py --scenario.
보통은 sim/nav2/record_demo.sh 가 장애물 레이어를 만들고 두 단계를 차례로 부른다:
  FD_DEMO_MODE=drive|replay FD_SCENARIO=eval/scenarios/<이름>.yaml FD_DIR=cw FD_LAYER=<레이어.usda> [FD_POSE="x0:=… y0:=… yaw0_deg:=…"] \\
      [FD_SPEED=3] envs/isaacsim_env/bin/python sim/nav2/record_demo.py <출력.mp4>
로그 표지: DEMO_ABORT(준비 실패 — Nav2 가 안 뜸 등, 감싸는 스크립트가 다시 함) · RESULT · DEMO_DRIVE_DONE · DEMO_DONE.
"""
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0, "multi_gpu": False})

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

HOME = os.environ["HOME"]
CD1 = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = sys.argv[1]
MODE = os.environ.get("FD_DEMO_MODE", "drive")
SCN = os.environ["FD_SCENARIO"]
DIR = os.environ.get("FD_DIR", "cw")
LAYER = os.environ["FD_LAYER"]
POSE = os.environ.get("FD_POSE", "")
SPEED = int(os.environ.get("FD_SPEED", "3"))
SCENE = os.environ.get("FD_SCENE", f"{CD1}/sim/scenes/parking/parking_v0.4b_r3.usda")
ODIR = os.path.splitext(OUT)[0] + "_logs"
os.makedirs(ODIR, exist_ok=True)
SC = yaml.safe_load(open(os.path.join(CD1, SCN), encoding="utf-8"))
SEGMENT = "s" in (SC.get("goal") or {})
RT = SC["route"] if SEGMENT else f"eval/routes/parking_r3_{DIR}.csv"   # Nav2 가 따르는 기준선(nav2_route.py 와 같음)
if SEGMENT:
    DIR = SC.get("direction", "cw")
ROBOT = "/World/Nova_Carter_ROS"
CHASE = f"{ROBOT}/chassis_link/ChaseCam"
FRONT = f"{ROBOT}/chassis_link/sensors/front_hawk/left/e2e_camera"
CHASSIS = f"{ROBOT}/chassis_link"
TOPCAM = "/Probe/topcam"
TCX, TCY, TCH, THALF = -17.5, -9.0, 100.0, 22.5   # 위에서 본 카메라: 중심 · 높이 · 가로 반폭(m) — r3 루프 둘레 45 × 32 m
TOPW, TOPH = 1280, 910
LOGSTEP, FPS, DT = 2 * SPEED, 30, 1 / 60          # 자세 기록 간격(60 Hz 스텝) — 3배속이면 0.1 s 한 장을 30 fps 로
ENV = ("source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp "
       "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0")
SUB_ENV = {"HOME": HOME, "PATH": "/usr/bin:/bin"}   # Isaac 파이썬 환경 변수를 하위 프로세스(시스템 파이썬)에 넘기지 않음
REG = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
F = {k: ImageFont.truetype(BOLD if k.startswith("b") else REG, s, index=1)
     for k, s in (("b30", 30), ("b22", 22), ("r22", 22), ("r20", 20), ("b26", 26), ("b40", 40))}
CMD_NAME = {0: "직진", 1: "좌회전", 2: "우회전"}

# /plan · /teacher/status 를 파일로 — Isaac 파이썬에서는 rclpy 를 안 쓰므로(xm 녹화기와 같은 이유) 시스템 파이썬 작은 노드가 씀.
# P 줄 = 받은 시뮬 시각 + 0.3 m 간격 점, S 줄 = 다리 상태 JSON(그 안의 t 가 시뮬 시각)
TAP_SRC = '''import sys, rclpy
from nav_msgs.msg import Path
from std_msgs.msg import String
from rclpy.parameter import Parameter
out = open(sys.argv[1], "w", buffering=1)
rclpy.init()
n = rclpy.create_node("fd_demo_tap", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
def on_plan(m):
    pts, last = [], None
    for p in m.poses:
        x, y = p.pose.position.x, p.pose.position.y
        if last is None or (x - last[0]) ** 2 + (y - last[1]) ** 2 > 0.09:
            pts.append(f"{x:.2f},{y:.2f}")
            last = (x, y)
    out.write(f"P {n.get_clock().now().nanoseconds * 1e-9:.3f} " + " ".join(pts) + "\\n")
n.create_subscription(Path, "/plan", on_plan, 5)
n.create_subscription(String, "/teacher/status", lambda m: out.write("S " + m.data + "\\n"), 50)
try:
    rclpy.spin(n)
except Exception:
    pass
'''


def route_pts(path):
    import csv
    rows = [r for r in csv.DictReader(open(os.path.join(CD1, path))) if r.get("run", "0") == "0"]
    return [(float(r["x"]), float(r["y"])) for r in rows]


ref = route_pts("eval/routes/parking_r3_cw.csv")
_rt = route_pts(RT)
L = sum(math.hypot(xb - xa, yb - ya) for (xa, ya), (xb, yb) in zip(_rt, _rt[1:]))
if SEGMENT:
    L = float(SC["goal"]["s"]) - float(SC["start"]["s"])
# 장애물 — 레이어에 make_obstacles.py 가 남긴 월드 좌표(시뮬과 같은 값)
_m = re.search(r"obstacles_json = '(.*?)'\n", open(os.path.join(CD1, LAYER), encoding="utf-8").read())
OBS = json.loads(_m.group(1)) if _m else []
MW = 640
MH = round(MW * TOPH / TOPW)


def mm(x, y):
    return (MW / 2 + (x - TCX) / THALF * MW / 2, MH / 2 - (y - TCY) / THALF * MW / 2)


def label(d, xy, text, font):
    x, y = xy
    l_, t_, r_, b_ = d.textbbox((x, y), text, font=font)
    d.rectangle((l_ - 8, t_ - 5, r_ + 8, b_ + 5), fill=(0, 0, 0))
    d.text((x, y), text, font=font, fill=(255, 255, 255))


class Tail:
    """하위 프로세스가 쓰는 파일에서 새 줄만 읽는다(끝나지 않은 줄은 다음 번에)."""

    def __init__(self, fn):
        self.fn, self.f, self.buf = fn, None, ""

    def lines(self):
        if self.f is None:
            if not os.path.exists(self.fn):
                return []
            self.f = open(self.fn, encoding="utf-8", errors="replace")
        self.buf += self.f.read()
        *done, self.buf = self.buf.split("\n")
        return done


def spawn(cmd, log):
    return subprocess.Popen(["bash", "-c", cmd], env=SUB_ENV, stdout=open(log, "w"), stderr=subprocess.STDOUT,
                            start_new_session=True)


def open_scene():
    t0 = time.time()
    omni.usd.get_context().open_stage(SCENE)
    while is_stage_loading() or time.time() - t0 < 5:
        app.update()
    stage = omni.usd.get_context().get_stage()
    stage.GetSessionLayer().subLayerPaths.append(os.path.join(CD1, LAYER))   # 장애물(구간이면 출발 자세도) — proxy_play.add_layers 와 같음
    for _ in range(10):
        app.update()
    print(f"DEMO_SCENE mode={MODE} {SCENE} layer={LAYER} obstacles={[o['name'] for o in OBS]} route={RT} dir={DIR} segment={SEGMENT}",
          flush=True)
    return stage


# ───────────────────────────── drive ─────────────────────────────
def drive():
    enable_extension("isaacsim.ros2.bridge")
    app.update()
    open_scene()
    from isaacsim.core.prims import XFormPrim

    xp = XFormPrim([CHASSIS])
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(120):  # 2 s — 센서 · odom · LiDAR 가 돌기 시작
        app.update()
    open(f"{ODIR}/tap.py", "w").write(TAP_SRC)
    route_arg = "" if SEGMENT else f"--route {RT}"
    procs = {
        "nav2": spawn(f"{ENV} && cd {CD1} && exec ros2 launch sim/nav2/nav2.launch.py map:={CD1}/data/nav2/parking_v0.4b_r3/map.yaml "
                      f"localization:=amcl cmd_vel_out:=nav2/cmd_vel {POSE}", f"{ODIR}/launch.log"),
        "bridge": spawn(f"{ENV} && cd {CD1} && exec /usr/bin/python3 -u sim/nav2/nav2_teacher.py --route {RT} --scenario {SCN} "
                        f"--summary {ODIR}/teacher.json", f"{ODIR}/teacher.log"),
        "tap": spawn(f"{ENV} && exec /usr/bin/python3 -u {ODIR}/tap.py {ODIR}/tap.txt", f"{ODIR}/tap.log"),
        "route": None,
    }
    open(f"{ODIR}/pgids.txt", "w").write(" ".join(str(p.pid) for p in procs.values() if p))
    rt_tail, tap_tail = Tail(f"{ODIR}/route.log"), Tail(f"{ODIR}/tap.txt")
    dj, pj = open(f"{ODIR}/drive.jsonl", "w", buffering=1), open(f"{ODIR}/plans.jsonl", "w", buffering=1)
    step, t_go, t_end, abort, wall0, st, n_plan = 0, None, None, None, time.time(), None, 0
    while True:
        app.update()
        step += 1
        t = step * DT
        if procs["route"] is None and t > 4.0:   # Nav2 활성(90 s) · 지도 · 센서 예열 · AMCL 은 nav2_route 가 확인
            procs["route"] = spawn(f"{ENV} && cd {CD1} && exec timeout -k 20 900 /usr/bin/python3 -u sim/nav2/nav2_route.py {route_arg} "
                                   f"--scenario {SCN} --timeout 300 --out {ODIR}/route.json", f"{ODIR}/route.log")
            open(f"{ODIR}/pgids.txt", "a").write(f" {procs['route'].pid}")
        if step % LOGSTEP:
            continue
        for ln in rt_tail.lines():
            if ln.startswith("NAV2_ROUTE_START"):
                t_go = t
                print(f"ROUTE_START t={t:.1f}s {ln[:160]}", flush=True)
            elif ln.startswith("NAV2_ROUTE_DONE"):
                print(f"RESULT {ln}", flush=True)
            elif ln.startswith("NAV2_ROUTE_NO_"):
                abort = ln
                print(f"DEMO_ABORT {ln}", flush=True)
            elif ln.startswith(("NAV2_ROUTE_AMCL", "NAV2_ROUTE_OBSTACLES", "NAV2_ROUTE_WARMUP")):
                print(ln[:200], flush=True)
        for ln in tap_tail.lines():   # 다리 상태 · Nav2 경로 계획 — 기록 줄에 그때 값을 같이 넣음(시각 맞추기 불필요)
            if ln.startswith("S "):
                try:
                    st = json.loads(ln[2:])
                except ValueError:
                    pass
            elif ln.startswith("P ") and len(ln.split()) > 2:
                pj.write(json.dumps([[float(v) for v in p.split(",")] for p in ln.split()[2:] if "," in p]) + "\n")
                n_plan += 1
        if abort or (procs["route"] is not None and procs["route"].poll() not in (None, 0) and t_go is None):
            if not abort:
                print(f"DEMO_ABORT nav2_route rc={procs['route'].returncode} 출발 전 끝남", flush=True)
                abort = "rc"
            break
        if t_go is None:
            if t > 300:
                print("DEMO_ABORT 출발 전 300 s 넘음", flush=True)
                abort = "start_timeout"
                break
            continue
        if step % LOGSTEP == 0:   # 0.1 s(시뮬)마다 — 시뮬 시각은 스텝 수(60 Hz 한 스텝 = update 한 번)로. 타임라인 시각은 이
            pos, quat = xp.get_world_poses(usd=False)   # 실행 방식에선 안 움직임(10.14 첫 시도에 한 줄만 남음)
            dj.write(json.dumps({"t": round(t, 3), "p": [round(float(v), 4) for v in pos[0]],
                                 "q": [round(float(v), 6) for v in quat[0]], "st": st, "plan": n_plan}) + "\n")
        if t_end is None and procs["route"].poll() is not None:
            t_end = t
            print(f"DRIVE_DONE rc={procs['route'].returncode} t={t:.1f}s", flush=True)
        if t_end is not None and t - t_end > 2.0:
            break
        if t - t_go > 420:
            print("TIME_CAP", flush=True)
            break
        if step % 900 == 0:
            print(f"PROG t={t:.1f}s go={t - t_go:.1f}s wall={time.time() - wall0:.0f}s rtf={t / max(time.time() - wall0, 1e-6):.2f}", flush=True)
    dj.close()
    pj.close()
    for name in ("route", "tap", "bridge", "nav2"):   # Ctrl+C → TERM → KILL(lifecycle_manager 가 신호를 무시하는 일, 10.14)
        p = procs.get(name)
        if p is None or p.poll() is not None:
            continue
        for sig, wait in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
            try:
                os.killpg(p.pid, sig)
                p.wait(timeout=wait)
                break
            except (subprocess.TimeoutExpired, ProcessLookupError):
                continue
    timeline.stop()
    print(f"DEMO_DRIVE_DONE sim_s={step * DT:.1f} wall_s={time.time() - wall0:.0f} abort={abort}", flush=True)


# ───────────────────────────── replay ─────────────────────────────
def qmat(q):   # (w, x, y, z) → 3×3
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mquat(m):  # 3×3 → (w, x, y, z)
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    return np.array([w, x, y, z])


def replay():
    stage = open_scene()
    rows = [json.loads(ln) for ln in open(f"{ODIR}/drive.jsonl") if ln.strip()]
    plans = [json.loads(ln) for ln in open(f"{ODIR}/plans.jsonl") if ln.strip()]
    res = next((ln for ln in open(f"{ODIR}/route.log", encoding="utf-8") if ln.startswith("NAV2_ROUTE_DONE")), None)
    kv = dict(x.split("=", 1) for x in res.split() if "=" in x) if res else {}
    print(f"REPLAY rows={len(rows)} plans={len(plans)} result={'yes' if res else 'no'}", flush=True)
    # 미니맵 — 위에서 본 카메라 한 장(장애물 포함)
    from pxr import Gf, Usd, UsdGeom

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        c_ = UsdGeom.Camera.Define(stage, TOPCAM)
        c_.CreateHorizontalApertureAttr(36.0)
        c_.CreateVerticalApertureAttr(36.0 * TOPH / TOPW)
        c_.CreateFocalLengthAttr(18.0 * TCH / THALF)
        c_.CreateClippingRangeAttr(Gf.Vec2f(1.0, 1000.0))
        UsdGeom.Xformable(c_).AddTranslateOp().Set(Gf.Vec3d(TCX, TCY, TCH))
    ta = rep.AnnotatorRegistry.get_annotator("rgb")
    trp = rep.create.render_product(TOPCAM, (TOPW, TOPH))
    ta.attach([trp])
    rep.orchestrator.step(rt_subframes=16)
    top = np.asarray(ta.get_data())
    for f_ in (ta.detach, trp.destroy):
        try:
            f_()
        except Exception as e:
            print(f"MINIMAP_CLEANUP_WARN {e}", flush=True)
    base = Image.fromarray(top[..., :3]).resize((MW, MH), Image.LANCZOS)
    d0 = ImageDraw.Draw(base)
    d0.line([mm(*p) for p in ref + ref[:1]], fill=(255, 210, 63), width=3)
    pxm = MW / 2 / THALF
    for o in OBS:   # 장애물 — 실제 크기보다 조금 키워(최소 반폭 5 px) 빨강으로
        cx, cy = mm(o["x"], o["y"])
        if o.get("type", "box") == "box":
            hx, hy, yw = max(o["size"][0] / 2 * pxm, 5), max(o["size"][1] / 2 * pxm, 5), math.radians(o.get("yaw_deg", 0.0))
            c2, s2 = math.cos(yw), math.sin(yw)
            d0.polygon([(cx + u * c2 - v * s2, cy - (u * s2 + v * c2)) for u, v in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))],
                       fill=(235, 64, 52), outline=(20, 20, 20))
        else:
            r_ = max(o["size"][0] / 2 * pxm, 5)
            d0.ellipse((cx - r_, cy - r_, cx + r_, cy + r_), fill=(235, 64, 52), outline=(20, 20, 20))
    ann = {}
    for name, prim_path, res_ in (("chase", CHASE, (1280, 720)), ("front", FRONT, (640, 480))):
        a = rep.AnnotatorRegistry.get_annotator("rgb")
        a.attach([rep.create.render_product(prim_path, res_)])
        ann[name] = a
    # 로봇 뿌리(ROBOT)를 옮겨 chassis_link 가 기록한 월드 자세에 오게 — 뿌리 → chassis 상대 변환은 처음(물리 없음) 자세에서 한 번 잼
    from isaacsim.core.prims import XFormPrim

    root, chs = XFormPrim([ROBOT]), XFormPrim([CHASSIS])
    rp0, rq0 = (np.asarray(v)[0] for v in root.get_world_poses())
    cp0, cq0 = (np.asarray(v)[0] for v in chs.get_world_poses())
    rel_r = qmat(rq0).T @ qmat(cq0)
    rel_p = qmat(rq0).T @ (cp0 - rp0)
    dname = "시계" if DIR == "cw" else "반시계"
    what = f"구간 {SC['name'].replace('parking_r3_', '')}" if SEGMENT else "기준선 한 바퀴"
    title = f"실제 주차장(맵 v0.4b r3) · Nav2 장애물 회피 — {what} ({dname} 방향, {SPEED}배속)"
    notes = ["※ Nav2(ROS 2 표준 자율주행) — 3D LiDAR + 지도로 위치 추정(AMCL). 장애물은 지도에 없고, 달리면서 LiDAR로 보고 피함",
             f"기준선(화단 3 m 선)에 4 m 간격 경유점 · 경로는 Nav2가 계획 · 상한 v 0.8 m/s · ω 0.7 rad/s · 장애물 {len(OBS)}개",
             f"앞 카메라 영상 + Nav2 명령을 E2E 교사 데이터로 녹화(10.14 Nav2 교사 1차 21개) · C 맵 v0.4b r3 · {time.strftime('%Y.%m.%d')}"]
    legend = "노랑 기준선 · 하늘색 Nav2 계획 경로 · 초록 지나온 길 · 빨강 장애물"
    for n_ in notes + [legend]:
        wpx = F["r20"].getlength(n_)
        print(f"TEXT_WIDTH {wpx:.0f}px{' WARN' if wpx > (1240 if n_ != legend else 610) else ''}", flush=True)
    passed = kv.get("pass") == "True"
    final = (f"{'완주' if passed else '판정 실패'} — 랩 {kv.get('lap_sim_s', '?')} s(시뮬) · 기준선 최대 이탈 {kv.get('max_cte', '?')} m · "
             f"장애물과 최소 {kv.get('min_clearance', '-')} m · 충돌 {'없음' if kv.get('collision') in (None, 'False') else '있음'}") if kv else None
    writer = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (1920, 1080))
    t0, trail, frames, canvas, wall0 = rows[0]["t"], [], 0, None, time.time()
    for k, row in enumerate(rows):
        t = row["t"]
        R_c, p_c = qmat(row["q"]), np.array(row["p"])
        R_r = R_c @ rel_r.T
        p_r = p_c - R_r @ rel_p
        root.set_world_poses(positions=p_r[None, :], orientations=mquat(R_r)[None, :])
        rep.orchestrator.step(rt_subframes=2)   # 타임라인이 멈춰 있어 app.update 로는 주석기(annotator)가 안 채워짐 — 렌더 한 단계
        chase, front = np.asarray(ann["chase"].get_data()), np.asarray(ann["front"].get_data())
        if chase.size == 0 or front.size == 0:
            continue
        st = row.get("st")
        plan = plans[row["plan"] - 1] if row.get("plan", 0) > 0 else []
        x, y = float(p_c[0]), float(p_c[1])
        yaw = math.atan2(R_c[1, 0], R_c[0, 0])
        trail.append((x, y))
        last = k == len(rows) - 1
        if last and final:
            status = final
        elif st and st.get("ph") == "FOLLOW":
            od = st.get("od")
            status = (f"Nav2 주행 중 — 경로 지시 {CMD_NAME.get(st.get('cmd', 0), '직진')}"
                      + (f" · 장애물 {od:.1f} m 앞 — 비켜 가는 중" if od is not None and od < 3.0 else ""))
        elif st and st.get("ph") == "STOP":
            status = final or "도착 — 정지"
        else:
            status = "Nav2 주행 시작"
        canvas = Image.new("RGB", (1920, 1080), (24, 25, 28))
        canvas.paste(Image.fromarray(chase[..., :3]), (0, 60))
        canvas.paste(Image.fromarray(front[..., :3]), (1280, 60))
        mini = base.copy()
        dm = ImageDraw.Draw(mini)
        if len(plan) > 1:
            dm.line([mm(*p) for p in plan], fill=(120, 220, 255), width=3)
        if len(trail) > 1:
            dm.line([mm(*p) for p in trail], fill=(56, 224, 123), width=4)
        px, py = mm(x, y)
        dm.ellipse((px - 8, py - 8, px + 8, py + 8), fill=(56, 224, 123), outline=(0, 0, 0), width=2)
        dm.line((px, py, px + 22 * math.cos(yaw), py - 22 * math.sin(yaw)), fill=(0, 0, 0), width=4)
        canvas.paste(mini, (1280, 1080 - MH))
        d = ImageDraw.Draw(canvas)
        d.text((24, 12), title, font=F["b26"], fill=(255, 255, 255))
        label(d, (16, 74), "3인칭 카메라 — 보기용", F["b22"])
        label(d, (1296, 74), "로봇 앞 카메라 640×480 — E2E 모델 입력 · 교사 데이터로 녹화", F["b22"])
        d.text((1296, 553), legend, font=F["r20"], fill=(220, 220, 220))
        prog = max(st.get("prog", 0.0), 0.0) if st else 0.0
        d.text((24, 792), f"주행 시간 {t - t0:5.1f} s     진행 {min(prog, L):5.1f} m / {L:.0f} m     속도 명령 {(st or {}).get('vl', 0.0):4.2f} m/s",
               font=F["b30"], fill=(255, 210, 63))
        if st:
            od = st.get("od")
            d.text((24, 838), f"Nav2 명령   v {st.get('vl', 0):+.2f} m/s   ω {st.get('wl', 0):+.2f} rad/s     기준선 이탈 {abs(st.get('cte', 0)):.2f} m"
                   + (f"     장애물까지 {max(od, 0):.1f} m" if od is not None else ""), font=F["b26"], fill=(120, 220, 255))
        d.text((24, 880), status, font=F["b22"], fill=(255, 255, 255))
        for i, n_ in enumerate(notes):
            d.text((24, 924 + i * 38), n_, font=F["r20"], fill=(220, 220, 220))
        writer.write(cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR))
        frames += 1
        if frames == 45 or frames % 300 == 0:
            canvas.save(f"{ODIR}/frame_{frames:05d}.jpg", quality=85)
        if frames % 300 == 0:
            print(f"REPLAY_PROG {frames}/{len(rows)} wall={time.time() - wall0:.0f}s", flush=True)
    if canvas is not None and final:   # 끝 화면 3 s — 결과 한 줄을 크게
        end = canvas.copy()
        de = ImageDraw.Draw(end)
        de.rectangle((0, 330, 1280, 520), fill=(0, 0, 0))
        de.text((40, 350), final.split(" — ")[0] + ("" if SEGMENT else f" — 장애물 {len(OBS)}개를 피해 한 바퀴"), font=F["b40"],
                fill=(255, 210, 63))
        de.text((40, 420), final.split(" — ", 1)[-1], font=F["b30"], fill=(255, 255, 255))
        for _ in range(3 * FPS):
            writer.write(cv2.cvtColor(np.asarray(end), cv2.COLOR_RGB2BGR))
            frames += 1
        end.save(f"{ODIR}/end.jpg", quality=90)
    writer.release()
    print(f"DEMO_DONE frames={frames} video_s={frames / FPS:.1f} wall_s={time.time() - wall0:.0f}", flush=True)


if MODE == "drive":
    drive()
else:
    replay()
app.close()
