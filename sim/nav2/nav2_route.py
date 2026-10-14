"""Nav2 로 기준선을 한 바퀴 돈다 — 기준선 CSV 에서 --spacing m 간격 경유점을 뽑아 NavigateThroughPoses 로 보낸다.

Nav2(sim/nav2/nav2.launch.py)가 떠 있어야 한다. 경유점 사이는 Nav2 가 스스로 경로를 짜므로 기준선을 그대로 따라가지는 않는다
(장애물을 피하는 교사 · 비교군의 동작). 이탈은 정답 위치 /ground_truth/tf(World → nova_carter)로 잰다 — 채점용일 뿐 Nav2 는 안 씀.

    python3 sim/nav2/nav2_route.py --route eval/routes/parking_r3_cw.csv [--spacing 4] [--out logs/nav2/<이름>.json] [--initial-pose]
끝에 NAV2_ROUTE_DONE result=... lap=...s mean_cte=... max_cte=... 한 줄.
--scenario eval/scenarios/<이름>.yaml 이면 장애물(sim/make_obstacles.py 와 같은 자리)을 알고 — 장애물 가까운 경유점을 빼고(교사는 장애물
자리를 알아도 됨 — 정답 쪽), 로봇 발자국(nav2_params.yaml 의 footprint)과 상자 사이 최소 거리 · 충돌 · 가장 긴 정지를 재서
판정(수행계획서 2.4: 완주 · 이탈 2.0 m 안 · 충돌 없음 · 10 s 넘는 정지 없음)을 붙인다.
시나리오의 goal 에 s 가 있으면 구간 시나리오 — 시나리오의 route 를 start.s 에서 goal.s 까지만 돌고(씬은 make_obstacles.py 레이어로
그 출발 자세에서 띄움, AMCL 초기 자세는 nav2.launch.py 의 x0 · y0 · yaw0_deg 로), 판정에 목표 반경(goal.radius_m, 기본 1.0 m) 도착을 더한다.
출발 전에 AMCL 위치가 정답 위치 0.5 m 안인지 확인하고, 아니면 /initialpose 를 다시 보낸다(그래도 안 맞으면 코드 3).
판정에는 실제로 달린 거리가 경로 길이의 80 % 이상인지도 넣는다 — 안 움직이고 SUCCEEDED 가 나는 경우를 거름(10.14).
"""
import argparse
import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

ap = argparse.ArgumentParser()
ap.add_argument("--route", default="eval/routes/parking_r3_cw.csv")
ap.add_argument("--spacing", type=float, default=4.0)
ap.add_argument("--out")
ap.add_argument("--initial-pose", action="store_true", help="AMCL 초기 자세를 기준선 출발점으로 다시 보냄(씬을 다시 띄우지 않고 이어 돌 때). 평소엔 nav2.launch.py 의 초기 자세(x0 · y0 · yaw0_deg)로 충분")
ap.add_argument("--timeout", type=float, default=600.0, help="시뮬 초")
ap.add_argument("--scenario", help="장애물 시나리오 yaml(eval/scenarios/)")
ap.add_argument("--clear", type=float, default=1.5, help="장애물 가장자리에서 이 거리 안의 경유점은 뺌(m)")
a = ap.parse_args()

SC = {}
if a.scenario:
    import yaml
    SC = yaml.safe_load(open(a.scenario))
SEGMENT = "s" in (SC.get("goal") or {})          # 구간 시나리오 — 시나리오의 route 를 그대로 따름
ROUTE = SC["route"] if SEGMENT else a.route
R = np.array([(float(r["x"]), float(r["y"])) for r in csv.DictReader(open(ROUTE)) if r.get("run", "0") == "0"])
seg = np.diff(R, axis=0)
s = np.concatenate([[0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
L = s[-1]


def heading(i):
    j = min(i + 1, len(R) - 1)
    k = j - 1 if j == i else i
    return math.atan2(R[j, 1] - R[k, 1], R[j, 0] - R[k, 0])


def cte(x, y):
    """기준선(닫힌 꺾은선)까지 거리."""
    a0, d = R[:-1], seg
    t = np.clip(((x - a0[:, 0]) * d[:, 0] + (y - a0[:, 1]) * d[:, 1]) / np.maximum((d ** 2).sum(1), 1e-12), 0, 1)
    px, py = a0[:, 0] + t * d[:, 0], a0[:, 1] + t * d[:, 1]
    return float(np.min(np.hypot(px - x, py - y)))


FOOT = [(0.14, 0.25), (0.14, -0.25), (-0.607, -0.25), (-0.607, 0.25)]   # base_link 기준 로봇 발자국(nav2_params.yaml)
OBS = []
if a.scenario:
    RS = np.array([(float(r["s"]), float(r["x"]), float(r["y"])) for r in csv.DictReader(open(SC["route"])) if r.get("run", "0") == "0"])
    for o in SC.get("obstacles", []):   # sim/make_obstacles.py 의 on_route 와 같은 계산
        i = int(np.clip(np.searchsorted(RS[:, 0], float(o["s"])), 1, len(RS) - 1))
        (s0, x0, y0), (s1, x1, y1) = RS[i - 1], RS[i]
        f = (float(o["s"]) - s0) / max(s1 - s0, 1e-9); th = math.atan2(y1 - y0, x1 - x0); lat = float(o.get("lateral", 0.0))
        ox, oy = x0 + f * (x1 - x0) - lat * math.sin(th), y0 + f * (y1 - y0) + lat * math.cos(th)
        yaw = th + math.radians(float(o.get("yaw_deg", 0.0)))
        if o.get("type", "box") == "box":
            hx, hy = float(o["size"][0]) / 2, float(o["size"][1]) / 2
            corners = [(hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy)]
        else:   # cylinder · cone — 밑면 원을 16각형으로
            hx = hy = float(o["size"][0]) / 2
            corners = [(hx * math.cos(k * math.pi / 8), hx * math.sin(k * math.pi / 8)) for k in range(16)]
        OBS.append({"name": o["name"], "x": ox, "y": oy, "r": math.hypot(hx, hy),
                    "poly": [(ox + px * math.cos(yaw) - py * math.sin(yaw), oy + px * math.sin(yaw) + py * math.cos(yaw))
                             for px, py in corners]})


def _seg_dist(p, a_, b_):
    ax, ay = a_; bx, by = b_; px, py = p
    dx, dy = bx - ax, by - ay; t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / max(dx * dx + dy * dy, 1e-12)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _inside(p, poly):
    sgn = [((b_[0] - a_[0]) * (p[1] - a_[1]) - (b_[1] - a_[1]) * (p[0] - a_[0])) for a_, b_ in zip(poly, poly[1:] + poly[:1])]
    return all(v >= 0 for v in sgn) or all(v <= 0 for v in sgn)


def clearance(x, y, th):
    """로봇 발자국과 가장 가까운 장애물 사이 거리(m, 겹치면 0) — 볼록 다각형끼리."""
    rob = [(x + px * math.cos(th) - py * math.sin(th), y + px * math.sin(th) + py * math.cos(th)) for px, py in FOOT]
    best = float("inf")
    for o in OBS:
        if any(_inside(p, o["poly"]) for p in rob) or any(_inside(p, rob) for p in o["poly"]):
            return 0.0
        for P1, P2 in ((rob, o["poly"]), (o["poly"], rob)):
            for p in P1:
                for a_, b_ in zip(P2, P2[1:] + P2[:1]):
                    best = min(best, _seg_dist(p, a_, b_))
    return best


rclpy.init()
nav = BasicNavigator()
nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])   # Nav2 · AMCL 과 같은 시뮬 시계


def pose(x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    # stamp 0 = TF 최신값 — 시뮬 시계와 엇갈리지 않게
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


track = []   # (시뮬 초, x, y, cte)


def on_gt(m):
    for t in m.transforms:
        if t.header.frame_id == "World" and t.child_frame_id == "nova_carter":
            st = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            if not track or st - track[-1][0] >= 0.1:
                x, y = t.transform.translation.x, t.transform.translation.y
                q = t.transform.rotation; th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                track.append((st, x, y, cte(x, y), clearance(x, y, th) if OBS else -1.0))


nav.create_subscription(TFMessage, "/ground_truth/tf", on_gt, QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))

i_start = int(np.searchsorted(s, float(SC["start"]["s"]))) if SEGMENT else 0
# 출발 자세(AMCL 초기 자세로 다시 보낼 값) — 구간이면 기준선 위 start.s, 바퀴 시나리오면 start 의 x · y · yaw_deg
if SEGMENT:
    P0 = (R[i_start, 0], R[i_start, 1], heading(i_start))
elif "x" in (SC.get("start") or {}):
    P0 = (float(SC["start"]["x"]), float(SC["start"]["y"]), math.radians(float(SC["start"]["yaw_deg"])))
else:
    P0 = (R[0, 0], R[0, 1], heading(0))
if a.initial_pose:
    nav.setInitialPose(pose(*P0))


def wait_nav2_active(limit=90.0):
    """bt_navigator 가 active 인지 — BasicNavigator.waitUntilNav2Active 는 시간 제한이 없어, 상태 응답이 통신 지연으로 빠지면 끝없이
    기다림(10.14 라바콘 반시계 15분). 응답 하나를 3 s 까지만 기다리며 되묻고, limit 안에 못 받으면 False. AMCL 활성은 lifecycle_manager
    가 이미 확인함(BasicNavigator 의 amcl/get_state 대기도 가끔 멈춰서 안 씀, 10.13)."""
    cli = nav.create_client(GetState, "bt_navigator/get_state")
    t0 = time.time()
    while time.time() - t0 < limit:
        if cli.wait_for_service(timeout_sec=1.0):
            fut = cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav, fut, timeout_sec=3.0)
            if fut.done() and fut.result() is not None and fut.result().current_state.label == "active":
                return True
        rclpy.spin_once(nav, timeout_sec=0.5)
    return False


if not wait_nav2_active():
    print("NAV2_ROUTE_NO_NAV2 bt_navigator 상태를 90 s 안에 못 받음", flush=True)
    raise SystemExit(3)
# 전역 비용 지도가 지도(/map_nav)를 받았는지 — 가끔 map_server_nav 활성화 응답이 늦어 지도 없이 뜸(10.14 east_start 1회).
# 그러면 '로봇이 비용 지도 밖' 으로 계획이 전부 실패하므로, 출발 전에 확인하고 없으면 코드 3 으로 끝냄(일괄 스크립트가 다시 띄움)
_gm = []
nav.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: _gm.append(m.info.width),
                        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
_t = time.time()
while time.time() - _t < 20 and not (_gm and _gm[-1] >= 500):
    rclpy.spin_once(nav, timeout_sec=0.2)
if not (_gm and _gm[-1] >= 500):
    print(f"NAV2_ROUTE_NO_MAP 전역 비용 지도 폭 {_gm[-1] if _gm else '없음'} — 지도를 못 받음", flush=True)
    raise SystemExit(3)

if SEGMENT:
    s0_, s1_ = float(SC["start"]["s"]), float(SC["goal"]["s"])
    i_goal = int(min(np.searchsorted(s, s1_), len(R) - 1))
    idx = [int(np.searchsorted(s, v)) for v in np.arange(s0_ + a.spacing, s1_ - 1.0, a.spacing)] + [i_goal]
else:   # 마지막 경유점은 결승 1 m 앞 — 출발점 바로 뒤(0.24 m)에 두면 안 움직이고 목표 도착으로 끝날 때가 있음(10.14 반시계 3회, 6 s)
    i_goal = int(np.searchsorted(s, L - 1.0))
    idx = [int(np.searchsorted(s, v)) for v in np.arange(a.spacing, L - 1.0, a.spacing)] + [i_goal]
near = [i for i in idx if any(math.hypot(R[i, 0] - o["x"], R[i, 1] - o["y"]) < o["r"] + a.clear for o in OBS)]
idx = [i for i in idx if i not in near]
goals = [pose(R[i, 0], R[i, 1], heading(i)) for i in idx]
if OBS:
    print(f"NAV2_ROUTE_OBSTACLES {[(o['name'], round(o['x'], 2), round(o['y'], 2)) for o in OBS]} · 뺀 경유점 {len(near)}개", flush=True)
print(f"NAV2_ROUTE_START route={ROUTE} " + (f"구간 s {s0_:.1f} → {s1_:.1f} m" if SEGMENT else f"L={L:.1f} m") +
      f" 경유점 {len(goals)}개(간격 {a.spacing} m)", flush=True)
_t = time.time()
while not track and time.time() - _t < 15:   # 새로 띄운 노드는 정답 위치를 받기까지 시간이 걸림 — 받은 뒤에 출발(10.14: 안 기다리면 t_goal=0)
    rclpy.spin_once(nav, timeout_sec=0.2)
if not track:
    print("NAV2_ROUTE_NO_GT /ground_truth/tf 를 못 받음", flush=True)
    raise SystemExit(3)
# AMCL 이 출발 자리를 아는지 — 구간 시나리오에서 /initialpose 가 새로 뜬 AMCL 에 안 닿아, AMCL 이 r3 시작 자세에 머문 채 달린 일이
# 있음(10.14, 이탈 5~13 m). 초기 자세는 nav2.launch.py(x0 · y0 · yaw0_deg)가 넘기고 여기서는 확인만 — 정답 위치와 0.5 m 넘게 다르면
# 구독자가 붙은 뒤 /initialpose(P0)를 1 s 마다 다시 보냄. 정답 위치는 비교에만 씀
_ap = []
nav.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: _ap.append((m.pose.pose.position.x, m.pose.pose.position.y)),
                        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
_ip = nav.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
amcl_err = lambda: math.hypot(_ap[-1][0] - track[-1][1], _ap[-1][1] - track[-1][2]) if _ap else float("inf")   # noqa: E731
_t, _last, n_init = time.time(), time.time(), 0
while amcl_err() > 0.5 and time.time() - _t < 30:
    if time.time() - _last > (1.0 if n_init else 5.0) and _ip.get_subscription_count() > 0:   # 처음 5 s 는 launch 초기 자세를 기다림
        m = PoseWithCovarianceStamped()
        m.header.frame_id, m.header.stamp = "map", nav.get_clock().now().to_msg()
        m.pose.pose = pose(*P0).pose
        _ip.publish(m)
        _last, n_init = time.time(), n_init + 1
    rclpy.spin_once(nav, timeout_sec=0.2)
print(f"NAV2_ROUTE_AMCL 출발 위치 오차 {amcl_err():.3f} m · /initialpose 다시 보냄 {n_init}회", flush=True)
if amcl_err() > 0.5:
    print("NAV2_ROUTE_NO_AMCL AMCL 위치가 출발 자리와 안 맞음", flush=True)
    raise SystemExit(3)
amcl_start_err = round(amcl_err(), 3)
if not nav.nav_through_poses_client.wait_for_server(timeout_sec=30.0):   # goThroughPoses 의 대기도 끝이 없음
    print("NAV2_ROUTE_NO_NAV2 NavigateThroughPoses 액션 서버를 30 s 안에 못 찾음", flush=True)
    raise SystemExit(3)
t_goal = track[-1][0]   # 목표를 보낸 시뮬 시각 — 랩 · 정지 · 제한 시간은 여기부터
nav.goThroughPoses(goals)
t0w, last = time.time(), 0.0
while not nav.isTaskComplete():
    fb = nav.getFeedback()
    if time.time() - last > 5 and fb:
        last = time.time()
        c = track[-1][3] if track else float("nan")
        print(f"  남은 거리 {fb.distance_remaining:.1f} m · 경유점 남음 {fb.number_of_poses_remaining} · 복구 {fb.number_of_recoveries} "
              f"· 이탈 {c:.2f} m", flush=True)
    if track and track[-1][0] - t_goal > a.timeout:
        nav.cancelTask()
        break
res = nav.getResult()
name = {TaskResult.SUCCEEDED: "SUCCEEDED", TaskResult.CANCELED: "CANCELED", TaskResult.FAILED: "FAILED"}.get(res, str(res))
c = np.array([t[3] for t in track]) if track else np.array([np.nan])
# 가장 긴 정지(0.05 m/s 미만) — 0.5 s 간격 속도로
stall, run, prev = 0.0, 0.0, None
for t in [t for t in track if t[0] >= t_goal][::5]:
    if prev is not None:
        v = math.hypot(t[1] - prev[1], t[2] - prev[2]) / max(t[0] - prev[0], 1e-6)
        run = run + (t[0] - prev[0]) if v < 0.05 else 0.0
        stall = max(stall, run)
    prev = t
clr = [t[4] for t in track if t[4] >= 0]
summ = {"route": ROUTE, "result": name, "spacing_m": a.spacing, "waypoints": len(goals),
        "lap_sim_s": round(track[-1][0] - t_goal, 1) if track else None, "wall_s": round(time.time() - t0w, 1),
        "mean_cte": round(float(np.nanmean(c)), 3), "max_cte": round(float(np.nanmax(c)), 3),
        "over_2m_frac": round(float(np.mean(c > 2.0)), 4), "recoveries": getattr(nav.getFeedback(), "number_of_recoveries", None),
        "max_stall_s": round(stall, 1)}
if OBS:
    summ.update({"scenario": a.scenario, "min_clearance": round(min(clr), 3) if clr else None, "collision": bool(clr and min(clr) <= 0.0)})
# 마지막 경유점(바퀴면 출발점 바로 앞, 구간이면 goal.s)까지 실제 거리 — Nav2 가 SUCCEEDED 라도 여기서 다시 확인(10.14: 안 움직이고
# SUCCEEDED 를 돌려준 일이 한 번 있었음)
gx, gy = R[i_goal]
goal_err = math.hypot(track[-1][1] - gx, track[-1][2] - gy) if track else float("inf")
# 실제로 달린 거리(출발 뒤 정답 위치 누적) — 경로 길이(바퀴 L, 구간 goal.s − start.s)의 80 % 는 넘어야 함
tr = [t for t in track if t[0] >= t_goal]
dist = float(sum(math.hypot(q[1] - p[1], q[2] - p[2]) for p, q in zip(tr, tr[1:])))
plan_len = (s1_ - s0_) if SEGMENT else L
summ.update({"goal_err": round(goal_err, 3), "dist_m": round(dist, 1), "plan_m": round(plan_len, 1), "amcl_start_err": amcl_start_err,
             "initialpose_resent": n_init})
if SEGMENT:
    summ.update({"scenario": a.scenario, "segment": [s0_, s1_]})
lim_dev, lim_stall = float(SC.get("deviation_limit_m", 2.0)), float(SC.get("stall_limit_s", 10))
summ["pass"] = bool(name == "SUCCEEDED" and summ["max_cte"] <= lim_dev and not summ.get("collision", False) and stall < lim_stall
                    and goal_err <= float((SC.get("goal") or {}).get("radius_m", 1.0)) and dist >= 0.8 * plan_len)
print("NAV2_ROUTE_DONE " + " ".join(f"{k}={v}" for k, v in summ.items()), flush=True)
if a.out:
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump({**summ, "track": [list(map(lambda v: round(v, 3), t)) for t in track]}, open(a.out, "w"))
nav.destroy_node()
rclpy.shutdown()
