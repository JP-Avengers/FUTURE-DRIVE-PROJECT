"""Nav2 로 기준선을 한 바퀴 돈다 — 기준선 CSV 에서 --spacing m 간격 경유점을 뽑아 NavigateThroughPoses 로 보낸다.

Nav2(sim/nav2/nav2.launch.py)가 떠 있어야 한다. 경유점 사이는 Nav2 가 스스로 경로를 짜므로 기준선을 그대로 따라가지는 않는다
(장애물을 피하는 교사 · 비교군의 동작). 이탈은 정답 위치 /ground_truth/tf(World → nova_carter)로 잰다 — 채점용일 뿐 Nav2 는 안 씀.

    python3 sim/nav2/nav2_route.py --route eval/routes/parking_r3_cw.csv [--spacing 4] [--out logs/nav2/<이름>.json] [--initial-pose]
끝에 NAV2_ROUTE_DONE result=... lap=...s mean_cte=... max_cte=... 한 줄.
"""
import argparse
import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

ap = argparse.ArgumentParser()
ap.add_argument("--route", default="eval/routes/parking_r3_cw.csv")
ap.add_argument("--spacing", type=float, default=4.0)
ap.add_argument("--out")
ap.add_argument("--initial-pose", action="store_true", help="AMCL 초기 자세를 기준선 출발점으로 다시 보냄(씬을 다시 띄우지 않고 이어 돌 때). 평소엔 nav2_params.yaml 의 초기 자세로 충분")
ap.add_argument("--timeout", type=float, default=600.0, help="시뮬 초")
a = ap.parse_args()

R = np.array([(float(r["x"]), float(r["y"])) for r in csv.DictReader(open(a.route)) if r.get("run", "0") == "0"])
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
                track.append((st, x, y, cte(x, y)))


nav.create_subscription(TFMessage, "/ground_truth/tf", on_gt, QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))

if a.initial_pose:
    nav.setInitialPose(pose(R[0, 0], R[0, 1], heading(0)))
# AMCL 활성 확인은 lifecycle_manager 가 이미 함 — BasicNavigator 의 amcl/get_state 대기가 가끔 서비스를 못 찾아 멈춰서(10.13) 건너뜀
nav.waitUntilNav2Active(localizer="robot_localization")

idx = [int(np.searchsorted(s, v)) for v in np.arange(a.spacing, L - 1.0, a.spacing)] + [len(R) - 2]
goals = [pose(R[i, 0], R[i, 1], heading(i)) for i in idx]
print(f"NAV2_ROUTE_START route={a.route} L={L:.1f} m 경유점 {len(goals)}개(간격 {a.spacing} m)", flush=True)
nav.goThroughPoses(goals)
t0w, last = time.time(), 0.0
while not nav.isTaskComplete():
    fb = nav.getFeedback()
    if time.time() - last > 5 and fb:
        last = time.time()
        c = track[-1][3] if track else float("nan")
        print(f"  남은 거리 {fb.distance_remaining:.1f} m · 경유점 남음 {fb.number_of_poses_remaining} · 복구 {fb.number_of_recoveries} "
              f"· 이탈 {c:.2f} m", flush=True)
    if track and track[-1][0] - track[0][0] > a.timeout:
        nav.cancelTask()
        break
res = nav.getResult()
name = {TaskResult.SUCCEEDED: "SUCCEEDED", TaskResult.CANCELED: "CANCELED", TaskResult.FAILED: "FAILED"}.get(res, str(res))
c = np.array([t[3] for t in track]) if track else np.array([np.nan])
summ = {"route": a.route, "result": name, "spacing_m": a.spacing, "waypoints": len(goals),
        "lap_sim_s": round(track[-1][0] - track[0][0], 1) if track else None, "wall_s": round(time.time() - t0w, 1),
        "mean_cte": round(float(np.nanmean(c)), 3), "max_cte": round(float(np.nanmax(c)), 3),
        "over_2m_frac": round(float(np.mean(c > 2.0)), 4), "recoveries": getattr(nav.getFeedback(), "number_of_recoveries", None)}
print("NAV2_ROUTE_DONE " + " ".join(f"{k}={v}" for k, v in summ.items()), flush=True)
if a.out:
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump({**summ, "track": [list(map(lambda v: round(v, 3), t)) for t in track]}, open(a.out, "w"))
nav.destroy_node()
rclpy.shutdown()
