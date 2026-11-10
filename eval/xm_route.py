#!/usr/bin/env python3
"""X-Mobility 폐루프 평가 한 회 — fd_xm_infer 노드를 띄워 몰게 하고 sim/nav2/nav2_route.py 와 같은 식으로 판정한다. (E 초안 · D 채택/폐기)

씬(sim/run_proxy.sh headless + 시나리오 레이어)이 떠 있어야 한다. 일괄은 eval/xm_batch.sh, 설명은 eval/XM_HARNESS.md.

    python3 eval/xm_route.py --scenario eval/scenarios/parking_r3_obs_cones.yaml --direction ccw --ckpt <...>.ckpt \
        --method xmobility_ft --seed 1 --out data/eval/<묶음>/runs/<이름>.json --csv data/eval/<묶음>/results.csv
    python3 eval/xm_route.py ... --dry-run          # ROS 없이 노드 명령 · 출발 자세 · 목표 지점만 찍음
    python3 eval/xm_route.py --rejudge <run.json> --scenario ... [--direction ..] --method nav2 --csv ...   # 저장된 궤적을 다시 판정

한 회 순서
  1. 준비 확인 — /ground_truth/tf 첫 메시지(15 s, nav2_route.py:213-217) · /cmd_vel 발행자 0(topics.md:40 · :63) ·
     odom + 출발 자세가 정답 위치와 0.5 m · 10° 안(nav2_route.py:221-238 의 AMCL 확인에 해당). 실패면 코드 3(일괄이 다시 띄움,
     scenario_batch.sh:96 과 같은 규칙) — CSV 행은 안 씀.
  2. 추론 노드(python -m fd_xm_infer.node, ml_env)를 자식 프로세스로 띄움 — route_source=csv · pose_source=odom · 출발 자세 · goal_s.
  3. 노드의 첫 /cmd_vel 이 오면 궤적 기록 시작. 첫 표본 시각 = t_goal(nav2_route.py:243 "목표를 보낸 시뮬 시각").
  4. 0.1 s 표본마다(nav2_route.py:137) 끝내기 판단: cte > deviation_limit_m → INTERVENED(results_schema.md §3) · 기준선 진행이
     마지막 경유점(nav2_route.py:198-204 의 i_goal)을 지남 → SUCCEEDED · t − t_goal > timeout → CANCELED(nav2_route.py:253-255) ·
     노드 종료 → FAILED.
  5. 노드에 SIGINT(노드가 정지 명령 5번, ros2_ws/src/fd_xm_infer/fd_xm_infer/node.py:434-443) → 노드가 사라진 뒤 이쪽도 정지 명령
     5번(발행자 1개 규칙) → 멈출 때까지 표본을 더 받음.
  6. 판정 = nav2_route.py:258-289 의 식(summarize) → JSON(nav2_route --out 과 같은 키 + 스키마 칸) · 결과 CSV 한 행(results_schema.md v0).

nav2_route.py 는 import 하면 argparse · rclpy · Nav2 가 바로 돌아(모듈 수준 코드) 가져올 수 없다 — 판정에 쓰는 순수 계산만 아래에 옮겨
적고 줄 번호를 달았다. 같은지는 nav2_route.py 의 문장을 ast 로 떼어 같은 궤적에 돌려 대조함(XM_HARNESS.md "확인").
필요: 판정 · CSV 는 numpy · PyYAML(시스템 python3), 실주행은 + rclpy(시스템 Jazzy). 노드는 ~/xmobility_test/venv python(D 11.10 노드 시험, --node-python —
xm_batch.sh 가 씬을 띄우기 전에 import 로 확인).
"""
import argparse
import csv
import datetime
import hashlib
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCHEMA_VERSION = "v0"                       # eval/results_schema.md · analyze_runs.py:40

# ---------------------------------------------------------------- nav2_route.py 와 같은 값
SAMPLE_DT = 0.1                             # nav2_route.py:137 — 정답 위치를 시뮬 0.1 s 마다 표본
STALL_V = 0.05                              # nav2_route.py:259 · :264 — 0.05 m/s 미만이면 정지
STALL_STRIDE = 5                            # nav2_route.py:261 — 표본 5개(0.5 s) 간격 속도
OVER_M = 2.0                                # nav2_route.py:271 — over_2m_frac 은 한도와 무관하게 2.0 고정
DIST_RATIO = 0.8                            # nav2_route.py:289
LAP_GOAL_BACK_M = 1.0                       # nav2_route.py:203 — 바퀴의 마지막 경유점 = 결승 1 m 앞
LIM_DEFAULT = (2.0, 10.0, 1.0)              # nav2_route.py:287 · :289 — deviation_limit_m · stall_limit_s · goal.radius_m
FOOT = [(0.14, 0.25), (0.14, -0.25), (-0.607, -0.25), (-0.607, 0.25)]   # nav2_route.py:70 = nav2_params.yaml:212
GT_PARENT, GT_CHILD = "World", "nova_carter"                            # nav2_route.py:135 · topics.md:39
TIMEOUT_S = 300.0                           # scenario_batch.sh:89 — nav2_route.py 에 --timeout 300 (yaml timeout_s 는 안 씀)
GOAL_XY_TOL = 0.25                          # nav2_params.yaml:156 xy_goal_tolerance — Nav2 가 SUCCEEDED 를 내는 반경.
                                            # 구간 s[i_goal] 은 goal.s 보다 0.21~0.24 m 뒤(계산 s ≠ CSV s 열)인데 노드 · 학습 경로는 goal.s 에서 끝남
# ---------------------------------------------------------------- 여기서 정한 값(nav2 에 없는 부분)
START_POS_TOL, START_YAW_TOL_DEG = 0.5, 10.0   # 출발 확인 — 위치는 nav2_route.py:227 의 0.5 m, 방향은 여기서 정함
PROJ_BACK, PROJ_FWD = 1.0, 10.0             # 진행 거리 사영 창 = fd_xm_infer core.py:27 PROJECT_BACK · FWD(= export_xmobility.py:239)
SETTLE_V, SETTLE_SIM_S = 0.05, 3.0          # 끝낸 뒤 odom |v| < 0.05 m/s 가 되거나 시뮬 3 s 까지 표본을 더 받음
METHODS = ("xmobility_ft", "xmobility_zeroshot", "nav2")   # analyze_runs.py:96
METHOD_SHORT = {"xmobility_ft": "xmft", "xmobility_zeroshot": "xmzs", "nav2": "nav2"}
TRACK_COLS = ["t_sim", "x", "y", "cte", "clearance", "yaw", "v_cmd", "w_cmd", "v_odom", "station"]   # 앞 5칸 = nav2_route.py:140
# 결과 CSV 열 — analyze_runs.py:42-93 COLS 의 순서 그대로(필수 33 + 선택 15). 시험이 COLS 와 같은지 확인
CSV_COLUMNS = [
    "schema_version", "run_id", "timestamp", "repo_rev", "method", "model_version", "scene", "scene_version", "scenario",
    "scenario_version", "direction", "seed", "result", "pass", "completed", "max_dev_ok", "no_collision", "no_long_stop",
    "goal_reached", "distance_ratio_ok", "fail_reason", "mean_cte", "max_cte", "p95_cte", "lap_sim_s", "max_stall_s",
    "collision", "min_clearance", "interventions", "dist_m", "plan_m", "goal_err", "log_path",
    "kind", "localization", "nav2_map", "route", "over_2m_frac", "recoveries", "wall_s", "amcl_start_err",
    "initialpose_resent", "deviation_limit_m", "stall_limit_s", "goal_radius_m", "timeout_s", "rc", "attempt", "note",
]
CRITERIA = ["completed", "max_dev_ok", "no_collision", "no_long_stop", "goal_reached", "distance_ratio_ok"]   # analyze_runs.py:94


def _abs(p):
    return p if os.path.isabs(p) else os.path.join(REPO, p)


# ---------------------------------------------------------------- 판정 계산 — nav2_route.py 에서 옮김(전역 → 인자만 바꿈)

def load_route(path):
    """nav2_route.py:50-53 — 기준선 CSV run 0 점 → (R, seg, s, L)."""
    R = np.array([(float(r["x"]), float(r["y"])) for r in csv.DictReader(open(_abs(path))) if r.get("run", "0") == "0"])
    seg = np.diff(R, axis=0)
    s = np.concatenate([[0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
    return R, seg, s, s[-1]


def cte(R, seg, x, y):
    """nav2_route.py:62-67 — 기준선(닫힌 꺾은선)까지 거리."""
    a0, d = R[:-1], seg
    t = np.clip(((x - a0[:, 0]) * d[:, 0] + (y - a0[:, 1]) * d[:, 1]) / np.maximum((d ** 2).sum(1), 1e-12), 0, 1)
    px, py = a0[:, 0] + t * d[:, 0], a0[:, 1] + t * d[:, 1]
    return float(np.min(np.hypot(px - x, py - y)))


def build_obstacles(SC):
    """nav2_route.py:72-88 — 장애물 다각형(sim/make_obstacles.py:34-39 on_route 와 같은 자리)."""
    OBS = []
    RS = np.array([(float(r["s"]), float(r["x"]), float(r["y"])) for r in csv.DictReader(open(_abs(SC["route"]))) if r.get("run", "0") == "0"])
    for o in SC.get("obstacles", []):
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
    return OBS


def _seg_dist(p, a_, b_):
    """nav2_route.py:91-94."""
    ax, ay = a_; bx, by = b_; px, py = p
    dx, dy = bx - ax, by - ay; t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / max(dx * dx + dy * dy, 1e-12)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _inside(p, poly):
    """nav2_route.py:97-99."""
    sgn = [((b_[0] - a_[0]) * (p[1] - a_[1]) - (b_[1] - a_[1]) * (p[0] - a_[0])) for a_, b_ in zip(poly, poly[1:] + poly[:1])]
    return all(v >= 0 for v in sgn) or all(v <= 0 for v in sgn)


def clearance(OBS, x, y, th):
    """nav2_route.py:102-113 — 로봇 발자국과 가장 가까운 장애물 사이 거리(m, 겹치면 0)."""
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


def quat_yaw(x, y, z, w):
    """nav2_route.py:139."""
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def max_stall(track, t_goal):
    """nav2_route.py:259-266 — 가장 긴 정지(0.05 m/s 미만), 0.5 s 간격 속도, t_goal 부터."""
    stall, run, prev = 0.0, 0.0, None
    for t in [t for t in track if t[0] >= t_goal][::STALL_STRIDE]:
        if prev is not None:
            v = math.hypot(t[1] - prev[1], t[2] - prev[2]) / max(t[0] - prev[0], 1e-6)
            run = run + (t[0] - prev[0]) if v < STALL_V else 0.0
            stall = max(stall, run)
        prev = t
    return stall


# ---------------------------------------------------------------- 시나리오 · 기준선

class Ctx:
    """시나리오 하나 · 방향 하나의 판정 재료 — nav2_route.py:44-53 · :72-88 · :198-204 · :282 · :287-289."""

    def __init__(self, scenario, direction=None, route=None):
        import yaml
        self.scenario_path = scenario
        self.SC = SC = yaml.safe_load(open(_abs(scenario)))
        self.segment = "s" in (SC.get("goal") or {})                               # nav2_route.py:48
        dirs = SC.get("directions") or ([SC["direction"]] if SC.get("direction") else [])
        if self.segment:
            direction = direction or SC.get("direction")
            if SC.get("direction") and direction != SC["direction"]:
                raise SystemExit(f"구간 시나리오 방향은 yaml 의 {SC['direction']} 뿐 — --direction {direction}")
        elif direction not in ("cw", "ccw"):
            raise SystemExit("바퀴 시나리오는 --direction cw|ccw 가 필요(scenario_batch.sh 의 yaml:cw · yaml:ccw)")
        if dirs and direction not in dirs:
            raise SystemExit(f"{scenario}: 방향 {direction} 이 yaml {dirs} 에 없음")
        self.direction = direction
        # 기준선 — 구간이면 시나리오 route, 바퀴면 방향별 CSV(nav2_route.py:49 · scenario_batch.sh:79)
        self.route = SC["route"] if self.segment else (route or f"eval/routes/parking_r3_{direction}.csv")
        self.R, self.seg, self.s, self.L = load_route(self.route)
        self.OBS = build_obstacles(SC)                                             # nav2_route.py:72-88
        if self.segment:                                                           # nav2_route.py:198-204 (i_goal 만)
            self.s0, self.s1 = float(SC["start"]["s"]), float(SC["goal"]["s"])
            self.i_goal = int(min(np.searchsorted(self.s, self.s1), len(self.R) - 1))
        else:
            self.s0 = self.s1 = None
            self.i_goal = int(np.searchsorted(self.s, self.L - LAP_GOAL_BACK_M))
        self.plan_len = (self.s1 - self.s0) if self.segment else self.L           # nav2_route.py:282
        self.lim_dev = float(SC.get("deviation_limit_m", LIM_DEFAULT[0]))         # nav2_route.py:287
        self.lim_stall = float(SC.get("stall_limit_s", LIM_DEFAULT[1]))
        self.radius = float((SC.get("goal") or {}).get("radius_m", LIM_DEFAULT[2]))   # nav2_route.py:289
        self.goal_station = float(self.s[self.i_goal])
        self.name = SC.get("name") or os.path.splitext(os.path.basename(scenario))[0]

    def sample(self, st, x, y, th):
        """nav2_route.py:140 의 표본 한 줄 (시뮬 초, x, y, cte, 장애물 거리 — 장애물 없으면 −1)."""
        return (st, x, y, cte(self.R, self.seg, x, y), clearance(self.OBS, x, y, th) if self.OBS else -1.0)

    def start_pose(self):
        """씬 출발 자세(지도 x, y, yaw°) — 노드의 odom → 지도 변환용(core.py:105-109).
        구간: sim/make_obstacles.py:34-39 · :115-116 의 on_route(start.s, 0)(레이어가 로봇을 그 자리에 놓음).
        바퀴: 시나리오 start(씬 루트 자세와 같음 — 시나리오 yaml 주석 "씬 시작 자세 = 기준선 출발점"). 반시계 바퀴도 씬은
        시계 방향(−96°)을 보고 뜸 — 노드 기본(CSV 옆 parking_r3_ccw.yaml 의 84°, node.py:389-395)을 쓰면 180° 틀림."""
        st = self.SC.get("start") or {}
        if "s" in st:
            Rs = np.array([(float(r["s"]), float(r["x"]), float(r["y"])) for r in csv.DictReader(open(_abs(self.SC["route"])))
                           if r.get("run", "0") == "0"])
            sv = float(st["s"])
            i = int(np.clip(np.searchsorted(Rs[:, 0], sv), 1, len(Rs) - 1))
            (s0, x0, y0), (s1, x1, y1) = Rs[i - 1], Rs[i]
            f = (sv - s0) / max(s1 - s0, 1e-9)
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0), math.degrees(math.atan2(y1 - y0, x1 - x0))
        return float(st["x"]), float(st["y"]), float(st["yaw_deg"])


def kind_of(name):
    """analyze_runs.py:255-264 와 같은 규칙."""
    if "_obs_" in name:
        return "obstacle"
    if "_straight_" in name or name.endswith("_straight"):
        return "straight"
    if "_curve_" in name:
        return "curve"
    if name.endswith("_lap"):
        return "lap"
    return "other"


def project_s(ctx, x, y, hint=None, back=PROJ_BACK, fwd=PROJ_FWD):
    """기준선 위 진행 거리 s(사영) · 거리 — fd_xm_infer core.py:173-188 Poly.project 와 같은 뜻(힌트 창 back · fwd, 없으면 전체)."""
    a0, d = ctx.R[:-1], ctx.seg
    l2 = (d ** 2).sum(1)
    t = np.clip(((x - a0[:, 0]) * d[:, 0] + (y - a0[:, 1]) * d[:, 1]) / np.maximum(l2, 1e-12), 0, 1)
    dist = np.hypot(a0[:, 0] + t * d[:, 0] - x, a0[:, 1] + t * d[:, 1] - y)
    st = (ctx.s[:-1] + t * np.sqrt(l2)) % ctx.L
    if hint is not None:
        dl = (st - hint + ctx.L / 2) % ctx.L - ctx.L / 2
        ok = (dl >= -back) & (dl <= fwd)
        if ok.any():
            dist = np.where(ok, dist, np.inf)
    i = int(np.argmin(dist))
    return float(st[i]), float(dist[i])


def _wrap_s(d, L):
    return (d + L / 2) % L - L / 2


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def odom_to_map(ox, oy, oyaw, start):
    """fd_xm_infer core.py:105-109 = topics.md:22-23 — 노드와 같은 식으로 출발 확인."""
    x0, y0, p0 = float(start[0]), float(start[1]), math.radians(float(start[2]))
    c, s = math.cos(p0), math.sin(p0)
    return x0 + c * ox - s * oy, y0 + s * ox + c * oy, _wrap(p0 + oyaw)


# ---------------------------------------------------------------- 한 회의 끝내기 판단(nav2 에는 없음 — Nav2 는 스스로 도착을 앎)

class Episode:
    """0.1 s 표본마다 update. 기준선 진행(station, 감은 s)이 마지막 경유점 s[i_goal] 을 지나면 도착."""

    def __init__(self, ctx, t_goal, x0, y0, timeout_s, intervene=True, stall_abort_s=0.0, goal_stop_s=0.0):
        self.ctx, self.t_goal, self.timeout_s = ctx, t_goal, timeout_s
        self.intervene, self.stall_abort_s, self.goal_stop_s = intervene, stall_abort_s, goal_stop_s
        self.gx, self.gy = (float(v) for v in ctx.R[ctx.i_goal])
        p0, _ = project_s(ctx, x0, y0)
        ref0 = ctx.s0 if ctx.segment else 0.0
        self.station = ref0 + _wrap_s(p0 - ref0, ctx.L)
        self.s_prev = p0
        # nav2_route.py:260-266 정지 루프를 표본마다 이어 셈 — t_goal 표본(첫 표본)이 [::5] 의 0번
        self.k, self.prev, self.run = 1, (t_goal, x0, y0), 0.0
        self.reason, self.detail = None, ""

    def update(self, t, x, y, cte_v):
        """→ None 또는 끝낸 이유("INTERVENED" · "SUCCEEDED" · "CANCELED"). 이미 끝났으면 진행 거리만 이어 셈."""
        ctx = self.ctx
        p, _ = project_s(ctx, x, y, hint=self.s_prev)
        self.station += _wrap_s(p - self.s_prev, ctx.L)
        self.s_prev = p
        if t >= self.t_goal:
            if self.k % STALL_STRIDE == 0:
                if self.prev is not None:
                    v = math.hypot(x - self.prev[1], y - self.prev[2]) / max(t - self.prev[0], 1e-6)
                    self.run = self.run + (t - self.prev[0]) if v < STALL_V else 0.0
                self.prev = (t, x, y)
            self.k += 1
        if self.reason:
            return None
        # max_dev_ok 가 쓰는 값(3자리 반올림, nav2_route.py:270 · :288)과 같은 값으로 비교 — 개입 ⇔ max_dev_ok 거짓
        if self.intervene and round(cte_v, 3) > ctx.lim_dev:
            self.reason, self.detail = "INTERVENED", f"cte {cte_v:.3f} > {ctx.lim_dev} at s {p:.1f}"
        elif self.station >= ctx.goal_station - GOAL_XY_TOL:
            self.reason, self.detail = "SUCCEEDED", f"진행 {self.station:.2f} ≥ 목표 지점 s {ctx.goal_station:.2f} − {GOAL_XY_TOL}"
        elif (self.goal_stop_s > 0 and self.run >= self.goal_stop_s and self.station >= ctx.goal_station - ctx.radius
              and math.hypot(x - self.gx, y - self.gy) <= ctx.radius):
            # 학습 교사(Nav2)는 목표 앞에서 멈춤 — 모델이 반경 안에서 멈추면 제한 시간(300 s)까지 기다리지 않고 도착으로 끝냄
            self.reason, self.detail = "SUCCEEDED", f"목표 반경 {ctx.radius} m 안에서 {self.run:.1f} s 정지(진행 {self.station:.2f})"
        elif t - self.t_goal > self.timeout_s:
            self.reason, self.detail = "CANCELED", f"제한 시간 {self.timeout_s:g} s"
        elif self.stall_abort_s > 0 and self.run >= self.stall_abort_s:
            self.reason, self.detail = "CANCELED", f"정지 {self.run:.1f} s ≥ --stall-abort-s {self.stall_abort_s:g}"
        return self.reason


# ---------------------------------------------------------------- 요약 · 판정 · 행

def count_interventions(c, lim):
    """2.0 m 개입 횟수 — cte(3자리 반올림)가 한도를 위로 넘는 순간마다 1(results_schema.md §2.3 interventions). 궤적 전체."""
    n, above = 0, False
    for v in c:
        if not np.isfinite(v):
            continue
        now = round(float(v), 3) > lim
        if now and not above:
            n += 1
        above = now
    return n


def summarize(ctx, track, t_goal, name, wall_s=None, recoveries=None, spacing=None, waypoints=None,
              amcl_start_err=None, initialpose_resent=None):
    """nav2_route.py:258-289 의 판정을 문장 순서 그대로 — summ(같은 키 · 같은 반올림)과 기준별 bool."""
    c = np.array([t[3] for t in track]) if track else np.array([np.nan])                  # :258
    stall = max_stall(track, t_goal)                                                         # :259-266
    clr = [t[4] for t in track if t[4] >= 0]                                                 # :267
    summ = {"route": ctx.route, "result": name, "spacing_m": spacing, "waypoints": waypoints,   # :268-272
            "lap_sim_s": round(track[-1][0] - t_goal, 1) if track else None, "wall_s": wall_s,
            "mean_cte": round(float(np.nanmean(c)), 3), "max_cte": round(float(np.nanmax(c)), 3),
            "over_2m_frac": round(float(np.mean(c > OVER_M)), 4), "recoveries": recoveries,
            "max_stall_s": round(stall, 1)}
    if ctx.OBS:                                                                              # :273-274
        summ.update({"scenario": ctx.scenario_path, "min_clearance": round(min(clr), 3) if clr else None,
                     "collision": bool(clr and min(clr) <= 0.0)})
    gx, gy = ctx.R[ctx.i_goal]                                                               # :277-278
    goal_err = math.hypot(track[-1][1] - gx, track[-1][2] - gy) if track else float("inf")
    tr = [t for t in track if t[0] >= t_goal]                                                # :280-281
    dist = float(sum(math.hypot(q[1] - p[1], q[2] - p[2]) for p, q in zip(tr, tr[1:])))
    plan_len = ctx.plan_len                                                                  # :282
    summ.update({"goal_err": round(goal_err, 3), "dist_m": round(dist, 1), "plan_m": round(plan_len, 1),   # :283-284
                 "amcl_start_err": amcl_start_err, "initialpose_resent": initialpose_resent})
    if ctx.segment:                                                                          # :285-286
        summ.update({"scenario": ctx.scenario_path, "segment": [ctx.s0, ctx.s1]})
    lim_dev, lim_stall = ctx.lim_dev, ctx.lim_stall                                          # :287
    # :288-289 의 AND 를 항목별로 — 같은 피연산자(max_cte 는 반올림 값, 정지 · 목표 · 거리는 반올림 전 값)
    crit = {"completed": name == "SUCCEEDED", "max_dev_ok": summ["max_cte"] <= lim_dev,
            "no_collision": not summ.get("collision", False), "no_long_stop": stall < lim_stall,
            "goal_reached": goal_err <= ctx.radius, "distance_ratio_ok": dist >= DIST_RATIO * plan_len}
    summ["pass"] = bool(name == "SUCCEEDED" and summ["max_cte"] <= lim_dev and not summ.get("collision", False) and stall < lim_stall
                        and goal_err <= ctx.radius and dist >= DIST_RATIO * plan_len)
    crit = {k: bool(v) for k, v in crit.items()}
    assert summ["pass"] == all(crit.values())
    raw = dict(stall=stall, goal_err=goal_err, dist=dist, c=c, clr=clr)
    return summ, crit, raw


def fail_reason(ctx, track, t_goal, summ, crit, raw, harness=""):
    """주 원인 코드 하나 + ':' 뒤 짧은 설명(results_schema.md §2.2). 순서: harness · collision · max_dev · long_stop · timeout ·
    not_completed · goal · distance. INTERVENED 는 max_dev 로 잡힘(개입 ⇔ max_dev_ok 거짓)."""
    if summ["pass"]:
        return ""
    if harness:
        return f"harness:{harness}"

    def where(i):
        return f"s {project_s(ctx, track[i][1], track[i][2])[0]:.1f}"
    if not crit["no_collision"]:
        i = next(k for k, t in enumerate(track) if 0 <= t[4] <= 0.0)
        return f"collision:{where(i)}"
    if not crit["max_dev_ok"]:
        i = int(np.nanargmax(raw["c"]))
        return f"max_dev:{where(i)} cte {summ['max_cte']:.2f} m"
    if not crit["no_long_stop"]:
        return f"long_stop:{raw['stall']:.1f} s"
    if summ["result"] == "CANCELED":
        return "timeout:" + (f"{summ['lap_sim_s']} s" if summ.get("lap_sim_s") is not None else "")
    if not crit["completed"]:
        return f"not_completed:{summ['result']}"
    if not crit["goal_reached"]:
        return f"goal:goal_err {raw['goal_err']:.2f} m"
    return f"distance:{raw['dist']:.1f}/{ctx.plan_len:.1f} m"


def repo_rev(root=REPO):
    """git rev-parse HEAD + 더러우면 -dirty(results_schema.md repo_rev, 판정은 node.py:84-93 · eval_offline.py:382-388 와 같게 diff HEAD)."""
    try:
        rev = subprocess.check_output(["git", "-C", root, "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
        dirty = subprocess.call(["git", "-C", root, "diff", "--quiet", "HEAD"], stderr=subprocess.DEVNULL) != 0
        return rev + ("-dirty" if dirty else "")
    except Exception:  # noqa: BLE001 — analyze_runs 가 형식 오류로 잡음
        return "unknown"


def sha12(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:12]


def model_version(ckpt, method, explicit=None):
    """results_schema.md model_version — 지정값 > ckpt 경로의 xm_ft_v<주>.<부>[_<변형>…](e2d_model.md 버전 칸 · MANIFEST 행:
    xm_ft_v0.1 · xm_ft_v0.1_aw30 · _aw30_p3 · _p3 · _seed1. 끝의 _e<에폭>(Drive 파일 xm_ft_v0.1_aw30_e8.ckpt · Kaggle 출력 폴더 _e10)은 뗌)
    > finetune 출력 폴더 이름(<out>/checkpoints/<ckpt> 옆 run_meta.json, e2e/finetune.py:173) > ckpt 파일 이름.
    실물 확인은 JSON ckpt_sha12 · CSV note 의 ckpt_sha12(후보들은 Kaggle 파일 이름이 같아 sha 로만 구분)."""
    if explicit:
        return explicit
    m = re.search(r"xm_ft_v\d+\.\d+(?:_[A-Za-z0-9]+)*", ckpt or "")
    if m:
        return re.sub(r"(_e\d+)+$", "", m.group(0))
    out = os.path.dirname(os.path.dirname(os.path.abspath(ckpt or ".")))
    if ckpt and os.path.isfile(os.path.join(out, "run_meta.json")):
        return f"{os.path.basename(out)}/{os.path.basename(ckpt)}"
    return ("zeroshot:" if method == "xmobility_zeroshot" else "") + os.path.basename(ckpt or "unknown")


def scene_version(scene):
    """씬 루트 파일 이름 → MANIFEST usd 행 표기(parking_v0.4b_r3.usda → 'v0.4b r3', parking_v0.6.usda → 'v0.6')."""
    m = re.search(r"_v(\d+\.\d+[a-z]?)(?:_(r\d+))?\.usda$", scene or "")
    return (f"v{m.group(1)}" + (f" {m.group(2)}" if m.group(2) else "")) if m else os.path.basename(scene or "")


def make_row(ctx, summ, crit, raw, meta):
    """results_schema.md v0 한 행. meta: run_id · timestamp · repo_rev · method · model_version · scene · seed · log_path · ..."""
    lim_note = meta.get("note", "")
    row = {
        "schema_version": SCHEMA_VERSION, "run_id": meta["run_id"], "timestamp": meta["timestamp"], "repo_rev": meta["repo_rev"],
        "method": meta["method"], "model_version": meta["model_version"], "scene": meta["scene"],
        "scene_version": meta.get("scene_version") or scene_version(meta["scene"]),
        "scenario": ctx.name, "scenario_version": str(ctx.SC.get("version", "")), "direction": ctx.direction, "seed": meta["seed"],
        "result": summ["result"], "pass": summ["pass"], **crit,
        "fail_reason": meta["fail_reason"],
        "mean_cte": summ["mean_cte"], "max_cte": summ["max_cte"],
        "p95_cte": round(float(np.nanpercentile(raw["c"], 95)), 3),
        "lap_sim_s": summ["lap_sim_s"], "max_stall_s": summ["max_stall_s"],
        "collision": bool(summ.get("collision", False)), "min_clearance": summ.get("min_clearance"),
        "interventions": count_interventions(raw["c"], ctx.lim_dev),
        "dist_m": summ["dist_m"], "plan_m": summ["plan_m"], "goal_err": summ["goal_err"], "log_path": meta["log_path"],
        "kind": kind_of(ctx.name), "localization": meta.get("localization", ""), "nav2_map": meta.get("nav2_map", ""),
        "route": ctx.route, "over_2m_frac": summ["over_2m_frac"], "recoveries": summ.get("recoveries"), "wall_s": summ.get("wall_s"),
        "amcl_start_err": summ.get("amcl_start_err"), "initialpose_resent": summ.get("initialpose_resent"),
        "deviation_limit_m": ctx.lim_dev, "stall_limit_s": ctx.lim_stall, "goal_radius_m": ctx.radius,
        "timeout_s": meta.get("timeout_s"), "rc": meta.get("rc", 0), "attempt": meta.get("attempt", 1), "note": lim_note,
    }
    assert list(row) == CSV_COLUMNS
    return row


def _cell(v):
    if v is None:
        return ""
    if isinstance(v, (bool, np.bool_)):
        return "true" if v else "false"
    if isinstance(v, (float, np.floating)):
        return "" if not math.isfinite(float(v)) else repr(round(float(v), 6))
    return str(v)


def append_csv(path, row):
    """머리글이 없으면 쓰고, 있으면 CSV_COLUMNS 와 같은지 확인한 뒤 한 행 덧붙임."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    exists = os.path.isfile(path) and os.path.getsize(path) > 0
    if exists:
        with open(path, newline="", encoding="utf-8") as f:
            head = next(csv.reader(f), [])
        if head != CSV_COLUMNS:
            raise RuntimeError(f"{path}: 머리글이 이 하네스의 열과 다름 — 다른 파일에 쓰세요")
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if not exists:
            w.writerow(CSV_COLUMNS)
        w.writerow([_cell(row[c]) for c in CSV_COLUMNS])


def run_id_of(ts, method, ctx, seed, attempt=1):
    """results_schema.md run_id 예 20261124_0931_xmft_obs_cones_ccw_s3 — 초까지(같은 분에 두 회가 겹치지 않게), 시도 2회째부터 _a<n>."""
    short = METHOD_SHORT.get(method, method)
    sc = ctx.name.replace("parking_r3_", "")
    sc = sc if sc.endswith("_" + ctx.direction) else f"{sc}_{ctx.direction}"   # curve_se_cw 처럼 이름에 방향이 있으면 한 번만
    return f"{ts:%Y%m%d_%H%M%S}_{short}_{sc}_s{seed}" + (f"_a{attempt}" if attempt > 1 else "")


def finish(ctx, a, track, t_goal, name, meta, extra, wall_s=None, harness="", summ_kw=None, t_end=None):
    """요약 · 판정 → JSON(nav2_route.py:291-293 모양 + 칸 추가) · CSV 행 · NAV2_ROUTE_DONE 같은 한 줄.
    t_end 가 있으면 판정은 끝내기 판단 시각까지의 표본만(nav2_route.py 궤적은 isTaskComplete 에서 끝남 — 멈추는 동안의 표본이
    lap_sim_s · goal_err · dist_m 에 안 들어감). JSON track 은 멈춘 뒤까지 전부."""
    judge = [t for t in track if t_end is None or t[0] <= t_end + 1e-6] or track
    summ, crit, raw = summarize(ctx, judge, t_goal, name, wall_s=wall_s, **(summ_kw or {}))
    meta["fail_reason"] = fail_reason(ctx, judge, t_goal, summ, crit, raw, harness)
    row = make_row(ctx, summ, crit, raw, meta)
    out = dict(summ)
    out.update({k: row[k] for k in ("schema_version", "run_id", "timestamp", "repo_rev", "method", "model_version", "scene",
                                     "scene_version", "direction", "seed", "p95_cte", "interventions", "fail_reason", "kind",
                                     "deviation_limit_m", "stall_limit_s", "goal_radius_m", "timeout_s", "attempt")})
    out.update({"criteria": crit, "scenario_yaml": ctx.scenario_path, "t_goal": round(t_goal, 3) if t_goal is not None else None,
                "judged_samples": len(judge), "track_columns": TRACK_COLS[:len(track[0])] if track else []})
    out.update(extra)
    out["track"] = [[round(float(v), 3) for v in t] for t in track]                       # nav2_route.py:293 과 같은 3자리
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False)
    if a.csv:
        try:
            append_csv(a.csv, row)
        except RuntimeError as e:
            print(f"XM_ROUTE_CSV_FAIL {e}", flush=True)
    keys = ("result", "pass", "fail_reason", "lap_sim_s", "mean_cte", "max_cte", "p95_cte", "max_stall_s", "min_clearance",
            "collision", "interventions", "goal_err", "dist_m", "plan_m")
    print("XM_ROUTE_DONE " + " ".join(f"{k}={row[k] if k in row else summ.get(k)}" for k in keys) + f" run_id={row['run_id']}", flush=True)
    return summ, crit, row


# ---------------------------------------------------------------- 저장된 궤적 다시 판정

def rejudge(a):
    """--rejudge — 이 하네스나 nav2_route.py --out 의 JSON 궤적을 같은 summarize 로 다시 판정(nav2 행 만들기 · 판정 바꿀 때 재계산).
    t_goal 이 JSON 에 없으면(nav2_route.py JSON) track[-1][0] − lap_sim_s 로 되짚음 — lap_sim_s 가 0.1 s 반올림이라 근사."""
    j = json.load(open(a.rejudge, encoding="utf-8"))
    route = a.route or j.get("route") or None
    m = re.search(r"_(cw|ccw)\.csv$", route or "")
    ctx = Ctx(a.scenario, a.direction or j.get("direction") or (m.group(1) if m else None), route)
    track = [tuple(t) for t in j["track"]]
    t_goal = j.get("t_goal")
    approx = t_goal is None
    if approx:
        # nav2_route.py:243 의 t_goal 은 표본 시각 그 자체 — lap_sim_s(0.1 s 반올림) · 3자리 반올림 · 부동소수 뺄셈 오차로
        # 첫 표본이 t ≥ t_goal 에서 빠지면 [::5] 정지 보폭이 한 칸 밀려 max_stall_s 가 0.5 s 까지 달라짐 → 가장 가까운 표본 시각에 붙임
        approx_t = track[-1][0] - float(j["lap_sim_s"])
        t_goal = min((t[0] for t in track), key=lambda v: abs(v - approx_t))
    ts = datetime.datetime.fromisoformat(j["timestamp"]) if j.get("timestamp") else \
        datetime.datetime.fromtimestamp(os.path.getmtime(a.rejudge)).astimezone()
    method = a.method or j.get("method") or "nav2"
    seed = a.seed if a.seed is not None else int(j.get("seed", 0))
    meta = dict(run_id=j.get("run_id") or run_id_of(ts, method, ctx, seed, a.attempt), timestamp=ts.isoformat(timespec="seconds"),
                repo_rev=a.repo_rev or j.get("repo_rev") or repo_rev(), method=method,
                model_version=a.model_version or j.get("model_version") or "unknown",
                scene=a.scene or j.get("scene") or ctx.SC.get("scene", ""), scene_version=a.scene_version, seed=seed,
                log_path=os.path.relpath(os.path.abspath(a.rejudge), REPO), localization=a.localization or "",
                nav2_map=a.nav2_map or "", timeout_s=j.get("timeout_s", a.timeout), rc=0, attempt=a.attempt,
                note=(a.note or "") + ("; t_goal 근사(lap_sim_s)" if approx else ""))
    summ_kw = dict(recoveries=j.get("recoveries"), spacing=j.get("spacing_m"), waypoints=j.get("waypoints"),
                   amcl_start_err=j.get("amcl_start_err"), initialpose_resent=j.get("initialpose_resent"))
    summ, crit, row = finish(ctx, a, track, t_goal, j["result"], meta, {"rejudged_from": a.rejudge}, wall_s=j.get("wall_s"),
                             harness=j.get("harness_failed") or "", summ_kw=summ_kw, t_end=j.get("t_end_decision"))
    diff = {k: (j[k], summ[k]) for k in ("mean_cte", "max_cte", "max_stall_s", "min_clearance", "collision", "goal_err", "dist_m",
                                         "plan_m", "pass") if k in j and k in summ and j[k] != summ[k]}
    if diff:
        print(f"XM_ROUTE_REJUDGE_DIFF 저장값과 다름 {diff}", flush=True)
    return 0


# ---------------------------------------------------------------- 실주행(ROS)

def node_command(a, ctx, start, log_csv):
    """fd_xm_infer 노드 명령 — 노드 README "colcon 없이 바로"(ros2_ws/src/fd_xm_infer/README.md:49-55). 런치 파일(start_x 등도
    인자로 받음)은 안 쓰고 직접 띄움 — 자식 프로세스(새 세션)의 pid · 신호 · 로그를 하네스가 다루려고."""
    params = [("use_sim_time", True), ("ckpt", os.path.abspath(a.ckpt) if a.ckpt else ""), ("route_source", "csv"),
              ("route_csv", _abs(ctx.route)), ("reverse_route", False),
              ("goal_s", float(ctx.s1) if ctx.segment else -1.0), ("pose_source", a.pose_source),
              ("start_x", float(start[0])), ("start_y", float(start[1])), ("start_yaw_deg", float(start[2])),
              ("seed", int(a.seed)), ("log_csv", log_csv)]
    fmt = lambda v: ("true" if v else "false") if isinstance(v, bool) else (repr(v) if isinstance(v, float) else str(v))   # noqa: E731
    args = []
    for k, v in params:
        args += ["-p", f"{k}:={fmt(v)}"]
    for kv in a.node_param or []:
        args += ["-p", kv]
    prefix = shlex.split(a.node_cmd) if a.node_cmd else [a.node_python, "-u", "-m", "fd_xm_infer.node"]
    env = dict(os.environ)
    env["PYTHONPATH"] = os.path.join(REPO, "ros2_ws", "src", "fd_xm_infer") + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    env.setdefault("CD1", REPO)
    env.setdefault("HF_HUB_OFFLINE", "1")                                      # 노드 README:19
    if a.cuda_visible_devices is not None:
        env["CUDA_VISIBLE_DEVICES"] = a.cuda_visible_devices                   # Isaac Sim 이 GPU 0 — 노드 README:19
    return prefix + ["--ros-args"] + args, env


class Live:
    """ROS 쪽 — 구독 · 노드 프로세스 · 끝내기. 계산은 위 순수 함수."""

    def __init__(self, a, ctx):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from tf2_msgs.msg import TFMessage
        self.rclpy, self.Twist = rclpy, Twist
        self.a, self.ctx = a, ctx
        rclpy.init()
        self.node = Node("xm_route", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])   # nav2_route.py:118
        self.track, self.recording, self.ep, self.t_goal = [], False, None, None
        self.gt_last = self.gt_wall = self.odom = None
        self.cmd, self.n_cmd = (0.0, 0.0), 0
        self.proc, self.node_log = None, None
        self.t_end = None
        # 정답 위치 — nav2_route.py:143 과 같은 QoS
        self.node.create_subscription(TFMessage, "/ground_truth/tf", self._on_gt, QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(Odometry, "/chassis/odom", self._on_odom, qos_profile_sensor_data)
        self.node.create_subscription(Twist, "/cmd_vel", self._on_cmd, qos_profile_sensor_data)

    # ---- 받기
    def _on_gt(self, m):
        for t in m.transforms:                                                     # nav2_route.py:133-140
            if t.header.frame_id == GT_PARENT and t.child_frame_id == GT_CHILD:
                st = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                x, y = t.transform.translation.x, t.transform.translation.y
                q = t.transform.rotation; th = quat_yaw(q.x, q.y, q.z, q.w)
                self.gt_last, self.gt_wall = (st, x, y, th), time.monotonic()
                if self.recording and (not self.track or st - self.track[-1][0] >= SAMPLE_DT):
                    smp = self.ctx.sample(st, x, y, th)
                    v_odom = self.odom[3] if self.odom else float("nan")
                    if self.ep is None:                                            # 첫 표본 = t_goal(nav2_route.py:243)
                        self.t_goal = st
                        self.ep = Episode(self.ctx, st, x, y, self.a.timeout, self.a.intervene == "stop", self.a.stall_abort_s,
                                          self.a.goal_stop_s)
                        self.t0w, self.ts = time.time(), datetime.datetime.now().astimezone()
                        ev = None
                    else:
                        ev = self.ep.update(st, x, y, smp[3])
                    self.track.append(smp + (th, self.cmd[0], self.cmd[1], v_odom, self.ep.station))
                    if ev and self.t_end is None:
                        self.t_end = st
                        print(f"XM_ROUTE_END {ev} t={st - self.t_goal:.1f} s · {self.ep.detail}", flush=True)

    def _on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom = (p.x, p.y, quat_yaw(q.x, q.y, q.z, q.w), m.twist.twist.linear.x, m.twist.twist.angular.z)

    def _on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.n_cmd += 1

    # ---- 도우미
    def spin_until(self, cond, wall_s):
        t0 = time.monotonic()
        while time.monotonic() - t0 < wall_s:
            if cond():
                return True
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        return cond()

    def other_cmd_pubs(self):
        return [f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in self.node.get_publishers_info_by_topic("/cmd_vel")]

    def node_alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop_node(self):
        """SIGINT → 노드가 정지 명령 5번 보내고 끝남(node.py:722-745 · :434-443). 안 끝나면 SIGTERM(같은 길, node.py:718-719) → SIGKILL."""
        if self.proc is None:
            return None
        for sig, wait in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
            if self.proc.poll() is not None:
                break
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                break
            self.spin_until(lambda: self.proc.poll() is not None, wait)
        rc = self.proc.poll()
        if self.node_log:
            self.node_log.close()
            self.node_log = None
        try:
            os.remove(self.a.out + ".node.pid")
        except OSError:
            pass
        return rc

    def zero_cmd(self):
        """노드가 없어진 뒤 이쪽에서 정지 명령 5번(sim/teleop_ramp.py:257-259 와 같은 횟수) — 다른 발행자가 보이면 안 냄(topics.md:40).
        죽은 노드(xm_infer)가 발견 정보에 잠깐 남는 것은 무시."""
        self.spin_until(lambda: not self.other_cmd_pubs(), 2.0)       # 끝난 노드의 발견 정보가 빠지기를 잠깐 기다림
        others = [p for p in self.other_cmd_pubs() if not (p.endswith("/xm_infer") and not self.node_alive())]
        if others:
            print(f"XM_ROUTE_WARN /cmd_vel 에 다른 발행자 {others} — 정지 명령 안 냄", flush=True)
            return False
        pub = self.node.create_publisher(self.Twist, "/cmd_vel", 10)
        for _ in range(5):
            pub.publish(self.Twist())
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
        self.node.destroy_publisher(pub)
        return True

    def close(self):
        try:
            self.node.destroy_node()
        finally:
            self.rclpy.try_shutdown()


def run_live(a):
    ctx = Ctx(a.scenario, a.direction, a.route)
    start = [float(v) for v in (a.start_pose if a.start_pose else ctx.start_pose())]
    if a.start_yaw_deg is not None:
        start[2] = a.start_yaw_deg
    if not a.ckpt:
        raise SystemExit("--ckpt 가 필요")
    method, seed = a.method, int(a.seed)
    if method not in METHODS:
        print(f"XM_ROUTE_WARN method {method!r} 가 제안 목록 {METHODS} 밖(analyze_runs.py:96)", flush=True)
    scene = a.scene or os.environ.get("FD_SCENE") or ctx.SC.get("scene", "")
    scene = os.path.relpath(os.path.abspath(scene), REPO) if os.path.isabs(scene) else scene
    timeout = float(ctx.SC.get("timeout_s", TIMEOUT_S)) if a.timeout_from_yaml else float(a.timeout)
    a.timeout = timeout
    ts0 = datetime.datetime.now().astimezone()
    if not a.out:
        a.out = os.path.join(REPO, "data", "eval", "xm", run_id_of(ts0, method, ctx, seed, a.attempt) + ".json")
    log_csv = os.path.splitext(a.out)[0] + ".xm_infer.csv"
    cmd, env = node_command(a, ctx, start, log_csv)
    info = dict(scenario=a.scenario, direction=ctx.direction, route=ctx.route, segment=[ctx.s0, ctx.s1] if ctx.segment else None,
                start_pose=[round(v, 3) for v in start], goal_station=round(ctx.goal_station, 2), goal_xy=[round(float(v), 3) for v in ctx.R[ctx.i_goal]],
                plan_m=round(ctx.plan_len, 1), limits=[ctx.lim_dev, ctx.lim_stall, ctx.radius], timeout_s=timeout,
                obstacles=[(o["name"], round(o["x"], 2), round(o["y"], 2)) for o in ctx.OBS], scene=scene)
    print("XM_ROUTE_PLAN " + json.dumps(info, ensure_ascii=False), flush=True)
    print("XM_ROUTE_NODE_CMD " + " ".join(shlex.quote(c) for c in cmd), flush=True)
    if a.dry_run:
        return 0
    if a.pose_source != "odom":
        print("XM_ROUTE_WARN pose_source != odom — 정답 위치를 모델 경로 입력에 씀(topics.md:39 평가 규칙 밖, 디버그 전용)", flush=True)

    L = Live(a, ctx)
    rc, harness, start_err = 3, "", None
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)

    def prep_fail(code, msg):
        print(f"XM_ROUTE_{code} {msg}", flush=True)
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump({"result": "PREP_FAIL", "code": code, "msg": msg, "rc": 3, **info}, f, ensure_ascii=False)
        return 3

    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)      # 일괄의 timeout(SIGTERM)에도 노드를 끄고 나감
    try:
        # 1. 준비 확인
        if not L.spin_until(lambda: L.gt_last is not None, a.gt_timeout):         # nav2_route.py:213-217
            return prep_fail("NO_GT", f"/ground_truth/tf 를 {a.gt_timeout:g} s 안에 못 받음")
        L.spin_until(lambda: False, 2.0)                                           # 발견 대기 — teleop_ramp.py:190-192 · node.py:466-469
        if L.other_cmd_pubs():
            return prep_fail("CMD_BUSY", f"/cmd_vel 발행자가 이미 있음 {L.other_cmd_pubs()} — 발행자는 1개(topics.md:40 · :63)")
        if not L.spin_until(lambda: L.odom is not None, 10.0):
            return prep_fail("NO_ODOM", "/chassis/odom 을 못 받음")
        mx, my, myaw = odom_to_map(L.odom[0], L.odom[1], L.odom[2], start)
        _, gx, gy, gth = L.gt_last
        start_err = (round(math.hypot(mx - gx, my - gy), 3), round(math.degrees(abs(_wrap(myaw - gth))), 2))
        print(f"XM_ROUTE_START_CHECK odom+출발 자세 vs 정답 위치 {start_err[0]:.3f} m · {start_err[1]:.1f}°", flush=True)
        if a.pose_source == "odom" and (start_err[0] > START_POS_TOL or start_err[1] > START_YAW_TOL_DEG):
            return prep_fail("NO_START", f"출발 자세 {start} 가 씬과 안 맞음 {start_err} (반시계 바퀴 · 레이어 확인)")
        # 2. 노드
        L.node_log = open(os.path.splitext(a.out)[0] + ".node.log", "w")
        L.proc = subprocess.Popen(cmd, env=env, stdout=L.node_log, stderr=subprocess.STDOUT, start_new_session=True)
        with open(a.out + ".node.pid", "w") as f:                                  # 일괄이 이 파일로 남은 노드를 끔(timeout 124 때)
            f.write(str(L.proc.pid))
        t0 = time.monotonic()
        if not L.spin_until(lambda: L.n_cmd > 0 or not L.node_alive(), a.ready_timeout):
            return prep_fail("NO_NODE", f"노드가 {a.ready_timeout:g} s 안에 /cmd_vel 을 안 냄(모델 로드?)")
        if not L.node_alive():
            return prep_fail("NO_NODE", f"노드가 시작 중 끝남 rc={L.proc.poll()} — {os.path.splitext(a.out)[0]}.node.log")
        print(f"XM_ROUTE_READY 노드 첫 /cmd_vel {time.monotonic() - t0:.1f} s", flush=True)
        # 3. 기록 · 4. 끝내기 판단
        L.recording = True
        if not L.spin_until(lambda: L.ep is not None, 5.0):
            return prep_fail("NO_GT", "기록 시작 뒤 정답 위치 표본이 안 옴")
        last_print = time.monotonic()
        while L.t_end is None:
            L.rclpy.spin_once(L.node, timeout_sec=0.05)
            if not L.node_alive():
                harness = f"노드 종료 rc={L.proc.poll()}"
                L.t_end = L.track[-1][0]
                print(f"XM_ROUTE_END FAILED {harness}", flush=True)
                break
            if time.monotonic() - L.gt_wall > a.gt_silence_s:
                return prep_fail("SIM_STALL", f"정답 위치가 실제 시간 {a.gt_silence_s:g} s 동안 안 옴(시뮬 멈춤?)")
            if time.monotonic() - last_print > 5:
                last_print = time.monotonic()
                t = L.track[-1]
                print(f"  t {t[0] - L.t_goal:.1f} s · 진행 {L.ep.station:.1f}/{ctx.goal_station:.1f} m · 이탈 {t[3]:.2f} m · "
                      f"v {L.cmd[0]:.2f} ω {L.cmd[1]:+.2f}", flush=True)
        name = "FAILED" if harness else L.ep.reason
        # 5. 멈추기 — 노드 끄기 → 이쪽 정지 명령 → 멈출 때까지 표본
        node_rc = L.stop_node()
        L.zero_cmd()
        t_stop = L.track[-1][0]
        L.spin_until(lambda: (L.odom is not None and abs(L.odom[3]) < SETTLE_V and L.track[-1][0] > t_stop)
                     or L.track[-1][0] - L.t_end > SETTLE_SIM_S, 10.0)
        L.recording = False
        wall_s = round(time.time() - L.t0w, 1)
        # 6. 판정 · 기록
        ck_meta = os.path.splitext(log_csv)[0] + ".meta.json"
        nm = json.load(open(ck_meta, encoding="utf-8")) if os.path.isfile(ck_meta) else {}
        mv = model_version(a.ckpt, method, a.model_version)
        meta = dict(run_id=run_id_of(L.ts, method, ctx, seed, a.attempt), timestamp=L.ts.isoformat(timespec="seconds"),
                    repo_rev=repo_rev(), method=method, model_version=mv, scene=scene, scene_version=a.scene_version, seed=seed,
                    log_path=os.path.relpath(os.path.abspath(a.out), REPO), timeout_s=timeout, rc=0, attempt=a.attempt,
                    note="; ".join(x for x in (a.note, f"pose_source={a.pose_source}", f"intervene={a.intervene}",
                                               f"ckpt_sha12={nm['ckpt_sha12']}" if nm.get("ckpt_sha12") else "") if x))
        extra = dict(end_reason=name, end_detail=harness or L.ep.detail, t_end_decision=round(L.t_end, 3),
                     harness_failed=harness, start_pose=info["start_pose"], start_check=start_err, pose_source=a.pose_source,
                     goal_station=info["goal_station"], station_end=round(L.ep.station, 2), ckpt=os.path.abspath(a.ckpt),
                     ckpt_sha12=nm.get("ckpt_sha12"), node_meta=os.path.relpath(ck_meta, REPO) if nm else None,
                     node_csv=os.path.relpath(log_csv, REPO), node_rc=node_rc, node_cmd=cmd, intervene=a.intervene)
        finish(ctx, a, L.track, L.t_goal, name, meta, extra, wall_s=wall_s, harness=harness, t_end=L.t_end)
        rc = 0
        return rc
    except KeyboardInterrupt:
        print("XM_ROUTE_ABORT 신호로 중단", flush=True)
        return 130
    finally:
        try:
            if L.proc is not None and L.proc.poll() is None:
                L.stop_node()
                L.zero_cmd()
        finally:
            if L.node_log:
                L.node_log.close()
            L.close()


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="X-Mobility 폐루프 평가 한 회 — sim/nav2/nav2_route.py 와 같은 판정 (eval/XM_HARNESS.md)")
    ap.add_argument("--scenario", required=True, help="eval/scenarios/<이름>.yaml")
    ap.add_argument("--direction", choices=["cw", "ccw"], help="바퀴 시나리오는 필수(구간은 yaml direction)")
    ap.add_argument("--route", help="바퀴의 기준선 CSV(기본 eval/routes/parking_r3_<방향>.csv = scenario_batch.sh:79)")
    ap.add_argument("--method", default="xmobility_ft", help=f"결과 CSV method — 제안 {METHODS}")
    ap.add_argument("--ckpt", help="파인튜닝 .ckpt (노드 ckpt 파라미터)")
    ap.add_argument("--model-version", help="결과 CSV model_version(기본: ckpt 경로의 xm_ft_v<주>.<부> 등)")
    ap.add_argument("--seed", type=int, default=None, help="결과 CSV seed = 반복 번호, 노드 seed(확산 noise)에도 넘김(기본 0)")
    ap.add_argument("--attempt", type=int, default=1, help="일괄의 시도 번호(1~3)")
    ap.add_argument("--out", help="회 JSON(기본 data/eval/xm/<run_id>.json). 옆에 .node.log · .xm_infer.csv · .meta.json")
    ap.add_argument("--csv", help="결과 CSV(results_schema.md v0) — 있으면 한 행 덧붙임")
    ap.add_argument("--timeout", type=float, default=TIMEOUT_S, help="시뮬 초 — 기본 300 = scenario_batch.sh:89 가 Nav2 에 준 값")
    ap.add_argument("--timeout-from-yaml", action="store_true", help="시나리오 yaml timeout_s 를 씀(Nav2 일괄과 달라짐)")
    ap.add_argument("--intervene", choices=["stop", "off"], default="stop",
                    help="stop: cte 가 한도를 넘으면 그 회를 끝냄(INTERVENED, results_schema.md §3) · off: Nav2 처럼 계속")
    ap.add_argument("--stall-abort-s", type=float, default=0.0, help="이만큼 정지하면 CANCELED 로 끝냄(0 = 끔, Nav2 일괄은 안 끔)")
    ap.add_argument("--goal-stop-s", type=float, default=3.0,
                    help="목표 반경(goal.radius_m) 안에서 이만큼 정지하면 SUCCEEDED 로 끝냄(0 = 끔 — 그러면 목표 앞에서 멈춘 회는 제한 시간까지 감)")
    ap.add_argument("--pose-source", choices=["odom", "ground_truth"], default="odom", help="노드 경로 입력 위치(기본 odom — 평가 규칙)")
    ap.add_argument("--start-pose", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"), help="씬 출발 자세(기본 시나리오에서 계산)")
    ap.add_argument("--start-yaw-deg", type=float, help="출발 방향만 덮어씀(반시계 바퀴를 레이어로 돌려 띄울 때)")
    ap.add_argument("--scene", help="결과 CSV scene(기본 $FD_SCENE 또는 yaml scene)")
    ap.add_argument("--scene-version", help="결과 CSV scene_version(기본 씬 파일 이름에서)")
    ap.add_argument("--node-python", default=os.path.expanduser("~/xmobility_test/venv/bin/python"),
                    help="노드 python — X-Mobility 의존성(gin · torch · lightning · transformers)이 있어야 함(xm_batch.sh 가 먼저 확인)")
    ap.add_argument("--node-cmd", help="노드 명령 앞부분 바꾸기(기본 '<node-python> -u -m fd_xm_infer.node') — 뒤에 --ros-args 가 붙음")
    ap.add_argument("--node-param", action="append", help="노드에 더 넘길 -p 이름:=값 (여러 번)")
    ap.add_argument("--cuda-visible-devices", default="1", help="노드 CUDA_VISIBLE_DEVICES(노드 README:19). 빈칸이면 안 건드림")
    ap.add_argument("--ready-timeout", type=float, default=300.0, help="노드 첫 /cmd_vel 까지 실제 초(모델 로드)")
    ap.add_argument("--gt-timeout", type=float, default=15.0, help="nav2_route.py:213 의 15 s")
    ap.add_argument("--gt-silence-s", type=float, default=30.0, help="주행 중 정답 위치가 이만큼 안 오면 시뮬 멈춤으로 보고 코드 3")
    ap.add_argument("--note", default="", help="결과 CSV note")
    ap.add_argument("--repo-rev", help="(rejudge) repo_rev 지정")
    ap.add_argument("--localization", help="(rejudge, nav2) amcl · gt")
    ap.add_argument("--nav2-map", help="(rejudge, nav2) MANIFEST nav2 행 버전")
    ap.add_argument("--rejudge", help="저장된 회 JSON 을 다시 판정(ROS 안 씀)")
    ap.add_argument("--dry-run", action="store_true", help="노드 명령 · 출발 자세 · 목표 지점만 찍고 끝(ROS 안 씀)")
    a = ap.parse_args(argv)
    if a.cuda_visible_devices == "":
        a.cuda_visible_devices = None
    if a.stall_abort_s and a.stall_abort_s < 10.0:
        print("XM_ROUTE_WARN --stall-abort-s 가 stall_limit_s(10)보다 작으면 no_long_stop 은 참인데 CANCELED 로 끝남", flush=True)
    return a


def main(argv=None):
    a = parse_args(argv)
    if a.rejudge:
        return rejudge(a)
    if a.seed is None:
        a.seed = 0
    return run_live(a)


if __name__ == "__main__":
    sys.exit(main())
