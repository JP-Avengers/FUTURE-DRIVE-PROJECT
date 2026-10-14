#!/usr/bin/env python3
"""Nav2 교사 다리 — Nav2 가 모는 주행을 자동 교사(sim/teacher.py)와 같은 형식으로 녹화되게 한다(10.13 멘토: Nav2 주행도 교사 데이터).

Nav2 를 nav2.launch.py cmd_vel_out:=nav2/cmd_vel 로 띄우면 최종 명령(collision_monitor 출력)이 /nav2/cmd_vel 로 나온다. 이 노드가
그것을 받아 자동 교사와 같은 토픽을 낸다:
  - /cmd_vel (Twist)               실행 명령 = Nav2 명령(+ --noise-duty > 0 이면 조향 잡음). 받는 즉시 냄 — /cmd_vel 발행자는 이 노드 하나
  - /teacher/cmd_vel (TwistStamped) 라벨 = Nav2 명령(잡음 없음), 헤더 = 시뮬 시각, 시뮬 시간 20 Hz
  - /route_command (UInt8)          0 직진 · 1 좌 · 2 우 — 기준선 기하(teacher_core.turn_windows, 자동 교사와 같은 규칙), 20 Hz
  - /teacher/status (String JSON)   자동 교사와 같은 칸(ph · lap · s · cte · he · cmd · vt · vl · wl · we · nw · x · y)
                                    + src "nav2" · prog(기준선 따라 나아간 m) · od(장애물 가장자리까지 m, --scenario 때) · age(Nav2 명령 나이 s)
위치는 정답 위치 /ground_truth/tf(World = 지도) — 지시 · 상태 계산용이고 모델 입력은 아님(자동 교사는 odom + 시작 자세로 같은 값을 냄).
단계 ph: WAIT(Nav2 명령 전) · FOLLOW(0.5 s 안에 받은 Nav2 명령이 있음 — Nav2 가 모는 중) · STOP(Nav2 명령이 끊김 — 도착 · 취소).
Nav2 명령이 0.5 s(시뮬) 넘게 안 오면 /cmd_vel 0 을 낸다(Nav2 가 죽어도 로봇이 서게). vt 는 Nav2 속도 상한(0.8 — 목표 속도가 따로 없음).
잡음(--noise-duty, 기본 0 = 끔)은 자동 교사의 SteerNoise 를 add 방식으로(Nav2 가 계속 고치는 위에 더함). 출발 뒤 2 m 안, --scenario 의
장애물 가장자리 --noise-clear m 안에서는 새 펄스를 시작하지 않음(피하는 중 잡음으로 부딪히지 않게).

    python3 sim/nav2/nav2_teacher.py --route eval/routes/parking_r3_cw.csv [--scenario eval/scenarios/<이름>.yaml] [--noise-duty 0.3]
--route 는 Nav2 가 따라가는 기준선(nav2_route.py 와 같은 파일 — 점 순서 = 진행 방향). 보통은 sim/nav2/scenario_batch.sh 가 FD_RECORD=1 일 때
녹화기(sim/record_teacher.sh)와 함께 띄운다. 끝(SIGINT · SIGTERM)에 --summary JSON 과 NAV2_TEACHER_EXIT 한 줄.
"""
import argparse
import csv
import json
import math
import os
import random
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from std_msgs.msg import String, UInt8  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

import teacher_core as tc  # noqa: E402

CD1 = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STALE = 0.5   # s(시뮬) — 이보다 오래된 Nav2 명령은 없는 것으로 봄


def obstacles(sc_path):
    """시나리오 장애물 (이름, x, y, 반경) — sim/make_obstacles.py · nav2_route.py 와 같은 자리 계산(기준선 길이 s · 옆 오프셋)."""
    import yaml
    sc = yaml.safe_load(open(sc_path, encoding="utf-8"))
    rows = [(float(r["s"]), float(r["x"]), float(r["y"])) for r in csv.DictReader(open(os.path.join(CD1, sc["route"])))
            if r.get("run", "0") == "0"]
    out = []
    for o in sc.get("obstacles", []):
        s_o = float(o["s"])
        i = max(1, min(len(rows) - 1, next((k for k, r in enumerate(rows) if r[0] >= s_o), len(rows) - 1)))
        (s0, x0, y0), (s1, x1, y1) = rows[i - 1], rows[i]
        f = (s_o - s0) / max(s1 - s0, 1e-9)
        th = math.atan2(y1 - y0, x1 - x0)
        lat = float(o.get("lateral", 0.0))
        r = (math.hypot(float(o["size"][0]), float(o["size"][1])) / 2 if o.get("type", "box") == "box"
             else float(o["size"][0]) / 2)
        out.append((o["name"], x0 + f * (x1 - x0) - lat * math.sin(th), y0 + f * (y1 - y0) + lat * math.cos(th), r))
    return out


class Bridge:
    def __init__(self, node, a, route, obs, noise):
        self.n, self.a, self.r, self.obs, self.noise = node, a, route, obs, noise
        self.windows = tc.turn_windows(route)
        self.pub_cmd = node.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_lab = node.create_publisher(TwistStamped, "/teacher/cmd_vel", 10)
        self.pub_rc = node.create_publisher(UInt8, "/route_command", 10)
        self.pub_st = node.create_publisher(String, "/teacher/status", 10)
        node.create_subscription(Twist, a.nav_topic, self.on_nav, 10)
        node.create_subscription(TFMessage, "/ground_truth/tf", self.on_gt, QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.v = self.w = 0.0            # 최근 Nav2 명령
        self.t_nav = None                # 그 시뮬 시각
        self.nw = 0.0                    # 지금 잡음 ω
        self.pose = None                 # (x, y, yaw) 정답 위치
        self.s_prev, self.prog = None, 0.0
        self.first_pose = None
        self.ph = "WAIT"
        self.t_prev = None
        self.n_nav = self.n_lab = 0
        self.ph_time = {"WAIT": 0.0, "FOLLOW": 0.0, "STOP": 0.0}
        self.cmd_time = {0: 0.0, 1: 0.0, 2: 0.0}
        self.noise_time = 0.0
        self.max_cte = 0.0
        self.last_print = 0.0
        node.create_timer(1.0 / a.rate, self.tick)   # use_sim_time — 시뮬 시간 20 Hz

    def now(self):
        return self.n.get_clock().now().nanoseconds * 1e-9

    def send(self, v, w):
        m = Twist()
        m.linear.x = float(max(-tc.V_MAX, min(tc.V_MAX, v)))
        m.angular.z = float(max(-tc.W_MAX, min(tc.W_MAX, w)))
        self.pub_cmd.publish(m)

    def on_nav(self, m):
        self.v, self.w, self.t_nav = m.linear.x, m.angular.z, self.now()
        self.n_nav += 1
        self.send(self.v, self.w + self.nw)   # 받는 즉시 로봇으로 — 다리가 늦춤을 더하지 않게

    def on_gt(self, m):
        for t in m.transforms:
            if t.header.frame_id == "World" and t.child_frame_id == "nova_carter":
                q = t.transform.rotation
                self.pose = (t.transform.translation.x, t.transform.translation.y,
                             math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
                if self.first_pose is None:
                    self.first_pose = self.pose
                    print(f"GT_FIRST x={self.pose[0]:.3f} y={self.pose[1]:.3f} yaw_deg={math.degrees(self.pose[2]):.1f}", flush=True)

    def tick(self):
        now = self.now()
        dt = 1.0 / self.a.rate if self.t_prev is None else max(0.0, min(0.2, now - self.t_prev))
        self.t_prev = now
        age = None if self.t_nav is None else now - self.t_nav
        fresh = age is not None and age <= STALE
        self.ph = "FOLLOW" if fresh else ("WAIT" if self.t_nav is None else "STOP")
        self.ph_time[self.ph] += dt
        v_lab, w_lab = (self.v, self.w) if fresh else (0.0, 0.0)
        if not fresh:
            self.nw = 0.0
            if self.noise is not None:
                self.noise.t0 = None
            self.send(0.0, 0.0)   # Nav2 명령이 없으면 세움
        if self.pose is None:
            return
        x, y, yaw = self.pose
        s, cte = self.r.project(x, y, hint=self.s_prev)
        if self.s_prev is not None:
            self.prog += (s - self.s_prev + self.r.L / 2) % self.r.L - self.r.L / 2
        self.s_prev = s
        cmd = tc.command_at(self.windows, s, self.r.L)
        od = min((math.hypot(x - ox, y - oy) - orad for _, ox, oy, orad in self.obs), default=None)
        if self.noise is not None and fresh:
            ok = self.prog > 2.0 and (od is None or od > self.a.noise_clear)
            self.nw = self.noise.step(now, dt, enabled=ok)
            if self.noise.active():
                self.noise_time += dt
        if self.ph == "FOLLOW":
            self.cmd_time[cmd] += dt
            self.max_cte = max(self.max_cte, abs(cte))
        we = max(-tc.W_MAX, min(tc.W_MAX, w_lab + self.nw))
        lab = TwistStamped()
        lab.header.stamp = self.n.get_clock().now().to_msg()
        lab.header.frame_id = "base_link"
        lab.twist.linear.x, lab.twist.angular.z = float(v_lab), float(w_lab)
        self.pub_lab.publish(lab)
        self.n_lab += 1
        self.pub_rc.publish(UInt8(data=int(cmd)))
        st = {"t": round(now, 3), "ph": self.ph, "lap": int(self.prog // self.r.L) if self.prog > 0 else 0, "s": round(s, 2),
              "cte": round(cte, 3), "he": round(math.degrees(tc.wrap(self.r.yaw_at(s) - yaw)), 1), "cmd": cmd, "vt": tc.V_MAX,
              "vl": round(v_lab, 3), "wl": round(w_lab, 3), "we": round(we, 3), "nw": round(we - w_lab, 3),
              "x": round(x, 3), "y": round(y, 3), "src": "nav2", "prog": round(self.prog, 2),
              "od": None if od is None else round(od, 2), "age": None if age is None else round(age, 3)}
        self.pub_st.publish(String(data=json.dumps(st, separators=(",", ":"))))
        if time.monotonic() - self.last_print >= 10.0:
            self.last_print = time.monotonic()
            print(f"t_sim={now:8.1f} ph={self.ph:<6} s={s:6.2f} prog={self.prog:6.1f} cte={cte:+.2f} cmd={cmd} v={v_lab:.2f} "
                  f"w={w_lab:+.2f} nw={self.nw:+.2f} od={'-' if od is None else f'{od:.1f}'} nav_msgs={self.n_nav}", flush=True)


def main():
    ap = argparse.ArgumentParser(description="Nav2 교사 다리 — /nav2/cmd_vel → /cmd_vel · /teacher/cmd_vel · /route_command · /teacher/status")
    ap.add_argument("--route", required=True, help="Nav2 가 따라가는 기준선 CSV(점 순서 = 진행 방향)")
    ap.add_argument("--scenario", default=None, help="시나리오 yaml — 장애물 거리(od) · 잡음 금지 구역")
    ap.add_argument("--nav-topic", default="/nav2/cmd_vel")
    ap.add_argument("--noise-duty", type=float, default=0.0, help="잡음이 켜진 시간 비율(0 = 끔)")
    ap.add_argument("--noise-amp", default="0.3,0.8", help="펄스 꼭대기 |Δω| 범위 rad/s(자동 교사와 같음)")
    ap.add_argument("--noise-dur", default="0.5,2.0", help="펄스 길이 범위 s")
    ap.add_argument("--noise-clear", type=float, default=3.0, help="장애물 가장자리에서 이 거리 안이면 새 펄스를 안 냄(m)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--rate", type=float, default=20.0, help="라벨 · 지시 · 상태 Hz(시뮬 시간)")
    ap.add_argument("--summary", default=None, help="끝날 때 요약 JSON 을 쓸 곳")
    a = ap.parse_args()
    path = a.route if os.path.isabs(a.route) else os.path.join(CD1, a.route)
    pts, cut, gap = tc.load_csv_route(path)
    route = tc.Route(pts, name=a.route, step=0.25, smooth=0.0)
    obs = obstacles(a.scenario) if a.scenario else []
    amp = tuple(float(v) for v in a.noise_amp.split(","))
    dur = tuple(float(v) for v in a.noise_dur.split(","))
    noise = tc.SteerNoise(duty=a.noise_duty, dur=dur, amp=amp, rng=random.Random(a.seed + 1000)) if a.noise_duty > 0 else None
    turns = " ".join(f"{'L' if c == tc.LEFT else 'R'}{deg:+.0f}@{apex:.1f}" for _, _, c, deg, apex in tc.turn_windows(route))
    print(f"NAV2_TEACHER route={a.route} L={route.L:.2f} turns=[{turns}] obstacles={[(o[0], round(o[1], 2), round(o[2], 2)) for o in obs]} "
          f"nav_topic={a.nav_topic} noise={a.noise_duty}/{amp}/{dur}/clear {a.noise_clear} m seed={a.seed}", flush=True)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("fd_nav2_teacher", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    br = Bridge(node, a, route, obs, noise)
    stop = {"sig": None}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("sig", "SIGTERM"))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("sig", "SIGINT"))
    wall0 = time.time()
    try:
        while rclpy.ok() and stop["sig"] is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        for _ in range(10):
            br.send(0.0, 0.0)
            time.sleep(0.05)
        follow = br.ph_time["FOLLOW"]
        summ = {"args": vars(a), "route": {"file": a.route, "L": round(route.L, 3), "n": route.n, "cut_at": cut, "close_gap_m": round(gap, 3)},
                "turns": [{"from_s": round(f0, 2), "to_s": round(t0, 2), "cmd": c, "deg": round(d, 1), "apex_s": round(ap_, 2)}
                          for f0, t0, c, d, ap_ in br.windows],
                "obstacles": [{"name": o[0], "x": round(o[1], 3), "y": round(o[2], 3), "r": round(o[3], 3)} for o in obs],
                "first_pose": None if br.first_pose is None else {"x": br.first_pose[0], "y": br.first_pose[1],
                                                                  "yaw_deg": math.degrees(br.first_pose[2])},
                "nav_msgs": br.n_nav, "labels": br.n_lab, "phase_s": {k: round(v, 1) for k, v in br.ph_time.items()},
                "cmd_frac_follow": {str(k): round(v / follow, 3) if follow > 0 else 0.0 for k, v in br.cmd_time.items()},
                "noise_frac_follow": round(br.noise_time / follow, 3) if follow > 0 else 0.0, "prog_m": round(br.prog, 2),
                "max_cte_follow": round(br.max_cte, 3), "stop": stop["sig"], "wall_s": round(time.time() - wall0, 1)}
        if a.summary:
            with open(a.summary, "w", encoding="utf-8") as f:
                json.dump(summ, f, ensure_ascii=False, indent=1)
        print(f"NAV2_TEACHER_EXIT nav_msgs={br.n_nav} labels={br.n_lab} follow_s={follow:.1f} prog={br.prog:.1f} "
              f"noise={summ['noise_frac_follow']} stop={stop['sig']}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
