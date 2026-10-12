#!/usr/bin/env python3
"""자동 교사 — 정답 자세(odom → 맵)와 기준 경로로 명령을 내는 규칙 노드. 교사 데이터 무인 수집용 (시스템 python3 + rclpy).

10.12 검토 결과 1차 교사 데이터는 사람 조작 대신 이 노드 위주로 모으는 안(사람 조작은 비교용). sim/pursuit.py 와 같은 조향식에
수집용 기능을 더했다:
  - 라벨 /teacher/cmd_vel (TwistStamped, 헤더 = 시뮬 시각) — 교사의 깨끗한 명령. 학습 라벨은 이것을 쓴다
  - 실행 /cmd_vel (Twist) = 라벨 + 조향 잡음(이탈 복구 장면). --noise-mode hold(기본)는 직선 구간 펄스 동안 교정을 멈춰 더 크게
    벗어나게 하고, add 는 교사가 계속 고치는 위에 더한다. 잡음은 시간의 --noise-duty 만큼, 0.5~2 s 삼각 펄스(CILRS 수집 방식)
  - /route_command (UInt8: 0 직진 · 1 좌 · 2 우) — 경로 기하로 정함(40° 넘는 꺾임의 꼭짓점 앞 5 m ~ 뒤 2 m, eval/routes 초안 규칙)
  - /teacher/status (String, JSON 한 줄) — 단계 · 바퀴 · 진행 s · 이탈 cte · 방향 오차 · 지시 · 목표 속도 · 잡음 (학습 때 거르기용)
  - 상한 v 0.8 · |ω| 0.7(규격서 — 사람 조작 · Nav2 와 같음), 가감속 한도는 키보드 램프와 같음, 시뮬 시간 20 Hz
  - 곡률 · 회전 구간(--v-turn) 감속, 속도 목표 무작위(--v-range, 약 --v-change m 마다), 바퀴마다 출발점에서 감속 정지 후 --dwell s
  - --label-only: /cmd_vel 을 내지 않고 라벨 · 지시만 — 다른 노드(학습 모델)가 몰 때 그 자리 라벨을 남긴다(DAgger)
경로: proxy (Proxy 루프, sim/scenes/proxy/params.yaml) 또는 csv:<파일>[@회차] (C 의 route_*.csv, 회차 기본 0). ccw 는 같은 출발점에서
거꾸로 돈다(출발 방향이 20° 넘게 다르면 제자리 회전부터). CSV 경로는 잔떨림을 라벨에 넣지 않으려고 앞뒤 1 m 이동 평균(--smooth).
시작 자세(odom → 맵 변환)는 FD_SCENE(run_proxy.sh 와 같은 변수) 루트의 Nova_Carter_ROS 덮어쓰기 → 같은 폴더 robot.usda →
Proxy robot.usda 순으로 읽는다(--start 로 직접). 카메라 · LiDAR 는 보지 않으므로 장애물은 피하지 않는다.

  python3 sim/teacher.py --route proxy --dir cw --laps 2 --seed 1
  FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda python3 sim/teacher.py \\
      --route csv:data/usd/parking_v0.4b_r3/route_bonnet_cw.csv --dir ccw --laps 1
보통은 sim/auto_teacher.sh 가 녹화기와 함께 띄운다. /cmd_vel 발행자는 하나여야 하므로 teleop · pursuit 와 같이 띄우지 않는다.
끝 줄 TEACHER_EXIT rc=0 이면 정한 바퀴를 다 돌고 멈춘 것. 실패(이탈 3 m · 10 s 정체 · 시간 초과 · odom 끊김 · 시뮬 재시작)는 rc=2.
"""
import argparse
import json
import math
import os
import random
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from std_msgs.msg import String, UInt8  # noqa: E402

import looppath  # noqa: E402
import teacher_core as tc  # noqa: E402

CD1 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROXY_ROBOT = os.path.join(CD1, "sim", "scenes", "proxy", "robot.usda")


def build_route(spec, direction, step, smooth):
    """spec = 'proxy' | 'csv:<파일>[@회차]'. (Route, 설명 dict)"""
    if spec == "proxy":
        pts = [(x, y) for _, x, y in looppath.load_loop()]
        info = {"source": "proxy", "file": os.path.relpath(looppath.PARAMS, CD1)}
        sm = 0.0 if smooth is None else smooth
    elif spec.startswith("csv:"):
        path, _, run = spec[4:].partition("@")
        path = path if os.path.isabs(path) else os.path.join(CD1, path)
        pts, k, gap = tc.load_csv_route(path, int(run or 0))
        info = {"source": "csv", "file": os.path.relpath(path, CD1), "run": int(run or 0), "cut_at": k,
                "close_gap_m": round(gap, 3)}
        sm = 1.0 if smooth is None else smooth
    else:
        raise SystemExit(f"--route 는 proxy 또는 csv:<파일>[@회차] — 받은 값: {spec}")
    r = tc.Route(pts, name=spec, step=step, smooth=sm)
    if direction == "ccw":
        r = r.reversed()
    info.update(direction=direction, L=round(r.L, 3), n=r.n, step=step, smooth=sm)
    return r, info


def find_start_pose(a):
    if a.start:
        x, y, yd = (float(v) for v in a.start.split(","))
        return (x, y, math.radians(yd)), "--start"
    scene = os.environ.get("FD_SCENE", "")
    cands = []
    if scene:
        scene = scene if os.path.isabs(scene) else os.path.join(CD1, scene)
        cands += [scene, os.path.join(os.path.dirname(scene), "robot.usda")]
    cands.append(PROXY_ROBOT)
    for c in cands:
        p = tc.start_pose_from_usda(c)
        if p is not None:
            return p, os.path.relpath(c, CD1)
    return (looppath.X0, looppath.Y0, looppath.PSI0), "sim/looppath.py"


class TeacherNode:
    def __init__(self, node, a, core, pose0):
        self.n, self.a, self.core, self.pose0 = node, a, core, pose0
        self.pub_cmd = None if a.label_only else node.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_lab = node.create_publisher(TwistStamped, "/teacher/cmd_vel", 10)
        self.pub_rc = node.create_publisher(UInt8, "/route_command", 10)
        self.pub_st = node.create_publisher(String, "/teacher/status", 10)
        node.create_subscription(Odometry, "/chassis/odom", self.on_odom, 50)
        self.od, self.last_stamp, self.first = None, -1.0, None
        self.last_odom_wall = time.monotonic()
        self.state, self.t_wait = "WAIT", time.monotonic()
        self.t_prev, self.t_run0, self.wall_run0 = None, None, None
        self.done, self.rc, self.reason = False, 0, None
        self.laps = []
        self.last_print = 0.0
        node.create_timer(1.0 / a.rate, self.tick)  # use_sim_time — 시뮬 시간 20 Hz

    def fail(self, why):
        if self.state != "END":
            print(f"FAIL {why} sim={self.od[0] if self.od else -1:.3f} wall={time.time():.3f}", flush=True)
        self.rc, self.reason, self.state, self.done = 2, why, "END", True

    def on_odom(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.od is not None and st < self.last_stamp - 0.5:
            return self.fail("SIM_RESET(odom 시각이 거꾸로 감 — Stop → Play)")
        if st <= self.last_stamp:
            return  # 같은 시각 중복(규격서 주의 25~29 %)
        self.last_stamp, self.last_odom_wall = st, time.monotonic()
        p, q = m.pose.pose.position, m.pose.pose.orientation
        _, _, oyaw = looppath.quat_rpy((q.x, q.y, q.z, q.w))
        if self.first is None:
            self.first = (st, p.x, p.y, oyaw)
            print(f"ODOM_FIRST stamp={st:.3f} odom=({p.x:.3f},{p.y:.3f}) yaw_deg={math.degrees(oyaw):.1f}", flush=True)
        x, y, yaw = tc.odom_to_map(self.pose0, p.x, p.y, oyaw)
        self.od = (st, x, y, yaw, m.twist.twist.linear.x, m.twist.twist.angular.z)

    def send(self, v, w):
        if self.pub_cmd is not None:
            m = Twist()
            m.linear.x, m.angular.z = float(v), float(w)
            self.pub_cmd.publish(m)

    def tick(self):
        now = self.n.get_clock().now().nanoseconds * 1e-9
        wall = time.monotonic()
        if self.state == "WAIT":
            self.send(0.0, 0.0)
            ready = self.od is not None and (self.pub_cmd is None or self.pub_cmd.get_subscription_count() >= 1)
            if wall - self.t_wait >= 2.0 and (ready or wall - self.t_wait >= 15.0):
                subs = -1 if self.pub_cmd is None else self.pub_cmd.get_subscription_count()
                print(f"DISCOVERY cmd_vel 구독자={subs} 대기 {wall - self.t_wait:.1f}s odom_ok={self.od is not None}", flush=True)
                if self.od is None:
                    return self.fail("NO_ODOM")
                self.state, self.t_run0, self.wall_run0 = "RUN", now, time.time()
            return
        if self.state != "RUN":
            return
        dt = 1.0 / self.a.rate if self.t_prev is None else max(0.0, min(0.2, now - self.t_prev))
        self.t_prev = now
        st, x, y, yaw, vx, _ = self.od
        o = self.core.step(now, dt, x, y, yaw, vx)
        self.send(o["v_exe"], o["w_exe"])
        lab = TwistStamped()
        lab.header.stamp = self.n.get_clock().now().to_msg()
        lab.header.frame_id = "base_link"
        lab.twist.linear.x, lab.twist.angular.z = float(o["v_lab"]), float(o["w_lab"])
        self.pub_lab.publish(lab)
        self.pub_rc.publish(UInt8(data=int(o["cmd"])))
        self.pub_st.publish(String(data=json.dumps({
            "t": round(now, 3), "ph": o["phase"], "lap": o["lap"], "s": round(o["s"], 2), "cte": round(o["cte"], 3),
            "he": round(math.degrees(o["he"]), 1), "cmd": o["cmd"], "vt": round(o["v_tgt"], 3),
            "vl": round(o["v_lab"], 3), "wl": round(o["w_lab"], 3), "we": round(o["w_exe"], 3), "nw": round(o["nw"], 3),
            "x": round(x, 3), "y": round(y, 3)}, separators=(",", ":"))))
        ev = o["event"]
        if ev == "LAP_START":
            print(f"LAP_START lap={o['lap'] + 1} sim={now:.3f} s={o['s']:.2f} cte={o['cte']:+.2f} v_tgt={o['v_tgt']:.2f}", flush=True)
        elif ev == "LAP_END":
            ls = o["lap_stats"]
            n = max(1, ls["n"])
            row = {"lap": ls["lap"], "t_sim": round(ls["t"], 2), "follow_s": round(ls["follow_s"], 2),
                   "mean_cte": round(ls["cte_sum"] / n, 3), "max_cte": round(ls["cte_max"], 3),
                   "noise_frac": round(ls["noise_s"] / max(ls["follow_s"], 1e-9), 3),
                   "recover_frac": round(ls["recover_s"] / max(ls["follow_s"], 1e-9), 3),
                   "turn_frac": round(ls["turn_s"] / max(ls["follow_s"], 1e-9), 3), "v_mean": round(ls["v_sum"] / n, 3)}
            self.laps.append(row)
            print("LAP_END " + " ".join(f"{k}={v}" for k, v in row.items()), flush=True)
        elif ev == "DONE":
            print(f"DONE laps={o['lap']} sim={now:.3f}", flush=True)
            self.state, self.done = "END", True
        elif ev and ev.startswith("FAIL:"):
            self.fail(ev[5:])
        if wall - self.last_print >= 10.0:
            self.last_print = wall
            print(f"t_sim={now - self.t_run0:7.1f} ph={o['phase']:<6} lap={o['lap']} s={o['s']:7.2f} cte={o['cte']:+.2f} "
                  f"cmd={o['cmd']} v={o['v_lab']:.2f} w={o['w_lab']:+.2f} nw={o['nw']:+.2f}", flush=True)


def pair(text, name):
    try:
        a, b = (float(v) for v in text.split(","))
    except ValueError:
        raise SystemExit(f"{name} 는 '작은값,큰값' — 받은 값: {text}")
    if a > b:
        raise SystemExit(f"{name}: {a} > {b}")
    return a, b


def main():
    ap = argparse.ArgumentParser(description="자동 교사 — 기준 경로 추종 + 잡음 주입, 라벨은 /teacher/cmd_vel")
    ap.add_argument("--route", default="proxy", help="proxy | csv:<파일>[@회차]")
    ap.add_argument("--dir", choices=["cw", "ccw"], default="cw")
    ap.add_argument("--laps", type=int, default=1, help="바퀴 수 (0 = 끌 때까지)")
    ap.add_argument("--v-range", default="0.5,0.8", help="속도 목표 범위 m/s — 바퀴 시작 · 약 --v-change m 마다 새로 뽑음")
    ap.add_argument("--v-change", type=float, default=30.0)
    ap.add_argument("--v-turn", type=float, default=0.5, help="회전 구간(지시 좌 · 우) 속도 목표 상한")
    ap.add_argument("--lookahead", type=float, default=2.0)
    ap.add_argument("--noise-duty", type=float, default=0.3, help="잡음이 켜진 시간 비율 (0 = 잡음 없음)")
    ap.add_argument("--noise-amp", default="0.3,0.6", help="펄스 꼭대기 |Δω| 범위 rad/s")
    ap.add_argument("--noise-dur", default="0.5,2.0", help="펄스 길이 범위 s")
    ap.add_argument("--noise-mode", choices=["hold", "add"], default="hold")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-stop", action="store_true", help="바퀴 끝에 멈추지 않고 이어 돎")
    ap.add_argument("--dwell", type=float, default=2.0, help="바퀴 끝 정지 뒤 대기 s(시뮬)")
    ap.add_argument("--label-only", action="store_true", help="/cmd_vel 을 내지 않음 — 다른 노드가 몰 때 라벨만(DAgger)")
    ap.add_argument("--rate", type=float, default=20.0, help="Hz (시뮬 시간)")
    ap.add_argument("--start", default=None, help="x,y,yaw_deg — 씬에서 읽는 대신")
    ap.add_argument("--step", type=float, default=0.25, help="경로 재표본 간격 m")
    ap.add_argument("--smooth", type=float, default=None, help="경로 이동 평균 반폭 m (기본: proxy 0 · csv 1.0)")
    ap.add_argument("--summary", default=None, help="끝날 때 요약 JSON 을 쓸 곳")
    a = ap.parse_args()
    v_range, amp, dur = pair(a.v_range, "--v-range"), pair(a.noise_amp, "--noise-amp"), pair(a.noise_dur, "--noise-dur")
    route, info = build_route(a.route, a.dir, a.step, a.smooth)
    pose0, pose_src = find_start_pose(a)
    rng = random.Random(a.seed)
    noise = tc.SteerNoise(duty=a.noise_duty, dur=dur, amp=amp, rng=random.Random(a.seed + 1000)) if a.noise_duty > 0 else None
    core = tc.TeacherCore(route, laps=a.laps, v_range=v_range, v_change_m=a.v_change, v_turn=a.v_turn,
                          lookahead=a.lookahead, noise=noise, noise_mode=a.noise_mode, rng=rng,
                          stop_each_lap=not a.no_stop, dwell=a.dwell, label_only=a.label_only)
    turns = " ".join(f"{'L' if c == tc.LEFT else 'R'}{deg:+.0f}@{apex:.1f}" for _, _, c, deg, apex in core.windows)
    print(f"TEACHER route={a.route} dir={a.dir} L={route.L:.2f} n={route.n} smooth={info['smooth']} turns=[{turns}] "
          f"start=({pose0[0]:.3f},{pose0[1]:.3f},{math.degrees(pose0[2]):.1f}°) from {pose_src} laps={a.laps} "
          f"v={v_range} v_turn={a.v_turn} noise={a.noise_duty}/{a.noise_mode}/{amp}/{dur} seed={a.seed} "
          f"label_only={a.label_only}", flush=True)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("fd_teacher", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    tn = TeacherNode(node, a, core, pose0)
    stop = {"sig": None}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("sig", "SIGTERM"))
    try:
        while rclpy.ok() and not tn.done and stop["sig"] is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if tn.state == "RUN" and time.monotonic() - tn.last_odom_wall > 20.0:
                tn.fail("NO_ODOM_STREAM(실제 시간 20 s 동안 odom 없음 — 시뮬 멈춤?)")
            elif tn.state == "WAIT" and time.monotonic() - tn.t_wait > 60.0:
                tn.fail("NO_CLOCK(60 s 동안 시뮬 시간이 안 감 — 씬이 Play 중인지, 도메인이 같은지)")
        if stop["sig"]:
            print(f"중단 신호 {stop['sig']} — 정지 명령", flush=True)
            tn.rc, tn.reason = 130, stop["sig"]
    except KeyboardInterrupt:
        print("중단 신호 — 정지 명령", flush=True)
        tn.rc, tn.reason = 130, "SIGINT"
    finally:
        for _ in range(10):
            tn.send(0.0, 0.0)
            time.sleep(0.05)
        if a.summary:
            with open(a.summary, "w", encoding="utf-8") as f:
                json.dump({"args": vars(a), "route": info, "turns": [
                    {"from_s": round(f0, 2), "to_s": round(t0, 2), "cmd": c, "deg": round(d, 1), "apex_s": round(ap_, 2)}
                    for f0, t0, c, d, ap_ in core.windows], "start_pose": {"x": pose0[0], "y": pose0[1],
                    "yaw_deg": math.degrees(pose0[2]), "from": pose_src}, "laps": tn.laps, "rc": tn.rc,
                    "reason": tn.reason, "sim_start": tn.t_run0, "wall_start": tn.wall_run0, "wall_end": time.time()},
                    f, ensure_ascii=False, indent=1)
        print(f"TEACHER_EXIT rc={tn.rc} reason={tn.reason} laps_done={len(tn.laps)}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(tn.rc)


if __name__ == "__main__":
    main()
