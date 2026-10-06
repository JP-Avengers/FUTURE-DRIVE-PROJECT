#!/usr/bin/env python3
"""루프 중심선 pure pursuit 주행 노드 — 규칙 기반 경로 추종 (시스템 python3 + rclpy, cv_bridge 없음).

/chassis/odom 만 받아 params.yaml 의 루프 중심선 위 lookahead 앞 점을 따라가도록 /cmd_vel 을 20 Hz 로 낸다.
카메라 · LiDAR 는 보지 않으므로 장애물은 피하지 않는다. 9.30 루프 주행 시험(최대 이탈 0.44 m)과 10.6 시연 영상에 쓴 노드.
누적 거리가 루프 길이에 이르면 멈추고 LAP_DONE 을 찍는다. 이탈 3.5 m · 10 s 정체 · odom 끊김 · 시간 초과면 FAIL 후 정지.
ccw 는 출발 전에 제자리 회전으로 남쪽을 본다(씬 시작 자세는 북향).

씬이 Play 된 뒤, 시스템 ROS 를 source 하고 씬과 같은 ROS_DOMAIN_ID(run_proxy.sh 는 42)를 둔 셸에서:
    python3 sim/pursuit.py --dir cw|ccw --v 0.5 --lookahead 2.0
/cmd_vel 발행자는 하나여야 하므로 teleop 과 같이 띄우지 않는다.
"""
import argparse
import collections
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402

from looppath import X0, Y0, Path, odom_to_map, quat_rpy, wrap  # noqa: E402

W_MAX, V_MAX, RATE = 1.0, 0.8, 20.0
STALL_S, STALL_M, OFFROAD_M = 10.0, 0.3, 3.5


class Pursuit:
    def __init__(self, node, a):
        self.n, self.a = node, a
        self.path = Path(a.dir)
        self.v = min(a.v, V_MAX)
        self.pub = node.create_publisher(Twist, "/cmd_vel", 10)
        node.create_subscription(Odometry, "/chassis/odom", self.on_odom, 50)
        self.od = None                   # (stamp, x, y, yaw, v, w) — map 자세
        self.last_stamp = -1.0
        self.nodup = 0
        self.phase, self.t_phase = "WAIT", time.monotonic()
        self.t_start = time.monotonic()
        self.first_printed = False
        self.s_prev = None
        self.cum = 0.0
        self.max_cte = 0.0
        self.hist = collections.deque()
        self.done, self.rc = False, 0
        self.sim_follow0 = self.wall_follow0 = None
        self.last_print = 0.0
        self.last_odom_wall = time.monotonic()
        self.limit = 2.0 * self.path.L / self.v + (60.0 if a.dir == "ccw" else 0.0)
        self.stop_reason = None
        node.create_timer(1.0 / RATE, self.tick)

    def on_odom(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st <= self.last_stamp:
            return                        # 같은 시각 중복 무시
        self.last_stamp = st
        self.last_odom_wall = time.monotonic()
        p, q = m.pose.pose.position, m.pose.pose.orientation
        _, _, yaw = quat_rpy((q.x, q.y, q.z, q.w))
        if not self.first_printed:
            self.first_printed = True
            print(f"ODOM_FIRST stamp={st:.3f} pos=({p.x:.4f},{p.y:.4f},{p.z:.4f}) yaw_deg={math.degrees(yaw):.2f}",
                  flush=True)
            self.odom_first_yaw = yaw
        x, y, ps = odom_to_map(p.x, p.y, yaw)
        self.od = (st, x, y, ps, m.twist.twist.linear.x, m.twist.twist.angular.z)

    def send(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.pub.publish(m)
        self.cmd = (v, w)

    def go(self, ph):
        self.phase, self.t_phase = ph, time.monotonic()

    def fail(self, why):
        print(f"FAIL {why} wall={time.time():.3f} sim={self.od[0] if self.od else -1:.3f} cum={self.cum:.2f}", flush=True)
        self.stop_reason, self.rc = why, 2
        self.go("STOP")

    def start_follow(self):
        _, x, y, ps, _, _ = self.od
        self.s_prev, cte = self.path.project(x, y)
        self.s0 = self.s_prev
        self.sim_follow0, self.wall_follow0 = self.od[0], time.time()
        print(f"LAP_START wall={self.wall_follow0:.3f} sim={self.sim_follow0:.3f} s0={self.s0:.3f} cte0={cte:.3f} "
              f"pos=({x:.3f},{y:.3f}) yaw_deg={math.degrees(ps):.1f} dir={self.a.dir} v={self.v} lookahead={self.a.lookahead} "
              f"L={self.path.L:.3f}", flush=True)
        self.go("FOLLOW")

    def tick(self):
        now = time.monotonic()
        if self.phase == "WAIT":
            self.send(0, 0)
            ready = self.pub.get_subscription_count() >= 2 and self.od is not None
            if now - self.t_phase >= 2.0 and (ready or now - self.t_phase >= 15.0):
                print(f"DISCOVERY cmd_vel 구독자={self.pub.get_subscription_count()} 대기 {now - self.t_phase:.2f}s "
                      f"odom_ok={self.od is not None}", flush=True)
                if self.od is None:
                    return self.fail("NO_ODOM")
                if abs(math.degrees(self.odom_first_yaw)) > 45:
                    return self.fail("ODOM_YAW_ABSOLUTE(첫 yaw 가 0 근처가 아님 — 변환식 재검토)")
                if self.a.dir == "ccw":
                    self.go("SPIN")
                else:
                    self.start_follow()
            return
        if self.phase == "STOP":
            self.send(0, 0)
            if now - self.t_phase >= 1.0:
                if self.rc == 0:
                    print(f"LAP_DONE wall={time.time():.3f} sim={self.od[0]:.3f} cum={self.cum:.2f} max_cte={self.max_cte:.3f}",
                          flush=True)
                self.done = True
            return
        if self.od is None or now - self.last_odom_wall > 20.0:
            return self.fail("NO_ODOM_STREAM") if self.phase != "STOP" else None
        st, x, y, ps, vo, wo = self.od
        if self.phase == "SPIN":
            e = wrap(math.radians(-82.7) - ps)
            if abs(e) < math.radians(2.0):
                self.send(0, 0)
                print(f"SPIN_DONE sim={st:.3f} yaw_deg={math.degrees(ps):.1f}", flush=True)
                self.start_follow()
                return
            self.send(0.0, math.copysign(max(0.1, min(0.5, 1.0 * abs(e))), e))
            if now - self.t_phase > 60:
                self.fail("SPIN_TIMEOUT")
            return
        # FOLLOW
        s, cte = self.path.project(x, y, self.s_prev)
        self.cum += (s - self.s_prev + self.path.L / 2) % self.path.L - self.path.L / 2
        self.s_prev = s
        self.max_cte = max(self.max_cte, cte)
        if cte > OFFROAD_M:
            return self.fail("OFFROAD")
        if st - self.sim_follow0 > self.limit:
            return self.fail("TIMEOUT")
        self.hist.append((st, x, y))
        while len(self.hist) > 1 and self.hist[1][0] <= st - STALL_S:
            self.hist.popleft()
        if st - self.hist[0][0] >= STALL_S and math.hypot(x - self.hist[0][1], y - self.hist[0][2]) < STALL_M:
            return self.fail("STALL")
        if self.cum >= self.path.L:
            print(f"LAP_END wall={time.time():.3f} sim={st:.3f}", flush=True)
            self.go("STOP")
            self.send(0, 0)
            return
        tx, ty = self.path.point_at(s + self.a.lookahead)
        alpha = wrap(math.atan2(ty - y, tx - x) - ps)
        d = max(0.5, math.hypot(tx - x, ty - y))
        w = max(-W_MAX, min(W_MAX, self.v * 2.0 * math.sin(alpha) / d))
        self.send(self.v, w)
        if now - self.last_print >= 5.0:
            self.last_print = now
            print(f"t_sim={st - self.sim_follow0:7.1f} s={s:7.2f} cum={self.cum:7.2f} cte={cte:5.2f} "
                  f"pos=({x:7.2f},{y:7.2f}) yaw={math.degrees(ps):7.1f} v_odom={vo:5.2f} w_cmd={w:+.2f}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", choices=["cw", "ccw"], required=True)
    ap.add_argument("--v", type=float, default=0.5)
    ap.add_argument("--lookahead", type=float, default=2.0)
    a = ap.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("fd_loop_pursuit")
    p = Pursuit(node, a)
    print(f"PURSUIT dir={a.dir} v={p.v} lookahead={a.lookahead} L={p.path.L:.3f} limit_sim={p.limit:.0f}s", flush=True)
    try:
        while rclpy.ok() and not p.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("중단 신호 — 정지 명령", flush=True)
        p.rc = 130
    finally:
        for _ in range(10):
            p.send(0, 0)
            time.sleep(0.05)
        print(f"PURSUIT_EXIT rc={p.rc} reason={p.stop_reason}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(p.rc)


if __name__ == "__main__":
    main()
