#!/usr/bin/env python3
"""키보드 조작 + 가감속 램프 — Proxy 씬 1차 수집용: sim/run_proxy.sh teleop [옵션]

키는 목표 (v, ω)만 정하고, /cmd_vel 은 가감속 한도(램프)를 거쳐 20 Hz 로 끊기지 않게 발행한다. 키 하나로 명령이 계단식으로
튀지 않아 교사 데이터의 (v, ω) 라벨이 연속값이 된다(10.5 결정: 게임패드는 보류, 장비 없이 키보드로).
터미널은 키를 뗀 순간을 알려 주지 않으므로, 목표는 누른 값이 유지된다 — 속도는 정속 주행처럼, 조향은 핸들처럼.

  w / ↑   속도 목표 +0.1 m/s (누르고 있으면 초당 +0.5)    0 ~ 8   속도 목표를 0.0 ~ 0.8 m/s 로 바로
  s / ↓   속도 목표 -0.1 m/s                               space   정지 — 속도·조향 목표를 0 으로, 빠르게 감속
  a / ←   왼쪽 조향 ω +0.05 rad/s (누르고 있으면 초당 +1.0)  e       조향을 가운데(ω 0)로
  d / →   오른쪽 조향 ω -0.05 rad/s                        Ctrl+C  끝 — 정지를 보내고 나감

상한은 v 0 ~ 0.8 m/s · |ω| ≤ 1.0 rad/s(규격서 제안값 — E 가 최종값을 정하면 --v-max · --w-max). 가감속 한도는 v 0.5(올릴 때) ·
1.0(내릴 때) m/s², ω 1.5 · 2.0 rad/s², 정지 키는 v 2.0 m/s². 시뮬을 Stop → Play 로 다시 시작하면(odom 시각이 거꾸로 감) 정지로 돌린다.
키 반응이 손에 안 맞으면 --v-step · --v-hold · --w-step · --w-hold(한 번 누를 때 · 누르고 있을 때 초당 변화량), 키 반복이 끊겨 보이면
--repeat-gap 을 키운다(연타가 덜 먹으면 줄인다). 램프는 실제 시간 기준이라 시뮬 RTF 가 1 이 아니면 시뮬 시간으로는 그만큼 빠르거나 느리다(스트리밍 약 0.97).
/cmd_vel 발행자는 이 노드 하나여야 한다(규격서) — 이미 있으면 뜨지 않는다. 시스템 python3 + ROS Jazzy(run_proxy.sh 가 source).
"""
import argparse
import math
import os
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

TOPIC = "/cmd_vel"
ODOM = "/chassis/odom"
REPEAT_GAP = 0.10          # s — 같은 키가 이보다 촘촘히 오면 누르고 있는 것(자동 반복은 0.03~0.09 s), 아니면 새로 누른 것 (--repeat-gap).
#                           0.15 일 때는 0.1~0.15 s 간격 연타가 누르고 있기로 잡혀 덜 올라갔다(10.5 시험: 5연타 0.35)
V_TAP, V_HOLD = 0.1, 0.5   # 속도 목표: 한 번 누를 때 m/s · 누르고 있을 때 초당 m/s (--v-step · --v-hold)
W_TAP, W_HOLD = 0.05, 1.0  # 조향 목표: 한 번 누를 때 rad/s · 누르고 있을 때 초당 rad/s (--w-step · --w-hold)
LETTERS = {b"w": "up", b"s": "down", b"a": "left", b"d": "right", b"e": "center", b" ": "stop",
           b"\r": "center", b"\n": "center"}
ARROWS = {b"A": "up", b"B": "down", b"C": "right", b"D": "left"}


def slew(cur, tgt, acc, dec, dt):
    """cur 를 tgt 쪽으로 가감속 한도(단위/s²)만큼만 움직인다. 0 쪽으로 줄이는 구간은 dec, 키우는 구간은 acc."""
    if cur * tgt < 0:  # 부호가 바뀜 — 먼저 0 까지 줄이고, 남는 시간으로 반대쪽을 키운다
        to_zero = abs(cur) / dec
        if dt <= to_zero:
            return cur - math.copysign(dec * dt, cur)
        cur, dt = 0.0, dt - to_zero
    step = (dec if abs(tgt) < abs(cur) else acc) * dt
    return tgt if abs(tgt - cur) <= step else cur + math.copysign(step, tgt - cur)


def parse(buf):
    """읽은 바이트 → (키 이름 목록, 아직 다 안 온 바이트). 화살표는 ESC [ A~D 또는 ESC O A~D, 그 밖의 ESC 시퀀스는 버린다."""
    names, i, n = [], 0, len(buf)
    while i < n:
        c = buf[i:i + 1]
        if c == b"\x1b":
            if i + 1 >= n:
                break                                 # ESC 뒤가 아직 안 옴
            if buf[i + 1:i + 2] in (b"[", b"O"):
                j = i + 2
                while j < n and not 0x40 <= buf[j] <= 0x7E:  # 매개변수 바이트를 건너뛰어 최종 바이트까지
                    j += 1
                if j >= n:
                    break                             # 시퀀스가 아직 다 안 옴
                if j == i + 2 and buf[j:j + 1] in ARROWS:
                    names.append(ARROWS[buf[j:j + 1]])
                i = j + 1
            else:
                i += 1                                # 그냥 Esc
            continue
        name = LETTERS.get(c.lower()) or (c.decode() if c.isdigit() else None)
        if name:
            names.append(name)
        i += 1
    rest = buf[i:]
    return names, (rest if len(rest) <= 16 else b"")


class Teleop:
    """목표(키로 정함)와 명령(/cmd_vel 로 나감)을 따로 든다. 명령은 update() 가 가감속 한도만큼만 목표를 따라간다."""

    def __init__(self, a):
        self.a = a
        self.v_tgt = self.w_tgt = 0.0
        self.v = self.w = 0.0
        self.braking = False
        self.last = {}  # 키 이름 → 마지막으로 받은 시각

    def bump(self, name, now, tap, hold):
        """같은 키가 연달아 오면(누르고 있음) 지난 이벤트 이후 시간 × hold, 처음 누른 것이면 tap 만큼."""
        gap = now - self.last.get(name, -1e9)
        self.last[name] = now
        return hold * gap if gap < self.a.repeat_gap else tap

    def key(self, name, now):
        a = self.a
        if name != "stop":
            self.braking = False
        if name == "up":
            self.v_tgt = min(a.v_max, self.v_tgt + self.bump(name, now, a.v_step, a.v_hold))
        elif name == "down":
            self.v_tgt = max(a.v_min, self.v_tgt - self.bump(name, now, a.v_step, a.v_hold))
        elif name == "left":
            self.w_tgt = min(a.w_max, self.w_tgt + self.bump(name, now, a.w_step, a.w_hold))
        elif name == "right":
            self.w_tgt = max(-a.w_max, self.w_tgt - self.bump(name, now, a.w_step, a.w_hold))
        elif name == "center":
            self.w_tgt = 0.0
        elif name == "stop":
            self.v_tgt = self.w_tgt = 0.0
            self.braking = True
        elif name.isdigit():
            self.v_tgt = min(a.v_max, max(a.v_min, int(name) * 0.1))

    def update(self, dt):
        """dt 만큼 지난 뒤의 명령 (v, ω)."""
        a = self.a
        self.v = slew(self.v, self.v_tgt, a.v_acc, a.brake if self.braking else a.v_dec, dt)
        self.w = slew(self.w, self.w_tgt, a.w_acc, a.w_dec, dt)
        if self.braking and self.v == 0.0 and self.w == 0.0:
            self.braking = False
        return self.v, self.w

    def restart(self):
        """시뮬이 처음부터 다시 시작 — 남은 목표로 곧장 달려 나가지 않게 전부 0 으로."""
        self.v_tgt = self.w_tgt = self.v = self.w = 0.0
        self.braking = False


def banner(a):
    return (f"키보드 조작 + 가감속 램프 — 끝내려면 Ctrl+C (한글 입력 상태면 영문으로 바꾸세요)\n"
            f"  w ↑  속도 목표 +{a.v_step:g} (누르고 있으면 계속)     s ↓  속도 목표 -{a.v_step:g}\n"
            f"  a ←  왼쪽 조향 +{a.w_step:g} (누르고 있으면 계속)    d →  오른쪽 조향\n"
            f"  e  조향 가운데로    0~8  속도 목표를 0.0~0.8 m/s 로    space  정지(빠르게 감속)\n"
            f"키를 한 번 누르면 그 값이 유지되고, 명령은 가감속 한도를 거쳐 서서히 바뀝니다.\n"
            f"상한 v {a.v_min:.1f} ~ {a.v_max:.1f} m/s · |ω| {a.w_max:.1f} rad/s · "
            f"가감속 v {a.v_acc:g}↑ {a.v_dec:g}↓ m/s² · ω {a.w_acc:g}↑ {a.w_dec:g}↓ rad/s² · {a.rate:g} Hz\n")


def status(tp, odom):
    o = f"odom v {odom['v']:.2f} ω {odom['w']:+.2f}" if odom["v"] is not None else "odom 없음"
    return f"\r목표 v {tp.v_tgt:.2f} ω {tp.w_tgt:+.2f} | 명령 v {tp.v:.2f} ω {tp.w:+.2f} | {o}\x1b[K"


def parse_args():
    p = argparse.ArgumentParser(description="키보드 조작 + 가감속 램프 — /cmd_vel 을 연속값으로 발행 (키는 파일 머리말 참고)")
    p.add_argument("--v-max", type=float, default=0.8, help="선속도 상한 m/s (규격서 제안 0.8)")
    p.add_argument("--v-min", type=float, default=0.0, help="선속도 하한 m/s — 0 이면 후진 없음")
    p.add_argument("--w-max", type=float, default=1.0, help="각속도 상한 rad/s (규격서 제안 1.0)")
    p.add_argument("--v-acc", type=float, default=0.5, help="선속도를 키울 때 가감속 한도 m/s²")
    p.add_argument("--v-dec", type=float, default=1.0, help="선속도를 줄일 때 한도 m/s²")
    p.add_argument("--brake", type=float, default=2.0, help="정지 키로 줄일 때 선속도 한도 m/s²")
    p.add_argument("--w-acc", type=float, default=1.5, help="각속도를 키울 때 한도 rad/s²")
    p.add_argument("--w-dec", type=float, default=2.0, help="각속도를 줄일 때(가운데로) 한도 rad/s²")
    p.add_argument("--rate", type=float, default=20.0, help="발행 주기 Hz (규격서 10 Hz 이상)")
    p.add_argument("--v-step", type=float, default=V_TAP, help="w/s 한 번에 속도 목표가 바뀌는 양 m/s")
    p.add_argument("--v-hold", type=float, default=V_HOLD, help="w/s 를 누르고 있을 때 초당 바뀌는 양 m/s")
    p.add_argument("--w-step", type=float, default=W_TAP, help="a/d 한 번에 조향 목표가 바뀌는 양 rad/s")
    p.add_argument("--w-hold", type=float, default=W_HOLD, help="a/d 를 누르고 있을 때 초당 바뀌는 양 rad/s")
    p.add_argument("--repeat-gap", type=float, default=REPEAT_GAP, help="같은 키가 이 간격(s)보다 촘촘히 오면 누르고 있는 것으로 봄")
    a = p.parse_args()
    if (a.rate < 10 or a.v_min > 0
            or min(a.v_acc, a.v_dec, a.brake, a.w_acc, a.w_dec, a.v_max, a.w_max, a.v_step, a.v_hold, a.w_step, a.w_hold, a.repeat_gap) <= 0):
        p.error("--rate 는 10 이상, 한도 · 상한 · 키 반응 값은 0 보다 크게, --v-min 은 0 이하")
    return a


def main():
    a = parse_args()
    if not sys.stdin.isatty():
        print("키 입력을 받을 터미널이 아님 — ssh 터미널(또는 tmux)에서 실행하세요", file=sys.stderr)
        sys.exit(2)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl+C 에 ROS 가 먼저 닫히면 마지막 정지 명령을 못 보낸다
    node = rclpy.create_node("fd_teleop_ramp")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:  # 발견(discovery) 대기
        rclpy.spin_once(node, timeout_sec=0.1)
    pubs = node.count_publishers(TOPIC)
    if pubs:
        print(f"{TOPIC} 발행자가 이미 {pubs}개 — 발행자는 항상 1개(규격서). 먼저 그쪽을 끄세요", file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(3)
    pub = node.create_publisher(Twist, TOPIC, 10)
    tp = Teleop(a)
    odom = {"v": None, "w": None, "stamp": None}

    def on_odom(m):
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if odom["stamp"] is not None and s < odom["stamp"] - 0.5:  # Stop → Play 로 시뮬 시간이 0 부터 다시 시작
            tp.restart()
            sys.stdout.write("\n시뮬 재시작 감지 — 목표를 0 으로 돌렸습니다\n")
        odom["stamp"], odom["v"], odom["w"] = s, m.twist.twist.linear.x, m.twist.twist.angular.z

    node.create_subscription(Odometry, ODOM, on_odom, qos_profile_sensor_data)
    if not node.count_subscribers(TOPIC):
        print(f"(참고) {TOPIC} 을 받는 쪽이 아직 없음 — 씬이 Play 중인지 확인")

    def hangup(signum, frame):  # ssh 가 끊기거나 kill 돼도 정지를 보내고 나간다
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, hangup)
    signal.signal(signal.SIGHUP, hangup)

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    print(banner(a))
    tty.setcbreak(fd)  # 한 글자씩 · 에코 없음 · Ctrl+C 는 그대로 SIGINT
    period, buf, n = 1.0 / a.rate, b"", 0
    last = nxt = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if select.select([fd], [], [], max(0.0, nxt - now))[0]:
                data = os.read(fd, 256)
                if not data:
                    break  # 터미널이 닫힘
                names, buf = parse(buf + data)
                now = time.monotonic()
                for name in names:
                    tp.key(name, now)
            now = time.monotonic()
            if now < nxt:
                continue
            dt, last = min(now - last, 2 * period), now  # 멈췄다 깨어나도 한 번에 크게 건너뛰지 않는다
            nxt += period
            if nxt < now:
                nxt = now + period
            msg = Twist()
            msg.linear.x, msg.angular.z = tp.update(dt)
            pub.publish(msg)
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0)
            n += 1
            if n % 2 == 0:
                sys.stdout.write(status(tp, odom))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print()
        for _ in range(5):  # 받는 쪽은 마지막 명령을 계속 따르므로 끝날 때 정지를 확실히 보낸다
            pub.publish(Twist())
            time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
