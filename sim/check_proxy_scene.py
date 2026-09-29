"""Proxy 씬을 헤드리스로 열고 Play — 열리는지, 토픽이 나오는지, RTF, (선택) 직진 주행 거리를 확인.

레포 루트(= $CD1)에서, 시스템 ROS 를 먼저 source 한 셸로(sim/README 의 Bridge 검증과 같음):
    OMNI_KIT_ACCEPT_EULA=YES envs/isaacsim_env/bin/python sim/check_proxy_scene.py 25
    OMNI_KIT_ACCEPT_EULA=YES envs/isaacsim_env/bin/python sim/check_proxy_scene.py 25 --drive 0.5
Play 10초 뒤 `ros2 topic list -t` 를 찍는다. RTF 는 /clock 두 번의 차이 ÷ 실제 시간.
--drive v 면 3초부터 /cmd_vel 로 v m/s 직진을 보내고 /chassis/odom 으로 움직인 거리·속도를 잰다
(시작 자세가 서쪽 변이라 26 m 안쪽으로). 끝에 "OK" 면 정상 종료.
물리 결과는 USD 가 아니라 Fabric 에 반영되므로 위치는 USD 로 읽지 않고 ROS 로 읽는다.
"""
import os
import subprocess
import sys
import time

from isaacsim import SimulationApp

args = [a for a in sys.argv[1:] if not a.startswith("--")]
DURATION = float(args[0]) if args else 20.0
DRIVE = float(sys.argv[sys.argv.index("--drive") + 1]) if "--drive" in sys.argv else None
SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenes", "proxy", "proxy_parking.usda")
app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0, "multi_gpu": False})

import omni.timeline  # noqa: E402  (SimulationApp 뒤에 import 해야 함)
import omni.usd  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402
from pxr import Usd  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
app.update()

t0 = time.time()
omni.usd.get_context().open_stage(SCENE)
while is_stage_loading() or time.time() - t0 < 5:
    app.update()
print(f"LOADED {time.time() - t0:.1f}s", flush=True)

stage = omni.usd.get_context().get_stage()
robot = stage.GetPrimAtPath("/World/Nova_Carter_ROS")
print("ROBOT", robot.IsValid() and len(list(Usd.PrimRange(robot))) or 0, "prims", flush=True)


def ros_once(topic, field):
    """토픽 한 개를 받아 {이름: 값} 으로 — 예: /clock 의 clock → {'sec': 12, 'nanosec': 5}."""
    # 기다리는 동안에도 시뮬을 돌린다 — 멈추면 메시지가 안 나와 영영 못 받음
    proc = subprocess.Popen(["ros2", "topic", "echo", "--once", "--field", field, topic],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    t = time.time()
    while proc.poll() is None and time.time() - t < 20:
        app.update()
    if proc.poll() is None:
        proc.kill()
    out = proc.communicate()[0]
    vals = {}
    for line in out.splitlines():
        k, _, v = line.strip().partition(":")
        try:
            vals[k.strip()] = float(v)
        except ValueError:
            pass
    return vals


def sim_now():
    c = ros_once("/clock", "clock")
    return c.get("sec", 0) + c.get("nanosec", 0) * 1e-9


timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(30):  # 그래프가 첫 메시지를 낼 때까지
    app.update()
wall0 = time.time()
listed, pub, sim0, odom0 = False, None, None, None
while time.time() - wall0 < DURATION:
    app.update()
    el = time.time() - wall0
    if sim0 is None and el > 2:
        sim0, wall_s = sim_now(), time.time()
        odom0 = ros_once("/chassis/odom", "pose.pose.position")
    if DRIVE is not None and pub is None and el > 3:
        pub = subprocess.Popen(["ros2", "topic", "pub", "-r", "20", "/cmd_vel", "geometry_msgs/msg/Twist",
                                f"{{linear: {{x: {DRIVE}}}}}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if not listed and el > 10:  # 토픽은 Play 후 몇 초 지나야 다 나옴
        listed = True
        try:
            out = subprocess.run(["ros2", "topic", "list", "-t"], capture_output=True, text=True, timeout=20).stdout
            print("TOPICS\n" + out.strip(), flush=True)
        except (OSError, subprocess.TimeoutExpired) as e:
            print("TOPICS 확인 실패 — 다른 터미널에서 ros2 topic list:", e, flush=True)
sim1, wall_e = sim_now(), time.time()
odom1 = ros_once("/chassis/odom", "pose.pose.position")
twist = ros_once("/chassis/odom", "twist.twist.linear")
if pub:
    pub.terminate()
    subprocess.run(["ros2", "topic", "pub", "--once", "/cmd_vel", "geometry_msgs/msg/Twist", "{}"],
                   capture_output=True, timeout=20)
sim, wall = sim1 - (sim0 or 0.0), wall_e - wall_s
print(f"RTF {sim / wall:.2f} (시뮬 {sim:.1f}s / 실제 {wall:.1f}s)", flush=True)
if odom0 and odom1:
    dx, dy = odom1.get("x", 0) - odom0.get("x", 0), odom1.get("y", 0) - odom0.get("y", 0)
    print(f"ODOM 이동 ({dx:+.2f}, {dy:+.2f}) m = {(dx * dx + dy * dy) ** 0.5:.2f} m · 끝 속도 {twist.get('x', 0):.2f} m/s", flush=True)
timeline.stop()
print("OK", flush=True)
app.close()
