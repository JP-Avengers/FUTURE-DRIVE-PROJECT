"""Proxy 씬을 헤드리스로 열고 Play — 열리는지, 로봇이 어디 섰는지, ROS 그래프가 어떤 토픽을 내는지 확인.

레포 루트(= $CD1)에서, 시스템 ROS 를 먼저 source 한 셸로(sim/README 의 Bridge 검증과 같음):
    OMNI_KIT_ACCEPT_EULA=YES envs/isaacsim_env/bin/python sim/check_proxy_scene.py 20
Play 10초 뒤 `ros2 topic list -t` 결과를 찍는다. 끝에 "OK" 가 찍히면 정상 종료.
"""
import os
import subprocess
import sys
import time

from isaacsim import SimulationApp

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenes", "proxy", "proxy_parking.usda")
app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0, "multi_gpu": False})

import omni.timeline  # noqa: E402  (SimulationApp 뒤에 import 해야 함)
import omni.usd  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

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
topics = []
for p in stage.Traverse():
    if p.GetTypeName() == "OmniGraphNode":
        nt = p.GetAttribute("node:type").Get() or ""
        topic = p.GetAttribute("inputs:topicName")
        if "ros2" in nt.lower() and topic and topic.Get():
            topics.append(f"{nt.split('.')[-1]}:{topic.Get()}")
print("ROSGRAPH", len(topics), sorted(topics), flush=True)

omni.timeline.get_timeline_interface().play()
t0, listed = time.time(), False
while time.time() - t0 < DURATION:
    app.update()
    if not listed and time.time() - t0 > 10:  # 토픽은 Play 후 몇 초 지나야 다 나옴(워밍업 2장)
        listed = True
        try:
            out = subprocess.run(["ros2", "topic", "list", "-t"], capture_output=True, text=True, timeout=20).stdout
            print("TOPICS\n" + out.strip(), flush=True)
        except (OSError, subprocess.TimeoutExpired) as e:
            print("TOPICS 확인 실패 — 다른 터미널에서 ros2 topic list:", e, flush=True)
pose = UsdGeom.Xformable(robot).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
print(f"ROBOT_POSE {pose[0]:.2f} {pose[1]:.2f} {pose[2]:.2f}", flush=True)
omni.timeline.get_timeline_interface().stop()
print("OK", flush=True)
app.close()
