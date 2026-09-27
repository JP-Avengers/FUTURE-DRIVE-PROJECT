"""Isaac Sim ROS 2 Bridge 검증 — 헤드리스로 /clock 을 발행하고, 호스트 ros2 CLI 로 수신 확인.

실행 (server207, 시스템 Jazzy 를 **기동 전에** source — 순서가 바뀌면 내장 라이브러리로 넘어감):
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    OMNI_KIT_ACCEPT_EULA=YES $CD1/envs/isaacsim_env/bin/python sim/check_ros2_bridge.py [초]
다른 터미널:
    ros2 topic echo /clock --once
"""
import sys

from isaacsim import SimulationApp

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0, "multi_gpu": False})

import omni.graph.core as og
import omni.timeline
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
app.update()

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/ClockGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
            ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
        ],
    },
)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
print("BRIDGE_READY", flush=True)

import time
t0 = time.time()
while time.time() - t0 < DURATION:
    app.update()

timeline.stop()
print("BRIDGE_DONE", flush=True)
app.close()
