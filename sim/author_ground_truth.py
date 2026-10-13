"""정답 위치 토픽 /ground_truth/tf 를 ros.usda 에 더한다 — 씬 월드 기준 로봇 자세(시뮬이 아는 정답).

ros.usda 는 Proxy · 실제 맵(v0.4b · r3 · r4) 씬 루트가 모두 subLayers 로 쓰므로 한 번 더하면 모든 씬에 들어간다.
/chassis/odom 은 출발 자세 기준이라 지도 좌표로 바꾸려면 출발 자세를 따로 알아야 한다(topics.md 변환식). 무작위 시작 ·
리셋이 들어가면 그 계산이 틀어지기 쉬워, 시뮬이 아는 월드 자세를 그대로 낸다(E 10.13 요청).
  /ground_truth/tf  tf2_msgs/TFMessage — World → chassis_link 변환 하나, 헤더 = 시뮬 시각. /tf 와 섞이지 않게 따로 낸다.
  World 는 씬 월드 = x · y 는 B 맵 좌표(z 만 씬 기준), chassis_link 는 /chassis/odom 의 base_link 와 같은 자리.
**학습 라벨 · 채점용 정답이다. 평가 때 모델 입력(경로 지시 · 경로 점)을 만드는 데 쓰지 않는다**(10.13 멘토: 평가는 센서로 구한 위치로).

레포 루트(= $CD1)에서 (ROS 2 Bridge 가 잡히도록 시스템 Jazzy 를 먼저 source — run_proxy.sh 와 같음):
    source /opt/ros/jazzy/setup.bash && OMNI_KIT_ACCEPT_EULA=YES envs/isaacsim_env/bin/python sim/author_ground_truth.py
이미 /World/ROS_GroundTruth 가 있으면 아무것도 바꾸지 않는다(ros.usda 의 다른 그래프는 건드리지 않음).
"""
import os

from isaacsim import SimulationApp

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "scenes", "proxy")
ROBOT = "/World/Nova_Carter_ROS"
GRAPH = "/World/ROS_GroundTruth"
TOPIC = "ground_truth/tf"

app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0, "multi_gpu": False})

import omni.graph.core as og  # noqa: E402  (SimulationApp 뒤에 import 해야 함)
import omni.usd  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402
from pxr import Sdf, Usd  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
app.update()

ctx = omni.usd.get_context()
ctx.open_stage(os.path.join(SCENE, "proxy_parking.usda"))
while is_stage_loading():
    app.update()
stage = ctx.get_stage()

ros = Sdf.Layer.FindOrOpen(os.path.join(SCENE, "ros.usda"))
if ros.GetPrimAtPath(GRAPH):
    print(f"SKIP — ros.usda 에 {GRAPH} 가 이미 있음")
else:
    stage.SetEditTarget(Usd.EditTarget(ros))
    K = og.Controller.Keys
    og.Controller.edit({"graph_path": GRAPH, "evaluator_name": "execution"}, {
        K.CREATE_NODES: [("tick", "omni.graph.action.OnPlaybackTick"),
                         ("context", "isaacsim.ros2.bridge.ROS2Context"),
                         ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                         ("publish_gt", "isaacsim.ros2.bridge.ROS2PublishTransformTree")],
        K.CONNECT: [("tick.outputs:tick", "publish_gt.inputs:execIn"),
                    ("context.outputs:context", "publish_gt.inputs:context"),
                    ("sim_time.outputs:simulationTime", "publish_gt.inputs:timeStamp")],
        K.SET_VALUES: [("publish_gt.inputs:topicName", TOPIC)],
    })
    node = stage.GetPrimAtPath(GRAPH + "/publish_gt")
    node.CreateRelationship("inputs:parentPrim").SetTargets([Sdf.Path("/World")])
    node.CreateRelationship("inputs:targetPrims").SetTargets([Sdf.Path(ROBOT + "/chassis_link")])
    ros.Save()
    print(f"OK ros.usda — {GRAPH}: /{TOPIC} (World → chassis_link)")

app.close()
