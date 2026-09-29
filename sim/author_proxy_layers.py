"""Proxy 씬의 ros.usda · sensors.usda 를 채운다 — Isaac Sim 을 헤드리스로 띄워 OmniGraph 를 만든다.

레포 루트(= $CD1)에서 (build_proxy_scene.py 를 먼저 돌린 뒤):
    OMNI_KIT_ACCEPT_EULA=YES envs/isaacsim_env/bin/python sim/author_proxy_layers.py [--force]

ros.usda      /World/ROS_Clock — 시뮬 시간을 /clock 으로 발행 (use_sim_time 노드용)
sensors.usda  E2E 전방 카메라 — 핀홀 640×480 을 Hawk 왼쪽 렌즈 자리에 달고 /front_camera/* 로 발행,
              Hawk 스테레오 두 대의 렌더는 끔(어안 투영·16:10 이라 4:3 으로 줄이면 왜곡·눌림, 9.29 비교)
두 레이어는 GUI 로도 고치는 레이어라, 내용이 있으면 --force 없이는 덮어쓰지 않는다.
"""
import os
import sys

from isaacsim import SimulationApp

FORCE = "--force" in sys.argv
HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "scenes", "proxy")
ROBOT = "/World/Nova_Carter_ROS"
HAWK_LEFT = ROBOT + "/chassis_link/sensors/front_hawk/left"
E2E_CAM = HAWK_LEFT + "/e2e_camera"          # Hawk 왼쪽 렌즈와 같은 자세 (바닥에서 약 0.35 m)
CAM = {"width": 640, "height": 480,           # 수행계획서 2.3
       "focal": 18.0, "h_ap": 36.0, "v_ap": 27.0,   # mm — 수평 화각 90°(Hawk 와 같음) · 4:3
       "topic": "front_camera/image_raw", "info": "front_camera/camera_info",
       "frame": "front_camera_optical", "skip": 2}  # 60 Hz 스텝 중 3프레임마다 → 20 Hz 목표

app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0, "multi_gpu": False})

import omni.graph.core as og  # noqa: E402  (SimulationApp 뒤에 import 해야 함)
import omni.usd  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
app.update()


def open_layer(name):
    layer = Sdf.Layer.FindOrOpen(os.path.join(SCENE, name))
    if layer.rootPrims and not FORCE:
        print(f"SKIP {name} — 이미 내용이 있음(GUI 로 고친 것일 수 있음). 덮어쓰려면 --force")
        return None
    layer.Clear() if layer.rootPrims else None
    return layer


ctx = omni.usd.get_context()
ctx.open_stage(os.path.join(SCENE, "proxy_parking.usda"))
while is_stage_loading():
    app.update()
stage = ctx.get_stage()
K = og.Controller.Keys

ros = open_layer("ros.usda")
if ros:
    stage.SetEditTarget(Usd.EditTarget(ros))
    og.Controller.edit({"graph_path": "/World/ROS_Clock", "evaluator_name": "execution"}, {
        K.CREATE_NODES: [("tick", "omni.graph.action.OnPlaybackTick"),
                         ("context", "isaacsim.ros2.bridge.ROS2Context"),
                         ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                         ("publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock")],
        K.CONNECT: [("tick.outputs:tick", "publish_clock.inputs:execIn"),
                    ("context.outputs:context", "publish_clock.inputs:context"),
                    ("sim_time.outputs:simulationTime", "publish_clock.inputs:timeStamp")],
        K.SET_VALUES: [("publish_clock.inputs:topicName", "clock")],
    })
    ros.documentation = "ROS 2 그래프 — sim/author_proxy_layers.py 로 처음 만듦, 이후 GUI 로 고쳐도 됨"
    ros.Save()
    print("OK ros.usda — /World/ROS_Clock")

sen = open_layer("sensors.usda")
if sen:
    stage.SetEditTarget(Usd.EditTarget(sen))
    hawk = stage.GetPrimAtPath(HAWK_LEFT + "/camera_left")
    cam = UsdGeom.Camera.Define(stage, E2E_CAM)
    cam.CreateFocalLengthAttr(CAM["focal"])
    cam.CreateHorizontalApertureAttr(CAM["h_ap"])
    cam.CreateVerticalApertureAttr(CAM["v_ap"])
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 500.0))
    UsdGeom.Xformable(cam).AddTransformOp().Set(UsdGeom.Xformable(hawk).GetLocalTransformation())

    g = "/World/ROS_E2ECamera"
    og.Controller.edit({"graph_path": g, "evaluator_name": "execution"}, {
        K.CREATE_NODES: [("tick", "omni.graph.action.OnPlaybackTick"),
                         ("context", "isaacsim.ros2.bridge.ROS2Context"),
                         ("render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                         ("publish_image", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                         ("publish_info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper")],
        K.CONNECT: [("tick.outputs:tick", "render_product.inputs:execIn"),
                    ("render_product.outputs:execOut", "publish_image.inputs:execIn"),
                    ("render_product.outputs:execOut", "publish_info.inputs:execIn"),
                    ("render_product.outputs:renderProductPath", "publish_image.inputs:renderProductPath"),
                    ("render_product.outputs:renderProductPath", "publish_info.inputs:renderProductPath"),
                    ("context.outputs:context", "publish_image.inputs:context"),
                    ("context.outputs:context", "publish_info.inputs:context")],
        K.SET_VALUES: [("render_product.inputs:width", CAM["width"]),
                       ("render_product.inputs:height", CAM["height"]),
                       ("publish_image.inputs:type", "rgb"),
                       ("publish_image.inputs:topicName", CAM["topic"]),
                       ("publish_image.inputs:frameId", CAM["frame"]),
                       ("publish_image.inputs:frameSkipCount", CAM["skip"]),
                       ("publish_info.inputs:topicName", CAM["info"]),
                       ("publish_info.inputs:frameId", CAM["frame"]),
                       ("publish_info.inputs:frameSkipCount", CAM["skip"])],
    })
    stage.GetPrimAtPath(g + "/render_product").GetRelationship("inputs:cameraPrim").SetTargets([Sdf.Path(E2E_CAM)])

    for side in ("left", "right"):  # Hawk 스테레오 렌더 끄기 — 기록에 안 쓰고 렌더 부하만 늘림
        p = stage.OverridePrim(f"{ROBOT}/front_hawk/{side}_camera_render_product")
        p.CreateAttribute("inputs:enabled", Sdf.ValueTypeNames.Bool).Set(False)
    sen.documentation = ("센서 덮어쓰기 — E2E 카메라 640×480 · Hawk 렌더 끄기. "
                         "sim/author_proxy_layers.py 로 처음 만듦, 이후 GUI 로 고쳐도 됨")
    sen.Save()
    print("OK sensors.usda — E2E 카메라", CAM["width"], "x", CAM["height"], "→ /" + CAM["topic"])

app.close()
