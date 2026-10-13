"""Nav2 띄우기 — 우리 씬(실제 맵 r3 · r4, Nova Carter)에서. 씬은 따로 띄운다: FD_LIDAR=1 로 3D LiDAR 를 켜야 함.

    FD_LIDAR=1 FD_DOMAIN=43 FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/run_proxy.sh headless   # tmux 안에서
    source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 launch sim/nav2/nav2.launch.py map:=$PWD/data/nav2/parking_v0.4b_r3/map.yaml [localization:=amcl|gt]

localization
  amcl (기본) — 3D LiDAR → /scan 으로 지도 위 위치를 추정(실제 로봇과 같은 방식, 10.13 멘토 의견). 초기 자세는 nav2_params.yaml.
  gt          — 추정 없이 정답 map → odom(시작 자세 x0 · y0 · yaw0 고정 변환). /chassis/odom 이 시뮬 정답 기반이라 지도 좌표가 정확함.
                Nav2 자체를 시험할 때만 — 비교 평가에는 amcl.
/scan 은 3D 점군을 2D 로 눌러 만든다 — 앞 ±90°(뒤쪽은 로봇 몸체가 가림), NVIDIA carter_navigation 예제와 같은 설정.
"""
import math
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
BRINGUP = os.path.join(get_package_share_directory("nav2_bringup"), "launch")


def setup(context):
    lc = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    params, mapf, loc = lc("params_file"), lc("map"), lc("localization")
    # 설정 파일의 __FD_NAV2_DIR__ 를 이 폴더 절대 경로로 바꾼 사본을 씀(행동 트리 XML 경로)
    txt = open(params).read().replace("__FD_NAV2_DIR__", HERE)
    params = os.path.join(tempfile.gettempdir(), f"fd_nav2_params_{os.getpid()}.yaml")
    open(params, "w").write(txt)
    actions = [
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node", name="pointcloud_to_laserscan",
             remappings=[("cloud_in", "/front_3d_lidar/lidar_points"), ("scan", "/scan")],
             parameters=[{"target_frame": "front_3d_lidar", "transform_tolerance": 0.01,
                          "min_height": float(lc("scan_min_h")), "max_height": float(lc("scan_max_h")),
                          "angle_min": -math.pi / 2, "angle_max": math.pi / 2, "angle_increment": 0.0087, "scan_time": 0.1,
                          "range_min": 0.05, "range_max": float(lc("scan_range_max")), "use_inf": True, "inf_epsilon": 1.0,
                          "use_sim_time": True}]),
    ]
    if loc == "amcl":
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(BRINGUP, "localization_launch.py")),
                       launch_arguments={"map": mapf, "params_file": params, "use_sim_time": "True", "autostart": "True"}.items()))
    elif loc == "gt":
        x0, y0, yaw0 = float(lc("x0")), float(lc("y0")), math.radians(float(lc("yaw0_deg")))
        actions += [
            Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
                 parameters=[params, {"yaml_filename": mapf, "use_sim_time": True}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
                 parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server"]}]),
            Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom_gt",
                 arguments=["--x", str(x0), "--y", str(y0), "--z", "0", "--yaw", str(yaw0), "--pitch", "0", "--roll", "0",
                            "--frame-id", "map", "--child-frame-id", "odom"]),
        ]
    else:
        raise RuntimeError(f"localization 은 amcl 또는 gt: {loc}")
    if lc("navigation").lower() in ("true", "1"):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(BRINGUP, "navigation_launch.py")),
                       launch_arguments={"params_file": params, "use_sim_time": "True", "autostart": "True"}.items()))
    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Nav2 지도 yaml — sim/nav2/make_map.py 로 만든 data/nav2/<씬>/map.yaml(절대 경로)"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(HERE, "nav2_params.yaml")),
        DeclareLaunchArgument("localization", default_value="amcl", description="amcl | gt"),
        DeclareLaunchArgument("navigation", default_value="true", description="false 면 위치 추정(+ /scan)만 — AMCL 시험용"),
        # /scan 높이 띠(3D LiDAR 프레임 기준 — LiDAR 는 base_link(≈ 바닥) 위 0.526 m). NVIDIA 예제는 -0.4 ~ 1.5 지만, 지도 장애물 띠
        # (바닥 위 0.12 ~ 1.0 m, make_map.py) 안에 들게 줄여야 AMCL 이 안 흐트러짐 — 1.0 m 위 나무 가지 · 지붕, 경사진 바닥이 스캔에만 잡힘(10.13)
        DeclareLaunchArgument("scan_min_h", default_value="-0.30"),   # 바닥 위 약 0.23 m
        DeclareLaunchArgument("scan_max_h", default_value="0.47"),    # 바닥 위 약 1.0 m
        DeclareLaunchArgument("scan_range_max", default_value="25.0"),
        DeclareLaunchArgument("x0", default_value="-1.756"),       # 실제 맵 r3 기준선 출발점(씬 시작 자세)
        DeclareLaunchArgument("y0", default_value="-7.621"),
        DeclareLaunchArgument("yaw0_deg", default_value="-96.0"),
        OpaqueFunction(function=setup),
    ])
