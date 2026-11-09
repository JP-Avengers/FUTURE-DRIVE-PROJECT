"""X-Mobility 추론 노드(fd_xm_infer) 실행 — ml_env python 으로 띄운다(docs/server207.md:27).

  ros2 launch fd_xm_infer xm_infer.launch.py \
      ckpt:=$CD1/data/checkpoints/xm_ft_v0.1_aw30/checkpoints/xm_ft_v0.1_aw30_e8.ckpt
    ckpt = best 파일(docs/interfaces/e2d_model.md 버전 칸 · MANIFEST sha 로 확인). last.ckpt 는 마지막 에폭이라 동결 모델이 아님.
    p3 계열 ckpt(경로 라벨 ×1/3)면 path_scale:=0.3333 — ~/path 디버그만 영향, 명령은 무관
  기준선 CSV 로 경로를 직접 만들 때: route_source:=csv pose_source:=odom start_yaw_deg:=-96.0 (기본 기준선 parking_r3_cw.csv)
    start_x · start_y · start_yaw_deg 를 안 주면 CSV 옆 YAML start — parking_r3_ccw.yaml 은 84° 라 씬 그대로(-96°,
    parking_v0.4b_r3.usda:27) 띄운 로봇에 반시계 CSV 를 쓰면 odom→지도 변환이 180° 틀림 → 그때는 start_yaw_deg:=-96.0 필수
    (README "실행")

GPU: Isaac Sim 이 GPU 0 을 쓰므로 CUDA_VISIBLE_DEVICES=1 을 붙이고 device:=cuda 로 두거나 device:=cuda:1(docs/server207.md:26).
여기 없는 파라미터는 node.py 의 XmInfer.__init__ 참고(런치 인자로 더 넘기려면 아래 ARGS 에 한 줄 추가).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CD1 = os.environ.get('CD1', os.path.expanduser('~/Capstone_Design_1'))

# (이름, 기본값, 설명, 문자열로 넘길지) — 문자열 파라미터는 ParameterValue(str) 로 넘겨 '' · 숫자처럼 보이는 경로도 그대로 받음
ARGS = [
    ('xm_repo', os.environ.get('XM_REPO', os.path.expanduser('~/xmobility_test/X-Mobility')),
     'X-Mobility 레포(5837aea)', True),
    ('gin', os.path.join(CD1, 'e2e', 'configs', 'finetune_parking_r3.gin'), '학습 gin', True),
    ('gin_bindings', '', 'gin 바인딩 덧붙이기(; 구분)', True),
    ('ckpt', '', '파인튜닝 .ckpt (필수)', True),
    ('device', 'cuda', 'cuda · cuda:1 · cpu', True),
    ('rate_hz', '5.0', '추론 주기 Hz(학습 5 Hz)', False),
    ('image_topic', '/front_camera/image_raw', '', True),
    ('odom_topic', '/chassis/odom', '', True),
    ('route_topic', '/route', '', True),
    ('cmd_topic', '/cmd_vel', '', True),
    ('route_source', 'topic', 'topic(/route) · csv(기준선 CSV + 위치)', True),
    ('route_csv', os.path.join(CD1, 'eval', 'routes', 'parking_r3_cw.csv'), 'csv 모드 기준선', True),
    ('goal_s', '-1.0', 'csv 모드 구간 목표 s(m), 음수 = 계속', False),
    ('pose_source', 'odom', 'csv 모드 위치 — odom(odom+시작 자세) · ground_truth(디버그 전용)', True),
    ('start_x', 'nan', 'csv·odom 모드 시작 자세 x(m) — nan = CSV 옆 YAML start(node.py 가 float() 로 받음)', True),
    ('start_y', 'nan', '', True),
    ('start_yaw_deg', 'nan', '씬 루트 출발 방향(r3 = -96.0) — 반시계 CSV 도 씬 그대로 띄웠으면 -96(YAML 은 84 라 180° 틀림)', True),
    ('jpeg_quality', '90', '학습과 같은 JPEG 왕복(0 = 끔)', False),
    ('v_max', '0.8', 'topics.md:61', False),
    ('w_max', '0.7', 'topics.md:61', False),
    ('stale_timeout_s', '0.5', '영상 · odom 끊김 판정(s)', False),
    ('route_timeout_s', '1.0', '경로 · 위치 끊김 판정(s)', False),
    ('publish_path', 'true', '~/path · ~/route 디버그 토픽', False),
    ('path_scale', '1.0', 'p3 계열 ckpt(경로 라벨 ×1/3)면 0.3333 — ~/path 만 /scale, 명령은 무관', False),
    ('log_csv', '', 'tick 마다 한 줄 CSV(+ .meta.json). 빈칸 = 끔', True),
    ('report_every', '25', 'N tick 마다 지연 P50/P95 로그', False),
    ('allow_other_cmd_publishers', 'false', '/cmd_vel 발행자 1개 규칙 무시(시험용)', False),
    ('use_sim_time', 'true', 'topics.md:19', False),
]


def generate_launch_description():
    decl = [DeclareLaunchArgument('python', default_value=os.path.join(CD1, 'envs', 'ml_env', 'bin', 'python'),
                                  description='노드를 띄울 python(torch · X-Mobility 의존성) — docs/server207.md:27')]
    params = {}
    for name, default, desc, as_str in ARGS:
        decl.append(DeclareLaunchArgument(name, default_value=default, description=desc))
        lc = LaunchConfiguration(name)
        params[name] = ParameterValue(lc, value_type=str) if as_str else lc
    node = Node(package='fd_xm_infer', executable='xm_infer', name='xm_infer', output='screen',
                emulate_tty=True, prefix=LaunchConfiguration('python'), parameters=[params])
    return LaunchDescription(decl + [node])
