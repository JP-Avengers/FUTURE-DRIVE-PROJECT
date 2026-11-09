"""X-Mobility 파인튜닝 ckpt 로 Isaac Sim 로봇(Nova Carter)을 모는 ROS 2 추론 노드 — 참고 구현 (D).

받는 것  image_topic  sensor_msgs/Image rgb8 640×480            (/front_camera/image_raw)
         odom_topic   nav_msgs/Odometry — twist.twist.linear.x = ego_speed (/chassis/odom)
         route_topic  nav_msgs/Path — 로봇 앞 기준선 20점 0.5 m 간격  (/route, 서버 tools/xm_route.py)
                      route_source=csv 이면 /route 대신 기준선 CSV + 위치(pose_source)로 학습 때와 같은 식으로 만듦
보내는 것 cmd_topic    geometry_msgs/Twist (/cmd_vel) — linear.x = v · angular.z = ω, 나머지 0
         ~/path       nav_msgs/Path — 모델이 낸 경로 5점(base_link, path_scale 로 나눠 m)  (publish_path)
         ~/route      nav_msgs/Path — 모델에 넣은 경로 20점(base_link)  (publish_path)
서비스   ~/reset      std_srvs/Trigger — 순환 상태(history · sample)와 이전 명령을 0 으로

옮겨 온 동작(줄 번호: 이 레포 · X-Mobility 5837aea 의 navigator.py = ros2_deployment/x_mobility_navigator/
x_mobility_navigator/x_mobility_navigator.py) — README.md 의 표에 같은 내용을 더 자세히 적었다.
  모델     gin 파싱 · 가중치 대조 · 한 프레임 추론 = e2e/eval_offline.py:328-361 · 282-325 · 419-441
  주기     timer 1/rate_hz, 기본 0.2 s (navigator.py:94 · 학습 5 Hz sim/export_xmobility.py:48)
  상태     history · sample 을 매 추론 뒤 이어 받음(navigator.py:267-268), 이전 명령 = 직전 예측 6칸(:242 · :265)
  안전     /cmd_vel 발행자 1개 규칙(topics.md:40 · :63 — sim/teleop_ramp.py:189-199 처럼 확인 뒤 발행자 생성),
           끌 때 정지 명령(sim/teleop_ramp.py:257-259 · ros2_ws/src/fd_sim_warmup/.../lidar_wander.py:94-112)

실행(server207): README.md — ml_env python 으로 띄워야 함(docs/server207.md:27).
"""
import csv
import datetime
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from fd_xm_infer import core

GIN_REL = os.path.join('e2e', 'configs', 'finetune_parking_r3.gin')
ROUTE_CSV_REL = os.path.join('eval', 'routes', 'parking_r3_cw.csv')
# 건너뛸 gin configurable — 추론에서 import 하지 않는 X-Mobility train.py · evaluate.py 의 train · evaluate_* 와
# e2e/finetune.py 의 finetune(finetune_parking_r3.gin:12-15) = e2e/eval_offline.py:79
GIN_SKIP = ('train', 'evaluate_observation', 'evaluate_prediction', 'finetune')
# 사전학습 ckpt 에는 있지만 ENABLE_SEMANTIC=False 에는 없는 모듈 — unexpected 로 나와도 정상(e2e/eval_offline.py:77)
EXPECTED_UNEXPECTED = ('model.semantic_decoder.',)
CSV_COLUMNS = ['t_sim', 't_wall', 'status', 'v_raw', 'w_raw', 'v_cmd', 'w_cmd', 'ego_speed',
               'pre_ms', 'model_ms', 'total_ms', 'image_age_s', 'odom_age_s', 'route_age_s',
               'image_stamp_age_s', 'cte', 'reset']


class StopNode(Exception):
    """노드를 끝내야 하는 상황(발행자 규칙 위반 · 시작 실패) — main 이 받아 정지 명령 뒤 종료."""


def _find_repo_root():
    """$CD1 또는 이 파일 위쪽에서 e2e/configs/finetune_parking_r3.gin 이 있는 폴더(= 레포 루트)."""
    cands = [os.environ.get('CD1', '')]
    d = os.path.dirname(os.path.realpath(__file__))
    for _ in range(8):
        cands.append(d)
        d = os.path.dirname(d)
    for c in cands:
        if c and os.path.isfile(os.path.join(c, GIN_REL)):
            return c
    return None


def _as_bool(v):
    if isinstance(v, str):
        return v.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(v)


def _git_rev(path):
    """e2e/eval_offline.py:382-388."""
    try:
        rev = subprocess.check_output(['git', '-C', path, 'rev-parse', '--short=12', 'HEAD'],
                                      text=True, stderr=subprocess.DEVNULL).strip()
        dirty = subprocess.call(['git', '-C', path, 'diff', '--quiet', 'HEAD'],
                                stderr=subprocess.DEVNULL) != 0
        return rev + ('-dirty' if dirty else '')
    except Exception:  # noqa: BLE001 — git 이 없거나 레포가 아니면 기록만 비움
        return None


def _sha12(path):
    """e2e/eval_offline.py:391-396."""
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()[:12]


# ---------------------------------------------------------------- 모델 (torch · X-Mobility)

class XmModel:
    """X-Mobility 한 프레임 추론 — e2e/eval_offline.py 의 parse_gin(:328-340) · load_weights(:343-361) ·
    XMobilityStepper(:282-325) · main(:419-441) 과 같은 동작. 그 파일은 pandas 를 import 해서(ml_env 에 없을 수 있음)
    import 하지 않고 옮겨 적었다."""

    def __init__(self, xm_repo, gin_files, gin_bindings, ckpt, device, seed, allow_missing, log):
        xm = os.path.abspath(os.path.expanduser(xm_repo))
        if not os.path.isfile(os.path.join(xm, 'train.py')):
            raise StopNode(f'xm_repo 에 X-Mobility 레포가 없음(train.py): {xm}')
        if not os.path.isfile(ckpt):
            raise StopNode(f'ckpt 없음: {ckpt}')
        self.log = log
        sys.path.insert(0, xm)                                        # eval_offline.py:419
        import gin
        import torch
        # gin 이 바인딩을 알아보도록 X-Mobility 모듈을 먼저 import(eval_offline.py:422-425 = train.py:26-28 순서)
        from model.dataset.isaac_sim_dataset import XMobilityIsaacSimDataModule  # noqa: F401
        from model.dataset.data_constants import INPUT_IMAGE_SIZE
        from model.trainer import XMobilityTrainer
        self.torch = torch

        # gin — 파싱하는 동안만 X-Mobility 레포로 cd(eval_offline.py:328-340, 이유는 그 docstring)
        cfgs = [os.path.abspath(os.path.expanduser(c)) for c in gin_files]
        gin.add_config_file_search_path(xm)
        cwd = os.getcwd()
        os.chdir(xm)
        try:
            gin.parse_config_files_and_bindings(cfgs, list(gin_bindings), skip_unknown=list(GIN_SKIP))
        finally:
            os.chdir(cwd)

        dev = torch.device(device)
        if dev.type == 'cuda' and not torch.cuda.is_available():
            raise StopNode(f'device {device} 인데 CUDA 를 못 씀 — CPU 로 돌리려면 device:=cpu')
        torch.manual_seed(seed)                                       # eval_offline.py:429
        trainer = XMobilityTrainer()          # gin 값으로 생성(ImageDINOEncoder 가 HF facebook/dinov2-small 을 받음)
        self.keys = self._load_weights(trainer, ckpt, allow_missing)
        self.model = trainer.model.to(dev).eval()                     # eval_offline.py:432
        if self.model.is_gwm_pretrain:                                # eval_offline.py:433-434
            raise StopNode('IS_GWM_PRETRAIN=True — action_policy 가 없어 명령을 못 냄(x_mobility.py:67-68)')
        self.size = tuple(int(s) for s in INPUT_IMAGE_SIZE)
        if self.size != (core.OUT_H, core.OUT_W):
            raise StopNode(f'INPUT_IMAGE_SIZE {self.size} ≠ 학습 이미지 {(core.OUT_H, core.OUT_W)}'
                           '(sim/export_xmobility.py:49)')
        self.noise = None
        self.diffusion = bool(self.model.action_policy.enable_policy_diffusion)
        if self.diffusion:                                            # eval_offline.py:436-440 — 한 번 뽑아 고정
            g = torch.Generator().manual_seed(seed)
            self.noise = torch.randn(
                (1, self.model.action_policy.policy_diffuser.num_input_channels()), generator=g,
                dtype=torch.float32).to(dev)
        self.device = dev
        self.hidden_dim = int(self.model.rssm.hidden_state_dim)      # 1024 (base_train_config.gin:100)
        self.state_dim = int(self.model.rssm.state_dim)              # 512 (base_train_config.gin:101)
        self.cuda_name = torch.cuda.get_device_name(dev) if dev.type == 'cuda' else None
        self.reset()

    def _load_weights(self, module, ckpt, allow_missing):
        """e2e/eval_offline.py:343-361 (= e2e/finetune.py:138-154) — missing 이 있으면 멈춤."""
        torch = self.torch
        sd = torch.load(ckpt, map_location='cpu', weights_only=False)
        hp = sd.get('hyper_parameters', {}) if isinstance(sd, dict) else {}
        sd = sd.get('state_dict', sd)
        res = module.load_state_dict(sd, strict=False)   # 모양이 다른 키는 여기서 RuntimeError
        unexp_other = [k for k in res.unexpected_keys if not k.startswith(EXPECTED_UNEXPECTED)]
        keys = dict(loaded=len(sd) - len(res.unexpected_keys), missing=list(res.missing_keys),
                    unexpected_expected=len(res.unexpected_keys) - len(unexp_other),
                    unexpected_other=unexp_other,
                    ckpt_hparams={k: hp[k] for k in ('lr', 'weight_decay', 'scheduler_pct_start')
                                  if k in hp})
        self.log.info(f'[ckpt] {ckpt} — 넣은 키 {keys["loaded"]} · missing {len(res.missing_keys)} · '
                      f'unexpected {len(res.unexpected_keys)}(그중 의미 분할 등 예상분 '
                      f'{keys["unexpected_expected"]})')
        for k in res.missing_keys[:20]:
            self.log.warning(f'  missing: {k}')
        for k in unexp_other[:20]:
            self.log.warning(f'  unexpected(예상 밖): {k}')
        if res.missing_keys and not allow_missing:
            raise StopNode('ckpt 에 없는 가중치가 있음(무작위로 남음) — gin 설정이 ckpt 학습 때와 다른지 확인. '
                           '알고도 진행하려면 allow_missing:=true')
        return keys

    def reset(self):
        """eval_offline.py:297-300 = navigator.py:103-104 · 135-136."""
        t = self.torch
        self.history = t.zeros((1, self.hidden_dim), dtype=t.float32, device=self.device)
        self.sample = t.zeros((1, self.state_dim), dtype=t.float32, device=self.device)

    def step(self, image, speed, route_vec, prev_action):
        """(3, 320, 512) float32 · 속도 · (19, 4) · 이전 명령 6 → (action 6, path 10) numpy — eval_offline.py:302-325."""
        t, dev = self.torch, self.device
        img = t.from_numpy(image).to(dev).unsqueeze(0)                       # (1, 3, H, W)
        if tuple(img.shape[-2:]) != self.size:
            raise ValueError(f'이미지 {tuple(img.shape[-2:])} ≠ {self.size}')   # core 가 늘 320×512 로 만듦
        batch = {
            'image': img.unsqueeze(0),                                                   # (1, 1, 3, 320, 512)
            'route': t.tensor(route_vec, dtype=t.float32, device=dev).view(1, 1, *route_vec.shape),
            'speed': t.tensor([[[speed]]], dtype=t.float32, device=dev),                 # (1, 1, 1)
            'action': t.tensor(np.asarray(prev_action, np.float32).reshape(1, -1), device=dev),   # (1, 6)
            'history': self.history,                                                     # (1, 1024)
            'sample': self.sample,                                                       # (1, 512)
        }
        if self.noise is not None:
            batch['policy_noise'] = self.noise
        with t.inference_mode():
            # XMobilityTrainer.inference(trainer.py:61-63)가 부르는 것과 같은 함수 — x_mobility.py:130-176
            out = self.model.inference(batch, enable_semantic_inference=False,
                                       enable_rgb_inference=False, enable_depth=False)
        action, path, history, sample = out[:4]
        if action is None or path is None:
            raise RuntimeError("inference 가 action 을 안 돌려줌 — IS_GWM_PRETRAIN 이거나 'route' 키 누락"
                               '(x_mobility.py:150)')
        self.history, self.sample = history, sample                  # navigator.py:267-268
        return (action.reshape(-1).float().cpu().numpy(),            # .cpu() 가 GPU 동기화 — 지연 측정에 포함
                path.reshape(-1).float().cpu().numpy())


# ---------------------------------------------------------------- 노드

class XmInfer(Node):

    def __init__(self):
        super().__init__('xm_infer')
        repo = _find_repo_root()
        self.repo = repo
        p = self._param
        self.xm_repo = str(p('xm_repo', os.environ.get('XM_REPO', '~/xmobility_test/X-Mobility'),
                             'X-Mobility 레포(5837aea) 경로'))
        self.gin = str(p('gin', os.path.join(repo, GIN_REL) if repo else '',
                         '학습 gin(쉼표로 여러 개)'))
        self.gin_bindings = str(p('gin_bindings', '', 'gin 바인딩 덧붙이기, ; 로 구분'))
        self.ckpt = os.path.expanduser(str(p('ckpt', '', '파인튜닝 .ckpt (필수)')))
        self.allow_missing = _as_bool(p('allow_missing', False, 'ckpt 에 없는 가중치가 있어도 진행'))
        self.device = str(p('device', 'cuda', 'cuda · cuda:1 · cpu'))
        self.seed = int(p('seed', 0, '확산 정책일 때 고정 noise 시드'))
        self.rate_hz = float(p('rate_hz', 5.0, '추론 주기(학습 5 Hz)'))
        self.image_topic = str(p('image_topic', '/front_camera/image_raw', ''))
        self.odom_topic = str(p('odom_topic', '/chassis/odom', ''))
        self.route_topic = str(p('route_topic', '/route', ''))
        self.cmd_topic = str(p('cmd_topic', '/cmd_vel', ''))
        self.goal_topic = str(p('goal_topic', '', 'PoseStamped 가 오면 상태 리셋(navigator.py:132-136). 빈칸=끔'))
        self.robot_frame = str(p('robot_frame', 'base_link', 'navigator.py:44'))
        self.route_source = str(p('route_source', 'topic', 'topic(/route) · csv(기준선 CSV + 위치)'))
        self.route_csv = os.path.expanduser(str(p('route_csv', os.path.join(repo, ROUTE_CSV_REL) if repo else '',
                                                  'route_source=csv 의 기준선')))
        self.reverse_route = _as_bool(p('reverse_route', False, 'CSV 를 같은 출발점에서 거꾸로'))
        self.goal_s = float(p('goal_s', -1.0, '구간 시나리오 목표 s(m), 음수=한 바퀴 계속'))
        self.pose_source = str(p('pose_source', 'odom', 'csv 모드 위치: odom(odom+시작 자세) · ground_truth'))
        self.gt_topic = str(p('gt_topic', '/ground_truth/tf', ''))
        self.gt_parent = str(p('gt_parent', 'World', 'topics.md:39'))
        self.gt_child = str(p('gt_child', 'nova_carter', 'topics.md:39'))
        self.start = [float(p('start_x', float('nan'), 'odom 모드 시작 자세 x — nan 이면 CSV 옆 YAML start')),
                      float(p('start_y', float('nan'), '')),
                      float(p('start_yaw_deg', float('nan'), ''))]
        self.jpeg_quality = int(p('jpeg_quality', core.JPEG_Q, '학습과 같은 JPEG 왕복(0=끔)'))
        self.v_min = float(p('v_min', core.V_MIN, 'topics.md:61'))
        self.v_max = float(p('v_max', core.V_MAX, 'topics.md:61'))
        self.w_max = float(p('w_max', core.W_MAX, 'topics.md:61'))
        self.stale_s = float(p('stale_timeout_s', 0.5, '영상 · odom 이 이보다 오래되면 정지'))
        self.route_stale_s = float(p('route_timeout_s', 1.0, '경로(csv 모드는 위치)가 이보다 오래되면 정지'))
        self.reset_on_stale = _as_bool(p('reset_on_stale', True, '끊겼다 재개하면 상태 리셋'))
        self.route_jump_m = float(p('route_jump_m', 1.5, '경로가 이만큼 뛰면 상태 리셋'))
        self.pose_jump_m = float(p('pose_jump_m', 0.5, 'csv 모드 위치가 이만큼 뛰면 상태 리셋'))
        self.feed_clamped = _as_bool(p('feed_clamped_action', False, '이전 명령 입력에 자른 값(기본 원래 예측)'))
        self.publish_path = _as_bool(p('publish_path', True, '~/path · ~/route 디버그'))
        self.path_scale = float(p('path_scale', 1.0, '학습 데이터 PATH_SCALE.json 의 scale(p3 계열 ckpt = 0.3333) — ~/path 를 '
                                  '이 값으로 나눠 m 로 냄. 명령 v · ω 는 무관'))
        self.log_csv = os.path.expanduser(str(p('log_csv', '', 'tick 마다 한 줄 CSV(빈칸=끔)')))
        self.report_every = max(1, int(p('report_every', 25, 'N tick 마다 지연 P50/P95 로그')))
        self.warmup_iters = int(p('warmup_iters', 3, '시작 때 빈 입력으로 몇 번 돌려 둠(지연 통계 제외)'))
        self.allow_other_pub = _as_bool(p('allow_other_cmd_publishers', False, '발행자 1개 규칙 무시'))
        self.pub_wait_s = float(p('pub_check_wait_s', 2.0, '발행자 확인 전 발견 대기(s)'))
        self.wall_timeout_s = float(p('wall_timeout_s', 1.0, '실제 시간으로 이만큼 tick 이 없으면 정지 명령'))

        if self.route_source not in ('topic', 'csv'):
            raise StopNode(f'route_source {self.route_source!r} — topic 또는 csv')
        if self.pose_source not in ('odom', 'ground_truth'):
            raise StopNode(f'pose_source {self.pose_source!r} — odom 또는 ground_truth')
        if not self.ckpt:
            raise StopNode('ckpt 파라미터가 비었음 — -p ckpt:=<.../checkpoints/<best>.ckpt>(e2d_model.md 버전 칸)')
        if not self.gin:
            raise StopNode('gin 을 못 찾음 — CD1 을 export 하거나 -p gin:=<레포>/' + GIN_REL)
        if self.rate_hz <= 0:
            raise StopNode('rate_hz > 0 이어야 함')
        if not self.path_scale > 0:
            raise StopNode(f'path_scale {self.path_scale} — 양수(p3 계열 ckpt 는 0.3333, 원래 축척 1.0)')

        self.model = None
        self.cmd_pub = None
        self.path_pub = self.route_pub = None
        self.poly = None
        self.goal = None
        self.tf_buffer = None
        self.stats = core.LatencyStats()
        self.counts = dict(ok=0, wait=0, stale=0, no_tf=0, nan=0, error=0)
        self.n_tick = self.n_reset = 0
        self.last_reason = ''
        self.reset_log = []             # 이번 tick 까지 쌓인 리셋 이유 — CSV reset 칸
        self.csv_file = self.csv_writer = None
        self.meta_path = None
        self.meta = None
        self._clear_inputs()
        self.prev_action = np.zeros(core.ACTION_SIZE, np.float32)
        self.last_cmd = (0.0, 0.0)
        self.last_now_ns = None
        self.last_tick_wall = time.monotonic()
        self.last_odom_stamp = None
        self.need_reset = False
        self.pending_reset = None

    def _param(self, name, default, desc):
        # dynamic_typing — 런치에서 5 · '5.0' · '' 처럼 형이 달라도 받고 여기서 바꿈
        d = ParameterDescriptor(description=desc, dynamic_typing=True)
        return self.declare_parameter(name, default, d).value

    def _clear_inputs(self):
        self.image_msg = None
        self.image_recv = None
        self.speed = None
        self.odom_recv = None
        self.odom_pose = None          # odom 좌표 (x, y, yaw) — 로봇 좌표 경로의 점프 판정용
        self.route = None              # topic 모드: dict(pts (N, 3), frame, recv)
        self.route_fixed = None        # 점프 판정용 직전 경로(고정 좌표) · 그 좌표계 이름
        self.pose = None               # csv 모드: dict(x, y, yaw, recv)
        self.s_hint = None
        self.cte = float('nan')

    # ------------------------------------------------------------ 시작 · 끝

    def startup(self):
        lg = self.get_logger()
        self._check_cmd_publishers(self.pub_wait_s, '시작 전')
        if self.route_source == 'csv':
            self._setup_csv_route()
        else:
            lg.info(f'경로: {self.route_topic} (nav_msgs/Path) — frame 이 {self.robot_frame} 가 아니면 tf 로 바꿈'
                    '(navigator.py:139-147)')
        gins = [g.strip() for g in self.gin.split(',') if g.strip()]
        binds = [b.strip() for b in self.gin_bindings.split(';') if b.strip()]
        lg.info(f'모델 로드: xm_repo {self.xm_repo} · gin {gins} {binds} · ckpt {self.ckpt} · device {self.device}')
        t0 = time.perf_counter()
        self.model = XmModel(self.xm_repo, gins, binds, self.ckpt, self.device, self.seed,
                             self.allow_missing, lg)
        lg.info(f'모델 로드 {time.perf_counter() - t0:.1f} s · 정책 {"확산" if self.model.diffusion else "MLP"}'
                f' · GPU {self.model.cuda_name}')
        self._warmup()
        self._write_meta(gins, binds)
        # 로드하는 동안 다른 발행자가 떴을 수 있음 — 발행자를 만들기 직전에 한 번 더(teleop_ramp.py:189-199 순서)
        self._check_cmd_publishers(0.3, '발행 직전')
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        if self.publish_path:
            self.path_pub = self.create_publisher(Path, '~/path', 10)
            self.route_pub = self.create_publisher(Path, '~/route', 10)
        q = qos_profile_sensor_data   # best effort — 발행 쪽이 reliable 이어도 받음
        self.create_subscription(Image, self.image_topic, self._on_image, q)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, q)
        if self.route_source == 'topic':
            from tf2_ros.buffer import Buffer
            from tf2_ros.transform_listener import TransformListener
            self.tf_buffer = Buffer()                                  # navigator.py:97-98
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(Path, self.route_topic, self._on_route, q)
        elif self.pose_source == 'ground_truth':
            self.create_subscription(TFMessage, self.gt_topic, self._on_gt, q)
        if self.goal_topic:
            self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_service(Trigger, '~/reset', self._on_reset_srv)
        if self.log_csv:
            os.makedirs(os.path.dirname(os.path.abspath(self.log_csv)), exist_ok=True)
            self.csv_file = open(self.log_csv, 'w', newline='', buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(CSV_COLUMNS)
        self.create_timer(1.0 / self.rate_hz, self._tick)                   # navigator.py:94
        # 시뮬 시간 타이머는 /clock 이 멈추면(Pause · 시뮬 죽음) 같이 멈춤 — 실제 시간 타이머로 지켜보며 정지 명령.
        # 이 타이머가 spin 을 주기적으로 깨워 Ctrl+C(SignalHandlerOptions.NO 에서 KeyboardInterrupt)도 바로 받음
        self.create_timer(0.5, self._wall_watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        lg.info(f'시작 — {1.0 / self.rate_hz:.3f} s 마다 추론 · 상한 v [{self.v_min}, {self.v_max}] · '
                f'|ω| ≤ {self.w_max} · 끊김 {self.stale_s}/{self.route_stale_s} s · '
                f'use_sim_time {self.get_parameter("use_sim_time").value}')

    def _setup_csv_route(self):
        lg = self.get_logger()
        if not os.path.isfile(self.route_csv):
            raise StopNode(f'route_csv 없음: {self.route_csv}')
        self.poly = core.Poly(core.load_route_csv(self.route_csv, self.reverse_route))
        self.goal = None if self.goal_s < 0 else self.goal_s % self.poly.L   # export_xmobility.py:172
        if self.pose_source == 'odom' and not all(math.isfinite(v) for v in self.start):
            ymp = os.path.splitext(self.route_csv)[0] + '.yaml'               # eval/routes/parking_r3_cw.yaml 의 start
            try:
                import yaml
                with open(ymp, encoding='utf-8') as f:
                    st = yaml.safe_load(f)['start']
                self.start = [float(st['x']), float(st['y']), float(st['yaw_deg'])]
            except Exception as e:  # noqa: BLE001
                raise StopNode(f'시작 자세를 못 읽음({ymp}: {e}) — start_x · start_y · start_yaw_deg 를 주세요')
        lg.info(f'경로: {self.route_csv}(L {self.poly.L:.2f} m, 거꾸로 {self.reverse_route}, 목표 s {self.goal})'
                f' + 위치 {self.pose_source}'
                + (f' · 시작 자세 {self.start}' if self.pose_source == 'odom' else
                   f' ({self.gt_topic} {self.gt_parent}→{self.gt_child})'))
        if self.pose_source == 'ground_truth':
            lg.warning('pose_source=ground_truth — 디버그용. 평가 때 모델 입력에는 정답 위치를 쓰지 않음(topics.md:39)')

    def _warmup(self):
        if self.warmup_iters <= 0:
            return
        img = np.zeros((3, core.OUT_H, core.OUT_W), np.float32)
        rv = np.zeros((core.NUM_ROUTE_POINTS - 1, core.ROUTE_VECTOR_SIZE), np.float32)
        ts = []
        for _ in range(self.warmup_iters):
            t0 = time.perf_counter()
            self.model.step(img, 0.0, rv, np.zeros(core.ACTION_SIZE, np.float32))
            ts.append((time.perf_counter() - t0) * 1e3)
        self.model.reset()
        self.get_logger().info('예열 ' + ' · '.join(f'{t:.1f}' for t in ts) + ' ms (지연 통계에서 뺌)')

    def _write_meta(self, gins, binds):
        if not self.log_csv:
            return
        self.meta_path = os.path.splitext(self.log_csv)[0] + '.meta.json'
        self.meta = dict(
            started=datetime.datetime.now().isoformat(timespec='seconds'), node='fd_xm_infer',
            repo=self.repo, repo_rev=_git_rev(self.repo) if self.repo else None,
            xm_repo=os.path.abspath(os.path.expanduser(self.xm_repo)),
            xm_rev=_git_rev(os.path.abspath(os.path.expanduser(self.xm_repo))),
            gin=gins, gin_bindings=binds, gin_skip=list(GIN_SKIP), ckpt=self.ckpt,
            ckpt_sha12=_sha12(self.ckpt), keys=self.model.keys, device=self.device,
            cuda_device=self.model.cuda_name, policy_diffusion=self.model.diffusion,
            params={k: v.value for k, v in self.get_parameters_by_prefix('').items()})
        with open(self.meta_path, 'w', encoding='utf-8') as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=1, default=str)

    def stop(self):
        """정지 명령 5번(sim/teleop_ramp.py:257-259 — 받는 쪽은 마지막 명령을 계속 따름) · 지연 요약 · 기록 닫기."""
        if self.cmd_pub is not None:
            for _ in range(5):
                try:
                    self.cmd_pub.publish(Twist())
                except Exception:  # noqa: BLE001 — 컨텍스트가 이미 닫혔으면 더 못 보냄
                    break
                time.sleep(0.02)
            self.get_logger().info('정지 명령 보냄 — 종료')
        tot = {k: self.stats.summary(k, window=False) for k in ('pre', 'model', 'total')}
        line = ' '.join(f'{k}_p50={s["p50"]:.2f} {k}_p95={s["p95"]:.2f} {k}_max={s["max"]:.2f}'
                        for k, s in tot.items() if s['n'])
        self.get_logger().info(f'XM_INFER_LATENCY_MS n={tot["model"]["n"]} {line} '
                               f'device={self.device} gpu={self.model.cuda_name if self.model else None} '
                               f'counts={self.counts} resets={self.n_reset}')
        if self.csv_file is not None:
            self.csv_file.close()
        if self.meta_path and self.meta is not None:
            self.meta.update(finished=datetime.datetime.now().isoformat(timespec='seconds'),
                             latency_ms=tot, counts=self.counts, ticks=self.n_tick, resets=self.n_reset)
            with open(self.meta_path, 'w', encoding='utf-8') as f:
                json.dump(self.meta, f, ensure_ascii=False, indent=1, default=str)

    # ------------------------------------------------------------ 발행자 1개 규칙

    def _other_cmd_publishers(self):
        me = (self.get_name(), self.get_namespace())
        return [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                for i in self.get_publishers_info_by_topic(self.cmd_topic)
                if (i.node_name, i.node_namespace) != me]

    def _check_cmd_publishers(self, wait_s, when):
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait_s:        # 발견(discovery) 대기 — sim/teleop_ramp.py:190-192
            rclpy.spin_once(self, timeout_sec=0.1)
        others = self._other_cmd_publishers()
        if not others:
            return
        msg = (f'{when}: {self.cmd_topic} 발행자가 이미 {len(others)}개({", ".join(others)}) — 발행자는 항상 1개'
               '(docs/interfaces/topics.md:40 · :63). 먼저 그쪽을 끄세요')
        if self.allow_other_pub:
            self.get_logger().warning(msg + ' — allow_other_cmd_publishers=true 라 계속')
        else:
            raise StopNode(msg + ' (무시하려면 allow_other_cmd_publishers:=true)')

    # ------------------------------------------------------------ 받기

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _on_image(self, msg):
        self.image_msg = msg                       # 변환은 tick 에서(20 Hz 중 5 Hz 만 씀)
        self.image_recv = self._now_ns()

    def _on_odom(self, msg):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_odom_stamp is not None and st < self.last_odom_stamp - 0.5:
            self.pending_reset = 'odom 시각 역행(Stop → Play, topics.md:58 · teleop_ramp.py:205)'
        self.last_odom_stamp = st
        self.speed = float(msg.twist.twist.linear.x)          # navigator.py:128-130 · e2d_model.md:13
        self.odom_recv = self._now_ns()
        pp, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom_pose = (pp.x, pp.y, core.quat_yaw(q.x, q.y, q.z, q.w))
        if self.route_source == 'csv' and self.pose_source == 'odom':
            self._set_pose(*core.odom_to_map(*self.odom_pose, self.start))

    def _on_gt(self, msg):
        for tr in msg.transforms:                  # sim/export_xmobility.py:201-204 — 바퀴 링크 변환은 버림
            if tr.header.frame_id == self.gt_parent and tr.child_frame_id == self.gt_child:
                q = tr.transform.rotation
                self._set_pose(tr.transform.translation.x, tr.transform.translation.y,
                               core.quat_yaw(q.x, q.y, q.z, q.w))

    def _set_pose(self, x, y, yaw):
        if self.pose is not None and math.hypot(x - self.pose['x'], y - self.pose['y']) > self.pose_jump_m:
            self.pending_reset = (f'위치 점프 {math.hypot(x - self.pose["x"], y - self.pose["y"]):.2f} m'
                                  f' > {self.pose_jump_m}')
        self.pose = dict(x=x, y=y, yaw=yaw, recv=self._now_ns())

    def _on_route(self, msg):
        n = len(msg.poses)
        if n == 0:                                 # navigator.py:151-153 — 빈 경로는 무시
            return
        pts = np.array([[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z] for ps in msg.poses],
                       np.float64)
        frame = msg.header.frame_id
        now = self._now_ns()
        # 점프 판정은 고정 좌표에서: 로봇 좌표 경로는 받은 순간의 odom 자세로 odom 좌표로 바꿔 비교
        sel = core.select_route_points(pts[:, :2])
        if frame in ('', self.robot_frame):
            fixed = core.robot_to_fixed(sel, *self.odom_pose) if self.odom_pose else None
            fframe = 'odom'
        else:
            fixed, fframe = sel, frame
        prev = self.route_fixed
        if (prev is not None and fixed is not None and self.route is not None
                and (now - self.route['recv']) * 1e-9 <= self.route_stale_s):
            if prev[1] != fframe:
                self.pending_reset = f'경로 frame 바뀜 {prev[1]} → {fframe}'
            else:
                jumped, d = core.route_jump(prev[0], fixed, self.route_jump_m)
                if jumped:
                    self.pending_reset = f'경로 점프 {d:.2f} m > {self.route_jump_m}'
        self.route_fixed = (fixed, fframe) if fixed is not None else None
        self.route = dict(pts=pts, frame=frame, recv=now)

    def _on_goal(self, msg):
        self.pending_reset = f'새 목표({self.goal_topic}, navigator.py:132-136)'

    def _on_reset_srv(self, req, res):
        self._reset('서비스 ~/reset')
        res.success, res.message = True, f'리셋 {self.n_reset}번째'
        return res

    def _reset(self, why):
        """순환 상태 · 이전 명령 0 — eval_offline.py:236-237(run 시작마다 history · sample · action 0)."""
        if self.model is not None:
            self.model.reset()
        self.prev_action = np.zeros(core.ACTION_SIZE, np.float32)
        self.s_hint = None                         # export_xmobility.py:229-231 — 끊기면 사영 힌트도 버림
        self.need_reset = False
        self.pending_reset = None
        self.n_reset += 1
        self.last_reason = why
        self.reset_log.append(why)
        self.get_logger().info(f'상태 리셋 #{self.n_reset}: {why}')

    # ------------------------------------------------------------ 추론 tick

    def _ages(self, now):
        def age(t):
            return None if t is None else (now - t) * 1e-9
        route_t = (self.route['recv'] if self.route else None) if self.route_source == 'topic' else \
            (self.pose['recv'] if self.pose else None)
        return age(self.image_recv), age(self.odom_recv), age(route_t)

    def _route_points(self):
        """모델에 넣을 로봇 좌표계 20점 (20, 2) 또는 None(tf 없음)."""
        if self.route_source == 'csv':
            ps = self.pose
            pts, self.s_hint, self.cte = core.build_route_poses(self.poly, ps['x'], ps['y'], ps['yaw'],
                                                                self.s_hint, self.goal)
            return pts
        r = self.route
        if r['frame'] in ('', self.robot_frame):
            xy = r['pts'][:, :2]
        else:
            from tf2_ros import TransformException
            try:                                                       # navigator.py:140-147
                tf = self.tf_buffer.lookup_transform(self.robot_frame, r['frame'], Time())
            except TransformException as ex:
                self.get_logger().error(f'tf {self.robot_frame} ← {r["frame"]} 없음: {ex}',
                                        throttle_duration_sec=2.0)
                return None
            tr, q = tf.transform.translation, tf.transform.rotation
            xy = core.transform_points(r['pts'], (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))[:, :2]
        return core.select_route_points(xy)                                # navigator.py:149-156

    def _publish(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)          # 나머지 성분 0 (topics.md:40)
        self.cmd_pub.publish(m)
        self.last_cmd = (float(v), float(w))

    def _publish_debug(self, route_pts, path10):
        stamp = self.get_clock().now().to_msg()
        # 모델 경로는 학습 라벨 축척(p3 계열 = ×1/3)이라 path_scale 로 나눠 m 로 — 명령에는 안 씀(디버그 발행만)
        path_m = np.asarray(path10, np.float64).reshape(-1, 2) / self.path_scale
        for pub, pts in ((self.route_pub, route_pts), (self.path_pub, path_m)):
            msg = Path()
            msg.header.frame_id, msg.header.stamp = self.robot_frame, stamp
            for x, y in pts:
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                ps.pose.orientation.w = 1.0
                msg.poses.append(ps)
            pub.publish(msg)

    def _wall_watchdog(self):
        if self.cmd_pub is not None and time.monotonic() - self.last_tick_wall > self.wall_timeout_s:
            self._publish(0.0, 0.0)
            self.get_logger().warning(f'실제 시간 {self.wall_timeout_s} s 넘게 tick 없음(/clock 멈춤?) → 정지 명령',
                                      throttle_duration_sec=10.0)

    def _tick(self):
        t0 = time.perf_counter()
        self.last_tick_wall = time.monotonic()
        now = self._now_ns()
        self.n_tick += 1
        lg = self.get_logger()
        if self.last_now_ns is not None and now < self.last_now_ns:   # Stop → Play: 시뮬 시간 0 부터(topics.md:58)
            self._clear_inputs()
            self._reset('시계 역행(Stop → Play?) — 입력 비움')
        self.last_now_ns = now
        if self.pending_reset:
            self._reset(self.pending_reset)

        a_img, a_odom, a_route = self._ages(now)
        row = dict(t_sim=now * 1e-9, t_wall=time.time(), image_age_s=a_img, odom_age_s=a_odom,
                   route_age_s=a_route)
        status, v_raw, w_raw, v, w = 'ok', None, None, 0.0, 0.0
        t_pre = t_model = None
        if a_img is None or a_odom is None or a_route is None:
            status = 'wait'
            lg.info(f'입력 기다리는 중 — 영상 {a_img is not None} · odom {a_odom is not None} · '
                    f'경로 {a_route is not None}', throttle_duration_sec=5.0)
        elif min(a_img, a_odom, a_route) < 0 or a_img > self.stale_s or a_odom > self.stale_s \
                or a_route > self.route_stale_s:
            status = 'stale'               # 정지 + 재개 때 리셋(export_xmobility.py:53 · 264-273)
            self.need_reset = self.reset_on_stale
            lg.warning(f'입력 끊김 → 정지: 영상 {a_img:.2f} · odom {a_odom:.2f} · 경로 {a_route:.2f} s',
                       throttle_duration_sec=2.0)
        else:
            if self.need_reset:
                self._reset('입력 끊김 뒤 재개(학습 run 은 0.3 s 넘게 끊기면 나뉨, export_xmobility.py:53)')
            try:
                route_pts = self._route_points()
                if route_pts is None:
                    status = 'no_tf'
                    self.need_reset = self.reset_on_stale
                else:
                    im = self.image_msg
                    img = core.image_to_chw(im.data, im.height, im.width, im.step, im.encoding,
                                            self.jpeg_quality)
                    rv = core.route_vectors(route_pts)
                    if self.route_source == 'csv':
                        row['cte'] = self.cte
                    t_pre = time.perf_counter()
                    action, path = self.model.step(img, self.speed, rv, self.prev_action)
                    t_model = time.perf_counter()
                    v_raw, w_raw = float(action[core.V_IDX]), float(action[core.W_IDX])
                    v, w, finite = core.clamp_cmd(v_raw, w_raw, self.v_min, self.v_max, self.w_max)
                    if not finite:
                        status = 'nan'
                        self._reset('모델 출력이 유한값 아님')
                    else:
                        fed = action.copy()
                        if self.feed_clamped:
                            fed[core.V_IDX], fed[core.W_IDX] = v, w
                        self.prev_action = fed.astype(np.float32)        # navigator.py:242 · 265
                        if self.publish_path:
                            self._publish_debug(route_pts, path)
                    st = im.header.stamp
                    row['image_stamp_age_s'] = now * 1e-9 - (st.sec + st.nanosec * 1e-9)
                    if (im.width, im.height) != (640, 480):
                        lg.warning(f'영상 {im.width}x{im.height} — 학습은 640×480 을 512×320 으로 줄인 것'
                                   '(topics.md:33)', throttle_duration_sec=30.0)
            except Exception as e:  # noqa: BLE001 — 한 번 실패로 노드를 죽이지 않고 정지 명령
                status, v, w = 'error', 0.0, 0.0
                lg.error(f'추론 실패 → 정지: {type(e).__name__}: {e}', throttle_duration_sec=2.0)
                self._reset(f'추론 실패({type(e).__name__})')
        if status != 'ok':
            v, w = 0.0, 0.0
        self._publish(v, w)
        t_end = time.perf_counter()
        self.counts[status] = self.counts.get(status, 0) + 1
        if status == 'ok':
            self.stats.add('pre', (t_pre - t0) * 1e3)
            self.stats.add('model', (t_model - t_pre) * 1e3)
            self.stats.add('total', (t_end - t0) * 1e3)
        if self.csv_writer is not None:
            row.update(status=status, v_raw=v_raw, w_raw=w_raw, v_cmd=v, w_cmd=w, ego_speed=self.speed,
                       pre_ms=None if t_pre is None else (t_pre - t0) * 1e3,
                       model_ms=None if t_model is None else (t_model - t_pre) * 1e3,
                       total_ms=(t_end - t0) * 1e3, reset='; '.join(self.reset_log))
            self.csv_writer.writerow(['' if row.get(c) is None else
                                      (f'{row[c]:.6f}' if isinstance(row[c], float) else row[c])
                                      for c in CSV_COLUMNS])
        self.reset_log = []
        if self.n_tick % self.report_every == 0:
            self._report()

    def _report(self):
        s = {k: self.stats.summary(k) for k in ('pre', 'model', 'total')}
        self.get_logger().info(
            f'최근 {s["model"]["n"]}회 지연 — 모델 {core.fmt_ms(s["model"])} · 전처리 {core.fmt_ms(s["pre"])} · '
            f'tick 전체 {core.fmt_ms(s["total"])} | 상태 {self.counts} · 리셋 {self.n_reset} · '
            f'v {self.last_cmd[0]:.2f} ω {self.last_cmd[1]:+.2f}')
        self.stats.reset_window()
        others = self._other_cmd_publishers()
        if others and not self.allow_other_pub:
            raise StopNode(f'{self.cmd_topic} 에 다른 발행자 {others} 가 생김 — 발행자 1개 규칙(topics.md:40), 종료')


def _on_term(*_):
    raise KeyboardInterrupt            # SIGTERM(kill · launch)도 Ctrl+C 와 같은 정리 경로로(lidar_wander.py:94-95)


def main(args=None):
    # rclpy 기본 신호 처리는 Ctrl+C 에서 ROS 연결부터 닫아 마지막 정지 명령을 못 보냄
    # → 신호 처리를 끄고 직접 받는다(lidar_wander.py:98-102 · teleop_ramp.py:188)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _on_term)
    node, rc = None, 0
    try:
        node = XmInfer()
        node.startup()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except StopNode as e:
        rc = 3
        if node is not None:
            node.get_logger().error(str(e))
        else:
            print(f'[xm_infer] {e}', file=sys.stderr)
    finally:
        if node is not None:
            node.stop()                # 발행자를 만들기 전에 멈췄으면 정지 명령도 안 보냄(남의 /cmd_vel 을 건드리지 않음)
            node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
