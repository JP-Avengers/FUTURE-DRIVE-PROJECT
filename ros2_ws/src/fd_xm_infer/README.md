# fd_xm_infer — X-Mobility 파인튜닝 체크포인트 추론 노드 (참고 구현)

- **담당**: D · 정동현
- **상태**: 참고 구현(11.5). **실제 ROS · GPU 실행은 아직 안 했습니다.** 순수 함수(`core.py`)는 단위 시험으로, 노드 분기는 가짜 rclpy 로 확인했습니다(아래 "확인한 것 · 안 한 것").
- **계약**: [`docs/interfaces/e2d_model.md`](../../../docs/interfaces/e2d_model.md) (입력 · 출력 · ckpt · forward 위치), [`docs/interfaces/topics.md`](../../../docs/interfaces/topics.md) (토픽 · 상한 · 발행자 1개)

카메라 · 속도 · 경로를 받아 X-Mobility(`5837aea`) 파인튜닝 `.ckpt` 로 한 프레임씩 추론하고 `/cmd_vel` 을 냅니다. 전처리 · 경로 · 상태 처리는 학습 데이터를 만든 코드(`sim/export_xmobility.py`)와 오프라인 평가(`e2e/eval_offline.py`)의 식을 그대로 옮겼습니다.

## 파일

| 파일 | 내용 |
| --- | --- |
| `fd_xm_infer/core.py` | 순수 계산(numpy · Pillow) — 영상 → (3, 320, 512), 경로 → 로봇 좌표 20점 → (19, 4), 좌표 변환, 경로 점프 판정, 상한 자르기, 지연 통계 |
| `fd_xm_infer/node.py` | ROS 2 노드 `xm_infer` + 모델 래퍼 `XmModel`(gin · 가중치 대조 · 한 프레임 추론) |
| `launch/xm_infer.launch.py` | 주요 파라미터를 런치 인자로. `python` 인자(기본 `$CD1/envs/ml_env/bin/python`)로 노드를 띄움 |

## 실행 (server207)

X-Mobility 의존성(torch · gin · pytorch_lightning · transformers)은 rosdep 밖이라 **`envs/ml_env` python 으로 띄웁니다**([`docs/server207.md`](../../../docs/server207.md):27). Isaac Sim 이 GPU 0 이므로 추론은 GPU 1(`CUDA_VISIBLE_DEVICES=1`, server207.md:26). 모델을 만들 때 HF `facebook/dinov2-small` 을 받으므로 캐시가 있으면 `HF_HUB_OFFLINE=1`.

```bash
export CD1=$HOME/Capstone_Design_1 && cd $CD1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # topics.md:65 — 씬과 같은 도메인으로: run_proxy.sh 기본 42, eval/xm_batch.sh · sim/nav2/scenario_batch.sh 가 띄운 씬은 43

# 빌드(한 번)
(cd ros2_ws && colcon build --symlink-install --packages-select fd_xm_infer)
source ros2_ws/install/setup.bash

# 1) 씬 — FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/run_proxy.sh headless  (tmux 안에서)
# 2) 경로 — 서버 ~/xmobility_test/tools/xm_route.py 가 /route 를 냄(레포 밖 도구, 사용법은 그 도구 기준)
#    또는 아래처럼 route_source:=csv 로 노드가 직접 만듦
# 3) /cmd_vel 발행자가 0 인지(topics.md:63) — 노드도 스스로 확인하고 있으면 시작을 거부함
ros2 topic info /cmd_vel

# 4) 추론 — ckpt 는 best 파일(이름 · sha 는 e2d_model.md 버전 칸 · MANIFEST; last.ckpt 는 마지막 에폭이라 동결 모델이 아님)
CUDA_VISIBLE_DEVICES=1 HF_HUB_OFFLINE=1 ros2 launch fd_xm_infer xm_infer.launch.py \
    ckpt:=$CD1/data/checkpoints/xm_ft_v0.1_aw30/checkpoints/xm_ft_v0.1_aw30_e8.ckpt \
    log_csv:=$CD1/logs/xm_infer/<이름>.csv
#    p3 계열 ckpt(경로 라벨 ×1/3, 예 xm_ft_v0.1_aw30_p3)면 path_scale:=0.3333 을 더함 — ~/path(RViz)만 영향, 명령은 무관
#    실행 명령 입력으로 학습한 ckpt(--prev-action-source exec, 예 xm_ft_v1.4_aw30_exec_pad05_e10 · xm_ft_v1.5_aw30_exec_e6)면
#    prev_action_mode:=clamped 를 더함(11.12 전 런치 파일은 이 인자를 노드에 안 넘겨 조용히 own 으로 돌았음)

# 경로를 기준선 CSV + odom(+ 시작 자세)으로 직접 만들 때(xm_route.py 없이). 시작 자세 = 씬이 로봇을 띄운 자세(r3 루트 -96°,
# parking_v0.4b_r3.usda:27). 안 주면 CSV 옆 YAML start 인데 parking_r3_ccw.yaml 은 84° → 반시계 CSV 를 씬 그대로 띄운 로봇에
# 쓰면 odom→지도 변환이 180° 틀려 경로가 엉뚱함. 로봇을 84° 로 돌려 띄웠을 때만 생략 가능(eval/xm_batch.sh 는 늘 넘김)
CUDA_VISIBLE_DEVICES=1 ros2 launch fd_xm_infer xm_infer.launch.py ckpt:=<...> route_source:=csv \
    route_csv:=$CD1/eval/routes/parking_r3_ccw.csv pose_source:=odom start_x:=-1.756 start_y:=-7.621 start_yaw_deg:=-96.0

# 상태 리셋(새 시도 · 로봇을 옮긴 뒤)
ros2 service call /xm_infer/reset std_srvs/srv/Trigger
```

colcon 없이 바로(같은 셸에서 `source /opt/ros/jazzy/setup.bash` 뒤):

```bash
PYTHONPATH=$CD1/ros2_ws/src/fd_xm_infer:$PYTHONPATH CUDA_VISIBLE_DEVICES=1 \
  $CD1/envs/ml_env/bin/python -m fd_xm_infer.node --ros-args -p use_sim_time:=true \
  -p ckpt:=<...> -p log_csv:=$CD1/logs/xm_infer/<이름>.csv
```

**추론 지연 측정(e2d_model.md "추론 지연 요구" 칸)**: `report_every` tick(기본 25 = 5 s)마다 최근 구간의 모델 · 전처리 · tick 전체 P50/P95 를 로그로 찍고, 끝(Ctrl+C)에 전체 구간을 `XM_INFER_LATENCY_MS n=… model_p50=… model_p95=… total_p50=… total_p95=…` 한 줄로 찍습니다. `log_csv` 를 주면 tick 마다 한 줄(CSV)과 `<이름>.meta.json`(ckpt sha256 12자리 · 레포 커밋 · gin · 키 대조 · GPU 이름 · 끝날 때 지연 요약)을 남깁니다. 모델 지연 = 텐서 만들기 → `inference` → 결과 `.cpu()`(GPU 동기화 포함), 시작할 때 예열 `warmup_iters`(3)번은 통계에서 뺍니다.

## 토픽 · 서비스

| | 이름(기본) | 타입 | 비고 |
| --- | --- | --- | --- |
| 받음 | `image_topic` `/front_camera/image_raw` | `sensor_msgs/Image` | `rgb8`(bgr8 도 받음). cv_bridge 안 씀 — 바이트를 numpy 로 직접(export_xmobility.py:234 와 같은 식) |
| 받음 | `odom_topic` `/chassis/odom` | `nav_msgs/Odometry` | `twist.twist.linear.x` = ego_speed. 자세는 경로 점프 판정 · `pose_source=odom` 에 씀 |
| 받음 | `route_topic` `/route` | `nav_msgs/Path` | `route_source=topic`(기본). frame 이 `base_link`(또는 빈칸)면 그대로, 아니면 tf 로 바꿈 |
| 받음 | `gt_topic` `/ground_truth/tf` | `tf2_msgs/TFMessage` | `route_source=csv` · `pose_source=ground_truth` 일 때만. `World → nova_carter` 만 씀. **디버그 전용** |
| 받음 | `goal_topic`(기본 끔) | `geometry_msgs/PoseStamped` | 오면 상태 리셋(navigator.py:132-136) |
| 냄 | `cmd_topic` `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` = v, `angular.z` = ω, 나머지 0. **발행자 1개** |
| 냄 | `/xm_infer/path` · `/xm_infer/route` | `nav_msgs/Path` | `publish_path`. 모델이 낸 경로 5점(`path_scale` 로 나눠 m) · 모델에 넣은 경로 20점(둘 다 `base_link`) |
| 서비스 | `/xm_infer/reset` | `std_srvs/Trigger` | history · sample · 이전 명령 0, csv 모드 사영 힌트 버림 |

모든 구독은 best effort(`qos_profile_sensor_data`) — 발행 쪽이 reliable 이어도 받습니다.

## 파라미터

| 이름 | 기본 | 설명 |
| --- | --- | --- |
| `xm_repo` | `$XM_REPO` 또는 `~/xmobility_test/X-Mobility` | X-Mobility 레포(`5837aea`) |
| `gin` | `<레포>/e2e/configs/finetune_parking_r3.gin` | 학습 gin(쉼표로 여러 개). 레포 루트는 `$CD1` 또는 이 파일 위쪽에서 찾음 |
| `gin_bindings` | `''` | 바인딩 덧붙이기, `;` 구분 |
| `ckpt` | (필수) | 파인튜닝 `.ckpt` |
| `allow_missing` | false | ckpt 에 없는 가중치가 있어도 진행(기본은 멈춤) |
| `device` · `seed` | `cuda` · 0 | `cuda` 를 못 쓰면 CPU 로 바꾸지 않고 멈춤. seed 는 확산 정책 noise 용 |
| `rate_hz` | 5.0 | 추론 주기(노드 시계 — `use_sim_time` 이면 시뮬 시간) |
| `robot_frame` | `base_link` | |
| `route_source` | `topic` | `topic`(/route) · `csv`(기준선 CSV + 위치로 직접) |
| `route_csv` · `reverse_route` · `goal_s` | `<레포>/eval/routes/parking_r3_cw.csv` · false · −1 | csv 모드. `goal_s` ≥ 0 이면 거기서 끊고 끝점 반복(구간 시나리오) |
| `pose_source` | `odom` | csv 모드 위치 — `odom`(odom + 시작 자세) · `ground_truth` |
| `start_x` · `start_y` · `start_yaw_deg` | nan | `pose_source=odom` 시작 자세 = 씬이 로봇을 띄운 자세(r3 −96°). nan 이면 CSV 옆 YAML(`parking_r3_cw.yaml` 등)의 `start` — `parking_r3_ccw.yaml` 은 84° 라 반시계 CSV 를 씬 그대로 띄웠으면 꼭 지정. 런치 인자로도 받음 |
| `jpeg_quality` | 90 | 학습과 같은 JPEG 왕복. 0 = 끔(학습 입력과 달라짐) |
| `v_min` · `v_max` · `w_max` | 0.0 · 0.8 · 0.7 | topics.md:61 상한 |
| `stale_timeout_s` · `route_timeout_s` | 0.5 · 1.0 | 영상 · odom / 경로(csv 모드는 위치) 이 시간보다 오래되면 정지 명령 |
| `reset_on_stale` | true | 끊겼다(또는 tf 실패) 재개하면 상태 리셋 |
| `route_jump_m` · `pose_jump_m` | 1.5 · 0.5 | 경로 · 위치가 이만큼 뛰면 상태 리셋 |
| `prev_action_mode` | `own` | 이전 명령 입력 — `own`(모델 원래 출력 6칸, navigator 와 같음) · `zero`(늘 0, 리셋 직후와 같은 입력 — 드롭아웃 학습판을 '직전 명령 0' 으로 볼 때) · `clamped`(v · ω 칸만 `v_min`–`v_max` · ±`w_max` 로 자른 값 = 보낸 명령, 나머지 4칸 그대로). 그 밖의 값(`teacher` 포함)은 시작 때 멈춤. 오프라인 평가 `e2e/eval_offline.py --prev-action-mode` 와 같은 이름 |
| `feed_clamped_action` | false | true = `prev_action_mode:=clamped`(예전 이름, 그대로 받음). `prev_action_mode:=zero` 와 같이 주면 충돌로 시작 때 멈춤 |
| `publish_path` · `log_csv` · `report_every` · `warmup_iters` | true · `''` · 25 · 3 | 디버그 · 기록 |
| `path_scale` | 1.0 | p3 계열 ckpt(경로 라벨 ×1/3, 데이터 `PATH_SCALE.json` 의 scale)면 0.3333 — `~/path` 를 이 값으로 나눠 m 로 냄. 명령 v · ω 는 무관(모델 출력 path 는 디버그 발행에만 씀). `.meta.json` params 에 남음 |
| `allow_other_cmd_publishers` · `pub_check_wait_s` | false · 2.0 | 발행자 1개 규칙 무시(시험용) · 확인 전 발견 대기 |
| `wall_timeout_s` | 1.0 | 실제 시간으로 이만큼 tick 이 없으면(/clock 멈춤) 0.5 s 마다 정지 명령 |

## 동작과 출처

X-Mobility 줄 번호는 `5837aea` 기준, `navigator.py` = `ros2_deployment/x_mobility_navigator/x_mobility_navigator/x_mobility_navigator.py`.

| 동작 | 여기 | 출처 |
| --- | --- | --- |
| gin 파싱 — 파싱하는 동안만 X-Mobility 레포로 cd, `train` · `evaluate_observation` · `evaluate_prediction` · `finetune` 만 건너뜀 | `XmModel.__init__` | `e2e/eval_offline.py`:79 · 328-340 (= `e2e/finetune.py`:38 · 126-132, 거기에 우리 gin 의 `finetune.*` 때문에 `finetune` 추가) |
| `XMobilityTrainer()` 로 만들고 `state_dict` 만 `strict=False` 로 넣음, missing 있으면 멈춤, `model.semantic_decoder.*` unexpected 는 정상 | `XmModel._load_weights` | `e2e/eval_offline.py`:77 · 343-361 (= `e2e/finetune.py`:135-154) |
| X-Mobility import 순서 · `trainer.model.to(device).eval()` · `IS_GWM_PRETRAIN` 거부 · 확산 정책 noise 한 번 뽑아 고정 | `XmModel.__init__` | `e2e/eval_offline.py`:419-441 · `onnx_conversion.py`:41-47 |
| 한 프레임 batch — image (1,1,3,320,512) · route (1,1,19,4) · speed (1,1,1) · action (1,6) · history (1,1024) · sample (1,512) | `XmModel.step` | `e2e/eval_offline.py`:302-325 · `onnx_conversion.py`:83-88 |
| forward = `model.inference(batch, enable_semantic_inference=False, …)` → action · path · history · sample | `XmModel.step` | `x_mobility.py`:130-176 (`XMobilityTrainer.inference` 는 이것을 부름, `trainer.py`:61-63) |
| 영상: 줄 끝 채움 잘라 (H, W, 3) | `core.image_to_hwc` | `sim/export_xmobility.py`:234 |
| 영상: Pillow BILINEAR 512×320 → JPEG q90 → 디코드 | `core.preprocess_rgb` | `sim/export_xmobility.py`:49-50 · 236, 읽기 `isaac_sim_dataset.py`:322-324 · `e2e/eval_offline.py`:98-104 |
| 영상: HWC → CHW, float32 /255 | `core.preprocess_rgb` | `e2e/eval_offline.py`:104 · `navigator.py`:292-293 |
| 경로(topic): 빈 경로 무시, frame → `base_link` tf(`lookup_transform(base_link, frame, Time())`), 앞 20점 · 끝점 반복 | `_on_route` · `_route_points` · `core.select_route_points` | `navigator.py`:138-156 |
| 경로 → (19, 4) `[x_i, y_i, x_i+1, y_i+1]` | `core.route_vectors` | `navigator.py`:164-170 = `e2e/eval_offline.py`:86-95 = `isaac_sim_dataset.py`:326-337 |
| 경로(csv): 로봇 위치 + 기준선 사영 s 앞 0.5 m 간격 19점(목표 s 에서 끊음), 사영 힌트 back 1 · fwd 10 m, 로봇 좌표로 | `core.build_route_poses` · `core.Poly` · `core.to_robot` | `sim/export_xmobility.py`:51 · 69-103 · 118-121 · 172 · 238-246 |
| 기준선 CSV 읽기(run 0), 거꾸로 = 같은 출발점 반대 방향 | `core.load_route_csv` | `sim/export_xmobility.py`:106-108 · 162-163 |
| 정답 위치: `/ground_truth/tf` 에서 `World → nova_carter` 만 | `_on_gt` | `sim/export_xmobility.py`:201-204 · `topics.md`:39 |
| odom + 시작 자세 → 지도 자세, 시작 자세 기본 = 기준선 YAML `start` | `core.odom_to_map` · `_setup_csv_route` | `sim/export_xmobility.py`:211-213 · `topics.md`:22-23 · `eval/routes/parking_r3_cw.yaml`:10 |
| ego_speed = odom `twist.twist.linear.x`(최신 값, 동기화 없음) | `_on_odom` | `navigator.py`:128-130 · 217 · `e2d_model.md`:13 |
| 0.2 s 타이머 | `startup` | `navigator.py`:94, 학습 5 Hz `sim/export_xmobility.py`:48 |
| history · sample 이어 받기, 이전 명령 = 직전 예측 6칸(`prev_action_mode` 기본 `own`) | `XmModel.step` · `_tick` · `core.prev_action_feed` | `navigator.py`:242 · 265-268 · `e2e/eval_offline.py`:247 |
| 리셋 = history · sample · 이전 명령 0 | `_reset` · `XmModel.reset` | `e2e/eval_offline.py`:236-237 · 297-300 · `navigator.py`:101-104 · 135-136 |
| v = action[0], ω = action[5] | `_tick` | `navigator.py`:272-273 |
| 상한 0 ≤ v ≤ 0.8, \|ω\| ≤ 0.7, 유한값 아니면 정지 + 리셋 | `core.clamp_cmd` | `topics.md`:61 |
| 시작 전 발견 대기 뒤 `/cmd_vel` 발행자가 있으면 거부(발행자는 확인 뒤에 만듦), 돌다가 다른 발행자가 생기면 종료 | `_check_cmd_publishers` · `_report` | `topics.md`:40 · 63 · `sim/teleop_ramp.py`:190-199 |
| 끌 때 정지 명령 5번(20 ms 간격), rclpy 신호 처리 끄고 SIGTERM 도 같은 길 | `stop` · `main` | `sim/teleop_ramp.py`:188 · 257-259 · `fd_sim_warmup/lidar_wander.py`:94-112 |
| 입력이 끊기면 정지 명령 | `_tick` | `fd_sim_warmup/lidar_wander.py`:81-86 (워치독) |
| `/clock` 이 멈춰 시뮬 시간 tick 이 안 돌면 실제 시간 타이머(0.5 s)가 정지 명령 — spin 을 깨워 Ctrl+C 도 받음 | `_wall_watchdog` | (여기서 더함) |
| 끊겼다 재개하면 리셋 | `_tick` | 학습 run 은 0.3 s 넘게 끊기면 나뉨(`sim/export_xmobility.py`:53 · 264-273) → run 마다 리셋(`e2e/eval_offline.py`:236-237) |
| odom 시각이 0.5 s 넘게 뒤로 / 노드 시계 역행 → 리셋(Stop → Play) | `_on_odom` · `_tick` | `sim/teleop_ramp.py`:205-206 · `topics.md`:58 |

## 배포 노드(navigator.py) · 오프라인 평가와 다른 점

- **영상 크기 맞추기**: navigator 는 TensorRT 엔진 안에서 `F.interpolate(bilinear, align_corners=False)`(onnx_conversion.py:53-58)로 줄입니다. 학습 영상은 Pillow BILINEAR(줄일 때 안티에일리어싱) + JPEG q90 이라 값이 다릅니다. 여기서는 Pillow + JPEG 를 그대로 해서 모델에는 이미 320×512 가 들어갑니다(`XmModel.step` 은 크기가 다르면 다시 줄이지 않고 오류).
- **경로 tf 시점**: navigator 는 `/route` 를 받은 순간 tf 로 바꿔 둡니다(navigator.py:139-170). 여기서는 받은 점을 원래 frame 으로 두고 **매 tick 최신 tf 로** 바꿉니다(지도 · odom 좌표 경로면 그 사이 움직임이 반영됨). frame 이 `base_link` 면 둘이 같습니다.
- **리셋 범위**: navigator 는 새 목표 때 history · sample 만 0, 이전 명령은 그대로(navigator.py:132-136). 여기서는 이전 명령도 0(eval_offline.py:236-237 · 학습 run 시작과 같음).
- **경로 점프 리셋(여기서 더함)**: 새 경로의 점마다 직전 경로(폴리라인)까지 거리의 최댓값이 `route_jump_m` 를 넘으면 리셋. 같은 좌표계에서 비교 — `base_link` 경로는 받은 순간의 odom 자세로 odom 좌표로 바꿔 비교. 로봇이 나아가 경로 창이 미끄러지는 것은 걸리지 않음(시험: 0.8 m/s · 5 Hz 한 바퀴에서 최대 0.21 m). 목표가 같은 경로 위 앞쪽으로만 바뀌면(경로가 짧아지기만 하면) 걸리지 않음 — 필요하면 `goal_topic` 이나 `~/reset`.
- **입력 끊김 · 준비 안 됨**: navigator 는 아무것도 안 내고 return(navigator.py:218-220). 여기서는 매 tick 정지 명령(Isaac Sim 은 마지막 명령을 계속 따름, teleop_ramp.py:257 주석). 시뮬 시간 tick 자체가 멈추면 실제 시간 워치독이 냄.
- **디버그 경로**: navigator 의 `/x_mobility_path` 는 입력 경로 구간의 시작점들(navigator.py:276-286). 여기서는 `~/route`(입력 20점)와 `~/path`(모델 출력 5점)를 따로 냅니다. 출력 경로는 Tanh 라 ±1 m 안입니다(커밋 64b117a 메모, action_policy.py:106-115).
- **정밀도**: 학습은 `16-mixed`(finetune_parking_r3.gin:13), 추론은 float32(eval_offline 과 같음).
- **ego_speed 시점**: 학습은 영상 시각에 odom 속도를 보간(export_xmobility.py:252), 여기는 tick 때 최신 odom(60 Hz 이상이라 차이 ≤ 17 ms).

## 확인한 것 · 안 한 것

확인(ROS · GPU 없는 컨테이너, X-Mobility 코드 import · 실행 없음):

- `core.py` 단위 시험 21개(python 3.13 · numpy 2.5 · Pillow 12.3 과 Pillow 11.3 두 환경 통과). 그중 학습 · 평가 코드와의 대조(아래 영상 · 경로 · 각도 · 상한)는 `test/test_core.py` 로 레포에 둠 — `python -m pytest ros2_ws/src/fd_xm_infer/test -q`(레포 루트에서. numpy · Pillow · pandas(`e2e/eval_offline.py` 가 import) 만, ROS · torch 불필요. pytest 없으면 `python ros2_ws/src/fd_xm_infer/test/test_core.py`). 노드를 고친 뒤 다시 돌려 "비트 단위 같음" 을 확인:
  - 영상: 학습 길(`export_xmobility.py`:234-236 옮겨 적음 → `eval_offline.decode_image`)과 **비트 단위 같음**(줄 끝 채움 0 · 8 · 64 B 포함). `float32 /255` 와 로더의 `float64 /255 → float32` 는 0~255 전부 같음. 학습 로더의 같은 크기 `tvf.resize` 는 항등(torchvision 0.29).
  - 경로: `eval_offline.route_vectors` · navigator 식(옮겨 적음)과 같음(1~40점). csv 모드 경로가 `export_xmobility.py` 의 `Poly` · `to_robot`(ast 로 떼어 실행) + 238-246 식과 cw · ccw · cw 거꾸로 × 목표 없음/있음 250자세에서 같음. `parking_r3_ccw.csv` = cw 거꾸로(같은 s 점 거리 0.0 mm).
  - 좌표: `transform_points`(쿼터니언) = `to_robot`(자세의 역 tf), 일반 쿼터니언 = 해밀턴 곱. `quat_yaw` · `odom_to_map` = export 식.
  - 상한 · NaN · 지연 백분위(numpy 기본 linear).
  - 이전 명령 입력(`core.prev_action_feed`, 11.11 · Windows python 3.12 · numpy 1.26): own · zero · clamped · 다른 값 거부, own · clamped 가 예전 `_tick` 식(`feed_clamped_action` false · true)과 비트 단위 같음(무작위 2000개, 범위 밖 값 포함).
- 참고 숫자(합성 640×480 영상): JPEG 왕복 있음/없음 평균 2.2/255 · 최대 81/255 차이. torch bilinear(antialias 없음 — onnx_conversion 길) vs 학습 영상 평균 2.5/255 · 최대 95/255. 전처리 약 5 ms(이 컨테이너 CPU).
- `node.py`: py_compile · flake8(줄 120). rclpy · 메시지 · tf2_ros 를 가짜 모듈로, 모델을 가짜로 바꿔 tick 을 손으로 돌린 연기 시험 12개 — 상한 자르기, 이전 명령 이어 받기, 서비스 리셋, 끊김 → 정지 → 재개 리셋, 시계 역행, tf 없음 → 정지, odom frame 경로의 tf 변환, 경로 점프(회전 보정), NaN, 발행자 거부 · 허용 · 실행 중 감지, csv 모드(odom · 정답 위치, 바퀴 링크 변환 거름, 위치 점프), CSV · meta.json · 끝 정지 5번, 실제 시간 워치독.
- `prev_action_mode`(11.11): 같은 식의 가짜 모듈로 파라미터 조합 16개 × tick 40번(범위 밖 v · ω, NaN 리셋 포함) — 기본 · `own` 과 `feed_clamped_action:=true` · `clamped` 가 바꾸기 전 노드와 이전 명령 입력 · 보낸 명령 · 리셋이 비트 단위 같음, `zero` 는 입력이 늘 0. `zero` + `feed_clamped_action:=true` · `teacher` · `Own` · 빈칸은 시작 때 멈춤(종료 코드 3).

안 한 것:

- **실제 ROS 실행** — 발견 · QoS 호환, `get_publishers_info_by_topic` 결과, 시뮬 시간 타이머가 Stop → Play 에서 어떻게 도는지 · STEADY 시계 워치독이 Pause 중 spin 을 깨우는지, launch `prefix`(ml_env python)로 콘솔 스크립트가 뜨는지, `ParameterValue` 형 변환.
- **실제 모델** — gin 파싱 · 가중치 대조 · `inference` 출력 모양, ml_env 의 Pillow · libjpeg 가 학습 데이터를 만든 `~/xmobility_test/venv` 와 같은 JPEG 를 내는지(다르면 영상이 미세하게 다름).
- **GPU 지연** — 측정 전(위 로그 줄로 채움).
- **tf · frame** — `tools/xm_route.py` 가 `/route` 를 어떤 frame 으로 내는지 모름. `/tf` 에는 `odom → base_link` 만 있고 `map → odom` 은 없음(topics.md:24 · 37) — `base_link` · `odom` 이 아닌 frame 이면 tf 가 없어 정지 명령만 냅니다. 이 카메라 frame 은 `/tf` 에 없음(topics.md:55, 여기서는 안 씀).
- **평가 규칙** — `pose_source=ground_truth` 는 디버그 전용(topics.md:39). 기본 `route_source=topic` 에서 `xm_route.py` 가 정답 위치로 경로를 만든다면 그것도 평가 입력 규칙에 걸리는지 E · D 확인이 필요합니다.
