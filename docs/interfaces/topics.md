# 인터페이스 규격서 — 토픽 · 좌표계 · 단위 · 파일 형식

- **정리**: D 정동현
- **상태**: 초안 (9.29) — 10.2 팀 회의에서 합의
- **근거**: 수행계획서 2.6 "토픽명, 메시지 형식, 좌표계·단위, 파일 형식을 9월 중 문서로 확정"

토픽 이름과 frame_id 는 파트마다 따로 정하지 않고 이 문서 한 곳에 모읍니다. 바꿀 때는 받는 쪽 동의 후 커밋하고 아래 변경 이력에 적습니다.

## 1. 좌표계 · 단위 · 시간

| 항목 | 규약 |
| --- | --- |
| 단위 | SI — m · s · rad (각속도 rad/s). USD 는 `metersPerUnit = 1.0` |
| 위쪽 축 | Z-up |
| 주차장 좌표 (`map`) | 원점 = 주행 루프의 **남서 꼭짓점 노면**(수행계획서 그림 1의 출발·도착점). **ENU** — +X 동, +Y 북, +Z 위 |
| 방향각 | yaw 0 = 동쪽, 반시계가 + |
| 로봇 좌표 | ROS REP 103 — `base_link` 기준 +X 앞, +Y 왼쪽, +Z 위 |
| 카메라 광학 좌표 | REP 103 광학 규약 — +Z 앞, +X 오른쪽, +Y 아래 (`*_optical`) |
| 시간 | 모든 노드 `use_sim_time: true`. 메시지 시각은 시뮬 시간(`/clock`) |

- Proxy 씬 · 캠퍼스 씬 · 시나리오 파일이 모두 `map` 좌표를 씁니다. 캠퍼스 씬 메쉬는 C 가 이 좌표로 옮겨 넘깁니다([`c2d_usd.md`](c2d_usd.md)).
- `odom` 원점은 로봇이 Play 된 자리(시나리오 시작 자세)입니다. `map` 좌표가 필요하면 시작 자세를 더합니다. `map → odom` 정적 변환이 필요해지면 D 가 씬에서 발행합니다.

## 2. 토픽

시뮬레이터(Isaac Sim, Nova Carter)가 내는 토픽은 로봇 에셋의 이름을 그대로 씁니다.

| 토픽 | 타입 | 누가 | frame_id | 주기 | 값 |
| --- | --- | --- | --- | --- | --- |
| `/front_camera/image_raw` | `sensor_msgs/Image` | 시뮬 | `front_camera_optical` | 목표 20 Hz · 최소 10 Hz (실측 약 14 Hz) | **640×480** `rgb8` (수행계획서 2.3) |
| `/front_camera/camera_info` | `sensor_msgs/CameraInfo` | 시뮬 | 같음 | 영상과 같음 | `plumb_bob` |
| `/chassis/odom` | `nav_msgs/Odometry` | 시뮬 | `odom` → `base_link` | 약 55 Hz | **현재 속도** = `twist.twist.linear.x`(v) · `twist.twist.angular.z`(ω) |
| `/chassis/imu` | `sensor_msgs/Imu` | 시뮬 | `chassis_imu` | 약 55 Hz | |
| `/tf` | `tf2_msgs/TFMessage` | 시뮬 | `odom` → `base_link` → 센서 | | |
| `/clock` | `rosgraph_msgs/Clock` | 시뮬 | — | 시뮬 스텝마다 (실측 약 74 Hz) | |
| `/cmd_vel` | `geometry_msgs/Twist` | 교사 리그 **또는** 추론 노드 — **발행자는 항상 1개** | — | 10 Hz 이상 | `linear.x` = v (m/s) · `angular.z` = ω (rad/s). 나머지 성분은 0 |
| `/route_command` | `std_msgs/UInt8` | 교사 리그 · 평가 스크립트 | — | 10 Hz | **0 직진 · 1 좌회전 · 2 우회전** — 제안, E 확정 |
| `/front_3d_lidar/lidar_points` | `sensor_msgs/PointCloud2` | 시뮬 | `front_3d_lidar` | 약 6.5 Hz | 수집·평가 설정에서는 끔 |

- **주기는 9.29 서버 헤드리스 실측(실제 시간 기준)입니다.** 이때 RTF(시뮬 시간 ÷ 실제 시간)는 0.90이었습니다. 사람이 모는 모니터 모드에서는 다시 잽니다.
- **E2E 카메라는 전용 핀홀 카메라입니다.** Hawk 왼쪽 렌즈 자리(바닥에서 약 0.35 m), 수평 화각 90°(Hawk 와 같음), 4:3. 에셋의 Hawk 스테레오는 어안 투영·16:10 이라 640×480 으로 줄이면 왜곡되고 눌려서 쓰지 않고, 렌더도 끕니다(9.29 화면 비교). 이 카메라의 frame 은 `/tf` 에 없습니다 — 필요해지면 추가합니다.
- **확인 필요 — `/chassis/odom` 축 방향**: 9.29 직진 시험에서 물리 엔진 기준으로는 북쪽으로 8.8 m 갔는데, odom 은 (+8.79, +2.33) m 로 나왔습니다. odom 좌표가 시작 자세 기준인지, 평가 스크립트에서 `map` 으로 바꿀 때 어떻게 계산할지 확인한 뒤 적습니다.
- **속도 상한 제안**: 0 ≤ v ≤ 0.8 m/s, |ω| ≤ 1.0 rad/s. 교사 리그 실행 파일에 고정합니다(수행계획서 2.3 "속도 상한은 실행 파일 하나로 고정"). 0.8 m/s 는 Nova Carter 공식 Nav2 설정의 최대 속도와 같습니다.
- **발행자 1개 규칙**: 새 노드를 띄우기 전에 `ros2 topic info /cmd_vel` 의 `Publisher count` 가 0 인지 봅니다.
- 교사 데이터 기록(10 fps)은 영상과 `/cmd_vel` 의 (v, ω)를 시뮬 시간으로 맞춰 샘플링합니다 — 기록 방식은 E 가 [`d2e_teacher.md`](d2e_teacher.md)에 정합니다.
- 서버 ROS 설정: `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`([`../server207.md`](../server207.md)).

## 3. 파일 형식

| 무엇 | 형식 | 위치 |
| --- | --- | --- |
| 씬 | USD 텍스트 레이어 `.usda`(m, Z-up). 메쉬 바이너리는 `.usdc` payload | 레이어는 `sim/scenes/`(추적), 바이너리는 `data/usd/<이름>_v<버전>/`(무시 → [`MANIFEST.md`](../../MANIFEST.md)) |
| 센서 원본 · 교사 원본 | rosbag2 **MCAP** | `data/raw/rosbag/` · `data/teacher/` |
| 학습용 데이터 | E 가 정함(HDF5 예정) | `data/dataset/` |
| 경로 지시 · 시나리오 | YAML, 좌표는 `map` | `eval/routes/` · `eval/scenarios/`(추적) |
| 모델 | E 가 정함 | `data/checkpoints/` |
| 평가 결과 | CSV + 주행 영상 | `data/eval/` |

## 변경 이력

| 날짜 | 변경 | 동의 |
| --- | --- | --- |
| 9.29 | 초안 — Proxy 씬 실측 frame_id · 주기 반영 | |
| 9.29 | E2E 카메라를 전용 핀홀 `/front_camera/*` 로 변경, `/clock` 추가, 실측 주기·RTF | |
