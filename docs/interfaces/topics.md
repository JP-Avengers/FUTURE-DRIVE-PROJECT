# 인터페이스 규격서 — 토픽 · 좌표계 · 단위 · 파일 형식

- **정리**: D 정동현
- **상태**: 초안 2판 (9.30) — 10.2 팀 회의가 취소돼 팀 채널에서 비대면으로 확인합니다. 10.6 1차 수집 전에 E 동의를 받습니다
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
- **`odom` 은 시작 자세 기준입니다.** 원점은 로봇이 Play 된 자리(시나리오 시작 자세 (x0, y0, ψ0))이고, +x 는 그때 로봇이 바라본 방향, yaw 는 0 에서 시작합니다. `map` 좌표는 시작 자세를 더해 구합니다:
  `x = x0 + cos ψ0 · ox − sin ψ0 · oy`, `y = y0 + sin ψ0 · ox + cos ψ0 · oy`, `ψ = ψ0 + (odom yaw)`
  Proxy 씬의 기본 시작 자세는 (−0.3, 2.0), ψ0 = 97.3° 입니다(`sim/scenes/proxy/params.yaml` 의 `robot.start`). 9.30 루프 한 바퀴 주행 3회(양방향)를 이 식으로 풀어 중심선 이탈 최대 0.44 m 로 완주해 확인했습니다(9.29 의 어긋난 (+8.79, +2.33) m 는 직진 출발 흔들림 탓). `map → odom` 정적 변환이 필요해지면 D 가 씬에서 발행합니다.
- odom 의 z 는 시작 자세가 0.09° 기울어 있어 멀리 갈수록 조금씩 변합니다(루프에서 최대 −3.8 cm, odom x 에 정확히 비례). 실제 높이 변화가 아니니 높이 판정에 쓰지 않습니다.

## 2. 토픽

시뮬레이터(Isaac Sim, Nova Carter)가 내는 토픽은 로봇 에셋의 이름을 그대로 씁니다.

| 토픽 | 타입 | 누가 | frame_id | 주기 | 값 |
| --- | --- | --- | --- | --- | --- |
| `/front_camera/image_raw` | `sensor_msgs/Image` | 시뮬 | `front_camera_optical` | 목표 20 Hz · 최소 10 Hz (9.30 실측 20.0 Hz) | **640×480** `rgb8` (수행계획서 2.3) |
| `/front_camera/camera_info` | `sensor_msgs/CameraInfo` | 시뮬 | 같음 | 영상과 같음 | `plumb_bob` |
| `/chassis/odom` | `nav_msgs/Odometry` | 시뮬 | `odom` → `base_link` | 80~84 Hz — 같은 시각 메시지가 섞임, 고유 시각은 60 Hz(아래) | **현재 속도** = `twist.twist.linear.x`(v) · `twist.twist.angular.z`(ω) |
| `/chassis/imu` | `sensor_msgs/Imu` | 시뮬 | `chassis_imu` | odom 과 같음 | |
| `/tf` | `tf2_msgs/TFMessage` | 시뮬 | `odom` → `base_link` → 센서 | odom 한 번에 메시지 4개 | |
| `/clock` | `rosgraph_msgs/Clock` | 시뮬 | — | 시뮬 스텝마다 — odom 과 같음 | |
| `/cmd_vel` | `geometry_msgs/Twist` | 교사 리그 **또는** 추론 노드 — **발행자는 항상 1개** | — | 10 Hz 이상 | `linear.x` = v (m/s) · `angular.z` = ω (rad/s). 나머지 성분은 0 |
| `/route_command` | `std_msgs/UInt8` | 교사 리그 · 평가 스크립트 | — | 10 Hz | **0 직진 · 1 좌회전 · 2 우회전** — 제안, E 확정 |
| `/front_3d_lidar/lidar_points` | `sensor_msgs/PointCloud2` | 시뮬 | `front_3d_lidar` | 약 6.5 Hz | 수집·평가 설정에서는 끔 — `sim/run_proxy.sh` 가 기본으로 끔(`FD_LIDAR=1` 로 켬) |

- **주기는 시뮬 시간(메시지 헤더 시각) 기준입니다** — 9.30 서버 실측, 물리 설정을 고친 뒤(CPU 물리 · 스텝 60 Hz, 팀 레포 `ea4cf1c`)입니다. 실제 시간 기준 주기는 RTF(시뮬 시간 ÷ 실제 시간)만큼 달라지고, RTF 는 실행 모드마다 다릅니다.

  | 실행 | RTF (9.30) |
  | --- | --- |
  | 헤드리스, `sim/run_proxy.sh`(LiDAR 끔) | 약 1.7 |
  | 헤드리스, LiDAR 켬(샘플 v0.2 · 루프 주행 시험) | 1.27~1.32 |
  | 스트리밍, `sim/run_proxy.sh`(클라이언트 없음) | 0.95~0.98 — 화면 한 프레임에 시뮬 한 스텝이 묶임 |
  | 스트리밍 + 클라이언트 연결(1차 수집) | 0.97~0.99 — 10.1 클라이언트를 붙인 15분 동안의 30초 기록. 서버 앞 모니터 모드는 이 서버에서 못 씀(10.1) |
- **E2E 카메라는 전용 핀홀 카메라입니다.** Hawk 왼쪽 렌즈 자리(바닥에서 약 0.35 m), 수평 화각 90°(Hawk 와 같음), 4:3. 에셋의 Hawk 스테레오는 어안 투영·16:10 이라 640×480 으로 줄이면 왜곡되고 눌려서 쓰지 않고, 렌더도 끕니다(9.29 화면 비교). 이 카메라의 frame 은 `/tf` 에 없습니다 — 필요해지면 추가합니다.
- **주의 — odom · IMU · `/clock` 에 같은 시각 메시지가 섞입니다(9.30).** 시뮬 스텝은 60 Hz(16.7 ms)인데 세 토픽 모두 80~84 Hz 로 나오고, 25~29 % 는 헤더 시각이 앞 메시지와 같습니다(고유 시각만 세면 60 Hz, 간격 16.7 ms 일정). 원인은 아직 모르지만 흔들림과는 무관합니다(9.30 확인). 10 fps 동기화 · 평가 스크립트에서 헤더 시각 기준으로 중복을 거를지 **E 확인이 필요합니다.**
- **주의 — 카메라 프레임이 빌 수 있습니다.** 9.30 샘플 v0.2 에서 0.48 s(약 9프레임) 빈 곳이 한 번 있었습니다(녹화 3.3 s 지점, 원인 미확인). 교사 데이터 QC 에서 카메라 간격 최댓값을 봅니다.
- **주의 — Stop → Play 하면 시뮬 시간이 0 으로 돌아갑니다(10.1).** 로봇은 출발점으로, odom 은 (0, 0) 으로 돌아가고 `/clock` 도 0 에서 다시 시작합니다. 한 녹화 파일에 Stop → Play 가 섞이면 시각이 뒤로 가므로, 기록은 Stop 전에 끊고 Play 뒤 새 파일로 시작합니다 — **E 확인이 필요합니다.**
- **주의 — 실제 회전이 명령 ω 보다 약 1.3배 빠릅니다(9.30).** ω = ±0.2 명령에 odom ωz 는 ±0.262 였고, 루프 주행에서도 명령이 일정한 구간은 1.31~1.35배였습니다. v 는 명령대로(0.50)입니다. 원인은 확인 중입니다. 교사 라벨은 명령값이라 학습에는 문제가 없고, 명령 추종이나 궤적을 비교할 때 감안합니다.
- **규격 밖 토픽**: 로봇 에셋이 스테레오 IMU 4개(`/front_stereo_imu/imu` · `/back_stereo_imu/imu` · `/left_stereo_imu/imu` · `/right_stereo_imu/imu`)를 더 냅니다. 기록하지 않습니다. `/tf_static` 은 나오지 않습니다.
- **속도 상한 제안**: 0 ≤ v ≤ 0.8 m/s, |ω| ≤ 1.0 rad/s. 교사 리그 실행 파일에 고정합니다(수행계획서 2.3 "속도 상한은 실행 파일 하나로 고정"). 0.8 m/s 는 Nova Carter 공식 Nav2 설정의 최대 속도와 같습니다.
- **발행자 1개 규칙**: 새 노드를 띄우기 전에 `ros2 topic info /cmd_vel` 의 `Publisher count` 가 0 인지 봅니다. `sim/run_proxy.sh teleop` 은 이것을 확인한 뒤에만 키보드 조작을 띄웁니다.
- 교사 데이터 기록(10 fps)은 영상과 `/cmd_vel` 의 (v, ω)를 시뮬 시간으로 맞춰 샘플링합니다 — 기록 방식은 E 가 [`d2e_teacher.md`](d2e_teacher.md)에 정합니다.
- 서버 ROS 설정: `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`([`../server207.md`](../server207.md)). 씬은 `sim/run_proxy.sh` 로 띄웁니다 — 이 설정과 GPU 0 · LiDAR 끔을 고정합니다([`sim/README.md`](../../sim/README.md)).

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
| 9.30 | 9.30 실측(시뮬 시간 기준) — 카메라 20 Hz · odom/IMU/`/clock` 80~84 Hz(고유 60 Hz) · 모드별 RTF. odom 축 확정(시작 자세 기준, 루프 주행 3회)과 `map` 변환식, odom z 주의. 주의 3건(같은 시각 메시지 · 카메라 빈 구간 · 회전 약 1.3배)과 규격 밖 토픽. LiDAR 기본 끔(`sim/run_proxy.sh`). 상태: 비대면 확인 | |
| 10.1 | 1차 수집 실행을 스트리밍으로(서버 앞 모니터 모드는 이 서버에서 못 씀) — RTF 표 | |
| 10.1 | 스트리밍 + 클라이언트 RTF 0.97~0.99, 주의 추가 — Stop → Play 하면 시뮬 시간이 0 으로 | |
