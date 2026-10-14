# sim/ — Isaac Sim

- **담당**: D · 정동현 (PM)
- **목적**: Isaac Sim 씬을 구성하고 가상 센서·ROS 2 Bridge·교사 데이터 리그·도메인 랜덤화를 제공한다.

## 간트 작업

| 작업 | 기간 |
| --- | --- |
| Proxy 씬 · 로봇 · 센서 · ROS 2 Bridge | 09.29–10.11 |
| 교사 데이터 기록 리그 | 10.05–10.18 |
| 씬 통합 · 도메인 랜덤화 (Proxy 기반) | 10.12–11.02 |
| 실제 메쉬 교체 | 11.02–11.16 |

## 입력 · 출력

- **받는 것**: C 의 USD · [`c2d_usd.md`](../docs/interfaces/c2d_usd.md)
- **넘기는 것**: 교사 데이터 → E · [`docs/interfaces/d2e_teacher.md`](../docs/interfaces/d2e_teacher.md)

## 환경

server207 · Isaac Sim 6.0.1 (pip, Python 3.12) · GPU 0 고정 — [`docs/server207.md`](../docs/server207.md). ROS 패키지는 [`ros2_ws/src/`](../ros2_ws/src)에 만듭니다.

> **`.usda`(텍스트)로 저장해야 추적됩니다.** `.usd`·`.usdc` 는 `.gitignore` 대상입니다.

> **Isaac Sim 안에서는 `cv_bridge` 를 쓰지 않습니다.** apt `cv_bridge` 는 numpy 1.x 로 빌드돼, Isaac Sim(numpy 2.3.1) 안에서 색 변환(`imgmsg_to_cv2(…, desired_encoding="bgr8")` 등)을 부르면 **Isaac Sim 이 세그폴트로 죽고**, import 만 해도 로그에 `[Error]` 줄이 찍힙니다(9.28 확인). 카메라 발행은 Isaac Sim 의 ROS 2 카메라 그래프로, 파이썬에서 직접 만들 땐 numpy 로 `sensor_msgs/Image` 필드를 채우고, cv_bridge 가 필요한 변환·추론은 `ml_env` 로 도는 [`ros2_ws/src/`](../ros2_ws/src) 노드에서 합니다. `sim/` 아래 `.py` 의 `cv_bridge` import 는 pre-commit 이 막습니다.

## 실행 방법

**헤드리스 기동 검증** (server207, 레포 루트 = `$CD1`에서):

```bash
OMNI_KIT_ACCEPT_EULA=YES $CD1/envs/isaacsim_env/bin/python sim/check_headless.py   # "OK" 후 종료 코드 0
```

**원격 화면 스트리밍** (Tailscale 경유 — [docs/server207.md](../docs/server207.md)):

```bash
setsid nohup sim/run_streaming.sh > logs/streaming.log 2>&1 < /dev/null & echo $! > logs/streaming.pid
# 로그에 "app ready" 가 뜨면 WebRTC Streaming Client 의 Server 칸에 서버 Tailscale IP → Connect
kill $(cat logs/streaming.pid)     # 종료 — 공용 서버이니 작업이 끝나면 반드시
```

스트리밍 중에는 NvStreamer 트레이스 로그(`*.etli`)가 약 3분마다 쌓입니다(시간당 ~140MB). 스크립트가 `logs/etli/` 에서 실행하므로 그 폴더에 모이고, git 은 무시합니다. 가끔 비워 주세요: `du -sh logs/etli`

**ROS 2 Bridge 검증** (헤드리스로 `/clock` 발행 → 호스트에서 수신):

```bash
source /opt/ros/jazzy/setup.bash        # 반드시 Isaac Sim 기동 **전에** — 안 하면 내장 라이브러리로 넘어가 Bridge 실패
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
OMNI_KIT_ACCEPT_EULA=YES $CD1/envs/isaacsim_env/bin/python sim/check_ros2_bridge.py 60
# 다른 터미널: ros2 topic echo /clock rosgraph_msgs/msg/Clock --once
```

GPU 0 고정은 스크립트 안의 `active_gpu=0`·`multi_gpu=False`로 합니다. **`CUDA_VISIBLE_DEVICES`는 걸지 마세요** — 종료 시 abort 가 납니다(9.27 실측).

## Proxy 씬

수행계획서 그림 1의 루프(IT대학 1호관 남측, 약 119 m)를 박스와 평면으로 흉내 낸 대역 씬입니다. 캠퍼스 Digital Twin 이 나오는 11.16 까지 씁니다.

```bash
$CD1/envs/isaacsim_env/bin/python sim/build_proxy_scene.py      # params.yaml → scenes/proxy/*.usda (몇 초, Isaac Sim 불필요)
OMNI_KIT_ACCEPT_EULA=YES $CD1/envs/isaacsim_env/bin/python sim/author_proxy_layers.py   # ros·sensors 레이어 처음 채우기 (/clock · E2E 카메라)
source /opt/ros/jazzy/setup.bash && OMNI_KIT_ACCEPT_EULA=YES $CD1/envs/isaacsim_env/bin/python sim/author_ground_truth.py   # ros 레이어에 정답 위치 /ground_truth/tf 더하기(10.13, 이미 있으면 그대로) — 모든 씬에 들어감
# 헤드리스로 열어 로봇·토픽 확인 — 위 Bridge 검증처럼 시스템 ROS 를 먼저 source 한 셸에서
OMNI_KIT_ACCEPT_EULA=YES $CD1/envs/isaacsim_env/bin/python sim/check_proxy_scene.py 25 --drive 0.5   # RTF · 직진 거리, 끝에 "OK"
```

- **치수는 `scenes/proxy/params.yaml` 만 고칩니다.** `proxy_parking`(루트)·`world`·`env_proxy`·`background`·`robot` 레이어는 생성물이라 손으로 고치지 않습니다. `sensors`·`ros` 는 GUI 로 고치는 레이어라 스크립트가 덮어쓰지 않습니다.
- **E2E 카메라는 전용 핀홀 640×480** 입니다(`/front_camera/*`, Hawk 왼쪽 렌즈 자리). Hawk 스테레오는 어안 투영이라 렌더를 끕니다. 토픽 규격은 [`docs/interfaces/topics.md`](../docs/interfaces/topics.md).
- 캠퍼스 씬으로 바꿀 때는 `env_proxy` 자리만 C 의 USD 로 바뀌고, 나머지 레이어는 그대로 씁니다.
- 좌표: 원점 = 루프 남서 꼭짓점 노면, +X 동 · +Y 북 · +Z 위(ENU), m.
- v0 치수는 그림 1 지도를 축척으로 잰 추정치(±1~2 m)입니다. A 의 기준 치수 실측표가 오면 바꿉니다.
- 로봇은 NVIDIA 에셋 서버의 `Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd`(2026-06-03 수정본)를 참조합니다. 인터넷 연결이 필요합니다.

### 실행 — `sim/run_proxy.sh`

스트리밍 · 헤드리스가 같은 설정으로 뜹니다 — GPU 0 고정, 시스템 Jazzy 먼저 source, `ROS_DOMAIN_ID=42`, 3D LiDAR 끔, 씬 열고 Play. LiDAR 끄기는 세션 레이어에만 쓰므로 GUI 에서 저장해도 씬 파일에는 남지 않습니다. GPU 에 계산 프로세스가 하나라도 있으면 띄우지 않습니다(공용 서버).

| 모드 | 어디서 | 명령 |
| --- | --- | --- |
| 스트리밍 — 1차 수집 | ssh → `tmux new -s fd_proxy` 안에서. 화면은 Mac 의 스트리밍 클라이언트(Server 칸에 서버 Tailscale IP) | `sim/run_proxy.sh stream` |
| 헤드리스 | ssh → `tmux new -s fd_proxy` 안에서 | `sim/run_proxy.sh headless [초]` — 초를 안 주면 `Ctrl+C` 까지 |
| 점검 | 씬이 Play 된 뒤 다른 터미널 | `sim/run_proxy.sh check [초]` — 토픽 · `/cmd_vel` · RTF · 주기를 재서 항목별 OK/NG |
| 키보드 조작 | 다른 터미널 | `sim/run_proxy.sh teleop [옵션]` — 가감속 램프(`sim/teleop_ramp.py`, 키는 실행하면 나오는 안내문). `/cmd_vel` 발행자가 0 일 때만 뜸. `FD_TELEOP=stock` 이면 예전 teleop_twist_keyboard |

- **끄기**: 띄운 터미널에서 `Ctrl+C`(tmux 밖에서는 `tmux send-keys -t fd_proxy C-c`). 끝에 `[Error]` 줄 수와 GPU 상태를 찍습니다. 로그는 `logs/run_proxy/<모드>_<시각>.log`. 스트리밍 클라이언트를 붙인 채 끄면 끝에 `NVST_R_BUSY` `[Error]` 가 몇 줄 찍힙니다(10.1) — 씬 오류는 아니지만, 클라이언트에서 먼저 Disconnect 하고 끕니다.
- **스트리밍 화면**: 뷰포트가 로봇 뒤 3인칭 카메라(`ChaseCam`)로 바뀝니다 — 운전자용이고 기록에는 안 씁니다. 화면의 Stop → Play 로 출발점에 돌아갑니다. 이때 시뮬 시간(`/clock`)과 odom 도 0 에서 다시 시작하니, 녹화는 Stop 전에 끊고 Play 뒤 새로 시작합니다(10.1). 조작 키는 `teleop` 을 띄운 ssh 터미널에 눌러야 합니다 — 스트리밍 화면을 클릭하면 키가 Isaac Sim 으로 갑니다.
- **서버 앞 모니터로는 띄우지 않습니다**(10.1 확인). 데스크톱 화면이 GPU 1 에만 나오는데 렌더를 GPU 0 에 고정하면 창을 못 그립니다(`createSwapchain failed`). `sim/run_proxy.sh monitor` 는 안내만 찍고 끝납니다.
- **환경변수**: `FD_LIDAR=1` LiDAR 켬(점검도 같은 값으로) · `FD_DOMAIN=43` 시험용 도메인 · `FD_VIEW_RES=1280x720` 뷰포트 해상도(스트리밍 — RTF 가 낮을 때 줄임).
- `run_streaming.sh` 는 씬 없이 빈 앱을 스트리밍할 때 씁니다.

### 교사 데이터 녹화 — `sim/record_teacher.sh`

규격서 토픽 전부(카메라 · `camera_info` · odom · IMU · `/tf` · `/clock` · `/cmd_vel` · `/route_command`)를 rosbag2 MCAP 으로 시뮬 시간 기준으로 남깁니다. **구독만 하므로 `/cmd_vel` 발행자(teleop) 수는 그대로 1개입니다.** 10 fps 샘플링 · 학습용 변환(HDF5)은 하지 않고 원본을 남깁니다 — 방식은 E 가 [`d2e_teacher.md`](../docs/interfaces/d2e_teacher.md) 에 정합니다.

```bash
# 1차 수집 순서 — 터미널 셋(모두 tmux 안)
sim/run_proxy.sh stream                   # ① 씬 (Mac 스트리밍 클라이언트로 봄)
sim/run_proxy.sh teleop                   # ② 키보드 조작(가감속 램프)
sim/record_teacher.sh <운전자> <cw|ccw> [메모]   # ③ 녹화 — Ctrl+C 로 끝내면 바로 점검
```

- **저장**: `data/teacher/raw_v1/<월일_시분초>_<운전자>_<방향>/` — MCAP · `meta.yaml`(운전자 · 방향 · 레포 커밋 · 씬 params 해시) · `qc.txt`. Git 에 안 올리고 `MANIFEST.md` 에 묶음 단위로 기록합니다.
- **Stop → Play 전에 녹화를 끕니다.** 시뮬 시간이 0 으로 돌아가 한 파일에 섞이면 점검이 NG 를 냅니다. Play 뒤 새로 띄웁니다.
- **압축**: 기본 `zstd_fast` — 10.12 헤드리스 시험에서 시뮬 1분에 0.19 GB(압축 없는 9.30 샘플은 1.14 GB). 1.5~2시간이면 약 17~23 GB. `FD_REC_COMPRESS=0` 이면 끔.
- **점검** `sim/qc_teacher.py <녹화 폴더>` — 따로 돌려도 됩니다. 카메라 장 수 · Hz · 최대 간격(헤더 시각, 0.5 s 초과 NG), `/clock` 이 뒤로 갔는지(NG), odom 같은 시각 비율, `/cmd_vel` 상한(v 0.8 · ω 0.7 초과 NG), 크기를 찍습니다. 9.30 샘플의 "0.48 s 빈 구간"은 수신 시각 간격이었고 헤더 시각은 이어져 있었습니다(`cam_rx` 줄).
- 서버 작업은 D 만 합니다. 운전은 D 가 연결한 D 의 Mac 에서 D 입회로 교대합니다.

### 경로 추종 주행 — `sim/pursuit.py`

규칙 기반 pure pursuit 노드입니다. `/chassis/odom` 만 받아 루프 중심선(`params.yaml` 의 `loop`, 약 119 m)에서 `--lookahead` 앞 점을 따라가도록 `/cmd_vel` 을 냅니다. **E2E 가 아니고 장애물도 피하지 않습니다**(카메라 · LiDAR 를 안 봄). 9.30 루프 주행 시험(양방향 완주, 최대 이탈 0.44 m)과 10.6 시연 영상에 썼습니다. 경로 · 좌표 변환은 `sim/looppath.py`.

```bash
# 씬이 Play 된 뒤 다른 터미널 — run_proxy.sh 와 같은 도메인(42)으로, teleop 은 띄우지 않은 상태에서
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 sim/pursuit.py --dir cw --v 0.8 --lookahead 2.0     # 한 바퀴 돌면 LAP_DONE … max_cte=… 찍고 정지
```

- 실패하면 `FAIL <이유>`(OFFROAD 3.5 m · STALL 10 s · NO_ODOM · TIMEOUT)를 찍고 멈춥니다. 종료 코드 0 = 완주.
- 시작 자세(`looppath.py` 의 `X0 · Y0 · PSI0`)는 `params.yaml` 의 `robot.start` 와 같아야 합니다 — 시작 자세를 바꾸면 같이 고칩니다.

### 자동 교사 — `sim/teacher.py` · `sim/auto_teacher.sh` · `sim/auto_batch.sh` (10.12)

교사 데이터를 사람 조작 대신 **정답 경로를 따르는 규칙 노드**로 무인 수집합니다(10.12 검토 — 사람 조작은 비교용 소량 · 교사 20회 평가로, 10.13 멘토 회의 질문 4). `pursuit.py` 와 같은 조향식에 수집용 기능을 더했습니다. 계산부는 `sim/teacher_core.py`(ROS 없음)이고, 노드와 시험(`sim/test_teacher_core.py`)이 같은 코드를 씁니다.

- **라벨 `/teacher/cmd_vel`**(TwistStamped, 시뮬 시각) = 교사의 깨끗한 명령. **실행 `/cmd_vel`** = 라벨 + 조향 잡음 — 학습 라벨로 `/cmd_vel` 을 쓰지 않습니다([`topics.md`](../docs/interfaces/topics.md) · [`d2e_teacher.md`](../docs/interfaces/d2e_teacher.md), E 확인 전).
- **이탈 복구 장면** — 시간의 40 %(`--noise-duty`, 10.13 에 0.3 → 0.4)에 0.5~2 s 삼각 펄스(꼭대기 0.3~0.8 rad/s, `--noise-amp` — 실행은 상한 0.7 에서 잘림)를 실행 명령에만 더합니다. 기준선에서 0.3 m 넘게 벗어난 장면 약 39 %(계획서 이탈 복구 40 %), 최대 이탈 약 1.1 m(0.3 · 0.6 은 17 % · 0.44 m). `hold`(기본)는 직선 구간 펄스 동안 교정을 멈춰 더 크게 벗어나게 하고(회전 구간에 걸치는 펄스는 `add`), `add` 는 교사가 계속 고치는 위에 더합니다.
- **`/route_command`** 를 녹화 때 함께 냅니다 — 40° 넘게 꺾이는 곳의 꼭짓점 앞 5 m ~ 뒤 2 m 를 좌 · 우회전(Proxy 는 `eval/routes/loop_*.yaml` 초안과 같은 구간, 26° BEND 는 직진).
- **`/teacher/status`**(JSON 한 줄, 20 Hz) — 단계(ALIGN · FOLLOW · DWELL · DONE) · 바퀴 · 진행 `s` · 이탈 `cte`(왼쪽 +) · 방향 오차 · 지시 · 목표 속도 · 잡음. 학습 때 FOLLOW 만 쓰는 식으로 거릅니다.
- **주행 규칙** — 상한 v 0.8 · |ω| 0.7(사람 · Nav2 와 같음), 가감속 한도는 키보드 램프와 같음, 시뮬 시간 20 Hz. 속도 목표는 0.5~0.8 m/s 에서 약 30 m 마다 새로 뽑고(`--v-range` · `--v-change`), 회전 구간은 0.5(`--v-turn`), 곡률이 크면 더 줄입니다. 바퀴 끝마다 출발점에서 감속 정지 후 2 s 대기(목표 도달 장면).
- **경로** — `proxy`(Proxy 루프) 또는 `csv:<파일>[@회차]`. 실제 맵은 **기준선 `eval/routes/parking_r3_cw.csv`**(화단 윤곽을 3 m 부풀린 선, 10.13 멘토 회의에서 채택 — `eval/README.md`). 촬영 궤적 CSV(C 의 `route_*.csv`)는 외곽 촬영용으로 크게 · 작게 돈 경로라 기준선으로 쓰지 않고, 쓸 때는 `--smooth 1`(앞뒤 1 m 이동 평균). `ccw` 는 같은 출발점에서 거꾸로 돌고, 출발 방향이 20° 넘게 다르면 제자리 회전부터 합니다. 시작 자세는 `FD_SCENE` 루트에서 읽습니다.
- **`--label-only`** — `/cmd_vel` 을 내지 않고 라벨 · 지시만 냅니다. 학습 모델이 몰 때 그 자리 라벨을 남기는 DAgger(2차 수집)용.

```bash
# 씬이 떠 있을 때 한 묶음 — 녹화(record_teacher.sh) + 교사를 같이 돌리고, 끝나면 녹화를 닫고 점검
sim/auto_teacher.sh proxy cw 3 잡음30 -- --seed 1
FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/auto_teacher.sh csv:eval/routes/parking_r3_cw.csv ccw 2

# 밤새 무인 — 헤드리스 씬을 띄우고 작업 파일의 줄마다 auto_teacher.sh, 끝나면 씬을 끔(tmux 안에서, 기본 도메인 43)
FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/auto_batch.sh <작업 파일>
python3 sim/test_teacher_core.py        # 계산부 시험 (ROS 없이)
```

- **녹화 폴더**는 사람 조작과 같은 `data/teacher/raw_v1/<월일_시분초>_auto_<방향>/` — `teacher/`(교사 로그 · 요약 JSON · 경로 CSV)와 `meta.yaml` 의 `teacher` 칸이 더 붙습니다. 점검(`qc.txt`)에 `label` · `teacher` 줄이 더 나옵니다(라벨 Hz · 상한, 주행 중 잡음 · 이탈 비율, 지시별 비율).
- **실패**(이탈 3 m · 10 s 정체 · 시간 초과 · odom 끊김 · 시뮬 재시작)면 교사가 정지 명령을 내고 끝나며, 녹화는 닫아 남깁니다(`meta.yaml` 의 `teacher.rc` 로 거름). `auto_batch.sh` 는 실패한 다음 작업 전에 씬을 다시 띄웁니다.
- 장애물은 피하지 않습니다(카메라 · LiDAR 를 안 봄) — 장애물 시나리오의 교사는 우회 경로 생성 또는 Nav2 를 따로 둡니다.

## 실제 주차장 씬 — 맵 v0.4b (10.12)

> **r3 를 쓰세요(10.12 저녁).** C 가 아래 문제(겹친 바닥 · sRGB 색 · 경로 위 덩어리)를 원본에서 고친 `data/usd/parking_v0.4b_r3/`(레포 `7355d1f` 파이프라인)는 D 보정 레이어 없이 올라갑니다. 루트 `sim/scenes/parking/parking_v0.4b_r3.usda` — dfix 레이어 2개를 빼고, 바닥 높이 기준이 바뀌어 씬 전체가 6.5 cm 낮아졌습니다. 로봇 시작 자세는 10.13 에 기준선 출발점 (−1.756, −7.621), yaw −96.0°, z −0.035 로 옮겼습니다. **D 보정 1개(10.13)**: 서쪽 끝 (−36.8, −7.35)에 바닥 면과 따로 떨어진 납작한 조각(6삼각형)이 바닥과 겹쳐 있어, 잡음을 키운 자동 교사가 밟자 PhysX 가 멈췄습니다 → `data/usd/parking_v0.4b_r3_dfix/collision_fix_w.usdc`(`sim/fix_double_floor.py`, 그 조각만 뺀 면 목록)를 루트 subLayers 에 얹었습니다. 기준선 ±2.35 m 띠의 겹친 바닥은 이것으로 0 입니다. C 가 원본에서 고치면 뺍니다(지도 전체의 비슷한 조각 목록: `sim/fix_double_floor.py <mesh> --list --th 0.08`). PhysX 멈춤 · 걸림 없이 X-Mobility 제로샷 완주(랩 173.4 s, 평균 이탈 0.62 · 최대 2.08 m). **기준선은 `eval/routes/parking_r3_cw.csv`**(화단 윤곽을 3 m 부풀린 선, 10.13 임시 — `eval/README.md`). 촬영 궤적 `data/usd/parking_v0.4b_r3/route_bonnet_cw.csv` 는 참고용(외곽 촬영용으로 크게 · 작게 돈 경로). 아래 내용은 처음 판(v0.4b + dfix) 기록입니다.
>
> ```bash
> FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/run_proxy.sh stream
> ```

C 가 D 에게 처음 넘긴 실제 주차장 USD(`data/usd/parking_v0.4b/`, B 맵 v0.4b 본네트 카메라 기준)를 Proxy 와 같은 로봇 · 센서 · ROS · 물리 레이어로 묶은 씬입니다. 루트는 `sim/scenes/parking/parking_v0.4b.usda` — Proxy 의 `env_proxy` · `background` 자리에 C 의 `parking_scene.usda` 가 들어갑니다. 실행은 `run_proxy.sh` 에 `FD_SCENE` 만 붙입니다.

```bash
FD_SCENE=sim/scenes/parking/parking_v0.4b.usda sim/run_proxy.sh stream     # 스트리밍 (headless 도 같음)
sim/run_proxy.sh teleop                                                      # 조작 · 점검(check) · 녹화는 그대로
```

- **좌표는 B 맵 좌표입니다** — 원점 = 10.5 촬영 첫 키프레임, 방위 임의, 주 바닥 z 0. Proxy 의 ENU(루프 남서 꼭짓점 원점)와 다르고, 아직 맞추지 않았습니다(`c2d_usd.md` 원점 칸 미합의).
- **시작 자세**: 처음 판(v0.4b)은 C 제안 그대로 경로 회차 0 첫 점 (−1.60, −4.89), yaw −97.4° — 시계 방향. **r3 루트는 10.13 에 기준선 출발점 (−1.756, −7.621), yaw −96.0°, z −0.035 로 옮김**(`eval/routes/parking_r3_cw.yaml` 의 `start` — 화단 3 m 선 NE 모서리 원호 끝). 루트 레이어에서 Proxy `robot.usda` 의 값을 덮어씁니다.
- **C 원본은 고치지 않고 D 임시 보정 레이어 2개를 앞에 얹습니다**(`data/usd/parking_v0.4b_dfix/`). C 가 원본에서 고치면 루트의 `subLayers` 에서 뺍니다.
  - `color_linear.usdc` (`sim/make_linear_color.py`) — 정점 색이 사진값(sRGB)이라 RTX 가 2~3배 밝게 그려 E2E 카메라 화면이 하얗게 날아갔습니다. 선형으로 바꿔 덮어씁니다.
  - `collision_onefloor.usdc` (`sim/make_single_floor.py`) — C 의 충돌 메쉬는 지형 메쉬 + 지도 전체 바닥 면 + 부분 바닥 면을 합친 것이라 바닥이 여러 겹입니다(회차 0 경로 바퀴 자리의 85 % 이상). 이대로 몰면 **PhysX 가 경로 12.4 m (−2.94, −17.18) · 75 m (−14.15, 1.37) 지점에서 끝나지 않는 계산에 빠져 시뮬이 멈춥니다**(헤드리스 · 스트리밍 모두, 출발점을 바꿔도 같은 자리 — CCD 끄기 · 중복 삼각형 제거로는 안 풀림). 바닥 면 한 장 + 그보다 5 cm 넘게 솟은 지형만 남기면 두 곳 모두 지나갑니다.
- **조명**: 돔 600 · 해 0 (Proxy 는 1000 · 1500). 사진 색에 그림자가 이미 들어 있어 해를 끄고, 사진과 밝기가 비슷한 돔 세기를 비교 렌더로 골랐습니다.
- **알려진 걸림**: 회차 0 경로 52.5 m 지점 (−33.9 ~ −33.4, −6.75 ~ −6.55)에 바닥보다 최대 15 cm 솟은 덩어리가 있어 Nova Carter 가 기울며(roll 9°) 멈춥니다. 실제 물체인지 잡음인지는 아직 모릅니다(C 확인 필요).
- `sim/pursuit.py` 는 Proxy 루프 전용이라 이 씬에서는 쓰지 않습니다(경로 · 시작 자세가 다름).
- 점검(10.12): `run_proxy.sh check` NG 0 — 규격 토픽 6/6, 카메라 20.1 Hz, RTF 0.98(스트리밍). 로드 5.5 s, `[Error]` 0.

## Nav2 — `sim/nav2/` (10.13)

비교군(수행계획서) 겸 교사(10.13 멘토: Nav2 주행도 학습 데이터로 써도 됨 — 장애물 시나리오부터). 서버 apt `ros-jazzy-navigation2` · `nav2-bringup` · `pointcloud-to-laserscan` · `robot-localization`(10.13 설치, 1.3.13).

```bash
envs/isaacsim_env/bin/python sim/nav2/make_map.py data/usd/parking_v0.4b_r3/usd/parking_mesh.usdc data/nav2/parking_v0.4b_r3 --keepout eval/routes/parking_r3_island.yaml   # 지도(한 번)
FD_LIDAR=1 FD_DOMAIN=43 FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/run_proxy.sh headless          # tmux 안 — LiDAR 를 켜야 함
ros2 launch sim/nav2/nav2.launch.py map:=$PWD/data/nav2/parking_v0.4b_r3/map.yaml [localization:=amcl|gt]    # 시스템 Jazzy · 도메인 43
python3 sim/nav2/nav2_route.py --route eval/routes/parking_r3_cw.csv --out logs/nav2/<이름>.json              # 기준선 한 바퀴(경유점 4 m), 이탈은 /ground_truth/tf 로
```

- **지도**(`make_map.py`) — 충돌 메쉬로 만든 PGM · YAML(0.05 m). 장애물 = 바닥 위 0.12 ~ 1.0 m 의 면 + 경계벽. 좌표는 B 맵 좌표(= Nav2 `map`). **두 장**(10.14):
  - `map.yaml` — 형상 그대로 → `/map`, AMCL 위치 추정용.
  - `map_keepout.yaml`(`--keepout`) — 가운데 화단을 채움 → `/map_nav`, 경로 계획 · 비용 지도용(메쉬로는 화단 안이 거의 비어 경로가 화단을 가로지름).
  - 화단 채운 지도를 AMCL 에 주면 LiDAR 가 못 보는 큰 장애물 덩어리에 엉뚱한 자세의 스캔도 걸려 점수가 높아지고, 먼 곳의 가짜 무리가 살아남아 위치가 10 m 씩 튐 — 지도를 나누자 해결.
- **설정**(`nav2_params.yaml`) — NVIDIA carter_navigation 설정(DWB, v 0.8 · ω 0.7 — 우리 상한과 같음)이 바탕, 고친 곳은 `# FD:` 주석.
  - 3D LiDAR → `/scan`(launch 의 pointcloud_to_laserscan) — 앞 ±90°, 높이 띠 바닥 위 약 0.23 ~ 1.0 m(지도 장애물 띠 안). NVIDIA 의 -0.4 ~ 1.5 m 띠는 1 m 위 나뭇가지 · 경사 바닥이 스캔에만 잡혀 지도와 안 맞음. 우리 씬의 2D LiDAR 두 대는 데이터가 안 나옴.
  - 가감속 한도는 키보드 램프 · 자동 교사와 같게 — DWB 와 velocity_smoother 둘 다(한쪽만 바꾸면 DWB 가 못 따라갈 회전을 짜 모서리에서 맴돎).
  - 경유점 판정 반경 0.7 → 1.5 m(`navigate_through_poses.xml`) — 모서리에서 경유점을 0.7 m 밖으로 지나면 그 점으로 돌아가려 제자리를 돎.
  - AMCL(10.14 조정) — alpha 0.2 → 0.02(시뮬 odom 은 정답과 0.35° 안), z_hit 0.9 · z_rand 0.1 · sigma_hit 0.1, 빔 120, 0.1 m · 0.1 rad 마다 갱신, 매번 재표집. 녹화를 13번 재생해 지도 · 설정을 비교해 고름(`sim/nav2/tune/` — `amcl_record.sh` 녹화 → `amcl_replay.sh` 여러 도메인에서 동시 재생 → 정답과 오차).
- **위치 추정** — `amcl`(기본, 실제 로봇과 같은 방식 — 10.13 멘토 의견) / `gt`(정답 map → odom 고정 변환, Nav2 자체 시험용).
- **결과**(r3, 기준선 경유점 4 m, 기본 DWB)

  | 위치 추정 | 방향 | 결과 | 랩(시뮬) | 기준선 평균 · 최대 이탈 | 복구 |
  | --- | --- | --- | --- | --- | --- |
  | gt(10.13) | 시계 | **완주** | 129 s | 0.22 · 1.42 m | 0 |
  | amcl 조정 전(10.13) | 시계 | 실패 — 25 m 뒤 AMCL 이 8 m 튐 | 39 s | 0.30 · 1.10 m | 3 |
  | **amcl 조정 뒤(10.14, 2회)** | 시계 | **완주 2/2** | 128 · 129 s | 0.29 · 1.41 / 0.30 · 1.43 m | 0 |
  | **amcl 조정 뒤(10.14, 2회)** | 반시계 | **완주 2/2** | 131 · 131 s | 0.24 · 1.54 / 0.25 · 1.89 m | 0 |

  조정 뒤 AMCL 위치 오차(정답 대비, 양방향 2바퀴) — 중앙 0.11 m · 최대 0.22 m · 방향 최대 3.2°, 0.5 m 넘음 0회(재생 시험과 같은 값).
- **주의**
  - ROS 노드를 `kill -9` 나 `tmux kill-session` 으로 끄면 `/dev/shm` 에 Fast DDS 조각이 남아, 새로 띄운 노드가 다른 노드의 서비스는 보이는데 통신을 못 함(Nav2 활성 대기에서 멈춤). Ctrl+C 로 끄고, 이상하면 `fastdds shm clean`.
  - tmux 세션 이름 앞부분이 겹치면 다른 세션을 끔(`-t nav2` 가 `nav2sim` 도 잡음) — 겹치지 않는 이름이나 `-t "=이름"`.
  - 설정 파일에 map_server 의 `yaml_filename` 을 적으면 launch 의 `map` 인자를 덮어써 지도가 빔.

### 시나리오 — `eval/scenarios/` · `sim/make_obstacles.py` · `sim/nav2/scenario_batch.sh` (10.14)

장애물 시나리오 8종(한 바퀴, 시계 · 반시계)과 구간 시나리오 5종(직진 · 곡선, 기준선의 한 구간) — 목록은 `eval/README.md` 시나리오 절.

```bash
sim/nav2/scenario_batch.sh eval/scenarios/parking_r3_obs_cones.yaml:ccw eval/scenarios/parking_r3_straight_south.yaml   # 일괄(tmux 안), FD_OUT=결과 폴더
# 하나씩 할 때
envs/isaacsim_env/bin/python sim/make_obstacles.py eval/scenarios/parking_r3_obs_south_box.yaml sim/scenes/parking/obstacles/parking_r3_obs_south_box.usda
FD_LAYERS=sim/scenes/parking/obstacles/parking_r3_obs_south_box.usda FD_LIDAR=1 FD_DOMAIN=43 FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/run_proxy.sh headless
ros2 launch sim/nav2/nav2.launch.py map:=$PWD/data/nav2/parking_v0.4b_r3/map.yaml [x0:=… y0:=… yaw0_deg:=…]   # 구간이면 make_obstacles 의 START 자세
python3 sim/nav2/nav2_route.py --route eval/routes/parking_r3_cw.csv --scenario eval/scenarios/parking_r3_obs_south_box.yaml --out logs/nav2/<이름>.json
python3 eval/scenarios/plot_scenarios.py   # 시나리오 그림 → logs/nav2/scenarios_r3.png
```

- **레이어**(`make_obstacles.py`) — 시나리오의 장애물(기준선 길이 s · 옆 오프셋)을 월드 좌표로 바꾸고 바닥 높이를 충돌 메쉬에서 재서, 정적 충돌 물체(상자 · 원기둥 · 원뿔, UsdPreviewSurface 재질 — 카메라 · 3D LiDAR 에 보임) USD 레이어로 씀. 구간 시나리오(`start.s`)면 로봇 출발 자세도 덮어씀(기준선 위, 진행 방향을 봄 — 로그 `START`).
- **`FD_LAYERS`**(`sim/proxy_play.py`) — 콜론으로 이은 레이어들을 세션 레이어 밑에 얹고 Play. 씬 파일 · 씬 루트를 늘리지 않고 시나리오마다 다른 것을 넣음. 로그 `PROXY_LAYERS`.
- **`nav2_route.py --scenario`**
  - 장애물 가장자리 1.5 m 안의 기준선 경유점은 뺌(교사는 장애물 자리를 알아도 됨). 로봇 발자국과 장애물 사이 최소 거리 · 충돌 · 가장 긴 정지를 잼.
  - 판정 `pass`(수행계획서 2.4) — 완주 · 이탈 2.0 m 안 · 충돌 없음 · 10 s 넘는 정지 없음 · 마지막 경유점 1.0 m 안 · 달린 거리가 경로 길이의 80 % 이상.
  - 바퀴의 마지막 경유점은 결승 1 m 앞 — 출발점 바로 뒤(0.24 m)에 두면 안 움직이고 '목표 도착'으로 끝날 때가 있었음(10.14 반시계 3회, 6 s).
  - 구간이면 `start.s` → `goal.s` 만 돎. 출발 전에 AMCL 위치가 정답 위치 0.5 m 안인지 확인하고, 아니면 `/initialpose` 를 다시 보냄(그래도 안 맞으면 코드 3).
  - 출발 전 센서 예열 — `/scan` 20개 · 전역 비용 지도 갱신 3번을 받은 뒤 목표를 보냄(최대 20 s). Nav2 를 켜자마자 출발하면 전역 비용 지도(1 Hz)에 LiDAR 장애물이 아직 없어 첫 경로가 장애물을 가로지르고, 지역 비용 지도(6 × 6 m)는 3 m 밖을 못 봐 출발 직후 장애물에 부딪힘(10.14 시연 녹화).
- **AMCL 초기 자세**는 `nav2.launch.py` 의 `x0 · y0 · yaw0_deg`(설정 파일 initial_pose 를 덮어씀) — 스크립트가 보내는 `/initialpose` 만으로는 새로 뜬 AMCL 에 안 닿아, AMCL 이 r3 출발점에 머문 채 달린 일이 있었음(10.14 구간 시나리오, 이탈 5 ~ 13 m).
- **`scenario_batch.sh`** — 항목마다 레이어를 만들고 씬 + Nav2(AMCL)를 새로 띄워 한 번. 준비 실패 — Nav2 lifecycle 3개(위치 추정 · 지도 · 주행) 중 덜 뜸(상태 전환 응답이 통신 지연으로 빠지면 Nav2 활성 대기에서 멈춤, 10.14) · nav2_route 코드 3(지도 · 정답 위치 · AMCL 확인, Nav2 상태 · 액션 서버를 90 · 30 s 안에 못 받음 — BasicNavigator 대기는 끝이 없어 15분 멈춘 일이 있음) — 나 벽시계 15분 초과면 다시 띄움(최대 3번). 끝나면 Ctrl+C → TERM → KILL(INT · TERM 을 무시하고 남은 lifecycle_manager 가 다음 Nav2 와 이름이 겹친 일이 있음) · `fastdds shm clean`.
- **10.14 결과**(Nav2 + AMCL, 경유점 4 m) — 장애물 8종 × 양방향 16가지 중 15가지 통과, 구간 5종 모두 통과(최대 이탈 직진 0.23 · 0.30 m, 곡선 0.64 ~ 1.73 m).
  - 실패 1종 — `obs_corner_exit` 시계 2회(최대 이탈 2.07 · 2.02 m). 남동 모서리를 도는 중(s 14)에 바로 뒤 상자(s 17.5)를 피하려 바깥으로 벌어짐. 장애물이 없어도 Nav2 는 모서리에서 1.4 ~ 1.9 m 벗어나므로(10.14 AMCL 확인), 모서리 근처 장애물이면 2.0 m 한도에 걸림.
  - `obs_south_box` 시계는 첫 회 남서 모서리(s 50)에서 2.04 m(복구 6회)로 실패, 다시 돌려 통과.
  - 장애물과 최소 거리 0.69 ~ 0.93 m, 충돌 0, 움직이기 시작한 뒤 가장 긴 정지 2.8 s 이하. 16가지 중 9가지는 최대 이탈이 장애물이 아니라 모서리에서 남. 결과 서버 `logs/nav2/obstacle/v1` · `scenario_v1b` · `scenario_v2`.
  - 첫 결과(남쪽 변 상자 v0) — 카메라에는 8 m 앞부터 상자가 보이고, `/scan` 은 8 m 앞에서 스캔마다 9 ~ 21점이 상자에 맞음.
- **녹화 주의** — 스크립트에서 `&` 로 띄운 `ros2 bag record` 는 SIGINT 를 무시함(비대화형 bash) → SIGTERM 으로 끔. `--compression-mode file` 은 멈출 때 파일 전체를 다시 압축해 오래 걸리니 `--storage-preset-profile zstd_fast`(청크 압축).


### Nav2 교사 녹화 — `sim/nav2/nav2_teacher.py` (10.14)

Nav2 가 모는 주행(장애물 회피 포함)을 자동 교사와 같은 형식으로 녹화 — 10.13 멘토: Nav2 주행도 교사 데이터로 씀.

```bash
FD_RECORD=1 sim/nav2/scenario_batch.sh eval/scenarios/parking_r3_obs_south_box.yaml:cw eval/scenarios/parking_r3_curve_se_ccw.yaml   # tmux 안
# FD_TEACHER_DIR=저장 위치(기본 data/teacher/nav2_v1) · FD_NOISE=조향 잡음 비율(기본 0) · FD_OUT=판정 · 로그 폴더
```

- **연결** — Nav2 를 `cmd_vel_out:=nav2/cmd_vel` 로 띄우면 최종 명령(collision_monitor)이 `/nav2/cmd_vel` 로 나옴. 다리가 받아 즉시 `/cmd_vel`(실행) 로 내고, 시뮬 시간 20 Hz 로 `/teacher/cmd_vel`(라벨 = Nav2 명령) · `/route_command`(자동 교사와 같은 규칙) · `/teacher/status`(자동 교사 칸 + `src` · `prog` · `od` · `age`)를 냄. `/cmd_vel` 발행자는 다리 하나 — Nav2 의 docking_server(안 씀)도 `/cmd_vel` 발행자를 만들어 `nav2.launch.py` 가 옆으로 돌림.
- **위치** — 지시 · 상태는 정답 위치 `/ground_truth/tf` 로 계산(라벨 쪽 정보, 모델 입력 아님). Nav2 자체는 AMCL 로 몲.
- **녹화** — 항목마다 다리 → `sim/record_teacher.sh nav2 <방향> <이름>` → `nav2_route.py` → 녹화 닫기(QC) 순. 녹화 폴더 `teacher/` 에 다리 · 판정 로그와 JSON, 시나리오 · 기준선 파일, `meta.yaml` 에 `teacher:` 칸(시나리오 · 판정 `pass` · 잡음). 준비 실패로 다시 띄운 회차의 녹화는 지움.
- **잡음**(`FD_NOISE`) — 자동 교사의 조향 잡음을 add 방식으로(Nav2 가 계속 고침). 출발 뒤 2 m · 장애물 가장자리 3 m 안에서는 새 펄스를 안 냄. 라벨은 늘 Nav2 의 깨끗한 명령.
- **10.14 시험** — 녹화 4개(장애물 없는 바퀴 시계 · 남쪽 변 상자 시계 · 남동 모서리 곡선 구간 반시계 · 남쪽 변 상자 반시계 + 잡음 0.3) 모두 QC OK · Nav2 판정 통과(충돌 0, 최대 이탈 0.68 ~ 1.51 m). 잡음 없을 때 라벨과 실행 명령 차이는 중앙 0 · 95 % 0.03 안(20 Hz 틱 사이 시간차). 잡음 0.3 이면 주행 중 잡음 25 %, 기준선에서 0.3 m 넘게 벗어난 시간 39 %(잡음 없는 바퀴 23 · 31 %). 시뮬 1분에 0.35 GB(자동 교사와 같음). 시험 녹화는 서버 `data/teacher/nav2_dev/`(정식 수집 아님).
- **10.14 본 수집** — `data/teacher/nav2_v1/`(MANIFEST `parking_r3_nav2` v1.0): 장애물 8종 × 양방향 + 구간 5종 = 21개, 잡음 0.3, Nav2 판정 21/21 통과(corner_exit 시계도 이번엔 1.55 m) · QC 21/21 OK, 시뮬 40.7분 · 14 GB, 약 80분 걸림(준비 실패 3회 재시도 포함).

### 시연 영상 — `sim/nav2/record_demo.sh` (10.14)

```bash
sim/nav2/record_demo.sh eval/scenarios/parking_r3_obs_demo.yaml cw    # tmux 안 — logs/nav2/demo/<이름>_<방향>{,_1080p,_720p}.mp4
```

- X-Mobility 시연 영상과 같은 화면(1920×1080) — 3인칭 카메라 · 로봇 앞 카메라 · 미니맵(기준선 · 장애물 · Nav2 계획 경로 · 지나온 길) · Nav2 명령 · 이탈 · 장애물 거리, 3배속.
- **두 단계** — 주행하면서 바로 그리면 렌더 · 합성 때문에 시뮬이 실시간의 0.35 ~ 0.6배로 느려지는데, Nav2 의 일부(행동 트리 재계획 · 제어 루프 · 속도 다듬기)는 실제 시간으로 돌아 동작이 달라짐(10.14: 수집 때 넉넉히 피한 상자에 부딪히거나 스침, 장애물 양옆을 오가며 망설임).
  - `drive` — 영상 렌더 없이 수집 때와 같은 조건(실시간 1.06배)으로 주행, 0.1 s(시뮬)마다 로봇 자세와 그때의 다리 상태 · Nav2 계획 번호를 기록. 판정 실패면 다시(최대 3번, 시도마다 `route_try<n>.log`).
  - `replay` — 물리 없이 기록한 자세로 로봇을 다시 놓으며 렌더(`rep.orchestrator.step` — 타임라인이 멈춰 있으면 `app.update` 로는 화면이 안 채워짐) · 합성. `FD_DEMO_SKIP_DRIVE=1` 이면 앞 기록으로 다시 렌더만.
- H.264 변환은 imageio-ffmpeg 정적 바이너리(서버에 ffmpeg 명령이 없음). 시연용 시나리오 `eval/scenarios/parking_r3_obs_demo.yaml`(장애물 6개 한 바퀴).
- **10.14 결과** — 시계 한 바퀴 랩 128.8 s · 기준선 최대 이탈 1.84 m · 장애물과 최소 0.65 m · 충돌 · 복구 0, 영상 46.8 s(볼트 05 `Video_2026-10-14_Nav2_장애물회피_실제맵_r3.mp4`). 실시간 조건 주행은 2/2 통과(첫 회는 자세 기록 버그로 다시 돎).

