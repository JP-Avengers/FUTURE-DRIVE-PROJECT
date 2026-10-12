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

## 실제 주차장 씬 — 맵 v0.4b (10.12)

C 가 D 에게 처음 넘긴 실제 주차장 USD(`data/usd/parking_v0.4b/`, B 맵 v0.4b 본네트 카메라 기준)를 Proxy 와 같은 로봇 · 센서 · ROS · 물리 레이어로 묶은 씬입니다. 루트는 `sim/scenes/parking/parking_v0.4b.usda` — Proxy 의 `env_proxy` · `background` 자리에 C 의 `parking_scene.usda` 가 들어갑니다. 실행은 `run_proxy.sh` 에 `FD_SCENE` 만 붙입니다.

```bash
FD_SCENE=sim/scenes/parking/parking_v0.4b.usda sim/run_proxy.sh stream     # 스트리밍 (headless 도 같음)
sim/run_proxy.sh teleop                                                      # 조작 · 점검(check) · 녹화는 그대로
```

- **좌표는 B 맵 좌표입니다** — 원점 = 10.5 촬영 첫 키프레임, 방위 임의, 주 바닥 z 0. Proxy 의 ENU(루프 남서 꼭짓점 원점)와 다르고, 아직 맞추지 않았습니다(`c2d_usd.md` 원점 칸 미합의).
- **시작 자세**: C 제안 그대로 경로 회차 0 첫 점 (−1.60, −4.89), yaw −97.4° — 시계 방향. 루트 레이어에서 Proxy `robot.usda` 의 값을 덮어씁니다.
- **C 원본은 고치지 않고 D 임시 보정 레이어 2개를 앞에 얹습니다**(`data/usd/parking_v0.4b_dfix/`). C 가 원본에서 고치면 루트의 `subLayers` 에서 뺍니다.
  - `color_linear.usdc` (`sim/make_linear_color.py`) — 정점 색이 사진값(sRGB)이라 RTX 가 2~3배 밝게 그려 E2E 카메라 화면이 하얗게 날아갔습니다. 선형으로 바꿔 덮어씁니다.
  - `collision_onefloor.usdc` (`sim/make_single_floor.py`) — C 의 충돌 메쉬는 지형 메쉬 + 지도 전체 바닥 면 + 부분 바닥 면을 합친 것이라 바닥이 여러 겹입니다(회차 0 경로 바퀴 자리의 85 % 이상). 이대로 몰면 **PhysX 가 경로 12.4 m (−2.94, −17.18) · 75 m (−14.15, 1.37) 지점에서 끝나지 않는 계산에 빠져 시뮬이 멈춥니다**(헤드리스 · 스트리밍 모두, 출발점을 바꿔도 같은 자리 — CCD 끄기 · 중복 삼각형 제거로는 안 풀림). 바닥 면 한 장 + 그보다 5 cm 넘게 솟은 지형만 남기면 두 곳 모두 지나갑니다.
- **조명**: 돔 600 · 해 0 (Proxy 는 1000 · 1500). 사진 색에 그림자가 이미 들어 있어 해를 끄고, 사진과 밝기가 비슷한 돔 세기를 비교 렌더로 골랐습니다.
- **알려진 걸림**: 회차 0 경로 52.5 m 지점 (−33.9 ~ −33.4, −6.75 ~ −6.55)에 바닥보다 최대 15 cm 솟은 덩어리가 있어 Nova Carter 가 기울며(roll 9°) 멈춥니다. 실제 물체인지 잡음인지는 아직 모릅니다(C 확인 필요).
- `sim/pursuit.py` 는 Proxy 루프 전용이라 이 씬에서는 쓰지 않습니다(경로 · 시작 자세가 다름).
- 점검(10.12): `run_proxy.sh check` NG 0 — 규격 토픽 6/6, 카메라 20.1 Hz, RTF 0.98(스트리밍). 로드 5.5 s, `[Error]` 0.
