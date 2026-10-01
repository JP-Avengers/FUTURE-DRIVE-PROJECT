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
| 키보드 조작 | 다른 터미널 | `sim/run_proxy.sh teleop` — `/cmd_vel` 발행자가 0 일 때만 뜸 |

- **끄기**: 띄운 터미널에서 `Ctrl+C`(tmux 밖에서는 `tmux send-keys -t fd_proxy C-c`). 끝에 `[Error]` 줄 수와 GPU 상태를 찍습니다. 로그는 `logs/run_proxy/<모드>_<시각>.log`.
- **스트리밍 화면**: 뷰포트가 로봇 뒤 3인칭 카메라(`ChaseCam`)로 바뀝니다 — 운전자용이고 기록에는 안 씁니다. 화면의 Stop → Play 로 출발점에 돌아갑니다. 조작 키는 `teleop` 을 띄운 ssh 터미널에 눌러야 합니다 — 스트리밍 화면을 클릭하면 키가 Isaac Sim 으로 갑니다.
- **서버 앞 모니터로는 띄우지 않습니다**(10.1 확인). 데스크톱 화면이 GPU 1 에만 나오는데 렌더를 GPU 0 에 고정하면 창을 못 그립니다(`createSwapchain failed`). `sim/run_proxy.sh monitor` 는 안내만 찍고 끝납니다.
- **환경변수**: `FD_LIDAR=1` LiDAR 켬(점검도 같은 값으로) · `FD_DOMAIN=43` 시험용 도메인 · `FD_VIEW_RES=1280x720` 뷰포트 해상도(스트리밍 — RTF 가 낮을 때 줄임).
- `run_streaming.sh` 는 씬 없이 빈 앱을 스트리밍할 때 씁니다.
