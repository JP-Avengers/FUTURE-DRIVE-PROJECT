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

**ROS 2 Bridge 검증** (헤드리스로 `/clock` 발행 → 호스트에서 수신):

```bash
source /opt/ros/jazzy/setup.bash        # 반드시 Isaac Sim 기동 **전에** — 안 하면 내장 라이브러리로 넘어가 Bridge 실패
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
OMNI_KIT_ACCEPT_EULA=YES $CD1/envs/isaacsim_env/bin/python sim/check_ros2_bridge.py 60
# 다른 터미널: ros2 topic echo /clock rosgraph_msgs/msg/Clock --once
```

GPU 0 고정은 스크립트 안의 `active_gpu=0`·`multi_gpu=False`로 합니다. **`CUDA_VISIBLE_DEVICES`는 걸지 마세요** — 종료 시 abort 가 납니다(9.27 실측).
