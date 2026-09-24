# server207 경로 규약

server207(RTX A5000 × 2, 공용 서버)에서는 **이 레포의 작업 폴더가 곧 프로젝트 루트 `$CD1`** 입니다.

```bash
export CD1=$HOME/Capstone_Design_1     # ~/.bashrc 에 등록됨 (비대화형 셸에서는 직접 선언)
```

## 구조

```
$CD1/                     = 이 레포 (git 추적)
├── ros2_ws/src/          ROS 2 패키지            ← 추적
│   (build/ install/ log/ 는 무시)
├── scripts/              평가 하네스 · 실행 스크립트 ← 추적
├── docs/                 문서                      ← 추적
├── MANIFEST.md           대용량 산출물 버전 목록     ← 추적
├── envs/                 venv (isaacsim_env · ml_env · eval_env · tools_env)  ← 무시
├── data/                 usd · teacher · dataset · checkpoints · eval          ← 무시, MANIFEST.md 로 기록
├── logs/                 설치 · 실행 로그                                       ← 무시
└── .cache/               uv · pip · pre-commit 캐시                             ← 무시
```

## 규칙

- **GPU**: Isaac Sim 은 GPU 0 고정. `CUDA_VISIBLE_DEVICES` 가 아니라 `SimulationApp({'active_gpu': 0, 'physics_gpu': 0, 'multi_gpu': False})` 로 지정합니다 (기본값이 멀티 GPU). torch 등 일반 CUDA 프로세스만 `CUDA_VISIBLE_DEVICES` 를 씁니다.
- **캐시**: `UV_CACHE_DIR` · `PIP_CACHE_DIR` · `PRE_COMMIT_HOME` 은 `$CD1/.cache/` 아래로 등록돼 있습니다.
- **루트 밖에 남는 것**: `~/.cache/ov` · `~/.local/share/ov`(Isaac Sim 셰이더 캐시), `~/.nvidia-omniverse/logs`, `~/.ros/`, `~/.local/bin/uv`, `/opt/ros/jazzy`
- **커밋 전 훅**: `$CD1/envs/tools_env/bin/pre-commit` 이 설치돼 있어 5MB 초과 파일·개인키가 막힙니다.
- **접속 정보(IP·포트·계정)는 이 레포에 적지 않습니다** — public 레포입니다.
