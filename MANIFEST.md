# MANIFEST — 대용량 산출물 버전 목록

메쉬·USD 바이너리·rosbag·데이터셋·모델 가중치는 **Git에 올리지 않습니다** (`.gitignore`, `pre-commit` 5MB 상한).
실물은 server207 의 `$CD1/data/` 에 두고, **이 파일에 한 줄씩 기록한 뒤 커밋**합니다. 이 목록이 곧 "어느 버전으로 학습·평가했는가"의 근거입니다.

## 기록 규칙

- **새 버전을 `data/` 에 넣을 때마다 한 줄 추가** — 기존 줄은 고치지 않습니다(덮어쓰기 금지, 버전을 올리세요).
- **버전**: `v<주>.<부>` — 형식이 바뀌면 주 번호, 내용만 보완하면 부 번호 (예: `v0.1` → `v0.2` → `v1.0`)
- **경로**: `$CD1` 기준 상대경로 (예: `data/usd/proxy_v0.1/`)
- **sha256**: 앞 12자리만 적어도 됩니다. 아래 명령으로 계산합니다.
- **출처**: 어느 버전에서 만들어졌는지 (예: `teacher v0.2 → dataset v0.1`) — 학습·평가 결과를 거슬러 올라갈 수 있어야 합니다.

```bash
# 파일 하나
sha256sum data/checkpoints/e2e_v0.1.pt | cut -c1-12
# 디렉터리 (파일 목록과 내용 전체)
(cd data/usd/proxy_v0.1 && find . -type f -print0 | sort -z | xargs -0 sha256sum) | sha256sum | cut -c1-12
# 크기
du -sh data/usd/proxy_v0.1
```

## 목록

| 날짜 | 구분 | 이름 | 버전 | 경로 (`$CD1` 기준) | 크기 | sha256 (12) | 출처 | 담당 | 비고 |
|---|---|---|---|---|---|---|---|---|---|
| 9.29 | `teacher` | `proxy_sample` | v0.1 | `data/teacher/proxy_sample_v0.1/` | 366M | 0ea2043fa2ea | `Proxy 씬 params v0 · 레포 9d4939f` | D | `E 기록기 개발용 형식 샘플 — 스크립트 주행(사람 조작 아님), /route_command 는 0 고정 더미, --use-sim-time` |
| 9.29 | `teacher` | `warehouse_sample` | v0.1 | `data/teacher/warehouse_sample_v0.1/` | 1.3G | 87710e6a407f | `Isaac 6.0 carter_warehouse_navigation.usd(NVIDIA 샘플) · lidar_wander` | D | `X-Mobility 오프라인 추론 시험용 — 교사 데이터 아님, Hawk 왼쪽 1920×1200 원본` |
| 9.30 | `teacher` | `proxy_sample` | v0.2 | `data/teacher/proxy_sample_v0.2/` | 525M | 3c283ee59480 | `Proxy 씬 params v0 · 레포 af4efa1 + 생성 스크립트 수정(PhysX 설정 · 평면 바닥, 9.30 커밋 전, build_proxy_scene.py sha cd0b9b39a48d)` | D | `v0.1 대체 — 직진 출발 흔들림 제거(9.30), 스크립트 주행, /route_command 는 0 고정 더미, --use-sim-time, 카메라 3.3 s 지점에서 0.48 s 빔` |
| 10.12 | `usd` | `parking` | v0.4b | `data/usd/parking_v0.4b/` | 92M | b07a82333fe8 | `B 맵 v0.4b(본네트 카메라) → C 메쉬 파이프라인(레포 2d6947e)` | C | `C 가 zip 으로 D 에게 전달, D 가 서버에 올림. 씬 루트 sim/scenes/parking/parking_v0.4b.usda — 아래 dfix 와 함께 씀` |
| 10.12 | `usd` | `parking_dfix` | v0.4b | `data/usd/parking_v0.4b_dfix/` | 25M | 380ce4b5c516 | `parking v0.4b(b07a82333fe8) → sim/make_linear_color.py · sim/make_single_floor.py(5 cm)` | D | `D 임시 보정 — 정점 색 sRGB→선형, 충돌 겹친 바닥을 한 겹으로(원본 충돌로는 PhysX 가 2곳에서 멈춤). C 가 원본에서 고치면 폐기` |
| 10.12 | `usd` | `parking` | v0.4b r3 | `data/usd/parking_v0.4b_r3/` | 66M | bc5436d55df5 | `B 맵 v0.4b(본네트 카메라) → C 메쉬 파이프라인(레포 7355d1f), sha 는 usd/parking_mesh.usdc` | C | `D 10.12 피드백 반영(바닥 한 장 · 색 선형 · 잔상 지움 · 양면 표시 · 바닥 높이 기준 고정, 씬 6.5 cm 낮아짐) — dfix 불필요. 씬 루트 sim/scenes/parking/parking_v0.4b_r3.usda` |
| 10.13 | `usd` | `parking_dfix` | v0.4b r3 | `data/usd/parking_v0.4b_r3_dfix/` | 476K | 4a4006a9d448 | `parking v0.4b r3(bc5436d55df5) → sim/fix_double_floor.py --th 0.08 -- -36.8,-7.35,0.5` | D | `D 임시 보정 — 충돌 바닥 위 납작한 조각 한 곳(6삼각형, 바닥 위 최대 7.4 cm, 서쪽 끝) 지움: 자동 교사 잡음 0.4 · 펄스 0.8 시험 중 그 자리에서 PhysX 멈춤(10.13). r3 루트가 씀. C 가 원본에서 고치면 폐기` |

**구분** — `usd` 씬·메쉬 · `teacher` 교사 주행 원본 로그 · `dataset` 학습용 HDF5 · `checkpoint` 모델 가중치 · `eval` 평가 결과 · `raw` 센서 원본(rosbag)
