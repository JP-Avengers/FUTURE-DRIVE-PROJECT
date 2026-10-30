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
| 10.13 | `teacher` | `parking_r3_auto` | v1.0 | `data/teacher/raw_v1/` | 42G | dba3db1dbf40 | `parking v0.4b r3(bc5436d55df5) + r3_dfix(4a4006a9d448) · 기준선 eval/routes/parking_r3_cw.csv(화단 3 m 선, 97932cb) · 자동 교사 레포 76b9b77 기본값(잡음 0.4 · 펄스 0.3~0.8) · sim/auto_batch.sh, 시드 101~109` | D | `1차 교사 데이터 — 묶음 9개(1013_110557 ~ 1013_122008, 시계 5 · 반시계 4) × 5바퀴 = 45바퀴, 실패 0 · QC 9/9 OK. 녹화 시뮬 125.6분(주행 121.2분), 카메라 150,656장(20 Hz), 라벨 /teacher/cmd_vel, 0.3 m 넘는 이탈 장면 39.1 % · 최대 1.56 m` |
| 10.13 | `usd` | `parking` | v0.4b r4 | `data/usd/parking_v0.4b_r4/` | 94M | e34c04fb73d5 | `B 맵 v0.4b(본네트 카메라) → C 메쉬 파이프라인(레포 232f245), sha 는 usd/parking_mesh.usdc` | C | `r3 + 납작한 조각 원본에서 지움(47 → 0, r3_dfix 불필요) · 북쪽 차로 바닥 · 먼 구역 3D(VisualFar, 충돌 없음) · 하늘 돔 텍스처(usd/sky.usda). 좌표 · 바닥 높이 r3 와 같음. 씬 루트 sim/scenes/parking/parking_v0.4b_r4.usda — D 10.13 자동 교사 4바퀴 확인(멈춤 0)` |
| 10.13 | `nav2` | `parking_map` | v0.4b r3 | `data/nav2/parking_v0.4b_r3/` | 1.7M | 7b72c5f8aecd | `parking v0.4b r3(bc5436d55df5) → sim/nav2/make_map.py --keepout eval/routes/parking_r3_island.yaml` | D | `Nav2 지도(0.05 m) — 장애물 바닥 위 0.12~1.0 m + 경계벽 + 가운데 화단 채움. map.pgm · map.yaml · preview.png` |
| 10.13 | `nav2` | `parking_map` | v0.4b r4 | `data/nav2/parking_v0.4b_r4/` | 1.7M | 02e075021b0e | `parking v0.4b r4(e34c04fb73d5) → sim/nav2/make_map.py --keepout eval/routes/parking_r3_island.yaml` | D | `r3 와 같은 방식` |
| 10.14 | `nav2` | `parking_map` | v0.4b r3 · 2판 | `data/nav2/parking_v0.4b_r3/` | 3.2M | 23289a2dd861 | `parking v0.4b r3(bc5436d55df5) → sim/nav2/make_map.py --keepout eval/routes/parking_r3_island.yaml` | D | `10.13 행 대체(같은 폴더를 다시 만듦) — map.yaml(형상 그대로, AMCL 용 /map) + map_keepout.yaml(화단 채움, 경로 계획용 /map_nav)` |
| 10.14 | `nav2` | `parking_map` | v0.4b r4 · 2판 | `data/nav2/parking_v0.4b_r4/` | 3.2M | 458974308cfe | `parking v0.4b r4(e34c04fb73d5) → sim/nav2/make_map.py --keepout eval/routes/parking_r3_island.yaml` | D | `r3 2판과 같은 방식` |
| 10.14 | `teacher` | `parking_r3_nav2` | v1.0 | `data/teacher/nav2_v1/` | 14G | 80ecbeb54757 | `parking v0.4b r3(씬 루트 50d487be2864) · 시나리오 eval/scenarios 장애물 8종 · 구간 5종(48ba335) · Nav2+AMCL(sim/nav2, AMCL 7c96871) · 다리 sim/nav2/nav2_teacher.py 잡음 0.3 · FD_RECORD=1 sim/nav2/scenario_batch.sh(레포 d893e8e — 19개 -dirty 는 장애물 레이어 4개의 메타 한 줄 start_json 재생성뿐, 형상 같음), 시드 1~21` | D | `Nav2 교사 1차 — 녹화 21개(장애물 8종 × 양방향 16 + 구간 5), Nav2 판정 21/21 통과 · QC 21/21 OK · 충돌 0. 녹화 시뮬 40.7분(주행 38.2분), 카메라 48,794장(20 Hz), 라벨 /teacher/cmd_vel(= Nav2 명령), 주행 중 잡음 27.8 % · 0.3 m 넘는 이탈 35.6 % · 최대 1.65 m, 장애물과 최소 0.68 m. 준비 실패 3회(Nav2 기동 2 · 접속 멈춤 1)는 녹화 지우고 재시도` |
| 10.14 | `usd` | `parking` | v0.5 | `data/usd/parking_v0.5/` | 104M | ef13822c6ded | `B 맵 v0.5(B 형상 정리판 geometry_clean_5cm + 바닥 높이 지도 floor_height.npz) → C 메쉬 파이프라인(레포 d2c7828, recon/configs/map_v05.yaml), sha 는 usd/parking_mesh.usdc` | C | `r4 와 구조 같음(Visual · VisualFar · Collision · Boundary · 하늘 돔). 바닥 = B 바닥 높이 지도 한 장, 충돌 바닥 10 cm 격자(63만 삼각형, r4 20만), 씬 z 이동 +1.413(r4 +1.439), 좌표 x · y · 기준선은 r4 와 같음. 씬 루트 sim/scenes/parking/parking_v0.5.usda(시작 z −0.039) — D 10.14 확인: 기준선 띠 겹친 바닥 0 · 납작한 조각 0, 자동 교사 4바퀴 멈춤 0 · QC OK, RTF r4 와 같음(1.44)` |
| 10.14 | `nav2` | `parking_map` | v0.5 | `data/nav2/parking_v0.5/` | 3.2M | 90983408b8cf | `parking v0.5(ef13822c6ded) → sim/nav2/make_map.py --keepout eval/routes/parking_r3_island.yaml` | D | `r4 2판과 같은 방식(map.yaml /map AMCL 용 + map_keepout.yaml /map_nav). 기준선 ±2.35 m 장애물 칸 59(r4 105), 바깥 연석 · 물체가 더 잡혀 장애물 칸 전체는 +30 %. D 10.14 확인: Nav2+AMCL 기준선 양방향 2번씩 4/4 완주(복구 0), AMCL 오차 중앙 0.10~0.11 · 최대 0.21 m(r3 와 같음)` |
| 10.30 | `usd` | `parking` | v0.6 | `data/usd/parking_v0.6/` | 131M | 0f4bd2d82acb | `v0.5 형상 + 바닥 1 cm 사진 텍스처(C 파이프라인 recon/configs/map_v06.yaml · recon/pipeline/floor_texture.py), sha 는 usd/parking_mesh.usdc` | C | `v0.5 와 충돌(63만) · 원경 · 하늘 같음(D 10.30 pxr 해시 확인), 바닥만 Floor prim(6500×5125 텍스처, 차 33대 자리 아스팔트로 채움). 씬 루트 sim/scenes/parking/parking_v0.6.usda(v0.5 루트와 경로만 다름), Nav2 지도는 v0.5 그대로. D 10.30 확인: 원경 끔 기준선 시계 X-Mobility 완주(랩 167.4 s · 평균 이탈 0.63 m) · Nav2 완주(128.2 s · 0.24 m · 복구 0), RTF v0.5 와 같음`

**구분** — `usd` 씬·메쉬 · `teacher` 교사 주행 원본 로그 · `dataset` 학습용 HDF5 · `checkpoint` 모델 가중치 · `eval` 평가 결과 · `raw` 센서 원본(rosbag) · `nav2` Nav2 지도
