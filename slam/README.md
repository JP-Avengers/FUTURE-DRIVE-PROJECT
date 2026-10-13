# slam/ — Visual SLAM

- **담당**: B · 오윤성
- **목적**: RTAB-Map 기준선으로 실제 환경을 매핑하고, 궤적·맵 품질을 검증한다.

## 간트 작업

| 작업 | 기간 |
| --- | --- |
| RTAB-Map 기준선 · 맵 v0.1 | 09.29–10.11 |
| SLAM 튜닝 · 궤적 검증 · 최종 맵 | 10.12–11.06 |

## 입력 · 출력

- **받는 것**: A 의 rosbag2 · [`a2b_rosbag.md`](../docs/interfaces/a2b_rosbag.md)
- **넘기는 것**: 포인트클라우드 · 궤적 → C · [`docs/interfaces/b2c_map.md`](../docs/interfaces/b2c_map.md)

## 현재 상태 (2026-10-13)

- 맵 **v0.4a**(v0.3 기준 빈 곳 채우기)와 **v0.4b**(본네트 카메라 기준)를 C에게 넘겼다. 두 판의 차이는 [`docs/deliver/V04_A_B_비교.md`](docs/deliver/V04_A_B_비교.md)에 있다.
- 다음: v0.5(C 지적 2 색 투영 어긋남 측정, 남은 이음새 단차, 반시계 맵 칸 간격 압축 원인) → 최종 맵 10.30.

## 맵 버전

| 버전 | 날짜 | 데이터 | 바뀐 점 | 차로 바닥 빈칸 |
| --- | --- | --- | --- | --- |
| v0.1 | 10.06 | 10/5 지붕 카메라 시계 3회차 (`run11`) | 첫 맵. 수평 스케일 5~9% 과대 | — |
| v0.2 | 10.06 | 같은 데이터, depth 보정 후 다시 매핑 (`run18`) | Z' = Z/(1+0.007·Z) → 주차칸 폭 249.7cm(실측 250) | 6.8% |
| v0.3 | 10.07 | + 10/5 지붕 반시계 2회차 (`run20`, 비강체 정합) | v0.2를 그대로 두고 빈 곳만 채움 | 3.5% |
| v0.4a | 10.11 | + 10/9·10/10 본네트 카메라 9회차, 10/9 도보 R7 조각 | v0.3을 그대로 두고 빈 곳만 채움 | 1.3% |
| v0.4b | 10.11 | v0.4a와 같은 데이터 | 본네트 시계 맵을 기준으로 바닥 형상을 새로 만듦. 키프레임−메시 바닥 +6.2 → +3.0cm | 1.6% |

- 패키지 zip은 Git에 올리지 않고 C에게 직접 넘겼다: `FutureDrive_map_v0.4a.zip`(793MB, SHA-256 `8d9fcf28…`), `FutureDrive_map_v0.4b.zip`(778MB, `3f1ceb71…`).
- 판별 변경 내용은 [`docs/deliver/CHANGES.md`](docs/deliver/CHANGES.md)에 있다. 패키지 설명은 [`map_v0.4a_README.md`](docs/deliver/map_v0.4a_README.md)와 [`map_v0.4b_README.md`](docs/deliver/map_v0.4b_README.md), 카메라 장착값은 [`site_measurements.md`](docs/deliver/site_measurements.md)에 있다.
- 실행마다 쓴 데이터, 바꾼 파라미터, 지표는 [`runs.csv`](runs.csv)에 쌓는다.

## 환경

- WSL2 Ubuntu 24.04 · ROS 2 Jazzy · RTAB-Map 0.23.7 (`rtabmap_ros`)
- Python 가상환경 두 개
  - `~/slam_env`: rosbags, numpy, open3d 0.20. 매핑 지표용
  - `~/tsdf_env`: open3d **0.19.0**, scipy, opencv. TSDF·정합·패키지용. open3d 0.20은 TSDF가 빈 메시를 낸다.
- 설치: [`setup/`](setup/)
  1. `sudo bash 00_system.sh`
  2. `bash 10_user.sh`
  3. `bash 99_check.sh`
- 환경 함정과 그동안 알게 된 것: [`docs/NOTES.md`](docs/NOTES.md)

> `rtabmap*.db`, bag, 메시·점군은 커밋하지 않습니다 — [`MANIFEST.md`](../MANIFEST.md) 에 기록.

## 폴더

| 경로 | 내용 |
| --- | --- |
| [`scripts/`](scripts/) | 도구: 촬영 QC, depth 보정, 매핑, 내보내기, 지표, TSDF, 맵 정합, 패키지 조립·검증 |
| [`pipeline/`](pipeline/) | v0.4a·v0.4b를 만든 실행 순서 그대로의 스크립트 |
| [`setup/`](setup/) | 환경 설치 스크립트 |
| [`docs/`](docs/) | 작업 메모(`NOTES.md`), C에게 보낸 패키지 문서(`deliver/`) |
| [`runs.csv`](runs.csv) | 실행 기록 |

스크립트는 B 작업 폴더 구조를 가정한다.

| 경로 | 내용 |
| --- | --- |
| `~/b_slam/scripts/` | 이 저장소의 `slam/scripts/` |
| `~/data/<촬영>/bag` | 원본 bag |
| `~/data_corr/c0.007/<촬영>/bag` | depth 보정 bag |
| `~/maps/<run>.db` | RTAB-Map DB |
| `~/b_slam/exports/<run>/` | 내보낸 궤적, 점군, 키프레임 |
| `~/b_slam/deliver/` | C에게 넘기는 패키지 |

## 실행 방법

```bash
S=~/b_slam/scripts

# 1) 촬영 가져오기 · QC
$S/import_capture.sh <촬영폴더>                              # ~/data 로 복사, 체크섬 확인
~/slam_env/bin/python $S/field_qc_v2.py ~/data/<촬영>/bag cw   # fps · 누락 · 피치 · 진동

# 2) depth 거리 보정 bag (c = 0.007, ROS 2 source 상태의 시스템 python3)
python3 $S/depth_correct_bag.py ~/data/<촬영>/bag ~/data_corr/c0.007/<촬영>/bag 0.007

# 3) 매핑 (헤드리스, 재생~SLAM 단일 컨테이너) → ~/maps/<run>.db
IMU=true BAG_ALIGNED=true $S/run_rtabmap.sh ~/data_corr/c0.007/<촬영>/bag <run> 0.5
#    같은 방향 회차 이어 붙이기: INIT_DB=~/maps/<이전 run>.db 를 앞에 붙인다 (원본 DB는 그대로)

# 4) 내보내기 · 지표 → ~/b_slam/exports/<run>/
$S/export_run.sh <run>
~/slam_env/bin/python $S/run_metrics.py <run>

# 5) 원본 depth 형상 메시 (5cm TSDF)
~/tsdf_env/bin/python $S/raw_depth_tsdf.py <run> ~/data_corr/c0.007/<촬영>/bag [...]
```

맵 v0.4a·v0.4b를 다시 만드는 순서는 다음과 같다.

| 판 | 실행 순서 |
| --- | --- |
| 공통 준비 | `pipeline/v04_1010_chain.sh`: 10/10 회차 매핑과 v0.3 좌표 정합 |
| v0.4a | `pipeline/v04a_build.sh` |
| v0.4b | `v04b_1_multisession.sh` → `v04b_2_align.sh` → `v04b_3_build.sh` |

각 스크립트 첫 줄 주석에 앞서 있어야 하는 결과물이 적혀 있다.
