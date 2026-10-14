# D → E · 교사 데이터

- **넘기는 쪽 → 받는 쪽**: D 정동현 → E 서진석
- **상태**: 초안 · 합의 · **동결** 중 — (초안)
- **합의일 / 합의자**: —

> 이 문서가 두 파트 사이의 계약입니다. 항목을 바꿀 때는 **받는 쪽 동의 후** 커밋하고, 동결 후 변경은 버전을 올리세요.

## 규약

| 항목 | 값 | 비고 |
| --- | --- | --- |
| 원본 형식 | rosbag2 MCAP | 자동 교사(`sim/auto_teacher.sh`) 또는 수동 조작 동기 로그 — **10.13 멘토 회의: 자동 교사로 모아도 됨 · Nav2 주행도 학습 데이터로 써도 됨 · 특수한 상황을 학습시킬 때는 사람이 직접 조작해 만듦** |
| 학습 형식 | | HDF5 — 필드 목록 |
| 카메라 해상도 · fps | 640×480 `rgb8` · 기록 10 fps (발행 20 Hz 목표) | 수행계획서 2.3 — 바꾸면 모은 데이터를 버려야 함 |
| 조작 신호 | 선속도 v (m/s) · 각속도 ω (rad/s) — `/cmd_vel`. 키보드 + 가감속 램프(`sim/teleop_ramp.py`, 10.5 결정): 키는 목표만 정하고 `/cmd_vel` 은 20 Hz 연속값 | 상한 제안 v ≤ 0.8 m/s, \|ω\| ≤ 0.7 rad/s — Nav2 기본 설정과 같은 값(비교가 공정하도록), [topics.md](topics.md). 가감속 한도 v 0.5↑ · 1.0↓ m/s², ω 1.5↑ · 2.0↓ rad/s², 정지 키 v 2.0 m/s². 발행자는 이 노드 하나 — 리그는 구독 · 기록만(E 확인) |
| 자동 교사 라벨 | `/teacher/cmd_vel` (TwistStamped, 헤더 = 시뮬 시각, 20 Hz) — 교사의 깨끗한 명령. `/cmd_vel` 은 실행 명령(라벨 + 조향 잡음)이라 라벨로 쓰지 않음. **Nav2 교사 녹화도 같은 토픽 · 형식**(라벨 = Nav2 최종 명령, 10.14) | 10.12 제안 · E 확인 필요. 거르기용 `/teacher/status`(단계 · 이탈 · 잡음, Nav2 면 `src` "nav2" · 장애물 거리 `od`) — [topics.md](topics.md) |
| 동기 기준 | 시뮬 시간(`/clock`) | 10 fps 샘플링 방식은 E |
| 구역 · 시나리오 태그 | 경로 지시(0 직진 · 1 좌회전 · 2 우회전) 구간 | `eval/routes/` — 제안. 자동 교사는 녹화 때 `/route_command` 를 함께 냄(40° 넘는 꺾임의 꼭짓점 앞 5 m ~ 뒤 2 m) |
| 전달 방식 | | 1차 수집은 서버에서(9.29). rsync / 외장 SSD / 서버 학습 중 10.2 결정 |

## 인계 방법

- 실물 위치: server207 `$CD1/data/teacher/` — [`MANIFEST.md`](../../MANIFEST.md) 에 버전 기록
- **1차 교사 데이터(10.13)**: `data/teacher/raw_v1/` — 묶음 폴더 9개(`<시각>_auto_<cw|ccw>/` 안에 MCAP 1개 · `meta.yaml` · `qc.txt` · `teacher/summary.json`(바퀴별 이탈 · 잡음 통계)). 실제 맵 r3, 기준선 화단 3 m 선, 45바퀴 · 시뮬 약 2시간. 카메라는 20 Hz로 녹화했으니 10 fps 샘플링은 E 쪽에서
- **정답 위치(10.13~)**: 다음 녹화부터 `/ground_truth/tf`([topics.md](topics.md))가 들어감. 1차 데이터(`raw_v1`)엔 없으니 `/chassis/odom` + 출발 자세(`teacher/summary.json` 의 `start_pose` — 9개 모두 (−1.756, −7.621, −96.0°))로 계산(topics.md 변환식). `/teacher/status` 에도 map x · y 가 20 Hz로 있음
- **E2E 학습 · 추론 환경(서버)**: `envs/ml_env` — [server207.md](../server207.md)
- **Nav2 교사 데이터 1차(10.14)**: `data/teacher/nav2_v1/` — 녹화 21개(장애물 8종 × 양방향 16 · 구간 5), 시뮬 40.7분 · 14 GB, 잡음 0.3, Nav2 판정 21/21 통과 · QC 21/21 OK. 시나리오는 `meta.yaml` 의 `teacher.scenario`, 장애물 근처 장면은 `/teacher/status` 의 `od`(장애물 가장자리까지 m)로 고름. MANIFEST `parking_r3_nav2` v1.0
- **Nav2 교사 녹화(10.14~)**: `FD_RECORD=1 sim/nav2/scenario_batch.sh <시나리오>[:cw|ccw] …` — 장애물 · 구간 시나리오를 Nav2 + AMCL 이 몰고, 다리(`sim/nav2/nav2_teacher.py`)가 자동 교사와 같은 토픽을 냄. 저장 `data/teacher/nav2_v1/<시각>_nav2_<방향>/`(자동 교사와 같은 구성 + `teacher/` 에 시나리오 · 판정 JSON), `meta.yaml` 의 `teacher.node` · `teacher.scenario` · `teacher.pass` 로 구별 · 거름. 상태 `ph` 는 WAIT · FOLLOW · STOP(FOLLOW 만 쓰면 됨). 잡음은 기본 끔(`FD_NOISE`)
- 납기: 수집용 Proxy 씬 10.6 · Proxy 씬 10.11 · 캠퍼스 씬 11.16 (수행계획서 그림 2)
- 토픽 · frame_id · 좌표계: [`topics.md`](topics.md)

## 변경 이력

| 날짜 | 버전 | 변경 | 동의 |
| --- | --- | --- | --- |
| 9.29 | v0 | 시뮬 쪽 칸 채움 — 해상도 · 조작 신호 · 동기 기준 (D) | |
| 10.5 | v0 | 조작 신호 — 게임패드 보류, 키보드 + 가감속 램프로 결정하고 한도를 적음 (D) | |
| 10.5 | v0 | 조작 신호 — 상한 제안 ω 1.0 → 0.7(v 0.8 과 함께 Nav2 기본 설정과 같게) (D) | |
| 10.12 | v0 | 자동 교사 안 — 라벨 `/teacher/cmd_vel` · 상태 `/teacher/status` 추가, `/route_command` 를 녹화 때 냄, 원본 형식 비고 (D, 10.13 멘토 회의 질문 4) | |
| 10.13 | v0 | 1차 교사 데이터 위치 · 구성 추가(`data/teacher/raw_v1/`, MANIFEST `parking_r3_auto` v1.0) (D) | |
| 10.13 | v0 | 원본 형식 — 멘토 회의 결론(자동 교사 · Nav2 주행 · 특수 상황은 사람 조작), 기준선(화단 3 m 선) 채택 (D) | |
| 10.13 | v0 | 정답 위치 `/ground_truth/tf`(다음 녹화부터) · 1차 데이터 지도 좌표 구하는 법 · 서버 학습 · 추론 환경(`ml_env`) (D) | |
| 10.14 | v0 | Nav2 교사 녹화 — 라벨 · 지시 · 상태를 자동 교사와 같은 토픽으로(다리 `sim/nav2/nav2_teacher.py`), 저장 위치 · 구별법 (D) | |
| 10.14 | v0 | Nav2 교사 데이터 1차 위치 · 구성(`data/teacher/nav2_v1/`, MANIFEST `parking_r3_nav2` v1.0) (D) | |
