# eval/ — 평가

- **담당**: D·E · 정동현 · 서진석
- **목적**: 고정 시나리오 60회 실행으로 E2E 주행을 검증하고 지표를 해석한다.

## 간트 작업

| 작업 | 기간 |
| --- | --- |
| 평가 하네스 · 자동 리셋 · 추론 노드 (D) | 10.19–11.06 |
| 평가 하네스 리허설 (D) | 11.16–11.23 |
| 고정 시나리오 60회 실행 (D) | 11.24–11.30 |
| 60회 결과 분석 · 지표 해석 (E) | 11.24–11.30 |

## 입력 · 출력

- **받는 것**: E 의 동결 모델 · D 의 통합 씬
- **넘기는 것**: 결과 CSV · 지표 → 결과발표 · 최종보고서

## 환경

server207 · 하네스는 `isaacsim_env`, 분석은 `eval_env`.

> **60회는 재실행 여지가 없습니다.** 시나리오·지표·로그 스키마를 리허설(11.16–23) 전에 커밋으로 고정하고, 실행 시 커밋 해시를 결과 CSV 에 함께 남기세요.

## 기준선 · 경로 파일 — `eval/routes/`

평가(기준선 대비 이탈 · 2.0 m 개입), 자동 교사(`sim/teacher.py`), 경로 입력(X-Mobility `/route`)이 **같은 선**을 씁니다.

| 씬 | 정의 | 생성 파일 | 상태 |
| --- | --- | --- | --- |
| 실제 주차장 r3 | `parking_r3_island.yaml` — 가운데 화단 윤곽(꼭짓점 5)을 d 3.0 m 바깥으로 부풀린 선(변 평행 이동 + 반경 3 m 원호), 89.96 m | `parking_r3_cw/ccw.csv`(0.25 m 간격 run,s,x,y) · `.yaml`(길이 · 출발 자세 · 경로 지시 구간) | **채택(10.13 멘토 회의)** — 꼭짓점은 D 눈 초안 ±1 m, B · C 확인 전 |
| Proxy | `loop_cw.yaml` · `loop_ccw.yaml` — 지도 추정 루프 중심선(꼭짓점 4) | — | 초안 |

```bash
python3 eval/routes/make_refline.py eval/routes/parking_r3_island.yaml   # 꼭짓점 · d 를 고친 뒤 다시 생성(서버 시스템 python3)
```

- 촬영 궤적(`data/usd/*/route_bonnet_cw.csv`)은 외곽을 담으려고 일부러 크게 · 작게 돈 경로라 기준선으로 쓰지 않습니다(4회차가 같은 지점에서 최대 10 m 다름).
- 씬 시작 자세(`sim/scenes/parking/parking_v0.4b_r3.usda`)는 생성 YAML 의 `start` 와 같게 둡니다 — 기준선을 다시 만들면 같이 고칩니다.
- 장애물 여유 검사(±2.35 m 띠 = 이탈 2.0 + 로봇 반폭 0.35)와 위에서 본 그림은 메쉬가 필요해 서버 `logs/refline/` 도구로 합니다.

## 실행 방법

(작성 예정 — 명령 한 줄이라도 동작하는 것부터 적어 주세요)

## 시나리오 — `eval/scenarios/` (10.14 ~)

시나리오 파일 하나를 E2E · Nav2 · 평가 스크립트가 함께 씁니다(형식은 Proxy 계획 9장 초안을 따름). 들어가는 것: 씬 · 기준선 · 방향 · 시작 자세 · 목표 · 장애물 · 판정 한도(이탈 2.0 m · 정지 10 s, 수행계획서 2.4) · 제한 시간.

- **장애물 시나리오**(`parking_r3_obs_*`, 8종) — 기준선 한 바퀴(시계 · 반시계 둘 다), 기준선 위 장애물을 피함. 장애물 자리는 시계 방향 `s` 로 적고 반시계로 돌아도 같은 자리.
- **구간 시나리오**(`parking_r3_straight_*` · `parking_r3_curve_*`, 5종) — 평가 4종 중 직진 · 곡선. 기준선의 `start.s` → `goal.s` 만 달림(목표 반경 1.0 m). 로봇은 `start.s` 의 기준선 위에서 진행 방향을 보고 출발(`sim/make_obstacles.py` 레이어가 출발 자세를 덮어씀). `commands` 는 그 구간의 경로 지시(E2E 평가용).
- **교차**는 r3 에 갈림길이 없어(기준선이 화단 둘레 한 바퀴) 아직 못 정함.

| 파일 | 종류 | 내용 | Nav2 + AMCL(10.14) |
| --- | --- | --- | --- |
| `parking_r3_obs_south_box.yaml` | 장애물 | 남쪽 변 가운데(s 28) 상자 0.6×0.6×0.8 m — 기본 회피(v0 `parking_r3_obstacle` 의 이름만 바꿈) | 양방향 통과 |
| `parking_r3_obs_south_left.yaml` | 장애물 | 같은 자리에서 바깥(통로) 쪽 0.6 m | 양방향 통과 |
| `parking_r3_obs_south_right.yaml` | 장애물 | 같은 자리에서 화단 쪽 0.6 m | 양방향 통과 |
| `parking_r3_obs_slalom.yaml` | 장애물 | 남쪽 변 s 22(화단 쪽 0.5 m) · s 32(바깥 0.5 m) 상자 둘 — 연달아 피하기 | 양방향 통과 |
| `parking_r3_obs_corner_exit.yaml` | 장애물 | 남동 모서리를 빠져나온 직후(s 17.5) 상자 — 회전 중이라 늦게 보임 | 반시계 통과 · **시계 2회 이탈 초과**(2.07 · 2.02 m, 모서리) |
| `parking_r3_obs_east_start.yaml` | 장애물 | 출발점 4.75 m 앞(동쪽 변) 상자 — 시계는 출발 직후, 반시계는 도착 직전 | 양방향 통과 |
| `parking_r3_obs_north_ped.yaml` | 장애물 | 북쪽 변 s 74.2 사람 크기 원기둥(지름 0.5 · 높이 1.7 m) | 양방향 통과 |
| `parking_r3_obs_cones.yaml` | 장애물 | 북쪽 변 s 68 라바콘 둘(밑지름 0.4 · 높이 0.7 m — 가운데 · 화단 쪽 0.7 m) | 양방향 통과 |
| `parking_r3_straight_south.yaml` | 구간 · 직진 | 시계 s 16 → 40(남쪽 변 24 m) | 통과 |
| `parking_r3_straight_north_ccw.yaml` | 구간 · 직진 | 반시계 s 5.5 → 26(북쪽 변 20.5 m) | 통과 |
| `parking_r3_curve_se_cw.yaml` | 구간 · 곡선 | 시계 s 3 → 21(남동 모서리 우회전) | 통과 |
| `parking_r3_curve_se_ccw.yaml` | 구간 · 곡선 | 반시계 s 69 → 87(남동 모서리 좌회전) | 통과 |
| `parking_r3_curve_west_cw.yaml` | 구간 · 곡선 | 시계 s 37 → 66(서쪽 끝 꺾임 연달아) | 통과 |

- 실행은 `sim/nav2/scenario_batch.sh`(항목마다 씬 + Nav2 를 새로 띄움), 판정 · 주의는 `sim/README.md` Nav2 절. 시나리오 그림은 `python3 eval/scenarios/plot_scenarios.py`(장애물 = 숫자, 구간 = 글자).
- 장애물 자리는 기준선 길이 `s` 와 옆 오프셋 `lateral`(진행 방향 왼쪽 +)로 적습니다 — 기준선을 다시 만들어도 같은 뜻. `sim/make_obstacles.py` 가 월드 좌표와 바닥 높이를 붙여 USD 레이어로 만들고, 씬은 `FD_LAYERS` 로 얹어 띄웁니다.

