# 60회 결과 CSV 스키마 — v0 (초안)

- **정리**: E 서진석 · **확인**: D 정동현(하네스가 이 열을 그대로 써야 함)
- **상태**: **초안 v0 (11.5)** — 리허설(11.16–23) 전에 D 확인 후 커밋으로 동결. 동결일: (미정)
- **근거**: `eval/README.md`(60회 · 재실행 없음 · 커밋 해시 필수 · 이탈 2.0 m · 정지 10 s · 2.0 m 개입), `eval/scenarios/*.yaml`(판정 한도 · 목표 반경), `sim/nav2/nav2_route.py`(판정 `pass` 와 결과 JSON 칸), `sim/README.md` Nav2 절(판정 6항목), `docs/interfaces/topics.md` 3장(평가 결과 = CSV + 주행 영상, `data/eval/`), `docs/interfaces/e2d_model.md`(모델 버전), `MANIFEST.md`(씬 · 지도 버전)
- **분석**: [`analyze_runs.py`](analyze_runs.py) 가 이 문서의 열 이름 · 형식을 그대로 검사합니다(열 정의는 스크립트의 `COLS`).

> 60회는 다시 돌릴 수 없으므로, 리허설에서 이 CSV 를 실제로 한 번 만들어 `python3 eval/analyze_runs.py` 가 오류 0 으로 통과하는지 확인한 뒤 동결합니다.

## 1. 원칙

- **한 행 = 한 회 주행(run).** 60회면 60행. 준비 실패로 하네스가 다시 띄운 시도(`scenario_batch.sh` 코드 3 · 124)는 행으로 남길지 D 확인 필요 — 남긴다면 `rc` · `attempt` 를 채우고, 분석 대상에서 뺄 규칙을 동결 전에 정합니다(§6 질문 3).
- **열 이름은 `sim/nav2/nav2_route.py` 결과 JSON 의 키를 그대로 씁니다**(`max_cte` · `mean_cte` · `lap_sim_s` · `max_stall_s` · `collision` · `min_clearance` · `goal_err` · `dist_m` · `plan_m` · `result` · `pass` …). X-Mobility 하네스도 같은 키 · 같은 계산으로 냅니다(§3).
- **파일**: UTF-8 CSV, 첫 줄 머리글, 쉼표 구분. 위치는 `data/eval/<묶음>/results.csv`(topics.md 3장 "평가 결과 — CSV + 주행 영상, `data/eval/`"). 여러 날로 나눠 돌리면 파일을 여러 개 두고 분석기에 함께 넘깁니다.
- **값 형식**: bool 은 `true`/`false`(대소문자 무관, `1`/`0` 도 받음) · 실수는 점 소수 · 빈칸 = 값 없음(허용된 열만) · 시각은 ISO 8601(시간대 포함).
- **출처 표시**: 아래 표의 "출처" 칸 — 문서 · 코드에 이미 있는 것은 그 위치, 문서에 없는 것은 **제안 (D 확인 필요)**.

## 2. 열

### 2.1 식별 · 버전 (필수)

| 열 | 형식 | 뜻 | 출처 |
| --- | --- | --- | --- |
| `schema_version` | str | 이 문서 버전 — 지금 `v0` | 제안 (D 확인 필요) |
| `run_id` | str, 겹치면 안 됨 | 회 식별자. 예 `20261124_0931_xmft_obs_cones_ccw_s3` | 제안 (D 확인 필요) |
| `timestamp` | ISO 8601 | 회 시작 벽시계 시각(목표를 보낸 때), 예 `2026-11-24T09:31:05+09:00` | 제안 (D 확인 필요) |
| `repo_rev` | 16진 7~40자 (`-dirty` 붙을 수 있음) | 실행한 레포 커밋 `git rev-parse HEAD`. 작업 트리가 더러우면 `-dirty` 를 붙임(MANIFEST 관례). 60회 전체가 한 값이어야 함 | `eval/README.md` "실행 시 커밋 해시를 결과 CSV 에 함께 남기세요" |
| `method` | str | 주행 방법. 제안 값: `xmobility_ft`(파인튜닝 동결 모델) · `xmobility_zeroshot`(공개 가중치) · `nav2`(비교군, AMCL) | 값 이름은 제안 (D 확인 필요). Nav2 = 비교군(`sim/README.md` Nav2 절 "비교군(수행계획서)"), 제로샷 주행 기록(`sim/README.md` 맵 v0.4b 절), 파인튜닝(`e2d_model.md`) |
| `model_version` | str | X-Mobility: MANIFEST `checkpoint` 행 버전 `xm_ft_v<주>.<부>` (+ 제로샷이면 공개 가중치 이름). Nav2: `nav2_params.yaml` 이 들어 있는 커밋 또는 설정 이름 | X-Mobility 는 `e2d_model.md` "버전" 칸. Nav2 쪽 표기는 제안 (D 확인 필요) |
| `scene` | str | 씬 루트 경로 — 시나리오 yaml 의 `scene`, 예 `sim/scenes/parking/parking_v0.4b_r3.usda` | `eval/scenarios/*.yaml` `scene` |
| `scene_version` | str | MANIFEST `usd` 행의 버전, 예 `v0.4b r3` · `v0.6` | 버전 표기는 `MANIFEST.md`. 열은 제안 (D 확인 필요) |
| `scenario` | str | 시나리오 yaml 의 `name`(= 파일 이름에서 `.yaml` 뺀 것), 예 `parking_r3_obs_cones` | `eval/scenarios/*.yaml` `name`. nav2_route.py JSON 의 `scenario` 는 yaml **경로**라 파일 이름만 씀 |
| `scenario_version` | str | 시나리오 yaml 의 `version`, 예 `v1` | `eval/scenarios/*.yaml` `version` |
| `direction` | `cw` / `ccw` | 진행 방향. 구간 시나리오는 yaml `direction`, 바퀴 시나리오는 `directions` 중 하나 | `eval/scenarios/*.yaml` · `scenario_batch.sh` 의 `yaml:cw` |
| `seed` | int ≥ 0 | 난수 시드 겸 **같은 조건 반복 번호**. 방법 사이 짝 비교 열쇠(`scenario` · `direction` · `seed`) | `scenario_batch.sh` 의 `SEED`(항목마다 1씩 증가 — 지금은 방법 간 짝 의미 없음). 짝 열쇠로 쓰는 것은 제안 (D 확인 필요) |

### 2.2 판정 (필수)

판정은 `nav2_route.py` 의 `pass` 와 같은 정의입니다(`sim/README.md` Nav2 절, 수행계획서 2.4) — 아래 기준 6개의 AND. nav2_route.py 는 `pass` 만 저장하므로 **기준별 bool 열은 제안 (D 확인 필요)** 이지만 계산식은 코드 그대로입니다. 한도는 시나리오 yaml 값(`deviation_limit_m` 2.0 · `stall_limit_s` 10 · `goal.radius_m` 1.0).

| 열 | 형식 | 계산(`nav2_route.py`) | 출처 |
| --- | --- | --- | --- |
| `result` | str | 하네스 종료 상태. Nav2: `SUCCEEDED` · `CANCELED`(제한 시간 초과로 취소) · `FAILED`. X-Mobility 하네스도 같은 말을 쓰고, 2.0 m 개입으로 끝나면 `INTERVENED` 를 제안 | `nav2_route.py` `result`. `INTERVENED` 는 제안 (D 확인 필요) |
| `pass` | bool | 아래 6개가 모두 참 | `nav2_route.py` `pass` |
| `completed` | bool | 완주 — `result == SUCCEEDED`(제한 시간 `timeout_s` 안에 끝남) | 계산 = 코드, 열은 제안 |
| `max_dev_ok` | bool | `max_cte <= deviation_limit_m`(2.0 m) | 계산 = 코드, 열은 제안 |
| `no_collision` | bool | `not collision` | 계산 = 코드, 열은 제안 |
| `no_long_stop` | bool | `max_stall_s < stall_limit_s`(10 s) | 계산 = 코드, 열은 제안 |
| `goal_reached` | bool | `goal_err <= goal.radius_m`(1.0 m) — 마지막 경유점(바퀴면 결승 1 m 앞, 구간이면 `goal.s`)까지 거리 | 계산 = 코드, 열은 제안 |
| `distance_ratio_ok` | bool | `dist_m >= 0.8 × plan_m` | 계산 = 코드, 열은 제안 |
| `fail_reason` | str, 통과면 빈칸 | 실패의 **주 원인** 코드 하나 + 필요하면 `:` 뒤 짧은 설명. 코드: `not_completed` · `timeout` · `max_dev` · `collision` · `long_stop` · `goal` · `distance` · `harness`. 예 `max_dev:남동 모서리 s 14`. 기준별 실패 수는 위 bool 열로 따로 셈 | 제안 (D 확인 필요) |

### 2.3 지표 (필수)

위치는 정답 위치 `/ground_truth/tf`(World → `nova_carter`)를 시뮬 0.1 s 마다 뽑은 궤적, 이탈(cte)은 그 점에서 **기준선 CSV(`run == 0` 점들을 이은 닫힌 꺾은선)** 까지 거리입니다 — `nav2_route.py` 의 `on_gt` · `cte()`. 구간 시나리오도 기준선 전체까지 거리로 잽니다.

| 열 | 형식 · 단위 | 계산 | 출처 |
| --- | --- | --- | --- |
| `mean_cte` | float, m | 궤적 전체 cte 평균 (요청 이름 mean_dev_m) | `nav2_route.py` `mean_cte` |
| `max_cte` | float, m | 궤적 전체 cte 최댓값 (요청 이름 max_dev_m) | `nav2_route.py` `max_cte` |
| `p95_cte` | float, m | 같은 표본의 95 백분위(`numpy.percentile(c, 95)`) — 모서리 한 점에 끌리는 최댓값을 보완 | 제안 (D 확인 필요) |
| `lap_sim_s` | float, s(시뮬) | 목표를 보낸 시각부터 끝날 때까지. 바퀴면 랩 시간, 구간이면 구간 시간. 실패한 회는 끊긴 때까지라 **분석은 통과한 회만** 평균함 (요청 이름 lap_time_s) | `nav2_route.py` `lap_sim_s` |
| `max_stall_s` | float, s(시뮬) | 가장 긴 정지 — 0.5 s 간격 속도가 0.05 m/s 미만으로 이어진 시간의 최댓값, 목표를 보낸 뒤부터 (요청 이름 longest_stop_s) | `nav2_route.py` `max_stall_s` |
| `collision` | bool | 로봇 발자국(`FOOT`, nav2_params.yaml footprint)과 장애물 다각형 사이 거리가 한 번이라도 0. **장애물 없는 시나리오는 `false`** (요청 이름 collisions — 지금 코드는 횟수가 아니라 있음/없음) | `nav2_route.py` `collision` |
| `min_clearance` | float, m · 빈칸 허용 | 로봇 발자국과 가장 가까운 장애물 사이 최소 거리(겹치면 0). 장애물 없는 시나리오는 빈칸 (요청 이름 min_obstacle_dist_m) | `nav2_route.py` `min_clearance` |
| `interventions` | int ≥ 0 | 2.0 m 개입 횟수 — cte 가 `deviation_limit_m` 을 **위로 넘는 순간마다 1**. 하네스가 넘자마자 종료 · 리셋하면 회마다 0 또는 1 | "2.0 m 개입"은 `eval/README.md`, "넘으면 개입(자동 종료 · 리셋)"은 장애물 시나리오 yaml 주석. 세는 방법은 제안 (D 확인 필요) |
| `dist_m` | float, m | 목표를 보낸 뒤 정답 위치를 이어 잰 실제 주행 거리 (요청 이름 distance_m) | `nav2_route.py` `dist_m` |
| `plan_m` | float, m | 경로 길이 — 바퀴면 기준선 한 바퀴 길이 `L`(r3 89.96 m), 구간이면 `goal.s − start.s` (요청 이름 route_length_m) | `nav2_route.py` `plan_m` |
| `goal_err` | float, m | 끝났을 때 마지막 경유점까지 거리 | `nav2_route.py` `goal_err` |
| `log_path` | str | 이 회의 궤적 · 로그 파일 경로(`$CD1` 기준), 예 `data/eval/1124/<run_id>.json` | 제안 (D 확인 필요). 형식은 §4 |

### 2.4 선택 열 (있으면 검사 · 분석에 씀)

| 열 | 형식 | 뜻 | 출처 |
| --- | --- | --- | --- |
| `kind` | `straight` · `curve` · `obstacle` · `lap` | 시나리오 종류. 비면 분석기가 이름(`_straight_` · `_curve_` · `_obs_` · `_lap`)으로 채움 | 종류 구분은 `eval/README.md` 시나리오 절. 열은 제안 |
| `localization` | `amcl` · `gt` · 빈칸 | Nav2 위치 추정 방식. X-Mobility 는 빈칸 | `sim/README.md` Nav2 절 |
| `nav2_map` | str | MANIFEST `nav2` 행 버전, 예 `v0.4b r3 · 2판` (Nav2 만) | `MANIFEST.md`. 열은 제안 |
| `route` | str | 기준선 CSV 경로 | `nav2_route.py` `route` |
| `over_2m_frac` | float 0~1 | 궤적 중 cte > 2.0 m 표본 비율 | `nav2_route.py` `over_2m_frac` |
| `recoveries` | int | Nav2 복구 동작 횟수 | `nav2_route.py` `recoveries` |
| `wall_s` | float, s | 벽시계 주행 시간 | `nav2_route.py` `wall_s` |
| `amcl_start_err` · `initialpose_resent` | float m · int | 출발 때 AMCL 위치 오차 · `/initialpose` 다시 보낸 횟수 | `nav2_route.py` 같은 키 |
| `deviation_limit_m` · `stall_limit_s` · `goal_radius_m` · `timeout_s` | float | 이 회에 쓴 판정 한도(yaml 값). 비면 분석기는 2.0 · 10 · 1.0 · (검사 안 함)으로 봄 | `eval/scenarios/*.yaml`. 열은 제안 |
| `rc` · `attempt` | int | `nav2_route.py` 종료 코드 · `scenario_batch.sh` 시도 번호(1~3) | `scenario_batch.sh`. 열은 제안 |
| `note` | str | 자유 메모 | 제안 |

## 3. X-Mobility 하네스가 맞출 것 (제안, D 확인 필요)

- cte · 정지 · 거리 · 목표 거리 · 장애물 거리는 `nav2_route.py` 의 함수(`cte` · `clearance` · 정지 루프 · `FOOT`)와 **같은 코드**로 계산 — 방법 사이 차이가 계산 차이가 아니게. 함수를 공용 모듈로 빼는 것을 제안합니다.
- 표본: `/ground_truth/tf` 를 시뮬 0.1 s 마다. 판정 · 채점에만 쓰고 모델 입력에는 쓰지 않음(topics.md `/ground_truth/tf` 칸, 10.13 멘토).
- `completed` 의 X-Mobility 뜻: 제한 시간 `timeout_s` 안에 마지막 경유점 `goal.radius_m` 안에 들어와 하네스가 정상 종료. 2.0 m 개입으로 끝나면 `result = INTERVENED`, `completed = false`.
- 개입 뒤 리셋해서 **같은 회를 이어 가는지**(→ `interventions` ≥ 2 가능), 그 회를 끝내는지(→ 0/1)는 동결 전에 정합니다.

## 4. 회마다 시계열 로그 (선택, 제안)

`log_path` 가 가리키는 JSON — `nav2_route.py --out` 과 같은 모양(요약 키 + `track`). X-Mobility 하네스도 같은 모양을 쓰면 같은 도구로 그림을 그릴 수 있습니다.

```json
{"route": "...", "result": "SUCCEEDED", "max_cte": 1.41, "...": "...",
 "track": [[t_sim, x, y, cte, clearance], ...]}
```

- `track` 칸: 시뮬 초 · x · y(B 맵 좌표) · cte(m) · 장애물 거리(m, 장애물 없으면 −1) — `nav2_route.py` 와 같음, 0.1 s 간격.
- 추가 제안(X-Mobility): `yaw` · 명령 `v` · `ω`(`/cmd_vel`) · `/route_command` · 개입 표시. 칸 순서를 바꾸지 말고 뒤에 붙입니다.
- 주행 영상은 topics.md 3장대로 `data/eval/` 에 두고 `run_id` 를 파일 이름에 넣습니다.

## 5. 분석기가 하는 검사 (`analyze_runs.py`)

| 종류 | 검사 | 걸리면 |
| --- | --- | --- |
| 열 | 필수 열 누락 · 형식(bool · 실수 · 정수 · 시각) · 빈칸 불가 열의 빈칸 · 음수 | 오류(종료 코드 1) |
| 버전 | `schema_version` ≠ `v0` · `repo_rev` 형식 · `repo_rev` 가 두 값 이상 | 오류(여러 값은 `--allow-mixed-rev` 면 경고) |
| 중복 | `run_id` 겹침 | 오류 |
| 판정 | `pass` ≠ 기준 6개 AND · `direction` 이 cw/ccw 아님 | 오류 |
| 정합 | 기준 bool 이 수치 열 · 한도와 안 맞음 · `interventions` 와 `max_cte` 모순 · 실패인데 `fail_reason` 빈칸 · 모르는 `fail_reason` 코드 · `eval/scenarios/` 에 없는 시나리오 · yaml 과 다른 방향 · 버전 · `-dirty` · 행 수 ≠ `--expect-n` | 경고 |

## 6. 동결 전 정할 것 (D 확인)

1. **60회 구성** — 방법(파인튜닝 · 제로샷 · Nav2 중 무엇) × 시나리오(장애물 8종 × 2방향 · 구간 5종 = 21가지 중 무엇) × 반복 수. 문서에 없음. 분석기는 구성과 무관하게 돌지만, 방법 비교는 같은 `scenario` · `direction` · `seed` 짝이 있어야 짝 검정이 됨.
2. **개입 정의** — 2.0 m 를 넘으면 그 회를 끝내는지(0/1), 리셋 후 이어 가는지(횟수). "리셋"이 기준선 위 어디로 되돌리는지.
3. **준비 실패 · 재시도** — `scenario_batch.sh` 는 준비 실패(코드 3 · 124)면 최대 3번 다시 띄움. 재시도한 회를 60회에 세는지, 행으로 남기는지.
4. **충돌 판정** — 지금은 시나리오 장애물(상자 · 원기둥 · 원뿔)과 로봇 발자국의 2D 겹침만 셈. 화단 경계석 · 연석 · 지도 물체와의 접촉, PhysX 접촉 보고는 안 셈. 이대로 갈지.
5. **`seed` 의 뜻** — 시뮬이 결정적이면 같은 시드 = 같은 조건인지. Nav2 는 AMCL 난수가 있음.
6. **씬** — 60회를 r3 로 할지 v0.6(시나리오 yaml 은 r3 를 가리킴) 등 다른 판으로 할지 — `scene` · `scene_version` 값.
7. **평가 4종** — `eval/README.md` 는 직진 · 곡선만 이름이 있고 교차는 "아직 못 정함", 네 번째는 적혀 있지 않음. `kind` 값 목록을 확정.

## 변경 이력

| 날짜 | 버전 | 변경 | 동의 |
| --- | --- | --- | --- |
| 11.5 | v0 | 초안 — 열 · 판정 · 시계열 로그 제안, 분석기 `eval/analyze_runs.py` (E) | |
