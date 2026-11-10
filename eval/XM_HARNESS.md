# X-Mobility 폐루프 평가 하네스 — E 초안

- **작성**: E 서진석 · **담당(채택 · 폐기)**: D 정동현
- **상태**: 초안(11.6). **ROS · Isaac Sim 에서 돌려 보지 않았습니다.** 판정 계산은 `sim/nav2/nav2_route.py` 의 문장과 같은 값인지 합성 궤적으로 대조했고, ROS 쪽은 가짜 rclpy 로만 확인했습니다(아래 "확인한 것 · 안 한 것").
- **파일**: [`xm_route.py`](xm_route.py)(한 회 — 판정 + 노드 실행) · [`xm_batch.sh`](xm_batch.sh)(일괄, `sim/nav2/scenario_batch.sh` 와 같은 순서)
- **근거**: `sim/nav2/nav2_route.py` · `sim/nav2/scenario_batch.sh` · [`results_schema.md`](results_schema.md) v0 · [`analyze_runs.py`](analyze_runs.py) · `ros2_ws/src/fd_xm_infer/` · `docs/interfaces/topics.md`

## 1. 목적

고정 시나리오 60회에서 X-Mobility(파인튜닝 · 제로샷)와 Nav2 를 **같은 판정 코드**로 비교합니다. Nav2 쪽 판정은 `nav2_route.py` 에 이미 있으므로, X-Mobility 하네스는 그 계산을 그대로 쓰고 `results_schema.md` v0 의 열을 한 회 한 행으로 냅니다. 같은 코드로 Nav2 JSON 도 다시 판정해 같은 열의 행을 만들 수 있습니다(`--rejudge`).

## 2. 사용법 (server207)

```bash
cd $CD1   # = ~/Capstone_Design_1, tmux 안에서
# ckpt 는 best 파일(동결 후보) — Drive 에서 받은 이름 그대로 두고 sha 를 docs/interfaces/e2d_model.md 버전 칸 · MANIFEST 와 대조
# (last.ckpt 는 마지막 에폭이라 동결 모델이 아님. 후보들은 Kaggle 파일 이름이 모두 epoch=8-step=10719.ckpt 라 sha 로만 구분)
sha256sum data/checkpoints/xm_ft_v0.1_aw30/checkpoints/xm_ft_v0.1_aw30_e8.ckpt | cut -c1-12   # aw30 4fdf249cda0a · v0.1 d86970ce1faf · aw30_p3 114284109957
# 일괄 — 노드 python 사전 확인(import, 실패면 코드 2 → FD_NODE_PYTHON 으로 바꿈) → 항목마다 레이어 → 씬 → xm_route.py(노드를 띄우고 끔) → 씬 끄기. 준비 실패면 최대 3번
FD_CKPT=data/checkpoints/xm_ft_v0.1_aw30/checkpoints/xm_ft_v0.1_aw30_e8.ckpt FD_SEEDS="1 2 3" FD_OUT=data/eval/1124_xmft \
  eval/xm_batch.sh eval/scenarios/parking_r3_straight_south.yaml eval/scenarios/parking_r3_obs_cones.yaml:ccw
# → data/eval/1124_xmft/results.csv · runs/<시나리오>[_방향]_s<시드>.json(+ .node.log · .xm_infer.csv · .xm_infer.meta.json) · route_*.log · layers/
#   끝에 eval/analyze_runs.py 로 스키마 검사(ANALYZE_RUNS_OK / FAIL)

# 씬이 이미 떠 있을 때 한 회(시스템 Jazzy · 도메인 43 셸)
python3 eval/xm_route.py --scenario eval/scenarios/parking_r3_obs_cones.yaml --direction ccw \
  --ckpt data/checkpoints/xm_ft_v0.1_aw30/checkpoints/xm_ft_v0.1_aw30_e8.ckpt --method xmobility_ft --seed 1 \
  --out data/eval/1124_xmft/runs/obs_cones_ccw_s1.json --csv data/eval/1124_xmft/results.csv
python3 eval/xm_route.py ... --dry-run     # ROS 없이 출발 자세 · 목표 지점 · 노드 명령만 출력

# Nav2 결과 JSON(nav2_route.py --out)을 같은 판정으로 행으로
python3 eval/xm_route.py --rejudge logs/nav2/scenario/parking_r3_obs_cones_ccw.json --scenario eval/scenarios/parking_r3_obs_cones.yaml \
  --method nav2 --model-version <nav2 설정 커밋> --localization amcl --nav2-map "v0.4b r3 · 2판" --seed 1 --csv data/eval/1124_nav2/results.csv
```

- 환경변수(`xm_batch.sh`): `FD_CKPT`(필수) · `FD_METHOD`(`xmobility_ft` 기본 · `xmobility_zeroshot`) · `FD_MODEL_VERSION` · `FD_SEEDS`(기본 `1`) · `FD_OUT` · `FD_CSV` · `FD_SCENE`(비면 yaml `scene`) · `FD_LIDAR`(기본 1) · `FD_XM_CCW_START`(`scene` 기본 · `align`) · `FD_NODE_PYTHON`(노드 python, 기본 `envs/ml_env/bin/python` — 사전 확인에 걸리면 `~/xmobility_test/venv/bin/python`) · `XM_REPO`(사전 확인용, 노드 기본값과 같음) · `FD_XM_ARGS`(예 `--intervene off`, `--node-param gin:=...` — 제로샷이면 gin 을 여기로, p3 계열 ckpt 면 `--node-param path_scale:=0.3333`).
- 노드는 `xm_route.py` 가 노드 README 의 "colcon 없이" 방식(`ros2_ws/src/fd_xm_infer/README.md:49-55`)으로 띄웁니다: `<--node-python> -u -m fd_xm_infer.node --ros-args -p ...`, `PYTHONPATH=ros2_ws/src/fd_xm_infer`, `CUDA_VISIBLE_DEVICES=1`, `HF_HUB_OFFLINE=1`(README:19). 런치 파일(`start_x` 등도 인자로 받음)은 쓰지 않고 직접 띄웁니다 — 자식 프로세스(새 세션)의 pid · 신호 · 로그를 하네스가 다루려고.
- 종료 코드: 0 = 판정한 회(통과 여부와 무관, CSV 한 행) · 3 = 준비 실패(행 없음, 일괄이 다시 띄움 — `scenario_batch.sh:96` 과 같은 규칙) · 130 = 신호로 중단. 일괄의 `timeout -k 20 900`(`scenario_batch.sh:89`)에 걸리면 124 → 다시 띄움, 남은 노드는 `<out>.node.pid` 로 끔.

### 2.1 DAgger 녹화 (D, 11.13 예정 — 노드 시험이 된 뒤)

모델이 모는 동안 자동 교사를 라벨 전용(`sim/teacher.py --label-only`, `/cmd_vel` 안 냄)으로 같이 띄워 그 자리 교사 라벨을 남깁니다(`e2e/PLAN.md` "DAgger"). 일괄 스크립트가 회마다 씬 → 라벨 교사 · 녹화(`auto_teacher.sh`) → 모델(`xm_route.py`, 2 m 넘게 벗어나면 개입으로 멈춤) → 교사 SIGTERM → 녹화 닫기 순서로 돌립니다. 장애물 없는 바퀴에서만(장애물 장면의 자동 교사 라벨은 '장애물로 직진'이라 틀림).

```bash
cd $CD1 && git pull
# cw · ccw × 시드 3 = 6 바퀴. 반시계는 출발 방향을 맞춤(align — 출발 제자리 회전 없이, xm_v3 와 같은 뜻)
FD_DAGGER=1 FD_XM_CCW_START=align FD_CKPT=<ckpt> FD_SEEDS="1 2 3" FD_OUT=data/eval/1113_dagger \
  eval/xm_batch.sh eval/scenarios/parking_r3_lap.yaml:cw eval/scenarios/parking_r3_lap.yaml:ccw
# → 녹화 data/teacher/dagger_v1/<묶음>/ (회마다 '  DAgger AUTO_REC …' · '  DAgger AUTO_DONE … qc=OK' 줄) + 모델 바퀴 결과 results.csv
# 내보내기 — 실행 명령(exec_command) = 모델이 낸 명령, 라벨 = 교사
source /opt/ros/jazzy/setup.bash
for b in data/teacher/dagger_v1/*/; do ~/xmobility_test/venv/bin/python sim/export_xmobility.py $b data/teacher/xm_dagger_v1 --scenario r3_dagger --split train --exec-cmd; done
```

- 시험하지 않은 부분(서버 없음): `FD_DAGGER` 분기(`eval/xm_batch.sh`)는 `bash -n` · 경로 추출만 확인. 첫 회 로그 `data/eval/1113_dagger/dagger_*.log` 에 `AUTO_REC` · `TEACHER_EXIT` · `AUTO_DONE` 이 다 있는지 보고 나머지를 돌려 주세요.
- 손으로 할 때(같은 순서): 씬(도메인 43) → `FD_DOMAIN=43 FD_TEACHER_DIR=data/teacher/dagger_v1 sim/auto_teacher.sh csv:eval/routes/parking_r3_cw.csv cw 1 parking_r3_lap_cw_dagger -- --label-only` 가 `AUTO_REC` 를 찍은 뒤 → `python3 eval/xm_route.py --scenario eval/scenarios/parking_r3_lap.yaml --direction cw --ckpt <ckpt> ...` → 끝나면 `pkill -TERM -f sim/teacher.py`(교사가 요약을 쓰고 끝나면 녹화도 닫힘).
- 모델이 몇 m 못 가고 개입되면 그 회는 짧은 녹화만 남습니다 — 되돌아오는 장면이 핵심이라 짧아도 씁니다.

## 3. 한 회 순서 (`xm_route.py` `run_live`)

| 단계 | 동작 | 출처 · 대응 |
| --- | --- | --- |
| 준비 1 | `/ground_truth/tf` 첫 메시지 15 s | `nav2_route.py:213-217` |
| 준비 2 | `/cmd_vel` 발행자 0 (2 s 발견 대기 뒤) — 있으면 코드 3 | `topics.md:40` · `:63`, 대기는 `node.py:466-469` |
| 준비 3 | odom + 출발 자세(노드와 같은 식)가 정답 위치와 0.5 m · 10° 안 — 아니면 코드 3 | 위치 0.5 m 는 `nav2_route.py:227` 의 AMCL 확인과 같은 값, 식은 `core.py:105-109` = `topics.md:22-23`. 방향 10° 는 여기서 정함 |
| 노드 | 자식 프로세스(새 세션)로 띄움 — `route_source=csv` · `pose_source=odom` · `route_csv` · `goal_s`(구간 = `goal.s`, 바퀴 = −1) · 출발 자세 · `seed` · `log_csv` | 노드 파라미터는 `node.py:249-260` |
| 시작 | 노드의 첫 `/cmd_vel` 을 받으면 궤적 기록 시작, 첫 표본 시각 = `t_goal` | `nav2_route.py:243`("목표를 보낸 시뮬 시각") |
| 표본 | 정답 위치 `World → nova_carter` 를 시뮬 0.1 s 마다 `(t, x, y, cte, 장애물 거리)` + 덧붙임 `(yaw, v_cmd, ω_cmd, odom v, 진행 s)` | `nav2_route.py:133-140`, 덧붙임 순서는 `results_schema.md` §4 |
| 끝내기 | ① cte(3자리 반올림) > `deviation_limit_m` → `INTERVENED`(`--intervene stop`, 기본) ② 기준선 진행이 마지막 경유점 `s[i_goal]` 을 지남 → `SUCCEEDED` ③ `t − t_goal > timeout` → `CANCELED` ④ 노드 종료 → `FAILED`(fail_reason `harness`) ⑤ (선택) `--stall-abort-s` 정지 → `CANCELED` | ① `results_schema.md` §3 · 장애물 yaml 주석 "넘으면 개입(자동 종료 · 리셋)" ② `i_goal` = `nav2_route.py:198-204` ③ `nav2_route.py:253-255` |
| 멈추기 | 노드에 SIGINT → (10 s) SIGTERM → (5 s) SIGKILL. 노드가 정지 명령 5번 내고 끝남 → 노드가 사라진 뒤 하네스도 정지 명령 5번 → odom \|v\| < 0.05 m/s 또는 시뮬 3 s 까지 표본 더 받음 | `node.py:434-443` · `:718-745`, 5번은 `sim/teleop_ramp.py:257-259` |
| 판정 | `summarize` = `nav2_route.py:258-289` 문장 순서 그대로 → JSON + CSV 한 행 + `XM_ROUTE_DONE` 한 줄 | §5 |

## 4. 결과 CSV 열 대응 (`results_schema.md` v0, 열 순서 = `analyze_runs.py:42-93` `COLS`)

| 열 | 값 | 어디서 |
| --- | --- | --- |
| `schema_version` | `v0` | `analyze_runs.py:40` |
| `run_id` | `<YYYYmmdd_HHMMSS>_<xmft\|xmzs\|nav2>_<시나리오(parking_r3_ 뺌)>[_방향]_s<시드>[_a<시도>]` — 시각은 `t_goal` 의 벽시계 | 예시(`results_schema.md:25`)에 초를 더함(같은 분 겹침 방지). 이름에 방향이 있으면(`curve_se_cw`) 한 번만 |
| `timestamp` | `t_goal` 벽시계, ISO 8601 + 시간대 | §2.1 |
| `repo_rev` | `git rev-parse HEAD` + `git diff --quiet HEAD` 실패면 `-dirty`, 회마다 | `node.py:84-93` 과 같은 dirty 판정 |
| `method` | `--method`(`xmobility_ft` 기본) | `analyze_runs.py:96` |
| `model_version` | `--model-version` > ckpt 경로의 `xm_ft_v<주>.<부>[_<변형>…]`(`xm_ft_v0.1` · `xm_ft_v0.1_aw30` · `_aw30_p3` · `_p3` · `_seed1`; 끝의 `_e<에폭>` 은 뗌 — `xm_ft_v0.1_aw30_e8.ckpt` → `xm_ft_v0.1_aw30`) > `<finetune 출력 폴더>/<ckpt>`(옆에 `run_meta.json` 있을 때) > (제로샷이면 `zeroshot:`)ckpt 파일 이름. JSON 과 CSV `note` 에 `ckpt_sha12`(노드 `.meta.json` 에서) — 실물은 sha 로 확인 | `e2d_model.md` 버전 칸 · MANIFEST `checkpoint` 행 · `e2e/finetune.py:173` |
| `scene` · `scene_version` | `--scene` > `$FD_SCENE` > yaml `scene` · 파일 이름에서 `v0.4b r3` 꼴 | MANIFEST 표기 |
| `scenario` · `scenario_version` · `direction` | yaml `name` · `version` · 구간은 yaml `direction`, 바퀴는 `--direction`(yaml `directions` 안인지 확인) | `results_schema.md:32-34` |
| `seed` | `--seed` = 반복 번호. 노드 `seed`(확산 정책 noise, `node.py:241`)에도 같은 값 | 짝 열쇠(`scenario` · `direction` · `seed`) |
| `result` | `SUCCEEDED` · `INTERVENED` · `CANCELED` · `FAILED` | §3 끝내기 |
| `pass` · 기준 6개 | `nav2_route.py:288-289` 의 AND 를 항목별로 — **같은 피연산자**(`max_cte` 는 3자리 반올림 값, 정지 · 목표 거리 · 달린 거리는 반올림 전 값). `pass == AND(기준)` 를 assert | `summarize` |
| `fail_reason` | 주 원인 하나: `harness` > `collision` > `max_dev` > `long_stop` > `timeout`(CANCELED) > `not_completed` > `goal` > `distance`, `:` 뒤에 위치(기준선 s) · 값. `INTERVENED` 는 `max_dev` 로 잡힘 | 코드 목록 `analyze_runs.py:95` |
| `mean_cte` · `max_cte` · `lap_sim_s` · `max_stall_s` · `collision` · `min_clearance` · `dist_m` · `plan_m` · `goal_err` · `over_2m_frac` | `nav2_route.py` JSON 과 같은 키 · 같은 계산 · 같은 반올림. 장애물 없으면 `collision=false` · `min_clearance` 빈칸 | `nav2_route.py:258-284` |
| `p95_cte` | `numpy.nanpercentile(c, 95)`, c = `mean_cte` 와 같은 표본 | `results_schema.md:61` |
| `interventions` | cte(3자리 반올림)가 한도를 **위로 넘는 순간 수**, 궤적 전체 — Nav2 JSON 에도 같은 함수(`count_interventions`). `stop` 모드면 0 또는 1 | `results_schema.md:66` |
| `log_path` | 회 JSON 경로(레포 루트 기준) | §4 |
| 선택 열 | `kind`(`analyze_runs.py:255-264` 규칙) · `route` · `wall_s` · 한도 4개(yaml 값, `timeout_s` 는 실제 쓴 값) · `rc`(0) · `attempt` · `note`(`pose_source` · `intervene` · `ckpt_sha12`). `localization` · `nav2_map` · `recoveries` · `amcl_*` 는 X-Mobility 에서 빈칸 | `results_schema.md` §2.4 |

회 JSON 은 `nav2_route.py --out` 과 같은 키(`route` · `result` · `spacing_m` · `waypoints` · `lap_sim_s` · `wall_s` · `mean_cte` · `max_cte` · `over_2m_frac` · `recoveries` · `max_stall_s` · [`scenario` · `min_clearance` · `collision`] · `goal_err` · `dist_m` · `plan_m` · `amcl_start_err` · `initialpose_resent` · [`segment`] · `pass`, 대괄호는 nav2 와 같은 조건일 때만, Nav2 전용 칸은 null) + 스키마 칸 · `criteria` · `t_goal` · `end_reason` · `start_check` · 노드 명령 · `track`(앞 5칸 = nav2, 3자리 반올림 = `nav2_route.py:293`).

## 5. Nav2 와 같은 것 — 옮긴 곳

`nav2_route.py` 는 모듈 수준에서 argparse · `rclpy.init()` · `BasicNavigator()` 를 실행해(`nav2_route.py:34-42` · `:116-118`) import 할 수 없습니다. 판정에 쓰는 순수 계산만 옮겨 적고 줄 번호를 달았습니다(전역 → 인자만 바꿈). 아래 `xm_route.py` 쪽은 함수 이름만 — 줄 번호는 고칠 때마다 밀려 뺐고, 각 함수 docstring 에 `nav2_route.py` 줄 번호가 있습니다.

| `xm_route.py` | `nav2_route.py` |
| --- | --- |
| `load_route` | 50-53 (기준선 CSV run 0, 닫힌 꺾은선) |
| `cte` | 62-67 |
| `FOOT` | 70 (= `nav2_params.yaml:212` footprint) |
| `build_obstacles` | 72-88 (= `sim/make_obstacles.py:34-39` 자리 계산) |
| `_seg_dist` · `_inside` · `clearance` | 91-113 |
| `quat_yaw` · `GT_PARENT/CHILD` | 139 · 135 |
| `SAMPLE_DT` 0.1 | 137 |
| `max_stall` · `STALL_V` · `STALL_STRIDE` | 259-266 |
| `Ctx` 의 `segment` · `route` · `i_goal` · `plan_len` · 한도 | 48-49 · 198-204 · 282 · 287 · 289 |
| `summarize` | 258-289 |
| 바퀴 기준선 `eval/routes/parking_r3_<방향>.csv` | `scenario_batch.sh:79` |
| `TIMEOUT_S` 300 | `scenario_batch.sh:89` 가 nav2_route 에 준 `--timeout 300`(yaml `timeout_s` 는 안 씀) |
| `xm_batch.sh` 씬 띄우기 · 끄기 · 재시도 · 15분 | `scenario_batch.sh:13-15` · `:21-24` · `:67-71` · `:75-78` · `:83-97` |

## 6. Nav2 와 다른 것

1. **도착 판단** — Nav2 는 자기 goal checker(`nav2_params.yaml:156` `xy_goal_tolerance` 0.25 m, AMCL 위치)로 끝냅니다. X-Mobility 노드는 멈추지 않는 경로 추종기라 하네스가 정답 위치의 기준선 진행(감은 s, 사영 창 = `core.py:173-188` 과 같은 back 1 · fwd 10 m)이 `s[i_goal]` 을 지나면 끝냅니다. 정답 위치를 종료 판단에만 쓰고 모델 입력에는 안 씀(`topics.md:39` 채점용 범위). 진행 거리로 거르므로 바퀴 출발 직후(마지막 경유점이 1 m 뒤) 도착으로 끝나는 일(`nav2_route.py:202` 주석의 Nav2 경우)은 생기지 않습니다(시험 T5b).
2. **개입** — Nav2 일괄은 2.0 m 를 넘어도 계속 달립니다(10.14 `corner_exit` 시계 2.07 m 로 완주 · 실패). X-Mobility 는 기본으로 넘는 순간 끝냄(`INTERVENED`). `pass` 는 어느 쪽이든 같음(`max_dev_ok` 거짓). `lap_sim_s` · `dist_m` · `goal_err` 는 끊긴 때까지라 다름(분석기는 랩을 통과한 회만 평균, `analyze_runs.py:397`). `--intervene off` 면 Nav2 와 같게 계속.
3. **목표 전 표본** — `nav2_route.py:258` 은 `mean_cte` · `max_cte` 에 궤적 **전체**(구독 시작부터, 목표를 보내기 전 예열 · AMCL 확인 수 s 포함)를 씁니다. X-Mobility 는 노드가 준비된 뒤 기록을 시작해 목표 전 표본이 거의 없습니다(첫 표본 = `t_goal`). 출발 자리에서 서 있는 표본은 cte ≈ 0 이라 Nav2 `mean_cte` 가 그만큼 낮게 나옵니다. 정지 · 거리는 둘 다 `t ≥ t_goal` 만.
4. **멈춘 뒤 표본** — 끝낸 뒤 로봇이 멈출 때까지(최대 시뮬 3 s) 표본을 더 받아 `lap_sim_s` · `goal_err` 에 들어감. Nav2 는 스스로 감속해 멈춘 시점에 끝남.
5. **레이어 위치** — `scenario_batch.sh:74` 는 `sim/scenes/parking/obstacles/<이름>.usda` 를 다시 써 작업 트리가 더러워졌습니다(`MANIFEST.md:40` "19개 -dirty 는 … start_json 재생성뿐"). 여기서는 `$FD_OUT/layers/` 에 씀(형상 같음).
6. **씬** — `scenario_batch.sh:22` 는 r3 고정, 여기는 yaml `scene`(또는 `FD_SCENE`). r3 시나리오는 같음, `parking_v05_lap` · `v06_lap` 만 다름.
7. **준비 확인** — Nav2 의 AMCL 0.5 m 확인 대신 odom + 출발 자세 확인(위치 0.5 m · 방향 10°). 반시계 바퀴에서 노드 기본 출발 자세(아래 질문 1)를 쓰면 여기서 코드 3 으로 멈춤.

## 7. D 에게 질문 (동결 전)

1. **반시계 바퀴 출발 방향** — 씬 루트는 시계 방향(yaw −96°, `sim/scenes/parking/parking_v0.4b_r3.usda:27`)으로 띄우고, 장애물 레이어는 바퀴 시나리오에서 로봇을 안 건드립니다(`make_obstacles.py:115` 는 `start.s` 일 때만). Nav2 는 제자리에서 돌아 출발하지만, X-Mobility 학습 데이터는 FOLLOW 구간만이라(MANIFEST `xm_v1` 행) 180° 돌기는 배운 적이 없습니다. (a) 같은 조건(`FD_XM_CCW_START=scene`, 기본) (b) X-Mobility 만 84° 로 띄움(`align` — `Nova_Carter_ROS` `xformOp:rotateZ` 만 덮는 레이어) (c) Nav2 도 84° 로 다시. 또 노드 기본값 주의: `start_x` 등이 nan 이면 CSV 옆 YAML `start` 를 쓰는데(`node.py:389-395`) `parking_r3_ccw.yaml:10` 은 84° 라 (a) 에서 180° 틀립니다 — 하네스는 항상 출발 자세를 넘김.
2. **위치 입력 `pose_source=odom`** — 평가 규칙(`topics.md:39` "평가는 센서로 구한 위치로")에 맞춰 odom + 출발 자세. 학습 데이터 경로는 정답 위치로 만들었습니다(`sim/export_xmobility.py:208-209`, 정답 위치 있으면 그것). odom 과 정답 위치 차는 1.4 cm · 0.8°(`topics.md:39`, r3 양방향)라 영향은 작다고 보지만 확인 필요. 서버 `tools/xm_route.py` 의 `/route`(`route_source=topic`)로 갈지도(그 도구가 정답 위치를 쓰면 같은 질문). 이름이 이 파일과 같아 헷갈리면 이 파일 이름을 바꿔 주세요.
3. **개입 정의**(`results_schema.md` §6 질문 2) — 지금은 넘으면 그 회 끝(0/1). 리셋 후 이어 가기는 로봇을 옮기는 수단이 없어 안 만들었습니다.
4. **제한 시간** — Nav2 일괄과 같게 300 s. yaml `timeout_s`(구간 120 · 바퀴 300)를 쓰려면 `--timeout-from-yaml` — 그러면 Nav2 도 같이 바꿔야 같은 조건.
5. **`mean_cte` 의 목표 전 표본**(§6-3) — 두 방법 모두 `t ≥ t_goal` 로 맞출지. 바꾸려면 `nav2_route.py:258` 한 줄(D 영역)과 `summarize` 를 같이.
6. **재시도 · 행**(`results_schema.md` §6 질문 3) — 준비 실패(코드 3 · 124)는 행을 안 쓰고 JSON(`result: PREP_FAIL`)만 남김. 주행 중 노드가 죽으면 `FAILED` · `harness:` 행을 씀(다시 안 돌림). 주행 중 시뮬이 멈추면(정답 위치 30 s 없음) 코드 3 → 다시 띄움(Nav2 일괄의 15분 → 124 와 같은 취급).
7. **LiDAR** — 기본 켬(`scenario_batch.sh:22` 와 같게 — RTF 를 Nav2 회와 맞춤, `topics.md:51-52`: 끄면 약 1.7, 켜면 1.27~1.32). `topics.md:45` 는 "수집·평가 설정에서는 끔". 노드 주기 · 상태는 시뮬 시간이라 RTF 가 다르면 추론 지연이 시뮬 시간으로 다르게 들어갑니다.
8. **공용 모듈** — `results_schema.md` §3 제안대로 판정 함수를 `eval/` 공용 모듈로 빼면 이 복사본이 필요 없어집니다. 그 전까지는 복사 + 시험(§8)으로 같음을 확인.
9. **Nav2 JSON 에 `t_goal` · `direction` 추가** — `nav2_route.py:283` 부근 한 줄이면 `--rejudge` 가 `lap_sim_s` 로 되짚는 근사(0.1 s 반올림) 없이 Nav2 행을 만듭니다.
10. **충돌 범위**(`results_schema.md` §6 질문 4) — 계산은 Nav2 와 같음(시나리오 장애물 다각형 vs 발자국 2D). 충돌해도 끝내지 않음(Nav2 와 같음) — 상자에 걸려 서면 제한 시간까지 감. `--stall-abort-s` 로 일찍 끝낼 수 있음(기본 끔, Nav2 일괄과 다름).

## 8. 확인한 것 · 안 한 것

확인(ROS · Isaac 없는 컨테이너, python 3.13 · numpy 2.5 · pandas 3.0, 시험은 scratchpad — 레포에 안 넣음):

- `py_compile` · `bash -n xm_batch.sh` 통과.
- **nav2_route.py 와 같은 값** — `nav2_route.py` 를 ast 로 읽어 그 파일의 문장(44-113 · 130-140 · 198-204 · 258-289행)을 그대로 실행한 결과와 대조. 8가지 합성 궤적(60 Hz 정답 위치, 바퀴 링크 변환 섞음)에서 `R` · `s` · `L` · `i_goal` · 장애물 다각형 · `plan_len` · 표본(`on_gt`) 전부 같고, 요약 13~16키(`mean_cte` · `max_cte` · `over_2m_frac` · `max_stall_s` · `min_clearance` · `collision` · `goal_err` · `dist_m` · `plan_m` · `lap_sim_s` · `scenario` · `segment` · `pass` …)가 모두 같음. `Episode` 가 이어 센 정지 시간 = 정지 루프(259-266)의 값.
- **기준별 판정** — 직진 통과(출발 전 2 s 정지 포함) · 남동 모서리 2.4 m 넘침(개입 → `INTERVENED` · `max_dev:s 12.2`, 끝까지 가면 `SUCCEEDED` 인데 `max_dev`) · 12 s 정지(`long_stop:11.7 s`) · 상자 가로지름(`collision:s 27.6`, `min_clearance` 0) · 0.5 m 만 움직이고 SUCCEEDED(`distance`, 같은 궤적에서 `Episode` 는 도착을 안 냄) · 반시계 라바콘 우회 통과(최소 거리 0.74 m) · 0.1 m/s 로 기어감(`CANCELED` → `timeout`). 결과 CSV 8행 → `python3 -I eval/analyze_runs.py <csv> --expect-n 8` **ANALYZE_RUNS_OK · 경고 0**. `CSV_COLUMNS` = `analyze_runs.COLS`(49열), `CRITERIA` · `SCHEMA_VERSION` · fail_reason 코드 = 분석기.
- **실주행 경로(가짜 rclpy)** — 가짜 rclpy · 메시지 + 대본 세계(정답 위치 · odom · `/cmd_vel`) + 가짜 노드 프로세스로 7가지: 도착(노드 파라미터 · `model_version` · 노드 rc 0 · pid 파일 정리 · 노드가 끝난 뒤 하네스 정지 명령 5번) · 개입 · 출발 방향 틀림(84°, 코드 3, 노드 안 띄움) · `/cmd_vel` 발행자 있음(코드 3) · 노드 시작 실패(코드 3) · 주행 중 노드 죽음(`FAILED` · `harness`) · 반시계 바퀴 장애물. 실주행 궤적 앞 5칸이 같은 메시지를 `nav2_route.py` 의 `on_gt` 에 넣은 결과와 같음. CSV 4행(준비 실패 3회는 행 없음) 분석기 통과, `--rejudge`(nav2 모양 JSON, `t_goal` 없음)로 만든 nav2 행과 합쳐 `--ref nav2` 짝 비교 1쌍 잡힘 · 경고 0.

안 한 것:

- **실제 ROS · Isaac Sim 실행** — QoS · 발견 지연, `get_publishers_info_by_topic` 결과, 실제 노드에 SIGINT 가 새 세션 그룹으로 가서 정지 명령 5번이 나가는지, `tmux` · `run_proxy.sh` GPU 사용 검사(`run_proxy.sh:55-57`)와 노드 정리 순서, `make_obstacles.py` START 줄 읽기(정규식은 `scenario_batch.sh:78` 에서 옮김), `align` 레이어가 실제로 로봇을 돌리는지.
- **노드 실행** — ml_env python 이 시스템 Jazzy rclpy(python 3.12)를 import 하는지(노드 README 가 적은 방식이지만 노드 자체도 아직 실제 실행 전, 노드 README "안 한 것"). 모델 로드 시간(`--ready-timeout` 300 s).
- **출발 확인 한도**(0.5 m · 10°)가 실제 씬에서 맞는지 — 물리 안착 · odom 원점.
- **실데이터 판정 대조** — 서버의 Nav2 JSON(`logs/nav2/scenario*`)을 `--rejudge` 로 다시 판정해 저장값과 같은지(다르면 `XM_ROUTE_REJUDGE_DIFF` 줄).
- **DAgger 같이 띄우기** — 이 하네스(모델이 /cmd_vel)와 `sim/teacher.py --label-only`(라벨 /teacher/cmd_vel 만) · `sim/record_teacher.sh` 를 한 번에 띄워 모델이 가는 자리의 교사 라벨을 녹화하는 것. 도구는 다 있고 묶은 적이 없음 — 순서 · 부탁은 `e2e/PLAN.md` "DAgger". /cmd_vel 발행자는 노드 하나여야 함(`sim/check_proxy_topics.py`)이므로 teacher 는 꼭 `--label-only`.
