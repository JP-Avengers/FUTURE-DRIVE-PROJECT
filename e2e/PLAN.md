# E 파트 계획 — 11.7 → 11.30 (xm_ft)

일정: 11.16 기능 동결 · 11.16–23 재학습 · 11.23 모델 동결 · 11.24–30 60회 폐루프 평가(D 서버). 지금 상태는 [REPORT_DRAFT.md](REPORT_DRAFT.md) 한계 10–13, 결과는 [results/](results/).

## 모델 고르는 규칙

1. **폐루프 결과가 있으면 그것으로** — 통과율 → 평균 cte → 충돌 순(`eval/XM_HARNESS.md` 기준).
2. 없으면 오프라인, **같은 test(v1.1)** 에서 `--common` 으로: ① 자동 교사 ω 기울기 0.86 유지(`steer_diag.py`) ② Nav2 기울기 상승 ③ 2 s 적분 가로 이탈(`rollout_drift.py`) 자동 교사 0.027 m 유지 · Nav2 0.168 m 하락 ④ val `action` 손실 최소. ①·③의 자동 교사가 나빠지면 탈락.
3. 같은 설정 시드 2개 차이보다 작은 개선은 "같다"로 본다(시드 실험 1회가 필요한 이유).

## 제일 큰 알고리즘 지렛대: DAgger (모델이 몰고 교사가 라벨)

행동 복제의 폐루프 실패는 "모델이 조금 벗어난 자리"를 학습 데이터에서 본 적이 없어서 생긴다. 잡음 녹화(아래 "확인한 것")가 일부를 덮지만, **모델이 실제로 가는 자리**의 라벨이 제일 정확하다. 도구는 이미 있다 — 묶는 것만 필요.

| 단계 | 도구 | 상태 |
|---|---|---|
| 모델이 운전 | `eval/xm_route.py`(노드 `fd_xm_infer` 띄움, /cmd_vel 발행) | 11.7–9 D 시뮬 시험에서 처음 돌아감 |
| 교사가 같은 자리 라벨 | `sim/teacher.py --label-only` — /cmd_vel 안 내고 /teacher/cmd_vel · /teacher/status 만(D 가 DAgger 용으로 만들어 둠, teacher.py:13) | 있음 · 모델 노드와 같이 띄운 적 없음 |
| 녹화 | `sim/record_teacher.sh`(auto_teacher.sh 가 쓰는 것과 같음) | 있음 |
| 내보내기 | `sim/export_xmobility.py <묶음> <출력> --scenario r3_dagger --split train` — 라벨은 /teacher/cmd_vel 이라 그대로 맞음 | 있음 |
| 점검 · 학습 | `e2e/check_xm_dataset.py` → 기존 train + `r3_dagger` 로 재학습(로더가 train 아래 시나리오를 다 읽음) | 있음 |

- **자동 교사(경로 추종)만** 쓴다 — 장애물 장면에서는 자동 교사 라벨이 "장애물로 직진"이라 틀림. 장애물 없는 r3 코스에서 cw · ccw 각 3–5바퀴면 충분(한 바퀴 ≈ 2–3분 시뮬, 5 Hz 로 약 600–900 프레임). `sim/nav2/nav2_teacher.py` 는 label-only 가 없어 Nav2 DAgger 는 이번엔 안 함.
- DAgger 데이터는 **train 에만**(val · test 는 그대로). 섞는 비율은 그냥 합침(기본 DAgger).
- 모델이 2 m 넘게 벗어나면 하네스가 끝내므로, 그 전까지의 되돌아오는 구간이 핵심 데이터.
- 일정: 11.7–9 노드 시험 → 11.10–13 D 녹화(모델 v0.2 또는 v0.1) → export → 11.16–23 재학습에 포함. 노드 시험이 11.13 넘게 늦어지면 접는다.
- D 에게 부탁할 것: ① `xm_route.py` 로 모델이 모는 동안 `teacher.py --label-only --route … --dir cw|ccw --laps N` + `record_teacher.sh` 를 같이 띄워 줄 수 있는지 ② 녹화 묶음을 `--scenario r3_dagger` 로 export. (둘 다 제가 서버 없이 시험 못 하는 부분.)

## Kaggle 예산

T4×2, 주 30 h. 1회 학습: v0.1 데이터 10에폭 4 h 40 m · v1.2(1.5배) 10에폭 ≈ 7 h → 주 3–4회. 자취방 RTX 5060 Ti(16 GB, 배치 2×누적 2)로 돌아가면 두 곳에서 나눠 돌림(`SETUP_LOCAL.md`).

## 날짜별

| 날짜 | 할 일 | 판단 |
|---|---|---|
| 11.7 | v0.2 결과: ZS · v0.1 · v0.2 `steer_diag --common` · `rollout_drift --common` · `fill_report` · `weak_segments`. D 답(시뮬 시험 · 녹화 · v1.1 · 파일 수) 반영 | Nav2 ↑ 이고 자동 교사 유지 → v0.2 후보. 자동 교사 ↓ → v0.3(교사 역할 나누기) |
| 11.7–9 | D 시뮬 시험에서 노드 오류 나오면 수정(11.16 전 마지막 기능 변경). 흔들림(weaving) 보이면 ω 스무딩 · dω/dt 한도 플래그(기본 off) 추가. 학습 1회(`--save-top-k 3`): v0.2 결과 따라 (a) v0.3 = 코너는 자동 교사만 · Nav2 는 장애물 3 m 안 구간만(train 752프레임 · 15구간 · 중앙값 50프레임) 또는 (b) v0.2 설정 20에폭(OneCycle 이라 9에폭 끝에서 lr 소진 · val 0.3224→0.3244 평탄) | |
| 11.10–13 | **DAgger 녹화(D)** → `r3_dagger` export → 점검. D 장애물 녹화(같은 쪽) 오면 `resplit_xm`(v1.3). 집에 가면 `setup_windows.ps1` + 스모크 | |
| 11.14–16 | 기능 동결: README · XM_HARNESS 최종, 노드 파라미터 고정, 동결 후보 ckpt 2개(종합 best · 자동 교사 best) 드라이브 업로드 + MANIFEST. 시드 재현 1회(분산) | |
| 11.16–23 | 최종 데이터(+ DAgger + 장애물 녹화)로 최대 2–3회(배치 설정 고정). val best → D 짧은 폐루프 2–3회 → **11.23 동결**: ckpt sha · run_meta · MANIFEST 행 · `results/xm_ft_v1.0/` | 규칙 1→2→3 |
| 11.24–30 | D 60회. E: `eval/xm_batch.sh` 출력 → `eval/analyze_runs.py` 표, 실패 회 분석(cte · 충돌 위치를 `weak_segments` · `rollout_drift` 구간과 대조), 보고서 E-16 | |

## 작은 실험 후보 (재학습 주, 시간 남으면 순서대로)

1. **체크포인트 평균**(model soup): `--save-top-k 3` 으로 받은 에폭 7–9 가중치를 평균 → 오프라인 평가. 학습 비용 0, 평가 1회. OneCycle 끝은 lr 이 작아 세 체크포인트가 가까우므로 보통 조금 좋아진다.
2. **Nav2 라벨 평활**: Nav2 교사는 초당 부호 바뀜 0.29(자동 0.13). ω 라벨에 0.6 s 이동평균을 넣고 학습 — 로봇이 못 따라가는 떨림을 라벨에서 뺌. 효과 불확실(영상 · 상태는 떨린 궤적 그대로).
3. **노드 플래그**: ω EMA · dω/dt 한도(교사 p99 기준) — 폐루프에서 흔들림이 보일 때만, 기본 off.

## 11.6 에 확인한 것

- **회복 데이터는 이미 있음**: `teacher_noise`≠0 이 자동 교사 37 % · Nav2 26 %(|잡음| 중앙값 0.3 rad/s), 교사 |cte| p95 0.6–1.0 m. 폐루프 이탈 한도 2 m 안에서 되돌아오는 예시를 배우고 있으므로 잡음 녹화를 따로 부탁할 필요 없음.
- **적분 이탈(v0.1, 2 s)**: 자동 교사 평균 0.027 m(p95 0.074, 치우침 0) · Nav2 0.168 m(p95 0.479). 제일 큰 구간은 Nav2 시계 방향 s 10–15 · 35–55 m(0.26–0.30 m, s 10–15 는 +0.25 로 한쪽 치우침 = 우회전 덜 꺾어 바깥으로) — `results/xm_ft_v0.1/rollout_drift.md`.
- **추론 속도**: T4 17.6 프레임/s(≈ 57 ms, `results/xm_ft_v0.1/eval_ft_test_summary.json` 옆 run_meta) ≫ 노드 5 Hz. 서버 사전 점검은 `eval_offline.py --max-files 1`(README).
- **Nav2 '덜 꺾기' 원인**은 장애물이 아니라 코너 방식 차이(한계 13, `results/teacher_style_xm_v1_1.md`). 오프라인 Nav2 ω 오차는 성능 기준으로 부적절 → 규칙 1.

## 안 하는 것

- 구조 변경(확산 정책 · 액션 청킹 · 교사 id 입력) — 동결 전에 검증할 시간 없음.
- 영상 증강 — 같은 렌더러. 폐루프에서 영상 민감성이 보이면 재학습 주에 검토.
- v0.1 을 v1.1 test 로 재평가 — `--common` 으로 대신. 새 test 좌회전 행을 v0.1 로도 보고 싶을 때만.
