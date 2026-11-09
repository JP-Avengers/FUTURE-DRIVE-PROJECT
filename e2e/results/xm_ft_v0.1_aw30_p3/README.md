# xm_ft v0.1_aw30_p3 — 명령 가중 30 + 경로 정답 × 1/3 (11.8 학습 · 11.9 교차 평가 · 판정, Kaggle T4 × 2)

- 체크포인트: `epoch=8-step=10719.ckpt`(best, `validation/losses/action` 0.9622 = 30 × 0.0321, v1.1 val) · sha12 `114284109957` · 1,034,138,620 B(0.96 GiB) — 늘 "A" 또는 "aw30_p3" 로 부름. 로컬 `~/Downloads/xm_ft_aw30p3_kaggle/xm_ft_v0.1_aw30_p3_e10/checkpoints/`, Kaggle 데이터셋 `fd-xm-ckpts-2` 의 `xm_ft_v0.1_aw30_p3_e8.ckpt`. 파일 이름이 v0.1 · aw30 best 와 같으므로 **sha 로 구분**.
- 설정: aw30(`--gin XMobilityLoss.action_weight=30`) + 데이터 `xm_v1_1_p3`(v1.1 split · train · val · test `path` × 1/3, Kaggle `dataset-06-xm-v1-1-p3`). 10 에폭 · 11,910 스텝 · 실효 배치 8 · lr 1e-5 · seed 미지정 · 사전학습 `339c68f9e027` · 레포 `2bbbdf9` · Kaggle `fd-xm-finetune` v12(CLI, 사전학습 ckpt 는 `fd-xm-pretrained`). train 은 v1.0 · v1.1 이 같은 54파일 · 38,211행.
- 평가 두 벌, ADE · FDE 는 `PATH_SCALE.json` 으로 ÷ 0.3333 → m:
  - v1.1 축척 test(4,721) — 학습 run 자동 평가(`eval_ft_test_*`)
  - v1.0 축척 test(4,753, v0.1 · seed1 · aw30 · p3 와 같은 프레임) — `fd-xm-eval` v3, `dataset-04-xm-v1-p3`(`eval_ft_v1.0test_*`)
- 다른 파일: `curves_vs_aw30.md`(에폭별, val 이 달라 추세만) · `steer_diag_v1.0test.md` · `rollout_drift_v1.0test.md`(일곱 모델) · `steer_diag_v1.1test.md` · `rollout_drift_v1.1test.md` · `weak_segments_vs_aw30.md`(v1.0 test) · `report_tables.md`(제로샷 열 없음 — 축척 데이터 제로샷은 안 돌림)

## 선택 치우침 — 왜 두 test 를 같이 보나

- A 의 best 에폭은 v1.1 val(슬라럼 시계 포함 = v1.0 test 의 Nav2 녹화)로 골랐다. v0.1 · seed1 · aw30 · p3 는 v1.0 val(콘 반시계 포함 = v1.1 test 의 Nav2 녹화)로 골랐다. 그래서 Nav2 행은 v1.0 test 에서 A 가, v1.1 test 에서 나머지가 조금 유리하다.
- 자동 교사 5바퀴는 두 test 가 같은 파일이라 치우침이 없다.
- p3 는 v0.1 · aw30 과 같은 val 로 골랐으므로, v1.1 test 에서 p3 와 v0.1 의 차이는 치우침이 아니다(표 2 아래).
- 시드 바닥 = |seed1 − v0.1|(`../seed_noise_11_8.md`). 콘 반시계 녹화의 바닥은 11.9 에 seed1 을 v1.1 test 로 평가해 처음 쟀다(`fd-xm-eval` v5).

## 표 1 — 자동 교사(공통 4,068프레임, 치우침 없음)

| 지표 | v0.1 | seed1 | aw30 | p3 | **A** | 시드 바닥 |
| --- | --- | --- | --- | --- | --- | --- |
| ① ω 기울기 | 0.86 | 0.87 | 0.88 | 0.90 | **0.91** | 0.006 |
| ③ 2 s 가로 이탈 평균 (m) | 0.027 | 0.028 | 0.024 | 0.026 | **0.024** | 0.0016 |
| ω MAE (rad/s) | 0.0290 | 0.0311 | 0.0259 | 0.0281 | **0.0260** | 0.0021 |

## 표 2 — Nav2(녹화가 test 마다 다름)

| v1.0 test 슬라럼 시계 (685, A 유리) | v0.1 | seed1 | aw30 | p3 | **A** | 시드 바닥 |
| --- | --- | --- | --- | --- | --- | --- |
| ② ω 기울기 | 0.38 | 0.41 | 0.41 | 0.41 | **0.47** | 0.02 |
| ③ 2 s 이탈 (m) | 0.168 | 0.167 | 0.164 | 0.167 | **0.160** | 0.001 |
| ω MAE | 0.1768 | 0.1806 | 0.1749 | 0.1760 | 0.1749 | 0.0038 |
| ω 부호 일치 | 76.6 % | 76.6 % | 77.4 % | 76.4 % | 79.8 % | 0 |

| v1.1 test 콘 반시계 (653, v0.1 · seed1 · aw30 · p3 유리) | v0.1 | seed1 | aw30 | p3 | **A** | 시드 바닥 |
| --- | --- | --- | --- | --- | --- | --- |
| ② ω 기울기 | 0.49 | 0.49 | 0.53 | 0.53 | **0.57** | 0.00 |
| ③ 2 s 이탈 (m) | 0.158 | 0.162 | 0.155 | 0.175 | 0.171 | 0.004 |
| ω MAE | 0.1501 | 0.1539 | 0.1474 | 0.1619 | 0.1584 | 0.0038 |
| 좌회전 지시 ω MAE (261) | 0.208 | 0.218 | 0.204 | 0.226 | 0.213 | 0.010 |

- **꺾는 크기**: Nav2 기울기는 두 녹화 모두 A 가 가장 크다(0.47 · 0.57, aw30 0.41 · 0.53). 치우침이 반대인 콘 반시계에서도 높으므로 실제 차이다.
- **콘 반시계에서 경로 축척의 손해**: p3 가 v0.1 보다 이탈 +0.017 m(바닥의 4배) · ω MAE +0.012(3배) 나쁘다. p3 와 v0.1 은 같은 val 로 골랐으니 치우침이 아니다. A 의 aw30 대비 +0.016 m · +0.011 도 같은 크기라, 치우침보다 경로 축척 효과로 본다. 슬라럼 시계에서는 p3 가 v0.1 과 같다(0.167 · 0.168). 원인은 미확인.
- 장애물 3 m 안 기울기(98프레임 · 급회전 12)는 시드만으로 0.09 흔들려 판정에 안 쓴다(v0.1 0.43 · aw30 0.38 · p3 0.31 · A 0.35).

## 표 3 — 경로

| ADE · FDE (m) | v0.1 | aw30 | p3 | **A** |
| --- | --- | --- | --- | --- |
| v1.0 test | 0.306 · 0.903 | 0.305 · 0.900 | 0.102 · 0.229 | 0.108 · 0.242 |
| v1.1 test | 0.305 · 0.904 | 0.304 · 0.903 | 0.098 · 0.219 | 0.102 · 0.229 |

경로 축척 두 모델이 ÷ 3 으로 같다. 시드 바닥은 ADE 0.001 · FDE 0.003 m.

## 판정 (PLAN 규칙 2 · 3)

- **A = 종합 1위 후보.** 자동 교사(① 0.91, v0.1 대비 바닥의 8배 · ③ 0.024 = aw30), Nav2 꺾는 크기(두 녹화), 경로(÷ 3)가 가장 좋다. 규칙 2 의 탈락 조건(자동 교사 ① · ③ 악화)에 걸리지 않는다.
- **aw30 = 백업.** 콘 반시계(왼쪽으로 도는 장애물 녹화, test 에 하나)에서 경로 축척의 이탈 손해가 없고, 노드에 `path_scale` 이 필요 없다.
- 둘 다 11.14–16 동결 후보로 Drive 에 올리고, 폐루프(특히 반시계 장애물 시나리오)로 고른다.
- A 를 노드로 돌릴 때는 `path_scale:=0.3333` 을 준다(`~/path` 디버그 발행만 m 로 바뀌고 v · ω 명령은 무관 — `ros2_ws/src/fd_xm_infer/README.md`).
- 바닥은 시드 한 쌍이라 거칠다. 콘 반시계 바닥은 seed1 의 v1.1 평가 하나로 쟀다.

## 명령

```bash
# Windows Git Bash, torch 없이, PYTHONIOENCODING=utf-8, 레포 e2e/ 에서. D=~/Downloads
A10=$D/xm_eval_kaggle_v3_A_v10p3/eval_aw30_p3_test          # v1.0 축척 test
A11=$D/xm_ft_aw30p3_kaggle/xm_ft_v0.1_aw30_p3_e10/eval_test   # v1.1 축척 test
S11=$D/xm_eval_kaggle_v5_seed1_v11/eval_seed1_test           # seed1 v1.1 test(콘 반시계 바닥)
P11=$D/xm_eval_kaggle_v4_p3_v11/eval_p3_test                 # p3 v1.1 축척 test
python steer_diag.py    ZS=... v0.1=... seed1=... aw30=... v0.2=... p3=... aw30_p3=$A10/frames.csv -o results/reeval_v1.1_11_7/steer_diag_v1.0test_7.md
python rollout_drift.py (같은 입력) -o results/reeval_v1.1_11_7/rollout_drift_v1.0test_7.md
python steer_diag.py    ZS=... v0.1=... aw30=... v0.2=... p3=$P11/frames.csv aw30_p3=$A11/frames.csv -o results/reeval_v1.1_11_7/steer_diag_v1.1test_6.md
python rollout_drift.py (같은 입력) -o results/reeval_v1.1_11_7/rollout_drift_v1.1test_6.md
```
