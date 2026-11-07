# xm_ft v0.2 — Nav2 ×3 복제 파인튜닝 결과 (11.6 학습 · 11.7 정리, Kaggle T4 × 2)

- 체크포인트: `epoch=9-step=18060.ckpt`(best = 마지막 에폭, `validation/losses/action` 0.3719 — v1.1 val) · 0.96 GB · sha12 `0fec1d572199`(내려받은 파일 = Kaggle 평가 run_meta 값) — git 밖(E 보관)
- 데이터 `xm_parking_r3` v1.2(xm_v1_2) = split v1.1 + train 의 `r3_nav2_obs` 14 · `r3_nav2_seg` 5 파일 ×3 복사(`OVERSAMPLE.json`). train 행 38,211 → 57,953 · Nav2 비중 25.8 → 51.1 % · 학습 표본 9,529 → 14,445 · 에폭당 2,382 → 3,611 스텝. 그 밖은 v0.1 과 같음(gin_operative 같음, 시작 ckpt sha12 `339c68f9e027`, 실효 배치 8, 10 에폭) · 7 h 06 min
- 오프라인 test(v1.1, 4,721프레임) 제로샷 → FT: v MAE 0.098 → 0.009 · ω MAE 0.120 → 0.050 · ω 부호 일치 84.2 → 95.7 %. Nav2(콘 반시계 653) ω MAE 0.181 → 0.153 · 좌회전 지시(261, 이 test 에만) 0.259 → 0.198 · 장애물 1.5 m 안(20) 0.149 → 0.209
- **v1.0 test 와 v1.1 test 는 r3_auto 5파일만 같고 Nav2 파일은 하나도 안 겹침**(v1.0 슬라럼 시계 `run_1014_115111_nav2_cw_00` 685 · v1.1 콘 반시계 `run_1014_123928_nav2_ccw_00` 653) → v0.1 과의 `--common` 비교는 자동 교사 4,068프레임뿐. 공통 파일의 교사 값 · ZS 예측은 두 test 에서 같음(차 0)

## 판정 (PLAN 규칙 2, 같은 프레임 `--common`)

| 기준 | v0.1 | v0.2 | 판단 |
| --- | --- | --- | --- |
| ① 자동 교사 ω 기울기 (공통 r3_auto) | 0.86 · 잡음 중 0.86 | 0.85 · 잡음 중 0.83 | 유지 — 구분 불가(시드 분산 미측정) |
| ② Nav2 ω 기울기 | 0.38 (슬라럼 시계, ZS 0.23) | 0.54 (콘 반시계, ZS 0.16) | **판정 불가** — 공통 Nav2 파일 0, 다른 녹화 |
| ③ 2 s 가로 이탈 자동 교사 (공통) | 0.027 · p95 0.074 · 코너 창 0.038 | 0.031 · p95 0.090 · 코너 창 0.048 | 조금 나빠짐 — 5바퀴 모두 같은 쪽(파일별 +0.0011 – +0.0065 m), 치우침 없음(부호 평균 −0.000 둘 다). 탈락 조건인지는 구분 불가(시드 분산 미측정) |
| ③ 2 s 가로 이탈 Nav2 | 0.168 (슬라럼 시계, ZS 0.134) | 0.162 (콘 반시계, ZS 0.115) | 판정 불가 — 다른 녹화. 둘 다 ZS 보다 큼 |
| ④ val 명령 손실 | 0.3224 (v1.0 val) | 0.3719 (v1.1 val) | 비교 불가 — val split 다름 |

- 자동 교사의 다른 지표도 같은 쪽으로 조금 나쁨: ω MAE 0.029 → 0.034 · 부호 일치 99.9 → 99.4 %(같은 4,068프레임) · 2 m 구간 45 중 37 에서 ω MAE 나쁨, +0.01 넘는 9 구간 중 8 이 꺾임 없는 직선(`weak_segments.md` ②).
- 상태: PLAN 11.7 'Nav2 ↑ · 자동 교사 유지 → v0.2 후보' 를 오프라인으로 확인 못 함. 규칙 2 를 글자대로 읽으면 ③ 자동 교사가 나빠져(0.027 → 0.031, 5바퀴 모두) 탈락 쪽이지만 규칙 3(시드 분산)이 없어 확정 못 함 → **판정 보류**. PLAN '안 하는 것'의 'v0.1 을 v1.1 test 로 재평가 — `--common` 으로 대신' 은 Nav2 에서는 성립 안 함(공통 Nav2 0).
- 필요한 것: ① 교차 평가 — v0.1 ckpt(`d86970ce1faf`)로 v1.1 test, v0.2 ckpt(`0fec1d572199`)로 v1.0 test(`eval_offline.py --split test`, T4 1회 약 260 s). 각 Nav2 test 파일이 다른 모델의 val(ckpt 고르기)에 들어 있었으므로 두 방향을 다 봐야 치우침이 상쇄됨 ② 시드 run 1회(③ 자동 교사 0.0268 → 0.0314 m, +0.0045 m 가 시드 차보다 큰지).

## 장애물 1.5 m 안 ω 부호 일치 73.3 → 20.0 % (표 E-15 마지막 행)

- 20프레임 모두 콘 반시계 `run_1014_123928_nav2_ccw_00`(v1.1 test 에만 있음) idx 163–182 — 장애물 하나를 **한 번** 지나는 4 s(s 20.9–23.5 m, 최소 0.93 m @ idx 172). 3 m 안 47프레임도 같은 통과 1회(v1.0 test 슬라럼의 1.5 m 안 40 = 통과 2회 21 · 19).
- 교사 ω 는 작은 좌회전(0.000 – +0.258, 평균 +0.153), 부호 프레임 15. ZS 일치 11 은 거의 안 꺾은 값(|ω 예측| 평균 0.045)의 부호 — 이 파일 ZS Nav2 기울기 0.16.
- v0.2 불일치 12 = 0 근처(|ω| < 0.04) 6 + idx 177–182 우회전 −0.076 → −0.257 6. 이 우회전은 교사의 다음 동작(기준선 복귀 우회전: ω < −0.1 시작 idx 186 · 최소 −0.405 @ 190, cte 0.60 → 0.01 @ 189)을 **1.6 s 일찍(idx 178) · 더 크게(최소 −0.579 @ 189)** 한 것. 이 파일 전체에서 v0.2 예측은 교사보다 1프레임 앞섬(±8 이동 상관 최대 −1, 0.751 vs 0.720) — `steer_diag` 지연은 뒤처짐(0–5)만 봄.
- 같은 통과의 3 m 안 47프레임: 기울기 ZS 0.07 → v0.2 0.82 · 급회전 6 중 부호 반대 ZS 33.3 % → v0.2 0 %(`steer_diag.md` ②).
- 판단: 통과 1회(실질 표본 1)라 장애물 근처가 나빠졌다 · 좋아졌다 둘 다 말할 수 없음. 내용은 반대로 피하기가 아니라 복귀를 일찍 · 세게 꺾은 것 — 폐루프 영향은 시뮬에서 확인.

## ×3 복제와 '덜 꺾기' (PLAN RESEARCH, PilotNet 류 보고)

- 자동 교사(공통 r3_auto): 기울기 0.86 → 0.85 · 급회전(|ω|>0.3) 예측 / 교사 0.32 / 0.37 그대로 · 지시별 기울기 직진 0.872 → 0.855 · 우회전 0.825 → 0.836 · 2 s 코너 창 부호 평균 +0.002 → +0.002 → 덜 꺾기는 늘지 않음, 오차만 치우침 없이 커짐(코너 창 |가로| 0.038 → 0.048). Nav2 비중 25.8 → 51.1 % 인데 자동 교사 코너가 Nav2 쪽(더 세게)으로 옮겨 가지도 않음(한계 13 의 평균 가설로 예상한 방향).
- Nav2(콘 반시계): v0.2 기울기 0.54(지시 직진 0.519 · 좌회전 0.550) · 급회전 0.28 / 0.52 · 2 s 코너 창 부호 평균 −0.124(반시계 = 좌회전을 덜 꺾어 오른쪽으로 밀림, ZS +0.015) → 여전히 절반쯤만 꺾음. 같은 녹화의 v0.1 값이 없어 복제 효과로 나눌 수 없음(위 '필요한 것' ①).

| 파일 | 내용 |
| --- | --- |
| `report_tables.md` | 표 E-14 · E-15 · MANIFEST 행 초안(`e2e/fill_report.py`, `--ckpt` 로 크기 · sha 채움) |
| `curves.md` · `curves.png` | 에폭별 검증 곡선(Kaggle 의 `e2e/plot_training.py` 출력 복사 — 여기서 `csv/metrics.csv` 로 다시 돌려 curves.md 같음 확인) |
| `steer_diag.md` | ① ZS · v0.1 · v0.2 `--common`(r3_auto) ② ZS · v0.2 v1.1 test 전체(`e2e/steer_diag.py`) |
| `rollout_drift.md` | 같은 ① ②(`e2e/rollout_drift.py`) |
| `weak_segments.md` | ① 제로샷 → v0.2 2 m 구간(v1.1 test) ② v0.1 → v0.2 공통 r3_auto(`e2e/weak_segments.py`) |
| `eval_*_summary.json` | `e2e/eval_offline.py` 묶음별 지표(v1.1 test) |
| `run_meta.json` | 학습 기록(`e2e/finetune.py`) — 경로는 Kaggle 기준 |

명령(Git Bash · 레포 루트 · `export PYTHONIOENCODING=utf-8`):

```bash
K=~/Downloads/xm_ft_v0.2_kaggle; ZS=$K/eval_zeroshot_test; V02=$K/xm_ft_v0.2_e10/eval_test
V01=~/Downloads/xm_ft_kaggle_full/xm_ft_kaggle_e10/eval_test; R=e2e/results/xm_ft_v0.2
python e2e/fill_report.py --train $K/xm_ft_v0.2_e10 --zs $ZS --ft $V02 --version xm_ft_v0.2 \
    --ckpt "$K/xm_ft_v0.2_e10/checkpoints/epoch=9-step=18060.ckpt" --out $R/report_tables.md
python e2e/steer_diag.py    ZS=$ZS/frames.csv v0.1=$V01/frames.csv v0.2=$V02/frames.csv --common   # ①
python e2e/steer_diag.py    ZS=$ZS/frames.csv v0.2=$V02/frames.csv                                 # ②
python e2e/rollout_drift.py ZS=$ZS/frames.csv v0.1=$V01/frames.csv v0.2=$V02/frames.csv --common   # ①
python e2e/rollout_drift.py ZS=$ZS/frames.csv v0.2=$V02/frames.csv                                 # ②
python e2e/weak_segments.py --a $ZS --b $V02 -o <임시 폴더> --top 8                                  # ①
# weak_segments ②: v0.1 · v0.2 frames.csv 를 scenario == r3_auto 로 거른 폴더(run_meta.json 같이)로 같은 명령
# 장애물 1.5 m 안 · 파일별 · 이동 상관: ZS · v0.2 frames.csv 를 (scenario, file, idx) 로 합쳐 계산(파일별 기울기 · 이탈은 steer_diag.diag · rollout_drift.drift_file)
```
