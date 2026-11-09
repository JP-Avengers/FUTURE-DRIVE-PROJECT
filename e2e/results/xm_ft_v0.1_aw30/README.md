# xm_ft v0.1_aw30 — 명령 손실 가중 10 → 30 (11.6 학습 · 11.7 분석, Kaggle T4 × 2)

- 체크포인트: `epoch=8-step=10719.ckpt`(best, `validation/losses/action` 0.9244 = 30 × 0.0308) · sha12 `4fdf249cda0a` — **v0.1 best 와 파일 이름이 같지만 다른 모델**(v0.1 = `d86970ce1faf`). 늘 "aw30" 으로 부름. git 밖(E 보관, `~/Downloads/xm_ft_aw30_kaggle/xm_ft_v0.1_aw30_e10/checkpoints/`) · 1,034,137,918 B(0.96 GiB, v0.1 과 같은 크기 — 11.7 내려받아 sha 확인)
- v0.1 과 다른 것은 `--gin XMobilityLoss.action_weight=30`(v0.1 10.0) 하나 — 두 run `gin_operative.txt` 차이도 이 한 줄. 데이터 `xm_parking_r3` v1.0(xm_v1) · 시작 ckpt `339c68f9e027` · 실효 배치 8 · 10 에폭 · lr 1e-5 · seed 미지정 같음. 레포 `e1d3b445b2f0`(v0.1 `e14ef9d85756` — 사이 변경 `--seed` · 평가 경로 축척은 기본값이라 영향 없음, 제로샷 frames.csv 바이트까지 같음) · 4 h 23 min
- 오프라인 test(v1.0, 4,753프레임 — v0.1 과 같은 test) ZS → v0.1 → aw30: ω MAE 0.123 → 0.050 → 0.047 · 자동 교사 0.110 → 0.029 → 0.026 · Nav2 장애물 0.196 → 0.177 → 0.175 · ω 부호 일치 84.8 → 95.8 → 96.0 %

## 규칙 2 (PLAN.md) — 같은 test v1.0 전체

| 항목 | ZS | v0.1 | aw30 | aw30 − v0.1 | 판단 |
| --- | --- | --- | --- | --- | --- |
| ① 자동 교사 ω 기울기 (`steer_diag.md`) | 0.58 | 0.86 | 0.88 | +0.02 | 유지 — 탈락 아님 |
| ② Nav2 ω 기울기 | 0.23 | 0.38 | 0.41 | +0.03 | 구분 불가(시드 분산 미측정) |
| ③ 2 s 가로 이탈 자동 교사 m (`rollout_drift.md`) | 0.084 | 0.027 | 0.024 | −0.003 | 유지 — 탈락 아님 |
| ③ 2 s 가로 이탈 Nav2 m | 0.134 | 0.168 | 0.164 | −0.004 | 구분 불가(시드 분산 미측정) |
| ④ val 명령 손실 ÷ 가중 (v1.0 val, 에폭 8) | — | 0.0322 | 0.0308 | −0.0014 | 구분 불가(시드 분산 미측정) |

→ **탈락 아님**(①·③ 자동 교사 안 나빠짐). ② · ③ Nav2 · ④ 는 모두 v0.1 보다 좋은 쪽이지만 작아서 v0.1 과 같다고 봄(규칙 3).

- **11.8 시드 run**(`../xm_ft_v0.1_seed1/`, v0.1 설정 + `--seed 1`, 같은 v1.0 test): ① 0.87 · ② 0.41 · ③ 자동 0.028 · Nav2 0.167 m · ④ 0.3271(÷ 10 = 0.0327) → 시드 차 ≈ ① ±0.01 · ② ±0.03 · ③ 자동 ±0.001 m · ④ ±0.0005. aw30 의 ① +0.02 · ③ 자동 −0.003 m · ④ −0.0014 는 그 2–3배(이득), ② +0.03 은 시드 차와 같음, ③ Nav2 −0.004 m 는 시드 차(0.001)보다 크나 녹화 1개. 정리: `../seed_noise_11_8.md`.

- Nav2 세부: 급회전 ω 크기 예측 0.26 → 0.28(교사 0.50) · 부호 반대 20.4 → 18.8 % · 장애물 3 m 밖 기울기 0.37 → 0.41 · 잡음 중 0.50 → 0.55. 장애물 3 m 안(98프레임 · 급회전 12) 0.43 → 0.38, 1.5 m 안 ω MAE 0.172 → 0.189(40프레임) — 나빠진 쪽이지만 표본 작고 통과 1회라 구분 불가.
- 덜 꺾기(한계 13)는 그대로. 해석: 명령 가중은 모든 프레임에 똑같이 곱해져 두 교사가 섞인 코너의 L1 최소점이 안 바뀜 — Nav2 기울기가 거의 안 움직인 것과 맞음.
- 2 s 이탈 큰 구간도 v0.1 과 같은 자리(Nav2 cw s 35–40 · 10–15 · 50–55 m, s 10–15 부호 +0.23 = 우회전 덜 꺾기).

## 경로 (명령 : 경로 가중 10 : 5 → 30 : 5)

| 지표 | ZS | v0.1 | aw30 |
| --- | --- | --- | --- |
| test ADE · FDE 전체 m (`eval_*_summary.json`) | 0.354 · 0.942 | 0.306 · 0.903 | 0.305 · 0.900 |
| 자동 교사 | 0.325 · 0.888 | 0.283 · 0.851 | 0.282 · 0.848 |
| Nav2 장애물 | 0.527 · 1.266 | 0.446 · 1.212 | 0.443 · 1.210 |
| val 경로 손실 · 경로 MAE (에폭 8) | — | 6.990 · 0.1958 | 6.993 · 0.1961 |
| test 경로 손실 · 경로 MAE (Lightning 내장) | — | 6.047 · 0.1824 | 6.000 · 0.1807 |

→ 경로 머리 비중이 줄었어도 경로 지표 안 나빠짐(ADE · FDE 차 0.003 m 이하 — 구분 불가).

## 학습 곡선 (`curves_vs_v0.1.md`)

- **기록된 명령 손실은 가중 포함** — 기록값 ÷ 가중 ÷ (v MAE + ω MAE) = 1.007–1.030(두 run · 전 에폭), val_loss = 명령 + 경로 + KL. `curves.md` · `report_tables.md` 의 aw30 명령(best 0.9244)은 ×30 값이라 v0.1(0.3224, ×10)과 바로 비교 불가 → ÷ 가중 0.0308 · 0.0322.
- 경로(×5): val 에폭 8 6.990 · 6.993 같음. 학습 에폭 8 · 9 평균 6.495 · 6.486 → 6.701 · 6.639 — 학습 쪽만 조금 높음.
- KL(×0.001, 두 run 같음): val 에폭 8 0.148 → 0.243(×1.6), 에폭 0 부터 높음(0.152 → 0.226). KL = 사후(관측 포함) · 사전(세계 모델 예측) 분포 차 — 명령 가중만 바꿨으므로 그 영향. 오프라인 지표에서 나빠진 것 없음 · 폐루프 영향 미확인.
- 학습 명령 ÷ 가중 에폭 9: 0.0305 → 0.0258 (val 0.0324 → 0.0310).
- RGB · depth 손실 없음(디코더 끔 — `metrics.csv` 손실 열은 명령 · 경로 · KL 뿐).

## v0.2(v1.1 test)와 한 표 — `--common` (`steer_diag_common.md` · `rollout_drift_common.md`)

공통 파일 = r3_auto 5개(4,068프레임, 모두 cw)뿐. v1.0 test Nav2 `run_1014_115111_nav2_cw_00`(685)와 v1.1 test Nav2 `run_1014_123928_nav2_ccw_00`(653)가 안 겹쳐 **② · ③ Nav2 는 v0.2 와 `--common` 비교 불가** — 네 모델 Nav2 를 한 표에 넣으려면 v0.1 · aw30 을 v1.1 test 로(또는 v0.2 를 v1.0 test 로) 다시 평가해야 함 → 11.7 Kaggle `fd-xm-eval` 로 함(`../reeval_v1.1_11_7/README.md`, aw30 ② 0.53 · ③ Nav2 0.155 m). 제로샷 v1.0 · v1.1 평가는 공통 파일에서 프레임별로 같음(전 열 차 0).

| r3_auto (공통 4,068프레임) | ZS | v0.1 | v0.2 | aw30 |
| --- | --- | --- | --- | --- |
| ① ω 기울기 | 0.58 | 0.86 | 0.85 | 0.88 |
| ω 기울기 · 잡음 중 | 0.53 | 0.86 | 0.83 | 0.88 |
| 급회전 ω 크기 예측 / 교사 | 0.22 / 0.37 | 0.32 / 0.37 | 0.32 / 0.37 | 0.32 / 0.37 |
| ③ 2 s 가로 이탈 평균 · p95 m | 0.084 · 0.201 | 0.027 · 0.074 | 0.031 · 0.090 | 0.024 · 0.064 |
| 2 s 코너 창 가로 평균 m | 0.071 | 0.038 | 0.048 | 0.035 |

## 약한 구간

- 제로샷 → aw30(`weak_segments.md`): 89구간 중 나빠짐 9 · 좋아짐 69 · 그 사이 11(v0.1 은 11 · 67 · 11). 나쁜 구간 상위 8 모두 Nav2 cw.
- v0.1 → aw30(`weak_segments_vs_v0.1.md`): 나빠짐 4 · 좋아짐 7 · 그 사이 78 — 바뀐 11구간 모두 Nav2 cw · 통과 1회. 자동 교사 45구간 Δω MAE −0.012 … +0.006.

| 파일 | 내용 |
| --- | --- |
| `report_tables.md` | 표 E-14 · E-15 · MANIFEST 행 초안(`e2e/fill_report.py`) — best 0.9244 는 ×30 값 |
| `curves.md` · `curves.png` | 에폭별 검증 곡선(`e2e/plot_training.py`, Kaggle 출력) — 명령 열은 ×30 값 |
| `curves_vs_v0.1.md` | v0.1 과 에폭별 비교(명령 ÷ 가중 · 경로 · KL, 검증 · 학습 · test) |
| `steer_diag.md` · `rollout_drift.md` | ZS · v0.1 · aw30 조향 진단 · 적분 이탈(test v1.0 전체) |
| `steer_diag_common.md` · `rollout_drift_common.md` | ZS · v0.1 · v0.2 · aw30 `--common`(r3_auto 5파일만) |
| `weak_segments.md` · `weak_segments_vs_v0.1.md` | 제로샷 → aw30 · v0.1 → aw30 2 m 구간별 약한 곳(`e2e/weak_segments.py`) |
| `eval_*_summary.json` | `e2e/eval_offline.py` 묶음별 지표(Kaggle) — 제로샷은 v0.1 때 것과 같음 |
| `run_meta.json` | 학습 기록(`e2e/finetune.py`) — 경로는 Kaggle 기준 |

## 명령 (11.7, Windows Git Bash, torch 없이 · `PYTHONIOENCODING=utf-8`)

```bash
ZS=~/Downloads/xm_ft_aw30_kaggle/eval_zeroshot_test            # 제로샷 · v1.0 test
V01=~/Downloads/xm_ft_kaggle_full/xm_ft_kaggle_e10/eval_test
AW=~/Downloads/xm_ft_aw30_kaggle/xm_ft_v0.1_aw30_e10/eval_test
V02=~/Downloads/xm_ft_v0.2_kaggle/xm_ft_v0.2_e10/eval_test      # v1.1 test
O=e2e/results/xm_ft_v0.1_aw30
python e2e/steer_diag.py ZS=$ZS/frames.csv v0.1=$V01/frames.csv aw30=$AW/frames.csv -o $O/steer_diag.md
python e2e/rollout_drift.py ZS=$ZS/frames.csv v0.1=$V01/frames.csv aw30=$AW/frames.csv -o $O/rollout_drift.md
python e2e/steer_diag.py ZS=$ZS/frames.csv v0.1=$V01/frames.csv v0.2=$V02/frames.csv aw30=$AW/frames.csv --common -o $O/steer_diag_common.md
python e2e/rollout_drift.py ZS=$ZS/frames.csv v0.1=$V01/frames.csv v0.2=$V02/frames.csv aw30=$AW/frames.csv --common -o $O/rollout_drift_common.md
python e2e/fill_report.py --train ~/Downloads/xm_ft_aw30_kaggle/xm_ft_v0.1_aw30_e10 --zs $ZS --ft $AW --version xm_ft_v0.1_aw30 --out $O/report_tables.md
python e2e/weak_segments.py --a $ZS --b $AW -o <임시> --top 8     # → weak_segments.md
python e2e/weak_segments.py --a $V01 --b $AW -o <임시> --top 8    # → weak_segments_vs_v0.1.md
```

`curves_vs_v0.1.md` 는 두 run 의 `csv/metrics.csv` · `run_meta.json` test 를 pandas 로 나란히 놓은 표(명령 ÷ 10 · 30, 학습 = 에폭 안 기록 행 평균).
