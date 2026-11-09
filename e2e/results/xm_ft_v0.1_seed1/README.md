# xm_ft v0.1_seed1 — v0.1 설정 그대로 + `--seed 1` (11.7 학습 · 11.8 분석, Kaggle T4 × 2)

- **목적**: PLAN 규칙 3 의 기준선 — 같은 설정 두 run 이 얼마나 다른지 재서, aw30 · p3 · v0.2 의 작은 차이가 실제인지 가른다. 판정은 **`../seed_noise_11_8.md`**. 이 폴더는 run 자체의 기록.
- 체크포인트: `epoch=9-step=11910.ckpt`(best, `validation/losses/action` 0.3271 = 10 × 0.0327) · sha12 `78b34c3d7644`(평가 run_meta) — v0.1 best 는 에폭 8(`epoch=8-step=10719.ckpt`, `d86970ce1faf`). git 밖(E 보관) · 크기 미확인(ckpt 안 받음, `--save-top-k 3` 이라 Kaggle 출력엔 에폭 7–9 셋 있음)
- v0.1 과 다른 것은 **`--seed 1`**(`pl.seed_everything(1, workers=True)` — 초기화 · 셔플 · 드롭아웃) 하나. `gin_operative.txt` 학습 · 평가 둘 다 v0.1 과 byte 같음(diff 없음). 데이터 xm_v1 · 시작 ckpt `339c68f9e027` · 실효 배치 8 · 10 에폭 · lr 1e-5 · 명령 가중 10 같음. v0.1 은 seed 미지정(run_meta `seed: null`)이라 '시드 0 vs 1' 이 아니라 '정하지 않은 난수 vs 시드 1' — 어느 쪽이든 같은 설정의 독립 2회. `--save-top-k 3`(v0.1 1)은 저장만 달라짐. 레포 `7effa01f6104`(v0.1 `e14ef9d85756` — 사이 변경은 `--seed` UnboundLocalError 수정 · 평가 경로 축척(기본값) · 진단 묶음이라 학습 · 평가에 영향 없음, 제로샷 frames.csv 가 aw30 커널 것과 byte 같음). 2026-11-07 06:58 → 11:44 = **4 h 45 min**(v0.1 4 h 39 min), 스텝 0.687 s(v0.1 0.655), GPU 최대 10.38 GB 같음.
- **Kaggle 커널 이름은 `fd-xm-finetune-aw30`**(aw30 때 커널을 그대로 씀) — 그래서 로그가 `~/Downloads/xm_ft_seed1_kaggle/fd-xm-finetune-aw30.log`. 설정은 run_meta `argv` 대로 seed1(action_weight 기본 10, 로그에 `action_weight` · aw30 없음). 로그는 마지막 803줄(에폭 9 검증 → test → 그림 → 평가 → fill_report)만 남아 'Seed set to 1' 줄은 없고 seed=1 은 `run_meta.json` 에 기록. 본 학습 앞 smoke(1 에폭, 06:52–06:53)도 `--seed 1`.
- 오프라인 test(v1.0, 4,753프레임 — v0.1 과 같은 test) ZS → v0.1 → seed1: ω MAE 0.123 → 0.050 → 0.053 · 자동 교사 0.110 → 0.029 → 0.031 · Nav2 장애물 0.196 → 0.177 → 0.181 · ω 부호 일치 84.8 → 95.8 → 95.8 %(일치 프레임 수까지 같음, 2,508/2,618). → seed1 은 v0.1 보다 **MAE · 이탈은 조금 나쁜 쪽**(ω MAE +0.002 · 자동 교사 2 s 이탈 +0.0016 m), **기울기는 조금 좋은 쪽**(0.86 → 0.87 · Nav2 0.38 → 0.41) — 이 크기가 '시드 차'.

## 규칙 2 (PLAN.md) — 같은 test v1.0 전체 · seed1 − v0.1 = 시드 바닥

| 항목 | ZS | v0.1 | seed1 | seed1 − v0.1 (바닥) | 판단 |
| --- | --- | --- | --- | --- | --- |
| ① 자동 교사 ω 기울기 (`steer_diag.md`) | 0.58 | 0.862 | 0.868 | +0.006 | 유지(같은 설정이니 당연) |
| ② Nav2 ω 기울기 | 0.23 | 0.385 | 0.405 | +0.021 | 바닥이 큼 — aw30 · p3 의 +0.02–0.03 을 못 가름 |
| ③ 2 s 가로 이탈 자동 교사 m (`rollout_drift.md`) | 0.084 | 0.0268 | 0.0284 | +0.0016 | 5바퀴 모두 + (0.0007–0.0028) |
| ③ 2 s 가로 이탈 Nav2 m | 0.134 | 0.1676 | 0.1666 | −0.0010 | 녹화 1개 |
| ④ val 명령 손실 ÷ 10 (v1.0 val, best) | — | 0.03224 (에폭 8) | 0.03271 (에폭 9) | +0.00047 | 에폭 7–9 흔들림과 같은 크기 |

→ seed1 은 규칙 2 로는 '탈락 아님 · v0.1 과 같음'(당연). 위 바닥 × 2 를 '구분 가능' 문턱으로 쓴 결과(`../seed_noise_11_8.md`): **aw30 은 ① · ③ Nav2 · ④ 가 바닥의 3–4 배 → v0.1 보다 좋음**(③ 자동 1.96× 문턱 바로 아래 · 5바퀴 일관), **p3 는 ① 7× · ④ 2.7× · 경로 확실, ③ 은 같음**, **v0.2 는 ① · ③ 자동 교사 나빠짐이 3× → 탈락 확정**.

- Nav2 세부(`steer_diag.md`): 급회전 ω 크기 예측 0.26 → 0.28(교사 0.50) · 부호 반대 20.4 → 19.4 % · **장애물 3 m 안 기울기 0.43 → 0.34**(98프레임 · 급회전 12 — 시드만으로 0.09 움직임 → 이 묶음은 판정에 못 씀, 바닥이 어떤 run 차보다 큼) · 3 m 밖 0.37 → 0.40 · 잡음 중 0.50 → 0.50 · 1.5 m 안 ω MAE 0.172 → 0.182(40프레임).
- 2 s 이탈 큰 구간(`rollout_drift.md`)은 v0.1 · aw30 과 같은 자리(Nav2 cw s 35–40 0.306 · 10–15 0.293 · 50–55 0.267 m, s 10–15 부호 +0.18 = 우회전 덜 꺾기). 덜 꺾기(한계 13)는 시드와 무관하게 그대로.

## 경로 (명령 : 경로 가중 10 : 5, 두 run 같음)

| 지표 | ZS | v0.1 | seed1 | seed1 − v0.1 |
| --- | --- | --- | --- | --- |
| test ADE · FDE 전체 m (`eval_*_summary.json`) | 0.354 · 0.942 | 0.3064 · 0.9028 | 0.3053 · 0.9005 | −0.0011 · −0.0023 |
| 자동 교사 | 0.325 · 0.888 | 0.2829 · 0.8507 | 0.2828 · 0.8497 | −0.0002 · −0.0010 |
| Nav2 장애물 | 0.527 · 1.266 | 0.4460 · 1.2120 | 0.4392 · 1.2020 | −0.0069 · −0.0100 |
| val 경로 손실 · 경로 MAE (best 에폭) | — | 6.990 · 0.1958 (8) | 7.007 · 0.1964 (9) | +0.016 · +0.0006 |
| test 경로 손실 · 경로 MAE (Lightning 내장) | — | 6.047 · 0.1824 | 6.029 · 0.1819 | −0.018 · −0.0005 |

→ 경로 지표의 시드 차는 0.001–0.002 m(Nav2 0.007–0.010 m). aw30 의 −0.0016 · −0.0027 m 는 그 안(구분 불가), p3 의 0.306 → 0.102 m 는 축척 효과라 비교 자체가 다름.

## 학습 곡선 (`curves_vs_v0.1.md`)

- 명령 ÷ 10(val): 에폭 0–9 차 seed1 − v0.1 = −0.008 … +0.006, **에폭 5 이후 |차| ≤ 0.0008**. best v0.1 에폭 8 0.03224 · seed1 에폭 9 0.03271(seed1 에폭 8 은 0.03300). 에폭 7–9 평균 0.0327 · 0.0330.
- **best 에폭 8 → 9 는 시드 효과** — 두 run 모두 에폭 8 · 9 의 명령 손실 차가 0.0003 이하라 어느 쪽이 best 가 되든 우연. aw30(8) · p3(9) · v0.2(9) 의 best 에폭 차이도 같은 이유로 뜻 없음.
- 경로(×5) · KL(×0.001): 최소 에폭 4 둘 다, 값 차 ≤ 0.03 · 0.009. val_loss 최소 에폭 7 둘 다. 기록된 명령 손실은 가중 포함(기록값 ÷ 10 ÷ (v MAE + ω MAE) = 1.008–1.030, 두 run 같음).
- 학습 명령 ÷ 10 에폭 9: 0.0305 · 0.0298. test(Lightning, 각자 best): 명령 ÷ 10 0.0319 · 0.0320, 경로 6.047 · 6.029, KL 0.148 · 0.151.
- RGB · depth 손실 없음(디코더 끔 — `metrics.csv` 손실 열은 명령 · 경로 · KL 뿐).

## 약한 구간

- 제로샷 → seed1(`weak_segments.md`): 89구간 중 나빠짐 13 · 좋아짐 65 · 그 사이 11(v0.1 11 · 67 · 11, aw30 9 · 69 · 11). 나쁜 구간 상위 8 모두 Nav2 cw — v0.1 과 같은 자리(1 · 2위가 s 40–42 ↔ 50–52 로 바뀜, 0.522 · 0.512).
- v0.1 → seed1(`weak_segments_vs_v0.1.md`): 나빠짐 5 · 좋아짐 2 · 그 사이 82 — 바뀐 7구간 모두 Nav2 cw · 통과 1회, Δω MAE 최대 +0.037(s 38–40). 자동 교사 45구간 Δω MAE −0.004 … +0.013, Nav2 45구간 −0.022 … +0.037 — **이것이 2 m 구간 단위의 시드 차**(aw30 의 v0.1 대비 바뀐 11구간 · 자동 교사 −0.012 … +0.006 은 이 안).

| 파일 | 내용 |
| --- | --- |
| `report_tables.md` | 표 E-14 · E-15 · MANIFEST 행 초안(`e2e/fill_report.py`) — sha12 는 평가 run_meta 값, 크기는 자리 표시(ckpt 안 받음) |
| `curves.md` · `curves.png` | 에폭별 검증 곡선(`e2e/plot_training.py`, Kaggle 출력) |
| `curves_vs_v0.1.md` | v0.1 과 에폭별 비교(명령 ÷ 10 · 경로 · KL, 검증 · 학습 · test) + 지표별 최소 에폭 |
| `steer_diag.md` · `rollout_drift.md` | ZS · v0.1 · seed1 조향 진단 · 적분 이탈(test v1.0 전체). 여섯 모델 한 표는 `../reeval_v1.1_11_7/*_v1.0test_6.md` |
| `weak_segments.md` · `weak_segments_vs_v0.1.md` | 제로샷 → seed1 · v0.1 → seed1 2 m 구간별 약한 곳(`e2e/weak_segments.py`) |
| `eval_*_summary.json` · `eval_ft_test_run_meta.json` | `e2e/eval_offline.py` 묶음별 지표(Kaggle) — 제로샷은 v0.1 · aw30 때 것과 같음, 평가 run_meta 에 ckpt sha12 |
| `run_meta.json` | 학습 기록(`e2e/finetune.py`, `seed: 1`) — 경로는 Kaggle 기준 |

## 명령 (11.8, Windows Git Bash, torch 없이 · `PYTHONIOENCODING=utf-8`)

```bash
ZS=~/Downloads/xm_ft_aw30_kaggle/eval_zeroshot_test            # 제로샷 · v1.0 test (seed1 커널의 eval_zeroshot_test 와 byte 같음)
V01=~/Downloads/xm_ft_kaggle_full/xm_ft_kaggle_e10/eval_test
S1R=~/Downloads/xm_ft_seed1_kaggle/xm_ft_v0.1_seed1_e10; S1=$S1R/eval_test
O=e2e/results/xm_ft_v0.1_seed1
python e2e/steer_diag.py ZS=$ZS/frames.csv v0.1=$V01/frames.csv seed1=$S1/frames.csv -o $O/steer_diag.md
python e2e/rollout_drift.py ZS=$ZS/frames.csv v0.1=$V01/frames.csv seed1=$S1/frames.csv -o $O/rollout_drift.md
python e2e/fill_report.py --train $S1R --zs $ZS --ft $S1 --ckpt "$S1R/checkpoints/epoch=9-step=11910.ckpt" --version xm_ft_v0.1_seed1 --out $O/report_tables.md
#   ckpt 파일이 없어 【sha12】 → 78b34c3d7644(평가 run_meta) 로 바꿈, 【크기】 는 그대로
python e2e/weak_segments.py --a $ZS --b $S1 -o <임시> --top 8     # → weak_segments.md
python e2e/weak_segments.py --a $V01 --b $S1 -o <임시> --top 8    # → weak_segments_vs_v0.1.md
```

`curves_vs_v0.1.md` 는 두 run 의 `csv/metrics.csv`(검증 = 에폭 마지막 행 · 학습 = 에폭 안 기록 행 평균) · `run_meta.json` test 를 pandas 로 나란히 놓은 표.
