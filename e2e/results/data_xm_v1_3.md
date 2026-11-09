# 데이터 xm_v1_3 = xm_v1.1 + train Nav2 ×2 — '같은 옵티마이저 스텝' 실험용 (11.8, E)

- **무엇**: `~/Downloads/xm_v1_3` = xm_v1.1(split v1.1, `RESPLIT.json`)에서 train 의 `r3_nav2_obs` 14 · `r3_nav2_seg` 5 파일을 1벌 더 복사(×2, `OVERSAMPLE.json` factor 2 · 복사본 19개 `run_<묶음>_50.pqt`). val · test · `index/` 는 v1.1 그대로. 업로드용 `~/Downloads/05_xm_v1_3.zip`(top `xm_v1_3/` 119 항목, `03_xm_v1_2.zip` 과 같은 배치: 무압축 · 폴더 항목 없음) — 971,795,543 B(926.8 MiB), sha256 앞 12 `a7ce38e1d947`.
- **왜**: v0.2(= v1.1 + Nav2 ×3, 10에폭)는 Nav2 기울기를 올렸지만(콘 반시계 0.49 → 0.54 · 슬라럼 시계 0.38 → 0.43) 자동 교사 2 s 이탈이 0.027 → 0.031 m 로 나빠져 탈락(`compare_11_7.md` ⓪). 그런데 v0.2 는 **Nav2 비중**(25.8 → 51.1 %)과 **학습 길이**(옵티마이저 11,910 → 18,060 스텝, +52 %, OneCycle 이라 lr 일정도 그만큼 길어짐)를 같이 바꿨다 — 자동 교사를 망친 쪽이 어느 것인지 모른다. v1_3 은 Nav2 ×2 로 비중만 올리고(25.8 → 41.1 %) **`NUM_EPOCHS=8` 로 스텝 수를 v0.1 과 맞춰**(11,992 vs 11,910) 'Nav2 강조' 효과만 본다. 그 밖은 v0.1 설정(명령 가중 10, 배치 2 × 누적 2 × T4 2 = 실효 8), 평가는 같은 v1.1 test(자동 교사 5바퀴 4,068 + 콘 반시계 653)라 `reeval_v1.1_11_7/README.md` 표와 바로 견줌.
- **읽는 법(규칙 2)**: ③ 자동 교사 이탈이 v0.1 0.027 m 근처(시드 run 분산 안)에 머물고 ② Nav2 기울기가 0.49 보다 오르면 → '긴 학습'이 범인, Nav2 비중은 쓸 만함(다음: aw30 + ×2). 자동 교사가 v0.2 처럼 0.031 m 쪽으로 가면 → 비중 자체가 원인(v0.3 교사 역할 나누기로). seed1 결과 없이는 ±0.004 m 를 판정 못 하므로 seed1 과 같이 읽는다.

## 수 (`e2e/check_xm_dataset.py --image-stride 20`: v1.1 · v1.3 둘 다 오류 0 · 경고 0)

| split | 시나리오 | v1.1 파일 · 행 | v1.3 파일 · 행 |
| --- | --- | --- | --- |
| train | r3_auto | 35 · 28,340 | 35 · 28,340 |
| train | r3_nav2_obs | 14 · 9,052 | 28 · 18,104 |
| train | r3_nav2_seg | 5 · 819 | 10 · 1,638 |
| train 합 | | 54 · 38,211 · 시퀀스(길이 4) 9,529 · 버림 95 | 73 · 48,082 · 시퀀스 11,987 · 버림 134 |
| val | r3_auto + r3_nav2_obs | 5 · 3,955 + 1 · 685 = 6 · 4,640 · 시퀀스 1,157 | 같음 |
| test | r3_auto + r3_nav2_obs | 5 · 4,068 + 1 · 653 = 6 · 4,721 · 시퀀스 1,179 | 같음 |

- Nav2 train 비중 9,871 / 38,211 = 25.8 % → 19,742 / 48,082 = 41.1 %(v1.2 ×3 은 51.1 %). 점검기 분포(train): v 평균 0.60 → 0.62 · |ω| 평균 0.154 → 0.164 · 장애물 1.5 m 안 337 → 674 행.
- `~/Downloads/xm_v1_3` 은 11.7 에 같은 명령으로 이미 만들어져 있었다(README 머리말만 손으로 더한 상태). 11.8 에 스크립트를 스크래치에 다시 돌려 sha256 으로 견줌 — 119 파일 중 118 바이트 같음, 다른 건 README.md 뿐 → 그대로 쓰고 README 에 에폭 8 줄만 더함(7,341 B). 복사본 19개는 원본과 바이트 같음.

## 에폭 수 — 옵티마이저 스텝을 v0.1 과 맞춤

`run_meta.json` 두 개(v0.1 `~/Downloads/xm_ft_kaggle_full/xm_ft_kaggle_e10/`, v0.2 `~/Downloads/xm_ft_v0.2_kaggle/xm_ft_v0.2_e10/`)와 `finetune.py` 의 식 — `per_rank = ceil(시퀀스 / world_size)`, `steps_per_epoch = per_rank // batch_size`(drop_last), 옵티마이저 스텝/에폭 = `ceil(steps_per_epoch / 누적)`(마지막 부분 누적도 1스텝 — v0.2 3,611 / 2 = 1,805.5 → 1,806 × 10 = 18,060 이 run_meta 와 같음).

| | train 시퀀스 (행) | 시퀀스/행 | T4×2 · 배치 2 · 누적 2 | 에폭 | 총 옵티마이저 스텝 |
| --- | --- | --- | --- | --- | --- |
| v0.1 (xm_v1) | 9,529 (38,211) | 0.2494 | ceil(9,529/2) = 4,765 → //2 = 2,382 → ceil(2,382/2) = 1,191 | 10 | 11,910 (run_meta `global_step` 같음) |
| v0.2 (xm_v1_2, ×3) | 14,445 (57,953) | 0.2493 | 7,223 → 3,611 → 1,806 | 10 | 18,060 (같음) |
| **v1_3 (×2)** | **11,987** (48,082) | 0.2493 | 5,994 → 2,997 → **1,499** | **8** | **11,992** (+82, +0.7 %) |

- 예측: Nav2 1벌 = (14,445 − 9,529) / 2 = 2,458 시퀀스(9,871 행 중 4 × 2,458 = 9,832 쓰고 39 버림) → 9,529 + 2,458 = 11,987 — 점검기 값과 같음. 행도 38,211 + 9,871 = 48,082 · 버림 95 + 39 = 134 로 같음.
- 에폭별 총 스텝: 7 → 10,493(−1,417) · **8 → 11,992(+82)** · 9 → 13,491(+1,581) · 10 → 14,990 → **`--gin NUM_EPOCHS=8`**(finetune.py 직접 실행) / Kaggle 노트북에서는 `MAX_EPOCHS = 8`(EXTRA_ARGS 의 NUM_EPOCHS 는 cell 8 이 덮어씀). OneCycle 은 총 스텝으로 일정을 잡으므로 lr 곡선 길이도 v0.1 과 같아진다.
- 자취방 RTX 5060 Ti(GPU 1 · 배치 2 · 누적 4, 실효 8): per_rank 11,987 → steps_per_epoch 5,993 → ceil(5,993/4) = 1,499 → 같은 8에폭 11,992.
- Kaggle 노트북 `kaggle_finetune.ipynb` 은 에폭을 min(`MAX_EPOCHS`, floor((`TIME_BUDGET_H` 10 h × 3600 − 1,200) / per_epoch_s)) 로 정하고 per_epoch_s = steps_per_epoch × step_time_s × 1.15 = 2,997 × 0.687 × 1.15 ≈ 2,368 s(v0.2 속도; v0.1 속도 0.655 면 2,257 s) → 시간으로는 14–15 에폭이라 기본 `MAX_EPOCHS = 10` 이면 10에폭이 돈다. **`MAX_EPOCHS = 8` 로 바꿀 것.** `EXTRA_ARGS` 에 NUM_EPOCHS 를 넣지 말 것(노트북이 그 뒤에 `--gin NUM_EPOCHS={epochs}` 를 또 붙임). 출력 폴더 `<RUN_NAME>_e8`.
- 시간(Kaggle T4×2): 배치 스텝 2,997 × 8 = 23,976 × 0.703 s(v0.1 wall 4 h 39 min ÷ 23,820 배치 스텝, 검증 · test 포함) ≈ 4 h 40 min(v0.2 속도 0.709 s 면 4 h 43 min) — `--max-time 00:10:00:00` 안.

## 명령 (11.8, Windows Git Bash, 레포 7effa01, torch 없이, `PYTHONIOENCODING=utf-8`)

```bash
python e2e/oversample_xm.py -s r3_nav2_obs -s r3_nav2_seg -n 2 ~/Downloads/xm_v1_1 ~/Downloads/xm_v1_3   # 복사본 19개 (대상이 있으면 거부)
python e2e/check_xm_dataset.py ~/Downloads/xm_v1_3 --image-stride 20   # 오류 0 · 경고 0 · 위 표
python e2e/check_xm_dataset.py ~/Downloads/xm_v1_1 --image-stride 20   # 비교용
# zip — 03_xm_v1_2.zip 과 같은 배치(python zipfile, ZIP_STORED, 폴더 항목 없이 os.walk 순서, top xm_v1_3/)
cd ~/Downloads && python -X utf8 -c "
import zipfile, os
z = zipfile.ZipFile('05_xm_v1_3.zip', 'w', zipfile.ZIP_STORED)
for d, _, fs in os.walk('xm_v1_3'):
    for f in fs:
        p = os.path.join(d, f); z.write(p, 'xm_v1_3/' + os.path.relpath(p, 'xm_v1_3').replace(os.sep, '/'))
z.close()"
sha256sum ~/Downloads/05_xm_v1_3.zip | cut -c1-12   # a7ce38e1d947 · 971,795,543 B
```

Kaggle 노트북으로: `RUN_NAME` 새 이름 · `MAX_EPOCHS = 8` · `EXTRA_ARGS = ""` · `SAVE_TOP_K = 3` · `BATCH, ACCUM = 2, 2` 그대로 · Input = `05_xm_v1_3.zip` 을 올린 데이터셋(노트북이 train/val/test 폴더를 glob 으로 찾음). CLI 로 직접(v0.1 run_meta argv 에서 데이터 · 에폭 · save-top-k 만 바꿈):

```bash
python e2e/finetune.py --xm-repo /tmp/X-Mobility -c e2e/configs/finetune_parking_r3.gin -d <입력>/xm_v1_3 -p /tmp/ckpt/x_mobility-nav2-semantic_action_path.ckpt --wandb offline --save-top-k 3 --gin XMobilityIsaacSimDataModule.batch_size=2 --accumulate-grad-batches 2 --gin NUM_EPOCHS=8 --max-time 00:10:00:00 -o /kaggle/working/<RUN_NAME>
# 자취방(SETUP_LOCAL.md): --accumulate-grad-batches 4 로 바꾸고 나머지 같음 → 8에폭 11,992 스텝
```
