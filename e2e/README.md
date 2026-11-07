# e2e/ — E2E 모델

- **담당**: E · 서진석
- **목적**: 교사 데이터로 E2E 주행 모델을 학습하고, 추론 인터페이스를 D 에 넘긴다.

## 간트 작업

| 작업 | 기간 |
| --- | --- |
| E2E 기준선 재현 · 입출력 파이프라인 | 09.22–10.05 |
| 교사 데이터 연출 · 첫 폐루프 · 중간 데모 | 10.05–10.18 |
| 통합 씬 데이터 · Fine-tuning · 모델 통합 | 10.19–11.16 |
| 실패구간 보정 · 1회 재학습 · 모델 동결 | 11.16–11.23 |

## 입력 · 출력

- **받는 것**: D 의 교사 데이터 · [`d2e_teacher.md`](../docs/interfaces/d2e_teacher.md)
- **넘기는 것**: 모델 · forward pass → D · [`docs/interfaces/e2d_model.md`](../docs/interfaces/e2d_model.md)

## 환경

- **모델**: X-Mobility(NVlabs/X-Mobility, 11.2 팀 회의 확정) — 사전학습 ckpt `x_mobility-nav2-semantic_action_path.ckpt`(Hugging Face `nvidia/X-Mobility`, 로그인 · 약관 동의 필요)에서 파인튜닝.
- **학습 환경**: server207, X-Mobility 의존성이 깔린 python(D 의 `~/xmobility_test/venv` 또는 `envs/ml_env`). X-Mobility 레포 고정 버전 `5837aea`(학습 형식 xm_v1 이 맞춘 커밋).
- 데이터 점검(`check_xm_dataset.py`)은 numpy · pandas · pyarrow · pillow 만 있으면 됩니다(torch · GPU 불필요).

> 가중치(`*.pt` `*.ckpt` …)·데이터셋은 커밋하지 않습니다 — [`MANIFEST.md`](../MANIFEST.md) 에 기록.

## 파일

| 파일 | 하는 일 |
| --- | --- |
| `check_xm_dataset.py` | 학습 형식(xm_v1) 점검 — X-Mobility 로더가 가정하는 폴더 구조 · 열 길이 · JPEG 512×320 · NaN · 명령 가운데 4칸 0 · 녹화 묶음 split 누수, split · 시나리오별 v · ω · 경로 지시 분포. 오류면 종료 코드 1 |
| `finetune.py` | 파인튜닝 실행기 — X-Mobility `train.py` 와 같은 루프, 세 가지를 고침(아래) |
| `eval_offline.py` | 오프라인(개루프) 평가 — test split 을 run 파일마다 처음부터 한 프레임씩, 배포 노드(`x_mobility_navigator`)와 같게 상태 · 직전 명령을 이어 가며 추론. 시나리오별 · 경로 지시별 · 장애물 근처 v · ω MAE, ω 부호 일치, 경로 ADE/FDE. 폐루프 주행 성능과는 다름 |
| `weak_segments.py` | 약한 구간 — `eval_offline.py` 의 frames.csv 를 (시나리오 · 방향 · 기준선 s 2 m 구간)으로 묶어 ω MAE 등이 나쁜 구간 순위, 두 평가(제로샷 · FT) 비교 시 나빠진 구간 표시, 기준선 위 색 그림. 재학습용 추가 수집 자리 고르기(11.16–23) |
| `fill_report.py` | 결과 → 보고서 표 — run_meta.json · 평가 summary.json → 표 E-14 · E-15 · MANIFEST checkpoint 행 초안 |
| `plot_training.py` | 학습 곡선 — `<out>/csv/metrics.csv`(finetune.py 가 남김) → 에폭별 검증 표 `curves.md` · 그림 `curves.png`(matplotlib 있을 때). 보고서 표 E-14 |
| `kaggle_finetune.ipynb` | Kaggle 무료 GPU(T4 × 2)용 노트북 — 시험(40배치 1에폭, 스텝 시간 실측) + 제로샷 오프라인 평가 → 본 학습(남은 시간에 맞춰 에폭 수 자동) + 학습 곡선 + 파인튜닝 오프라인 평가. 실험은 `RUN_NAME` · `EXTRA_ARGS` · `SAVE_TOP_K` + Input 데이터셋으로 바꿈. 준비 순서는 노트북 첫 칸 |
| `kaggle_eval.ipynb` | 여러 ckpt 를 같은 test 에서 재평가 + 체크포인트 평균(`avg_ckpt.py`) 평가. 학습 없음 · HF 토큰 필요 없음(CLI push 로 실행). T4 2장에서 2개씩 동시에 |
| `configs/finetune_parking_r3.gin` | 파인튜닝 설정 — X-Mobility `base_train_config.gin` 위에 `ENABLE_SEMANTIC=False` · 에폭 · lr · 배치만 바꿈 |

`finetune.py` 가 원본 `train.py` 와 다른 점:

1. **lr 이 실제로 적용됨** — 원본은 `load_from_checkpoint` 라 ckpt 에 저장된 lr · weight_decay 가 gin 값을 덮어씁니다. 여기서는 gin 값으로 모델을 만들고 가중치만 넣습니다.
2. **가중치 대조** — 원본은 `strict=False` 라 이름이 안 맞는 가중치가 말없이 무작위로 남습니다. 여기서는 missing · unexpected 키를 출력하고, missing 이 있으면 멈춥니다. 의미 분할을 끈 탓에 `model.semantic_decoder.*` 가 unexpected 로 나오는 것은 정상.
3. **기록** — `run_meta.json`(이 레포 · X-Mobility 커밋, ckpt sha256 12자리, 데이터 경로, 명령, 키 대조, best val_loss, test 지표) · `gin_operative.txt`. MANIFEST `checkpoint` 행의 출처 칸에 옮깁니다. gin 의 모르는 이름은 무시하지 않고 멈춥니다.

## 실행 방법

```bash
# 1) 데이터 점검 (받은 zip 을 푼 폴더, 어디서든)
python3 e2e/check_xm_dataset.py data/teacher/xm_v1 --json logs/xm_v1_check.json

# 2) 짧은 시험 — 1에폭 20배치, 키 대조 · 메모리 · 스텝 시간
python e2e/finetune.py --xm-repo ~/xmobility_test/X-Mobility \
    -c e2e/configs/finetune_parking_r3.gin -d data/teacher/xm_v1 \
    -p <x_mobility-nav2-semantic_action_path.ckpt> -o logs/xm_ft_smoke \
    --limit-train-batches 20 --limit-val-batches 2 --no-test --gin NUM_EPOCHS=1

# 3) 본 학습 — 출력 폴더 이름 = MANIFEST checkpoint 버전
CUDA_VISIBLE_DEVICES=1 python e2e/finetune.py --xm-repo ~/xmobility_test/X-Mobility \
    -c e2e/configs/finetune_parking_r3.gin -d data/teacher/xm_v1 \
    -p <x_mobility-nav2-semantic_action_path.ckpt> -o data/checkpoints/xm_ft_v0.1
```

- 설정 덧붙이기: `--gin "NUM_EPOCHS=3"` · `--gin "finetune.freeze_prefixes=['model.observation_encoder.image_encoder.']"`(DINOv2 고정).
- W&B 는 기본 오프라인(출력 폴더 `wandb/`). 온라인은 `--wandb online` + `WANDB_API_KEY`.
- best ckpt 는 `--monitor`(기본 `validation/losses/action` = 명령 손실)로 고릅니다. `val_loss` 에는 경로 손실이 들어가는데, X-Mobility 경로 출력이 Tanh 라 ±1 m 를 못 넘고 우리 path 정답(3.36 s 앞, 0.57 m/s 면 약 1.9 m)은 넘어서 줄일 수 없는 몫이 생깁니다(`action_policy.py:106-115`). D 가 쓰는 출력은 명령뿐.
- Windows 는 데이터 로더가 프로세스마다 데이터셋(약 600 MB)을 복사하므로 RAM 이 빠듯하면 `--gin "XMobilityIsaacSimDataModule.num_workers=2"`.
- 그 밖의 옵션: `--max-time DD:HH:MM:SS`(시간 상한, 닿으면 last.ckpt 저장 후 멈춤 — lr 스케줄이 끝까지 안 내려가므로 에폭 수를 시간에 맞추는 게 우선) · `--save-top-k 1`(디스크 절약) · `--ddp-backend gloo|nccl`(Windows 는 gloo 기본 — X-Mobility 로더가 DistributedSampler 를 써서 GPU 1장이어도 DDP 가 필요).
- `run_meta.json` 에 스텝 시간(`step_time_s`, 데이터 로딩 포함 중앙값) · 에폭당 스텝(`steps_per_epoch`) · GPU 최대 메모리가 남습니다 — 짧은 시험으로 본 학습 시간을 정할 때 씁니다.
- GPU 0 은 Isaac Sim 고정이라 학습은 `CUDA_VISIBLE_DEVICES=1` 을 권합니다([`docs/server207.md`](../docs/server207.md)).
- 같은 출력 폴더에 체크포인트가 있으면 덮어쓰지 않고 멈춥니다.

### 오프라인 평가 (학습 뒤)

```bash
python e2e/eval_offline.py --xm-repo ~/xmobility_test/X-Mobility -c e2e/configs/finetune_parking_r3.gin \
    -d data/teacher/xm_v1 -p data/checkpoints/xm_ft_v0.1/checkpoints/<best>.ckpt -o logs/xm_eval_v0.1
# 비교용: 같은 명령에 -p <사전학습 ckpt> → 제로샷. --teacher-forcing 은 직전 명령을 교사 값으로(학습 때 조건)
# 서버 사전 점검(약 1분): 위 명령에 --max-files 1 → ckpt 로드 · 출력 유한 · run_meta 의 frames_per_s(T4 17.6 ≫ 노드 5 Hz) 확인
```
→ `frames.csv`(프레임별) · `summary.json`(묶음별 지표) · `run_meta.json`. xm_v1 test 는 경로 지시에 좌회전이 없음(val/test 분할 문제, `resplit_xm.py` 참고).

## 검증 상태 (11.7)

- `check_xm_dataset.py` — 실제 xm_v1(47,572장) 오류 0 · 경고 0(로컬 · Kaggle 둘 다).
- `finetune.py` — Kaggle T4 × 2 에서 짧은 시험 통과: 사전학습 ckpt 키 넣은 297 · missing 0 · unexpected 61(모두 예상분) · lr 1e-5, 학습 20스텝 + 검증. 본 학습 10에폭 3회 완료(v0.1 · v0.2 · v0.1_aw30, 결과 `results/`), 11.7 경로 축척 run(v0.1_p3) 진행.
- `eval_offline.py` — 제로샷 test 4,753프레임 완료(T4 1장 19.6 프레임/초).
- `fd_xm_infer` — 입력 전처리(영상 · 경로 20점 · 상한)가 export · eval_offline 과 비트 단위로 같음(ROS 없이 대조). **ROS 위에서는 아직 안 돌림.**

### GPU 메모리별 배치

| GPU | 설정 | 실측 |
| --- | --- | --- |
| T4 15 GB | `--gin XMobilityIsaacSimDataModule.batch_size=2 --accumulate-grad-batches 2` | 배치 4 는 첫 스텝 OOM, 배치 2 최대 10.4 GB · 0.60 s/스텝 · 에폭 약 28분(2장) |
| 24 GB 이상 | 기본(배치 4) | 안 재봄 — 2) 짧은 시험의 `peak_gpu_mem_gb` 로 확인 |

실효 배치(배치 × 누적 × GPU 수)를 같게 두면 lr 그대로 써도 됩니다. `run_meta.json` 에 `batch_size` · `accumulate_grad_batches` · `effective_batch` 가 남습니다.
