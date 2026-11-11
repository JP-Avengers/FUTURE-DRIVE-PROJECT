# 로컬 학습 준비 — Windows · RTX 5060 Ti 16 GB

`e2e/setup_windows.ps1` 한 번으로 환경 설치 → GPU 확인 → 사전학습 ckpt → 데이터 점검 → 40배치 시험 → 본 학습 명령 출력까지 갑니다. 흐름은 `kaggle_finetune.ipynb` 와 같습니다(시험으로 스텝 시간을 재고, 시간 예산에 맞춰 에폭 수를 정함).

## 집 가기 전(노트북에서) · 도착 후 순서

**가기 전 — 챙길 것(이 노트북 → 집 PC 같은 경로)**

1. **사전학습 ckpt** — `Downloads\xm_pretrained_from_seed_try2\pretrained\x_mobility-nav2-semantic_action_path.ckpt`(1.44 GB · 1,444,751,035 B · sha12 `339c68f9e027` = v0.1 · aw30 run_meta 의 checkpoint_sha12). `-Ckpt` 로 주면 7단계가 HF 다운로드를 건너뜀.
2. **데이터** — `Downloads\02_xm_v1.zip`(805 MB, 안에 `xm_v1\`; v0.1 · aw30 · seed1 과 같은 v1.0 split). 또는 집에서 `kaggle datasets download henryseo711/dataset-02-xm-v1`. E3 용으로 `Downloads\03_xm_v1_1_p3.zip`(805 MB, 안에 `xm_v1_1_p3\`)도 — 또는 `kaggle datasets download henryseo711/dataset-06-xm-v1-1-p3`. xm_v1_2(`03_xm_v1_2.zip`)는 v0.2 탈락(11.7, `results/compare_11_7.md`)이라 안 가져감.
3. **Kaggle 토큰**(결과 업로드에 필수) — `$HOME\.kaggle\access_token` · `kaggle.json` 파일을 USB · 드라이브로 옮겨 집 PC 의 같은 경로에(데이터 · 결과 내려받기용). 내용은 채팅에 붙이지 말 것.
4. **비교용 ckpt(선택)** — aw30 best `Downloads\xm_ft_aw30_kaggle\xm_ft_v0.1_aw30_e10\checkpoints\epoch=8-step=10719.ckpt`(0.96 GB, sha12 4fdf249cda0a) · v0.1 best `Downloads\xm_ft_kaggle_full\xm_ft_kaggle_e10\checkpoints\epoch=8-step=10719.ckpt`(sha12 d86970ce1faf) — 집에서 오프라인 평가 비교용.
5. **HF 토큰(선택)** — 1번을 `-Ckpt` 로 주면 불필요(학습 중 받는 DINOv2 `facebook/dinov2-small` 은 공개 모델). 못 가져갔을 때만 Kaggle 비밀 `HF_TOKEN` 과 같은 것을 `$env:HF_TOKEN='hf_...'` 로.
6. 레포 push 확인(`git status` 깨끗, `git log origin/main..HEAD` 비어 있음).

**11.9 계획 — 집 PC 순서**(`PLAN.md` "11.9 계획", 판정 기준은 거기 고정)

| 순서 | run(`-o` 이름) | 데이터 | 설정 | 언제 |
| --- | --- | --- | --- | --- |
| **E1** | aw30 + 직전 명령 드롭아웃 0.5 → `xm_ft_v0.1_aw30_pad05_e10` | xm_v1(v1.0 split) | `XMobilityLoss.action_weight=30` · `--prev-action-dropout 0.5` · 10 에폭 · `--save-top-k 3` | 11.9 밤 |
| **E2** | aw30 + 드롭아웃 1.0(직전 명령 입력 끔) → `xm_ft_v0.1_aw30_pad10_e10` | xm_v1 | 위와 같고 1.0 | 11.10 낮 |
| E3 | aw30 + p3 + E1 · E2 중 나은 쪽 → `xm_ft_v0.1_aw30_p3_padXX_e10` | xm_v1_1_p3(`03_xm_v1_1_p3.zip` · Kaggle `dataset-06-xm-v1-1-p3`) | 결정 1(11.11) 뒤 | 11.11 밤 |
| E4 | 실행 명령 입력판(동현님 재변환이 오면) 또는 경로 축척 1/2 | — | 11.12 에 정함 | 11.12 |

- 왜 드롭아웃: 교사 강제 입력 평가에서 모델이 직전 명령 입력을 거의 그대로 이어 가는 것을 확인(`results/tf_eval_11_9/README.md`). 학습 때 그 입력을 흐려 덜 기대게 함.
- 집 run 은 torch **2.7.1+cu128**, Kaggle 학습은 **2.11.0+cu130** — "같은 설정, 다른 torch". aw30(Kaggle)과 비교할 때 시드 바닥(`results/seed_noise_11_8.md`)보다 큰 차이만 봄.
- E4 가 경로 축척 1/2 이면 데이터는 집 PC 에서 직접 만듦(업로드 불필요, 몇 분). 11.10 노트북에서 같은 두 줄로 만든 v1.1 이 원본 v1.1 과 같음 확인(RESPLIT.json 뺀 sha12 `872a64f9d0e7`):
  `.\.venv-xm\Scripts\python.exe e2e\resplit_xm.py C:\fd\data\xm_v1 C:\fd\data\xm_v1_1 --preset v1.1` → `.\.venv-xm\Scripts\python.exe e2e\scale_path_xm.py C:\fd\data\xm_v1_1 C:\fd\data\xm_v1_1_p2 --scale 0.5`. 노드는 `path_scale:=0.5`.
- 결과 받기: 학습이 끝나면 블록 마지막 줄(`home_after_train.ps1`)이 평가 → 곡선 → Kaggle 비공개 데이터셋 `henryseo711/fd-xm-home-<run>` 업로드까지 함. 노트북 쪽에 run 이름만 알려 주면 됨.

**도착 후(약 1시간 + 본 학습)** — PowerShell 일반 콘솔, 아래 블록을 그대로:

```powershell
cd C:\fd\FUTURE-DRIVE-PROJECT
git pull                                                  # 11.9 푸시 이후 — e2e\home_after_train.ps1 이 있어야 함
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
powercfg /change standby-timeout-ac 0
$CK = "$HOME\Downloads\xm_pretrained_from_seed_try2\pretrained\x_mobility-nav2-semantic_action_path.ckpt"
# E1  aw30 + 직전 명령 드롭아웃 0.5 — xm_v1, 10 에폭. 설치 → 시험 40배치(드롭아웃 포함) → 본 학습 명령 출력(실행은 직접)
.\e2e\setup_windows.ps1 -Data C:\fd\data\xm_v1 -Zip "$HOME\Downloads\02_xm_v1.zip" -Ckpt $CK -ExtraGin 'XMobilityLoss.action_weight=30' -ExtraArgs '--prev-action-dropout','0.5','--save-top-k','3' -MaxEpochs 10 -HoursBudget 14
# 시험 로그에 "[직전 명령 입력] 학습 때 드롭아웃 p=0.5" 가 찍혔는지 확인
# 출력된 본 학습 명령: --gin NUM_EPOCHS=10 인지 확인(아니면 -HoursBudget 을 올려 다시) → -o 만 아래로 바꿔 붙여 넣기
#   -o 'C:\fd\FUTURE-DRIVE-PROJECT\data\checkpoints\xm_ft_v0.1_aw30_pad05_e10'
# 학습 끝: 평가 → 곡선 → Kaggle 업로드(노트북에서 받아 판정)
.\e2e\home_after_train.ps1 -Run data\checkpoints\xm_ft_v0.1_aw30_pad05_e10 -Data C:\fd\data\xm_v1
# E2: 위 setup 줄의 '0.5' → '1.0' 로 다시(설치 · 시험은 건너뜀), -o 와 home_after_train 의 이름은 xm_ft_v0.1_aw30_pad10_e10
```

- `-HoursBudget 14` 는 `--max-time` 상한일 뿐. 10 에폭 ≈ 4,764 스텝 × step_time_s × 1.15 × 10 — 0.5 s 면 7.6 h, T4 실측 0.655 s 면 10.0 h. `NUM_EPOCHS` 가 10 보다 작게 찍히면 OneCycle lr 스케줄(total_steps)이 달라져 v0.1 · aw30 · seed1 과 비교 불가 → 예산을 올려 다시.
- `--save-top-k 3`: 스크립트는 `--save-top-k 1` 을 고정으로 넣고 그 **뒤에** `-ExtraArgs` 를 붙이므로 argparse 가 마지막 값 3 을 씀(Kaggle `SAVE_TOP_K = 3` 과 같음, ckpt 1 GB × 4).
- `-o`: 스크립트 출력은 `data\checkpoints\xm_ft_local_e{N}` 고정(옵션 없음) → 이름을 안 바꾸면 두 번째 run 부터 "이미 체크포인트가 있음"으로 멈춤.

| 분 | 할 일 | 확인 |
| --- | --- | --- |
| 0–5 | `nvidia-smi` · Windows 업데이트 재시작 미루기 · `powercfg /change standby-timeout-ac 0` | 드라이버 ≥ 570.65 |
| 5–10 | `winget install -e --id Python.Python.3.12`, `winget install -e --id Git.Git` → PowerShell **새로** 열기 | `py -3.12 -V` |
| 10–15 | `git clone https://github.com/JP-Avengers/FUTURE-DRIVE-PROJECT.git C:\fd\FUTURE-DRIVE-PROJECT`, 챙겨 온 zip · ckpt 를 `$HOME\Downloads\` 같은 경로에(zip 은 `-Zip` 으로 스크립트가 `C:\fd\data\` 에 풂, 미리 풀어도 됨) | `$CK` 파일 있음 · (풀었으면) `C:\fd\data\xm_v1\train` |
| 15–45 | 위 블록의 `setup_windows.ps1` 줄 | pip 10–30 분 · 시험 약 5 분 |
| 45 | 시험 `logs\xm_ft_smoke\run_meta.json`: `missing 0` · `peak_gpu_mem_gb` ≤ 14.5 · `step_time_s` | 값을 `e2e/README.md` "GPU 메모리별 배치" 표의 RTX 5060 Ti 행(11.8 자리 표시)에 적기 |
| 45– | 출력 명령의 `NUM_EPOCHS=10` 확인 → `-o` 바꿔 실행 | 에폭당 ≈ 4,764 × step_time_s × 1.15 — GPU 1장이라 Kaggle(2,382 스텝 · 28 분, T4×2)의 2배 스텝. 0.5 s 면 46 분, 0.655 s 면 60 분 |
| 끝 | 위 블록 마지막 줄(오프라인 평가) | `eval_test\summary.json` |

막히면 아래 "문제 해결" 표 → 그래도 안 되면 WSL2 절(맨 아래).

## 어느 OS 로

| 경로 | 판단 | 이유 |
| --- | --- | --- |
| **Windows 그대로** | 1순위(스크립트) | OS 설치 없음. GPU 1장이라 NCCL 이 필요 없고 gloo 로 충분(`finetune.py` 가 Windows 에서 gloo 기본). torch 2.7.1+cu128 Windows 휠에 sm_120 이 들어 있음(아래 근거) |
| WSL2 Ubuntu 24.04 | Windows 에서 막히면 | Windows 드라이버를 그대로 씀. 워커가 fork 라 데이터셋 사본 RAM 부담이 작음, nccl 가능. 대신 WSL 메모리 상한 · pinned memory 제한(NVIDIA 문서), 데이터를 리눅스 쪽 디스크에 둬야 함 |
| Ubuntu 직접 설치 | 이번 주말엔 비추천 | 가장 표준적인 환경(X-Mobility 원 환경은 리눅스 컨테이너)이지만 설치 · 드라이버(570 이상, `-open` 모듈 — 미검증) 시간이 듦 |

## 준비물

| 항목 | 내용 |
| --- | --- |
| NVIDIA 드라이버 | **570.65 이상**(CUDA 12.8 GA 의 Windows 최소 — NVIDIA CUDA Toolkit Release Notes). 5060 Ti 를 인식하는 드라이버면 보통 이미 넘음(미검증). CUDA Toolkit 은 따로 안 깔아도 됨 — torch 휠 안에 CUDA 12.8 런타임 · cuBLAS · cuDNN 9 DLL 이 들어 있음 |
| Python 3.12 | `winget install -e --id Python.Python.3.12` → PowerShell 새로 열기. 다른 PC 의 3.14 와 같이 깔려도 됨(`py -3.12` 로 고름). **3.14 · 3.13 은 못 씀** — 고정판 numpy 1.26.4 에 cp313 · cp314 휠이 없음 |
| Git | `winget install -e --id Git.Git` |
| 디스크 | 여유 **20 GB 이상** — `.venv-xm` 약 6 GB(torch 만 5.4 GB) · pip 캐시 3.3 GB · 사전학습 ckpt 1.44 GB · 데이터 zip + 압축 해제 약 1.6 GB · 본 학습 ckpt 약 4 GB(`--save-top-k 3` + last, 1 GB × 4) |
| RAM | 16 GB 이상, 32 GB 권장(추정 — 아래 "워커 RAM") |
| Hugging Face(선택) | `-Ckpt` 로 로컬 사전학습 ckpt 를 주면 불필요. 아니면 huggingface.co 로그인 → [`nvidia/X-Mobility`](https://huggingface.co/nvidia/X-Mobility) 약관 동의 → Settings → Access Tokens 에서 **Read** 토큰 |
| 데이터 | 학습은 **xm_v1**(`02_xm_v1.zip` 805 MB, 안에 `xm_v1\` · Kaggle `henryseo711/dataset-02-xm-v1` — v0.1 · aw30 · seed1 과 같은 v1.0 split) → `-Data <폴더>`(train/val/test 가 바로 아래) 또는 `-Zip`(기본값 `$HOME\Downloads\02_xm_v1.zip`). xm_v1_2(`03_xm_v1_2.zip` 1.14 GB · `dataset-03-xm-v1-2`)는 v0.2 탈락(11.7, `results/compare_11_7.md`) — 안 씀 |

레포는 짧은 경로(예: `C:\fd\FUTURE-DRIVE-PROJECT`)에, OneDrive 밖에 두세요 — 설치 파일 경로가 260자 한도에 걸리거나 6 GB 가 동기화됩니다.

## 실행

PowerShell(ISE 말고 일반 콘솔)에서:

```powershell
cd C:\fd\FUTURE-DRIVE-PROJECT
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass   # 이 창에서만 스크립트 허용
$env:HF_TOKEN = 'hf_...'                                       # -Ckpt 로 로컬 ckpt 를 주면 불필요. 아니면 이것 또는 huggingface-cli login
.\e2e\setup_windows.ps1 -HoursBudget 8                               # 오늘 밤 실제 줄은 위 "도착 후" 블록
```

다시 돌려도 됩니다 — 설치 · clone · ckpt · 시험 결과가 있으면 건너뜁니다. 주요 옵션:

| 옵션 | 기본값 | 뜻 |
| --- | --- | --- |
| `-Data` | `$HOME\Downloads\xm_v1_unzip\xm_v1` | train/val/test 가 있는 폴더 |
| `-Zip` | `$HOME\Downloads\02_xm_v1.zip` | `-Data` 가 없으면 이 zip 을 풂 |
| `-Ckpt` / `-CkptDir` | — / `$HOME\xm_ckpt` | 이미 받은 ckpt 파일(주면 HF 를 안 탐) / 받을 폴더 |
| `-HoursBudget` | 8 | 본 학습 시간(시간). 에폭 수 계산과 `--max-time` 상한에 씀. 에폭 수가 `-MaxEpochs` 보다 작게 나오면 올릴 것(에폭 수 = lr 스케줄) |
| `-MaxEpochs` | 10 | 에폭 상한(gin 기본값) |
| `-NumWorkers` | 2 | DataLoader 워커 수 |
| `-BatchSize` / `-Accum` | 2 / 4 | 배치 · 기울기 누적(실효 배치 8 = Kaggle T4×2 와 같음). 16 GB 에서 배치 4 는 OOM(11.6 실측) |
| `-ExtraArgs` | — | `finetune.py` 인자 덧붙이기, 예: `-ExtraArgs '--seed','1','--save-top-k','3'`(`--save-top-k` 는 스크립트가 1 로 고정 → 여기서 3 으로 덮음, 뒤 값이 이김) |
| `-ExtraGin` | — | gin 덧붙이기(시험 · 본 학습 둘 다), 예: `-ExtraGin 'XMobilityLoss.action_weight=30'` |
| `-XmDir` | 레포 옆 `..\X-Mobility` | X-Mobility clone 위치 |
| `-ForceSmoke` / `-SkipSmoke` / `-SkipDataCheck` | — | 시험 다시 / 시험 건너뜀 / 데이터 점검 건너뜀 |

출력 폴더 옵션은 없음 — 출력된 본 학습 명령의 `-o`(`data\checkpoints\xm_ft_local_e{N}`)를 run 이름으로 바꿔 실행.

## 단계와 기대 출력

| 단계 | 하는 일 | 정상이면 |
| --- | --- | --- |
| 1 | `nvidia-smi` — 이름 · 드라이버 · VRAM · compute capability, RAM · 디스크 | `드라이버 5xx.xx >= 570.65`, `compute capability 12.0` |
| 2 | `py -3.12`, git | 경로 출력 |
| 3 | 레포 루트에 `.venv-xm` (스스로 `.gitignore`) | `만듦` 또는 `있음` |
| 4 | `pip install -r` 고정 목록 72줄(torch 2.7.1+cu128 은 download.pytorch.org/whl/cu128) | 처음 10~30 분(torch 3.3 GB). 다음부턴 건너뜀 |
| 5 | X-Mobility 를 레포 옆 `..\X-Mobility` 에 clone, `5837aea` checkout | `HEAD 5837aead8902` |
| 6 | CUDA · 아키텍처 · fp32/fp16 matmul · gloo 1프로세스 all_reduce · X-Mobility import | `torch 빌드 아키텍처: sm_50 sm_60 sm_61 sm_70 sm_75 sm_80 sm_86 sm_90 sm_100 sm_120` |
| 7 | `-Ckpt` 면 그 파일 확인, 아니면 HF 에서 ckpt(1.44 GB) | 파일 경로(와 크기) |
| 8 | zip 풀기, `check_xm_dataset.py --image-stride 20` | `결과: 오류 0`, 요약 `logs\xm_v1_check.json` |
| 9 | `finetune.py` 1에폭 40배치 + 검증 4배치(`--no-test --gin NUM_EPOCHS=1`) → `logs\xm_ft_smoke\run_meta.json` | 키 대조 `missing 0`, unexpected 는 `model.semantic_decoder.*` 만. 시험 ckpt 는 끝나고 지움 |

마지막에 이렇게 나옵니다(숫자는 시험에서 잰 값):

```
step_time_s      x.xxx s (데이터 로딩 포함 중앙값)
steps_per_epoch  4764 (train 시퀀스 9529 · world_size 1)
peak_gpu_mem_gb  xx.x
키 대조          missing 0 · 예상 밖 unexpected 0
에폭당 예상      x.xx 시간 (= 스텝 × 스텝 시간 × 1.15, 검증 · 저장 몫 포함)
→ 예산 14 시간(여유 20 분 뺌) · 상한 10 에폭 → NUM_EPOCHS=10 (예상 x.x 시간)
==> 본 학습 명령 ...
& 'C:\...\.venv-xm\Scripts\python.exe' 'C:\...\e2e\finetune.py' --xm-repo ... --save-top-k 1 --accumulate-grad-batches 4 --seed 1 --save-top-k 3 -o 'C:\...\data\checkpoints\xm_ft_local_e10' --gin NUM_EPOCHS=10 --max-time '00:14:00:00' --gin XMobilityIsaacSimDataModule.num_workers=2 --gin XMobilityIsaacSimDataModule.batch_size=2 --gin XMobilityLoss.action_weight=30
```

steps_per_epoch 4,764 = xm_v1 train 9,529 시퀀스 ÷ 배치 2(GPU 1장) = Kaggle run_meta 2,382(T4×2) × 2. 에폭 수 = `floor((예산 초 − 1200) / (steps_per_epoch × step_time_s × 1.15))`, 1~`MaxEpochs`. 참고로 스텝 0.5 s 면 에폭당 약 0.76 시간, T4 실측 0.655 s 면 약 1.0 시간(4,764 × 스텝 × 1.15, 계산값). 10 에폭이 다 들어가려면 예산 8 시간은 스텝 0.50 s 이하, 14 시간은 0.90 s 이하.

## 본 학습

- 출력된 명령의 `-o` 를 run 이름으로 바꿔 붙여 넣습니다(11.9 밤 E1 = `xm_ft_v0.1_aw30_pad05_e10`). 끝나면 `home_after_train.ps1`. 결과는 그 폴더(`run_meta.json` · `checkpoints\` · `wandb\` 오프라인) → MANIFEST `checkpoint` 행. best = `run_meta.json` 의 `best_model_path`(콘솔 `[끝] best ...`).
- `--gin NUM_EPOCHS` 가 `-MaxEpochs` 보다 작으면 돌리지 말고 `-HoursBudget` 을 올려 다시 — 에폭 수가 다르면 OneCycle lr 스케줄(total_steps)이 달라 Kaggle 10 에폭 run 과 비교 불가.
- 돌리기 전: 절전 끄기 `powercfg /change standby-timeout-ac 0`, Windows 업데이트 재시작 미루기, 콘솔 창 열어 두기.
- 이어 하기 기능은 없습니다. 끊기면 `-p <그 폴더>\checkpoints\last.ckpt` 로 가중치만 이어 새 폴더에 다시(lr 스케줄은 처음부터, 미검증).
- 학습 뒤 오프라인 평가는 스크립트에 없음 — Kaggle 노트북처럼: `& .\.venv-xm\Scripts\python.exe e2e\eval_offline.py --xm-repo ..\X-Mobility -c e2e\configs\finetune_parking_r3.gin -d <데이터> --split test -p <best ckpt> -o <run 폴더>\eval_test` → `eval_test\summary.json` · `frames.csv`. 비교 상대(aw30 · v0.1 best)도 같은 명령에 `-p` 만 바꿔.

## 문제 해결

| 증상 | 원인 · 조치 |
| --- | --- |
| 6단계 `sm_120 이(가) 이 torch 빌드에 없음` · 학습 중 `no kernel image is available` | cu128 빌드가 아님(CPU 판 · cu126 이하). `Remove-Item -Recurse -Force .venv-xm` 후 다시. 확인: `.venv-xm\Scripts\python.exe -c "import torch; print(torch.__version__)"` → `2.7.1+cu128` |
| `torch.cuda.is_available() = False` | 드라이버(`nvidia-smi`) 또는 `CUDA_VISIBLE_DEVICES` |
| `CUDA out of memory` | `-BatchSize 1 -Accum 8 -ForceSmoke`(실효 배치 그대로 8) 또는 DINOv2 고정 `-ExtraGin "finetune.freeze_prefixes=['model.observation_encoder.image_encoder.']"` |
| `WinError 1455`(페이징 파일이 너무 작음) · `MemoryError` · `DataLoader worker ... exited unexpectedly` | 워커 RAM/커밋 부족. `-NumWorkers 1 -ForceSmoke`, 페이지 파일 "시스템이 관리" 확인 |
| 7단계 HTTP 401 | 토큰 없음/틀림 — `$env:HF_TOKEN` 또는 `.venv-xm\Scripts\huggingface-cli.exe login` |
| 7단계 HTTP 403 | 약관 미동의, 또는 fine-grained 토큰에 "public gated repos" 읽기 권한 없음 → Read 토큰으로 |
| `스크립트를 실행할 수 없습니다`(실행 정책) | 같은 창에서 `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass` |
| pip 설치 중 경로 오류(긴 경로) | 레포를 짧은 경로로 옮기거나 LongPathsEnabled=1(관리자) |
| 6단계 gloo 에서 실패 | Windows 쪽 통신 문제 — WSL2 경로로 |
| `이미 체크포인트가 있음` | 같은 출력 폴더 — `-o` 이름을 바꿈(덮어쓰지 않게 막아 둔 것) |

### 워커 RAM (Windows)

`base_train_config.gin` 이 `use_lazy_loading=False` 라 메인 프로세스가 train · val parquet 전부(JPEG 바이트)를 pandas 로 들고 있습니다. Windows 는 DataLoader 워커를 spawn 으로 띄워서 **워커마다 train 데이터셋을 pickle 사본으로 받습니다**(Linux 는 fork 라 대부분 공유). 합성 데이터에서 pickle 크기 ≈ parquet 크기(11.82 vs 11.79 MB)였고, xm_v1 zip 805 MB 중 train 이 80 %(38,211/47,572장)라 **워커당 약 0.65 GB + 파이썬 · torch 기본분**(추정, 0.3~0.6 GB). 워커 2개 ≈ 2~2.5 GB, 4개 ≈ 4~5 GB 가 메인(수 GB) 위에 얹힙니다. `persistent_workers=False` 라 에폭 · 검증마다 워커를 다시 띄우며 그때마다 사본을 다시 보냅니다(수 초). 데이터 로딩이 병목인지 보려면 `-NumWorkers 4 -ForceSmoke` 로 `step_time_s` 를 비교하세요.

## WSL2 / Ubuntu (bash)

```bash
# WSL2 만: PowerShell(관리자)에서  wsl --install -d Ubuntu-24.04  → 재부팅 → 사용자 만들기
#   24.04 를 지정(기본 파이썬 3.12). WSL 안에는 NVIDIA 리눅스 드라이버를 깔지 말 것(NVIDIA CUDA on WSL 안내)
#   메모리 상한: %UserProfile%\.wslconfig 에 [wsl2] memory=24GB → wsl --shutdown
# Ubuntu 직접 설치: 드라이버 570 이상(open 모듈, 예: sudo ubuntu-drivers install) — 미검증
nvidia-smi
sudo apt update && sudo apt install -y python3.12-venv git unzip
mkdir -p ~/fd && cd ~/fd
git clone https://github.com/JP-Avengers/FUTURE-DRIVE-PROJECT.git
git clone https://github.com/NVlabs/X-Mobility.git && git -C X-Mobility checkout -q 5837aea
python3.12 -m venv ~/fd/venv-xm && . ~/fd/venv-xm/bin/activate && pip install -U pip
# 고정 목록은 setup_windows.ps1 의 표시 줄 사이를 그대로 씀(리눅스 휠도 같은 판으로 풀리는 것 확인)
sed -n '/^# >>> xm-requirements/,/^# <<< xm-requirements/p' FUTURE-DRIVE-PROJECT/e2e/setup_windows.ps1 > /tmp/xm_req.txt
pip install --extra-index-url https://download.pytorch.org/whl/cu128 -r /tmp/xm_req.txt
python -c "import torch; print(torch.__version__, torch.cuda.get_device_name(0), torch.cuda.get_arch_list())"

# 데이터는 리눅스 쪽 디스크로(/mnt/c 는 느림)
mkdir -p ~/fd/data && unzip -q /mnt/c/Users/<사용자>/Downloads/02_xm_v1.zip -d ~/fd/data
export HF_TOKEN=hf_...
python -c "from huggingface_hub import hf_hub_download as d; print(d('nvidia/X-Mobility', 'x_mobility-nav2-semantic_action_path.ckpt', local_dir='$HOME/fd/ckpt'))"

cd ~/fd/FUTURE-DRIVE-PROJECT && mkdir -p logs
python e2e/check_xm_dataset.py ~/fd/data/xm_v1 --image-stride 20 --json logs/xm_v1_check.json
CK=~/fd/ckpt/x_mobility-nav2-semantic_action_path.ckpt
python e2e/finetune.py --xm-repo ~/fd/X-Mobility -c e2e/configs/finetune_parking_r3.gin -d ~/fd/data/xm_v1 -p $CK \
    -o logs/xm_ft_smoke --wandb offline --save-top-k 1 --limit-train-batches 40 --limit-val-batches 4 --no-test \
    --gin NUM_EPOCHS=1 --gin XMobilityIsaacSimDataModule.num_workers=4
python -c "import json, math; m = json.load(open('logs/xm_ft_smoke/run_meta.json')); e = m['steps_per_epoch'] * m['step_time_s'] * 1.15; \
print(m['step_time_s'], m['steps_per_epoch'], m['peak_gpu_mem_gb'], 'h/epoch', round(e / 3600, 2), 'NUM_EPOCHS(8h)', max(1, min(10, math.floor((8 * 3600 - 1200) / e))))"
# 본 학습(N = 위 값). 리눅스는 --ddp-backend 기본 nccl(torch 리눅스 휠에 nvidia-nccl-cu12 포함), GPU 1장이라 gloo 도 됨
python e2e/finetune.py --xm-repo ~/fd/X-Mobility -c e2e/configs/finetune_parking_r3.gin -d ~/fd/data/xm_v1 -p $CK \
    -o data/checkpoints/xm_ft_local_eN --wandb offline --save-top-k 1 --gin NUM_EPOCHS=N --max-time 00:08:00:00 \
    --gin XMobilityIsaacSimDataModule.num_workers=4
```

## 근거 · 검증 상태 (11.5)

확인한 것:
- **torch 2.7.1+cu128**: `https://download.pytorch.org/whl/cu128/torch/` 에 `torch-2.7.1+cu128-cp312-cp312-win_amd64.whl`(3.27 GB). 이 인덱스의 Windows 휠은 2.7.0 부터(2.7.0 ~ 2.11.0, cp310~cp313, cp314 는 2.9.0 부터). PyTorch 2.7 릴리스 글: "support for the NVIDIA Blackwell GPU architecture and pre-built wheels for CUDA 12.8". 휠 안 `torch/version.py` 의 cuda = `12.8`, `torch/lib/torch_python.dll` 의 아키텍처 문자열 = `sm_50 sm_60 sm_61 sm_70 sm_75 sm_80 sm_86 sm_90 sm_100 sm_120`(range 요청으로 그 파일만 꺼내 봄). torchvision `0.22.1+cu128` cp312 win 휠 있음.
- **판 고정**: Kaggle 노트북의 고정 8개(gin-config · einops · pytorch-lightning · transformers · diffusers · huggingface_hub · timm · torcheval)와 X-Mobility requirements 의 numpy 1.26.4 · wandb 0.19.4 는 같은 판. X-Mobility 의 pandas 1.5.3(cp312 휠 없음) · pyarrow 14.0.1.dev0(NGC 빌드, PyPI 에 없음)은 못 써서, 나머지는 2025-06-30 기준으로 풀어 고정(총 72줄). `pip install --dry-run --platform win_amd64 --python-version 3.12 --only-binary=:all:` 로 전부 바이너리 휠로 풀림(tokenizers 0.21.2 = `cp39-abi3-win_amd64`). 리눅스 cp312 + cu128 로도 풀림(nvidia-nccl-cu12 2.26.2 · triton 3.3.1 포함).
- **동작**: 같은 고정 목록(torch 만 2.7.1+cpu)으로 리눅스 Python 3.12 에서 `finetune.py` 를 합성 xm_v1 형식 데이터로 끝까지 돌림 — gloo DDP(world_size 1), **spawn 워커 2개**(Windows 와 같은 방식), 학습 8스텝 · 검증 · ckpt 저장 · test(ckpt 다시 읽기) 통과, `run_meta.json` 에 `step_time_s` · `steps_per_epoch` 기록. 이 스크립트에 든 보조 파이썬 3개도 리눅스에서 돌려 봄(HF 토큰 없이 → `GatedRepoError` HTTP 401 → 종료 코드 4).
- 드라이버 최소: CUDA 12.8 GA = Windows ≥ 570.65 · Linux ≥ 570.26(NVIDIA CUDA Toolkit Release Notes). WSL: "Do not install any Linux display driver in WSL", "Pinned system memory ... availability for applications is limited"(NVIDIA CUDA on WSL User Guide).
- torch 2.7.1 Windows 는 TCPStore 가 libuv 를 기본으로 안 씀(`torch/distributed/rendezvous.py`) — `USE_LIBUV` 설정 불필요.
- **Kaggle 과 torch 가 다름**: Kaggle 본 학습(v0.1 · aw30 · v0.2 · seed1 · p3)은 torch 2.11.0+cu130(run_meta) · torchmetrics 미고정, 집은 2.7.1+cu128 · torchmetrics 1.7.3. 고정 8개 · numpy · wandb 는 같지만 torch 가 달라 수치가 완전히 같지 않음 — 집 run 은 "같은 설정, 다른 torch" 로만 읽음(11.8 에는 고정판을 안 바꿈 — cu128 경로가 검증된 쪽).

확인 못 한 것:
- **실제 Windows · RTX 5060 Ti 에서 한 번도 안 돌림** — CUDA 커널, Windows gloo + CUDA 텐서, spawn 워커 RAM, WinError 1455, 배치 4 가 16 GB 에 들어가는지, 스텝 시간.
- ~~`setup_windows.ps1` 파싱~~ → 11.7 PowerShell 7.6.9 파서(`Parser::ParseFile`)로 구문 오류 0. **실행**은 여전히 GPU 있는 PC 에서만 가능.
- 사전학습 ckpt 키 대조는 Kaggle 에서만 확인(297 · missing 0 · unexpected 61 예상분; Windows 에서는 9단계가 처음), 5060 Ti 출시 드라이버 판, Ubuntu 직접 설치 시 open 모듈 필수 여부, WSL 기본 메모리 상한.
