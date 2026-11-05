# 로컬 학습 준비 — Windows · RTX 5060 Ti 16 GB

`e2e/setup_windows.ps1` 한 번으로 환경 설치 → GPU 확인 → 사전학습 ckpt → 데이터 점검 → 40배치 시험 → 본 학습 명령 출력까지 갑니다. 흐름은 `kaggle_finetune.ipynb` 와 같습니다(시험으로 스텝 시간을 재고, 시간 예산에 맞춰 에폭 수를 정함).

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
| 디스크 | 여유 **20 GB 이상** — `.venv-xm` 약 6 GB(torch 만 5.4 GB) · pip 캐시 3.3 GB · 사전학습 ckpt 1.44 GB · 데이터 zip + 압축 해제 약 1.6 GB · 본 학습 ckpt 약 2 GB(1 GB × best + last) |
| RAM | 16 GB 이상, 32 GB 권장(추정 — 아래 "워커 RAM") |
| Hugging Face | huggingface.co 로그인 → [`nvidia/X-Mobility`](https://huggingface.co/nvidia/X-Mobility) 약관 동의 → Settings → Access Tokens 에서 **Read** 토큰 |
| 데이터 | 팀 드라이브 `02_xm_v1.zip`(805 MB)을 `Downloads` 에 두면 스크립트가 `Downloads\xm_v1_unzip\` 에 풂 |

레포는 짧은 경로(예: `C:\fd\FUTURE-DRIVE-PROJECT`)에, OneDrive 밖에 두세요 — 설치 파일 경로가 260자 한도에 걸리거나 6 GB 가 동기화됩니다.

## 실행

PowerShell(ISE 말고 일반 콘솔)에서:

```powershell
cd C:\fd\FUTURE-DRIVE-PROJECT
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass   # 이 창에서만 스크립트 허용
$env:HF_TOKEN = 'hf_...'                                       # 또는 나중에 huggingface-cli login
.\e2e\setup_windows.ps1 -HoursBudget 8
```

다시 돌려도 됩니다 — 설치 · clone · ckpt · 시험 결과가 있으면 건너뜁니다. 주요 옵션:

| 옵션 | 기본값 | 뜻 |
| --- | --- | --- |
| `-Data` | `$HOME\Downloads\xm_v1_unzip\xm_v1` | train/val/test 가 있는 폴더 |
| `-Zip` | `$HOME\Downloads\02_xm_v1.zip` | `-Data` 가 없으면 이 zip 을 풂 |
| `-Ckpt` / `-CkptDir` | — / `$HOME\xm_ckpt` | 이미 받은 ckpt 파일 / 받을 폴더 |
| `-HoursBudget` | 8 | 본 학습 시간(시간). 에폭 수 계산과 `--max-time` 상한에 씀 |
| `-MaxEpochs` | 10 | 에폭 상한(gin 기본값) |
| `-NumWorkers` | 2 | DataLoader 워커 수 |
| `-ExtraGin` | — | gin 덧붙이기(시험 · 본 학습 둘 다), 예: `-ExtraGin 'XMobilityIsaacSimDataModule.batch_size=2'` |
| `-ForceSmoke` / `-SkipSmoke` / `-SkipDataCheck` | — | 시험 다시 / 시험 건너뜀 / 데이터 점검 건너뜀 |

## 단계와 기대 출력

| 단계 | 하는 일 | 정상이면 |
| --- | --- | --- |
| 1 | `nvidia-smi` — 이름 · 드라이버 · VRAM · compute capability, RAM · 디스크 | `드라이버 5xx.xx >= 570.65`, `compute capability 12.0` |
| 2 | `py -3.12`, git | 경로 출력 |
| 3 | 레포 루트에 `.venv-xm` (스스로 `.gitignore`) | `만듦` 또는 `있음` |
| 4 | `pip install -r` 고정 목록 74줄(torch 2.7.1+cu128 은 download.pytorch.org/whl/cu128) | 처음 10~30 분(torch 3.3 GB). 다음부턴 건너뜀 |
| 5 | X-Mobility 를 레포 옆 `..\X-Mobility` 에 clone, `5837aea` checkout | `HEAD 5837aead8902` |
| 6 | CUDA · 아키텍처 · fp32/fp16 matmul · gloo 1프로세스 all_reduce · X-Mobility import | `torch 빌드 아키텍처: sm_50 sm_60 sm_61 sm_70 sm_75 sm_80 sm_86 sm_90 sm_100 sm_120` |
| 7 | HF 에서 ckpt(1.44 GB) | 파일 경로와 크기 |
| 8 | zip 풀기, `check_xm_dataset.py --image-stride 20` | `결과: 오류 0`, 요약 `logs\xm_v1_check.json` |
| 9 | `finetune.py` 1에폭 40배치 + 검증 4배치(`--no-test --gin NUM_EPOCHS=1`) → `logs\xm_ft_smoke\run_meta.json` | 키 대조 `missing 0`, unexpected 는 `model.semantic_decoder.*` 만. 시험 ckpt 는 끝나고 지움 |

마지막에 이렇게 나옵니다(숫자는 시험에서 잰 값):

```
step_time_s      x.xxx s (데이터 로딩 포함 중앙값)
steps_per_epoch  약 2390 (xm_v1, 배치 4, GPU 1장 — gin 주석 기준 계산값)
peak_gpu_mem_gb  xx.x
에폭당 예상      x.xx 시간 (= 스텝 × 스텝 시간 × 1.15)
→ 예산 8 시간(여유 20 분 뺌) · 상한 10 에폭 → NUM_EPOCHS=N
==> 본 학습 명령 ...
& 'C:\...\.venv-xm\Scripts\python.exe' 'C:\...\e2e\finetune.py' --xm-repo ... -o '...\data\checkpoints\xm_ft_local_eN' --gin NUM_EPOCHS=N --max-time '00:08:00:00' ...
```

에폭 수 = `floor((예산 초 − 1200) / (steps_per_epoch × step_time_s × 1.15))`, 1~`MaxEpochs`. 참고로 스텝 0.5 s 면 에폭당 약 0.38 시간, 1.0 s 면 약 0.76 시간(계산값, 실측 아님).

## 본 학습

- 출력된 명령을 그대로 붙여 넣습니다. 결과는 `data\checkpoints\xm_ft_local_eN\`(`run_meta.json` · `checkpoints\` · `wandb\` 오프라인) → MANIFEST `checkpoint` 행.
- 돌리기 전: 절전 끄기 `powercfg /change standby-timeout-ac 0`, Windows 업데이트 재시작 미루기, 콘솔 창 열어 두기.
- 이어 하기 기능은 없습니다. 끊기면 `-p <그 폴더>\checkpoints\last.ckpt` 로 가중치만 이어 새 폴더에 다시(lr 스케줄은 처음부터, 미검증).

## 문제 해결

| 증상 | 원인 · 조치 |
| --- | --- |
| 6단계 `sm_120 이(가) 이 torch 빌드에 없음` · 학습 중 `no kernel image is available` | cu128 빌드가 아님(CPU 판 · cu126 이하). `Remove-Item -Recurse -Force .venv-xm` 후 다시. 확인: `.venv-xm\Scripts\python.exe -c "import torch; print(torch.__version__)"` → `2.7.1+cu128` |
| `torch.cuda.is_available() = False` | 드라이버(`nvidia-smi`) 또는 `CUDA_VISIBLE_DEVICES` |
| `CUDA out of memory` | `-ExtraGin 'XMobilityIsaacSimDataModule.batch_size=2' -ForceSmoke`(에폭당 스텝 2배, lr 그대로) 또는 DINOv2 고정 `-ExtraGin "finetune.freeze_prefixes=['model.observation_encoder.image_encoder.']" -ForceSmoke`(22.1 M 파라미터 고정, 그만큼 역전파 메모리 감소) |
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
- **판 고정**: Kaggle 노트북의 고정 8개(gin-config · einops · pytorch-lightning · transformers · diffusers · huggingface_hub · timm · torcheval)와 X-Mobility requirements 의 numpy 1.26.4 · wandb 0.19.4 는 같은 판. X-Mobility 의 pandas 1.5.3(cp312 휠 없음) · pyarrow 14.0.1.dev0(NGC 빌드, PyPI 에 없음)은 못 써서, 나머지는 2025-06-30 기준으로 풀어 고정(총 74줄). `pip install --dry-run --platform win_amd64 --python-version 3.12 --only-binary=:all:` 로 전부 바이너리 휠로 풀림(tokenizers 0.21.2 = `cp39-abi3-win_amd64`). 리눅스 cp312 + cu128 로도 풀림(nvidia-nccl-cu12 2.26.2 · triton 3.3.1 포함).
- **동작**: 같은 고정 목록(torch 만 2.7.1+cpu)으로 리눅스 Python 3.12 에서 `finetune.py` 를 합성 xm_v1 형식 데이터로 끝까지 돌림 — gloo DDP(world_size 1), **spawn 워커 2개**(Windows 와 같은 방식), 학습 8스텝 · 검증 · ckpt 저장 · test(ckpt 다시 읽기) 통과, `run_meta.json` 에 `step_time_s` · `steps_per_epoch` 기록. 이 스크립트에 든 보조 파이썬 3개도 리눅스에서 돌려 봄(HF 토큰 없이 → `GatedRepoError` HTTP 401 → 종료 코드 4).
- 드라이버 최소: CUDA 12.8 GA = Windows ≥ 570.65 · Linux ≥ 570.26(NVIDIA CUDA Toolkit Release Notes). WSL: "Do not install any Linux display driver in WSL", "Pinned system memory ... availability for applications is limited"(NVIDIA CUDA on WSL User Guide).
- torch 2.7.1 Windows 는 TCPStore 가 libuv 를 기본으로 안 씀(`torch/distributed/rendezvous.py`) — `USE_LIBUV` 설정 불필요.

확인 못 한 것:
- **실제 Windows · RTX 5060 Ti 에서 한 번도 안 돌림** — CUDA 커널, Windows gloo + CUDA 텐서, spawn 워커 RAM, WinError 1455, 배치 4 가 16 GB 에 들어가는지, 스텝 시간.
- **`setup_windows.ps1` 은 PowerShell 로 파싱하지 못함**(이 환경에 pwsh 없음). 괄호 · 따옴표 · here-string 짝과 PS7 전용 연산자만 간이 검사.
- 사전학습 ckpt 키 대조(게이트라 못 받음), 5060 Ti 출시 드라이버 판, Ubuntu 직접 설치 시 open 모듈 필수 여부, WSL 기본 메모리 상한.
