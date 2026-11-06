#requires -Version 5.1
<#
.SYNOPSIS
  X-Mobility 파인튜닝(e2e/finetune.py) 로컬 준비 + 40배치 시험 — Windows 10/11 · RTX 5060 Ti 16 GB(Blackwell, sm_120).

.DESCRIPTION
  순서: GPU · 드라이버 → Python 3.12 → .venv-xm → torch 2.7.1+cu128 · 고정 패키지 → X-Mobility 5837aea
        → CUDA · sm_120 · gloo 확인 → 사전학습 ckpt(Hugging Face) → xm_v1 점검 → 40배치 시험 → 본 학습 명령 출력.
  여러 번 돌려도 됨(이미 된 단계는 건너뜀). 오류가 나면 그 자리에서 멈춤.
  PowerShell 5.1 / 7 콘솔에서 실행(ISE 는 네이티브 stderr 를 오류로 바꿔서 안 맞음). 설명서: e2e/SETUP_LOCAL.md

.EXAMPLE
  Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
  $env:HF_TOKEN = 'hf_...'
  .\e2e\setup_windows.ps1 -HoursBudget 8
#>
[CmdletBinding()]
param(
    # xm_v1 데이터 루트(train/val/test 가 바로 아래에 있는 폴더)
    [string]$Data = (Join-Path $HOME 'Downloads\xm_v1_unzip\xm_v1'),
    # $Data 가 없을 때 풀 zip(팀 드라이브 02_xm_v1.zip). $Data 의 부모 폴더에 풂
    [string]$Zip = (Join-Path $HOME 'Downloads\02_xm_v1.zip'),
    # 사전학습 ckpt 를 받을 폴더. 이미 받은 파일이 있으면 -Ckpt 로 직접 지정
    [string]$CkptDir = (Join-Path $HOME 'xm_ckpt'),
    [string]$Ckpt = '',
    # 본 학습에 쓸 시간(시간). 시험에서 잰 스텝 시간으로 에폭 수를 정하고, 같은 값을 --max-time 상한으로 씀
    [double]$HoursBudget = 8,
    [int]$MaxEpochs = 10,
    # DataLoader 워커 수. Windows(spawn)는 워커마다 train 데이터셋 사본(약 0.65 GB, 추정)을 가짐
    [int]$NumWorkers = 2,
    # 배치 · 기울기 누적. T4 16 GB 에서 배치 4 는 OOM, 배치 2 는 최대 10.4 GB(11.6 Kaggle 실측) → 16 GB 카드 기본 2 × 4 = Kaggle 과 같은 실효 배치 8
    [int]$BatchSize = 2,
    [int]$Accum = 4,
    # finetune.py 에 그대로 덧붙일 인자(시험 · 본 학습 둘 다). 예: -ExtraArgs '--seed','1'
    [string[]]$ExtraArgs = @(),
    # gin 바인딩 덧붙이기(시험 · 본 학습 둘 다). 예: 'XMobilityIsaacSimDataModule.batch_size=2'
    [string[]]$ExtraGin = @(),
    # X-Mobility 레포 위치(기본: 이 레포 옆 폴더 X-Mobility)
    [string]$XmDir = '',
    # 이전 시험 결과가 있어도 다시 돌림
    [switch]$ForceSmoke,
    # 시험을 건너뜀(이전 시험 결과가 있으면 그것으로 계산)
    [switch]$SkipSmoke,
    # 데이터 점검을 건너뜀
    [switch]$SkipDataCheck
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 1.0

# ---------------------------------------------------------------- 고정값
$PyVer           = '3.12'
$TorchVersion    = '2.7.1+cu128'
$TorchIndex      = 'https://download.pytorch.org/whl/cu128'
$XmUrl           = 'https://github.com/NVlabs/X-Mobility.git'
$XmRev           = '5837aead8902a2f4afdeb372548c45a6b0f8d69b'
$CkptRepo        = 'nvidia/X-Mobility'
$CkptFile        = 'x_mobility-nav2-semantic_action_path.ckpt'
$MinDriver       = [version]'570.65'   # CUDA 12.8 GA 의 Windows 최소 드라이버(NVIDIA CUDA Toolkit Release Notes 표)
$SmokeBatches    = 40
$SmokeValBatches = 4
$MarginS         = 1200                # 본 학습 시간 예산에서 빼는 여유(시작 · test · 저장), 초
$EpochOverhead   = 1.15                # 에폭당 검증 · ckpt 저장 몫(Kaggle 노트북과 같은 값)

$RepoRoot = Split-Path -Parent $PSScriptRoot
$Venv     = Join-Path $RepoRoot '.venv-xm'
$VenvPy   = Join-Path $Venv 'Scripts\python.exe'
$Helpers  = Join-Path $Venv 'xm_setup'
$ReqFile  = Join-Path $Helpers 'requirements.txt'
$Stamp    = Join-Path $Helpers 'installed.sha256'
$LogsDir  = Join-Path $RepoRoot 'logs'
$SmokeDir = Join-Path $LogsDir 'xm_ft_smoke'
$GinFile  = Join-Path $RepoRoot 'e2e\configs\finetune_parking_r3.gin'
$Finetune = Join-Path $RepoRoot 'e2e\finetune.py'
$CheckDs  = Join-Path $RepoRoot 'e2e\check_xm_dataset.py'
if (-not $XmDir) { $XmDir = Join-Path (Split-Path -Parent $RepoRoot) 'X-Mobility' }

# ---------------------------------------------------------------- 고정 패키지(전부 win_amd64 · cp312 휠 확인)
# Kaggle 노트북 고정 8개 · numpy 1.26.4 · wandb 0.19.4 는 X-Mobility 와 같은 판. X-Mobility 의 pandas 1.5.3(cp312 휠 없음) ·
# pyarrow 14.0.1.dev0(NGC 빌드)는 못 쓰므로 나머지(간접 의존 포함)는 2025-06-30 기준으로 푼 값.
# 아래 두 표시 줄 사이를 SETUP_LOCAL.md 의 리눅스(WSL2/Ubuntu) 명령이 그대로 잘라 씀 — 표시 줄을 지우지 말 것.
$Requirements = @'
# >>> xm-requirements
torch==2.7.1+cu128
torchvision==0.22.1+cu128
numpy==1.26.4
gin-config==0.5.0
einops==0.7.0
pytorch-lightning==2.5.0.post0
transformers==4.48.1
diffusers==0.29.2
huggingface-hub==0.25.2
timm==1.0.14
torcheval==0.0.7
torchmetrics==1.7.3
wandb==0.19.4
matplotlib==3.10.3
pandas==2.3.0
pyarrow==20.0.0
pillow==11.2.1
aiohappyeyeballs==2.6.1
aiohttp==3.12.13
aiosignal==1.3.2
annotated-types==0.7.0
attrs==25.3.0
certifi==2025.6.15
charset-normalizer==3.4.2
click==8.2.1
colorama==0.4.6
contourpy==1.3.2
cycler==0.12.1
docker-pycreds==0.4.0
filelock==3.18.0
fonttools==4.58.4
frozenlist==1.7.0
fsspec==2025.5.1
gitdb==4.0.12
gitpython==3.1.44
idna==3.10
importlib-metadata==8.7.0
jinja2==3.1.6
kiwisolver==1.4.8
lightning-utilities==0.14.3
markupsafe==3.0.2
mpmath==1.3.0
multidict==6.6.2
networkx==3.5
packaging==25.0
platformdirs==4.3.8
propcache==0.3.2
protobuf==5.29.5
psutil==7.0.0
pydantic==2.11.7
pydantic-core==2.33.2
pyparsing==3.2.3
python-dateutil==2.9.0.post0
pytz==2025.2
pyyaml==6.0.2
regex==2024.11.6
requests==2.32.4
safetensors==0.5.3
sentry-sdk==2.32.0
setproctitle==1.3.6
setuptools==80.9.0
six==1.17.0
smmap==5.0.2
sympy==1.14.0
tokenizers==0.21.2
tqdm==4.67.1
typing-extensions==4.14.0
typing-inspection==0.4.1
tzdata==2025.2
urllib3==2.5.0
yarl==1.20.1
zipp==3.23.0
# <<< xm-requirements
'@

# ---------------------------------------------------------------- 보조 파이썬(.venv-xm\xm_setup 에 씀)
# GPU · torch 빌드 · gloo · import 확인. 종료 코드: 2 CUDA 없음 · 3 아키텍처 없음 · 4 torch 판 다름 · 5 import 실패
$GpuCheckPy = @'
import os, socket, sys
want_torch, xm = sys.argv[1], sys.argv[2]
import torch
print(f"  python {sys.version.split()[0]} | torch {torch.__version__} | CUDA(빌드) {torch.version.cuda} | cuDNN {torch.backends.cudnn.version()}")
if torch.__version__ != want_torch:
    print(f"  torch 판이 {want_torch} 가 아님 — CPU 판이 깔렸거나 다른 판. .venv-xm 을 지우고 다시")
    sys.exit(4)
if not torch.cuda.is_available():
    print("  torch.cuda.is_available() = False — 드라이버 확인(nvidia-smi), 또는 CUDA_VISIBLE_DEVICES 확인")
    sys.exit(2)
cap = torch.cuda.get_device_capability(0)
arch = torch.cuda.get_arch_list()
print(f"  GPU {torch.cuda.get_device_name(0)} | compute capability {cap[0]}.{cap[1]} | GPU 수 {torch.cuda.device_count()}")
print(f"  torch 빌드 아키텍처: {' '.join(arch)}")
need = f"sm_{cap[0]}{cap[1]}"
if need not in arch:
    print(f"  {need} 이(가) 이 torch 빌드에 없음 — cu128 이상 빌드(torch 2.7+)가 필요")
    sys.exit(3)
if "sm_120" not in arch:
    print("  주의: sm_120 이 빌드에 없음(이 GPU 는 괜찮지만 RTX 50 계열엔 안 맞음)")
a = torch.randn(4096, 4096, device="cuda")
b = a @ a
with torch.autocast("cuda", dtype=torch.float16):
    c = a @ a
torch.cuda.synchronize()
free, total = torch.cuda.mem_get_info()
print(f"  matmul fp32 · fp16 OK ({c.dtype}) | VRAM 여유 {free / 2**30:.1f} / {total / 2**30:.1f} GB")
import torch.distributed as dist
s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
os.environ["MASTER_ADDR"], os.environ["MASTER_PORT"] = "127.0.0.1", str(port)
dist.init_process_group("gloo", rank=0, world_size=1)
t = torch.ones(8, device="cuda")
dist.all_reduce(t)
dist.barrier()
dist.destroy_process_group()
print(f"  gloo 프로세스 그룹(1개) · CUDA 텐서 all_reduce OK (Lightning DDP 와 같은 경로)")
try:
    sys.path.insert(0, xm)
    import gin, pytorch_lightning as pl, transformers, diffusers, timm, torchmetrics, wandb, pandas, pyarrow
    from model.dataset.isaac_sim_dataset import XMobilityIsaacSimDataModule  # noqa: F401
    from model.trainer import XMobilityTrainer  # noqa: F401
except Exception as e:  # noqa: BLE001
    print(f"  import 실패: {type(e).__name__}: {e}")
    sys.exit(5)
print(f"  import OK — lightning {pl.__version__} · transformers {transformers.__version__} · diffusers {diffusers.__version__}"
      f" · timm {timm.__version__} · torchmetrics {torchmetrics.__version__} · wandb {wandb.__version__} · pandas {pandas.__version__}")
'@

# HF 게이트 레포에서 ckpt 받기. 토큰은 HF_TOKEN 환경변수 또는 huggingface-cli login 으로 저장된 것.
# 종료 코드: 4 게이트(401 로그인 없음 · 403 약관 미동의/토큰 권한) · 5 레포 없음 · 6 그 밖의 HTTP · 7 네트워크
$HfDownloadPy = @'
import os, sys
from huggingface_hub import hf_hub_download
from huggingface_hub.utils import GatedRepoError, RepositoryNotFoundError, HfHubHTTPError, LocalEntryNotFoundError
repo, fname, outdir = sys.argv[1:4]
def status(e):
    r = getattr(e, "response", None)
    return getattr(r, "status_code", None)
try:
    p = hf_hub_download(repo, fname, local_dir=outdir, token=os.environ.get("HF_TOKEN") or None)
except GatedRepoError as e:
    print(f"  게이트 레포 거부(HTTP {status(e)}): 401 = 토큰 없음/틀림, 403 = 약관 미동의 또는 토큰 권한 부족")
    sys.exit(4)
except RepositoryNotFoundError as e:
    print(f"  레포 없음 또는 권한 없음(HTTP {status(e)}): {repo}")
    sys.exit(5)
except HfHubHTTPError as e:
    print(f"  HTTP 오류 {status(e)}: {e}")
    sys.exit(6)
except LocalEntryNotFoundError as e:
    print(f"  네트워크 오류(huggingface.co 에 못 닿음): {e}")
    sys.exit(7)
print(f"  {p} ({os.path.getsize(p) / 2**30:.2f} GB)")
'@

# run_meta.json 에서 필요한 값만 key=value 로(UTF-8). 없는 값은 None
$MetaReadPy = @'
import json, sys
m = json.load(open(sys.argv[1], encoding="utf-8"))
k = m.get("keys") or {}
out = dict(step_time_s=m.get("step_time_s"), steps_per_epoch=m.get("steps_per_epoch"),
           peak_gpu_mem_gb=m.get("peak_gpu_mem_gb"), world_size=m.get("world_size"),
           global_step=m.get("global_step"), train_sequences=m.get("train_sequences"),
           missing=len(k.get("missing", [])), unexpected_other=len(k.get("unexpected_other", [])),
           gin_extra="|".join(m.get("gin_extra") or []), dataset=m.get("dataset"), checkpoint=m.get("checkpoint"))
for key, v in out.items():
    print(f"{key}={v}")
'@

# ---------------------------------------------------------------- 보조 함수
function Write-Step([string]$Msg) { Write-Host ''; Write-Host "==> $Msg" -ForegroundColor Cyan }
function Write-Ok([string]$Msg) { Write-Host "    [OK] $Msg" -ForegroundColor Green }
function Write-Note([string]$Msg) { Write-Host "    $Msg" }
function Write-Caution([string]$Msg) { Write-Host "    [주의] $Msg" -ForegroundColor Yellow }
function Stop-Setup([string]$Msg) { throw $Msg }
function Assert-Exit([string]$What) {
    if ($LASTEXITCODE -ne 0) { Stop-Setup ('{0} 실패 (종료 코드 {1})' -f $What, $LASTEXITCODE) }
}
function Write-Utf8([string]$Path, [string]$Text) {
    [System.IO.File]::WriteAllText($Path, $Text, (New-Object System.Text.UTF8Encoding($false)))
}
# 출력하는 명령을 그대로 복사해 붙일 수 있게 인자를 PowerShell 작은따옴표로 감쌈
function Format-PsArg([string]$A) {
    if ($A -match '^[A-Za-z0-9_\-\.=/]+$') { return $A }
    return "'" + ($A -replace "'", "''") + "'"
}
function Format-PsCommand([string]$Exe, [string[]]$ArgList) {
    $parts = @('&', (Format-PsArg $Exe))
    foreach ($x in $ArgList) { $parts += (Format-PsArg $x) }
    return ($parts -join ' ')
}
function Read-RunMeta([string]$Path) {
    $h = @{}
    $lines = & $VenvPy (Join-Path $Helpers 'meta_read.py') $Path
    Assert-Exit 'run_meta.json 읽기'
    foreach ($line in @($lines)) {
        $kv = $line -split '=', 2
        if ($kv.Count -eq 2) { $h[$kv[0]] = $kv[1] }
    }
    return $h
}
function Test-Number([string]$S) {
    $d = 0.0
    return [double]::TryParse($S, [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$d)
}
function ConvertTo-Number([string]$S) {
    return [double]::Parse($S, [System.Globalization.CultureInfo]::InvariantCulture)
}
# 사용자가 준 상대 경로는 실행한 위치 기준 절대 경로로(아래에서 레포 폴더로 cd 하기 전에)
function Resolve-UserPath([string]$P) {
    if (-not $P) { return $P }
    return $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($P)
}

# ---------------------------------------------------------------- 본문
$Data = Resolve-UserPath $Data
$Zip = Resolve-UserPath $Zip
$CkptDir = Resolve-UserPath $CkptDir
$Ckpt = Resolve-UserPath $Ckpt
$XmDir = Resolve-UserPath $XmDir
$prevOutEnc = $null
$exitCode = 0
Push-Location $RepoRoot
try {
    # 파이썬 출력(한글)이 콘솔 · 파이프 어느 쪽으로 가도 깨지지 않게
    try { $prevOutEnc = [Console]::OutputEncoding; [Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false) } catch { $prevOutEnc = $null }
    $env:PYTHONUTF8 = '1'
    $env:PYTHONIOENCODING = 'utf-8'
    $env:PYTHONUNBUFFERED = '1'
    $env:PIP_DISABLE_PIP_VERSION_CHECK = '1'
    $env:HF_HUB_DISABLE_SYMLINKS_WARNING = '1'

    Write-Host ('X-Mobility 로컬 학습 준비 — 레포 {0}' -f $RepoRoot)
    foreach ($f in @($Finetune, $CheckDs, $GinFile)) {
        if (-not (Test-Path -LiteralPath $f)) { Stop-Setup ('레포 파일 없음: {0} — 이 스크립트는 레포의 e2e\ 안에서 실행' -f $f) }
    }

    # ---- 1. GPU · 드라이버 · RAM · 디스크
    Write-Step '1/9 GPU · 드라이버 · RAM · 디스크'
    $smi = Get-Command nvidia-smi -ErrorAction SilentlyContinue
    if ($smi) { $smiPath = $smi.Path } else { $smiPath = Join-Path $env:ProgramFiles 'NVIDIA Corporation\NVSMI\nvidia-smi.exe' }
    if (-not (Test-Path -LiteralPath $smiPath)) {
        Stop-Setup 'nvidia-smi 없음 — NVIDIA 드라이버를 먼저 설치(https://www.nvidia.com/drivers/, Game Ready 또는 Studio)'
    }
    $rows = & $smiPath --query-gpu=name,driver_version,memory.total,compute_cap --format=csv,noheader
    Assert-Exit 'nvidia-smi'
    $first = @($rows)[0]
    $parts = @($first -split ',' | ForEach-Object { $_.Trim() })
    $gpuName = $parts[0]; $drvText = $parts[1]; $vram = $parts[2]; $cc = $parts[3]
    Write-Note ('GPU {0} | 드라이버 {1} | VRAM {2} | compute capability {3}' -f $gpuName, $drvText, $vram, $cc)
    if (@($rows).Count -gt 1) { Write-Caution ('GPU {0}장 — 학습은 devices=auto(전부) 로 잡힘. 한 장만 쓰려면 $env:CUDA_VISIBLE_DEVICES=0' -f @($rows).Count) }
    $drv = $null
    try { $drv = [version]$drvText } catch { $drv = $null }
    if ($null -eq $drv) { Write-Caution ('드라이버 판을 못 읽음: {0}' -f $drvText) }
    elseif ($drv -lt $MinDriver) {
        Stop-Setup ('드라이버 {0} < {1}(CUDA 12.8 최소). 최신 Game Ready/Studio 드라이버로 올리고 다시' -f $drvText, $MinDriver)
    } else { Write-Ok ('드라이버 {0} >= {1} (CUDA 12.8 최소)' -f $drvText, $MinDriver) }
    if ($cc -and $cc -ne '12.0') { Write-Caution ('compute capability {0} — 이 스크립트는 12.0(RTX 50) 기준. cu128 빌드는 7.5~12.0 을 포함' -f $cc) }

    try {
        $ramGB = [math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory / 1GB, 1)
        Write-Note ('RAM {0} GB | DataLoader 워커 {1} (워커당 약 1.2 GB 추정: 파이썬+torch + train 데이터셋 사본)' -f $ramGB, $NumWorkers)
        if ($ramGB -lt 16) { Write-Caution 'RAM 16 GB 미만 — -NumWorkers 1 권장' }
    } catch { Write-Caution 'RAM 용량을 못 읽음 — 건너뜀' }
    try {
        $drive = (Get-Item -LiteralPath $RepoRoot).PSDrive
        $freeGB = [math]::Round($drive.Free / 1GB, 1)
        Write-Note ('레포 드라이브 {0}: 여유 {1} GB (필요 약 20 GB: 환경 ~6 · pip 캐시 ~3.3 · ckpt 1.4 · 데이터 ~1.6 · 시험/학습 ckpt 각 ~2)' -f $drive.Name, $freeGB)
        if ($freeGB -lt 20) { Write-Caution '여유 공간 20 GB 미만 — 중간에 디스크가 찰 수 있음' }
    } catch { Write-Caution '디스크 여유를 못 읽음 — 건너뜀' }
    # site-packages 안 가장 긴 상대 경로가 약 135자(diffusers · transformers .pyc) — Windows 기본 경로 한도 260자
    $spLen = (Join-Path $Venv 'Lib\site-packages\').Length
    if ($spLen + 135 -ge 260) {
        Write-Caution ('레포 경로가 깊어 설치 중 경로 260자 한도에 걸릴 수 있음(site-packages {0}자). 레포를 짧은 경로(예: C:\fd)로 옮기거나 LongPathsEnabled=1(관리자)' -f $spLen)
    }
    if ($RepoRoot -match 'OneDrive') { Write-Caution '레포가 OneDrive 폴더 안 — .venv-xm(약 6 GB) · 체크포인트가 동기화됨. OneDrive 밖으로 옮기는 것 권장' }

    # ---- 2. Python 3.12 · git
    Write-Step ('2/9 Python {0} · git' -f $PyVer)
    $basePy = $null
    $pyLauncher = Get-Command py -ErrorAction SilentlyContinue
    if ($pyLauncher) {
        $out = & $pyLauncher.Path "-$PyVer" -c 'import sys; print(sys.executable)'
        if ($LASTEXITCODE -eq 0 -and $out) { $basePy = (@($out)[-1]).Trim() }
    }
    if (-not $basePy) {
        Write-Host ''
        Write-Host ('    Python {0} 을(를) py 런처로 못 찾음. 설치 후 PowerShell 을 새로 열고 다시 실행:' -f $PyVer)
        Write-Host '      winget install -e --id Python.Python.3.12'
        Write-Host '    (다른 PC 의 3.14 는 이 환경에 못 씀 — numpy 1.26.4 · pandas · pyarrow 고정판에 cp314 휠이 없음)'
        Stop-Setup ('Python {0} 없음' -f $PyVer)
    }
    Write-Ok ('Python {0}: {1}' -f $PyVer, $basePy)
    $git = Get-Command git -ErrorAction SilentlyContinue
    if (-not $git) {
        Write-Host '    git 없음. 설치 후 PowerShell 을 새로 열고 다시 실행:'
        Write-Host '      winget install -e --id Git.Git'
        Stop-Setup 'git 없음'
    }
    $gitExe = $git.Path
    Write-Ok ('git: {0}' -f $gitExe)

    # ---- 3. venv
    Write-Step '3/9 가상환경 .venv-xm'
    if (-not (Test-Path -LiteralPath $VenvPy)) {
        & $basePy -m venv $Venv
        Assert-Exit 'venv 만들기'
        Write-Ok ('만듦: {0}' -f $Venv)
    } else { Write-Ok ('있음: {0}' -f $Venv) }
    $vv = & $VenvPy -c "import sys; print('{0}.{1}'.format(*sys.version_info))"
    Assert-Exit 'venv python 확인'
    if ((@($vv)[-1]).Trim() -ne $PyVer) {
        Stop-Setup ('.venv-xm 이 Python {0} 로 만들어져 있음 — 지우고 다시: Remove-Item -Recurse -Force ''{1}''' -f (@($vv)[-1]).Trim(), $Venv)
    }
    New-Item -ItemType Directory -Force -Path $Helpers | Out-Null
    Write-Utf8 (Join-Path $Venv '.gitignore') "*`n"          # .venv-xm 은 레포 .gitignore 에 없음 → 스스로 무시
    Write-Utf8 $ReqFile $Requirements
    Write-Utf8 (Join-Path $Helpers 'gpu_check.py') $GpuCheckPy
    Write-Utf8 (Join-Path $Helpers 'hf_download.py') $HfDownloadPy
    Write-Utf8 (Join-Path $Helpers 'meta_read.py') $MetaReadPy

    # ---- 4. 패키지
    Write-Step ('4/9 패키지 — torch {0}(약 3.3 GB 받음) · X-Mobility 고정판' -f $TorchVersion)
    $reqHash = (Get-FileHash -LiteralPath $ReqFile -Algorithm SHA256).Hash
    $done = $false
    if (Test-Path -LiteralPath $Stamp) { $done = ((Get-Content -LiteralPath $Stamp -Raw).Trim() -eq $reqHash) }
    if ($done) {
        Write-Ok '이미 설치됨(같은 고정 목록) — 건너뜀. 다시 깔려면 .venv-xm\xm_setup\installed.sha256 삭제'
    } else {
        & $VenvPy -m pip install --upgrade pip
        Assert-Exit 'pip 업그레이드'
        & $VenvPy -m pip install --timeout 60 --extra-index-url $TorchIndex -r $ReqFile
        Assert-Exit 'pip install'
        & $VenvPy -m pip check
        if ($LASTEXITCODE -ne 0) { Write-Caution 'pip check 가 의존성 경고를 냄(위 출력) — 학습엔 영향 없을 수 있음' }
        Write-Utf8 $Stamp $reqHash
        Write-Ok '설치 끝'
    }

    # ---- 5. X-Mobility
    Write-Step ('5/9 X-Mobility {0} → {1}' -f $XmRev.Substring(0, 7), $XmDir)
    if (-not (Test-Path -LiteralPath (Join-Path $XmDir 'train.py'))) {
        if (Test-Path -LiteralPath $XmDir) { Stop-Setup ('{0} 이(가) 있는데 train.py 가 없음 — 폴더를 옮기거나 -XmDir 지정' -f $XmDir) }
        & $gitExe clone $XmUrl $XmDir
        Assert-Exit 'git clone X-Mobility'
    }
    $head = & $gitExe -C $XmDir rev-parse HEAD
    Assert-Exit 'git rev-parse'
    if ((@($head)[-1]).Trim() -ne $XmRev) {
        & $gitExe -C $XmDir checkout -q $XmRev
        if ($LASTEXITCODE -ne 0) {
            & $gitExe -C $XmDir fetch origin
            Assert-Exit 'git fetch'
            & $gitExe -C $XmDir checkout -q $XmRev
            Assert-Exit 'git checkout 5837aea'
        }
    }
    $dirty = & $gitExe -C $XmDir status --porcelain --untracked-files=no   # finetune.py 의 -dirty 판정(git diff HEAD)과 같은 기준
    Assert-Exit 'git status'
    if ($dirty) { Write-Caution 'X-Mobility 작업 트리에 바뀐 파일이 있음(run_meta 의 xm_rev 에 -dirty 로 남음)' }
    Write-Ok ('HEAD {0}' -f $XmRev.Substring(0, 12))

    # ---- 6. CUDA · sm_120 · gloo · import
    Write-Step '6/9 torch CUDA · sm_120 · gloo · import 확인'
    & $VenvPy (Join-Path $Helpers 'gpu_check.py') $TorchVersion $XmDir
    $code = $LASTEXITCODE
    if ($code -eq 3) { Stop-Setup 'torch 빌드에 이 GPU 아키텍처가 없음 — cu128 빌드가 아님. .venv-xm 삭제 후 다시' }
    if ($code -eq 4) { Stop-Setup ('torch 가 {0} 가 아님 — .venv-xm 삭제 후 다시' -f $TorchVersion) }
    if ($code -ne 0) { Stop-Setup ('GPU/import 확인 실패 (종료 코드 {0}) — 위 메시지 참고' -f $code) }
    Write-Ok 'CUDA · sm_120 · gloo · import'

    # ---- 7. 사전학습 ckpt
    Write-Step ('7/9 사전학습 ckpt {0}/{1}' -f $CkptRepo, $CkptFile)
    if ($Ckpt) {
        if (-not (Test-Path -LiteralPath $Ckpt)) { Stop-Setup ('-Ckpt 파일 없음: {0}' -f $Ckpt) }
        $Ckpt = (Resolve-Path -LiteralPath $Ckpt).Path
        Write-Ok ('지정한 파일: {0}' -f $Ckpt)
    } else {
        $Ckpt = Join-Path $CkptDir $CkptFile
        if ((Test-Path -LiteralPath $Ckpt) -and ((Get-Item -LiteralPath $Ckpt).Length -gt 1GB)) {
            Write-Ok ('있음: {0} ({1:N2} GB)' -f $Ckpt, ((Get-Item -LiteralPath $Ckpt).Length / 1GB))
        } else {
            if (-not $env:HF_TOKEN) { Write-Note 'HF_TOKEN 환경변수 없음 — huggingface-cli login 으로 저장한 토큰을 씀(없으면 401)' }
            New-Item -ItemType Directory -Force -Path $CkptDir | Out-Null
            & $VenvPy (Join-Path $Helpers 'hf_download.py') $CkptRepo $CkptFile $CkptDir
            $code = $LASTEXITCODE
            if ($code -ne 0) {
                Write-Host ''
                Write-Host '    해결: (1) https://huggingface.co/nvidia/X-Mobility 에 로그인 → 약관(라이선스) 동의'
                Write-Host '          (2) Settings → Access Tokens → 토큰 종류 Read 로 발급 (fine-grained 면 "public gated repos" 읽기 권한 체크)'
                Write-Host '          (3) $env:HF_TOKEN = ''hf_...'' 후 다시 실행, 또는:'
                Write-Host ('              & ''{0}'' login' -f (Join-Path $Venv 'Scripts\huggingface-cli.exe'))
                Write-Host '          이미 받은 파일이 있으면: -Ckpt <경로>'
                Stop-Setup ('ckpt 받기 실패 (종료 코드 {0})' -f $code)
            }
        }
    }

    # ---- 8. 데이터 · 점검
    Write-Step '8/9 데이터 xm_v1 · 점검'
    if (-not (Test-Path -LiteralPath (Join-Path $Data 'train'))) {
        $dest = Split-Path -Parent $Data
        if (Test-Path -LiteralPath $Zip) {
            Write-Note ('{0} 없음 → {1} 를 {2} 에 풂' -f $Data, $Zip, $dest)
            New-Item -ItemType Directory -Force -Path $dest | Out-Null
            $tar = Get-Command tar.exe -ErrorAction SilentlyContinue
            if ($tar) {
                & $tar.Path -xf $Zip -C $dest
                Assert-Exit 'zip 풀기(tar)'
            } else {
                Expand-Archive -LiteralPath $Zip -DestinationPath $dest -Force
            }
        }
        if (-not (Test-Path -LiteralPath (Join-Path $Data 'train')) -and (Test-Path -LiteralPath $dest)) {
            # zip 안 폴더 이름이 다르면 train/val/test 가 함께 있는 폴더를 찾음
            $found = Get-ChildItem -LiteralPath $dest -Directory -Recurse -Depth 3 -ErrorAction SilentlyContinue |
                Where-Object { (Test-Path -LiteralPath (Join-Path $_.FullName 'train')) -and (Test-Path -LiteralPath (Join-Path $_.FullName 'val')) -and (Test-Path -LiteralPath (Join-Path $_.FullName 'test')) } |
                Select-Object -First 1
            if ($found) { $Data = $found.FullName; Write-Note ('데이터 루트를 찾음: {0}' -f $Data) }
        }
        if (-not (Test-Path -LiteralPath (Join-Path $Data 'train'))) {
            Stop-Setup ('데이터 없음: {0} — 02_xm_v1.zip 을 받아 -Zip <zip 경로> 또는 -Data <train/val/test 가 있는 폴더> 로 지정' -f $Data)
        }
    }
    $Data = (Resolve-Path -LiteralPath $Data).Path
    foreach ($sp in @('train', 'val', 'test')) {
        if (-not (Test-Path -LiteralPath (Join-Path $Data $sp))) { Stop-Setup ('{0}\{1} 없음' -f $Data, $sp) }
    }
    Write-Ok ('데이터 루트: {0}' -f $Data)
    New-Item -ItemType Directory -Force -Path $LogsDir | Out-Null
    if ($SkipDataCheck) { Write-Note '-SkipDataCheck — 점검 건너뜀' }
    else {
        & $VenvPy $CheckDs $Data --image-stride 20 --json (Join-Path $LogsDir 'xm_v1_check.json')
        Assert-Exit 'check_xm_dataset.py (오류 있음 — 위 출력)'
        Write-Ok ('점검 통과 — 요약 {0}' -f (Join-Path $LogsDir 'xm_v1_check.json'))
    }

    # ---- 9. 40배치 시험 → 스텝 시간 → 본 학습 명령
    Write-Step ('9/9 시험 — 1에폭 {0}배치(+검증 {1}배치), 키 대조 · VRAM · 스텝 시간' -f $SmokeBatches, $SmokeValBatches)
    $ginArgs = @('--gin', ('XMobilityIsaacSimDataModule.num_workers={0}' -f $NumWorkers), '--gin', ('XMobilityIsaacSimDataModule.batch_size={0}' -f $BatchSize))
    foreach ($g in $ExtraGin) { if ($g) { $ginArgs += @('--gin', $g) } }
    $baseArgs = @($Finetune, '--xm-repo', $XmDir, '-c', $GinFile, '-d', $Data, '-p', $Ckpt, '--wandb', 'offline', '--save-top-k', '1', '--accumulate-grad-batches', "$Accum") + $ExtraArgs
    # finetune.py 는 smoke 의 --gin 목록을 run_meta.json 의 gin_extra 에 남김 → 설정이 바뀌면 다시 돌림
    $smokeGin = @('--gin', 'NUM_EPOCHS=1') + $ginArgs
    $smokeArgs = $baseArgs + @('-o', $SmokeDir, '--limit-train-batches', "$SmokeBatches", '--limit-val-batches', "$SmokeValBatches", '--no-test') + $smokeGin
    $wantGinExtra = (@($smokeGin | Where-Object { $_ -ne '--gin' }) -join '|')
    $smokeMeta = Join-Path $SmokeDir 'run_meta.json'

    $reuse = $false
    if ((Test-Path -LiteralPath $smokeMeta) -and -not $ForceSmoke) {
        $m0 = Read-RunMeta $smokeMeta
        # 같은 설정(--gin) · 같은 데이터 · 같은 ckpt 일 때만 재사용(데이터가 바뀌면 steps_per_epoch 가 달라짐)
        if ((Test-Number $m0['step_time_s']) -and ($m0['gin_extra'] -eq $wantGinExtra) -and ($m0['dataset'] -eq $Data) -and ($m0['checkpoint'] -eq $Ckpt)) { $reuse = $true }
    }
    if ($reuse) {
        Write-Ok ('이전 시험 결과를 씀: {0} (다시 돌리려면 -ForceSmoke)' -f $smokeMeta)
    } elseif ($SkipSmoke) {
        Write-Note '-SkipSmoke — 시험 건너뜀'
    } else {
        if (Test-Path -LiteralPath $SmokeDir) { Remove-Item -LiteralPath $SmokeDir -Recurse -Force }
        Write-Note 'DINOv2(facebook/dinov2-small) 를 처음 한 번 HF 에서 받음. 데이터 로딩(약 1~3 분) 뒤 학습 시작'
        Write-Note (Format-PsCommand $VenvPy $smokeArgs)
        $t0 = Get-Date
        & $VenvPy @smokeArgs
        $code = $LASTEXITCODE
        if ($code -ne 0) {
            Write-Host ''
            Write-Host '    자주 나는 원인 (SETUP_LOCAL.md 문제 해결 표):'
            Write-Host '      CUDA out of memory    → -BatchSize 1 -Accum 8 -ForceSmoke'
            Write-Host '                              또는 -ExtraGin "finetune.freeze_prefixes=[''model.observation_encoder.image_encoder.'']" (DINOv2 고정)'
            Write-Host '      WinError 1455 / MemoryError → -NumWorkers 1 -ForceSmoke, 페이지 파일 "시스템이 관리" 확인'
            Write-Host '      missing 키 있음        → gin 설정이 사전학습 때와 다름(finetune.py 출력 확인)'
            Stop-Setup ('시험 실패 (종료 코드 {0})' -f $code)
        }
        Write-Ok ('시험 끝 — 전체 {0:N1} 분' -f ((Get-Date) - $t0).TotalMinutes)
        # 시험 ckpt(약 1 GB × 2)는 쓸 데가 없어 지움. run_meta.json · gin_operative.txt · wandb\ 는 남김
        $smokeCk = Join-Path $SmokeDir 'checkpoints'
        if (Test-Path -LiteralPath $smokeCk) { Remove-Item -LiteralPath $smokeCk -Recurse -Force; Write-Note '시험 체크포인트 삭제(디스크 약 2 GB)' }
    }

    if (-not (Test-Path -LiteralPath $smokeMeta)) {
        Write-Caution '시험 결과(run_meta.json)가 없어 에폭 수를 못 정함 — 아래 명령의 NUM_EPOCHS 는 gin 기본값'
        $epochs = $MaxEpochs
    } else {
        $m = Read-RunMeta $smokeMeta
        if (-not (Test-Number $m['step_time_s']) -or -not (Test-Number $m['steps_per_epoch'])) {
            Stop-Setup ('run_meta.json 에 step_time_s/steps_per_epoch 없음: {0}' -f $smokeMeta)
        }
        $st = ConvertTo-Number $m['step_time_s']
        $spe = ConvertTo-Number $m['steps_per_epoch']
        $perEpochS = $spe * $st * $EpochOverhead
        if ($perEpochS -le 0) { Stop-Setup ('에폭 시간 계산 불가(step_time_s={0}, steps_per_epoch={1})' -f $st, $spe) }
        $budgetS = $HoursBudget * 3600
        $epochs = [int][math]::Floor(($budgetS - $MarginS) / $perEpochS)
        if ($epochs -gt $MaxEpochs) { $epochs = $MaxEpochs }
        if ($epochs -lt 1) { $epochs = 1 }
        Write-Host ''
        Write-Host '    ---- 시험 결과 (run_meta.json) ----'
        Write-Note ('step_time_s      {0:N3} s (데이터 로딩 포함 중앙값)' -f $st)
        Write-Note ('steps_per_epoch  {0} (train 시퀀스 {1} · world_size {2})' -f $spe, $m['train_sequences'], $m['world_size'])
        Write-Note ('peak_gpu_mem_gb  {0}' -f $m['peak_gpu_mem_gb'])
        Write-Note ('키 대조          missing {0} · 예상 밖 unexpected {1}' -f $m['missing'], $m['unexpected_other'])
        Write-Note ('에폭당 예상      {0:N2} 시간 (= 스텝 × 스텝 시간 × {1}, 검증 · 저장 몫 포함)' -f ($perEpochS / 3600), $EpochOverhead)
        if ((Test-Number $m['peak_gpu_mem_gb']) -and ((ConvertTo-Number $m['peak_gpu_mem_gb']) -gt 14.5)) {
            Write-Caution 'VRAM 최대치가 14.5 GB 넘음 — 본 학습 중 OOM 가능. -BatchSize 1 -Accum 8 또는 DINOv2 고정 검토'
        }
        if ($m['missing'] -ne '0') { Write-Caution 'missing 키가 있음 — finetune.py 는 원래 여기서 멈춤. 출력 확인' }
        if (($perEpochS * $epochs + $MarginS) -gt $budgetS) {
            Write-Caution ('1에폭도 예산 {0} 시간에 안 들어감 — --max-time 에서 끊김(lr 스케줄 미완). -HoursBudget 을 늘리는 것 권장' -f $HoursBudget)
        }
        Write-Note ('→ 예산 {0} 시간(여유 {1} 분 뺌) · 상한 {2} 에폭 → NUM_EPOCHS={3} (예상 {4:N1} 시간)' -f $HoursBudget, ($MarginS / 60), $MaxEpochs, $epochs, ($perEpochS * $epochs / 3600))
    }

    $ts = [TimeSpan]::FromHours($HoursBudget)
    $maxTime = '{0:00}:{1:00}:{2:00}:00' -f $ts.Days, $ts.Hours, $ts.Minutes
    $fullOut = Join-Path $RepoRoot ('data\checkpoints\xm_ft_local_e{0}' -f $epochs)
    $fullArgs = $baseArgs + @('-o', $fullOut, '--gin', ('NUM_EPOCHS={0}' -f $epochs), '--max-time', $maxTime) + $ginArgs
    Write-Host ''
    Write-Host '==> 본 학습 명령 (경로가 모두 절대 경로 — PowerShell 에 그대로 붙여 넣기)' -ForegroundColor Cyan
    Write-Host ''
    Write-Host (Format-PsCommand $VenvPy $fullArgs) -ForegroundColor White
    Write-Host ''
    Write-Note '돌리기 전: 절전 끄기(powercfg /change standby-timeout-ac 0), Windows 업데이트 재시작 미루기. 콘솔 창은 열어 둠.'
    Write-Note ('결과: {0}\run_meta.json · checkpoints\ (같은 폴더에 ckpt 가 있으면 finetune.py 가 멈춤 → 이름을 바꿀 것)' -f $fullOut)
}
catch {
    Write-Host ''
    Write-Host ('[중단] {0}' -f $_.Exception.Message) -ForegroundColor Red
    $exitCode = 1
}
finally {
    Pop-Location
    if ($null -ne $prevOutEnc) { try { [Console]::OutputEncoding = $prevOutEnc } catch { } }
}
exit $exitCode
