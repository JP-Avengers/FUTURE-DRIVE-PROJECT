#requires -Version 5.1
<#
.SYNOPSIS
  집 PC 이어 돌리기 (11.11, E): 지금 학습이 끝나기를 기다림 → 결과 업로드 → 다음 run 시험 · 본 학습 → 그 결과도 업로드.
  밤 · 수업 중에 끝나도 GPU 가 놀지 않게. 사람이 보던 시험 확인(missing 0 · 실행 ≠ 교사 > 0 · 에폭 수)은 setup_windows.ps1 -Launch 가 대신 함.

.DESCRIPTION
  -Next (HANDOFF_HOME_11_10.md 6절과 같은 이름)
    6A  E5 = v1.4 + aw30 + 실행 명령 입력 + 직전 명령 드롭아웃 0.5, 10 에폭 → xm_ft_v1.4_aw30_exec_pad05_e10
    6B  E6 = v1.5 + aw30 + 실행 명령 입력, 7 에폭 → xm_ft_v1.5_aw30_exec_e7 (dataset-08-xm-v1-5 가 없으면 받음)
  기다리는 run 이 실패(ckpt 없음)하면 업로드 없이 다음 run 으로 감(-StopOnFail 이면 멈춤).
  로그: logs\home_chain_<시각>.log (Start-Transcript).

.EXAMPLE
  cd C:\fd\FUTURE-DRIVE-PROJECT; git pull; Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
  .\e2e\home_chain.ps1 -Next 6A                      # E4 를 기다렸다가 E5
  .\e2e\home_chain.ps1 -Next 6B -Wait ''             # 기다릴 것 없이 바로 E6
#>
param(
    [Parameter(Mandatory = $true)][ValidateSet('6A', '6B')][string]$Next,
    [string]$Wait = 'data\checkpoints\xm_ft_v1.4_aw30_exec_e10',   # 끝나기를 기다릴 run 폴더('' = 안 기다림)
    [string]$WaitData = 'C:\fd\data\xm_v1_4',                      # 그 run 의 데이터(업로드 전 test 평가용)
    [string]$DataRoot = 'C:\fd\data',
    [string]$Ckpt = "$HOME\Downloads\xm_pretrained_from_seed_try2\pretrained\x_mobility-nav2-semantic_action_path.ckpt",
    [int]$PollMin = 5,
    [switch]$StopOnFail
)
$ErrorActionPreference = 'Stop'
$RepoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $RepoRoot
$Py = Join-Path $RepoRoot '.venv-xm\Scripts\python.exe'
New-Item -ItemType Directory -Force -Path (Join-Path $RepoRoot 'logs') | Out-Null
Start-Transcript -Path (Join-Path $RepoRoot ('logs\home_chain_{0:MMdd_HHmm}.log' -f (Get-Date))) | Out-Null
& powercfg /change standby-timeout-ac 0

function Say([string]$m, [string]$c = 'Cyan') { Write-Host ('[{0:MM-dd HH:mm}] {1}' -f (Get-Date), $m) -ForegroundColor $c }
function Test-Training([string]$name) {
    @(Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
      Where-Object { $_.CommandLine -like '*finetune.py*' -and $_.CommandLine -like "*$name*" }).Count -gt 0
}
function Test-Done([string]$run) {   # best ckpt 또는 last.ckpt 가 있으면 끝난 것
    $meta = Join-Path $run 'run_meta.json'
    if (Test-Path -LiteralPath $meta) {
        $b = (Get-Content -LiteralPath $meta -Raw -Encoding UTF8 | ConvertFrom-Json).best_model_path
        if ($b -and (Test-Path -LiteralPath $b)) { return $true }
    }
    return (Test-Path -LiteralPath (Join-Path $run 'checkpoints\last.ckpt'))
}
function Upload([string]$run, [string]$data) {
    Say "업로드: $run"
    try {   # 업로드가 실패해도 이어 돌리기는 계속(다음 학습이 더 중요)
        & (Join-Path $PSScriptRoot 'home_after_train.ps1') -Run $run -Data $data
        Say ('올림 — 노트북에 "{0} 올림"' -f (Split-Path -Leaf $run)) 'Green'
    } catch { Say ("업로드 실패: {0} — 나중에 손으로: .\e2e\home_after_train.ps1 -Run {1} -Data {2}" -f $_.Exception.Message, $run, $data) 'Yellow' }
}

try {
    if (-not (Test-Path -LiteralPath $Ckpt)) { throw "사전학습 ckpt 없음: $Ckpt (-Ckpt 로 지정)" }

    # 1. 지금 학습 기다리기 → 올리기
    if ($Wait) {
        $waitRun = if ([System.IO.Path]::IsPathRooted($Wait)) { $Wait } else { Join-Path $RepoRoot $Wait }
        $name = Split-Path -Leaf $waitRun
        while (Test-Training $name) { Say "학습 중: $name — $PollMin 분 뒤 다시 봄" 'DarkGray'; Start-Sleep -Seconds ($PollMin * 60) }
        if (Test-Done $waitRun) { Upload $waitRun $WaitData }
        else {
            Say "$name 이 ckpt 없이 끝남(실패 · 안 돌았음) — 학습 창의 마지막 30줄을 노트북에" 'Red'
            if ($StopOnFail) { throw '-StopOnFail' }
        }
    }

    # 2. 다음 run
    # 해시 테이블 스플랫(이름 있는 인자). -ForceSmoke: 시험 재사용 검사가 ExtraArgs(드롭아웃 등)를 안 봄
    $a2 = @{ Ckpt = $Ckpt; ExtraGin = @('XMobilityLoss.action_weight=30'); ForceSmoke = $true; Launch = $true }
    if ($Next -eq '6A') {
        $data = Join-Path $DataRoot 'xm_v1_4'
        $out = 'data\checkpoints\xm_ft_v1.4_aw30_exec_pad05_e10'
        $a2 += @{ ExtraArgs = @('--prev-action-source', 'exec', '--prev-action-dropout', '0.5', '--save-top-k', '3'); MaxEpochs = 10; HoursBudget = 14 }
    } else {
        $data = Join-Path $DataRoot 'xm_v1_5'
        if (-not (Test-Path -LiteralPath (Join-Path $data 'train'))) {
            Say 'dataset-08-xm-v1-5 받기'
            & $Py -m kaggle datasets download henryseo711/dataset-08-xm-v1-5 -p $DataRoot --unzip
            if ($LASTEXITCODE -ne 0) { throw 'v1.5 다운로드 실패 — 노트북이 아직 안 만들었거나 Kaggle 토큰' }
            if (-not (Test-Path -LiteralPath (Join-Path $data 'train'))) {
                $t = Get-ChildItem -LiteralPath $DataRoot -Recurse -Directory -Filter train -Depth 3 | Where-Object { $_.FullName -like '*xm_v1_5*' } | Select-Object -First 1
                if (-not $t) { throw "v1.5 train 폴더를 못 찾음($DataRoot)" }
                $data = $t.Parent.FullName
            }
        }
        $out = 'data\checkpoints\xm_ft_v1.5_aw30_exec_e7'
        $a2 += @{ ExtraArgs = @('--prev-action-source', 'exec', '--save-top-k', '3'); MaxEpochs = 7; HoursBudget = 16 }
    }
    Say "다음: $Next → $out (시험 → 확인 → 본 학습)"
    $a2.Data = $data; $a2.OutDir = $out
    & (Join-Path $PSScriptRoot 'setup_windows.ps1') @a2
    if ($LASTEXITCODE -ne 0) { throw "setup_windows.ps1 -Launch 실패($LASTEXITCODE) — 위 [중단] 줄 확인" }

    # 3. 그 결과도 올리기
    Upload (Join-Path $RepoRoot $out) $data
    Say '이어 돌리기 끝' 'Green'
}
catch { Say ("[중단] " + $_.Exception.Message) 'Red'; Stop-Transcript | Out-Null; exit 1 }
Stop-Transcript | Out-Null
