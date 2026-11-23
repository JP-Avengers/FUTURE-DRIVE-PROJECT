<#
.SYNOPSIS
  집 PC 학습이 끝난 뒤 한 줄: 오프라인 평가(test) → 학습 곡선 → 결과 파일 · best ckpt 를 비공개 Kaggle 데이터셋으로 올림 (E)

.DESCRIPTION
  노트북은 집 PC 에 직접 접근하지 못하므로, 결과를 Kaggle 비공개 데이터셋(henryseo711/fd-xm-home-<run>)으로 올려 받는다.
  올리는 것: run_meta.json · gin_operative.txt · metrics.csv · curves.md(.png) · report_tables.md · eval_test 의 summary.json ·
  frames.csv · run_meta.json · best ckpt(<run>_best.ckpt, 약 1 GB). 같은 run 을 다시 올리면 새 판(version)으로 바뀜.
  Kaggle 토큰: $HOME\.kaggle\access_token(또는 kaggle.json) — SETUP_LOCAL.md "챙길 것" 3번.

.EXAMPLE
  .\e2e\home_after_train.ps1 -Run data\checkpoints\xm_ft_v0.1_aw30_pad05_e10 -Data C:\fd\data\xm_v1
.EXAMPLE
  .\e2e\home_after_train.ps1 -Run data\checkpoints\xm_ft_v0.1_aw30_pad05_e10 -Data C:\fd\data\xm_v1 -NoCkpt   # 결과만(빠름)
#>
param(
    [Parameter(Mandatory = $true)][string]$Run,     # 학습 출력 폴더(run_meta.json 이 있는 곳)
    [Parameter(Mandatory = $true)][string]$Data,    # 학습에 쓴 데이터 폴더(train/val/test 가 바로 아래) — test 평가용
    [string]$XmDir = '',                            # X-Mobility 레포(기본: 레포 옆 ..\X-Mobility)
    [string]$KaggleUser = '',                       # 기본: ~\.kaggle\kaggle.json 의 username, 없으면 henryseo711
    [switch]$SkipEval,
    [switch]$SkipUpload,
    [switch]$NoCkpt                                 # ckpt 는 안 올림(결과 파일만)
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 1.0

$RepoRoot = Split-Path -Parent $PSScriptRoot
$Py = Join-Path $RepoRoot '.venv-xm\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $Py)) { throw ".venv-xm 이 없음 — 먼저 e2e\setup_windows.ps1" }
if (-not $XmDir) { $XmDir = Join-Path (Split-Path -Parent $RepoRoot) 'X-Mobility' }

Push-Location $RepoRoot
try {
    $Run = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Run)
    $Data = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Data)
    $XmDir = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($XmDir)
    $meta = Join-Path $Run 'run_meta.json'
    if (-not (Test-Path -LiteralPath $meta)) { throw "run_meta.json 없음: $meta" }
    $rm = Get-Content -LiteralPath $meta -Raw -Encoding UTF8 | ConvertFrom-Json
    $best = $rm.best_model_path
    if (-not $best -or -not (Test-Path -LiteralPath $best)) {
        $last = Join-Path $Run 'checkpoints\last.ckpt'
        if (Test-Path -LiteralPath $last) {
            Write-Host "best_model_path 가 없어 last.ckpt 를 씀 — 학습이 중간에 끝났는지 확인" -ForegroundColor Yellow
            $best = $last
        } else { throw "ckpt 없음 — 학습이 끝나지 않았거나 실패(로그 확인)" }
    }
    $sha = (Get-FileHash -Algorithm SHA256 -LiteralPath $best).Hash.ToLower().Substring(0, 12)
    Write-Host ("best: {0} (sha12 {1})" -f $best, $sha)
    $env:PYTHONIOENCODING = 'utf-8'

    $evalDir = Join-Path $Run 'eval_test'
    if (-not $SkipEval -and -not (Test-Path -LiteralPath (Join-Path $evalDir 'summary.json'))) {
        Write-Host "[1/3] 오프라인 평가(test, 직전 예측 입력 = 배포 노드와 같음) — 5–15 분" -ForegroundColor Cyan
        & $Py e2e\eval_offline.py --xm-repo $XmDir -c e2e\configs\finetune_parking_r3.gin -d $Data --split test -p $best -o $evalDir
        if ($LASTEXITCODE -ne 0) { throw "eval_offline 실패($LASTEXITCODE)" }
    } else { Write-Host "[1/3] 평가 건너뜀(이미 있음 또는 -SkipEval)" }

    Write-Host "[2/3] 학습 곡선(curves.md, matplotlib 있으면 curves.png)" -ForegroundColor Cyan
    & $Py e2e\plot_training.py $Run
    if ($SkipUpload) { Write-Host "업로드 건너뜀(-SkipUpload)"; return }

    Write-Host "[3/3] Kaggle 비공개 데이터셋으로 올림" -ForegroundColor Cyan
    & $Py -m kaggle --version *> $null
    if ($LASTEXITCODE -ne 0) {
        & $Py -m pip install -q kaggle
        if ($LASTEXITCODE -ne 0) { throw "kaggle CLI 설치 실패" }
    }
    if (-not $KaggleUser) {
        $kj = Join-Path $HOME '.kaggle\kaggle.json'
        if (Test-Path -LiteralPath $kj) { $KaggleUser = (Get-Content -LiteralPath $kj -Raw | ConvertFrom-Json).username }
        if (-not $KaggleUser) { $KaggleUser = 'henryseo711' }
    }
    $name = Split-Path -Leaf $Run
    $slug = ('fd-xm-home-' + ($name.ToLower() -replace '[^a-z0-9]+', '-')).Trim('-')
    if ($slug.Length -gt 50) { $slug = $slug.Substring(0, 50).Trim('-') }
    $stage = Join-Path $env:TEMP ('fd_upload_' + $slug)
    if (Test-Path -LiteralPath $stage) { Remove-Item -Recurse -Force -LiteralPath $stage }
    New-Item -ItemType Directory -Path $stage | Out-Null
    $files = [ordered]@{
        'run_meta.json' = 'run_meta.json'; 'gin_operative.txt' = 'gin_operative.txt'; 'csv\metrics.csv' = 'metrics.csv'
        'curves.md' = 'curves.md'; 'curves.png' = 'curves.png'; 'report_tables.md' = 'report_tables.md'
        'eval_test\summary.json' = 'eval_test_summary.json'; 'eval_test\frames.csv' = 'eval_test_frames.csv'
        'eval_test\run_meta.json' = 'eval_test_run_meta.json'
    }
    foreach ($k in $files.Keys) {
        $src = Join-Path $Run $k
        if (Test-Path -LiteralPath $src) { Copy-Item -LiteralPath $src -Destination (Join-Path $stage $files[$k]) }
    }
    if (-not $NoCkpt) {
        $dst = Join-Path $stage ($name + '_best.ckpt')
        try { New-Item -ItemType HardLink -Path $dst -Target $best -ErrorAction Stop | Out-Null }
        catch { Copy-Item -LiteralPath $best -Destination $dst }
    }
    $metaObj = [ordered]@{
        title = $slug; id = "$KaggleUser/$slug"; licenses = @(@{ name = 'other' })
        description = "E home-PC run $name, best ckpt sha12 $sha"
    }
    [System.IO.File]::WriteAllText((Join-Path $stage 'dataset-metadata.json'), ($metaObj | ConvertTo-Json -Depth 4),
        (New-Object System.Text.UTF8Encoding($false)))
    & $Py -m kaggle datasets create -p $stage
    if ($LASTEXITCODE -ne 0) {
        Write-Host "새로 만들기 실패 — 이미 있으면 새 판으로 올림" -ForegroundColor Yellow
        & $Py -m kaggle datasets version -p $stage -m ("update " + $name)
        if ($LASTEXITCODE -ne 0) { throw "업로드 실패 — Kaggle 토큰($HOME\.kaggle) 확인" }
    }
    Write-Host ("올림: https://www.kaggle.com/datasets/{0}/{1} (best sha12 {2}) — 노트북 쪽에 이 run 이름을 알려 주세요" -f $KaggleUser, $slug, $sha) -ForegroundColor Green
}
finally { Pop-Location }
