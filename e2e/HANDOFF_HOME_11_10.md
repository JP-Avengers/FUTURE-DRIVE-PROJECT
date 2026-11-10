# 집 PC 인수인계 (11.10 밤 → 집 도착 후)

노트북에서 11.10에 한 일과 집 PC(RTX 5060 Ti 16 GB, `C:\fd\FUTURE-DRIVE-PROJECT`)에서 할 일을 정리했습니다. 집 PC의 Claude Code에 이 파일을 그대로 주면 이어서 진행할 수 있습니다.

## 0. 지금 상황 (한 줄씩)

- **D 노드 시험 통과**: v0.1, 하네스 그대로 3/3 통과.
- **하네스 고침** (`3f85478`): 씬 준비는 카메라 영상이 올 때까지 기다리고, 노드 python 기본값은 `~/xmobility_test/venv`, 끝의 스키마 검사는 이제 소리 없이 빠지지 않음.
- **D 폐루프 30회를 2 m 기준으로 다시 셈**: 공개 모델 1/30, 파인튜닝 24 · 22 · 22. 2 m 규칙으로는 파인튜닝이 앞섬(`PLAN.md` 11.10 저녁 절).
- **장애물 진단 결론**: 원인은 **직전 명령 되먹임**이고, 좌우 평균이 아님(`results/obstacle_diag_11_10/README.md` 11.10 밤 절).
  - aw30에 교사 명령을 입력으로 주면 중앙 장애물을 교사의 0.83–0.85만큼 피함.
  - 자기 예측을 입력으로 받는 기본 평가에서는 0.18–0.23만큼만 피함(거의 직진).
  - 같은 쪽 피하기 재녹화는 필요 없음. 처방은 드롭아웃 · 실행 명령 입력 · DAgger.
- **새 학습 데이터 v1.4**: Kaggle 비공개 데이터셋 `henryseo711/dataset-07-xm-v1-4`, 1.3 GB.
  - 구성: D `07_xm_v3_exec` + v1.1 맞바꿈, train에 잡음 교사 4,918행과 DAgger 5,303행을 더함. train 68,441행, 전체 113파일.
  - 점검: `check_xm_dataset` 오류 0 · 경고 0. 113파일 모두 `exec_command` 열이 있고, train NaN은 0.
- **Kaggle GPU**: 2.35 h 남음, 갱신은 **11.14(토) 09:00 KST**. 그 전에는 Kaggle 학습이 불가능하므로 학습은 집 PC에서만 함.

## 1. E1 확인 · 업로드 (도착하자마자)

E1은 aw30 + 직전 명령 드롭아웃 0.5, xm_v1, 10에폭입니다. 11.9 밤에 띄웠고 14 h 상한이라 끝나 있어야 합니다.

```powershell
cd C:\fd\FUTURE-DRIVE-PROJECT
git pull
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
dir data\checkpoints\xm_ft_v0.1_aw30_pad05_e10              # run_meta.json · checkpoints\*.ckpt 가 있으면 끝남
```

- **끝났으면** 아래를 실행합니다. 오프라인 test 평가 → 학습 곡선 → Kaggle `henryseo711/fd-xm-home-xm-ft-v0-1-aw30-pad05-e10` 업로드까지 하고, 20–30분 걸립니다.
  ```powershell
  .\e2e\home_after_train.ps1 -Run data\checkpoints\xm_ft_v0.1_aw30_pad05_e10 -Data C:\fd\data\xm_v1
  ```
  끝나면 노트북 Claude에 "E1 올림"이라고 알립니다. 판정은 결정 1(`PLAN.md` "11.9 계획"의 기준)로 합니다.
- **죽었거나 안 돌았으면** 학습 창의 마지막 30줄이나 오류를 그대로 보냅니다. 다시 돌리지 말고 2번(E4)으로 먼저 갑니다. 되먹임이 원인으로 확인돼서 E4가 더 급합니다.
- **집에서 직접 판정하려면**(선택) aw30 기준 파일이 필요합니다. 노트북의 `Downloads\xm_ft_aw30_kaggle\xm_ft_v0.1_aw30_e10\eval_test\frames.csv`를 Kaggle에서 받아서:
  ```powershell
  .\.venv-xm\Scripts\python.exe e2e\judge_d1.py --base aw30=<aw30 frames.csv> E1=data\checkpoints\xm_ft_v0.1_aw30_pad05_e10\eval_test\frames.csv
  ```
  먼저 `run_meta.json`의 `prev_action_dropout`이 0.5인지 확인합니다.

## 2. E4 학습 띄우기 (밤새)

E4는 aw30 + **실행 명령 입력**(`--prev-action-source exec`)이고, 데이터는 v1.4, 10에폭입니다.
- RSSM의 직전 명령 입력에 교사 명령 대신 실제로 실행된 명령(`exec_command`)을 넣습니다. 정답 라벨은 교사 명령 그대로입니다.
- 잡음 교사 · DAgger 구간에서 "실행 ≠ 교사"가 되어, 모델이 직전 명령을 그대로 이어 가는 버릇을 덜 배우게 하려는 것입니다.

```powershell
cd C:\fd\FUTURE-DRIVE-PROJECT
powercfg /change standby-timeout-ac 0
.\.venv-xm\Scripts\python.exe -m kaggle datasets download henryseo711/dataset-07-xm-v1-4 -p C:\fd\data --unzip
dir C:\fd\data\xm_v1_4                                     # train · val · test · index 가 바로 보여야 함(한 단계 안쪽이면 -Data 를 그쪽으로)
$CK = "$HOME\Downloads\xm_pretrained_from_seed_try2\pretrained\x_mobility-nav2-semantic_action_path.ckpt"
.\e2e\setup_windows.ps1 -Data C:\fd\data\xm_v1_4 -Ckpt $CK -ExtraGin 'XMobilityLoss.action_weight=30' -ExtraArgs '--prev-action-source','exec','--save-top-k','3' -MaxEpochs 10 -HoursBudget 14
```

시험(40배치)에서 아래 세 가지를 확인합니다.
1. `run_meta.json`(시험 출력)의 `exec_stats`에서 train 실행 ≠ 교사 칸 수가 0이 아니고, NaN이 0인지. 0이면 이 옵션은 효과가 없으므로 멈추고 알립니다.
2. 배치 로그에 "실행 ≠ 교사 N/8 칸"이 찍히는지. 실제로 입력에 들어간다는 뜻입니다. 이 옵션의 **Windows 첫 실행**이라 spawn 작업자에서도 찍히는지가 이번 확인 항목입니다.
3. `missing 0`.

출력된 본 학습 명령에서:
- `--gin NUM_EPOCHS=10`인지 확인합니다. 작게 나오면 `-HoursBudget`을 올려 다시 돌립니다.
- `-o`만 아래로 바꿔 실행합니다.
```
-o 'C:\fd\FUTURE-DRIVE-PROJECT\data\checkpoints\xm_ft_v1.4_aw30_exec_e10'
```

- **E1에서 드롭아웃이 채택되면**: 다음 run(E5)은 `-ExtraArgs`에 `'--prev-action-dropout','0.5'`를 더한 판으로 합니다. 이름은 `xm_ft_v1.4_aw30_exec_pad05_e10`입니다.
- **시간**: v1.4 train은 xm_v1보다 행이 많아 에폭당 시간이 늘어납니다. 시험 로그의 `step_time_s` × `steps_per_epoch`로 10에폭이 14 h 안에 들어가는지 봅니다.

## 3. 학습 끝나면 (아침)

```powershell
.\e2e\home_after_train.ps1 -Run data\checkpoints\xm_ft_v1.4_aw30_exec_e10 -Data C:\fd\data\xm_v1_4
```

노트북 Claude에 "E4 올림"이라고 알립니다. 노트북에서 할 판정은 두 가지입니다.
- **장애물 진단**: `obstacle_diag.py`, 기본 평가 · 중앙 장애물 · 회피 중의 크기 비. 기준은 aw30의 0.18(test) · 0.23(val)이고, 이게 오르는지 봅니다.
- **오프라인 지표**: v1.4 test에서 aw30과 비교합니다. aw30의 v1.4 test는 w_mae 0.082, Nav2 ω 부호 0.75, 자동 교사 w_mae 0.026입니다(`Downloads\xm_eval_kaggle_v7_xm_v3`).

## 4. 막히면

| 증상 | 할 것 |
|---|---|
| `exec_command 열이 없는 파일 …`로 바로 멈춤 | `-Data`가 v1.4가 아님(xm_v1 등). 경로 확인 |
| Kaggle 다운로드 401 · 403 | `$HOME\.kaggle\access_token` 또는 `kaggle.json`이 있는지(`SETUP_LOCAL.md` "챙길 것" 3번) |
| 시험 OOM | `-BatchSize 2 -Accum 4`가 기본. 그대로인지 확인(배치 4는 16 GB에서 OOM, 11.6 실측) |
| `git pull` 충돌 | 집 PC에서 고친 파일이 있으면 `git stash` 뒤 pull |
| 그 밖 | 오류 화면이나 마지막 30줄을 노트북 Claude에 |

## 5. 일정 (`PLAN.md` 표가 기준)

- 11.12: D가 씬 판을 결정(C 맵 v0.9 · v0.8 r3 또는 r3). 새 맵이면 11.15에 그 맵 교사 녹화가 오고 → v1.5로 다시 만듦.
- 11.14 09:00: Kaggle 30 h 갱신 → 집 PC와 Kaggle 두 곳에서 학습.
- 11.15: 결정 2(최종 설정).
- 11.16: 기능 동결.
- 11.20: 후보 ckpt 2–3개를 D에 전달(짧은 폐루프).
- 11.23: 모델 동결.
