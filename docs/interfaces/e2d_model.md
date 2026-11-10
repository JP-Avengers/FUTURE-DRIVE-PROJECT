# E → D · 모델 · 추론 인터페이스

- **넘기는 쪽 → 받는 쪽**: E 서진석 → D 정동현
- **상태**: 초안 · 합의 · **동결** 중 — (초안)
- **합의일 / 합의자**: —

> 이 문서가 두 파트 사이의 계약입니다. 항목을 바꿀 때는 **받는 쪽 동의 후** 커밋하고, 동결 후 변경은 버전을 올리세요.

## 규약

| 항목 | 값 | 비고 |
| --- | --- | --- |
| 입력 토픽 · 형식 | `/front_camera/image_raw`(640×480 `rgb8`) · `/chassis/odom`(`twist.twist.linear.x` = ego_speed) · `/route`(`nav_msgs/Path`, 로봇 앞 기준선 20점 0.5 m 간격) | 전처리는 학습 형식과 같게: 512×320 로 줄인 뒤 모델 안에서 (320, 512) · DINOv2 정규화. route 는 로봇 좌표계 20점 → 19개 구간 벡터(X-Mobility `x_mobility_navigator` 와 같음). **E 제안 · D 확인 필요**(서버 `tools/xm_route.py` 가 이미 `/route` 를 냄) |
| 출력 토픽 · 형식 | `/cmd_vel` `geometry_msgs/Twist` — `linear.x` = v (m/s) · `angular.z` = ω (rad/s), 나머지 0 | 모델 출력 6차원 `[v,0,0,0,0,ω]` 의 0 · 5번. 발행자 1개 규칙([topics.md](topics.md)) |
| 모델 파일 형식 | PyTorch Lightning `.ckpt`(`e2e/finetune.py` 출력 `checkpoints/`, best = 명령 손실 최소) | 약 1 GB. ONNX/TensorRT 는 지연이 모자랄 때만 X-Mobility `onnx_conversion.py` 로 |
| forward pass 코드 위치 | 추론 노드 `ros2_ws/src/fd_xm_infer`(E, 11.5) — X-Mobility 레포 `5837aea` 의 `XMobility.inference` 를 학습 설정 `e2e/configs/finetune_parking_r3.gin`(`ENABLE_SEMANTIC=False`)으로 만든 모델에 씀. 영상 전처리는 학습 데이터와 같게(Pillow BILINEAR 512×320 + JPEG q90 왕복), 상태 · 직전 명령은 배포 노드(`x_mobility_navigator`)처럼 이어 받음 | D 의 기존 추론 도구(`~/xmobility_test/tools`)로도 되면 그쪽을 써도 됨 — 같은 gin · ckpt 를 주면 같은 모델. **D 확인 필요**: 어느 쪽으로 60회를 돌릴지 |
| 추론 지연 요구 | 5 Hz(0.2 s) 안 — 학습 형식과 같은 주기 | `fd_xm_infer` 가 25 tick 마다 · 끝날 때 `XM_INFER_LATENCY_MS` 로 모델 · 전처리 · 전체 P50/P95 를 찍음(`log_csv` 면 `.meta.json` 에도). 측정 후 이 칸에 값. 참고: `xm_ft_v0.1` 오프라인 평가(배치) 처리량 T4 1장 19.57 프레임/s — 노드 지연은 아직 안 잼 |
| 버전 | MANIFEST `checkpoint` 행(`xm_ft_v<주>.<부>`) + 출력 폴더 `run_meta.json` | 첫 판 **`xm_ft_v0.1`**(11.6) = `epoch=8-step=10719.ckpt` — best = `validation/losses/action` 0.3224(10 에폭 중 에폭 8) · 987M(1,034,137,918 B, 0.96 GiB) · sha256 `d86970ce1faf`. 학습: `xm_parking_r3` v1.0(`xm_v1`) + `nvidia/X-Mobility` 사전학습 ckpt(sha `339c68f9e027`) · `e2e/configs/finetune_parking_r3.gin`(`ENABLE_SEMANTIC=False`) · Kaggle T4×2. **동결 후보**(같은 데이터 · 사전학습 ckpt · 10 에폭 · best 는 모두 `epoch=8-step=10719.ckpt` 라 **sha 로만 구분**): `xm_ft_v0.1_aw30`(11.7, `XMobilityLoss.action_weight=30` — v0.1 은 10) 987M · sha256 `4fdf249cda0a` · `xm_ft_v0.1_aw30_p3`(11.9, aw30 + 경로 라벨 ×1/3 데이터) sha256 `114284109957` — 현재 1순위, 11.9 교차 평가 완료 결정. p3 계열은 모델 경로 출력이 1/3 m 단위라 노드 `path_scale:=0.3333`(`~/path` 디버그만 영향, 명령 v · ω 는 무관) |

## 인계 방법

- 실물 위치: server207 `$CD1/data/checkpoints/` — [`MANIFEST.md`](../../MANIFEST.md) 에 버전 기록
- 현황(11.10): Git 에 안 올림(용량). **11.9 E Drive 폴더에 3개 업로드 · D 에 링크 전달**(링크는 메시지로만 — 이 레포는 공개라 적지 않음). 11.10 Drive 크기 = 로컬 원본과 바이트 단위로 같음 확인. 서버 `data/checkpoints/` 배치는 D(11.11 노드 시험 때, sha 대조). 파일 이름은 Kaggle 데이터셋 `fd-xm-ckpts` 와 같게: **`xm_ft_v0.1_e8.ckpt`**(= v0.1 best `epoch=8-step=10719.ckpt`, sha `d86970ce1faf`) · **`xm_ft_v0.1_aw30_e8.ckpt`**(= aw30 best, sha `4fdf249cda0a`). `xm_ft_v0.1_aw30_p3`(sha `114284109957`, 11.9 교차 평가 완료 · 종합 1위 후보)도 같은 식(`xm_ft_v0.1_aw30_p3_e8.ckpt`)으로
- D 가 할 것: 받은 파일을 이름 그대로 `$CD1/data/checkpoints/xm_ft_v0.1/checkpoints/xm_ft_v0.1_e8.ckpt` · `.../xm_ft_v0.1_aw30/checkpoints/xm_ft_v0.1_aw30_e8.ckpt` 에 두고 `sha256sum <파일> | cut -c1-12` 로 위 sha 확인 → 첫 노드 시험(`eval/XM_HARNESS.md` §2, `--dry-run` 먼저)은 v0.1 로 충분, DAgger 주행은 aw30(또는 결정된 1순위). `eval/xm_batch.sh` 가 시작 줄에 sha 를 찍고 결과 CSV `note` 에 `ckpt_sha12` 를 남김
- 납기: 모델 동결: 11.23

## 변경 이력

| 날짜 | 버전 | 변경 | 동의 |
| --- | --- | --- | --- |
| 11.4 | v0 | 입력 · 출력 · 모델 형식 · forward 위치 초안(X-Mobility 파인튜닝 기준) (E) | |
| 11.5 | v0 | forward 위치 = 추론 노드 `fd_xm_infer`, 지연 측정 방법, best ckpt 기준(명령 손실) (E) | |
| 11.6 | v0 | 첫 모델 파일 `xm_ft_v0.1`(크기 · sha · 학습 출처), 인계 현황(D 서버 전달 대기), 오프라인 처리량 19.57 프레임/s (E) | |
| 11.9 | v0 | 동결 후보 `xm_ft_v0.1_aw30` · `xm_ft_v0.1_aw30_p3`(sha), 인계 = Drive 업로드 예정(11.9, 파일 이름 · sha) + D 서버 배치 경로 · sha 확인, p3 계열 `path_scale` (E) | |
