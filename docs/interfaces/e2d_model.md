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
| 추론 지연 요구 | 5 Hz(0.2 s) 안 — 학습 형식과 같은 주기 | `fd_xm_infer` 가 25 tick 마다 · 끝날 때 `XM_INFER_LATENCY_MS` 로 모델 · 전처리 · 전체 P50/P95 를 찍음(`log_csv` 면 `.meta.json` 에도). 측정 후 이 칸에 값 |
| 버전 | MANIFEST `checkpoint` 행(`xm_ft_v<주>.<부>`) + 출력 폴더 `run_meta.json` | |

## 인계 방법

- 실물 위치: server207 `$CD1/data/checkpoints/` — [`MANIFEST.md`](../../MANIFEST.md) 에 버전 기록
- 납기: 모델 동결: 11.23

## 변경 이력

| 날짜 | 버전 | 변경 | 동의 |
| --- | --- | --- | --- |
| 11.4 | v0 | 입력 · 출력 · 모델 형식 · forward 위치 초안(X-Mobility 파인튜닝 기준) (E) | |
| 11.5 | v0 | forward 위치 = 추론 노드 `fd_xm_infer`, 지연 측정 방법, best ckpt 기준(명령 손실) (E) | |
