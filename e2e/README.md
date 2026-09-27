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

PyTorch (`requirements.txt` 로 고정 예정). 추론 노드는 [`ros2_ws/src/`](../ros2_ws/src)에 만듭니다.

> 가중치(`*.pt` `*.ckpt` …)·데이터셋은 커밋하지 않습니다 — [`MANIFEST.md`](../MANIFEST.md) 에 기록.

## 실행 방법

(작성 예정 — 명령 한 줄이라도 동작하는 것부터 적어 주세요)
