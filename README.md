# FUTURE-DRIVE-PROJECT

경북대학교 종합설계프로젝트1 · SW중심대학 산학협력프로젝트 (참여기업: ㈜퓨처드라이브)

## 과제명
Digital Twin 기반 자율주행 알고리즘 개발 및 시뮬레이션 검증

실제 캠퍼스 보행로 일부를 촬영해 Visual SLAM·3D Reconstruction으로 Digital Twin을 구축하고,
NVIDIA Isaac Sim으로 옮겨 ROS2 기반 자율주행(카메라 기반 E2E 모델, Nav2 미사용)을 시뮬레이션으로 검증합니다.

## 팀 구성
| 담당 | 이름 | 역할 | 폴더 |
|---|---|---|---|
| A | 정동기 | 센서 / 실데이터 | [`sensor/`](sensor/) |
| B | 오윤성 | Visual SLAM | [`slam/`](slam/) |
| C | 이진우 | Mesh / USD | [`recon/`](recon/) |
| D | 정동현 | Isaac Sim / **PM** | [`sim/`](sim/) |
| E | 서진석 | E2E / 평가 | [`e2e/`](e2e/) |
| D·E | | 평가 | [`eval/`](eval/) |

파이프라인: A → B → C → D → E, E의 모델은 다시 D의 평가로 돌아옵니다. 파트 사이 인계 규약은 [`docs/interfaces/`](docs/interfaces/)에 있습니다.

## 폴더 구조
```
sensor/  slam/  recon/  sim/  e2e/  eval/   파트별 코드·설정 (각 폴더 README 참고)
ros2_ws/src/                               ROS 2 패키지 (전부 여기)
docs/interfaces/                           파트 간 인계 규약
docs/                                      환경·서버 문서
scripts/                                   공용 도구
MANIFEST.md                                대용량 산출물 버전 목록
```

## 공식 일정
- 2026.09.21 계획 및 요구분석 발표
- 2026.09.28 수행계획서 제출
- 2026.10.19 중간 발표
- 2026.11.02 중간 보고서 제출
- 2026.12.07 결과 발표
- 2026.12.20 결과보고서 및 실적 제출

## 저장소 규칙
- 코드는 이 저장소에서 Git으로 관리합니다.
- 대용량 원본 데이터, 3D Mesh, 학습된 모델 가중치는 이 저장소에 커밋하지 않고 별도 저장 위치와 버전 목록을 공유합니다 (`.gitignore` 참고). 버전 목록은 [`MANIFEST.md`](MANIFEST.md)입니다.
- 커밋 전 훅: `pre-commit install` (5MB 초과 파일·개인키·충돌 표식 차단, `sim/` 의 `cv_bridge` import 차단).
- 브랜치·PR 사용 여부는 각자 판단합니다. `main` 에 바로 커밋·푸시해도 되며, 이때는 푸시 직전 `git pull --rebase` 를 하고 `main` 에 `--force` 푸시는 하지 않습니다. 실행되는 공통 버전(main)을 깨지 않게 유지합니다.
