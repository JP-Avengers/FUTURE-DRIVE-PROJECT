# slam/ — Visual SLAM

- **담당**: B · 오윤성
- **목적**: RTAB-Map 기준선으로 실제 환경을 매핑하고, 궤적·맵 품질을 검증한다.

## 간트 작업

| 작업 | 기간 |
| --- | --- |
| RTAB-Map 기준선 · 맵 v0.1 | 09.29–10.11 |
| SLAM 튜닝 · 궤적 검증 · 최종 맵 | 10.12–11.06 |

## 입력 · 출력

- **받는 것**: A 의 rosbag2 · [`a2b_rosbag.md`](../docs/interfaces/a2b_rosbag.md)
- **넘기는 것**: 포인트클라우드 · 궤적 → C · [`docs/interfaces/b2c_map.md`](../docs/interfaces/b2c_map.md)

## 구조

```
slam/
├── config/           RTAB-Map 파라미터 (기준선 v0.1 → 튜닝본)
└── evaluation/       궤적 검증 · 맵 품질 지표
```

## 환경

공용 노트북 · ROS 2 Jazzy · rtabmap_ros. ROS 패키지는 [`ros2_ws/src/fd_slam`](../ros2_ws/src/fd_slam).

> `rtabmap*.db` 는 커밋하지 않습니다 — [`MANIFEST.md`](../MANIFEST.md) 에 기록.

## 실행 방법

(작성 예정 — 명령 한 줄이라도 동작하는 것부터 적어 주세요)
