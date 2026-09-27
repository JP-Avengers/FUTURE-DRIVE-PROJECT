# sensor/ — 센서 / 실데이터

- **담당**: A · 정동기
- **목적**: RealSense D435i 로 RGB·Depth·IMU 를 취득하고, ROS 2 로 수집·검증한다.

## 간트 작업

| 작업 | 기간 |
| --- | --- |
| D435i 캘리브레이션 · rosbag2 · TF · 샘플 v0 | 09.22–09.28 |
| 1·2차 촬영 · 보완 촬영 · 데이터 QC | 09.29–11.06 |

## 입력 · 출력

- **받는 것**: —
- **넘기는 것**: rosbag2(MCAP) → B · 규약: [`docs/interfaces/a2b_rosbag.md`](../docs/interfaces/a2b_rosbag.md)

## 환경

공용 노트북 · ROS 2 Jazzy · librealsense2. ROS 패키지는 [`ros2_ws/src/`](../ros2_ws/src)에 만듭니다.

> rosbag 원본은 커밋하지 않습니다 — server207 `$CD1/data/` 로 옮기고 [`MANIFEST.md`](../MANIFEST.md) 에 기록.

## 실행 방법

(작성 예정 — 명령 한 줄이라도 동작하는 것부터 적어 주세요)
