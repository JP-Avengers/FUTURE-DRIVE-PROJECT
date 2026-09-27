# A → B · rosbag2 녹화본

- **넘기는 쪽 → 받는 쪽**: A 정동기 → B 오윤성
- **상태**: 초안 · 합의 · **동결** 중 — (초안)
- **합의일 / 합의자**: —

> 이 문서가 두 파트 사이의 계약입니다. 항목을 바꿀 때는 **받는 쪽 동의 후** 커밋하고, 동결 후 변경은 버전을 올리세요.

## 규약

| 항목 | 값 | 비고 |
| --- | --- | --- |
| 저장 형식 | | MCAP 권장 (`ros-jazzy-rosbag2-storage-mcap`) |
| 토픽 목록 | | RGB · Depth · IMU · camera_info · tf_static |
| 해상도 · fps | | RGB / Depth 각각 |
| depth 정렬 | | aligned_depth_to_color 여부 |
| TF 트리 · frame_id | | `fd_interfaces` 규약과 일치 |
| 시간 동기 | | 하드웨어 동기 / 소프트웨어 |
| 촬영 조건 메타데이터 | | 시간대 · 날씨 · 구역 (실외 depth 품질 리스크) |

## 인계 방법

- 실물 위치: server207 `$CD1/data/raw/rosbag/` (raw/ 는 신설 필요) — [`MANIFEST.md`](../../MANIFEST.md) 에 버전 기록
- 납기: 샘플 v0: 09.28

## 변경 이력

| 날짜 | 버전 | 변경 | 동의 |
| --- | --- | --- | --- |
| | | | |
