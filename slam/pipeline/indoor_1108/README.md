# 실내 복도 맵 처리 순서 (11/8 `inside_*` 5회차)

| 단계 | 스크립트 | 하는 일 | 결과 |
|---|---|---|---|
| 1 | `01_map_single.sh` | 회차별 단독 매핑 | in2 · in1r 성공, in1 · inrw · inlw 추적 끊김 |
| 2 | `02_retry_reset.sh` | 끊긴 3회차를 `Odom/ResetCountdown 1`로 재시도 | in1b · inrwb · inlwb — 추적은 이어지나 맵이 조각남 |
| 3 | `03_manhattan.sh` | 주 묶음(in2) 보정: 바닥 수평 · 벽 방향 나침반 · 복도 벽 직선 맞춤 | `in2m` (건물 좌표) → 맵 v0.1 |
| 4 | `04_localize.sh` | 나머지 회차를 `in2m` 맵에 키프레임마다 맞춰 넣기 | `in1bL` · `in1rL` · `inrwbL` · `inlwbL` |
| 5 | `05_fuse_eval.sh` | 구성 4종 TSDF + 검증 회차 · 로봇 시점 평가 + 복도 폭 · 벽 높이 점검 | 구성 C(in2 + in1b + rightwall) 채택 |
| 6 | `06_package_v02.sh` | 패키지 조립 · 체크섬 · C 입력 점검 | 맵 v0.2 (`docs/deliver/corridor_v0.2_README.md`) |

- 좌표계는 건물 좌표(복도 축 x, `inside_2` 출발점 원점, 바닥 z = 0)다. 주차장 맵과는 이어져 있지 않다.
- `in_selfalign.py`(벽 따라가기)와 `in_xcheck.py`(복도 방향 상관)는 시험했으나 쓰지 않았다. 이유는 각 파일 머리말과 커밋 기록에 있다.
- 실행별 수치는 `slam/runs.csv`의 `in*`, `eval_corridor_v02`, `pkg_corridor_*` 줄에 있다.
