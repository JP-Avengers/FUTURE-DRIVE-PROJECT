# 11/8 손 촬영(feature_*) 처리 순서

D435i 손 촬영 6회차(IT1 벽·IT1 서쪽·422동 벽 2회·주차장 가운데·둘레 한 바퀴)를 v0.5 좌표에 맞추려는 작업.

| 단계 | 스크립트 | 내용 |
|---|---|---|
| 1 | `01_map_single.sh` | depth 거리 보정(c=0.007) → 단독 매핑(IMU, 정렬 depth) → 내보내기 → 지표 |
| 2 | `02_multisession.sh` | feone 바탕에 나머지 5회차를 차례로 이어 붙임(fems1~fems5) |
| 3 | `03_image_match.sh` | 손 촬영 지도 ↔ 기준 지도 사진 매칭 |
| 4 | `scripts/fe_icp.py`, `seg_icp_warp*.py`, `local_paint_search.py` | ICP·구간별 ICP·주차선 탐색 |
| 5 | `scripts/verify_align.py`, `verify_paint.py`, `verify_ortho.py` | 정합 검증 |

현재 상태(11/9): 사진 매칭은 IT1 입구 근처만 성공. ICP 결과는 바닥 정사 영상 합성 대조로 확인한 결과
주차장 안에서 어긋남(상관 r≈0) → 바닥 정사 영상 상관을 정합 목적 함수로 쓰는 방식으로 다시 진행 예정.
