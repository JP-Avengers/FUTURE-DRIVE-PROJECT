# site-pipeline — 장소 하나를 명령 하나로 들이기 (D 사이드 작업 사본)

2026-11-23, D. 서버 사이드 작업 브랜치 `d/site-pipeline`(끝 커밋 `f5d7dab`, main `5a565ae` 에서 가름)이 main 과 다른 파일 16개를 **원래 경로 그대로** 이 폴더 아래에 복사해 둔 것임.

## 주의

- **여기 있는 것은 사본임.** 파일들은 서로를 레포의 원래 경로(`sim/…`, `eval/…`, `recon/…`)로 부르므로, 이 폴더 안에서 그대로 실행하면 동작하지 않음. 쓰려면 원래 자리로 옮기거나 경로를 고쳐야 함.
- "고친 파일" 10개는 main 에 같은 이름의 원본이 있음. 이 사본은 11.16~11.19 시점의 main 에서 고친 것이라, 그 뒤 main 에 들어간 변경은 들어 있지 않음.
- 혼자 쓸 수 있는 것은 `sim/stray_to_tum.py` 하나임(numpy · Pillow · ffmpeg 만 있으면 됨).

## 새 파일 (6)

| 파일 | 하는 일 |
|---|---|
| `sim/stray_to_tum.py` | Stray Scanner 녹화 폴더(아이폰 LiDAR) → RGB-D + 자세 폴더(tum 모양). 가짜 녹화 폴더로만 확인했고 실제 아이폰 촬영분으로는 아직 돌리지 않았음. `rgb.mp4` 를 풀려면 ffmpeg 가 필요함 |
| `sim/add_site.py` | 입력(RGB-D + 자세 폴더 · 맵 패키지) → 맵 → Nav2 지도 · 경로 · 시나리오 · 씬 루트 → 맵 판정까지 명령 하나로 |
| `eval/routes/make_route.py` | 찍은 점으로 기준선 · 출발 자세 · 경로 지시 구간을 만듦 |
| `eval/scenario_site.py` | 시나리오 yaml 에서 장소별 씬 · 지도 · 기준선을 찾음 |
| `eval/sites/parking_r3.yaml` · `sim/export_split_parking_r3.txt` | 주차장 r3 의 장소 설정과 내보내기 구간 |

## 고친 파일 (10)

장소(주차장 r3)에 묶여 있던 기본값과 작업 폴더 · ROS 도메인 · tmux 세션 이름을 걷어내고 시나리오 yaml 과 환경 변수에서 받게 한 것임.

`eval/scenarios/gen_rand_obstacles.py` · `eval/xm_batch.sh` · `eval/xm_route.py` · `sim/dagger_batch.sh` · `sim/export_xmobility.sh` · `sim/make_obstacles.py` · `sim/nav2/nav2_route.py` · `sim/nav2/record_demo.py` · `sim/nav2/record_demo.sh` · `sim/nav2/scenario_batch.sh`

## 들어 있지 않은 것

- 맵 판정 도구(`map_gate.py` · `gate_verdict.py`)는 레포가 아니라 서버의 `~/fd_side/gate` 에 있음.
