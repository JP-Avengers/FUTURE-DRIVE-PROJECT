# 묶음을 장소로 들여 주행해 보기 (D)

웹앱이 만든 `bundle.zip`이 `sim/add_site.py`를 거쳐 GPU 서버의 Isaac Sim에서 주행되는지 확인한 기록. 계획은 `implementation.md` 3-2 · 6-D.

**11.25 현재: 들이기(2번)까지 함. Nav2 + AMCL 주행(3번)은 아직 안 함.**

## 1. zip 구조 — 고침 (11.25)

`bundle.zip`의 맨 위가 `bundle/` 폴더 하나가 됨. 보고서(`report.md`)와 결함 목록(`defects_*.json`)도 그 안에 넣음(`web/server.py`의 `make_bundle`).
`add_site.py`는 맨 위가 폴더 하나면 그 안을 맵 패키지로 보므로, 내려받은 zip을 풀지 않고 그대로 줄 수 있음.

```
bundle/
  usd/input_mesh.usdc · input_scene.usda
  nav2/map.pgm · map.yaml · preview.png
  scene_root.usda · scene_root_sim.usda · start_pose.json · route.csv · bundle.json
  report.md · defects_before.json (· defects_after.json)
```

11.25 전에 끝난 작업의 `bundle.zip`은 한 번 만들면 다시 만들지 않으므로 예전 구조(맨 위에 `bundle/`과 `report.md`가 나란히) 그대로임.

## 2. 들이기 — 두 건 (11.25, server207)

```bash
FD_USD_PY=<pxr 파이썬> FD_RECON_PY=<recon 파이썬> FD_GATE_DIR=<맵 판정 도구 폴더> \
  python3 sim/add_site.py <bundle.zip> --site <장소> --ver v1 --dest <놓을 곳>
```

레포가 아닌 시험 폴더(`--dest`)에 놓았음. 경로(`--route`)는 주지 않음.

| | 가상의 방 (GLB 입력) | 실제 촬영본 (Stray 입력) |
|---|---|---|
| 입력 | `samples/make_room.py`의 `room.glb`를 예시 파일로 돌린 작업의 `bundle.zip` (13개 파일) | 실내 촬영본 zip 540 MB를 돌린 작업의 결과에서 `make_bundle`로 만든 `bundle.zip` (12개 파일) |
| 웹앱 판정 | 주의 | 올릴 수 있으나 주의 항목을 볼 것 — 천장 있음 · 범위 11×10 m · 빈 바닥 62 m² · 보이는 바닥 없음 6.2 % |
| input · usd | ok — 메쉬 이름이 `<장소>_mesh.usdc`가 아니라는 말만 붙음(찾는 데는 문제없음) | ok — 같음 |
| nav2 | ok — 247×207 칸. 묶음의 `nav2/map.pgm`과 바이트가 같음 | ok — 305×279 칸. 묶음의 것과 바이트가 같음 |
| route | 건너뜀(경로를 안 줌) | 건너뜀 |
| scene | 새로 씀 — 출발 (1.525, 1.075) 200° · 여유 1.95 m · 제자리 회전 가능 넓이 22.28 m² | 새로 씀 — 출발 (−0.226, −2.147) 90° · 여유 0.951 m · 제자리 회전 가능 넓이 1.35 m² |
| gate (맵 판정) | 통과 | **실패 예상** — 주행 공간 1.4 m²(5.0 m² 미만) · 주의: 화면 바닥 구멍 평균 2.86 % |
| 끝 줄 | `SITE_OK` | `SITE_FAIL` |

### 걸린 것

1. **`add_site.py`가 출발 x가 음수면 맵 판정을 못 돌림 — 고침.** `--start -0.226,…`처럼 값을 따로 넘겨서 `map_gate.py`의 argparse가 값을 옵션으로 읽고 멈춤(끝 줄은 `SITE_WARN`). `--start=<값>` 한 덩어리로 넘기게 한 줄 고침. 고친 뒤 위 표의 결과가 나옴.
2. **씬 루트에 그림자 설정이 빠짐 — 열림(C · D가 정할 것).** 실제 촬영본 맵은 천장이 있는데, `add_site.py`가 새로 쓴 씬 루트(`sim/scenes/<장소>/<이름>.usda`)에는 `doNotCastShadows`가 없음. 묶음의 `scene_root.usda`에는 들어 있지만 `add_site.py`는 `scene_suggestion/<장소>_<판>.usda`만 찾으므로 쓰지 않음. 이 씬 루트를 Isaac Sim에서 열어 보지는 않았음 — 묶음을 만드는 쪽 설명대로면 화면이 검게 나옴. 정할 것: 묶음이 씬 루트를 `scene_suggestion/`에 넣을지, `add_site.py`가 묶음의 `scene_root.usda`를 받을지. **정함(11.26, C12): `add_site.py`가 묶음의 것을 받음.** 받는 파일은 팀 시뮬 배치용 `scene_root_sim.usda`임(`scene_root.usda`는 로봇 · 센서가 없어 씬 루트로 못 씀). 패키지 안에 그 파일이 있으면 맵 경로 한 줄만 `data/usd/<장소>_<판>/`으로 바꿔 `sim/scenes/<장소>/<이름>.usda`로 씀 — 출발 자세와 그림자 설정이 그대로 옴. 없으면 예전처럼 새로 씀. 따로 주려면 `--scene-root <파일>`. 실제 촬영본 묶음으로 들여 `doNotCastShadows` 2곳이 들어간 것까지 확인했고, **Isaac Sim에서 열어 보지는 않았음.**
3. **이름.** 묶음 안의 이름은 모두 `input`(`input_mesh.usdc` · `input_scene.usda`). 들이는 데는 문제없고, 놓인 폴더 이름은 `<장소>_<판>`이 됨.
4. **출발 자세가 서로 다름.** 실제 촬영본에서 위치는 같고((−0.226, −2.147)) 방향만 다름 — 묶음 126.1°, `add_site.py` 90.0°. 경로 없이 들이면 `add_site.py`는 묶음의 `start_pose.json` · `route.csv`를 쓰지 않고 지도에서 다시 잡음.

## 3. 주행 — 아직 안 함

할 것: 찍은 점 몇 개로 `<이름>_route.yaml`을 써서 `--route`로 다시 들이고, `site_report.md` 끝의 명령으로 Nav2 + AMCL 주행을 한 번 돌림. 볼 것은 씬이 열리는지, 로봇이 바닥에 서는지, 천장이 있는 맵에서 카메라 화면이 검게 나오지 않는지, 충돌 형상이 보이는 형상과 맞는지.

- 실제 촬영본 묶음은 맵 판정이 "실패 예상"(제자리 회전 가능 넓이 1.4 m²)이라, 주행은 더 넓게 찍은 촬영본(E의 11/26 시험 촬영 묶음)으로 해 보는 것이 맞음.
- 시뮬은 60회 평가가 안 도는 시간에만 띄움.

참고: 서버에는 다른 날 돌린 화면 확인 기록이 있음(이 문서를 쓰면서 다시 돌리지 않았고 로그의 숫자만 봄). 11.25 12:55 `~/fd_side/deploy_test/isaac3` — Isaac Sim 6.0.1에서 묶음 씬(메쉬 4개 · 범위 약 11×10×3.5 m)을 열어 네 방향을 그림. 그대로 연 것의 검은 화소는 최대 1.0 %, 평균 밝기 110~165. 로봇 주행이 아니라 고정 카메라 화면임.

## 4. 두 판정 비교

| 맵 | 웹앱 진단 | `add_site.py` 맵 판정 | 다른 점 |
|---|---|---|---|
| 가상의 방 | 주의 | 통과 | 웹앱은 일부러 넣은 결함을 짚음. 맵 판정은 주행 공간 · 화면 바닥 구멍만 봄 |
| 실제 촬영본 | 올릴 수 있으나 주의 | **실패 예상** | 웹앱은 "빈 바닥 62 m²"로 통과시키고, 맵 판정은 "로봇이 제자리에서 돌 수 있는 넓이 1.4 m²"로 실패를 예상함. 재는 것이 달라서로 보임(추정 — 두 코드의 계산을 대조하지는 않음): 웹앱은 지도의 빈 칸 넓이, 맵 판정은 로봇이 돌 만큼 여유가 있는 자리의 넓이 |

웹앱 판정 "올릴 수 있음"은 "이 맵 때문에 시뮬레이터가 멈추거나 화면이 깨지지 않는다"는 뜻이고 주행이 된다는 뜻이 아님(보고서에 적혀 있음). 두 번째 줄 같은 맵을 사용자에게 어떻게 알릴지는 C와 정할 것 — 묶음의 `start_pose.json`에 있는 여유(0.95 m)만으로는 드러나지 않음.

## 확인에 쓴 것

- 맵 판정 도구(`map_gate.py` · `gate_verdict.py`)는 저장소에 없고 서버에만 있음(3-2).
- 가상의 방 한 건은 `deploy/start.sh`로 켠 서버에 예시 파일(`POST /api/samples/room.glb/jobs`)로 넣어 만듦. 실제 촬영본은 11.25 12:48에 끝난 작업의 결과 폴더를 그대로 씀(다시 처리하지 않음).
