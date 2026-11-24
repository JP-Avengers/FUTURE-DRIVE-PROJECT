# CONTRACT.md — scan2sim 약속 (엔진 · 서버 · 화면이 주고받는 것)

**기준 코드: `b/yunseong` `d019a22`** (main 의 C · D 11/24 변경 `736eb68` ~ `ed7220e` 을 합친 `ed4ee25` + 시험 · 기록 커밋 하나). 이 문서의 `파일:줄` 은 모두 이 커밋 기준이다.
**고침 11/24 저녁 (`ccaca39`, C 의 `620a2b0` 을 합침):** 씬 루트가 둘로 나뉜 것(2-4)과 R3 · R4 가 붙은 것을 반영했다. `620a2b0` 이 `stages.py` 에 줄을 더해, 이 문서의 `stages.py:줄` 가운데 진단 · 묶음 단계 쪽은 지금 코드보다 앞을 가리킬 수 있다 — 함수 이름으로 찾을 것.

작성 2026-11-23 · B · 고침 2026-11-24 (C · D 의 11/24 변경 반영)

- 이 문서는 **지금 코드가 하는 일**을 적는다. 계획(`docs/implementation.md` 5장)과 다른 곳과 버그로 보이는 것은 8장에 모았다.
- 근거 표시: 코드 위치는 `파일:줄`. 경로는 `recon/scan2sim/` 기준이고, `pipeline/` · `tools/` · `configs/` 는 `recon/` 아래다. 코드를 읽지 않고 짐작한 것은 **[추정]** 이라고 적었다.
- 예시 JSON 은 실제 실행 결과에서 가져왔다. 긴 배열은 줄였다(`…`).
  - `room` = 서버를 띄워 방 촬영본(Stray zip `6bc1ae021f.zip`, 118초 · 379 MB, E 가 찍음)을 API 로 올린 시험. 기준 코드에서 `tests/e2e.py` 로 돌림. 작업 `694ddc986821`, 결과 · 엔진 출력은 B 노트북 작업 폴더(저장소 밖). 15/15 통과 · `done` · 판정 `warn` · 82초.
  - `e2e_glb` = 11/23 에 서버로 GLB(`corridor_v04.glb`)를 올린 시험 (작업 `b8ce80baff04`). 그때 환경에 `scikit-image` 가 없어 변환 단계에서 실패했다. 8장에서 고쳐진 항목을 설명할 때만 쓴다.
  - 11/23 판의 `stray1` · `e2e_stray` 예시는 Stray 를 맵 폴더 길로 돌린 결과라 지금 출력과 모양이 달라 뺐다 (1-2).

---

## 0. 이 약속을 바꾸는 법

1. API 경로 · 응답 필드, 작업 폴더의 파일 이름, 결함 목록의 필드를 **바꾸거나 없앨 때**는 먼저 팀 채팅에 알린다.
2. 그 변경과 **같은 커밋**에서 이 파일을 고친다.
3. **필드를 더하는 것은 자유**다(알리지 않아도 됨). 대신 화면 · 시험 · 다른 도구는 모르는 필드를 무시해야 한다.
   - 화면은 모르는 결함 `type` 이 와도 목록에 보여 줘야 한다. 지금 화면은 `type` 을 보지 않고 `severity` · `title` 로 그린다 (`web/static/index.html:256-275`).
4. `tests/` 의 시험이 이 약속을 검사한다. 시험이 깨지면 바꾼 사람이 그날 안에 고친다.
   - `tests/e2e.py`: 서버를 띄워 API 로 올리고 상태 · 결함 필드 · 결과 파일 4개를 검사.
     검사하는 키는 `tests/e2e.py:35-38` (`JOB_KEYS` · `DONE_KEYS` · `STAGE_KEYS` · `DEFECT_KEYS`).
   - `tests/repeat.py`: 같은 입력을 두 번 돌려 결함 목록이 같은지 검사.
   - `tests/run.sh`: 가상의 방 GLB · 가상 Stray 촬영본으로 위 둘을 돌리고, 넣어 둔 결함 3개가 제자리(0.5 m 안)에 잡히는지 본다 (`--truth`). Stray 도 11/24 부터 견준다 (`d019a22`).
   - 이 문서의 필드 표를 고치면 위 키 목록도 같이 고친다.
5. 이 약속을 쓰는 다른 도구: D 의 `deploy/check.py`. 화면과 같은 순서로 올리고, 상태 `done` · `has_bundle` · `report.md` · `preview.glb` · `bundle.zip` 을 다 받으면 `CHECK_OK` (`deploy/check.py:86`).

---

## 1. 좌표계

### 1-1. 세 가지 좌표

| 이름 | 정의 | 쓰는 곳 |
|---|---|---|
| **장면 좌표** (scene) | **+Z 가 위**, 단위 m. 바닥 높이 ≈ 0 (1-2) | 엔진 내부, USD, Nav2 지도, `start_pose.json`, `route.csv`, 결함의 `center_scene`, 단계 metrics 의 위치 값 |
| **화면 좌표** (view) | Three.js. **+Y 가 위**. `화면 (x, y, z) = 장면 (x, z, −y)` | `preview.glb`, 화면의 `toView()` |
| **입력 좌표** (input) | 입력 종류마다 다름 (1-3). `p_장면 = T_input_to_scene · p_입력` | 결함의 `center_input` · `up_input` · `floor_point_input` (지금 화면은 쓰지 않음) |

- 장면 → 화면 변환은 **두 곳에서만** 한다.
  - **엔진**이 `preview.glb` 를 쓸 때: `preview.py:111`. 정점을 `(x, z, −y)` 로 바꿔 씀.
    서버는 이 파일을 주기만 하고, 파일이 없거나 USD 보다 오래됐을 때만 같은 엔진 함수(`preview.write`)를 부른다 (`web/server.py:207-213`).
    11/23 까지는 서버가 직접 변환했다. 11/24 에 엔진으로 옮겨졌다 (C `2a6e89f`, D `ed7220e`).
  - **화면**이 결함 표시를 놓을 때: `web/static/index.html:293` `toView = (p) => (p[0], p[2], -p[1])`.
- 거꾸로(화면 → 장면)는 `장면 (x, y, z) = 화면 (x, −z, y)`.
- Nav2 지도(`bundle/nav2/map.pgm`)는 장면 좌표의 위에서 본 그림이다.
  - `map.yaml` 의 `origin` 은 그림 왼쪽 아래 칸 모서리의 장면 (x, y).
  - 그림의 행 0 = 위쪽(y 가 큰 쪽) (`checks.py:8`). 칸 = `resolution` m (지금 0.05).

### 1-2. 장면 좌표의 원점과 바닥

| 입력 | 수평 맞춤 | 바닥 z = 0 | x · y 원점 |
|---|---|---|---|
| GLB · **Stray** | 바닥 평면 법선 → +Z (`pipeline/glb_to_usd.py:79-130` `find_floor`). 위를 향한 면(15° 안)의 높이 분포에서 넓은 층 중 가장 낮은 층 ± 5 cm 의 면으로 넓이 가중 최소제곱 평면을 맞춤. 2 cm 넘게 벗어난 면은 빼고 되풀이. 무작위 표본 없음 (C `736eb68`) | 그 바닥 평면 (`pipeline/glb_to_usd.py:126`) | 수평을 맞춘 메쉬 전체 정점의 x · y 범위 가운데 (자동 설정 `orient: origin: center`, `detect.py:100`; 이동은 `pipeline/glb_to_usd.py:489-493`) |
| 맵 폴더 (리얼센스) | 바닥 법선 → +Z (`pipeline/build_usd.py:70-85`) | **카메라 경로 아래 바닥 높이의 중앙값** (`pipeline/build_usd.py:251-261`, `transform.json` 의 `floor_zero`) | 입력 좌표 그대로 (틀 `configs/templates/indoor_rtabmap.yaml:33` 의 `orient: origin: keep`) |

- **Stray 는 11/24 (C `0534d95`) 부터 GLB 와 같은 길**로 간다 (`stages.py:41-59`).
  - `stray.py` 로 맵 폴더 형식(`out/input_map/`)을 만들고, `pipeline/tsdf.py` 로 깊이를 합쳐 형상 메쉬 `build/mesh_raw.ply` 를 만든다.
  - 그 메쉬를 GLB 처럼 `pipeline/glb_to_usd.py` 에 넣는다. 그래서 원점 · 바닥의 정의가 GLB 와 같다.
  - 11/23 까지는 맵 폴더 길(`build_usd.py`)이라 "x · y = 입력 그대로, z = 경로 아래 중앙값" 이었다.
- GLB · Stray 에서 바닥이 평평하지 않으면 스캔 바닥 높이 지도를 쓴다.
  - 평평함 판정: 바닥 근처 위 향한 면 높이의 5~95% 폭이 0.12 m 미만 (`detect.py:67`, `:75`).
  - 평평하지 않으면 자동 설정에 `collision.ground: scan` 이 붙고 (`detect.py:104-105`), `build/ground.npz` 를 만든다 (`pipeline/glb_to_usd.py:556`).
  - 충돌 바닥은 이 지도를 따라 깔린다. 장면 z = 0 은 그대로 맞춘 바닥 평면이다.
- 결함의 `center_scene` z 와 시작 자세의 바닥 높이는 `ground.npz` 에서 읽는다. 파일이 없으면 0 이다 (`pipeline/mesh_utils.py:77-78`).
  - 그래서 바닥이 평평한 GLB · Stray 는 z = 0 이다. `room` 처럼 평평하지 않은 바닥은 그 자리 높이다 (−0.102 ~ 0.007).
- 맵 폴더는 실내 틀에 `ground:` 항목이 있어서 `outdoor = "ground" in cfg` 가 참이 된다 (`pipeline/build_usd.py:222`).
  그래서 "바닥 = 경로 아래 중앙값" 으로 z 를 한 번 더 맞춘다. 이름은 outdoor 지만 실내에서도 이 길로 간다.

### 1-3. 입력 좌표와 `T_input_to_scene`

- `T_input_to_scene` 은 4×4 동차 행렬, **행 단위 리스트** (`T[0]` = 첫 행). 위치는 열 벡터로 곱한다: `p_장면 = T · [x, y, z, 1]`.
- 파일: `out/build/transform.json` (빌드가 씀). 결함 목록 파일에도 같은 값이 들어간다 (`stages.py:195-205`).
- `transform.json` 이 없으면 단위행렬로 친다 (`stages.py:196`). 하지만 **세 입력 모두 빌드가 이 파일을 쓴다** — 단위행렬이 되는 경우는 빌드가 비정상일 때뿐이다.

| 입력 | "입력 좌표" 가 뜻하는 것 | `T_input_to_scene` | 쓰는 곳 |
|---|---|---|---|
| GLB | 올린 GLB 의 glTF 좌표 그대로 (+Y 위) | `M(원점 · 수평 · 바닥) · Y_UP_TO_Z_UP · 배율` | `pipeline/glb_to_usd.py:630-632` |
| B 형식 맵 폴더 (리얼센스) | 맵 폴더의 세계 좌표 = `trajectory_camera_optical.txt` 의 세계 (+Z 위) | 수평 맞춤 회전 + 바닥 높이 이동 | `pipeline/build_usd.py:248`, `:260` |
| **Stray (아이폰)** | **`out/input_map/` 의 세계 좌표** = Stray 세계를 `Y_UP_TO_Z_UP` 으로 돌린 것 (+Z 위). 형상 메쉬 `build/mesh_raw.ply` 가 이 좌표다. **Stray 원본 세계(+Y 위)가 아님** | `M(원점 · 수평 · 바닥) · 배율`. 입력 메쉬가 이미 Z 위라 `Y_UP_TO_Z_UP` 을 곱하지 않음 (설정 `input: up: z`, `stages.py:59`, `detect.py:99`) | `pipeline/glb_to_usd.py:630-632` (메쉬를 만든 곳은 `stray.py:30`, `:335` + `pipeline/tsdf.py`) |

- 11/24 에 Stray 의 길이 바뀌었지만 **Stray 의 "입력 좌표" 는 그대로**다 (`stages.py:45` 주석). 바뀐 것은 T 의 내용(x · y 원점이 범위 가운데로, z = 0 이 맞춘 바닥 평면으로)뿐이다.

**Stray 의 입력 좌표 (자세히)**

- Stray `odometry.csv` 의 자세 = 카메라 광학 좌표계(x 오른쪽 · y 아래 · z 앞) → Stray 세계. Stray 세계는 **+Y 가 위** (`stray.py:18-19`).
- `stray.py` 는 카메라 쪽은 그대로 두고 세계만 돌린다: `T_맵폴더 = Y_UP_TO_Z_UP · T_stray` (`stray.py:335`).
  - `Y_UP_TO_Z_UP = [[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]]` → `p_입력 = (x, −z, y)_stray` (`stray.py:30`)
  - 거꾸로: `p_stray = (x, z, −y)_입력`
- 이 돌린 세계로 `input_map/trajectory_camera_optical.txt` 를 쓴다 (`stray.py:341-344`).
  `pipeline/tsdf.py` 가 이 자세로 깊이를 합쳐 `build/mesh_raw.ply` 를 같은 좌표로 만들고, 빌드 설정(`build/auto_config.yaml`)의 `input.mesh` 가 이 파일이다.
- 그래서 Stray 결함의 `center_input` 을 Stray 원본 좌표로 보려면 `(x, z, −y)` 를 한 번 더 해야 한다.

실제 값 (`room` 의 `build/transform.json`) — 약 0.38° 기울기 보정, x · y 를 (+4.506, −2.313) 옮겨 범위 가운데를 원점으로, z 를 1.580 m 올림. 입력 좌표에서 바닥이 z ≈ −1.555 인 것은 입력 좌표의 원점이 촬영을 시작한 폰 자리이기 때문이다 [추정: ARKit 세계 원점].

```json
{
 "T_input_to_scene": [
  [0.9999971374540524, 7.44147959498414e-06, -0.002392703141948365, 4.506043015862286],
  [7.44147959498414e-06, 0.9999806551162581, 0.006220075392132838, -2.3125912064420673],
  [0.002392703141948365, -0.006220075392132838, 0.9999777925703105, 1.580082332951277],
  [0.0, 0.0, 0.0, 1.0]
 ],
 "note": "입력 메쉬 좌표 (Z-up 그대로) → 씬 (Z-up, 바닥 z 0, m)"
}
```

(실제 파일은 수 하나마다 줄을 바꾸고 `note` 를 `\uXXXX` 로 적는다.)

- `transform.json` 의 다른 키는 입력마다 다르다. GLB · Stray 는 `T_input_to_scene` · `note` 뿐이고, 맵 폴더는 `source` · `orient` · `floor_zero` 도 있다. **약속된 키는 `T_input_to_scene` 하나**다.

---

## 2. 작업 폴더

### 2-1. 서버 쪽 (`<jobs>/<작업 번호>/`)

`<jobs>` 기본값 `~/scan2sim_jobs`, 서버의 `--jobs` 로 바꿈 (`web/server.py:41`, `:266`). 저장소 밖에 둔다. 배포 스크립트는 `S2S_JOBS` 로 정한다 (기본 같음, `deploy/common.sh:8`).

```
<jobs>/<id>/                  id = 16진수 소문자 12자리 (uuid4 앞 12자리, web/server.py:97)
  job.json                    서버가 쓰는 작업 기록 (2-2)
  input/input.zip             올린 파일. 이름은 항상 "input" + 확장자 (web/server.py:99)
  input/input.glb               — 원래 이름은 job.json 의 name 에만 남음
  run.txt                     엔진의 표준 출력 + 표준 오류 (web/server.py:145-147)
  out/                        엔진 출력 폴더 (2-3)
```

- 올린 파일 이름을 바꾸는 이유: 경로에 한글이 있으면 Open3D 가 못 읽음 (`web/server.py:99` 주석).
- 그 결과 **웹으로 올린 작업은 장면 이름이 모두 `input`** 이 된다 (2-4).

### 2-2. `job.json`

서버가 메모리에 든 작업 dict 를 그대로 쓴 것 (`web/server.py:68-69`). 4-3 의 `GET /api/jobs/<id>` 응답의 앞부분과 같다.

| 필드 | 형 | 언제 생기나 | 뜻 |
|---|---|---|---|
| `id` | 문자열 | 만들 때 | 작업 번호 |
| `name` | 문자열 | 만들 때 | 올린 파일의 원래 이름 (경로 빼고, 앞 120자) |
| `size` | 정수 | 만들 때 | 알린 파일 크기 (바이트) |
| `received` | 정수 | 만들 때 (0) | 지금까지 받은 바이트. **조각을 받을 때마다 메모리에서만 늘고 `job.json` 에는 `start` 때 저장됨** |
| `status` | 문자열 | 만들 때 | `uploading` · `queued` · `running` · `done` · `failed` (4-3) |
| `created` | 실수 | 만들 때 | 유닉스 시각(초) |
| `file` | 문자열 | 만들 때 | `input.zip` 또는 `input.glb` |
| `queued_at` | 실수 | `start` 때 | 줄에 넣은 시각 |
| `started` | 실수 | 처리 시작 | |
| `exit_code` | 정수 | 엔진이 끝남 | 엔진 종료 코드 (3-1). 엔진이 끝났으면 `failed` 여도 있음. 엔진을 못 띄운 경우에는 없음 |
| `finished` | 실수 | 처리 끝 | |
| `error` | 문자열 | 서버 쪽 실패일 때만 | 서버 쪽 예외 (`"TypeError: …"`) 또는 `"서버가 다시 켜져서 끊김"`. 엔진 단계가 실패한 `failed` 에는 없음 |

실제 `job.json` (`room`):

```json
{
 "id": "694ddc986821",
 "name": "6bc1ae021f.zip",
 "size": 379159774,
 "received": 379159774,
 "status": "done",
 "created": 1795504598.078851,
 "file": "input.zip",
 "queued_at": 1795504598.7145457,
 "started": 1795504598.7151208,
 "exit_code": 0,
 "finished": 1795504680.2617059
}
```

- 11/23 의 `e2e_glb` 는 변환 단계가 실패했는데도 `done` · `exit_code 1` 로 남았다. 지금 규칙(4-3, D `98da703`)으로는 `failed` 다.

### 2-3. 엔진 쪽 (`out/`)

`room`(Stray)에서 실제로 본 파일 이름 + 코드에서 확인한 입력별 차이.

```
out/
  stages/NN_<단계>.json       단계별 판정 (3-2). NN = 00~07
  log.txt                     형상 메쉬 · 빌드 · 지도 하위 프로그램의 출력 (stages.py:23-27)
  input_map/                  Stray 입력일 때만 — Stray 를 맵 폴더 형식으로 바꾼 것 (6장)
    _unzip/                     zip 을 푼 원본 촬영 (rgb.mp4 · depth/ · odometry.csv …). 사람 얼굴이 든 영상이 그대로 남음
    keyframes/rgb/<n>.jpg · keyframes/depth/<n>.png · keyframes/calib/<n>.yaml
    trajectory_camera_optical.txt
    stray_meta.json
    stray_quality.json          Stray 입력이면 진단 단계가 씀 (`stray_quality.measure`, 통합 기록 R3 — C `620a2b0`). 진단 수치의 `stray_quality` 와 같은 값
  build/                      빌드 결과
    auto_config.yaml            엔진이 만든 빌드 설정
    auto_config_corrected.yaml  보정을 했을 때만 (stages.py:319)
    transform.json              1-3
    visual.ply                  보이는 형상 (장면 좌표)
    report.json                 빌드 보고 (엔진의 report.json 과 다른 파일)
    usd/<이름>_mesh.usdc · usd/<이름>_scene.usda
    usd/textures/               사진 텍스처가 있을 때만 (GLB). Stray 는 없음
    ground.npz                  바닥 높이 지도. GLB · Stray 는 바닥이 평평하지 않을 때만 (1-2). 있으면 진단 · 시작 자세가 씀
    (GLB · Stray) drive_path.json   가상 주행 경로 → bundle/route.csv 의 재료
    (GLB · Stray) glb_tex/          GLB 텍스처를 풀어 두는 곳. Stray 는 빈 폴더
    (Stray 만) tsdf_config.yaml · mesh_raw.ply · tsdf_meta.json   형상 메쉬와 그 설정 (stages.py:53-58, detect.config_tsdf)
    (맵 폴더만) mesh_raw.ply · collider.ply · floor.ply · boundary.ply · check_input.json · tsdf_meta.json …
  defects_before.json         보정 전 결함 목록 (5장). 진단 단계가 돌면 항상
  defects_after.json          보정 뒤 다시 진단한 결함 목록. **보정이 돌았을 때만** (5-4)
  bundle/                     Isaac Sim 용 묶음 (2-4)
  preview.glb                 화면용 GLB. 엔진이 묶음 단계 끝에 씀 (stages.py:420-425 → preview.py). 없으면 서버가 요청 때 만듦 (4-2)
  report.md                   사람이 읽는 보고서 (s_report, stages.py:478)
  report.json                 {name, verdict, stages} (stages.py:479, 3-4)
  bundle.zip                  서버가 처음 요청 때 만듦 (4-2)
```

- Stray 의 `build/` 는 11/24 에 바뀌었다 (C `0534d95`). `floor.ply` · `collider.ply` · `boundary.ply` · `check_input.json` 이 없어지고 `drive_path.json` · `glb_tex/` · `tsdf_config.yaml` 이 생겼다. `mesh_raw.ply` · `tsdf_meta.json` · `visual.ply` · `transform.json` 은 그대로 있다.
- 보정이 돌면 `build/` 는 **같은 자리에 다시 만들어진다** (보정 설정이 원래 설정을 `base:` 로 물려받고 `out:` 을 바꾸지 않음, `stages.py:319-329`).
  - 그래서 보정 뒤의 `build/` · `bundle/` · `preview.glb` 는 모두 보정된 것이다.
  - Stray 의 형상 메쉬(`mesh_raw.ply`)는 보정 때 다시 만들지 않는다 (입력 단계는 다시 돌지 않음).

### 2-4. 이름과 `bundle/`

**장면 이름** = `detect.safe_name(입력 경로)` (`detect.py:32-34`)

1. 파일 이름에서 확장자를 뺌.
2. `[A-Za-z0-9_]` 밖의 글자를 `_` 로 바꾸고 앞뒤 `_` 를 지움. 비면 `scene`.
3. 숫자로 시작하면 앞에 `m_` 를 붙임.
4. 소문자로.

| 입력 | 장면 이름 | USD 파일 |
|---|---|---|
| CLI `9a5813208f.zip` | `m_9a5813208f` | `m_9a5813208f_mesh.usdc` · `m_9a5813208f_scene.usda` |
| 웹으로 올린 모든 파일 (`input/input.zip` · `input/input.glb`) | `input` | `input_mesh.usdc` · `input_scene.usda` (`room` 의 `bundle.zip` 에서 확인) |

**`bundle/`** (`s_bundle`, `stages.py:342-426`) — `room` 실제 목록:

```
bundle/
  usd/input_mesh.usdc              build/usd/ 를 통째로 복사 (stages.py:364-366)
  usd/input_scene.usda
  usd/textures/…                   사진 텍스처가 있을 때만 (GLB). room 은 없음
  nav2/map.pgm · map.yaml · preview.png    sim/nav2/make_map.py 출력 (stages.py:131)
  scene_root.usda                  묶음만으로 열리는 씬 루트 — 맵 + 돔 조명 600 + (shadow_fix 면) 그림자 설정. 로봇 · 센서 없음
  scene_root_sim.usda              팀 시뮬 배치용 씬 루트 (11/24 전의 scene_root.usda) — 혼자서는 안 열림
  start_pose.json                  시작 자세 (장면 좌표)
  route.csv                        GLB · Stray (build/drive_path.json 이 있을 때, stages.py:357-363). 맵 폴더는 없음
  bundle.json                      {name, input, kind, files}. files 에 bundle.json 자신은 없음
```

```json
// bundle/bundle.json (room)
{"name": "input", "input": "input.zip", "kind": "glb",
 "files": ["nav2/map.pgm", "nav2/map.yaml", "nav2/preview.png", "route.csv", "scene_root.usda", "scene_root_sim.usda", "start_pose.json",
           "usd/input_mesh.usdc", "usd/input_scene.usda"]}

// bundle/start_pose.json (room) — z = 바닥 + 0.022, yaw 는 도
{"x": -4.106, "y": 1.389, "z": 0.02, "yaw_deg": 16.5, "clearance_m": 1.11, "goal_xy": [4.644, 3.989],
 "note": "빈 바닥에서 장애물과 가장 먼 자리 · 가장 먼 빈 자리를 향함. 높이 = 바닥 + 0.022 (Nova Carter)"}
```

```yaml
# bundle/nav2/map.yaml (room, 첫 줄 주석 줄임)
image: map.pgm
mode: trinary
resolution: 0.05
origin: [-7.581, -6.886, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

- `kind` 는 빌드가 간 길이다. **Stray 도 `glb`** 로 적힌다 (`stages.py:59` 에서 GLB 길로 바꿈, C `0534d95`). 11/23 까지 Stray 는 `map_folder` 였다. Stray 인지는 `input` 의 확장자(`.zip`)로만 알 수 있다.
- `route.csv` 는 머리줄 `x,y` + 장면 좌표 점들이다. 바닥 빈 곳의 뼈대로 만든 **가상 경로**이고, 촬영 경로가 아니다 (`pipeline/glb_to_usd.py:614`, `:622-624`). `room` 은 285점.
- **씬 루트가 둘이다** (C `620a2b0`, `stages.s_bundle`).
  - `scene_root.usda`: **묶음만으로 열린다.** subLayer 는 `./usd/<이름>_scene.usda` 하나. `/World/Lights/Dome`(DomeLight, 세기 600)을 직접 정의한다. 로봇 · 센서는 없다. [C 측정] usd-core 로 열면 못 찾는 층 0. Isaac Sim 에서는 아직 열어 보지 않았다.
  - `scene_root_sim.usda`: 11/24 전의 `scene_root.usda` 그대로 — `../proxy/*.usda` 와 `../../../data/usd/<이름>/usd/<이름>_scene.usda` 를 subLayer 로 건다. `sim/scenes/<장소>/` 에 두는 것을 가정하고, 혼자서는 안 열린다. 웹 작업은 `<이름>` 이 모두 `input` 이다.
  - 맵 파일(`usd/<이름>_scene.usda`)에는 조명이 없다. 맵만 열면 검게 보인다.
- 그림자 설정: 진단의 `shadow_fix` 가 참이면(3-5) 보이는 prim 마다 `primvars:doNotCastShadows = 1` 을 건다 (`stages.py:367-369`). `room` 은 `Visual` · `FloorFill` 2개.
- `bundle/` 구조는 D 의 `sim/add_site.py` 가 받는 형식에 **아직 맞추지 않았다** (8장).

---

## 3. 엔진

### 3-1. 실행과 종료 코드

```bash
cd recon
python -m scan2sim <입력> --out <출력 폴더> [--force] [--resume]
```

- 입력 (`detect.kind_of`, `detect.py:13-29`)
  - GLB 파일 (`.glb` · `.gltf`), 또는 GLB 가 딱 하나 든 폴더
  - Stray 촬영 폴더, 또는 그 폴더가 든 zip (촬영 폴더 판별은 6-1, `stray._zip_captures`). 맵 폴더 형식으로 바꾼 뒤 형상 메쉬를 만들어 GLB 길로 감 (1-2)
  - B 형식 맵 폴더 (`keyframes/` + `trajectory*camera_optical.txt`) — **폴더만**. 맵 폴더를 묶은 zip 은 알아보지 못함 (8장)
  - 녹화 파일(`.mcap` · `.db3` · `.bag`)은 알아보지만 입력 단계에서 실패시킴 (`stages.py:37-38`)
- `--force`: 실패한 단계가 있어도 다음 단계를 함. `--resume`: `out/build` 에 결과가 있으면 변환을 건너뜀 (`__main__.py:20-23`). **서버는 둘 다 쓰지 않는다** (`web/server.py:146`).
- 종료 코드 (`__main__.py:29`)
  - `0` = 어느 단계도 `fail` 이 아님 (통과 또는 주의)
  - `1` = 한 단계라도 `fail`
  - 단계 안의 `Exception` 은 그 단계의 `fail` 로 바뀐다 (`core.py:53-54`). 요약은 `"예외: <예외 이름>: <문장>"`, `messages` 는 traceback 마지막 6줄.
    - 단, `stray.py` 의 `CaptureError` 는 `input` 단계가 직접 받아 **그 문장만** 요약에 넣고 `messages` 는 비운다 (R4, C `620a2b0`; 6-1).
    - 단, `SystemExit` 은 잡히지 않아 단계 파일 없이 엔진이 끝난다 (종료 코드 1, 8-3 의 1번).
  - 인자가 틀리면 argparse 의 `2` [추정: argparse 기본 동작].

### 3-2. 단계 파일 `out/stages/NN_<단계>.json`

`core.run` 이 단계마다 하나씩 쓴다 (`core.py:41-64`). 파일 이름 = `f"{k:02d}_{name}.json"` (`core.py:56`).

```json
{"stage": "nav2", "status": "pass", "summary": "304×277 칸 (0.05 m) · 빈 바닥 61 m²",
 "metrics": {"cells": [304, 277], "resolution": 0.05, "free_m2": 61.4, "occupied_cells": 15338},
 "messages": [], "seconds": 0.8}
```

| 필드 | 형 | 뜻 |
|---|---|---|
| `stage` | 문자열 | 단계 이름 (3-3) |
| `status` | 문자열 | `pass` · `warn` · `fail` · `skip` (3-3) |
| `summary` | 문자열 | 한 줄 요약. 화면에 그대로 나감 |
| `metrics` | 객체 | 단계별 수치. 키는 단계마다 다름 (3-5). 비어 있을 수 있음 |
| `messages` | 문자열 배열 | 사용자가 읽을 문장. 예외면 traceback 마지막 6줄 (`core.py:54`) |
| `seconds` | 실수 | 걸린 시간 (소수 1자리) |

- `summary` 와 `messages` 는 화면에 그대로 나간다. 한국어, 사용자가 읽을 문장으로 쓴다.
- `status` 의 뜻 (`core.py:1-6`)
  - `pass` 통과
  - `warn` 쓸 수 있지만 볼 것이 있음. `Result.warn()` 은 `pass` 일 때만 `warn` 으로 바꿈
  - `fail` 여기서 멈춤 (이유는 `messages`)
  - `skip` 이번 실행에서 안 함 (이유는 `summary`)
- 한 단계가 `fail` 이면 그 뒤 단계는 `report` 를 빼고 모두 `skip` 이 되고, `summary` 는 `"앞 단계(<단계>)가 실패해서 하지 않음"` (`core.py:48-49`).

### 3-3. 단계 목록

순서 · 이름은 `stages.py:483`, 화면 이름(`label`)은 `web/server.py:35-36`.

| NN | `stage` | `label` | 하는 일 | 나올 수 있는 `status` |
|---|---|---|---|---|
| 00 | `input` | 입력 확인 | 입력 종류 판별, 형태 재기, 빌드 설정 만들기. Stray 면 맵 폴더로 바꾸고 형상 메쉬(`pipeline/tsdf.py`)까지 만듦 | pass · warn · fail (Stray 는 늘 warn 이상, 6-3) |
| 01 | `convert` | 메쉬 · USD 만들기 | GLB · Stray: `pipeline/glb_to_usd.py`. 맵 폴더: `check_input.py` + `run.py --from tsdf` | pass · fail |
| 02 | `nav2` | Nav2 지도 | `sim/nav2/make_map.py`. 빈 바닥 4 m² 미만이면 fail | pass · fail · skip(`make_map.py` 없음) |
| 03 | `diagnose` | 진단 | 충돌 바닥 층수 · 보이는 바닥 · 보이는 바닥 두 높이 · 낮은 턱 · 물체 일치 · 천장. `defects_before.json` 씀 | pass · warn · fail |
| 04 | `correct` | 보정 · 다시 진단 | 두 겹 바닥이 있을 때만 다시 빌드 · 다시 진단. `defects_after.json` 씀 | pass · warn · fail · skip |
| 05 | `bundle` | 묶음 만들기 | `bundle/` 만들기, 끝에 `preview.glb` 쓰기 | pass · warn |
| 06 | `verify` | 주행 검증 | 지금은 항상 skip (Isaac Sim 자리, `stages.py:429-431`) | skip |
| 07 | `report` | 보고서 | `report.md` · `report.json`. 앞 단계가 실패해도 항상 돎 | pass · warn (**fail 이 되지 않음**, `stages.py:480`) |

### 3-4. 엔진의 `report.json`

`stages.py:479`: `{"name": <장면 이름>, "verdict": "pass|warn|fail", "stages": [단계 dict …]}`

- `verdict` = `input` ~ `verify` 단계 중 가장 나쁜 것 (`fail` > `warn` > `pass`, `skip` 은 무시) (`stages.py:437`).
- `stages` 에는 `report` 단계 자신이 **없다** (쓰는 시점에 아직 결과가 없음).
- `report` 단계의 `status` 는 `verdict` 가 `fail` 이면 `pass` 로 바뀐다. 그래서 전체 판정은 `report` 단계가 아니라 `verdict` 를 본다.

### 3-5. 단계별 `metrics` 의 키 (지금 코드)

약속은 "있으면 이 이름" 이다. 화면이 쓰는 것은 `diagnose` 의 `samples` · `floor_single_pct` · `floor_double` · `visible_floor_missing_pct` 뿐 (`index.html:244-247` `renderCompare`).

| 단계 | 키 |
|---|---|
| `input` (GLB · Stray) | `parts` · `textured` · `triangles` · `extent_m` · `floor_tilt_deg` · `floor_area_m2` · `floor_spread_m` · `floor_main_share_pct` · `floor_flat` · `ceiling_area_m2` · `ceiling_over_floor_pct` · `has_ceiling` · `wall_top_m` (`detect.py:74-77`). Stray 면 여기에 `stray` (= `stray_meta.json` 전체, `stages.py:77`). Stray 는 형상 메쉬를 잰 값이다. 형상 메쉬를 못 만들면 `stray` 만 있음 (`stages.py:58`) |
| `input` (맵 폴더) | `main_keyframes` · `main_poses` · `extra_sets` · `geometry_mesh` · `has_validation` (`detect.py:91-92`). 진단이 같은 dict 에 `ceiling_over_floor_pct` · `has_ceiling` 을 더함 (`stages.py:295-298`) |
| `convert` | `mesh_mb` · `prims` ({prim 이름: 삼각형 수}) · `collision` · `clean`. 실패 시 `check_input` |
| `nav2` | `cells` [가로, 세로] · `resolution` · `free_m2` · `occupied_cells` |
| `diagnose` | `samples` · `floor_single_pct` · `floor_double` · `floor_double_pct` · `floor_under` · (`visual.ply` 가 있으면) `floor_two_levels` {samples, pct, gap_cm_median} · `visible_floor_missing_pct` · `visible_minus_collision_cm` {p5, median, p95} · `low_bumps` {samples_over_3cm_pct, runs_over_0p6m, longest} · `obstacle_match` {…} · `has_ceiling` · `shadow_fix` · `defects` ({type: 개수}) |
| `correct` (돌았을 때) | `fixed` (예: `["double_floor"]`) · `before` · `after` (각각 `diagnose` 의 metrics 와 같은 모양) |
| `bundle` | `start_pose` (= `start_pose.json`) · `files` (개수) · `shadow_overrides` · `preview` ({prims, triangles, triangles_in, mb, seconds, cell_m}, 못 만들면 {error}) |

- **단계 파일과 엔진 `report.json` 의 `input` metrics 가 다르다.** 진단 단계가 입력 단계의 dict 에 나중에 키를 더하는데, 입력 단계 파일은 그 전에 쓰였기 때문이다.
  - 모든 입력: `shadow_fix` (`stages.py:302`). 맵 폴더는 `ceiling_over_floor_pct` · `has_ceiling` 도 (`stages.py:295-298`).
  - 이 키들은 단계 파일 · 상태 응답의 `metrics.input` 에는 없고 엔진 `report.json` 에만 있다 (`room` 에서 확인).
- **천장 기준이 둘이다.**
  - `has_ceiling` = 빈 바닥 위 천장 덮음 > 30% (`detect.py:76`, 맵 폴더는 `stages.py:298`).
  - `shadow_fix` = `has_ceiling` 이거나 덮음 ≥ 10% (`stages.py:302`, C `ed35a87`). 참이면 씬 루트에 그림자 설정을 넣는다 (2-4).
  - `room` 은 덮음 26.8% 라 `has_ceiling` 거짓 · `shadow_fix` 참이다.
- `convert` 의 `prims` 이름
  - GLB: `Visual` (부분 하나) 또는 `Visual0` · `Visual1` … (부분이 여럿, `pipeline/glb_to_usd.py:530`) + `FloorFill` (`:606`) + `Collision` · `Boundary`.
  - Stray: 형상 메쉬가 한 덩어리라 `Visual` · `FloorFill` · `Collision` · `Boundary` (`room` 확인). 11/23 까지는 맵 폴더 길의 prim 이었다.
  - Stray 의 보이는 prim 은 **정점 색만** 있다 (사진 텍스처 없음).

`diagnose` 실제 예 (`room` 의 `stages/03_diagnose.json`):

```json
{"stage": "diagnose", "status": "warn",
 "summary": "빈 바닥 표본 2,061 · 바닥 한 겹 100.0% · 두 겹 0 · 보이는 바닥 없음 2.4%",
 "metrics": {"samples": 2061, "floor_single_pct": 100.0, "floor_double": 0, "floor_double_pct": 0.0, "floor_under": 0,
  "floor_two_levels": {"samples": 301, "pct": 14.6, "gap_cm_median": 7.5},
  "visible_floor_missing_pct": 2.4, "visible_minus_collision_cm": {"p5": -0.7, "median": 0.2, "p95": 2.3},
  "low_bumps": {"samples_over_3cm_pct": 3.03, "runs_over_0p6m": 2, "longest": [[2.3, 4.0, 2.4], [1.2, 4.4, 0.9]]},
  "obstacle_match": {"visible_cells": 14393, "collision_cells": 13612, "visible_in_collision_pct": 99.7,
                     "collision_in_visible_pct": 100.0, "tol_m": 0.1, "height_band_m": [0.12, 1.0]},
  "has_ceiling": false, "shadow_fix": true, "defects": {"floor_two_levels": 4, "low_bump": 2}},
 "messages": ["보이는 바닥이 두 높이로 겹친 자리가 빈 바닥의 14.6% (간격 중앙 7.5 cm) — 촬영 중 자세가 위아래로 틀어진 흔적일 수 있음. …",
              "보이는 바닥이 3~12 cm 솟은 자리가 0.6 m 넘게 이어진 곳 2군데 (가장 긴 것 2.3 m, 위치 (4.0, 2.4)) — …",
              "천장이 일부만 찍힘 (빈 바닥 위 27%) — 그 아래가 어두워지지 않게 씬 루트에 그림자 설정을 넣음"],
 "seconds": 1.8}
```

---

## 4. 웹 API

서버: `python -m scan2sim.web.server [--port 8000] [--host 127.0.0.1] [--jobs ~/scan2sim_jobs] [--code <접속 코드>]` (`web/server.py:262-273`).

**배포 (D, `deploy/`)** — 팀 ThinkPad 에서 켜는 방법. 약속에 닿는 것만 적는다.

- `deploy/start.sh` 가 서버를 `--host 127.0.0.1 --port $S2S_PORT --jobs $S2S_JOBS --code <코드>` 로 켠다 (`deploy/start.sh:34`). 기본 포트 8000, 작업 폴더 `~/scan2sim_jobs` (`deploy/common.sh:8-11`).
- **접속 코드를 늘 준다.**
  - 처음 켤 때 6자리 숫자를 만들어 `~/scan2sim_run/code` (권한 600)에 두고 계속 쓴다. `S2S_CODE=<코드>` 로 정할 수 있다 (`deploy/start.sh:18-21`).
  - 주소 · 코드는 `deploy/status.sh` 가 보여 준다.
- 밖에서는 Cloudflare 임시 터널(`cloudflared tunnel --url http://127.0.0.1:<포트>`, `deploy/start.sh:53`)로 들어온다.
  - 주소 `https://<이름>.trycloudflare.com` 은 켤 때마다 바뀐다. `S2S_TUNNEL=0` 이면 터널 없이 이 컴퓨터에서만.
- 서버가 죽으면 2초 뒤 다시 켠다 (`deploy/start.sh:31-37`). 그때 처리 중이던 작업은 `failed` · `"서버가 다시 켜져서 끊김"` 이 된다 (4-3).
- 환경: Python 3.12 가상 환경 `~/scan2sim_env` 에 `recon/requirements.txt` 를 설치 (`deploy/install.sh`).

### 4-1. 공통

- **접속 코드:** `--code` 를 주면 `/api/` 로 시작하는 모든 요청에 코드가 있어야 한다 (`web/server.py:72-75`).
  - 머리말 `X-Code: <코드>` 또는 쿼리 `?code=<코드>`. 머리말이 있으면 머리말만 본다.
  - 없거나 틀리면 `401` `{"error": "접속 코드가 맞지 않음"}`.
  - `GET /` 와 `/static/…` 은 코드를 보지 않는다.
  - 화면은 `fetch` 에는 머리말을 (`index.html:149`), `<img>` · GLB 로더 · 내려받기 링크에는 `?code=` 를 붙인다 (`index.html:157`).
- **오류 응답:** 400 · 401 · 409 는 JSON `{"error": "문장", …}`.
  - **404 는 JSON 이 아니라 Flask 기본 HTML** 이다 (`abort(404)`). 클라이언트는 상태 코드만 보고 판단한다.
  - 작업 번호가 `^[0-9a-f]{12}$` 가 아니거나(`web/server.py:47-48`), 그 작업이 메모리에도 `job.json` 에도 없으면 404 (`:58-59`).
- **진행 상태는 화면이 `GET /api/jobs/<id>` 를 1.5초마다 불러서 본다** (`index.html:202`). SSE · WebSocket 없음.
- 응답 JSON 의 키 순서는 알파벳 순이다 (Flask 기본). 순서에 기대지 않는다.

### 4-2. 경로

| 메서드 · 경로 | 요청 | 성공 응답 | 실패 |
|---|---|---|---|
| `GET /` | — | `index.html` | — |
| `GET /api/info` | — | `{"ok": true, "stages": [{"name", "label"} ×8], "accept": [".zip", ".glb"]}` (`:83-85`) | 401 |
| `POST /api/jobs` | JSON `{"name": 파일 이름, "size": 바이트}` (`Content-Type` 은 안 봄) | `{"id": "<12자리>"}` (`:88-103`) | 400 `{"error": "받는 파일은 .zip · .glb 임"}` (확장자, 대소문자 무시) · 400 `{"error": "파일 크기가 0 이거나 4 GB 를 넘음"}` · 401 |
| `PUT /api/jobs/<id>/chunk?offset=N` | 본문 = 파일 조각의 날것 바이트. `N` = 지금까지 받은 바이트 수 | `{"received": 누적 바이트}` (`:106-123`) | 409 `{"error": "조각 순서가 맞지 않음", "received": n}` (상태가 `uploading` 이 아니거나 `N ≠ received`) · 400 `{"error": "알린 크기보다 많이 옴"}` · 401 · 404 |
| `POST /api/jobs/<id>/start` | 본문 없음 | `{"status": "queued"}` (`:126-134`) | 409 `{"error": "파일이 다 오지 않음", "received": n}` (상태가 `uploading` 이 아니거나 `received ≠ size` — 이미 넣은 작업을 다시 넣어도 이 문장) · 401 · 404 |
| `GET /api/jobs/<id>` | — | 작업 상태 (4-3) (`:173-196`) | 401 · 404 |
| `GET /api/jobs/<id>/preview.glb` | — | `model/gltf-binary`. 화면 좌표 (1-1). 엔진이 써 둔 `out/preview.glb` (`:199-216`) | 404 (파일이 없고 USD 도 없음, 또는 서버가 만들다 실패) · 401 |
| `GET /api/jobs/<id>/map.png` | — | `image/png`. `bundle/nav2/map.pgm` 을 PNG 로 (`:219-229`) | 404 · 401 |
| `GET /api/jobs/<id>/report.md` | `?dl=<아무 값>` 이면 내려받기 | `text/markdown; charset=utf-8`. 내려받기 이름 `scan2sim_report_<id>.md` (`:232-238`) | 404 · 401 |
| `GET /api/jobs/<id>/bundle.zip` | — | 내려받기 `scan2sim_<id>.zip` (`:241-259`) | 404 (`bundle/usd/` 없음) · 401 |

**`preview.glb`**

- **만드는 곳은 엔진**이다. 묶음 단계 끝에 `build/usd/<이름>_mesh.usdc` 로 `out/preview.glb` 를 쓴다 (`stages.py:420-425` → `preview.write`, `preview.py:46-116`, C `2a6e89f`).
  - 못 만들면 묶음 단계 `metrics.preview` 에 `{"error": …}` 와 메시지가 남고, 묶음은 그대로 나온다 (`stages.py:423-425`).
- **서버는 주기만 한다** (D `ed7220e`).
  - `out/preview.glb` 가 없거나 `build/usd/*_mesh.usdc`(이름순 첫 파일)보다 오래됐을 때만 같은 엔진 함수로 한 번 만들어 둔다 (`web/server.py:207-213`). 예: 묶음 단계 전에 멈춘 작업, `preview.py` 가 생기기 전의 작업 폴더.
  - 한 서버 안에서는 잠금(`PREVIEW_LOCK`)으로 한 번에 하나만 만든다.
  - 만들다 실패하면 서버 로그에 경고만 남기고, 파일이 없으면 404.
  - 서버가 만들 때는 서버 환경에 엔진 패키지(`pxr` · `trimesh` · `numpy` · `PIL`)가 있어야 한다. 엔진이 써 둔 파일을 줄 때는 필요 없다.
  - 11/23 판에 있던 `build/visual.ply` 대체(색 없음)는 없어졌다. USD 가 없으면 404 다.
- 내용 (`preview.py:54-111`)
  - USD 의 Mesh prim 중 `CollisionAPI` 가 있거나 안 보이는 것(`Collision` · `Boundary`)은 뺀다. glTF 노드 이름 = prim 이름.
  - 사진 텍스처(정점 단위 `st`)가 묶인 prim: 형상은 그대로 두고 텍스처만 긴 변 2048 화소 안으로 줄여 JPEG 로 넣는다.
  - 정점 색(`displayColor`) prim: 텍스처 prim 몫을 뺀 삼각형 합이 30만을 넘으면 정점을 격자 칸으로 모아 줄인다 (칸 3 cm 부터 1.25배씩).
  - 무작위를 쓰지 않는다. 같은 입력이면 같은 파일 [C 측정, `C_TRACK.md` 2-3].
  - `room`: 삼각형 219만 → 23.1만 (칸 5.9 cm) · 4.7 MB · 2.0초 (묶음 단계 `metrics.preview`).
- 보정이 돌았으면 **보정 뒤 메쉬**다. 화면에서 "보정 전" 결함을 골라도 메쉬는 보정 뒤 것.

**`bundle.zip`**

- 안의 구조: `bundle/…` (2-4 의 `bundle/` 전체) + 맨 위에 `report.md` · `defects_before.json` · `defects_after.json` (있는 것만).
- `out/bundle.zip` 에 한 번 만들면 **다시 만들지 않는다** (`web/server.py:249`).
- `room` 실제: 파일 11개 (`bundle/` 9개 + `report.md` + `defects_before.json`).

**올리기 순서 (화면이 하는 것, `index.html:179-187`)**

1. `POST /api/jobs {name, size}` → `id`
2. `offset = 0, 8 MiB, 16 MiB …` 차례로 `PUT …/chunk?offset=…`. 조각 크기 8 MiB (`index.html:144`)
3. `POST …/start`
4. 주소에 `#<id>` 를 붙이고 `GET /api/jobs/<id>` 를 1.5초마다 부름. `done` · `failed` 면 멈춤

- 이어 올리기: 서버는 받은 만큼(`received`, 409 응답에도 들어 있음)부터 다시 받는다. 단, 같은 서버 프로세스 안에서만 된다 (재시작하면 `uploading` 작업은 `failed`). **화면은 아직 이어 올리기를 하지 않는다.**
- `deploy/check.py` 도 같은 순서다. 조각 크기는 `--chunk-mb` 로 바꿀 수 있다 (기본 8).

### 4-3. 작업 상태 (`GET /api/jobs/<id>`)

`job.json` 의 필드 (2-2) 전부 + 아래 필드 (`web/server.py:173-196`).

| 필드 | 형 | 언제 | 뜻 |
|---|---|---|---|
| `stages` | 배열 (8개, 3-3 순서) | 항상 | `{name, label, status}` + 결과 파일이 있으면 `summary` · `messages` · `seconds` |
| `queue_ahead` | 정수 | 항상 | `queued` 일 때만 계산, 그 밖은 0. 나보다 먼저 줄에 선 `queued` · `running` 작업 수 (**처리 중인 것도 셈**) |
| `defects_before` | 객체 또는 `null` | `done` · `failed` | `defects_before.json` 내용 (5장). 파일이 없으면 `null` |
| `defects_after` | 객체 또는 `null` | `done` · `failed` | `defects_after.json` 내용. **보정을 안 했으면 `null`** |
| `verdict` | 문자열 | `done` · `failed` | 엔진 `report.json` 의 `verdict` (`pass` · `warn` · `fail`). 없으면 `report` 단계의 `status` (8-3 의 2번: `pending` 이 나올 수 있음) (`:191`) |
| `has_bundle` | 참거짓 | `done` · `failed` | `out/bundle/usd/` 가 있음 = `bundle.zip` 을 받을 수 있음 |
| `metrics` | 객체 | `done` · `failed` | `{"before": …, "after": …, "input": …}` (아래) |

- `metrics.before` = 보정이 돌았으면 `correct` 단계의 `metrics.before`, 아니면 `diagnose` 단계의 `metrics`. 둘 다 없으면 `{}`.
- `metrics.after` = `correct` 단계의 `metrics.after` 또는 `null`.
- `metrics.input` = `input` 단계 파일의 `metrics` 또는 `null` (3-5: `shadow_fix` 는 여기에 없음).

**작업 `status`**

| 값 | 뜻 |
|---|---|
| `uploading` | 자리를 만들고 조각을 받는 중 |
| `queued` | 줄에 들어감 |
| `running` | 엔진이 돌고 있음 (한 번에 한 건) |
| `done` | 엔진이 끝났고, `out/report.md` 가 있고, **종료 코드가 0 이고, 단계 파일에 `fail` 이 없음** (`web/server.py:149-150`, D `98da703`). 그래서 `verdict` 는 `pass` 또는 `warn` |
| `failed` | 그 밖의 끝남. ① 단계가 하나라도 `fail` — 엔진 종료 코드 1, `verdict` `fail`, `report.md` · 결함 목록은 있음. ② 엔진이 중간에 죽음 — `report.md` 없음 (8-3 의 1번). ③ 엔진을 못 띄움 (`error`). ④ 서버가 다시 켜져서 끊김 (`error`) |

- 11/23 까지는 `report.md` 만 있으면 `done` 이었다. 그래서 단계가 실패해도 `done` 이 됐다 (`e2e_glb`: `done` · `exit_code 1` · `verdict fail`). 지금은 ① 이 `failed` 다 (통합 기록 R2).
- `failed` 여도 엔진이 만든 것(단계 결과 · 결함 목록 · `report.md` · 있으면 `preview.glb` · `map.png`)은 그대로 받을 수 있다. 화면은 `failed` 면 "실패" 로 적고, 3D · 결함 목록은 같이 보여 준다 (`index.html` `finish`).
- 서버를 다시 켜면 `uploading` · `queued` · `running` 이던 작업은 읽을 때 `failed` + `error: "서버가 다시 켜져서 끊김"` 이 된다 (`web/server.py:61-62`). 이것은 메모리에서만 바뀌고 `job.json` 에는 다시 쓰지 않는다.

**`stages[].status`**

- 3-2 의 `pass` · `warn` · `fail` · `skip` 에 더해 두 값이 있다 (`web/server.py:177-185`).
  - `pending` = 아직 결과 파일이 없음
  - `running` = 작업이 `running` 이고, 결과 파일이 없는 첫 단계
- `uploading` · `queued` 일 때는 단계 파일을 읽지 않으므로 8개 모두 `pending`.
- `done` · `failed` 여도 결과 파일이 없는 단계는 `pending` 으로 나온다 (엔진이 중간에 죽은 경우).
- `pending` · `running` 단계에는 `summary` · `messages` · `seconds` 가 없다.

실제 예 — `room` 작업 폴더를 서버의 `status()` 로 읽은 응답 (Flask 시험 클라이언트로 부름). 읽기 쉽게 키 순서를 바꾸고 배열 · 문장을 줄였다.

```json
{
 "id": "694ddc986821", "name": "6bc1ae021f.zip", "size": 379159774, "received": 379159774,
 "status": "done", "created": 1795504598.078851, "file": "input.zip",
 "queued_at": 1795504598.7145457, "started": 1795504598.7151208, "exit_code": 0, "finished": 1795504680.2617059,
 "queue_ahead": 0,
 "stages": [
  {"name": "input", "label": "입력 확인", "status": "warn", "seconds": 49.8,
   "summary": "아이폰 원본 → 형상 메쉬 · 천장 없음 · 바닥 평평하지 않음 · 범위 11×10 m",
   "messages": ["바닥의 17% 가 다른 높이에 찍힘 (높이 퍼짐 0.195 m) — 충돌 바닥은 스캔 바닥 높이를 따라 깖. 어디가 어긋났는지는 진단 단계에서 봄",
                "아이폰 자세에는 되돌아온 자리를 맞추는 보정이 없음 — 긴 구간은 틀어질 수 있음 (얼마나인지는 재지 않음)"]},
  {"name": "convert", "label": "메쉬 · USD 만들기", "status": "pass", "summary": "USD 50.15 MB · prim 4개", "messages": [], "seconds": 26.2},
  {"name": "nav2", "label": "Nav2 지도", "status": "pass", "summary": "304×277 칸 (0.05 m) · 빈 바닥 61 m²", "messages": [], "seconds": 0.8},
  {"name": "diagnose", "label": "진단", "status": "warn", "summary": "빈 바닥 표본 2,061 · …", "messages": ["…"], "seconds": 1.8},
  {"name": "correct", "label": "보정 · 다시 진단", "status": "skip", "summary": "진단에 걸린 항목이 없어 보정하지 않음", "messages": [], "seconds": 0.0},
  {"name": "bundle", "label": "묶음 만들기", "status": "pass", "summary": "묶음 파일 8개 · 시작 (-4.106, 1.389) 16.5° · 여유 1.11 m", "messages": [], "seconds": 2.0},
  {"name": "verify", "label": "주행 검증", "status": "skip", "summary": "Isaac Sim 이 필요함 — …", "messages": [], "seconds": 0.0},
  {"name": "report", "label": "보고서", "status": "warn", "summary": "report.md — 올릴 수 있으나 아래 주의 항목을 볼 것", "messages": [], "seconds": 0.0}
 ],
 "verdict": "warn",
 "has_bundle": true,
 "metrics": {
  "before": {"samples": 2061, "floor_single_pct": 100.0, "floor_double": 0,
             "floor_two_levels": {"samples": 301, "pct": 14.6, "gap_cm_median": 7.5},
             "visible_floor_missing_pct": 2.4, "shadow_fix": true, "…": "…"},
  "after": null,
  "input": {"parts": 1, "textured": 0, "triangles": 2284955, "extent_m": [11.2, 9.81, 3.97], "floor_tilt_deg": 0.38,
            "floor_flat": false, "floor_main_share_pct": 82.6, "has_ceiling": false, "ceiling_over_floor_pct": 26.8, "…": "…",
            "stray": {"source": "6bc1ae021f", "keyframes": 681, "…": "…"}}
 },
 "defects_before": {"phase": "before", "T_input_to_scene": [["…"]], "up_input": [0.00239, -0.00622, 0.99998],
                    "floor_point_input": [-4.5098, 2.3223, -1.5549], "defects": [{"id": "D1", "type": "floor_two_levels", "…": "…"}]},
 "defects_after": null
}
```

처리 중(`running`)이면 `verdict` · `has_bundle` · `metrics` · `defects_*` 가 없고, `stages` 는 예를 들어 `input warn · convert running · nav2 pending · …` 처럼 나온다.

---

## 5. 결함 목록 (`defects_before.json` · `defects_after.json`)

`stages._defects` 가 쓴다 (`stages.py:158-210`). 진단 단계(`s_diagnose`)의 끝에서 불린다 (`stages.py:307`).

### 5-1. 파일

| 필드 | 형 | 뜻 |
|---|---|---|
| `phase` | `"before"` · `"after"` | 보정 전 / 보정 뒤 다시 진단 (`stages.py:204`) |
| `T_input_to_scene` | 4×4 수 배열 | `build/transform.json` 의 값. 없으면 단위행렬 (1-3) |
| `up_input` | [3] 수 | 장면의 +Z 를 입력 좌표로 나타낸 단위 벡터 = `T⁻¹` 의 셋째 열 (소수 5자리) |
| `floor_point_input` | [3] 수 | 장면 원점 (0, 0, 0) 을 입력 좌표로 나타낸 점 = `T⁻¹` 의 이동 부분 (소수 4자리). 장면 바닥 z = 0 위의 한 점 |
| `defects` | 배열 | 결함 (5-2). 비어 있을 수 있음 |

### 5-2. 결함 하나

| 필드 | 형 | 뜻 |
|---|---|---|
| `id` | 문자열 | `D1`, `D2` … 정렬한 뒤 순서대로 매김 (`stages.py:201`). 한 파일 안에서만 유일. **보정 전과 뒤의 번호는 서로 관계없음** |
| `type` | 문자열 | `double_floor` · `no_visible_floor` · `low_bump` · `floor_two_levels` (5-4) |
| `title` | 문자열 | 화면에 보일 이름 (한국어) |
| `severity` | `"fail"` · `"warn"` | `double_floor` 만 `fail` |
| `fixable` | 참거짓 | `true` = 자동 보정 대상 (`double_floor` 만) |
| `center_scene` | [x, y, z] 수 | 장면 좌표의 중심. x · y = 묶인 표본의 평균, z = 그 자리의 바닥 높이(`ground.height`, 1-2) (소수 3자리) |
| `center_input` | [x, y, z] 수 | `T⁻¹ · center_scene` (소수 4자리). 입력 좌표 (1-3) |
| `radius_m` | 수 | 중심에서 가장 먼 표본까지 거리 **+ 지도 2칸(0.1 m)** (`checks.py:43`) |
| `extent_m` | 수 | 묶인 표본의 x · y 범위 상자의 대각선 **+ 지도 1칸(0.05 m)** (`checks.py:44`) |
| `samples` | 정수 | 묶인 표본 수 |
| `basis` | 문자열 | 판정 근거 문장 |
| `action` | 문자열 | 처리 방법 문장 |
| `near_low_bump` | 참 | **있을 때만** (C `3ea14ff`). `double_floor` 가 `low_bump` 결함과 붙어 있으면 `true` (값은 늘 `true`, 아니면 키가 없음). `basis` 끝에 "낮은 턱 의심 자리와 붙어 있음 — …" 이 덧붙음 (`stages.py:188-192`) |

- **정렬:** `fail` 먼저, 그 안에서 `samples` 가 많은 순 (`stages.py:193`).
- **최대 30개:** 정렬한 뒤 앞 30개만 남김 (`stages.py:158` `top=30`, `:194`).
- 화면은 `center_scene` · `radius_m` · `extent_m` · `severity` · `fixable` · `title` · `basis` · `action` · `id` 를 쓴다.
- "붙어 있음" = 두 결함 중심 사이 거리 < 두 `radius_m` 의 합.

실제 예 (`room` 의 `defects_before.json`, 결함 6개 중 1개):

```json
{
 "phase": "before",
 "T_input_to_scene": [[0.999997, 7.4e-06, -0.00239, 4.50604], [7.4e-06, 0.99998, 0.00622, -2.31259], [0.00239, -0.00622, 0.99998, 1.58008], [0.0, 0.0, 0.0, 1.0]],
 "up_input": [0.00239, -0.00622, 0.99998],
 "floor_point_input": [-4.5098, 2.3223, -1.5549],
 "defects": [
  {
   "type": "floor_two_levels",
   "title": "바닥이 두 높이로 찍힘 (재촬영 후보)",
   "severity": "warn",
   "fixable": false,
   "radius_m": 0.77,
   "extent_m": 1.54,
   "samples": 63,
   "basis": "빈 바닥 표본 63개(약 0.6 m²)에서 보이는 바닥 아래 7 cm 쯤에 바닥 면이 한 겹 더 있음. 촬영 중 자세가 위아래로 틀어지면 같은 바닥이 두 높이로 찍힘",
   "action": "고치지 않음 — 어느 높이가 실제 바닥인지 스캔만으로는 알 수 없음. 다시 찍을 후보",
   "id": "D1",
   "center_scene": [1.884, -1.835, -0.015],
   "center_input": [-2.6258, 0.4875, -1.5854]
  }
 ]
}
```

(`T_input_to_scene` 은 읽기 쉽게 줄였다. 실제 파일은 전체 자릿수.) 위 값은 1-3 을 그대로 따른다: `center_input` 의 z(−1.5854) ≈ 장면 바닥(−0.015) − 1.580 + 기울기 몫.

### 5-3. 표본과 묶기

- **표본** (`stages._free_xy`, `stages.py:147-155`, 부르는 곳 `:220`)
  - Nav2 지도의 빈 칸 중 장애물 · 미지 칸에서 0.3 m(`ROBOT_RADIUS_M`, `stages.py:20`) 넘게 떨어진 칸의 중심.
  - 그 칸들을 행 우선 순서로 세서 4개마다 하나 (`step=4`).
  - 표본 하나가 대표하는 넓이를 `4 × 칸²` 로 친다 (= 0.01 m², `stages.py:163`).
  - 표본이 50개 미만이면 진단 단계가 `fail` (`stages.py:221-222`).
- **위에서 쏘는 광선:** 표본마다 그 자리 바닥 높이 + 1.0 m 에서 아래로 (`stages.py:226`, `:246`).
- **묶기** (`checks.clusters`, `checks.py:26-45`)
  - 걸린 표본을 Nav2 지도 칸에 찍고, `join_cells` 칸 넓혀(`binary_dilation`) 8방향으로 이어진 것끼리 한 덩어리.
  - `join_cells` 는 기본 2칸(0.1 m). **`double_floor` 만 `round(0.5 / 칸)` = 10칸(0.5 m)** 이라 표본 사이가 약 1 m 안이면 한 결함이 된다 (`stages.py:165`, C `3ea14ff`). 얇은 판 · 턱의 가장자리가 여러 결함으로 쪼개지지 않게 하려는 것.
  - 덩어리마다 `center` · `radius_m` · `extent_m` · `samples`.

### 5-4. 결함 종류 (지금 구현의 기준값)

| `type` | `severity` · `fixable` | 걸리는 표본 | 결함으로 남기는 덩어리 | 근거 위치 |
|---|---|---|---|---|
| `double_floor` 충돌 바닥 두 겹 | `fail` · `true` | 충돌 prim `Collision` 에 쏜 광선이 맨 위 면을 만난 뒤, 그 바로 아래(1e-4 m)에서 다시 쏜 광선이 **3 cm 안**에서 면을 또 만남 | 모든 덩어리 (크기 하한 없음). 약 1 m 안은 하나로 묶음 (5-3) | `tools/check_floor_layers.py:57-60`, `stages.py:165-168` |
| `no_visible_floor` 보이는 바닥 없음 | `warn` · `false` | `build/visual.ply` 에 쏜 광선이 안 맞거나, 맞은 높이가 충돌 바닥과 **0.3 m 이상** 차이 | `samples × 0.01 m² ≥ 0.25 m²` (= 표본 25개 이상, 칸 0.05 m 기준) | `stages.py:249`, `:170-174` |
| `low_bump` 낮은 턱 의심 | `warn` · `false` | 보이는 바닥이 충돌 바닥보다 **3 cm 초과 12 cm 미만** 높음 | `extent_m ≥ 0.6` | `stages.py:272`, `:175-179` |
| `floor_two_levels` 바닥이 두 높이로 찍힘 (C `ed35a87`) | `warn` · `false` | 보이는 바닥이 있는 표본(`no_visible_floor` 가 아닌 것)에서, `visual.ply` 의 맨 위 면 1 mm 아래에서 다시 쏜 광선이 **3 cm 초과 30 cm 미만** 아래에서 수평에 가까운 면(법선 z 성분의 절댓값 > 0.7)을 또 만남 | `samples × 0.01 m² ≥ 0.25 m²` | `stages.py:250-257`, `:180-187` |

- `no_visible_floor` · `low_bump` · `floor_two_levels` 는 `build/visual.ply` 가 있을 때만 찾는다 (`stages.py:242`).
- **`double_floor` 와 `floor_two_levels` 는 다른 것이다.**
  - `double_floor` 는 **충돌** 메쉬가 3 cm 안에서 겹친 것이다. 물리 엔진을 멈출 수 있어 자동으로 보정한다.
  - `floor_two_levels` 는 **보이는** 메쉬에 바닥이 3~30 cm 아래로 한 겹 더 있는 것이다. 촬영 중 자세가 위아래로 틀어진 흔적으로 보고, 재촬영 후보로 둔다.
  - `floor_two_levels` 의 `basis` 에 나오는 간격은 그 덩어리 안 표본의 간격 중앙값(cm)이다.
- 진단 단계 `metrics.floor_two_levels` = `{samples, pct, gap_cm_median}` (`stages.py:257`). 빈 바닥 표본의 1% 를 넘으면 진단이 `warn` (`stages.py:258-260`). 보정 대상은 아니다.
- `diagnose` metrics 의 `low_bumps.runs_over_0p6m` 은 **다른 방법**(지도에서 이은 영역의 상자 대각선)으로 센 값이다 (`stages.py:274-282`). 결함 목록의 `low_bump` 개수와 다를 수 있다.
- `low_bump` 의 `basis` 문장은 11/24 에 "충돌 바닥은 평평해서" → "충돌 바닥에는 이 솟음이 없어" 로 바뀌었다 (C `3ea14ff`). 문장은 약속이 아니다.

**보정 (`s_correct`, `stages.py:312-339`)**

- 진단에서 `double_floor` 가 하나라도 나오면 돈다 (`ctx["needs"]`, `stages.py:236`). 다른 종류로는 돌지 않는다.
- 하는 일
  - GLB · **Stray**: 충돌 메쉬에서 바닥 위 5 cm 아래를 지움 (`collision.low_h_m: 0.05`, `stages.py:322-324`). Stray 는 11/24 부터 이쪽이다 (GLB 길로 가므로).
  - 맵 폴더: 바닥 위 10 cm 안의 낮은 조각과 6 cm 안의 수평 면을 지움 (`single_floor.collision_low_h_m: 0.10`, `collision_flat_h_m: 0.06`, `stages.py:325-327`)
- 그 뒤 `convert` → `nav2` → `diagnose` 를 다시 돌린다 (`stages.py:330-334`). 이때 `defects_after.json` 을 쓴다.
- **`defects_after.json` 은 보정이 돌았을 때만 생긴다.** 보정 단계가 `skip` 이면 없고, 상태 응답의 `defects_after` 는 `null`.
- 보정 뒤에도 두 겹 바닥이 남으면 `correct` 단계가 `fail` (`stages.py:231-232`). 그러면 작업 상태는 `failed` 다 (4-3).

---

## 6. Stray Scanner 입력 (`stray.py`)

### 6-1. 받는 것

- 촬영 폴더에 `rgb.mp4` · `odometry.csv` · `camera_matrix.csv` · `depth/` 가 있어야 한다 (`stray.NEED`, `stray.py:49`). 넷 중 둘 이상이 있으면 촬영 폴더로 보고(`stray._looks_stray`), 빠진 것을 이름으로 알려 주며 실패한다.
- zip 은 풀지 않고 이름만 보고 촬영 폴더를 찾는다 (`stray._zip_captures`). macOS 가 넣는 `__MACOSX/` · `._*` 는 무시한다.
- 처리할 때는 `out/input_map/_unzip/` 에 풀고 그 안의 촬영 폴더를 쓴다 (`stages.py:52`). 폴더가 몇 겹 아래에 있어도 된다.
- 풀기 전에 막는 zip (`stray.find_capture`, `stray.py:85-117`): 깨진 zip, 촬영 폴더가 여러 개, 파일 10만 개 또는 풀면 6 GB 넘음(`ZIP_MAX_FILES` · `ZIP_MAX_BYTES`, `stray.py:55`), 폴더 밖을 가리키는 경로.
  - 입력 판별(`detect.kind_of`, `detect.py:16`)도 이 함수를 부르므로, 깨진 zip 은 판별 때 이미 `CaptureError` 가 난다.
- **입력 검사** (`stray.validate`, `convert` 첫머리). 실패면 `stray.CaptureError` 를 낸다.
  - `input` 단계가 이 예외를 받아 `fail` 로 끝내고, **요약은 그 문장 하나**, `messages` 는 비어 있다 (`stages.s_input`, 통합 기록 R4 — C `620a2b0`). 종류 판별(`detect.kind_of`) 때 난 것도 같다.
  - [C 측정] 깨진 zip → 요약 "zip 파일을 열 수 없음 — …" 한 문장.

| 검사 | 실패 | 주의만 |
|---|---|---|
| 촬영 길이 | 2 초 또는 100 장보다 짧음 (`MIN_S` · `MIN_ROWS`) | — |
| `odometry.csv` | 열이 13개보다 적음 · 숫자 아님 · NaN · 사원수 길이가 1 에서 1% 넘게 벗어남 · 시각이 커지지 않음 (줄 번호와 함께) | — |
| `depth/` · `confidence/` 장 수 | 자세 줄 수와 3장 넘게 다름 | 1~2장 다름 (`COUNT_TOL`), `confidence/` 없음(신뢰도로 거르지 않음) |
| `rgb.mp4` | 열 수 없음 · 장 수가 자세 줄 수의 90% 미만 (`VIDEO_MIN`) · 4:3 이 아님 | — |
| 내부 파라미터 | `camera_matrix.csv` 가 3×3 숫자가 아님 · fx · fy ≤ 0 · 주점이 영상 밖 | — |
| 키프레임 수 | — | 2,000장을 넘어 간격을 넓힘 (6-2) |

- 주의는 `stray_meta.json` 의 `checks.warnings` 에 남고 (6-3), `input` 단계의 주의로도 나간다 (R4, C `620a2b0`).

### 6-2. 변환 값 (`stray.convert` 기본값, `stray.py:272`; 엔진은 기본값만 씀)

| 항목 | 값 | 뜻 |
|---|---|---|
| `width` | 없음 = 깊이 원래 크기 | 키프레임을 깊이 원래 크기(아이폰 256×192)로 저장. 색은 1920 → 960 → 256 두 번에 나눠 줄임. 내부 파라미터는 `camera_matrix.csv` 를 화소 중심 기준으로 줄임: `c' = (c + 0.5)·s − 0.5`, `f' = f·s` (`stray_quality._intr` 와 같음). `--width 960` 을 주면 전처럼 960×720 (깊이는 최근접으로 키움) — 사진 텍스처를 구울 때만 필요. 11/24 B 측정: 960 과 메쉬 차이 중앙 0.4 mm, 합치기 약 2배 빠름 |
| `min_conf` | 1 | 신뢰도(0 · 1 · 2) 1 미만 깊이는 0 으로 (`stray.py:326`) |
| `max_depth_m` | 5.0 | 5 m 넘는 깊이는 0 으로 (`stray.py:327`) |
| `move_m` · `turn_deg` | 0.05 · 5.0 | 앞에 고른 장에서 5 cm 넘게 움직였거나 5° 넘게 돌았을 때 키프레임으로 고름 (`stray.select`, `stray.py:256-269`) |
| `max_keyframes` | 2000 (`MAX_KEYFRAMES`) | 고른 장이 이보다 많으면 `move_m` · `turn_deg` 를 함께 넓혀 상한에 가장 가깝게 다시 고름 (이분 탐색, 최대 32배). 넓히면 `checks.warnings` 에 주의. 11/24 전에는 800장 · 1.2배씩이었음 |
| `max_speed` | 1.5 m/s | 5프레임 평균 속력(`stray.py:296`)이 이보다 빠른 장은 뺌 (흐림) |
| `skip_s` | 1.0 | 촬영 첫 1초는 건너뜀 (시작 직후 자세가 튐, `stray.py:297`) |
| 색 저장 | JPEG 품질 92, 영역 평균 축소 | (`stray.py:330`) |
| 깊이 저장 | 16비트 PNG(mm) | 기본은 원래 크기 그대로. `--width` 로 키울 때만 **최근접** 확대 (섞으면 가장자리에 없는 점이 생김) |
| 자세 | `Y_UP_TO_Z_UP · T_stray` | 1-3 (`stray.py:335`) |

- 형상 메쉬 (`detect.config_tsdf`, `detect.py:130-137` → `pipeline/tsdf.py`, `stages.py:53-58`)
  - TSDF 칸 2 cm, 깊이 3 m 까지, 키프레임 모두(`frame_step: 1`). 결과는 `build/mesh_raw.ply` · `build/tsdf_meta.json`.
  - 변환은 5 m 까지 남기지만 메쉬에는 3 m 까지만 들어간다.
  - `pipeline/tsdf.py` 의 `tsdf.image_step`(그림을 줄여 읽기, C `39c02e6`)은 선택 사항이고 엔진은 쓰지 않는다.
  - 형상 메쉬를 못 만들면 입력 단계 `fail` · `"촬영본에서 형상을 만들지 못함"`.
- 쓸 수 있는 장이 2장 미만이면 `CaptureError` (`stray.py:339-340`).

### 6-3. `input_map/stray_meta.json` (`stray.convert` 끝, `stray.py:348-361`)

| 필드 | 형 | 뜻 |
|---|---|---|
| `source` | 문자열 | 촬영 폴더 이름 (zip 안의 폴더 이름. 웹 작업이어도 `input` 이 아님) |
| `format` | 문자열 | `"stray_scanner"` |
| `frames_total` | 정수 | `odometry.csv` 의 줄 수 |
| `frames_read` | 정수 | 영상에서 읽은 프레임 수 |
| `keyframes` | 정수 | 고른 키프레임 수 (= 자세 줄 수) |
| `dropped_fast` | 정수 | 너무 빨라서 뺀 장 수 |
| `duration_s` | 수 | 촬영 길이 (초) |
| `path_length_m` | 수 | 카메라가 움직인 길이 (모든 프레임 기준) |
| `extent_m` | [3] 수 | 고른 카메라 위치의 x · y · z 범위 (입력 좌표, Z 위) |
| `camera_height_range_m` | [2] 수 | 고른 카메라 위치의 z 최소 · 최대 (입력 좌표. 바닥 기준이 아님) |
| `speed_median_mps` · `speed_max_mps` | 수 | 5프레임 평균 속력의 중앙값 · 최댓값 |
| `color_size` · `depth_size_in` · `keyframe_size` | [가로, 세로] | 원본 색 · 원본 깊이(첫 깊이 파일에서 잼) · 저장한 키프레임 (기본은 깊이와 같음) |
| `intrinsics` | {fx, fy, cx, cy} | 키프레임 크기 기준 |
| `focal_variation_pct` | 수 | 프레임별 초점거리의 변동 폭 (%) — 한 값으로 씀 |
| `confidence_pct` | [3] 수 | 고른 장의 신뢰도 0 · 1 · 2 비율 (%) |
| `min_confidence` | 정수 | `min_conf` |
| `valid_depth_pct` | 수 | 걸러낸 뒤 남은 깊이 화소 비율의 평균 (%) |
| `select` | 객체 | `{move_m, turn_deg, max_speed_mps, max_depth_m, skip_s, max_keyframes, widen, move_m_used, turn_deg_used}`. `widen` = 키프레임 상한 때문에 넓힌 배율 (1 = 안 넓힘), `*_used` = 실제로 쓴 간격 |
| `checks` | 객체 | `{warnings: [문장], info: {odometry_rows, duration_s, depth_files, confidence_files, video_size, video_frames, video_fps}}` — 입력 검사 결과 |

실제 파일 (`room`, `checks` 줄임):

```json
{"source": "6bc1ae021f", "format": "stray_scanner", "frames_total": 6611, "frames_read": 6610, "keyframes": 681,
 "dropped_fast": 3, "duration_s": 118.3, "path_length_m": 43.83, "extent_m": [9.41, 5.29, 0.89],
 "camera_height_range_m": [-0.4, 0.5], "speed_median_mps": 0.37, "speed_max_mps": 3.41,
 "color_size": [1920, 1440], "depth_size_in": [256, 192], "keyframe_size": [960, 720],   ← 11/24 전 예시. 지금 기본은 [256, 192]
 "intrinsics": {"fx": 664.972, "fy": 664.972, "cx": 479.022, "cy": 362.739}, "focal_variation_pct": 1.56,
 "confidence_pct": [29.0, 26.7, 44.3], "min_confidence": 1, "valid_depth_pct": 66.0,
 "select": {"move_m": 0.05, "turn_deg": 5.0, "max_speed_mps": 1.5, "max_depth_m": 5.0, "skip_s": 1.0,
            "max_keyframes": 800, "widen": 1.728,   ← 11/24 전 예시. 지금은 상한 2,000 이라 이 촬영본은 넓히지 않고 1,133장 "move_m_used": 0.0864, "turn_deg_used": 8.64},
 "checks": {"warnings": ["촬영이 길어 키프레임 간격을 1.73배 넓힘 (9 cm · 8.6°, 상한 800장) — 처리 시간을 줄이려는 것"],
            "info": {"odometry_rows": 6611, "duration_s": 118.33, "…": "…"}}}
```

**입력 단계가 Stray 에 대해 내는 판정**

- Stray 만 붙는 주의 (`stages.py:78-84`)
  - 촬영 범위가 좁음: `path_length_m < 5` 또는 `max(extent_m[:2]) < 3`
  - 깊이가 적음: `valid_depth_pct < 50`
  - 촬영이 빠름: `speed_median_mps > 0.8`
  - 되돌아온 자리 보정 없음: **항상** 붙음. 그래서 Stray 입력의 `input` 단계는 언제나 `warn` 이상이다.
- GLB 와 같이 형상 메쉬를 보고 내는 판정 (`stages.py:66-73`, 11/24 부터 Stray 에도)
  - 실패: 범위가 1 m 미만 · 300 m 초과, 바닥(위 향한 면) 2 m² 미만.
  - 주의: 바닥이 평평하지 않음. 바닥 대부분(75% 이상)이 한 높이면 "바닥의 n% 가 다른 높이에 찍힘" 으로 적는다.
- "사진 텍스처가 없음" 주의는 Stray 에는 붙지 않는다 (`stages.py:74`).
- 요약은 `"아이폰 원본 → 형상 메쉬 · 천장 … · 바닥 … · 범위 …"` (`stages.py:65`). 11/23 까지는 `"아이폰 원본 → 맵 폴더 · …"` 였다.

`input_map/trajectory_camera_optical.txt` 의 형식: `시각 x y z qx qy qz qw id` (카메라 광학 → 입력 세계, 첫 줄은 `#` 주석).

### 6-4. 바닥 높이 보정과 새 결과 파일 (B `stray_fix.py`, 11/24 저녁)

- `stray_mesh.build` 가 합치기 전에 키프레임 자세의 높이를 고친다 (`fix="floor"` 기본, `fix=None` 끔, CLI `--no-fix`). 방법: 키프레임마다 바닥 칸(20 cm) 높이 = 자세 높이 오류 + 칸의 실제 높이로 풀고(Huber · 시간 사전), 고친 카메라 z = z − dz. 틀어짐이 작으면(2~98% 폭 < 1.5 cm) 건너뜀. 바닥 근거 부족 · 보정 40 cm 초과 · 잔차 3 cm 초과면 거부하고 자세를 그대로 씀.
- **`out/input_map/floor_fix.json`** (Stray 경로에서 늘 생김, 끄면 status "off"): `method, version, params, keyframes, applied, status(applied · skipped · refused · off · error · custom), reasons, flags, t, dz, dz_estimate, tilt, frames_supported(_pct), cells, observations(_dropped), resid_rms_cm, resid_med_cm, dz_range_cm, dz_p5_p95_cm, dz_max_abs_cm, max_unsupported_s, jump_frames, jump_times, levels{…}, seconds{observe, solve, total}`
- **`build/tsdf_meta.json` 의 `pose_fix`**: 늘 요약 dict — `name, file, applied, status, method, reasons, flags, keyframes, frames_supported_pct, dz_p5_p95_cm, dz_max_abs_cm, resid_rms_cm, max_unsupported_s, stand_other_pct, jump_frames, seconds, moved_m, turned_deg`. `timing.pose_fix_s` 추가.
- `trajectory_camera_optical.txt` 는 **보정 전** 자세다. 다른 프레임에 같은 보정이 필요하면 `stray_fix.apply_fix(times, poses, info)`.
- **`input_map/stray_quality.json`**: 새 키 `pose_fix`, `held_out.method` ("time_grid"), `held_out.relaxed_cells`, `held_out.target_offset_s`, `drift.pose_fix`. 안 쓴 프레임은 촬영 시간을 n 구간으로 나눠 구간마다 하나 고른다(키프레임 수가 바뀌어도 덜 흔들림). 보정이 적용됐으면 안 쓴 프레임에도 같은 보정을 건다.
- 엔진에 붙는 것은 C 가 R10 을 넣은 뒤다. 그 전까지 엔진은 `pipeline/tsdf.py` 를 부르고 보정이 없다.

---

## 7. 한도 (구현된 값)

| 항목 | 값 | 위치 |
|---|---|---|
| 받는 확장자 | `.zip` · `.glb` (대소문자 무시). 화면도 같은 검사 | `web/server.py:37`, `:92-94` · `index.html:95`, `:162` |
| 파일 크기 | 1 바이트 ~ `MAX_BYTES` = 4 × 1024³ = 4,294,967,296 바이트 | `web/server.py:38`, `:95` |
| 조각 크기 | 화면이 8 MiB(8,388,608 바이트)로 나눔. 서버는 조각 크기를 제한하지 않고 1 MiB 씩 읽음 | `index.html:144` · `web/server.py:115` |
| 작업 번호 | `^[0-9a-f]{12}$` | `web/server.py:47` · `index.html:358` |
| 파일 이름 저장 길이 | 앞 120자 | `web/server.py:99` |
| 동시 처리 | 처리 스레드 **하나** (한 번에 한 작업, 나머지는 줄). 요청 받기는 Flask 내장 서버 `threaded=True` | `web/server.py:271`, `:273` |
| 기본 주소 | `127.0.0.1:8000`. 밖에 열 때는 `--code` 를 줄 것. 배포 스크립트는 늘 127.0.0.1 + 터널 + 접속 코드 (4장) | `web/server.py:264-265` · `deploy/start.sh:34` |
| 결함 수 | 파일마다 최대 30 | `stages.py:158` |
| 화면 폴링 | 1.5초 | `index.html:202` |
| 화면용 GLB | 정점 색 삼각형 30만 · 텍스처 긴 변 2048 화소 | `preview.py:46` |
| Stray 키프레임 | 최대 2,000장 (넘으면 간격을 넓힘) | `stray.MAX_KEYFRAMES` |
| Stray zip | 파일 10만 개 · 풀면 6 GB | `stray.py:55` |
| 엔진 쪽 실패 기준 (참고) | GLB · Stray 범위 1 m 미만 · 300 m 초과, 바닥 2 m² 미만 · 빈 바닥 4 m² 미만 · 진단 표본 50개 미만 | `stages.py:66-69`, `:142-143`, `:221-222` |
| 처리 시간 제한 | 없음 (엔진이 끝날 때까지 기다림) | `web/server.py:146` |

---

## 8. 계획과 다른 점

계획 = `docs/implementation.md` 5장 (2판, 11/23). 위가 코드에서 확인한 것이다. 11/24 에 C · D 가 고친 것은 **고침** 으로 적고 커밋을 붙였다.

### 8-1. 계획이 틀렸던 것

1. **Stray · 맵 폴더의 `T_input_to_scene` 은 단위행렬이 아니다.**
   - 계획 5-1 의 [가정] "Stray · 맵 폴더는 `transform.json` 이 없어 단위행렬, `center_input = center_scene`" 은 틀렸다.
   - 맵 폴더는 `pipeline/build_usd.py:248`, `:260` 이, Stray 는 11/24 부터 `pipeline/glb_to_usd.py:630-632` 가 `transform.json` 을 쓴다.
   - Stray 의 내용은 바닥 수평 맞춤 + x · y 를 범위 가운데로 + 바닥 z 이동이다 (`room`: 0.38°, (+4.506, −2.313, +1.580) m).
2. **Stray 의 "입력 좌표" 는 올린 원본 좌표가 아니다.**
   - 계획 5-1 은 "올린 GLB 의 좌표 그대로" 라고만 적었다.
   - Stray 는 `input_map` 세계(Stray +Y 위 세계를 `(x, −z, y)` 로 돌린 것)다 (1-3). 11/24 의 길 변경 뒤에도 같다.
3. **`radius_m` · `extent_m` 의 정의가 계획과 조금 다르다.**
   - `radius_m` 은 가장 먼 표본까지 거리에 0.1 m 를 더한 값이다.
   - `extent_m` 은 범위 상자의 대각선에 0.05 m 를 더한 값이다 (5-2).
4. **`no_visible_floor` 의 넓이 기준**은 "0.25 m² 넘는 곳" 이 아니다.
   - 실제로는 "표본 수 × 0.01 m² ≥ 0.25" 이다. 실제 구멍 넓이가 아니라 표본으로 어림한 넓이다. 새 `floor_two_levels` 도 같다.
5. **`route.csv` 는 늘 있는 것이 아니다.** 계획 5-2 는 늘 있는 것처럼 적었다.
   - GLB · Stray 에만 생기고 맵 폴더에는 없다. Stray 는 11/24 (C `0534d95`) 부터 생긴다.
   - 촬영 경로가 아니라 바닥 빈 곳의 뼈대로 만든 가상 경로다 (2-4).
6. **`out/input_map/` 에 `_unzip/`(원본 촬영 전체, 사람이 찍힌 `rgb.mp4` 포함)과 `keyframes/` 가 더 있다.** 개인정보 정리(계획 10장) 대상이다.
7. **[고침 — D `98da703`, 통합 기록 R2]** 상태 `done` 이 "엔진이 성공함" 이 아니었다.
   - 지금은 `report.md` 가 있고 종료 코드 0 이고 실패 단계가 없어야 `done` 이다 (4-3).
   - 11/23 의 `e2e_glb` 는 `done` · `exit_code 1` · `verdict fail` 이었다. 지금 규칙으로는 `failed`.
8. **404 응답은 `{error}` JSON 이 아니라 HTML 이다.** 계획 5-6 은 "오류는 `{error}`" 라고 적었다.
9. 상태 응답에 계획 예시에 없는 필드가 있다: `file` · `queued_at` · `error`.
10. **맵 폴더 zip 은 웹으로 올릴 수 없다.**
    - `web/server.py:6` 의 설명은 "맵 폴더를 묶은 zip" 을 받는다고 적었다.
    - 그러나 엔진의 `detect.kind_of` 는 Stray 가 아닌 zip 을 알아보지 못한다 (`detect.py:13-20`). 입력 단계에서 `"입력을 알아볼 수 없음"` 으로 실패한다.
11. **웹으로 올린 작업은 장면 이름이 모두 `input` 이다** (2-4).
    - USD 가 `input_mesh.usdc` 가 되고, `scene_root.usda` 가 `data/usd/input/…` 을 가리키고, `bundle.json` 의 `name` 도 `input` 이다.
    - 여러 장소를 `sim/` 에 들이면 이름이 겹친다. 계획 5-2 는 "이름은 서버가 정함" 까지만 적었다.
12. **Stray 결과의 `bundle.json` `kind` 가 `glb` 다** (11/24, C `0534d95`). 묶음만 보고는 Stray 인지 GLB 인지 `input` 확장자로만 가린다 (2-4).

### 8-2. 계획에는 있고 아직 없는 것

- `GET /api/samples`, `POST /api/samples/<name>/jobs`, `GET /api/health` (계획 5-6 "추가하기로 한 것", D 담당) — 서버에 없다. 지금 경로는 4-2 표가 전부다.
- 화면의 이어 올리기. 서버는 같은 프로세스 안에서만 된다 (`received` 를 `job.json` 에 조각마다 저장하지 않음).
- `bundle/` 을 D 의 `sim/add_site.py` 형식에 맞추기 (계획 3-2 · 5-2). 아직 지금 형식 그대로다.
- ~~Stray 복원 정확도를 진단 · 보고서에 붙이기~~ — 붙음 (R3, C `620a2b0`). 진단 수치 `stray_quality` = `{frames, median_cm, p90_cm, within_2cm_pct, hit_pct, verdict}`, 요약 문장은 `messages`, 통과가 아니면 이유가 주의(실패여도 주의). `report.md` 에 "복원 정확도 (아이폰 원본)" 절. [C 측정] 방 촬영본 중앙 2.17 cm · 2 cm 안 47.2% · 맵에 닿음 98.8% → 통과
- **[고침 — C `736eb68`]** "같은 입력에 같은 결과" (계획 5-3). 11/23 판에는 "아직 보장하지 않음" 이었다.
  - GLB · Stray 의 바닥 평면 찾기에서 무작위 표본(RANSAC)을 없앴다. 화면용 GLB 도 무작위를 쓰지 않는다.
  - C 측정 (Windows): 복도 GLB 4번 · 가상의 방 Stray 2번에서 변환 행렬 차이 0 · 결함 목록 같음 (`C_TRACK.md` 1장).
  - `tests/repeat.py` 가 계속 검사한다.

### 8-3. 버그로 보이는 것 (고칠 사람이 정할 것)

| # | 위치 | 내용 | 상태 |
|---|---|---|---|
| 1 | `core.py:53` + `pipeline/glb_to_usd.py:66`, `:88` (`detect.profile_glb` 경유, `detect.py:42-44`) | 단계가 `except Exception` 만 잡아서 `SystemExit` 이 새어 나간다. 이때 단계 파일도 `report.md` 도 없이 엔진이 끝난다. 그러면 서버 상태는 `failed`, 그 단계부터는 모두 `pending`, `verdict` 는 `"pending"` (#2) 이 된다. 예: 바닥을 못 찾는 GLB. **11/24 부터 Stray 도 해당한다** — 입력 단계가 형상 메쉬에 `profile_glb` 를 부르므로, 형상 메쉬에 위 향한 면이 거의 없으면 같은 길로 죽는다 [추정: 그런 촬영본으로 돌려 보지 않음]. (`stray.py` 쪽은 B 가 `CaptureError` 로 바꿔 고침, `ba960b4`) | 열림 |
| 2 | `web/server.py:191` | `report.json` 이 없으면 `verdict` 를 `report` 단계 상태로 대신하는데, 그 단계가 없으면 `"pending"` 이 들어간다. 화면은 그 글자를 그대로 보인다 (`job.verdict \|\| "fail"` 이라 `"pending"` 은 걸러지지 않음) | 열림 |
| 3 | `web/server.py:113-122` | 조각이 알린 크기를 넘으면 400 을 돌려주지만(`:119-120`), 그 전까지 읽은 부분은 이미 파일에 붙은 채이고 `received` 는 그대로다. 같은 `offset` 으로 다시 보내면 파일이 어긋난다 | 열림 |
| 4 | `web/server.py:249` | `bundle.zip` 을 한 번 만들면 다시 만들지 않는다. 처리 중(`bundle` 단계 뒤 · `report` 전)에 요청하면 `report.md` 가 빠진 zip 이 남는다 | 열림 |
| 5 | `web/server.py:91`, `:109` | `size` · `offset` 이 숫자가 아니면 `int()` 예외로 500 이 난다 (400 이 아님) | 열림 |
| 6 | `web/server.py:6` vs `detect.py:13-20` | 맵 폴더 zip 을 받는다는 설명과 실제 동작이 다르다 (8-1 의 10번) | 열림 |
| 7 | `recon/requirements.txt` | GLB 빌드가 쓰는 `scikit-image`, Stray 의 `opencv`, 서버의 `flask` · `pillow` 가 목록에 없었다 (`e2e_glb` 가 `ModuleNotFoundError: No module named 'skimage'` 로 `convert` 실패) | **고침** (D `98da703`, R1): `opencv-python-headless==5.1.0.95` · `scikit-image==0.26.0` · `Flask==3.1.3` · `pillow==13.0.0` |
| 8 | 상태 응답의 `status` (`web/server.py:149-150`) | 엔진 단계가 실패해도 `done` 이었다 (8-1 의 7번) | **고침** (D `98da703`, R2) |
| 9 | `stages.py:52` · `core.py:53-54` | Stray 촬영본 오류가 `"예외: CaptureError: <문장>"` 요약과 traceback 6줄로 나간다. 사용자 문장만 보이고, `checks.warnings` 도 입력 단계 주의로 보이게 해 달라고 요청함 (통합 기록 R4, C) | **고침** C `620a2b0` |
| 10 | `web/static/index.html:311-312` | 화면이 "천장이 있는가" 를 진단 `messages` 의 문장(`"천장이 있음"`)으로 판단해 3D 위쪽을 자른다. 문장이 바뀌면 깨지고, 지금도 천장이 일부만 찍힌 경우(`room`: 덮음 27%, 문장 "천장이 일부만 찍힘")는 자르지 않는다. 상태 응답의 `metrics.before.has_ceiling` · `shadow_fix` 를 보는 편이 약속에 맞다 [추정: 화면에서 직접 확인하지 않음] | 열림 (A) |
