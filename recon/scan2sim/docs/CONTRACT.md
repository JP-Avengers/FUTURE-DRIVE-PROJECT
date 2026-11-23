# CONTRACT.md — scan2sim 약속 (엔진 · 서버 · 화면이 주고받는 것)

작성 2026-11-23 · B · 기준 코드: 브랜치 `b/yunseong` (이 문서에 나오는 파일은 origin/main `a069f09` 와 같음)

- 이 문서는 **지금 코드가 하는 일**을 적는다. 계획(`docs/implementation.md` 5장)과 다른 곳은 8장에 모았다.
- 근거 표시: 코드 위치는 `파일:줄` (경로는 `recon/scan2sim/` 기준). 코드를 읽지 않고 짐작한 것은 **[추정]** 이라고 적었다.
- 예시 JSON 은 실제 실행 결과에서 가져왔다. 긴 배열은 줄였다(`…`).
  - `stray1` = 엔진을 Stray zip(`9a5813208f.zip`)에 직접 돌린 결과 (`~/webapp_runs/stray1/`)
  - `e2e_stray` = 서버를 띄워 같은 촬영본을 API 로 올린 시험 (`~/webapp_runs/e2e_stray.json`, 작업 `89d34e717918`)
  - `e2e_glb` = 서버로 GLB(`corridor_v04.glb`)를 올린 시험 (작업 `b8ce80baff04`, 변환 단계에서 실패한 사례)

---

## 0. 이 약속을 바꾸는 법

1. API 경로 · 응답 필드, 작업 폴더의 파일 이름, 결함 목록의 필드를 **바꾸거나 없앨 때**는 먼저 팀 채팅에 알린다.
2. 그 변경과 **같은 커밋**에서 이 파일을 고친다.
3. **필드를 더하는 것은 자유**다(알리지 않아도 됨). 대신 화면 · 시험 · 다른 도구는 모르는 필드를 무시해야 한다.
   - 화면은 모르는 결함 `type` 이 와도 목록에 보여 줘야 한다.
4. `tests/` 의 시험이 이 약속을 검사한다. 시험이 깨지면 바꾼 사람이 그날 안에 고친다.
   - `tests/e2e.py`: 서버를 띄워 API 로 올리고 상태 · 결함 필드 · 결과 파일 4개를 검사.
     검사하는 키는 `tests/e2e.py:29-32` (`JOB_KEYS` · `DONE_KEYS` · `STAGE_KEYS` · `DEFECT_KEYS`).
   - `tests/repeat.py`: 같은 입력을 두 번 돌려 결함 목록이 같은지 검사.
   - 이 문서의 필드 표를 고치면 위 키 목록도 같이 고친다.

---

## 1. 좌표계

### 1-1. 세 가지 좌표

| 이름 | 정의 | 쓰는 곳 |
|---|---|---|
| **장면 좌표** (scene) | **+Z 가 위**, 단위 m. 바닥 높이 ≈ 0 | 엔진 내부, USD, Nav2 지도, `start_pose.json`, 결함의 `center_scene`, 단계 metrics 의 위치 값 |
| **화면 좌표** (view) | Three.js. **+Y 가 위**. `화면 (x, y, z) = 장면 (x, z, −y)` | `preview.glb`, 화면의 `toView()` |
| **입력 좌표** (input) | 입력 종류마다 다름 (1-3). `p_장면 = T_input_to_scene · p_입력` | 결함의 `center_input` · `up_input` · `floor_point_input` (지금 화면은 쓰지 않음) |

- 장면 → 화면 변환은 **두 곳에서만** 한다.
  - 서버가 `preview.glb` 를 만들 때: `web/server.py:238` (USD 에서) · `web/server.py:243` (`visual.ply` 에서). 정점을 `(x, z, −y)` 로 바꿔 씀.
  - 화면이 결함 표시를 놓을 때: `web/static/index.html:293` `toView = (p) => (p[0], p[2], -p[1])`.
- 거꾸로(화면 → 장면)는 `장면 (x, y, z) = 화면 (x, −z, y)`.
- Nav2 지도(`bundle/nav2/map.pgm`)는 장면 좌표의 위에서 본 그림이다.
  - `map.yaml` 의 `origin` 은 그림 왼쪽 아래 칸 모서리의 장면 (x, y).
  - 그림의 행 0 = 위쪽(y 가 큰 쪽) (`checks.py:8`). 칸 = `resolution` m (지금 0.05).

### 1-2. 장면 좌표의 원점과 바닥

| 입력 | 수평 맞춤 | 바닥 z = 0 | x · y 원점 |
|---|---|---|---|
| GLB | 바닥 평면 법선 → +Z (`pipeline/glb_to_usd.py:79-113`) | 찾은 바닥 평면 | 범위의 가운데 (자동 설정 `orient: origin: center`, `detect.py:95`; 이동은 `pipeline/glb_to_usd.py:474-478`) |
| 맵 폴더 · Stray | 바닥 법선 → +Z (`pipeline/build_usd.py:70-85`) | **카메라 경로 아래 바닥 높이의 중앙값** (`pipeline/build_usd.py:251-261`, `transform.json` 의 `floor_zero`) | 입력 좌표 그대로 (틀 `configs/templates/indoor_rtabmap.yaml` 의 `orient: origin: keep`) |

- 맵 폴더 · Stray 는 실내 틀에 `ground:` 항목이 있어서 `outdoor = "ground" in cfg` 가 참이 된다 (`pipeline/build_usd.py:222`).
  그래서 "바닥 = 경로 아래 중앙값" 으로 z 를 한 번 더 맞춘다. 이름은 outdoor 지만 실내에서도 이 길로 간다.
- 바닥이 완전히 평평하지 않으므로 결함의 `center_scene` z 는 0 이 아니라 그 자리의 바닥 높이다 (예: 0.003, 0.038, −0.01).

### 1-3. 입력 좌표와 `T_input_to_scene`

- `T_input_to_scene` 은 4×4 동차 행렬, **행 단위 리스트** (`T[0]` = 첫 행). 위치는 열 벡터로 곱한다: `p_장면 = T · [x, y, z, 1]`.
- 파일: `out/build/transform.json` (빌드가 씀). 결함 목록 파일에도 같은 값이 들어간다 (`stages.py:169-179`).
- `transform.json` 이 없으면 단위행렬로 친다 (`stages.py:170`). 하지만 **지금 세 입력 모두 빌드가 이 파일을 쓴다** — 단위행렬이 되는 경우는 빌드가 비정상일 때뿐이다.

| 입력 | "입력 좌표" 가 뜻하는 것 | `T_input_to_scene` | 쓰는 곳 |
|---|---|---|---|
| GLB | 올린 GLB 의 glTF 좌표 그대로 (+Y 위) | `M(원점 · 수평 · 바닥) · Y_UP_TO_Z_UP · 배율` | `pipeline/glb_to_usd.py:606-608` |
| B 형식 맵 폴더 (리얼센스) | 맵 폴더의 세계 좌표 = `trajectory_camera_optical.txt` 의 세계 (+Z 위) | 수평 맞춤 회전 + 바닥 높이 이동 | `pipeline/build_usd.py:248`, `:260` |
| **Stray (아이폰)** | **`out/input_map/` 의 세계 좌표** = Stray 세계를 `Y_UP_TO_Z_UP` 으로 돌린 것 (+Z 위). **Stray 원본 세계(+Y 위)가 아님** | 맵 폴더와 같음 | `stray.py:30`, `stray.py:160-164` |

**Stray 의 입력 좌표 (자세히)**

- Stray `odometry.csv` 의 자세 = 카메라 광학 좌표계(x 오른쪽 · y 아래 · z 앞) → Stray 세계. Stray 세계는 **+Y 가 위** (`stray.py:18-19`).
- `stray.py` 는 카메라 쪽은 그대로 두고 세계만 돌린다: `T_맵폴더 = Y_UP_TO_Z_UP · T_stray` (`stray.convert`).
  - `Y_UP_TO_Z_UP = [[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]]` → `p_입력 = (x, −z, y)_stray` (`stray.py:30`)
  - 거꾸로: `p_stray = (x, z, −y)_입력`
- 이 돌린 세계로 `input_map/trajectory_camera_optical.txt` 를 쓰고, TSDF 메쉬(`build/mesh_raw.ply`)도 이 좌표로 나온다. `transform.json` 의 `source` 가 `build/mesh_raw.ply` 다.
- 그래서 Stray 결함의 `center_input` 을 Stray 원본 좌표로 보려면 `(x, z, −y)` 를 한 번 더 해야 한다.

실제 값 (`stray1/build/transform.json`) — 약 0.34° 기울기 보정 + z 를 0.247 m 올림 (입력 좌표에서 바닥이 z ≈ −0.247):

```json
{
 "source": "~/webapp_runs/stray1/build/mesh_raw.ply",
 "T_input_to_scene": [
  [0.9999891934924491, 8.298576300570192e-06, 0.004648960040151067, -0.0],
  [8.298576300570192e-06, 0.9999936273242496, -0.003570047902111379, -0.0],
  [-0.004648960040151067, 0.003570047902111379, 0.9999828208166986, 0.24695133270894354],
  [0.0, 0.0, 0.0, 1.0]
 ],
 "orient": {"up_hint": [0, 0, 1], "max_tilt_deg": 20, "origin": "keep", "crop_below_floor": 0.05},
 "floor_zero": "주행 경로 아래 바닥 중앙값 = 0"
}
```

- `transform.json` 의 다른 키는 입력마다 다르다 (GLB 는 `T_input_to_scene` · `note` 뿐). **약속된 키는 `T_input_to_scene` 하나**다.

---

## 2. 작업 폴더

### 2-1. 서버 쪽 (`<jobs>/<작업 번호>/`)

`<jobs>` 기본값 `~/scan2sim_jobs`, 서버의 `--jobs` 로 바꿈 (`web/server.py:41`, `:297`). 저장소 밖에 둔다.

```
<jobs>/<id>/                  id = 16진수 소문자 12자리 (uuid4 앞 12자리, web/server.py:96)
  job.json                    서버가 쓰는 작업 기록 (2-2)
  input/input.zip             올린 파일. 이름은 항상 "input" + 확장자 (web/server.py:98)
  input/input.glb               — 원래 이름은 job.json 의 name 에만 남음
  run.txt                     엔진의 표준 출력 + 표준 오류 (web/server.py:144-146)
  out/                        엔진 출력 폴더 (2-3)
```

- 올린 파일 이름을 바꾸는 이유: 경로에 한글이 있으면 Open3D 가 못 읽음 (`web/server.py:98` 주석).
- 그 결과 **웹으로 올린 작업은 장면 이름이 모두 `input`** 이 된다 (2-4).

### 2-2. `job.json`

서버가 메모리에 든 작업 dict 를 그대로 쓴 것 (`web/server.py:67-68`). 2-1 의 `GET /api/jobs/<id>` 응답의 앞부분과 같다.

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
| `exit_code` | 정수 | 엔진이 끝남 | 엔진 종료 코드 (3-1). 엔진을 못 띄운 경우에는 없음 |
| `finished` | 실수 | 처리 끝 | |
| `error` | 문자열 | 실패할 때만 | 서버 쪽 예외 (`"TypeError: …"`) 또는 `"서버가 다시 켜져서 끊김"` |

실제 `job.json` (`e2e_glb`, 변환 단계 실패 — 그래도 `status` 는 `done`, 4-3 참고):

```json
{
 "id": "b8ce80baff04",
 "name": "corridor_v04.glb",
 "size": 169398840,
 "received": 169398840,
 "status": "done",
 "created": 1795442409.8932807,
 "file": "input.glb",
 "queued_at": 1795442410.1229875,
 "started": 1795442410.1234465,
 "exit_code": 1,
 "finished": 1795442517.5178983
}
```

### 2-3. 엔진 쪽 (`out/`)

`stray1` 에서 실제로 본 파일 이름. GLB 는 `build/` 안이 다르다.

```
out/
  stages/NN_<단계>.json       단계별 판정 (3-2). NN = 00~07
  log.txt                     빌드 · 지도 하위 프로그램의 출력 (stages.py:23-27)
  input_map/                  Stray 입력일 때만 — Stray 를 맵 폴더 형식으로 바꾼 것 (6장)
    _unzip/                     zip 을 푼 원본 촬영 (rgb.mp4 · depth/ · odometry.csv …). 사람 얼굴이 든 영상이 그대로 남음
    keyframes/rgb/<n>.jpg · keyframes/depth/<n>.png · keyframes/calib/<n>.yaml
    trajectory_camera_optical.txt
    stray_meta.json
    stray_quality.json          엔진이 만들지 않음. `python -m scan2sim.stray_quality` 를 따로 돌렸을 때만 [B 작업 중]
  build/                      빌드 결과
    auto_config.yaml            엔진이 만든 빌드 설정
    auto_config_corrected.yaml  보정을 했을 때만 (stages.py:277)
    transform.json              1-3
    visual.ply                  보이는 형상 (장면 좌표)
    ground.npz                  바닥 높이 지도 (있으면 진단 · 시작 자세가 씀)
    usd/<이름>_mesh.usdc · usd/<이름>_scene.usda · usd/textures/
    report.json                 빌드 보고 (엔진의 report.json 과 다른 파일)
    (맵 폴더 · Stray 만) mesh_raw.ply · collider.ply · floor.ply · boundary.ply · check_input.json · tsdf_meta.json …
    (GLB 만) drive_path.json    가상 주행 경로 → bundle/route.csv 의 재료
  defects_before.json         보정 전 결함 목록 (5장). 진단 단계가 돌면 항상
  defects_after.json          보정 뒤 다시 진단한 결함 목록. **보정이 돌았을 때만** (5-4)
  bundle/                     Isaac Sim 용 묶음 (2-4)
  report.md                   사람이 읽는 보고서 (s_report, stages.py:430)
  report.json                 {name, verdict, stages} (stages.py:431, 3-4)
  preview.glb                 서버가 처음 요청 때 만듦 (4-2)
  bundle.zip                  서버가 처음 요청 때 만듦 (4-2)
```

- 보정이 돌면 `build/` 는 **같은 자리에 다시 만들어진다** (보정 설정이 원래 설정을 `base:` 로 물려받고 `out:` 을 바꾸지 않음, `stages.py:279-287`). 그래서 보정 뒤의 `build/` · `bundle/` · `preview.glb` 는 모두 보정된 것이다.

### 2-4. 이름과 `bundle/`

**장면 이름** = `detect.safe_name(입력 경로)` (`detect.py:32-34`)

1. 파일 이름에서 확장자를 뺌.
2. `[A-Za-z0-9_]` 밖의 글자를 `_` 로 바꾸고 앞뒤 `_` 를 지움. 비면 `scene`.
3. 숫자로 시작하면 앞에 `m_` 를 붙임.
4. 소문자로.

| 입력 | 장면 이름 | USD 파일 |
|---|---|---|
| CLI `9a5813208f.zip` | `m_9a5813208f` | `m_9a5813208f_mesh.usdc` · `m_9a5813208f_scene.usda` |
| 웹으로 올린 모든 파일 (`input/input.zip` · `input/input.glb`) | `input` | `input_mesh.usdc` · `input_scene.usda` (`e2e_stray` 의 `bundle.zip` 에서 확인) |

**`bundle/`** (`s_bundle`, `stages.py:300-378`) — `stray1` 실제 목록:

```
bundle/
  usd/m_9a5813208f_mesh.usdc       build/usd/ 를 통째로 복사 (stages.py:322-324)
  usd/m_9a5813208f_scene.usda
  usd/textures/floor_color.jpg · wall0_color.jpg · wall1_color.jpg
  nav2/map.pgm · map.yaml · preview.png    sim/nav2/make_map.py 출력 (stages.py:114-118)
  scene_root.usda                  씬 루트 제안 (Isaac Sim 에서 열어 보지 않음)
  start_pose.json                  시작 자세 (장면 좌표)
  route.csv                        GLB 만 (build/drive_path.json 이 있을 때, stages.py:315-321). Stray · 맵 폴더는 없음
  bundle.json                      {name, input, kind, files}. files 에 bundle.json 자신은 없음
```

```json
// bundle/bundle.json (stray1)
{"name": "m_9a5813208f", "input": "9a5813208f.zip", "kind": "map_folder",
 "files": ["nav2/map.pgm", "nav2/map.yaml", "nav2/preview.png", "scene_root.usda", "start_pose.json",
           "usd/m_9a5813208f_mesh.usdc", "usd/m_9a5813208f_scene.usda",
           "usd/textures/floor_color.jpg", "usd/textures/wall0_color.jpg", "usd/textures/wall1_color.jpg"]}

// bundle/start_pose.json (stray1) — z = 바닥 + 0.022, yaw 는 도
{"x": -3.178, "y": 0.456, "z": 0.06, "yaw_deg": 33.9, "clearance_m": 1.01, "goal_xy": [0.472, 2.906],
 "note": "빈 바닥에서 장애물과 가장 먼 자리 · 가장 먼 빈 자리를 향함. 높이 = 바닥 + 0.022 (Nova Carter)"}
```

```yaml
# bundle/nav2/map.yaml (stray1, 첫 줄 주석 줄임)
image: map.pgm
mode: trinary
resolution: 0.05
origin: [-6.953, -3.919, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

- Stray · 맵 폴더 입력은 `kind` 가 `map_folder` 로 적힌다 (Stray 는 변환 뒤 맵 폴더 길로 감, `stages.py:49`).
- `scene_root.usda` 는 `../proxy/*.usda` 와 `../../../data/usd/<이름>/usd/<이름>_scene.usda` 를 subLayer 로 건다 (`stages.py:338-344`). 즉 `sim/scenes/<장소>/` 에 두는 것을 가정한다. 웹 작업은 `<이름>` 이 모두 `input` 이다.
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
  - Stray 촬영 폴더, 또는 그 폴더가 든 zip (촬영 폴더 판별은 6-1, `stray._zip_captures`)
  - B 형식 맵 폴더 (`keyframes/` + `trajectory*camera_optical.txt`) — **폴더만**. 맵 폴더를 묶은 zip 은 알아보지 못함 (8장)
  - 녹화 파일(`.mcap` · `.db3` · `.bag`)은 알아보지만 입력 단계에서 실패시킴 (`stages.py:37-38`)
- `--force`: 실패한 단계가 있어도 다음 단계를 함. `--resume`: `out/build` 에 결과가 있으면 변환을 건너뜀 (`__main__.py:20-23`). **서버는 둘 다 쓰지 않는다** (`web/server.py:145`).
- 종료 코드 (`__main__.py:29`)
  - `0` = 어느 단계도 `fail` 이 아님 (통과 또는 주의)
  - `1` = 한 단계라도 `fail`
  - 단계 안의 `Exception` 은 그 단계의 `fail` 로 바뀐다 (`core.py:53-54`). 단, `SystemExit` 은 잡히지 않아 단계 파일 없이 엔진이 끝난다 (종료 코드 1, 8장 버그). `stray.py` 는 이제 `SystemExit` 대신 `CaptureError` 를 낸다.
  - 인자가 틀리면 argparse 의 `2` [추정: argparse 기본 동작].

### 3-2. 단계 파일 `out/stages/NN_<단계>.json`

`core.run` 이 단계마다 하나씩 쓴다 (`core.py:41-64`). 파일 이름 = `f"{k:02d}_{name}.json"` (`core.py:56`).

```json
{"stage": "nav2", "status": "pass", "summary": "217×203 칸 (0.05 m) · 빈 바닥 28 m²",
 "metrics": {"cells": [217, 203], "resolution": 0.05, "free_m2": 28.5, "occupied_cells": 1171},
 "messages": [], "seconds": 0.4}
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

순서 · 이름은 `stages.py:435`, 화면 이름(`label`)은 `web/server.py:35-36`.

| NN | `stage` | `label` | 하는 일 | 나올 수 있는 `status` |
|---|---|---|---|---|
| 00 | `input` | 입력 확인 | 입력 종류 판별, 형태 재기, 빌드 설정 만들기. Stray 면 맵 폴더로 변환 | pass · warn · fail |
| 01 | `convert` | 메쉬 · USD 만들기 | GLB: `pipeline/glb_to_usd.py`. 맵 폴더: `check_input.py` + `run.py --from tsdf` | pass · fail |
| 02 | `nav2` | Nav2 지도 | `sim/nav2/make_map.py`. 빈 바닥 4 m² 미만이면 fail | pass · fail · skip(`make_map.py` 없음) |
| 03 | `diagnose` | 진단 | 바닥 층수 · 보이는 바닥 · 낮은 턱 · 물체 일치 · 천장. `defects_before.json` 씀 | pass · warn · fail |
| 04 | `correct` | 보정 · 다시 진단 | 두 겹 바닥이 있을 때만 다시 빌드 · 다시 진단. `defects_after.json` 씀 | pass · warn · fail · skip |
| 05 | `bundle` | 묶음 만들기 | `bundle/` 만들기 | pass · warn |
| 06 | `verify` | 주행 검증 | 지금은 항상 skip (Isaac Sim 자리, `stages.py:381-383`) | skip |
| 07 | `report` | 보고서 | `report.md` · `report.json`. 앞 단계가 실패해도 항상 돎 | pass · warn (**fail 이 되지 않음**, `stages.py:432`) |

### 3-4. 엔진의 `report.json`

`stages.py:431`: `{"name": <장면 이름>, "verdict": "pass|warn|fail", "stages": [단계 dict …]}`

- `verdict` = `input` ~ `verify` 단계 중 가장 나쁜 것 (`fail` > `warn` > `pass`, `skip` 은 무시) (`stages.py:389`).
- `stages` 에는 `report` 단계 자신이 **없다** (쓰는 시점에 아직 결과가 없음).
- `report` 단계의 `status` 는 `verdict` 가 `fail` 이면 `pass` 로 바뀐다. 그래서 전체 판정은 `report` 단계가 아니라 `verdict` 를 본다.

### 3-5. 단계별 `metrics` 의 키 (지금 코드)

약속은 "있으면 이 이름" 이다. 화면이 쓰는 것은 `diagnose` 의 `samples` · `floor_single_pct` · `floor_double` · `visible_floor_missing_pct` 뿐 (`index.html` `renderCompare`).

| 단계 | 키 |
|---|---|
| `input` (GLB) | `parts` · `textured` · `triangles` · `extent_m` · `floor_tilt_deg` · `floor_area_m2` · `floor_spread_m` · `floor_flat` · `ceiling_area_m2` · `ceiling_over_floor_pct` · `has_ceiling` · `wall_top_m` (`detect.py:69-72`) |
| `input` (맵 폴더 · Stray) | `main_keyframes` · `main_poses` · `extra_sets` · `geometry_mesh` · `has_validation` (`detect.py:86-87`) + Stray 면 `stray` (= `stray_meta.json` 전체, `stages.py:73`). 진단이 같은 dict 에 `ceiling_over_floor_pct` · `has_ceiling` 을 더하는데 (`stages.py:258-261`), 단계 파일은 그 전에 쓰여서 **단계 파일 · 상태 응답의 `metrics.input` 에는 없고 엔진 `report.json` 에는 있다** (`stray1` 에서 확인) |
| `convert` | `mesh_mb` · `prims` ({prim 이름: 삼각형 수}) · `collision` · `clean`. 실패 시 `check_input` |
| `nav2` | `cells` [가로, 세로] · `resolution` · `free_m2` · `occupied_cells` |
| `diagnose` | `samples` · `floor_single_pct` · `floor_double` · `floor_double_pct` · `floor_under` · (`visual.ply` 가 있으면) `visible_floor_missing_pct` · `visible_minus_collision_cm` {p5, median, p95} · `low_bumps` {samples_over_3cm_pct, runs_over_0p6m, longest} · `obstacle_match` {…} · `has_ceiling` · `defects` ({type: 개수}) |
| `correct` (돌았을 때) | `fixed` (예: `["double_floor"]`) · `before` · `after` (각각 `diagnose` 의 metrics 와 같은 모양) |
| `bundle` | `start_pose` (= `start_pose.json`) · `files` (개수) · `shadow_overrides` |

`diagnose` 실제 예 (`stray1/stages/03_diagnose.json`):

```json
{"stage": "diagnose", "status": "warn",
 "summary": "빈 바닥 표본 1,312 · 바닥 한 겹 100.0% · 두 겹 0 · 보이는 바닥 없음 34.8%",
 "metrics": {"samples": 1312, "floor_single_pct": 100.0, "floor_double": 0, "floor_double_pct": 0.0, "floor_under": 0,
  "visible_floor_missing_pct": 34.8, "visible_minus_collision_cm": {"p5": -0.3, "median": 0.0, "p95": 0.1},
  "low_bumps": {"samples_over_3cm_pct": 0.12, "runs_over_0p6m": 0, "longest": []},
  "obstacle_match": {"visible_cells": 360, "collision_cells": 389, "visible_in_collision_pct": 99.7,
                     "collision_in_visible_pct": 100.0, "tol_m": 0.1, "height_band_m": [0.12, 1.0]},
  "has_ceiling": true, "defects": {"no_visible_floor": 4}},
 "messages": ["빈 바닥의 34.8% 에 보이는 바닥이 없음 (받침 바닥으로 메워짐)",
              "천장이 있음 — 씬 루트에 그림자 설정을 넣음 (안 넣으면 Isaac Sim 화면 밝기 0)"],
 "seconds": 2.1}
```

---

## 4. 웹 API

서버: `python -m scan2sim.web.server [--port 8000] [--host 127.0.0.1] [--jobs ~/scan2sim_jobs] [--code <접속 코드>]` (`web/server.py:293-304`).

### 4-1. 공통

- **접속 코드:** `--code` 를 주면 `/api/` 로 시작하는 모든 요청에 코드가 있어야 한다 (`web/server.py:71-74`).
  - 머리말 `X-Code: <코드>` 또는 쿼리 `?code=<코드>`. 머리말이 있으면 머리말만 본다.
  - 없거나 틀리면 `401` `{"error": "접속 코드가 맞지 않음"}`.
  - `GET /` 와 `/static/…` 은 코드를 보지 않는다.
  - 화면은 `fetch` 에는 머리말을 (`index.html:149`), `<img>` · GLB 로더 · 내려받기 링크에는 `?code=` 를 붙인다 (`index.html:157`).
- **오류 응답:** 400 · 401 · 409 는 JSON `{"error": "문장", …}`.
  - **404 는 JSON 이 아니라 Flask 기본 HTML** 이다 (`abort(404)`). 클라이언트는 상태 코드만 보고 판단한다.
  - 작업 번호가 `^[0-9a-f]{12}$` 가 아니거나(`web/server.py:46-47`), 그 작업이 메모리에도 `job.json` 에도 없으면 404.
- **진행 상태는 화면이 `GET /api/jobs/<id>` 를 1.5초마다 불러서 본다** (`index.html:202`). SSE · WebSocket 없음.

### 4-2. 경로

| 메서드 · 경로 | 요청 | 성공 응답 | 실패 |
|---|---|---|---|
| `GET /` | — | `index.html` | — |
| `GET /api/info` | — | `{"ok": true, "stages": [{"name", "label"} ×8], "accept": [".zip", ".glb"]}` (`:82-84`) | 401 |
| `POST /api/jobs` | JSON `{"name": 파일 이름, "size": 바이트}` (`Content-Type` 은 안 봄) | `{"id": "<12자리>"}` (`:87-102`) | 400 `{"error": "받는 파일은 .zip · .glb 임"}` (확장자, 대소문자 무시) · 400 `{"error": "파일 크기가 0 이거나 4 GB 를 넘음"}` · 401 |
| `PUT /api/jobs/<id>/chunk?offset=N` | 본문 = 파일 조각의 날것 바이트. `N` = 지금까지 받은 바이트 수 | `{"received": 누적 바이트}` (`:105-122`) | 409 `{"error": "조각 순서가 맞지 않음", "received": n}` (상태가 `uploading` 이 아니거나 `N ≠ received`) · 400 `{"error": "알린 크기보다 많이 옴"}` · 401 · 404 |
| `POST /api/jobs/<id>/start` | 본문 없음 | `{"status": "queued"}` (`:125-133`) | 409 `{"error": "파일이 다 오지 않음", "received": n}` (상태가 `uploading` 이 아니거나 `received ≠ size` — 이미 넣은 작업을 다시 넣어도 이 문장) · 401 · 404 |
| `GET /api/jobs/<id>` | — | 작업 상태 (4-3) (`:170-193`) | 401 · 404 |
| `GET /api/jobs/<id>/preview.glb` | — | `model/gltf-binary`. 화면 좌표 (1-1) (`:196-247`) | 404 (USD 도 `visual.ply` 도 없음) · 401 |
| `GET /api/jobs/<id>/map.png` | — | `image/png`. `bundle/nav2/map.pgm` 을 PNG 로 (`:250-260`) | 404 · 401 |
| `GET /api/jobs/<id>/report.md` | `?dl=<아무 값>` 이면 내려받기 | `text/markdown; charset=utf-8`. 내려받기 이름 `scan2sim_report_<id>.md` (`:263-269`) | 404 · 401 |
| `GET /api/jobs/<id>/bundle.zip` | — | 내려받기 `scan2sim_<id>.zip` (`:272-290`) | 404 (`bundle/usd/` 없음) · 401 |

**`preview.glb`**

- 원본: `out/build/usd/*_mesh.usdc` 중 이름순 첫 파일, 없으면 `out/build/visual.ply` (색 없음).
- USD 에서는 Mesh prim 중 `CollisionAPI` 가 있거나 안 보이는 것(`Collision` · `Boundary`)을 빼고 옮긴다. 사진 텍스처(정점 단위 `st`) 또는 정점 색(`displayColor`)을 같이 옮긴다. glTF 노드 이름 = prim 이름.
- `out/preview.glb` 에 저장해 두고, 원본이 더 새로우면 다시 만든다.
- 보정이 돌았으면 **보정 뒤 메쉬**다. 화면에서 "보정 전" 결함을 골라도 메쉬는 보정 뒤 것.

**`bundle.zip`**

- 안의 구조: `bundle/…` (2-4 의 `bundle/` 전체) + 맨 위에 `report.md` · `defects_before.json` · `defects_after.json` (있는 것만).
- `out/bundle.zip` 에 한 번 만들면 **다시 만들지 않는다**.
- `e2e_stray` 실제: 파일 13개 (`bundle/` 11개 + `report.md` + `defects_before.json`).

**올리기 순서 (화면이 하는 것, `index.html:179-187`)**

1. `POST /api/jobs {name, size}` → `id`
2. `offset = 0, 8 MiB, 16 MiB …` 차례로 `PUT …/chunk?offset=…`. 조각 크기 8 MiB (`index.html:144`)
3. `POST …/start`
4. 주소에 `#<id>` 를 붙이고 `GET /api/jobs/<id>` 를 1.5초마다 부름. `done` · `failed` 면 멈춤

- 이어 올리기: 서버는 받은 만큼(`received`, 409 응답에도 들어 있음)부터 다시 받는다. 단, 같은 서버 프로세스 안에서만 된다 (재시작하면 `uploading` 작업은 `failed`). **화면은 아직 이어 올리기를 하지 않는다.**

### 4-3. 작업 상태 (`GET /api/jobs/<id>`)

`job.json` 의 필드 (2-2) 전부 + 아래 필드 (`web/server.py:170-193`).

| 필드 | 형 | 언제 | 뜻 |
|---|---|---|---|
| `stages` | 배열 (8개, 3-3 순서) | 항상 | `{name, label, status}` + 결과 파일이 있으면 `summary` · `messages` · `seconds` |
| `queue_ahead` | 정수 | 항상 | `queued` 일 때만 계산, 그 밖은 0. 나보다 먼저 줄에 선 `queued` · `running` 작업 수 (**처리 중인 것도 셈**) |
| `defects_before` | 객체 또는 `null` | `done` · `failed` | `defects_before.json` 내용 (5장). 파일이 없으면 `null` |
| `defects_after` | 객체 또는 `null` | `done` · `failed` | `defects_after.json` 내용. **보정을 안 했으면 `null`** |
| `verdict` | 문자열 | `done` · `failed` | 엔진 `report.json` 의 `verdict` (`pass` · `warn` · `fail`). 없으면 `report` 단계의 `status` (8장 버그: `pending` 이 나올 수 있음) |
| `has_bundle` | 참거짓 | `done` · `failed` | `out/bundle/usd/` 가 있음 = `bundle.zip` 을 받을 수 있음 |
| `metrics` | 객체 | `done` · `failed` | `{"before": …, "after": …, "input": …}` (아래) |

- `metrics.before` = 보정이 돌았으면 `correct` 단계의 `metrics.before`, 아니면 `diagnose` 단계의 `metrics`. 둘 다 없으면 `{}`.
- `metrics.after` = `correct` 단계의 `metrics.after` 또는 `null`.
- `metrics.input` = `input` 단계의 `metrics` 또는 `null`.

**작업 `status`**

| 값 | 뜻 |
|---|---|
| `uploading` | 자리를 만들고 조각을 받는 중 |
| `queued` | 줄에 들어감 |
| `running` | 엔진이 돌고 있음 (한 번에 한 건) |
| `done` | 엔진이 끝났고 **`out/report.md` 가 있음** (`web/server.py:147`). **성공이라는 뜻이 아님** — 단계가 실패해도 `report` 단계는 돌므로 `done` 이 된다. 성공 여부는 `verdict` 와 `exit_code` 로 본다 (예: `e2e_glb` 는 `done` · `exit_code 1` · `verdict fail`) |
| `failed` | `report.md` 가 없음 (엔진이 중간에 죽음), 엔진을 못 띄움(`error`), 또는 서버가 다시 켜져서 끊김(`error`) |

- 서버를 다시 켜면 `uploading` · `queued` · `running` 이던 작업은 읽을 때 `failed` + `error: "서버가 다시 켜져서 끊김"` 이 된다 (`web/server.py:60-61`). 이것은 메모리에서만 바뀌고 `job.json` 에는 다시 쓰지 않는다.

**`stages[].status`**

- 3-2 의 `pass` · `warn` · `fail` · `skip` 에 더해 두 값이 있다 (`web/server.py:174-182`).
  - `pending` = 아직 결과 파일이 없음
  - `running` = 작업이 `running` 이고, 결과 파일이 없는 첫 단계
- `uploading` · `queued` 일 때는 단계 파일을 읽지 않으므로 8개 모두 `pending`.
- `done` · `failed` 여도 결과 파일이 없는 단계는 `pending` 으로 나온다 (엔진이 중간에 죽은 경우).
- `pending` · `running` 단계에는 `summary` · `messages` · `seconds` 가 없다.

실제 예 — `e2e_stray` 와 같은 촬영본의 결과 폴더(`stray1`)를 서버의 `status()` 가 읽은 응답. `created` · `started` 같은 시각과 `size` 는 예시로 넣은 값이고, 나머지는 서버 코드가 만든 그대로다. 배열은 줄였다.

```json
{
 "id": "89d34e717918", "name": "stray_9a58.zip", "size": 50394194, "received": 50394194,
 "status": "done", "created": 1795441200.0, "file": "input.zip",
 "queued_at": 1795441200.3, "started": 1795441200.3, "exit_code": 0, "finished": 1795441254.4,
 "queue_ahead": 0,
 "stages": [
  {"name": "input", "label": "입력 확인", "status": "warn", "seconds": 3.8,
   "summary": "아이폰 원본 → 맵 폴더 · 주 묶음 136장 · 보조 묶음 0개 · 형상 메쉬 없음",
   "messages": ["촬영 범위가 좁음 (이동 6.41 m · 카메라가 움직인 범위 1.0 m) — 로봇이 다닐 맵으로는 부족할 수 있음",
                "아이폰 자세에는 되돌아온 자리를 맞추는 보정이 없음 — 긴 구간은 틀어질 수 있음 (얼마나인지는 재지 않음)"]},
  {"name": "convert", "label": "메쉬 · USD 만들기", "status": "pass", "summary": "USD 3.33 MB · prim 7개", "messages": [], "seconds": 45.2},
  {"name": "nav2", "label": "Nav2 지도", "status": "pass", "summary": "217×203 칸 (0.05 m) · 빈 바닥 28 m²", "messages": [], "seconds": 0.4},
  {"name": "diagnose", "label": "진단", "status": "warn", "summary": "빈 바닥 표본 1,312 · …", "messages": ["…"], "seconds": 2.1},
  {"name": "correct", "label": "보정 · 다시 진단", "status": "skip", "summary": "진단에 걸린 항목이 없어 보정하지 않음", "messages": [], "seconds": 0.0},
  {"name": "bundle", "label": "묶음 만들기", "status": "pass", "summary": "묶음 파일 10개 · 시작 (-3.178, 0.456) 33.9° · 여유 1.01 m", "messages": [], "seconds": 0.0},
  {"name": "verify", "label": "주행 검증", "status": "skip", "summary": "Isaac Sim 이 필요함 — …", "messages": [], "seconds": 0.0},
  {"name": "report", "label": "보고서", "status": "warn", "summary": "report.md — 올릴 수 있으나 아래 주의 항목을 볼 것", "messages": [], "seconds": 0.0}
 ],
 "verdict": "warn",
 "has_bundle": true,
 "metrics": {
  "before": {"samples": 1312, "floor_single_pct": 100.0, "floor_double": 0, "visible_floor_missing_pct": 34.8, "…": "…"},
  "after": null,
  "input": {"main_keyframes": 136, "main_poses": 136, "extra_sets": [], "geometry_mesh": null, "has_validation": false,
            "stray": {"source": "9a5813208f", "keyframes": 136, "…": "…"}}
 },
 "defects_before": {"phase": "before", "T_input_to_scene": [["…"]], "up_input": [-0.00465, 0.00357, 0.99998],
                    "floor_point_input": [0.0011, -0.0009, -0.2469], "defects": [{"id": "D1", "…": "…"}]},
 "defects_after": null
}
```

처리 중(`running`)이면 `verdict` · `has_bundle` · `metrics` · `defects_*` 가 없고, `stages` 는 예를 들어 `input warn · convert running · nav2 pending · …` 처럼 나온다.

---

## 5. 결함 목록 (`defects_before.json` · `defects_after.json`)

`stages._defects` 가 쓴다 (`stages.py:145-184`). 진단 단계(`s_diagnose`)의 끝에서 불린다 (`stages.py:265`).

### 5-1. 파일

| 필드 | 형 | 뜻 |
|---|---|---|
| `phase` | `"before"` · `"after"` | 보정 전 / 보정 뒤 다시 진단 (`stages.py:178`) |
| `T_input_to_scene` | 4×4 수 배열 | `build/transform.json` 의 값. 없으면 단위행렬 (1-3) |
| `up_input` | [3] 수 | 장면의 +Z 를 입력 좌표로 나타낸 단위 벡터 = `T⁻¹` 의 셋째 열 (소수 5자리) |
| `floor_point_input` | [3] 수 | 장면 원점 (0, 0, 0) 을 입력 좌표로 나타낸 점 = `T⁻¹` 의 이동 부분 (소수 4자리). 장면 바닥 z = 0 위의 한 점 |
| `defects` | 배열 | 결함 (5-2). 비어 있을 수 있음 |

### 5-2. 결함 하나

| 필드 | 형 | 뜻 |
|---|---|---|
| `id` | 문자열 | `D1`, `D2` … 정렬한 뒤 순서대로 매김 (`stages.py:175`). 한 파일 안에서만 유일. **보정 전과 뒤의 번호는 서로 관계없음** |
| `type` | 문자열 | `double_floor` · `no_visible_floor` · `low_bump` (5-3) |
| `title` | 문자열 | 화면에 보일 이름 (한국어) |
| `severity` | `"fail"` · `"warn"` | `double_floor` 만 `fail` |
| `fixable` | 참거짓 | `true` = 자동 보정 대상 (`double_floor` 만) |
| `center_scene` | [x, y, z] 수 | 장면 좌표의 중심. x · y = 묶인 표본의 평균, z = 그 자리의 바닥 높이(`ground.height`) (소수 3자리) |
| `center_input` | [x, y, z] 수 | `T⁻¹ · center_scene` (소수 4자리). 입력 좌표 (1-3) |
| `radius_m` | 수 | 중심에서 가장 먼 표본까지 거리 **+ 지도 2칸(0.1 m)** (`checks.py:43`) |
| `extent_m` | 수 | 묶인 표본의 x · y 범위 상자의 대각선 **+ 지도 1칸(0.05 m)** (`checks.py:44`) |
| `samples` | 정수 | 묶인 표본 수 |
| `basis` | 문자열 | 판정 근거 문장 |
| `action` | 문자열 | 처리 방법 문장 |

- **정렬:** `fail` 먼저, 그 안에서 `samples` 가 많은 순 (`stages.py:167`).
- **최대 30개:** 정렬한 뒤 앞 30개만 남김 (`stages.py:145` `top=30`, `:168`).
- 화면은 `center_scene` · `radius_m` · `extent_m` · `severity` · `fixable` · `title` · `basis` · `action` · `id` 를 쓴다.

실제 예 (`stray1/defects_before.json`, 결함 4개 중 1개):

```json
{
 "phase": "before",
 "T_input_to_scene": [[0.99999, 8.3e-06, 0.00465, -0.0], [8.3e-06, 0.99999, -0.00357, -0.0], [-0.00465, 0.00357, 0.99998, 0.24695], [0.0, 0.0, 0.0, 1.0]],
 "up_input": [-0.00465, 0.00357, 0.99998],
 "floor_point_input": [0.0011, -0.0009, -0.2469],
 "defects": [
  {
   "type": "no_visible_floor",
   "title": "보이는 바닥 없음 (재촬영 후보)",
   "severity": "warn",
   "fixable": false,
   "radius_m": 1.51,
   "extent_m": 2.91,
   "samples": 228,
   "basis": "빈 바닥 표본 228개(약 2.3 m²)에서 보이는 바닥 면을 찾지 못함. 충돌 바닥은 받침 바닥으로 메워져 로봇은 지나가지만 카메라에는 구멍으로 보임",
   "action": "고치지 않음 — 촬영 때 가려졌거나 찍히지 않은 자리. 다시 찍을 후보",
   "id": "D1",
   "center_scene": [-0.236, -0.044, 0.003],
   "center_input": [-0.2349, -0.0449, -0.2444]
  }
 ]
}
```

(`T_input_to_scene` 은 읽기 쉽게 줄였다. 실제 파일은 전체 자릿수.) 위 값은 1-3 을 그대로 따른다: `center_input` 의 z(−0.2444) ≈ 장면 바닥(0.003) − 0.247.

### 5-3. 표본과 묶기

- **표본** (`stages.py:134-142`, `:194`)
  - Nav2 지도의 빈 칸 중 장애물 · 미지 칸에서 0.3 m(`ROBOT_RADIUS_M`, `stages.py:20`) 넘게 떨어진 칸의 중심.
  - 그 칸들을 행 우선 순서로 세서 4개마다 하나 (`step=4`).
  - 표본 하나가 대표하는 넓이를 `4 × 칸²` 로 친다 (= 0.01 m², `stages.py:150`).
  - 표본이 50개 미만이면 진단 단계가 `fail` (`stages.py:195-196`).
- **위에서 쏘는 광선:** 표본마다 그 자리 바닥 높이 + 1.0 m 에서 아래로.
- **묶기** (`checks.clusters`, `checks.py:26-45`)
  - 걸린 표본을 Nav2 지도 칸에 찍고, 2칸 넓혀(`binary_dilation`, `iterations=2`) 8방향으로 이어진 것끼리 한 덩어리.
  - 덩어리마다 `center` · `radius_m` · `extent_m` · `samples`.

### 5-4. 결함 종류 (지금 구현의 기준값)

| `type` | `severity` · `fixable` | 걸리는 표본 | 결함으로 남기는 덩어리 | 근거 위치 |
|---|---|---|---|---|
| `double_floor` 충돌 바닥 두 겹 | `fail` · `true` | 충돌 prim `Collision` 에 쏜 광선이 맨 위 면을 만난 뒤, 그 바로 아래(1e-4 m)에서 다시 쏜 광선이 **3 cm 안**에서 면을 또 만남 | 모든 덩어리 (크기 하한 없음) | `tools/check_floor_layers.py:57-60`, `stages.py:152-155` |
| `no_visible_floor` 보이는 바닥 없음 | `warn` · `false` | `build/visual.ply` 에 쏜 광선이 안 맞거나, 맞은 높이가 충돌 바닥과 **0.3 m 이상** 차이 | `samples × 0.01 m² ≥ 0.25 m²` (= 표본 25개 이상, 칸 0.05 m 기준) | `stages.py:223`, `:157-161` |
| `low_bump` 낮은 턱 의심 | `warn` · `false` | 보이는 바닥이 충돌 바닥보다 **3 cm 초과 12 cm 미만** 높음 | `extent_m ≥ 0.6` | `stages.py:235`, `:162-166` |

- `no_visible_floor` · `low_bump` 는 `build/visual.ply` 가 있을 때만 찾는다 (`stages.py:216`).
- `diagnose` metrics 의 `low_bumps.runs_over_0p6m` 은 **다른 방법**(지도에서 이은 영역의 상자 대각선)으로 센 값이다 (`stages.py:237-245`). 결함 목록의 `low_bump` 개수와 다를 수 있다.

**보정 (`s_correct`, `stages.py:270-297`)**

- 진단에서 `double_floor` 가 하나라도 나오면 돈다 (`ctx["needs"]`, `stages.py:210`). 다른 종류로는 돌지 않는다.
- 하는 일
  - GLB: 충돌 메쉬에서 바닥 위 5 cm 아래를 지움 (`collision.low_h_m: 0.05`)
  - 맵 폴더 · Stray: 바닥 위 10 cm 안의 낮은 조각과 6 cm 안의 수평 면을 지움 (`single_floor.collision_low_h_m: 0.10`, `collision_flat_h_m: 0.06`)
- 그 뒤 `convert` → `nav2` → `diagnose` 를 다시 돌린다. 이때 `defects_after.json` 을 쓴다.
- **`defects_after.json` 은 보정이 돌았을 때만 생긴다.** 보정 단계가 `skip` 이면 없고, 상태 응답의 `defects_after` 는 `null`.
- 보정 뒤에도 두 겹 바닥이 남으면 `correct` 단계가 `fail` (`stages.py:205-206`).

---

## 6. Stray Scanner 입력 (`stray.py`)

### 6-1. 받는 것

- 촬영 폴더에 `rgb.mp4` · `odometry.csv` · `camera_matrix.csv` · `depth/` 가 있어야 한다 (`stray.NEED`). 넷 중 둘 이상이 있으면 촬영 폴더로 보고(`stray._looks_stray`), 빠진 것을 이름으로 알려 주며 실패한다.
- zip 은 풀지 않고 이름만 보고 촬영 폴더를 찾는다 (`stray._zip_captures`). macOS 가 넣는 `__MACOSX/` · `._*` 는 무시한다.
- 처리할 때는 `out/input_map/_unzip/` 에 풀고 그 안의 촬영 폴더를 쓴다. 폴더가 몇 겹 아래에 있어도 된다.
- 풀기 전에 막는 zip (`stray.find_capture`): 깨진 zip, 촬영 폴더가 여러 개, 파일 10만 개 또는 풀면 6 GB 넘음(`ZIP_MAX_FILES` · `ZIP_MAX_BYTES`), 폴더 밖을 가리키는 경로.
- **입력 검사** (`stray.validate`, `convert` 첫머리). 실패면 `stray.CaptureError` 를 내고, 그 문장이 `input` 단계의 실패 이유가 된다.

| 검사 | 실패 | 주의만 |
|---|---|---|
| 촬영 길이 | 2 초 또는 100 장보다 짧음 (`MIN_S` · `MIN_ROWS`) | — |
| `odometry.csv` | 열이 13개보다 적음 · 숫자 아님 · NaN · 사원수 길이가 1 에서 1% 넘게 벗어남 · 시각이 커지지 않음 (줄 번호와 함께) | — |
| `depth/` · `confidence/` 장 수 | 자세 줄 수와 3장 넘게 다름 | 1~2장 다름 (`COUNT_TOL`), `confidence/` 없음(신뢰도로 거르지 않음) |
| `rgb.mp4` | 열 수 없음 · 장 수가 자세 줄 수의 90% 미만 (`VIDEO_MIN`) · 4:3 이 아님 | — |
| 내부 파라미터 | `camera_matrix.csv` 가 3×3 숫자가 아님 · fx · fy ≤ 0 · 주점이 영상 밖 | — |

- 주의는 `stray_meta.json` 의 `checks.warnings` 에 남는다 (6-3).

### 6-2. 변환 값 (`stray.convert` 기본값, `stray.py:115`; 엔진은 기본값만 씀)

| 항목 | 값 | 뜻 |
|---|---|---|
| `width` | 960 | 키프레임 가로 (색 1920×1440 → 960×720). 내부 파라미터는 `camera_matrix.csv` 값에 같은 비율을 곱함 |
| `min_conf` | 1 | 신뢰도(0 · 1 · 2) 1 미만 깊이는 0 으로 (`stray.py:154`) |
| `max_depth_m` | 5.0 | 5 m 넘는 깊이는 0 으로 (`stray.py:155`) |
| `move_m` · `turn_deg` | 0.05 · 5.0 | 앞에 고른 장에서 5 cm 넘게 움직였거나 5° 넘게 돌았을 때 키프레임으로 고름 (`stray.py:99-112`) |
| `max_speed` | 1.5 m/s | 5프레임 평균 속력이 이보다 빠른 장은 뺌 (흐림) |
| `skip_s` | 1.0 | 촬영 첫 1초는 건너뜀 (시작 직후 자세가 튐) |
| 색 저장 | JPEG 품질 92, 영역 평균 축소 | (`stray.py:158`) |
| 깊이 저장 | 16비트 PNG(mm), 색 크기로 **최근접** 확대 | 섞으면 가장자리에 없는 점이 생김 (`stray.py:159`) |
| 자세 | `Y_UP_TO_Z_UP · T_stray` | 1-3 |

- 빌드 설정에서 Stray 만 TSDF 칸 2 cm, 깊이 한계 3 m 를 쓴다 (`detect.py:116-117`). 변환은 5 m 까지 남기지만 맵에는 3 m 까지만 들어간다.
- 쓸 수 있는 장이 2장 미만이면 `CaptureError`.
- 6-2 표의 `stray.py:줄` 은 입력 검사를 넣기 전(main `a069f09`) 기준이다. 지금은 함수 `convert` · `select` 안에서 찾을 것.

### 6-3. `input_map/stray_meta.json` (`stray.convert` 끝)

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
| `color_size` · `depth_size_in` · `keyframe_size` | [가로, 세로] | 원본 색 · 원본 깊이 (고정 [256, 192]) · 저장한 키프레임 |
| `intrinsics` | {fx, fy, cx, cy} | 키프레임 크기 기준 |
| `focal_variation_pct` | 수 | 프레임별 초점거리의 변동 폭 (%) — 한 값으로 씀 |
| `confidence_pct` | [3] 수 | 고른 장의 신뢰도 0 · 1 · 2 비율 (%) |
| `min_confidence` | 정수 | `min_conf` |
| `valid_depth_pct` | 수 | 걸러낸 뒤 남은 깊이 화소 비율의 평균 (%) |
| `select` | 객체 | `{move_m, turn_deg, max_speed_mps, max_depth_m, skip_s}` |

실제 파일 (`stray1`):

```json
{"source": "9a5813208f", "format": "stray_scanner", "frames_total": 839, "frames_read": 838, "keyframes": 136,
 "dropped_fast": 0, "duration_s": 17.1, "path_length_m": 6.41, "extent_m": [1.0, 0.94, 0.44],
 "camera_height_range_m": [-0.08, 0.36], "speed_median_mps": 0.25, "speed_max_mps": 5.1,
 "color_size": [1920, 1440], "depth_size_in": [256, 192], "keyframe_size": [960, 720],
 "intrinsics": {"fx": 666.417, "fy": 666.417, "cx": 479.022, "cy": 362.894}, "focal_variation_pct": 1.49,
 "confidence_pct": [24.5, 23.0, 52.6], "min_confidence": 1, "valid_depth_pct": 60.1,
 "select": {"move_m": 0.05, "turn_deg": 5.0, "max_speed_mps": 1.5, "max_depth_m": 5.0, "skip_s": 1.0}}
```

**입력 단계가 Stray 에 대해 내는 주의 (`stages.py:74-80`)**

- 촬영 범위가 좁음: `path_length_m < 5` 또는 `max(extent_m[:2]) < 3`
- 깊이가 적음: `valid_depth_pct < 50`
- 촬영이 빠름: `speed_median_mps > 0.8`
- 되돌아온 자리 보정 없음: **항상** 붙음. 그래서 Stray 입력의 `input` 단계는 언제나 `warn` 이다.

`input_map/trajectory_camera_optical.txt` 의 형식: `시각 x y z qx qy qz qw id` (카메라 광학 → 입력 세계, 첫 줄은 `#` 주석).

---

## 7. 한도 (구현된 값)

| 항목 | 값 | 위치 |
|---|---|---|
| 받는 확장자 | `.zip` · `.glb` (대소문자 무시). 화면도 같은 검사 | `web/server.py:37`, `:91-93` · `index.html:162` |
| 파일 크기 | 1 바이트 ~ `MAX_BYTES` = 4 × 1024³ = 4,294,967,296 바이트 | `web/server.py:38`, `:94` |
| 조각 크기 | 화면이 8 MiB(8,388,608 바이트)로 나눔. 서버는 조각 크기를 제한하지 않고 1 MiB 씩 읽음 | `index.html:144` · `web/server.py:114` |
| 작업 번호 | `^[0-9a-f]{12}$` | `web/server.py:46` · `index.html:358` |
| 파일 이름 저장 길이 | 앞 120자 | `web/server.py:98` |
| 동시 처리 | 처리 스레드 **하나** (한 번에 한 작업, 나머지는 줄). 요청 받기는 Flask 내장 서버 `threaded=True` | `web/server.py:302`, `:304` |
| 기본 주소 | `127.0.0.1:8000`. 밖에 열 때는 `--code` 를 줄 것 | `web/server.py:295-296` |
| 결함 수 | 파일마다 최대 30 | `stages.py:145` |
| 화면 폴링 | 1.5초 | `index.html:202` |
| 엔진 쪽 실패 기준 (참고) | GLB 범위 1 m 미만 · 300 m 초과, 바닥 2 m² 미만 · 빈 바닥 4 m² 미만 · 진단 표본 50개 미만 | `stages.py:55-58`, `:129-130`, `:195-196` |
| 처리 시간 제한 | 없음 (엔진이 끝날 때까지 기다림) | `web/server.py:145` |

---

## 8. 계획과 다른 점

계획 = `docs/implementation.md` 5장 (2판, 11/23). 위가 코드에서 확인한 것이다.

### 8-1. 계획이 틀렸던 것

1. **Stray · 맵 폴더의 `T_input_to_scene` 은 단위행렬이 아니다.**
   - 계획 5-1 의 [가정] "Stray · 맵 폴더는 `transform.json` 이 없어 단위행렬, `center_input = center_scene`" 은 틀렸다.
   - `pipeline/build_usd.py:248`, `:260` 이 `transform.json` 을 쓴다. 내용은 바닥 수평 맞춤 + 바닥 z 이동이다 (`stray1`: 0.34°, +0.247 m).
2. **Stray 의 "입력 좌표" 는 올린 원본 좌표가 아니다.**
   - 계획 5-1 은 "올린 GLB 의 좌표 그대로" 라고만 적었다.
   - Stray 는 `input_map` 세계(Stray +Y 위 세계를 `(x, −z, y)` 로 돌린 것)다 (1-3).
3. **`radius_m` · `extent_m` 의 정의가 계획과 조금 다르다.**
   - `radius_m` 은 가장 먼 표본까지 거리에 0.1 m 를 더한 값이다.
   - `extent_m` 은 범위 상자의 대각선에 0.05 m 를 더한 값이다 (5-2).
4. **`no_visible_floor` 의 넓이 기준**은 "0.25 m² 넘는 곳" 이 아니다.
   - 실제로는 "표본 수 × 0.01 m² ≥ 0.25" 이다. 실제 구멍 넓이가 아니라 표본으로 어림한 넓이다.
5. **`route.csv` 는 GLB 입력에만 생긴다.** 계획 5-2 는 늘 있는 것처럼 적었다.
6. **`out/input_map/` 에 `_unzip/`(원본 촬영 전체, 사람이 찍힌 `rgb.mp4` 포함)과 `keyframes/` 가 더 있다.** 개인정보 정리(계획 10장) 대상이다.
7. **상태 `done` 은 "엔진이 성공함" 이 아니다** (4-3).
   - `report.md` 만 있으면 `done` 이다. 실제 `e2e_glb` 는 `done` · `exit_code 1` · `verdict fail` 이었다.
   - 계획 5-4 와 `tests/e2e.py` 의 "상태가 done" 검사는 이것을 성공으로 본다.
8. **404 응답은 `{error}` JSON 이 아니라 HTML 이다.** 계획 5-6 은 "오류는 `{error}`" 라고 적었다.
9. 상태 응답에 계획 예시에 없는 필드가 있다: `file` · `queued_at` · `error`.
10. **맵 폴더 zip 은 웹으로 올릴 수 없다.**
    - `web/server.py:6` 의 설명은 "맵 폴더를 묶은 zip" 을 받는다고 적었다.
    - 그러나 엔진의 `detect.kind_of` 는 Stray 가 아닌 zip 을 알아보지 못한다 (`detect.py:13-20`). 입력 단계에서 `"입력을 알아볼 수 없음"` 으로 실패한다.
11. **웹으로 올린 작업은 장면 이름이 모두 `input` 이다** (2-4).
    - USD 가 `input_mesh.usdc` 가 되고, `scene_root.usda` 가 `data/usd/input/…` 을 가리키고, `bundle.json` 의 `name` 도 `input` 이다.
    - 여러 장소를 `sim/` 에 들이면 이름이 겹친다. 계획 5-2 는 "이름은 서버가 정함" 까지만 적었다.

### 8-2. 계획에는 있고 아직 없는 것

- `GET /api/samples`, `POST /api/samples/<name>/jobs`, `GET /api/health` (계획 5-6 "추가하기로 한 것", D 담당) — 서버에 없다. 지금 경로는 4-2 표가 전부다.
- 화면의 이어 올리기. 서버는 같은 프로세스 안에서만 된다 (`received` 를 `job.json` 에 조각마다 저장하지 않음).
- `bundle/` 을 D 의 `sim/add_site.py` 형식에 맞추기 (계획 3-2 · 5-2). 아직 지금 형식 그대로다.
- "같은 입력에 같은 결과" (계획 5-3) — 아직 보장하지 않음 (`tests/repeat.py` 로 확인 중).

### 8-3. 버그로 보이는 것 (고칠 사람이 정할 것)

| # | 위치 | 내용 |
|---|---|---|
| 1 | `core.py:53` + `pipeline/glb_to_usd.py:88` (`detect.profile_glb` 경유) | 단계가 `except Exception` 만 잡아서 `SystemExit` 이 새어 나간다. 이때 단계 파일도 `report.md` 도 없이 엔진이 끝난다. 그러면 서버 상태는 `failed`, 단계는 모두 `pending`, `verdict` 는 `"pending"` (#2) 이 된다. 예: 바닥을 못 찾는 GLB. (`stray.py` 쪽은 B 가 `CaptureError` 로 바꿔 고침, `ba960b4`) |
| 2 | `web/server.py:188` | `report.json` 이 없으면 `verdict` 를 `report` 단계 상태로 대신하는데, 그 단계가 없으면 `"pending"` 이 들어간다. 화면은 그 글자를 그대로 보인다 |
| 3 | `web/server.py:117-120` | 조각이 알린 크기를 넘으면 400 을 돌려주지만, 그 전까지 읽은 부분은 이미 파일에 붙은 채이고 `received` 는 그대로다. 같은 `offset` 으로 다시 보내면 파일이 어긋난다 |
| 4 | `web/server.py:280` | `bundle.zip` 을 한 번 만들면 다시 만들지 않는다. 처리 중(`bundle` 단계 뒤 · `report` 전)에 요청하면 `report.md` 가 빠진 zip 이 남는다 |
| 5 | `web/server.py:90`, `:108` | `size` · `offset` 이 숫자가 아니면 `int()` 예외로 500 이 난다 (400 이 아님) |
| 6 | `web/server.py:6` vs `detect.py:13-20` | 맵 폴더 zip 을 받는다는 설명과 실제 동작이 다르다 (8-1 의 10번) |
| 7 | `recon/requirements.txt` | GLB 빌드가 쓰는 `scikit-image`(`pipeline/glb_to_usd.py:395`)가 목록에 없다. `e2e_glb` 는 이것 때문에 `convert` 에서 실패했다 (`ModuleNotFoundError: No module named 'skimage'`). `flask` · `opencv-python` 도 목록에 없다 [추정: 이 환경에는 따로 설치돼 있어 Stray 는 돌았음] |
