# CONTRACT.md — scan2sim 약속 (엔진 · 서버 · 화면이 주고받는 것)

**기준 코드: `b/yunseong` `4823a6e`** (main `6ec3237` 을 합친 것). B 의 `stray.py` · `stray_fix.py` · `stray_mesh.py` · `stray_quality.py` 는 줄 번호 대신 함수 · 상수 이름(`stray.convert`, `stray.MAX_KEYFRAMES` 등)으로 적는다.
처음 판의 기준은 `d019a22` (main 의 C · D 11/24 변경 `736eb68` ~ `ed7220e` 을 합친 `ed4ee25` + 시험 · 기록 커밋 하나)이고, 다른 파일의 `파일:줄` 은 그 커밋 기준이다.
**고침 11/24 저녁 (`ccaca39`, C 의 `620a2b0` 을 합침):** 씬 루트가 둘로 나뉜 것(2-4)과 R3 · R4 가 붙은 것을 반영했다. `620a2b0` 이 `stages.py` 에 줄을 더해, 이 문서의 `stages.py:줄` 가운데 입력(`s_input`, 5~7줄) · 진단 · 묶음 단계 쪽은 지금 코드보다 앞을 가리킬 수 있다 — 함수 이름으로 찾을 것.
**고침 11/24 밤 (`b900e76` + 작업 트리):** B 의 Stray 깊이 규칙 · 깊이 파일 짝짓기 · zip 처리(6-1 ~ 6-3), 자세 보정과 그 결과 파일(6-4), 아직 C 가 넣지 않은 R10 · R11 과 `--resume` 제안(8장)을 반영했다.
**고침 11/25 (`4823a6e`):** C 의 `be8310e` (R9 ~ R12 · `--resume`) · `105591c` (걸어간 길 진단) · `6ec3237` (`SystemExit`), D 의 `533bffe` (이어 올리기 · 작업 되살리기 · 작업 폴더 정리) · `6afb822` 를 반영했다 (C 의 요청 C1 · C2, `C_TRACK.md` 6장). `core.py` · `checks.py` · `web/server.py` · `web/static/index.html` 의 줄 번호는 모두 `4823a6e` 기준이다. `stages.py` 는 이번에 고친 곳만 `4823a6e` 기준이고, 다른 곳은 앞뒤로 어긋날 수 있다 — 함수 이름으로 찾을 것.
**고침 11/25 저녁 (`35063bc`, main `5db0573` 을 합친 것):** D 의 `9b2c95f` (서버 보호 · 예시 파일 · `/api/health`) · `1e70ddb` (`sim/add_site.py`) · `bef9cda` (`bundle.zip` 맨 위를 폴더 하나로 · 배포 스크립트의 새 선택 사항)를 반영했다 (2장 · 4장 · 7장 · 8장). **`web/server.py` 와 `deploy/*.sh` 의 줄 번호는 이제 `35063bc` 기준이다.** `core.py` · `checks.py` · `stages.py` · `web/static/index.html` 은 `4823a6e` 뒤로 바뀌지 않아 줄 번호가 그대로다. B 의 `5db0573` (6-4 자동 고르기 · 6-5)은 그 커밋에서 고쳤다.
**고침 11/26 (`647e3c7`, main `13d1c4d` 를 합친 것):** C 의 `13d1c4d` (R14 · R15 · R19 · R20 · 제자리 회전 가능 넓이)를 반영했다 (3-1 · 3-5 · 5-4 · 6-4 · 6-5 · 8-3 의 11번). 고친 곳의 `stages.py` · `checks.py` 줄 번호만 `647e3c7` 기준이다 — `13d1c4d` 가 `stages.py` 에 줄을 더해 다른 곳의 `stages.py:줄` 은 앞을 가리킬 수 있으니 함수 이름으로 찾을 것. D 의 `d03388f` (화면 Nav2 지도 숨기기 버튼)는 약속에 닿지 않는다.
**고침 11/26 낮 (`6bdab12`, main `9706eee` 를 합친 것):** C 의 `ecaae53` (충돌 물체 삼각형 15만) · `29a05ba` (R16 판만 더하기) · `9706eee` (C12 · C13 닫음), D 의 `6ad7114` (R17 · R21 — 끝난 작업 표시 · `offset` 400 · `--keep` 도움말 · `S2S_KEEP`) · `6e88feb` (`sim/add_site.py` 가 묶음의 `scene_root_sim.usda` 를 받음)를 반영했다 (2-1 · 2-4 · 4장 · 5-4 · 8장). 고친 곳의 `web/server.py` · `stages.py` 줄 번호만 `6bdab12` 기준이다.

작성 2026-11-23 · B · 고침 2026-11-24 (C · D 의 11/24 변경 반영 · B 의 Stray 깊이 규칙 · 자세 보정) · 고침 2026-11-25 (C · D 의 11/25 변경 반영) · 고침 2026-11-25 저녁 (D 의 서버 보호 · `bundle.zip` 구조) · 고침 2026-11-26 (C 의 `13d1c4d`) · 고침 2026-11-26 낮 (C · D 의 11/26 오전 변경)

- 이 문서는 **지금 코드가 하는 일**을 적는다. 계획(`docs/implementation.md` 5장)과 다른 곳과 버그로 보이는 것은 8장에 모았다.
- 근거 표시: 코드 위치는 `파일:줄`. 경로는 `recon/scan2sim/` 기준이고, `pipeline/` · `tools/` · `configs/` 는 `recon/` 아래다. 코드를 읽지 않고 짐작한 것은 **[추정]** 이라고 적었다.
- 예시 JSON 은 실제 실행 결과에서 가져왔다. 긴 배열은 줄였다(`…`).
  - `room` = 서버를 띄워 방 촬영본(Stray zip `6bc1ae021f.zip`, 118초 · 379 MB, E 가 찍음)을 API 로 올린 시험. 기준 코드에서 `tests/e2e.py` 로 돌림. 작업 `694ddc986821`, 결과 · 엔진 출력은 B 노트북 작업 폴더(저장소 밖). 15/15 통과 · `done` · 판정 `warn` · 82초.
    - `room` 은 `be8310e` 전 엔진(형상 메쉬 `pipeline/tsdf.py`)의 결과다. 지금 엔진으로 같은 촬영본을 돌리면 결함 · 수치 · 문장이 다르다 (C 측정: 결함 6 → 1곳, `floor_two_levels` 16~17 → 2.9%, `C_TRACK.md` 2-5).
  - `e2e_glb` = 11/23 에 서버로 GLB(`corridor_v04.glb`)를 올린 시험 (작업 `b8ce80baff04`). 그때 환경에 `scikit-image` 가 없어 변환 단계에서 실패했다. 8장에서 고쳐진 항목을 설명할 때만 쓴다.
  - 11/23 판의 `stray1` · `e2e_stray` 예시는 Stray 를 맵 폴더 길로 돌린 결과라 지금 출력과 모양이 달라 뺐다 (1-2).
  - `ref` = B 가 `room` 과 같은 촬영본을 엔진 밖에서 11/24 밤 작업 트리의 `stray.convert` → `stray_mesh.build` (기본값) → `stray_quality.measure` 로 돌린 결과. 지금 엔진(R10, C `be8310e`)도 같은 함수를 기본값으로 부르므로(`stages.s_input` · `s_diagnose`) Stray 중간 파일이 이 모양이다. C 가 엔진으로 잰 복원 정확도(중앙 1.78 cm)도 같다 (`C_TRACK.md` 2-5). 6장에서만 쓴다.

---

## 0. 이 약속을 바꾸는 법

1. API 경로 · 응답 필드, 작업 폴더의 파일 이름, 결함 목록의 필드를 **바꾸거나 없앨 때**는 먼저 팀 채팅에 알린다.
2. 그 변경과 **같은 커밋**에서 이 파일을 고친다.
3. **필드를 더하는 것은 자유**다(알리지 않아도 됨). 대신 화면 · 시험 · 다른 도구는 모르는 필드를 무시해야 한다.
   - 화면은 모르는 결함 `type` 이 와도 목록에 보여 줘야 한다. 지금 화면은 `type` 을 보지 않고 `severity` · `title` 로 그린다 (`web/static/index.html:270-289`).
4. `tests/` 의 시험이 이 약속을 검사한다. 시험이 깨지면 바꾼 사람이 그날 안에 고친다.
   - `tests/e2e.py`: 서버를 띄워 API 로 올리고 상태 · 결함 필드 · 결과 파일 4개를 검사.
     검사하는 키는 `tests/e2e.py:35-38` (`JOB_KEYS` · `DONE_KEYS` · `STAGE_KEYS` · `DEFECT_KEYS`).
     서버는 `--port` · `--jobs` · `--code` 만 주고 띄운다 (`tests/e2e.py:101`). 그래서 서버 보호의 기본값(4장 — 제한 시간 20분 · 디스크 여유 20 GB 등)이 시험에도 걸린다 (8-3 의 15번).
   - `tests/repeat.py`: 같은 입력을 두 번 돌려 결함 목록이 같은지 검사.
   - `tests/run.sh`: 가상의 방 GLB · 가상 Stray 촬영본으로 위 둘을 돌리고, 넣어 둔 결함 3개가 제자리(0.5 m 안)에 잡히는지 본다 (`--truth`). Stray 도 11/24 부터 견준다 (`d019a22`).
   - 이 문서의 필드 표를 고치면 위 키 목록도 같이 고친다.
5. 이 약속을 쓰는 다른 도구: D 의 `deploy/check.py`. 화면과 같은 순서로 올리고, 상태 `done` · `has_bundle` · `report.md` · `preview.glb` · `bundle.zip` 을 다 받으면 `CHECK_OK` (`deploy/check.py:86`). `verdict` 는 보지 않는다 — `done` 이어도 `verdict` 가 `fail` 일 수 있다 (3-4).

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
    서버는 이 파일을 주기만 하고, 파일이 없거나 USD 보다 오래됐을 때만 같은 엔진 함수(`preview.write`)를 부른다 (`web/server.py:483-490`).
    11/23 까지는 서버가 직접 변환했다. 11/24 에 엔진으로 옮겨졌다 (C `2a6e89f`, D `ed7220e`).
  - **화면**이 결함 표시를 놓을 때: `web/static/index.html:307` `toView = (p) => (p[0], p[2], -p[1])`.
- 거꾸로(화면 → 장면)는 `장면 (x, y, z) = 화면 (x, −z, y)`.
- Nav2 지도(`bundle/nav2/map.pgm`)는 장면 좌표의 위에서 본 그림이다.
  - `map.yaml` 의 `origin` 은 그림 왼쪽 아래 칸 모서리의 장면 (x, y).
  - 그림의 행 0 = 위쪽(y 가 큰 쪽) (`checks.py:8`). 칸 = `resolution` m (지금 0.05).

### 1-2. 장면 좌표의 원점과 바닥

| 입력 | 수평 맞춤 | 바닥 z = 0 | x · y 원점 |
|---|---|---|---|
| GLB · **Stray** | 바닥 평면 법선 → +Z (`pipeline/glb_to_usd.py:79-130` `find_floor`). 위를 향한 면(15° 안)의 높이 분포에서 넓은 층 중 가장 낮은 층 ± 5 cm 의 면으로 넓이 가중 최소제곱 평면을 맞춤. 2 cm 넘게 벗어난 면은 빼고 되풀이. 무작위 표본 없음 (C `736eb68`) | 그 바닥 평면 (`pipeline/glb_to_usd.py:126`) | 수평을 맞춘 메쉬 전체 정점의 x · y 범위 가운데 (자동 설정 `orient: origin: center`, `detect.py:100`; 이동은 `pipeline/glb_to_usd.py:489-493`) |
| 맵 폴더 (리얼센스) | 바닥 법선 → +Z (`pipeline/build_usd.py:70-85`) | **카메라 경로 아래 바닥 높이의 중앙값** (`pipeline/build_usd.py:251-261`, `transform.json` 의 `floor_zero`) | 입력 좌표 그대로 (틀 `configs/templates/indoor_rtabmap.yaml:33` 의 `orient: origin: keep`) |

- **Stray 는 11/24 (C `0534d95`) 부터 GLB 와 같은 길**로 간다 (`stages.py:46-71`).
  - `stray.py` 로 맵 폴더 형식(`out/input_map/`)을 만들고, B 의 `stray_mesh.build` (깊이 5 m · 자세 보정, 6-4)로 깊이를 합쳐 형상 메쉬 `build/mesh_raw.ply` 를 만든다.
  - 그 메쉬를 GLB 처럼 `pipeline/glb_to_usd.py` 에 넣는다. 그래서 원점 · 바닥의 정의가 GLB 와 같다.
  - 11/23 까지는 맵 폴더 길(`build_usd.py`)이라 "x · y = 입력 그대로, z = 경로 아래 중앙값" 이었다.
  - 형상 메쉬는 11/24 까지 `pipeline/tsdf.py` (깊이 3 m · 자세 보정 없음)로 만들었다. 11/25 R10 (C `be8310e`) 부터 `stray_mesh.build` 다. 메쉬 파일 이름 · 좌표는 같다.
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
| **Stray (아이폰)** | **`out/input_map/` 의 세계 좌표** = Stray 세계를 `Y_UP_TO_Z_UP` 으로 돌린 것 (+Z 위). 형상 메쉬 `build/mesh_raw.ply` 가 이 좌표다. **Stray 원본 세계(+Y 위)가 아님** | `M(원점 · 수평 · 바닥) · 배율`. 입력 메쉬가 이미 Z 위라 `Y_UP_TO_Z_UP` 을 곱하지 않음 (설정 `input: up: z`, `stages.py:71`, `detect.py:99`) | `pipeline/glb_to_usd.py:630-632` (메쉬를 만든 곳은 `stray.Y_UP_TO_Z_UP` · `stray.convert` 안의 `pose` + `stray_mesh.build`) |

- 11/24 에 Stray 의 길이 바뀌었지만 **Stray 의 "입력 좌표" 는 그대로**다 (`stages.py:45` 주석). 바뀐 것은 T 의 내용(x · y 원점이 범위 가운데로, z = 0 이 맞춘 바닥 평면으로)뿐이다.

**Stray 의 입력 좌표 (자세히)**

- Stray `odometry.csv` 의 자세 = 카메라 광학 좌표계(x 오른쪽 · y 아래 · z 앞) → Stray 세계. Stray 세계는 **+Y 가 위** (`stray.py` 머리말 "좌표").
- `stray.py` 는 카메라 쪽은 그대로 두고 세계만 돌린다: `T_맵폴더 = Y_UP_TO_Z_UP · T_stray` (`stray.convert` 안의 `pose`).
  - `Y_UP_TO_Z_UP = [[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]]` → `p_입력 = (x, −z, y)_stray` (`stray.Y_UP_TO_Z_UP`)
  - 거꾸로: `p_stray = (x, z, −y)_입력`
- 이 돌린 세계로 `input_map/trajectory_camera_optical.txt` 를 쓴다 (`stray.convert` 끝).
  `stray_mesh.build` 가 이 자세로 깊이를 합쳐 `build/mesh_raw.ply` 를 같은 좌표로 만들고, 빌드 설정(`build/auto_config.yaml`)의 `input.mesh` 가 이 파일이다.
  - `stray_mesh.build` 는 합치기 전에 키프레임 자세를 고친다 (6-4). 메쉬 좌표계(입력 좌표)는 같고, `trajectory_camera_optical.txt` 는 **보정 전 자세 그대로**다. 고친 자세는 `build/pose_fix.json` 으로 다시 만든다 (`stray_fix.apply_fix`).
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

`<jobs>` 기본값 `~/scan2sim_jobs`, 서버의 `--jobs` 로 바꿈 (`web/server.py:57`, `:551`). 저장소 밖에 둔다. 배포 스크립트는 `S2S_JOBS` 로 정한다 (기본 같음, `deploy/common.sh:8`).

```
<jobs>/<id>/                  id = 16진수 소문자 12자리 (uuid4 앞 12자리, web/server.py:309 · 예시 파일 작업은 :278)
  job.json                    서버가 쓰는 작업 기록 (2-2)
  input/input.zip             올린 파일. 이름은 항상 "input" + 확장자 (web/server.py:311)
  input/input.glb               — 원래 이름은 job.json 의 name 에만 남음
                                예시 파일 작업은 예시 파일을 하드 링크로 둠. 안 되면 복사 (web/server.py:280-284)
  run.txt                     엔진의 표준 출력 + 표준 오류 (web/server.py:392-394). 제한 시간에 걸리면 끝에 "[서버] 처리 제한 시간 N분을 넘어 엔진을 멈춤" 줄 (:400)
  out/                        엔진 출력 폴더 (2-3)
```

- 올린 파일 이름을 바꾸는 이유: 경로에 한글이 있으면 Open3D 가 못 읽음 (`web/server.py:311` 주석).
- 그 결과 **웹으로 올린 작업은 장면 이름이 모두 `input`** 이 된다 (2-4). 예시 파일 작업도 같다.
- **작업 폴더 정리** (D `533bffe`, `web/server.py:109-129` `tidy`). 작업이 하나 끝날 때마다 서버가 부른다 (`:417-418`). `status` 가 `done` · `failed` 이고 `finished` 가 있는 작업만 본다. 올리다 만 작업은 건드리지 않는다.
  - 끝난 작업 모두: `out/input_map/_unzip/` (올린 zip 을 풀어 둔 사본)을 지운다.
  - 끝난 시각이 최근 `--keep` 건(기본 5, `:57`, `:553`)보다 오래된 작업: `input/` · `out/input_map/` 도 지운다. `out/build/` 는 `out/preview.glb` 가 있거나 `out/build/usd/` 가 없을 때만 지운다 (화면용 파일을 만들 USD 는 남김).
  - 남는 것: `job.json` · `run.txt` · `out/stages/` · `out/bundle/` · `bundle.zip` · `preview.glb` · `report.md` · `report.json` · 결함 목록 · `log.txt` 등. 그래서 상태 응답 · `preview.glb` · `map.png` · `report.md` · `bundle.zip` 은 그대로 나온다 (아래 청소 전까지) [D 측정: 오래된 작업 1.1 GB → 86 MB 뒤에도 네 가지가 나옴, `533bffe` 커밋 설명].
  - `--keep` 이 0 보다 작으면(예: `-1`) 오래된 작업 지우기만 하지 않는다. **`_unzip/` 은 `--keep -1` 이어도 지운다** (`:123-124`).
- **오래된 작업 통째로 지우기 — 청소** (D `9b2c95f`, `web/server.py:132-164` `purge`). `tidy` 와 따로 돈다.
  - 부르는 때: 서버를 켤 때 (`:569`, 되살리기 `recover()` 뒤) · 작업이 하나 끝날 때 (`:419`) · 10분마다 (`sweeper`, `:424-432`).
  - `done` · `failed`: `finished` (없으면 `job.json` 고친 시각)부터 `--ttl-hours` 시간(기본 24, `:57`, `:558`)이 지나면 작업 폴더를 통째로 지운다.
  - `uploading`: 올린 파일을 마지막으로 고친 시각 (파일이 없으면 `created`)부터 같은 시간이 지나면 지운다.
  - `queued` · `running` 작업, `job.json` 이 없는 폴더, 이름이 작업 번호 꼴이 아닌 폴더는 건드리지 않는다. 지운 작업은 메모리에서도 빠져 상태 · 결과 파일 요청이 모두 404 가 된다. 화면의 결과 주소(`#<작업 번호>`)도 더 열리지 않는다.
  - `--ttl-hours` 가 0 이하면 하지 않는다 (`:137-138`). **`--keep -1` 은 청소를 막지 않는다** — 모두 남기려면 `--ttl-hours 0` 도 준다. 11/26 부터 `--keep` 도움말(`:559`)도 그렇게 적는다: "-1 이면 올린 파일 · 중간 파일을 지우지 않음. zip 을 풀어 둔 사본은 늘 지우고, 폴더를 통째로 지우는 것은 --ttl-hours 가 따로 함" (D `6ad7114`, 8-3 의 14번). 배포 스크립트는 `S2S_KEEP` 을 `--keep` 으로 넘긴다 (`deploy/start.sh`).
  - 미리 만들어 둔 작업 폴더를 이 서버로 열 때 조심할 것. 시연용 결과(`docs/demo_day.md` 는 당일 아침에 만들고, 전날 만들려면 `S2S_TTL_HOURS=72` 로 켜라고 적음)나 11/25 B 가 A 에게 준 작업 묶음(끝난 시각 11/25 16시쯤)을 기본값으로 열면, 24시간이 지난 뒤에는 서버가 켜질 때 지운다.
- 엔진을 직접 돌릴 때(CLI)는 아무것도 지우지 않는다.

### 2-2. `job.json`

서버가 메모리에 든 작업 dict 를 그대로 쓴 것 (`web/server.py:167-168` `save`). 4-3 의 `GET /api/jobs/<id>` 응답의 앞부분과 같다.

| 필드 | 형 | 언제 생기나 | 뜻 |
|---|---|---|---|
| `id` | 문자열 | 만들 때 | 작업 번호 |
| `name` | 문자열 | 만들 때 | 올린 파일의 원래 이름 (경로 빼고, 앞 120자). 예시 파일 작업은 예시 파일 이름 |
| `size` | 정수 | 만들 때 | 알린 파일 크기 (바이트). 예시 파일 작업은 그 파일 크기 |
| `received` | 정수 | 만들 때 (0) | 지금까지 받은 바이트. **조각을 받을 때마다 메모리에서만 늘고 `job.json` 에는 `start` 때 저장됨**. 서버를 다시 켠 뒤 `uploading` 작업을 읽을 때는 올린 파일의 크기(최대 `size`)로 되찾음 (`web/server.py:78-80`). 예시 파일 작업은 처음부터 `size` |
| `status` | 문자열 | 만들 때 | `uploading` · `queued` · `running` · `done` · `failed` (4-3) |
| `created` | 실수 | 만들 때 | 유닉스 시각(초) |
| `file` | 문자열 | 만들 때 | `input.zip` 또는 `input.glb` |
| `sample` | 문자열 | 예시 파일 작업만 | 예시 파일 이름 (D `9b2c95f`, `web/server.py:285`) |
| `queued_at` | 실수 | `start` 때 | 줄에 넣은 시각. 입력 검사(4-2)에 걸린 작업에는 없음 |
| `started` | 실수 | 처리 시작 | 입력 검사에 걸린 작업에는 없음 |
| `exit_code` | 정수 또는 `null` | 엔진이 끝남 | 엔진 종료 코드 (3-1). 엔진이 끝났으면 `failed` 여도 있음. **처리 제한 시간에 걸려 멈추면 `null`** (`:401-408`). 엔진을 못 띄웠거나 입력 검사에 걸린 경우에는 없음 |
| `finished` | 실수 | 처리 끝 | 서버가 `bundle.zip` 을 미리 만든 **뒤에** 씀 (`:411-415`, 4-3). 입력 검사에 걸릴 때 · 두 번째로 끊겨 `failed` 가 될 때도 씀 (4-3) |
| `timed_out` | 참 | 처리 제한 시간에 걸렸을 때만 | `true` (D `9b2c95f`, `:402`) |
| `restarts` | 정수 | 서버를 다시 켜서 줄에 다시 넣었을 때 | 다시 넣은 횟수. 지금은 1 까지 (D `533bffe`, `web/server.py:97-101`) |
| `error` | 문자열 | 서버 쪽 실패일 때만 | 서버 쪽 예외 (`"TypeError: …"`) · 입력 검사 문장 (4-2) · 처리 제한 시간 문장 `"처리가 20분을 넘어 멈춤 — 촬영본이 너무 길거나 넓을 수 있음. 더 짧게 나눠 찍어 다시 올릴 것"` (`:403`, 분은 `--timeout-min`) · `"서버가 다시 켜져서 끊김 (두 번째)"` · `"서버가 다시 켜져서 끊김"` (4-3). 엔진 단계가 실패한 `failed` 에는 없음. **지금 화면은 이 필드를 보여 주지 않는다** — 11/26 부터 이런 작업의 `verdict` 는 `"fail"` 이라 판정 칸에 "pending" 은 안 나오지만, 이유는 여전히 안 보인다 (8-3 의 2번, A) |

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
    _unzip/                     zip 을 푼 원본 촬영 (rgb.mp4 · depth/ · odometry.csv …). 사람 얼굴이 든 영상이 그대로 남음 — 서버는 작업이 끝나면 지움 (2-1)
    keyframes/rgb/<n>.jpg · keyframes/depth/<n>.png · keyframes/calib/<n>.yaml
    trajectory_camera_optical.txt   키프레임 자세 — 자세 보정 전 (6-4)
    stray_meta.json             변환이 끝나면 맨 끝에 씀 (6-3)
    stray_quality.json          Stray 입력이면 진단 단계가 씀 (`stray_quality.measure`, 통합 기록 R3 — C `620a2b0`). 진단 수치의 `stray_quality` 와 같은 값
    pose_fix.json               build/pose_fix.json 과 같은 바이트의 복사본. 없어도 됨 (6-4)
    (예전 맵만) floor_fix.json   af65f1b 판 B 메쉬가 쓴 높이 보정 info. 지금 코드는 쓰지 않고 새로 만들 때 지움 (6-4)
  build/                      빌드 결과
    auto_config.yaml            엔진이 만든 빌드 설정
    auto_config_corrected.yaml  보정을 했을 때만 (stages.py:458, :480)
    floor_patch_mask.npz        보정이 보이는 바닥 구멍을 덮을 때만 — 덮을 Nav2 지도 칸 (5-4, stages.py:472-475)
    floor_patch.ply             그 판의 메쉬 (= USD prim FloorPatch). 빌드가 판을 만들었을 때만. 빌드마다 먼저 지움 (pipeline/glb_to_usd.py:642-650)
    transform.json              1-3
    visual.ply                  보이는 형상 (장면 좌표)
    report.json                 빌드 보고 (엔진의 report.json 과 다른 파일)
    usd/<이름>_mesh.usdc · usd/<이름>_scene.usda
    usd/textures/               사진 텍스처가 있을 때만 (GLB). Stray 는 없음
    ground.npz                  바닥 높이 지도. GLB · Stray 는 바닥이 평평하지 않을 때만 (1-2). 있으면 진단 · 시작 자세가 씀
    (GLB · Stray) drive_path.json   가상 주행 경로 → bundle/route.csv 의 재료
    (GLB · Stray) glb_tex/          GLB 텍스처를 풀어 두는 곳. Stray 는 빈 폴더
    (Stray 만) mesh_raw.ply · tsdf_meta.json · pose_fix.json   형상 메쉬 · 그 정보 · 자세 보정 info (stages.s_input → stray_mesh.build, 6-4)
    (맵 폴더만) mesh_raw.ply · collider.ply · floor.ply · boundary.ply · check_input.json · tsdf_meta.json …
  defects_before.json         보정 전 결함 목록 (5장). 진단 단계가 돌면 항상
  defects_after.json          보정 뒤 다시 진단한 결함 목록. **보정이 돌았을 때만** (5-4)
  bundle/                     Isaac Sim 용 묶음 (2-4)
  preview.glb                 화면용 GLB. 엔진이 묶음 단계 끝에 씀 (stages.py:420-425 → preview.py). 없으면 서버가 요청 때 만듦 (4-2)
  report.md                   사람이 읽는 보고서 (s_report, stages.py:478)
  report.json                 {name, verdict, stages} (stages.py:479, 3-4)
  bundle.zip                  서버가 작업이 끝날 때 만듦. 그 전에 요청이 오면 그때 만듦 (4-2). 맨 위가 bundle/ 하나 (11/25 D bef9cda 부터)
```

- Stray 의 `build/` 는 11/24 에 바뀌었다 (C `0534d95`). `floor.ply` · `collider.ply` · `boundary.ply` · `check_input.json` 이 없어지고 `drive_path.json` · `glb_tex/` · `tsdf_config.yaml` 이 생겼다. `mesh_raw.ply` · `tsdf_meta.json` · `visual.ply` · `transform.json` 은 그대로 있다.
  - 11/25 (R10, C `be8310e`) 부터 `tsdf_config.yaml` 은 생기지 않고 (`detect.config_tsdf` 지움) `pose_fix.json` 이 생긴다.
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
- **씬 루트가 둘이다** (C `620a2b0`, `stages.s_bundle`, `stages.py:569-600`).
  - `scene_root.usda`: **묶음만으로 열린다.** subLayer 는 `./usd/<이름>_scene.usda` 하나. `/World/Lights/Dome`(DomeLight, 세기 600)을 직접 정의한다. 로봇 · 센서는 없다. [C 측정] usd-core 로 열면 못 찾는 층 0. [D 확인] Isaac Sim 6.0.1 (Ubuntu 서버, main `533bffe`)에서 조명까지 오류 없이 열린다. 로봇을 놓고 주행해 본 것은 아직 없다 (`C_TRACK.md` C8).
  - `scene_root_sim.usda`: 11/24 전의 `scene_root.usda` 그대로 — `../proxy/*.usda` 와 `../../../data/usd/<이름>/usd/<이름>_scene.usda` 를 subLayer 로 건다. `sim/scenes/<장소>/` 에 두는 것을 가정하고, 혼자서는 안 열린다. 웹 작업은 `<이름>` 이 모두 `input` 이다.
  - 맵 파일(`usd/<이름>_scene.usda`)에는 조명이 없다. 맵만 열면 검게 보인다.
- 그림자 설정: 진단의 `shadow_fix` 가 참이면(3-5) 보이는 prim 마다 `primvars:doNotCastShadows = 1` 을 건다 (`stages.py:524-525`). `room` 은 `Visual` · `FloorFill` 2개. 보정이 구멍을 덮었으면 `FloorPatch` 도 들어간다 (5-4).
- **`bundle.zip` 은 11/25 (D `bef9cda`) 부터 맨 위가 `bundle/` 폴더 하나다.** `report.md` · `defects_*.json` 도 그 안에 든다 (4-2). `bundle/` 폴더 자체(엔진이 쓰는 `out/bundle/`)는 그대로다.
  - D 의 `sim/add_site.py` 는 zip 을 풀어 맨 위가 폴더 하나면 그 안을 맵 패키지로 본다 (`sim/add_site.py:400-402`). 그래서 내려받은 zip 을 풀지 않고 그대로 준다: `python3 sim/add_site.py <bundle.zip> --site <장소> --ver <판>`. 예전 구조(맨 위에 `bundle/` 과 `report.md` 가 나란히)는 `usd/` 를 못 찾고 멈춘다 [코드로만 봄].
  - [D 측정, `docs/isaac_check.md` 2장] 새 구조 묶음 두 건을 들임: 가상의 방(GLB) `SITE_OK`. 실제 촬영본(Stray)은 들여지지만 맵 판정이 "실패 예상 — 주행 공간 1.4 m²" 라 `SITE_FAIL`. 이 실행에서 출발 x 가 음수면 맵 판정이 멈추던 `add_site.py` 의 버그를 고침 (D `1e70ddb`, `sim/add_site.py:543`).
  - 씬 루트 (C12, D `6e88feb`): `add_site.py` 는 패키지에 `scene_suggestion/<장소>_<판>.usda` 가 없으면 **묶음의 `scene_root_sim.usda`** (또는 `--scene-root` 로 준 파일)를 받아 맵 경로 줄만 `@../../../data/usd/<장소>_<판>/usd/<파일>@` 로 바꿔 쓴다 — 그림자 설정이 그대로 간다 (`sim/add_site.py:503-518`). 맵 경로 줄은 정규식 `@\.\./\.\./\.\./data/usd/[^/@]+/usd/[^/@]+@` 로 **딱 하나** 찾아야 하고, 0개나 2개 이상이면 묶음의 것을 버리고 새로 쓴다(그림자 설정 빠짐). 그래서 묶음 단계의 그 줄 꼴은 약속이다 (`stages.py` `s_bundle` 의 주석, C `9706eee`). 묶음은 지금 꼴 그대로다.
  - 아직 안 맞는 것 (8-2): 이름은 모두 `input`, `start_pose.json` · `route.csv` 도 쓰지 않는다.

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
- `--force`: 실패한 단계가 있어도 다음 단계를 함. `--resume`: `build/report.json` 이 있으면 변환(`convert`)을 건너뜀 (`__main__.py:22-23`, `stages.py:128-129`). 보정 단계는 `--resume` 이어도 늘 다시 만든다 (`stages.py:481`). **서버는 둘 다 쓰지 않는다** (`web/server.py:393`).
  - Stray 는 `--resume` 이면 `input_map/stray_meta.json` 이 있을 때 변환(`stray.convert`)을 건너뛴다. 입력이 바뀌었는지는 보지 않는다 (`stages.py:53`).
  - 형상 메쉬는 이번 실행에서 맵 폴더를 새로 만들었거나, `build/mesh_raw.ply` 가 없거나, `stray_meta.json` 보다 오래됐으면 다시 만든다. 그 밖에는 건너뛴다 (`stages.py:62-63`, C `be8310e`).
  - 메쉬를 다시 만들었으면 `--resume` 이어도 `convert` 를 건너뛰지 않는다 (`ctx["mesh_rebuilt"]`, `stages.py:73`, `:131`, C `13d1c4d` — 통합 기록 R15, 8-3 의 11번). 11/25 까지는 `build/report.json` 만 보고 건너뛸 수 있었다.
- 종료 코드 (`__main__.py:29`)
  - `0` = 어느 단계도 `fail` 이 아님 (통과 또는 주의). **이때도 `verdict` 는 `fail` 일 수 있다** (3-4).
  - `1` = 한 단계라도 `fail`
  - 단계 안의 `Exception` 은 그 단계의 `fail` 로 바뀐다 (`core.py:57-58`). 요약은 `"예외: <예외 이름>: <문장>"`, `messages` 는 traceback 마지막 6줄.
    - 단, `stray.py` 의 `CaptureError` 는 `input` 단계가 직접 받아 **그 문장만** 요약에 넣고 `messages` 는 비운다 (R4, C `620a2b0`; 6-1).
    - 단계 안의 `SystemExit` 도 그 단계의 `fail` 이 된다 (C `6ec3237`, `core.py:53-56`). 요약 = 그 문장 (문장이 없으면 `"멈춤 (종료 코드 N)"`), `messages` 는 비어 있다. 뒤 단계는 `skip`, `report.md` · `report.json` 은 남는다. 예: 바닥을 못 찾는 GLB 의 `"바닥(위를 향한 넓은 면)을 못 찾음 …"`.
  - 인자가 틀리면 argparse 의 `2` [추정: argparse 기본 동작].

### 3-2. 단계 파일 `out/stages/NN_<단계>.json`

`core.run` 이 단계마다 하나씩 쓴다 (`core.py:41-68`). 파일 이름 = `f"{k:02d}_{name}.json"` (`core.py:60`).

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
| `messages` | 문자열 배열 | 사용자가 읽을 문장. 예외면 traceback 마지막 6줄 (`core.py:58`) |
| `seconds` | 실수 | 걸린 시간 (소수 1자리) |

- `summary` 와 `messages` 는 화면에 그대로 나간다. 한국어, 사용자가 읽을 문장으로 쓴다.
- `status` 의 뜻 (`core.py:1-6`)
  - `pass` 통과
  - `warn` 쓸 수 있지만 볼 것이 있음. `Result.warn()` 은 `pass` 일 때만 `warn` 으로 바꿈
  - `fail` 여기서 멈춤 (이유는 `messages`)
  - `skip` 이번 실행에서 안 함 (이유는 `summary`)
- 한 단계가 `fail` 이면 그 뒤 단계는 `report` 를 빼고 모두 `skip` 이 되고, `summary` 는 `"앞 단계(<단계>)가 실패해서 하지 않음"` (`core.py:48-49`).

### 3-3. 단계 목록

순서 · 이름은 `stages.py:683`, 화면 이름(`label`)은 `web/server.py:49-50`.

| NN | `stage` | `label` | 하는 일 | 나올 수 있는 `status` |
|---|---|---|---|---|
| 00 | `input` | 입력 확인 | 입력 종류 판별, 형태 재기, 빌드 설정 만들기. Stray 면 맵 폴더로 바꾸고 형상 메쉬(`stray_mesh.build`)까지 만듦 | pass · warn · fail |
| 01 | `convert` | 메쉬 · USD 만들기 | GLB · Stray: `pipeline/glb_to_usd.py`. 맵 폴더: `check_input.py` + `run.py --from tsdf` | pass · fail |
| 02 | `nav2` | Nav2 지도 | `sim/nav2/make_map.py`. 이 파일이 없으면 fail `"Nav2 지도를 만드는 도구가 없음"` (R12, C `be8310e`, `stages.py:153-156`). 빈 바닥 4 m² 미만이면 fail | pass · fail |
| 03 | `diagnose` | 진단 | 충돌 바닥 층수 · 보이는 바닥 · 보이는 바닥 두 높이 · 낮은 턱 · 물체 일치 · 다닐 수 있는 모양(`drivable` — 제자리 회전 가능 넓이 포함) · 천장 · (Stray) 복원 정확도 · 되돌아온 자리 어긋남(`revisit`, 6-5). `defects_before.json` 씀. 지도가 없으면(`--force`) fail `"Nav2 지도가 없어 진단할 수 없음"` (`stages.py:256-257`) | pass · warn · fail |
| 04 | `correct` | 보정 · 다시 진단 | 두 겹 바닥, 또는 (GLB · Stray) 보이는 바닥 없음이 있을 때만 다시 빌드 · 다시 진단 (5-4). `defects_after.json` 씀 | pass · warn · fail · skip |
| 05 | `bundle` | 묶음 만들기 | `bundle/` 만들기, 끝에 `preview.glb` 쓰기 | pass · warn |
| 06 | `verify` | 주행 검증 | 지금은 항상 skip (Isaac Sim 자리, `stages.py:616-618`) | skip |
| 07 | `report` | 보고서 | `report.md` · `report.json`. 앞 단계가 실패해도 항상 돎 | pass · warn (**fail 이 되지 않음**, `stages.py:680`) |

- 11/24 까지 nav2 는 도구가 없으면 `skip` 이었고, 그러면 진단이 `KeyError` 로 죽었다. 지도가 없으면 진단 · 보정 · 묶음이 모두 못 돌아서 `fail` 로 바꿨다 (R12).

### 3-4. 엔진의 `report.json`

`stages.py:679`: `{"name": <장면 이름>, "verdict": "pass|warn|fail", "stages": [단계 dict …]}`

- `verdict` = `input` ~ `verify` 단계 중 가장 나쁜 것 (`fail` > `warn` > `pass`, `skip` 은 무시) (`stages.py:624`).
- **단, 실패한 단계가 없어도 `verdict` 가 `fail` 일 수 있다** (C `105591c`, `stages.py:625-628`).
  - 진단이 "이 맵으로는 주행할 수 없음" 이유를 남기면(`ctx["unusable"]`, 걸어간 길이 막힘 · 끊김, 5-4) `verdict` 를 `fail` 로 바꾼다. 진단 단계는 그 이유를 주의로 보여 `warn` 이다.
  - 단계는 끝까지 돌아 묶음 · `preview.glb` 가 나온다. 엔진 종료 코드 0 · 서버 상태 `done` · `verdict` `fail` 이 된다 (4-3).
  - `report.md` 의 판정은 `"쓸 수 없음 — <첫 이유>"`, `report` 단계 요약은 `"report.md — 쓸 수 없음 — <첫 이유>"`.
  - 그래서 화면 · 시험 · 다른 도구는 판정을 단계 상태에서 짐작하지 말고 `verdict` 를 본다.
- `stages` 에는 `report` 단계 자신이 **없다** (쓰는 시점에 아직 결과가 없음).
- `report` 단계의 `status` 는 `verdict` 가 `fail` 이면 `pass` 로 바뀐다. 그래서 전체 판정은 `report` 단계가 아니라 `verdict` 를 본다.

### 3-5. 단계별 `metrics` 의 키 (지금 코드)

약속은 "있으면 이 이름" 이다. 화면이 쓰는 것은 `diagnose` 의 `samples` · `floor_single_pct` · `floor_double` · `visible_floor_missing_pct` 뿐 (`index.html:258-261` `renderCompare`).

| 단계 | 키 |
|---|---|
| `input` (GLB · Stray) | `parts` · `textured` · `triangles` · `extent_m` · `floor_tilt_deg` · `floor_area_m2` · `floor_spread_m` · `floor_main_share_pct` · `floor_flat` · `ceiling_area_m2` · `ceiling_over_floor_pct` · `has_ceiling` · `wall_top_m` (`detect.py:74-77`). Stray 면 여기에 `stray` (= `stray_meta.json` 전체, `stages.py:89`) · `pose_fix` (아래). Stray 는 형상 메쉬를 잰 값이다. 형상 메쉬를 못 만들면 `stray` 만 있음 (`stages.py:70`) |
| `input` (맵 폴더) | `main_keyframes` · `main_poses` · `extra_sets` · `geometry_mesh` · `has_validation` (`detect.py:91-92`). 진단이 같은 dict 에 `ceiling_over_floor_pct` · `has_ceiling` 을 더함 (`stages.py:367-370`) |
| `convert` | `mesh_mb` · `prims` ({prim 이름: 삼각형 수}) · `collision` · `clean`. 실패 시 `check_input` |
| `nav2` | `cells` [가로, 세로] · `resolution` · `free_m2` · `occupied_cells` |
| `diagnose` | `samples` · `floor_single_pct` · `floor_double` · `floor_double_pct` · `floor_under` · (`visual.ply` 가 있으면) `floor_two_levels` {samples, pct, gap_cm_median} · `visible_floor_missing_pct` · `visible_minus_collision_cm` {p5, median, p95} · (판에 맞은 표본이 있으면) `floor_patch_samples` · `low_bumps` {samples_over_3cm_pct, runs_over_0p6m, longest} · `obstacle_match` {…} · `drivable` (아래) · `has_ceiling` · `shadow_fix` · (Stray) `stray_quality` {frames, median_cm, p90_cm, within_2cm_pct, hit_pct, verdict} · (Stray, 재었으면) `revisit` (아래) · `defects` ({type: 개수}) |
| `correct` (돌았을 때) | `fixed` (예: `["double_floor"]`. `no_visible_floor` 는 판을 깐 칸이 있을 때만) · `before` · `after` (각각 `diagnose` 의 metrics 와 같은 모양) · `floor_patch` (아래, 보정이 `no_visible_floor` 를 다루지 않았으면 `null`). 덮을 자리가 없어 `skip` 이면 `floor_patch` 만 |
| `bundle` | `start_pose` (= `start_pose.json`) · `files` (개수) · `shadow_overrides` · `preview` ({prims, triangles, triangles_in, mb, seconds, cell_m}, 못 만들면 {error}) |

- `input` 의 `pose_fix` (Stray, R11 · C `be8310e`, `stages.py:104`) = `build/tsdf_meta.json` 의 `pose_fix` (6-4) 에서 `method` · `applied` · `status` · `reasons` · `flags` · `auto` 만, 있는 키만 옮긴 것. 그 정보가 없으면 `{}`. 이 값으로 내는 문장은 6-3.
  - `auto` (C `13d1c4d`, 통합 기록 R20) = 높이 보정을 자동으로 고른 근거 `{chosen, threshold_cm, measured, source, every_s, verdict, pairs, reliable, z_max_cm, h_max_cm, reason, seconds (, unused_opts)}` (6-4 "자동 고르기"). 재지 못해 `"xy"` 로 갔으면 `measured` 가 거짓이고 까닭이 `reason`. `fix "auto"` 로 만든 메쉬에만 있다.
- `diagnose` 의 `drivable` (C `105591c` · `13d1c4d`, `checks.drivable`, `checks.py:48-83`) = `{clearance_m, free_m2, pieces, main_share_pct, turn_radius_m, turn_area_m2, path}`.
  - `clearance_m` = 0.3 (`ROBOT_RADIUS_M`). `free_m2` = Nav2 지도 빈 칸 넓이.
  - 장애물 · 미지 칸과 0.3 m 이상 떨어진 빈 칸을 8방향으로 잇는다. `pieces` = 그 덩어리 중 1 m² 이상인 것의 수. `main_share_pct` = 덩어리 전체 넓이 중 가장 큰 덩어리의 비율(%).
  - `turn_radius_m` = 0.656 (로봇 대각선 `hypot(0.607, 0.25)`). `turn_area_m2` = **제자리 회전 가능 넓이** — 빈 칸이 아닌 칸(장애물 · 미지)까지 `turn_radius_m` 이상 떨어진 빈 칸의 넓이 (`checks.py:62-64`, C `13d1c4d`). `sim/add_site.py` 의 맵 판정 `area_clear_ge_footprint_m2` 와 같은 계산이다 [C 측정: 방 촬영본 `6bc1ae021f` 1.35 m² 로 D 의 값과 같음]. 5 m² 미만이면 진단 주의 (5-4 아래).
  - `path` = `{length_m, blocked_m, blocked_pct, main_piece_pct}` — Stray 입력만 (촬영 경로가 있을 때). 뜻은 5-4 `path_blocked`. `main_piece_pct` = 빈 칸을 0.15 m 넓혀 이은 덩어리 중, 걸어간 길의 점이 가장 많이 든 덩어리에 든 점의 비율 (지도 밖 점은 어느 덩어리에도 안 듦).
- `diagnose` 의 `revisit` (Stray, C `13d1c4d`, 통합 기록 R19, `stages.py:397-411`) = `stray_revisit.measure(out, capture=<입력>)` (6-5) 결과에서 `{verdict, raw{…}, fixed{…}}`. `raw` · `fixed` 는 각 `summary` 의 `pairs` · `reliable` · `h_max_cm` · `z_max_cm` · `yaw_max_deg` · `h_worst` · `z_worst` 만 (`fixed` 는 맵을 만든 보정을 읽었을 때만). 재다 오류가 나면 키가 없고 진단 메시지 `"되돌아온 자리 어긋남을 재지 못함: …"`.
  - 한 작업에서 한 번만 잰다 (`ctx["stray_revisit"]` — 보정 뒤 다시 진단할 때 그대로 씀). `input_map/stray_revisit.json` 도 쓴다 (6-5).
  - 원래 자세(`raw`)는 입력 단계의 자동 고르기(6-4)도 한 번 재므로 Stray 작업은 두 번 잰다 (자동 고르기는 프레임 간격 max(0.1 초, 촬영 초 / 3000), 진단은 0.1 초) [코드로 봄].
- `correct` 의 `floor_patch` (R9) = `{cells, area_m2, regions: [{center, area_m2, low_pct, ring_floor_pct, reason, patched}]}` — 판을 깐 Nav2 지도 칸 수 · 넓이와 보정 전 `no_visible_floor` 에 걸친 구멍마다 덮었는지와 덮지 않은 이유 (`stages.py:438-447`, `:471`). 넓은 것부터.
- **단계 파일과 엔진 `report.json` 의 `input` metrics 가 다르다.** 진단 단계가 입력 단계의 dict 에 나중에 키를 더하는데, 입력 단계 파일은 그 전에 쓰였기 때문이다.
  - 모든 입력: `shadow_fix` (`stages.py:374`). 맵 폴더는 `ceiling_over_floor_pct` · `has_ceiling` 도 (`stages.py:367-370`).
  - 이 키들은 단계 파일 · 상태 응답의 `metrics.input` 에는 없고 엔진 `report.json` 에만 있다 (`room` 에서 확인).
- **천장 기준이 둘이다.**
  - `has_ceiling` = 빈 바닥 위 천장 덮음 > 30% (`detect.py:76`, 맵 폴더는 `stages.py:370`).
  - `shadow_fix` = `has_ceiling` 이거나 덮음 ≥ 10% (`stages.py:374`, C `ed35a87`). 참이면 씬 루트에 그림자 설정을 넣는다 (2-4).
  - `room` 은 덮음 26.8% 라 `has_ceiling` 거짓 · `shadow_fix` 참이다.
- `convert` 의 `prims` 이름
  - GLB: `Visual` (부분 하나) 또는 `Visual0` · `Visual1` … (부분이 여럿, `pipeline/glb_to_usd.py:530`) + `FloorFill` (`:606`) + `Collision` · `Boundary`.
  - Stray: 형상 메쉬가 한 덩어리라 `Visual` · `FloorFill` · `Collision` · `Boundary` (`room` 확인). 11/23 까지는 맵 폴더 길의 prim 이었다.
  - GLB · Stray 에서 보정이 보이는 바닥 구멍을 덮었으면 보이는 prim `FloorPatch` 가 하나 더 있다 (R9, `pipeline/glb_to_usd.py:646-650`). 충돌 prim 은 그대로다.
  - Stray 의 보이는 prim 은 **정점 색만** 있다 (사진 텍스처 없음).

`diagnose` 실제 예 (`room` 의 `stages/03_diagnose.json`. `be8310e` 전 판이라 `drivable` · `stray_quality` 가 없다):

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

서버: `python -m scan2sim.web.server [--port 8000] [--host 127.0.0.1] [--jobs ~/scan2sim_jobs] [--code <접속 코드>] [--keep 5] [--samples <예시 파일 폴더>] [--timeout-min 20] [--max-wait 5] [--min-free-gb 20] [--ttl-hours 24] [--dev]` (`web/server.py:547-562`). `--keep` 은 2-1 의 작업 폴더 정리 (D `533bffe`). 그 뒤의 것은 D `9b2c95f`.

| 선택 사항 | 기본 | 하는 일 (0 이면 그 검사를 끔) | 자세히 |
|---|---|---|---|
| `--samples <폴더>` | 없음 | 그 폴더의 `.zip` · `.glb` 를 예시 파일로 알려 주고 그 파일로 작업을 만듦 | 4-2 |
| `--timeout-min` | 20 | 엔진 처리 제한 시간(분). 넘으면 엔진을 죽이고 `failed` | 4-2 "거절 · 입력 검사 · 제한 시간" |
| `--max-wait` | 5 | 기다리는 작업(`queued`)이 이 수면 새 작업을 429 로 거절 | 같음 |
| `--min-free-gb` | 20 | 받고 나면 작업 폴더 디스크 여유가 이 값(GiB) 아래가 되는 새 작업을 507 로 거절 | 같음 |
| `--ttl-hours` | 24 | 끝난 지 이 시간이 지난 작업 폴더 · 그만큼 멈춘 올리다 만 작업을 통째로 지움 | 2-1 |
| `--dev` | 꺼짐 | `waitress` 가 있어도 Flask 내장 서버로 켬 | 아래 |

- **요청을 받는 서버:** `waitress` 가 설치돼 있으면 그것으로 켠다 (`threads=16` · `max_request_body_size` = 4 GiB · `channel_timeout=300`, `web/server.py:574-582`). 없거나 `--dev` 면 Flask 내장 서버 `threaded=True` (`:584`). `recon/requirements.txt` 에 `waitress==3.0.2`. 처리는 어느 쪽이든 `worker` 스레드 하나가 차례로 한다.

**배포 (D, `deploy/`)** — 팀 ThinkPad 에서 켜는 방법. 약속에 닿는 것만 적는다.

- `deploy/start.sh` 가 서버를 `--host 127.0.0.1 --port $S2S_PORT --jobs $S2S_JOBS --code <코드>` + 아래 선택 사항으로 켠다 (`deploy/start.sh:45`). 기본 포트 8000, 작업 폴더 `~/scan2sim_jobs` (`deploy/common.sh:8-11`).
  - 늘 넘김: `--min-free-gb $S2S_MIN_FREE_GB` (기본 20, `deploy/common.sh:14`). 켤 때 디스크 여유가 이보다 적으면 알려 준다 (`deploy/start.sh:36-38`).
  - 폴더가 있을 때만: `--samples $S2S_SAMPLES` (기본 `~/scan2sim_samples`, `deploy/common.sh:13`).
  - 환경 변수를 줄 때만: `--timeout-min` · `--max-wait` · `--ttl-hours` (`S2S_TIMEOUT_MIN` · `S2S_MAX_WAIT` · `S2S_TTL_HOURS`, `deploy/start.sh:30-35`). 안 주면 서버 기본값.
  - `--keep` 은 넘기지 않아 기본 5 다.
- **접속 코드를 늘 준다.**
  - 처음 켤 때 6자리 숫자를 만들어 `~/scan2sim_run/code` (권한 600)에 두고 계속 쓴다. `S2S_CODE=<코드>` 로 정할 수 있다 (`deploy/start.sh:19-21`).
  - 주소 · 코드는 `deploy/status.sh` 가 보여 준다. 11/25 (D `bef9cda`) 부터 코드가 든 링크 `<터널 주소>/?code=<코드>` 와 그 QR (터미널 + `~/scan2sim_run/qr.png`, `segno`, `S2S_QR=0` 이면 끔), `GET /api/health` 로 읽은 기다리는 작업 수 · 처리 중인지, 예시 파일 수, 디스크 여유와 기준도 보여 준다.
  - **화면은 아직 주소의 `?code=` 를 읽지 않는다** — `fetch` 머리말과 `withCode()` 모두 입력칸의 코드만 쓴다 (`index.html:149`, `:157`). 링크로 들어와도 첫 API 요청이 401 이 되어 코드 입력칸이 나온다 (A).
- 밖에서는 Cloudflare 임시 터널(`cloudflared tunnel --url http://127.0.0.1:<포트>`, `deploy/start.sh:64`)로 들어온다.
  - 주소 `https://<이름>.trycloudflare.com` 은 켤 때마다 바뀐다. `S2S_TUNNEL=0` 이면 터널 없이 이 컴퓨터에서만.
- 서버가 죽으면 2초 뒤 다시 켠다 (`deploy/start.sh:42-48`). 그때 기다리던 · 처리 중이던 작업은 줄에 다시 들어가 처음부터 돈다 (작업마다 한 번, 4-3).
- 환경: Python 3.12 가상 환경 `~/scan2sim_env` 에 `recon/requirements.txt` 를 설치 (`deploy/install.sh`). 불러오기 확인에 `waitress` · `segno` 가 들어갔다 (`bef9cda`).

### 4-1. 공통

- **접속 코드:** `--code` 를 주면 `/api/` 로 시작하는 모든 요청에 코드가 있어야 한다 (`web/server.py:241-244`). 새 경로 `/api/health` · `/api/samples…` 도 같다.
  - 머리말 `X-Code: <코드>` 또는 쿼리 `?code=<코드>`. 머리말이 있으면 머리말만 본다.
  - 없거나 틀리면 `401` `{"error": "접속 코드가 맞지 않음"}`.
  - `GET /` 와 `/static/…` 은 코드를 보지 않는다.
  - 화면은 `fetch` 에는 머리말을 (`index.html:149`), `<img>` · GLB 로더 · 내려받기 링크에는 `?code=` 를 붙인다 (`index.html:157`).
- **오류 응답:** 400 · 401 · 409 · 429 · 507 은 JSON `{"error": "문장", …}`. 화면은 `error` 문장을 그대로 `"실패: <문장>"` 으로 보인다 (`index.html:154`, `:205`).
  - **404 는 JSON 이 아니라 Flask 기본 HTML** 이다 (`abort(404)`). 클라이언트는 상태 코드만 보고 판단한다.
  - 작업 번호가 `^[0-9a-f]{12}$` 가 아니거나(`web/server.py:64-65`), 그 작업이 메모리에도 `job.json` 에도 없으면 404 (`:75-76`). 청소(2-1)로 지운 작업도 404.
  - **500 도 HTML** 이다. 알려진 경우는 없다 — 조각의 `offset` 이 숫자가 아닐 때의 500 은 11/26 에 400 으로 고침 (D `6ad7114`, 8-3 의 5번).
- **진행 상태는 화면이 `GET /api/jobs/<id>` 를 1.5초마다 불러서 본다** (`index.html:216`). SSE · WebSocket 없음.
- 응답 JSON 의 키 순서는 알파벳 순이다 (Flask 기본). 순서에 기대지 않는다.

### 4-2. 경로

| 메서드 · 경로 | 요청 | 성공 응답 | 실패 |
|---|---|---|---|
| `GET /` | — | `index.html` | — |
| `GET /api/info` | — | `{"ok": true, "stages": [{"name", "label"} ×8], "accept": [".zip", ".glb"], "max_bytes": 4294967296, "samples": <예시 파일 수>}` (`:252-254`) | 401 |
| `GET /api/health` | — | `{"ok": true, "queue_length": <기다리는 작업 수>, "running": 참거짓, "free_gb": <작업 폴더 디스크 여유, GiB, 소수 1자리>}` (`:257-261`, D `9b2c95f`). `queue_length` 는 `queued` 만 센다 — 처리 중인 것을 세는 `queue_ahead` (4-3)와 다르다 | 401 |
| `GET /api/samples` | — | **JSON 배열** `[{"name": 파일 이름, "label": 이름표, "size": 바이트}, …]` (`:264-266`, `samples` `:224-238`). 아래 "예시 파일" | 401 |
| `POST /api/samples/<name>/jobs` | 본문 없음. `<name>` = 목록의 `name` | `{"id": "<12자리>"}` (`:269-291`). 올리기 없이 입력 검사 → 줄에 넣음. 상태는 바로 `queued` | 404 (목록에 없는 이름, HTML) · 429 · 507 · 400 `{"error": <입력 검사 문장>, "id": …}` (작업은 `failed` 로 남음) · 401 |
| `POST /api/jobs` | JSON `{"name": 파일 이름, "size": 바이트}` (`Content-Type` 은 안 봄) | `{"id": "<12자리>"}` (`:294-315`) | 400 `{"error": "name · size 를 읽을 수 없음"}` (`size` 가 정수로 안 바뀜) · 400 `{"error": "받는 파일은 .zip · .glb 임"}` (확장자, 대소문자 무시) · 400 `{"error": "파일 크기가 0 이거나 4 GB 를 넘음"}` · **429 · 507** (아래) · 401 |
| `PUT /api/jobs/<id>/chunk?offset=N` | 본문 = 파일 조각의 날것 바이트. `N` = 지금까지 받은 바이트 수. 서버는 붙이기 전에 파일을 `received` 바이트로 자른다 (`:326`) | `{"received": 누적 바이트}` (`:318-336`) | 409 `{"error": "조각 순서가 맞지 않음", "received": n}` (상태가 `uploading` 이 아니거나 `N ≠ received`) · 400 `{"error": "알린 크기보다 많이 옴"}` · 400 `{"error": "offset 은 숫자여야 함", "received": n}` (`N` 이 숫자가 아님, `:321-324`, D `6ad7114` — 전에는 500) · 401 · 404 |
| `POST /api/jobs/<id>/start` | 본문 없음 | `{"status": "queued"}` (`:339-347`) | 409 `{"error": "파일이 다 오지 않음", "received": n}` (상태가 `uploading` 이 아니거나 `received ≠ size` — 이미 넣은 작업을 다시 넣어도 이 문장) · **400 `{"error": <입력 검사 문장>}`** (작업은 `failed` 로 남음, 아래) · 401 · 404 |
| `GET /api/jobs/<id>` | — | 작업 상태 (4-3) (`:450-473`) | 401 · 404 |
| `GET /api/jobs/<id>/preview.glb` | — | `model/gltf-binary`. 화면 좌표 (1-1). 엔진이 써 둔 `out/preview.glb` (`:476-493`) | 404 (파일이 없고 USD 도 없음, 또는 서버가 만들다 실패) · 401 |
| `GET /api/jobs/<id>/map.png` | — | `image/png`. `bundle/nav2/map.pgm` 을 PNG 로 (`:496-506`) | 404 · 401 |
| `GET /api/jobs/<id>/report.md` | `?dl=<아무 값>` 이면 내려받기 | `text/markdown; charset=utf-8`. 내려받기 이름 `scan2sim_report_<id>.md` (`:509-515`) | 404 · 401 |
| `GET /api/jobs/<id>/bundle.zip` | — | 내려받기 `scan2sim_<id>.zip` (`:538-544`, 만드는 곳 `make_bundle` `:518-535`) | 404 (`bundle/usd/` 없음) · 401 |

- 위 실패 응답의 모양 · 문장은 B 가 Flask 시험 클라이언트로 확인했다 (11/25 저녁, `35063bc`, 엔진은 돌리지 않음): 깨진 zip 예시 파일 → 400 + `id`, 목록에 없는 이름 → 404 HTML, `size: "abc"` → 400, `offset=abc` → 500 HTML, 기다림 1건 · `max_wait 1` → 429, `min_free_gb` 를 아주 크게 → 507.

**예시 파일** (D `9b2c95f`)

- `--samples <폴더>` 를 줬을 때만 있다. 안 줬거나 폴더가 없으면 `GET /api/samples` 는 `[]`, `/api/info` 의 `samples` 는 0.
- 목록: 그 폴더 바로 아래 파일 중 이름이 `[A-Za-z0-9][A-Za-z0-9._-]{0,79}` (`:54`) 이고 확장자가 `.zip` · `.glb` 인 것. 이름순. 요청마다 폴더를 다시 읽는다.
- `label` = 그 폴더 `samples.json` (`{"파일 이름": "이름표"}`) 의 값. 없거나 읽지 못하면 확장자를 뺀 파일 이름.
- `POST /api/samples/<name>/jobs` 는 목록에 있는 이름만 받는다 (경로를 직접 만들지 않음, `:271-273`). 작업 폴더에 예시 파일을 하드 링크로 두고, 안 되면 복사한다. 그다음은 올린 파일과 같다 (입력 검사 → 줄 → 엔진).
- 화면에는 아직 예시 파일 단추가 없다 (A, 계획 6-A 2번).

**거절 · 입력 검사 · 제한 시간** (D `9b2c95f`)

- **줄 길이 — 429** (`busy`, `:180-187`): 기다리는 작업(`queued`, 처리 중인 것은 빼고)이 `--max-wait` 건 이상이면 `{"error": "기다리는 작업이 N건이라 지금은 받을 수 없음 — 몇 분 뒤 다시 올릴 것"}`.
- **디스크 여유 — 507** (같은 함수): 작업 폴더 디스크의 빈 바이트 − 알린 크기가 `--min-free-gb` × 1024³ 보다 작으면 `{"error": "서버 디스크 여유가 모자라 지금은 받을 수 없음 — 서버를 맡은 사람에게 알릴 것"}`.
- 둘 다 **새 작업을 만들 때만** 본다 (`POST /api/jobs` · `POST /api/samples/<name>/jobs`). 큰 파일을 다 올린 뒤가 아니라 올리기 전에 거절하려는 것이다. `start` 에서는 다시 보지 않고, 올리는 중인 다른 작업은 세지 않는다 — 여럿이 동시에 올리면 줄이 `--max-wait` 를 넘거나 디스크 여유가 기준 아래로 갈 수 있다 [코드로만 봄].
- **입력 검사** (`check_input`, `:190-208` · `enqueue`, `:211-221`): 줄에 넣기 전에 (`start` · 예시 파일 작업). 파일을 풀지 않고 앞머리 · zip 목록만 본다.
  - `.glb`: 앞 4바이트가 `glTF` 가 아니면 `"GLB 파일이 아님 (파일 앞머리가 glTF 가 아님) — 스캔 앱에서 GLB 로 다시 내보내 올릴 것"`.
  - `.zip`: 못 열면 `"zip 파일을 열 수 없음 — zip 이 아니거나 올리는 중에 깨짐. 다시 올릴 것"`. 항목이 10만 개를 넘으면 `"zip 안의 파일이 N개로 너무 많음 (최대 100,000개)"`. 풀린 크기 합이 6 GiB 를 넘으면 `"zip 을 풀면 X GB 로 너무 큼 (최대 6 GB)"`. 두 값은 `stray.ZIP_MAX_FILES` · `ZIP_MAX_BYTES` 와 같다 (`:53`) — 엔진이 받는 것을 서버가 막지 않게.
  - 걸리면 응답 400 이고, 작업은 `failed` · `error` = 그 문장 · `finished` 로 `job.json` 에 남는다. 엔진을 돌리지 않아 `started` · `exit_code` · 단계 파일이 없고, 상태 응답의 `verdict` 는 `"fail"` 이다 (11/26 D `6ad7114` 부터 — 전에는 `"pending"`, 8-3 의 2번).
  - zip 안의 경로가 폴더 밖을 가리키는지 · Stray 촬영본으로서 맞는지는 엔진(`stray.py`)이 본다. 맵 폴더를 묶은 zip 도 이 검사는 통과하고 엔진 입력 단계에서 실패한다 (8-1 의 10번).
- **처리 제한 시간** (`worker`, `:390-403`): 엔진 프로세스를 띄운 때부터 잰다 (줄에서 기다린 시간은 빠짐). `--timeout-min` 분을 넘으면 엔진과 엔진이 띄운 하위 프로세스를 함께 죽인다 (`kill_tree`, `:350-377` — Linux 는 `/proc` 로 자식을 찾아 SIGKILL, Windows 는 `taskkill /F /T`).
  - 작업은 `failed` · `timed_out: true` · `exit_code: null` · `error` = `"처리가 N분을 넘어 멈춤 — …"` (2-2). `run.txt` 끝에 `[서버] …` 줄.
  - 그때까지 쓴 단계 파일 · 결과는 남는다. `report.md` · `report.json` 이 없어 `verdict` 는 `"fail"` (11/26 D `6ad7114` 부터 — 전에는 `"pending"`, 8-3 의 2번). 묶음 단계까지 갔으면 `bundle.zip` 도 만든다.
  - [D 측정, `9b2c95f` 커밋 설명] 3초 제한에서 3.3초에 멈추고 남은 프로세스 없음. Windows 의 제한 시간 처리는 돌려 보지 않음.
  - 참고 [B 측정, 통합 기록 11/25]: 가장 긴 실제 촬영본 "인문대 2층" `922acd4755` (182 s)가 B 노트북에서 엔진 6~7분이다. 메모리 16 GB PC 에서는 11분 27초였다 (C 보고). 20분 안이지만 느린 PC 에서는 여유가 크지 않다.

**`preview.glb`**

- **만드는 곳은 엔진**이다. 묶음 단계 끝에 `build/usd/<이름>_mesh.usdc` 로 `out/preview.glb` 를 쓴다 (`stages.py:420-425` → `preview.write`, `preview.py:46-116`, C `2a6e89f`).
  - 못 만들면 묶음 단계 `metrics.preview` 에 `{"error": …}` 와 메시지가 남고, 묶음은 그대로 나온다 (`stages.py:423-425`).
- **서버는 주기만 한다** (D `ed7220e`).
  - `out/preview.glb` 가 없거나 `build/usd/*_mesh.usdc`(이름순 첫 파일)보다 오래됐을 때만 같은 엔진 함수로 한 번 만들어 둔다 (`web/server.py:483-490`). 예: 묶음 단계 전에 멈춘 작업, `preview.py` 가 생기기 전의 작업 폴더.
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

- 안의 구조 (11/25, D `bef9cda`): **맨 위가 `bundle/` 폴더 하나**다. `bundle/…` (2-4 의 `bundle/` 전체) + 같은 `bundle/` 안에 `report.md` · `defects_before.json` · `defects_after.json` (있는 것만) (`web/server.py:528-533`).

  ```
  bundle/usd/input_mesh.usdc · usd/input_scene.usda (· usd/textures/…)
  bundle/nav2/map.pgm · map.yaml · preview.png
  bundle/scene_root.usda · scene_root_sim.usda · start_pose.json · route.csv · bundle.json
  bundle/report.md · defects_before.json (· defects_after.json)
  ```

  - `report.md` · `defects_*.json` 은 zip 에만 넣는다 (`out/bundle/` 에는 없다). 그래서 `bundle/bundle.json` 의 `files` 에는 없다.
  - 그 전 구조: `report.md` · `defects_*.json` 이 맨 위에 `bundle/` 과 나란히 있었다. 이 구조는 `sim/add_site.py` 가 받지 못한다 (2-4).
- **만드는 때:** 작업이 끝날 때 서버가 미리 만든다 (`worker`, `web/server.py:411-414`). `failed` 여도 `bundle/usd/` 가 있으면 만든다. 그 전에 `GET …/bundle.zip` 이 오면 그때 만든다. 같은 파일을 두 곳에서 함께 만들지 않게 잠근다 (`BUNDLE_LOCK`). 임시 이름(`bundle.zip.tmp`)으로 쓰고 바꾼다.
- `out/bundle.zip` 이 있으면 **다시 만들지 않는다** (`web/server.py:525`). 그래서 `bef9cda` 전에 만들어진 `bundle.zip` 은 예전 구조 그대로다. 예: 11/25 B 가 A 에게 준 작업 묶음 `scan2sim_devjobs_1125.zip` (main `c9c5e8a`) — 안의 `bundle.zip` 3개 모두 맨 위에 `report.md` 가 있다 [B 확인]. 새 구조로 보려면 그 작업의 `out/bundle.zip` 을 지우면 다음 요청 때 다시 만든다.
- `room` 실제 (예전 구조): 파일 11개 (`bundle/` 9개 + `report.md` + `defects_before.json`). 새 구조면 11개가 모두 `bundle/` 아래다 [코드로만 봄]. [D 측정, `docs/isaac_check.md`] 새 구조로 가상의 방 13개 · 실제 촬영본 12개.

**올리기 순서 (화면이 하는 것, `index.html:179-201`)**

1. `POST /api/jobs {name, size}` → `id`
2. `offset = 0` 부터 `PUT …/chunk?offset=…`, 다음 `offset` 은 응답의 `received`. 조각 크기 8 MiB (`index.html:144`)
3. `POST …/start`
4. 주소에 `#<id>` 를 붙이고 `GET /api/jobs/<id>` 를 1.5초마다 부름. `done` · `failed` 면 멈춤

- 1 이 429 · 507 이면 올리지 않고 그 문장을 `"실패: …"` 로 보인다. 3 이 입력 검사로 400 이면 같은 식으로 보이고, 주소에 `#<id>` 를 붙이지 않는다 (`index.html:201-205`).
- 예시 파일로는 1~3 대신 `POST /api/samples/<name>/jobs` 한 번 → `id` → 4. 화면에는 아직 없다 (A).

- **이어 올리기** (D `533bffe`)
  - 서버: 조각마다 파일을 `received` 바이트로 먼저 자르고 붙인다 (`web/server.py:326`). 앞 요청이 도중에 끊겨 받다 만 바이트가 남아도, 같은 `offset = received` 로 다시 보내면 된다. `received` 는 409 응답 · 상태 응답에 들어 있다.
  - 서버를 다시 켜도 된다. `uploading` 작업은 읽을 때 `received` 를 올린 파일의 크기로 되찾는다 (4-3).
  - 화면 (`index.html:182-195`): 조각이 실패하면 1 · 2 · 4 … 초(최대 30초) 기다렸다가 `GET /api/jobs/<id>` 의 `received` 부터 다시 보낸다. 8번까지 (`RETRIES`). 접속 코드 오류는 다시 시도하지 않고, 작업이 `uploading` 이 아니면 멈춘다. 서버에 닿지 않으면 같은 `offset` 으로 다시 시도한다.
  - 화면은 같은 페이지 안에서만 이어 올린다. 페이지를 다시 열면 파일을 다시 골라 새 작업으로 올린다.
  - [D 측정, `533bffe` 커밋 설명] 실제 촬영본 540 MB(융복 2층 `6bc1ae021f` 에 D 가 풀어 둔 `rgb/` 를 더 넣은 zip — 11/26 D 답)를 3 MB 에서 끊고 서버를 죽인 뒤 이어 올려 md5 같음. 화면의 다시 시도는 브라우저에서 돌려 보지 않았다.
- `deploy/check.py` 도 같은 순서다. 조각 크기는 `--chunk-mb` 로 바꿀 수 있다 (기본 8).

### 4-3. 작업 상태 (`GET /api/jobs/<id>`)

`job.json` 의 필드 (2-2) 전부 + 아래 필드 (`web/server.py:450-473`). 11/25 저녁(D `9b2c95f`)에 `job.json` 쪽에 `timed_out` · `sample` 이 더해졌다 (2-2). 아래 표의 필드는 그대로다.

| 필드 | 형 | 언제 | 뜻 |
|---|---|---|---|
| `stages` | 배열 (8개, 3-3 순서) | 항상 | `{name, label, status}` + 결과 파일이 있으면 `summary` · `messages` · `seconds` |
| `queue_ahead` | 정수 | 항상 | `queued` 일 때만 계산, 그 밖은 0. 나보다 먼저 줄에 선 `queued` · `running` 작업 수 (**처리 중인 것도 셈**) |
| `defects_before` | 객체 또는 `null` | `done` · `failed` | `defects_before.json` 내용 (5장). 파일이 없으면 `null` |
| `defects_after` | 객체 또는 `null` | `done` · `failed` | `defects_after.json` 내용. **보정을 안 했으면 `null`** |
| `verdict` | 문자열 | `done` · `failed` | 엔진 `report.json` 의 `verdict` (`pass` · `warn` · `fail`). 없으면 `report` 단계의 `status`. **`failed` 인데 그 값이 없거나 `pending` · `running` · `skip` 이면 `"fail"`** (`:471-474`, D `6ad7114`) — 엔진이 보고서까지 가지 못한 `failed`(입력 검사 · 처리 제한 시간 · 서버가 다시 켜져 끊김 · 엔진 프로세스가 죽음)는 11/25 까지 `"pending"` 이었다 (8-3 의 2번). 이유는 `error`. **단계 상태와 따로 볼 것** — `done` 이어도 `fail` 일 수 있다 (3-4) |
| `has_bundle` | 참거짓 | `done` · `failed` | `out/bundle/usd/` 가 있음 = `bundle.zip` 을 받을 수 있음 |
| `metrics` | 객체 | `done` · `failed` | `{"before": …, "after": …, "input": …}` (아래) |

- `metrics.before` = 보정이 돌았으면 `correct` 단계의 `metrics.before`, 아니면 `diagnose` 단계의 `metrics`. 둘 다 없으면 `{}`.
- `metrics.after` = `correct` 단계의 `metrics.after` 또는 `null`.
- `metrics.input` = `input` 단계 파일의 `metrics` 또는 `null` (3-5: `shadow_fix` 는 여기에 없음).

**작업 `status`**

| 값 | 뜻 |
|---|---|
| `uploading` | 자리를 만들고 조각을 받는 중 |
| `queued` | 줄에 들어감 (입력 검사를 통과함). 예시 파일 작업은 만들자마자 이 상태 |
| `running` | 엔진이 돌고 있음 (한 번에 한 건) |
| `done` | 엔진이 끝났고, `out/report.md` 가 있고, **종료 코드가 0 이고, 단계 파일에 `fail` 이 없음** (`web/server.py:405-407`, D `98da703`). `verdict` 는 대개 `pass` 또는 `warn` 이지만, 진단이 "주행할 수 없음" 이유를 남기면 `fail` 이다 (3-4, C `105591c`) |
| `failed` | 그 밖의 끝남. ① 단계가 하나라도 `fail` — 엔진 종료 코드 1, `verdict` `fail`, `report.md` · 결함 목록은 있음. 단계 안의 `SystemExit` 도 11/25 (C `6ec3237`) 부터 여기 든다. ② 엔진 프로세스가 중간에 죽음 — `report.md` 없음. ③ 엔진을 못 띄움 (`error`). ④ 서버가 다시 켜졌는데 그 작업을 이미 한 번 다시 돌렸음 (`error`, 아래). ⑤ **입력 검사에 걸림** (`error`, 엔진을 돌리지 않음 — `started` · `exit_code` 없음, 단계 8개 모두 `pending`, 4-2). ⑥ **처리 제한 시간을 넘음** (`timed_out` · `error` · `exit_code: null`, 4-2). ②~⑥ 은 `verdict` 가 `"pending"` |

- **`status` 와 `finished` 는 같이 바뀐다** (11/26 D `6ad7114`, `:418`). 서버는 엔진이 끝나면 결과를 정해 두고(`:410`), `bundle.zip` 을 만든 뒤에(`:415`) `status` 를 `done` · `failed` 로 바꾸면서 `finished` 를 쓴다. 그래서 `bundle.zip` 을 만드는 동안은 `running` 으로 보이고, 화면이 `done` · `failed` 를 볼 때는 늘 `finished` 가 있다. 11/25 까지는 `status` 를 먼저 바꿔 그 틈에 화면이 `NaN초` 를 적을 수 있었다 (8-3 의 13번).

- 11/23 까지는 `report.md` 만 있으면 `done` 이었다. 그래서 단계가 실패해도 `done` 이 됐다 (`e2e_glb`: `done` · `exit_code 1` · `verdict fail`). 지금은 ① 이 `failed` 다 (통합 기록 R2).
- `failed` 여도 엔진이 만든 것(단계 결과 · 결함 목록 · `report.md` · 있으면 `preview.glb` · `map.png`)은 그대로 받을 수 있다. 화면은 `failed` 면 "실패" 로 적고, 3D · 결함 목록은 같이 보여 준다 (`index.html` `finish`).
- **서버를 다시 켰을 때** (D `533bffe`)
  - 켤 때 `recover()` 가 `job.json` 이 `queued` · `running` 인 작업을 `queued_at` 차례로 줄에 다시 넣는다 (`web/server.py:88-106`, `:566`). 상태는 `queued`, `restarts` 를 1 올려 `job.json` 에 쓴다. 엔진은 처음부터 다시 돈다 (같은 `out/` 에, `--resume` 없이).
  - 이미 한 번 다시 넣은 작업(`restarts` ≥ 1)은 `failed` · `error: "서버가 다시 켜져서 끊김 (두 번째)"` · `finished` 를 `job.json` 에 쓴다. 서버를 죽이는 작업이 끝없이 되돌지 않게 하려는 것이다.
  - 되살리기 뒤에 청소(2-1)가 한 번 돈다 (`:569`). 다시 넣은 작업은 `queued` 라 지우지 않는다.
  - `uploading` 작업은 읽을 때 `received` 를 `input/<file>` 크기(최대 `size`)로 되찾는다 (`:78-80`). 그 자리부터 이어 올릴 수 있다 (4-2).
  - `recover()` 가 살리지 못했는데 `job.json` 이 `queued` · `running` 인 작업은 읽을 때 `failed` + `error: "서버가 다시 켜져서 끊김"` 이 된다 (`:81-82`). 이것은 메모리에서만 바뀐다.
  - [D 측정, `533bffe` 커밋 설명] 처리 중에 서버를 죽이면 다시 켜져 그 작업을 끝냄.

**`stages[].status`**

- 3-2 의 `pass` · `warn` · `fail` · `skip` 에 더해 두 값이 있다 (`web/server.py:454-462`).
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

- 이 예는 `be8310e` 전 판이다. 지금 엔진은 입력 단계 둘째 문장 자리에 자세 보정 문장(6-3)을 내고, `metrics.input` 에 `pose_fix`, `metrics.before` 에 `drivable` · `stray_quality` 가 더 있다.

처리 중(`running`)이면 `verdict` · `has_bundle` · `metrics` · `defects_*` 가 없고, `stages` 는 예를 들어 `input warn · convert running · nav2 pending · …` 처럼 나온다.

---

## 5. 결함 목록 (`defects_before.json` · `defects_after.json`)

`stages._defects` 가 쓴다 (`stages.py:184-246`). 진단 단계(`s_diagnose`)의 끝에서 불린다 (`stages.py:390`).

### 5-1. 파일

| 필드 | 형 | 뜻 |
|---|---|---|
| `phase` | `"before"` · `"after"` | 보정 전 / 보정 뒤 다시 진단 (`stages.py:240`) |
| `T_input_to_scene` | 4×4 수 배열 | `build/transform.json` 의 값. 없으면 단위행렬 (1-3) |
| `up_input` | [3] 수 | 장면의 +Z 를 입력 좌표로 나타낸 단위 벡터 = `T⁻¹` 의 셋째 열 (소수 5자리) |
| `floor_point_input` | [3] 수 | 장면 원점 (0, 0, 0) 을 입력 좌표로 나타낸 점 = `T⁻¹` 의 이동 부분 (소수 4자리). 장면 바닥 z = 0 위의 한 점 |
| `defects` | 배열 | 결함 (5-2). 비어 있을 수 있음 |

### 5-2. 결함 하나

| 필드 | 형 | 뜻 |
|---|---|---|
| `id` | 문자열 | `D1`, `D2` … 정렬한 뒤 순서대로 매김 (`stages.py:237`). 한 파일 안에서만 유일. **보정 전과 뒤의 번호는 서로 관계없음** |
| `type` | 문자열 | `double_floor` · `no_visible_floor` · `low_bump` · `floor_two_levels` · `path_blocked` (5-4) |
| `title` | 문자열 | 화면에 보일 이름 (한국어) |
| `severity` | `"fail"` · `"warn"` | `double_floor` · `path_blocked` 만 `fail` |
| `fixable` | 참거짓 | `true` = 자동 보정 대상: `double_floor`, 그리고 GLB · Stray 의 보정 전(`defects_before.json`) `no_visible_floor` (R9, `stages.py:198-199`). 보정 뒤 목록의 `no_visible_floor` 와 맵 폴더의 것은 `false`. `true` 여도 보정이 덮지 않을 수 있다 (5-4) |
| `center_scene` | [x, y, z] 수 | 장면 좌표의 중심. x · y = 묶인 표본의 평균, z = 그 자리의 바닥 높이(`ground.height`, 1-2) (소수 3자리) |
| `center_input` | [x, y, z] 수 | `T⁻¹ · center_scene` (소수 4자리). 입력 좌표 (1-3) |
| `radius_m` | 수 | 중심에서 가장 먼 표본까지 거리 **+ 지도 2칸(0.1 m)** (`checks.py:43`) |
| `extent_m` | 수 | 묶인 표본의 x · y 범위 상자의 대각선 **+ 지도 1칸(0.05 m)** (`checks.py:44`) |
| `samples` | 정수 | 묶인 표본 수. `path_blocked` 는 걸어간 길 위 10 cm 마다 찍은 점의 수 |
| `basis` | 문자열 | 판정 근거 문장 |
| `action` | 문자열 | 처리 방법 문장 |
| `near_low_bump` | 참 | **있을 때만** (C `3ea14ff`). `double_floor` 가 `low_bump` 결함과 붙어 있으면 `true` (값은 늘 `true`, 아니면 키가 없음). `basis` 끝에 "낮은 턱 의심 자리와 붙어 있음 — …" 이 덧붙음 (`stages.py:224-228`) |

- **정렬:** `fail` 먼저, 그 안에서 `samples` 가 많은 순 (`stages.py:229`). `path_blocked` 의 `samples` 는 길 위 점 수라 다른 종류와 단위가 다르다.
- **최대 30개:** 정렬한 뒤 앞 30개만 남김 (`stages.py:184` `top=30`, `:230`).
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

- **표본** (`stages._free_xy`, `stages.py:173-181`, 부르는 곳 `:258`)
  - Nav2 지도의 빈 칸 중 장애물 · 미지 칸에서 0.3 m(`ROBOT_RADIUS_M`, `stages.py:20`) 넘게 떨어진 칸의 중심.
  - 그 칸들을 행 우선 순서로 세서 4개마다 하나 (`step=4`).
  - 표본 하나가 대표하는 넓이를 `4 × 칸²` 로 친다 (= 0.01 m², `stages.py:189`).
  - 표본이 50개 미만이면 진단 단계가 `fail` (`stages.py:259-260`).
  - `path_blocked` 만 이 표본이 아니라 걸어간 길 위의 점을 쓴다 (5-4).
- **위에서 쏘는 광선:** 표본마다 그 자리 바닥 높이 + 1.0 m 에서 아래로 (`stages.py:264`, `:289`).
- **묶기** (`checks.clusters`, `checks.py:26-45`)
  - 걸린 표본을 Nav2 지도 칸에 찍고, `join_cells` 칸 넓혀(`binary_dilation`) 8방향으로 이어진 것끼리 한 덩어리.
  - `join_cells` 는 기본 2칸(0.1 m). **`double_floor` · `path_blocked` 는 `round(0.5 / 칸)` = 10칸(0.5 m)** 이라 표본 사이가 약 1 m 안이면 한 결함이 된다 (`stages.py:191`, `:219`, C `3ea14ff` · `105591c`). 얇은 판 · 턱의 가장자리가 여러 결함으로 쪼개지지 않게 하려는 것.
  - 덩어리마다 `center` · `radius_m` · `extent_m` · `samples`.

### 5-4. 결함 종류 (지금 구현의 기준값)

| `type` | `severity` · `fixable` | 걸리는 표본 | 결함으로 남기는 덩어리 | 근거 위치 |
|---|---|---|---|---|
| `double_floor` 충돌 바닥 두 겹 | `fail` · `true` | 충돌 prim `Collision` 에 쏜 광선이 맨 위 면을 만난 뒤, 그 바로 아래(1e-4 m)에서 다시 쏜 광선이 **3 cm 안**에서 면을 또 만남 | 모든 덩어리 (크기 하한 없음). 약 1 m 안은 하나로 묶음 (5-3) | `tools/check_floor_layers.py:57-60`, `stages.py:191-194` |
| `no_visible_floor` 보이는 바닥 없음 | `warn` · GLB · Stray 의 보정 전은 `true` (R9), 그 밖 `false` | `build/visual.ply` 에 쏜 광선이 안 맞거나, 맞은 높이가 충돌 바닥과 **0.3 m 이상** 차이. 보정 뒤에는 판(`floor_patch.ply`)에 맞아도 보이는 바닥으로 셈 | `samples × 0.01 m² ≥ 0.25 m²` (= 표본 25개 이상, 칸 0.05 m 기준) | `stages.py:293`, `:195-204` |
| `low_bump` 낮은 턱 의심 | `warn` · `false` | 보이는 바닥이 충돌 바닥보다 **3 cm 초과 12 cm 미만** 높음 | `extent_m ≥ 0.6` | `stages.py:319`, `:205-209` |
| `floor_two_levels` 바닥이 두 높이로 찍힘 (C `ed35a87`) | `warn` · `false` | 보이는 바닥이 있는 표본(`no_visible_floor` 가 아닌 것)에서, `visual.ply` 의 맨 위 면 1 mm 아래에서 다시 쏜 광선이 **3 cm 초과 30 cm 미만** 아래에서 수평에 가까운 면(법선 z 성분의 절댓값 > 0.7)을 또 만남 | `samples × 0.01 m² ≥ 0.25 m²` | `stages.py:294-301`, `:210-217` |
| `path_blocked` 걸어간 길이 장애물로 찍힘 (C `105591c`) | `fail` · `false` | **Stray 만.** 촬영 경로(`input_map/trajectory_camera_optical.txt` 의 카메라 위치 = 자세 보정 전)를 `T_input_to_scene` 으로 장면에 옮겨 10 cm 마다 찍은 점 중, Nav2 지도에서 장애물 칸이고 가장 가까운 빈 칸까지 **0.15 m 넘는** 점 | 약 1 m 안끼리 묶어(5-3) 점 5개 이상 (= 0.5 m 넘게) | `checks.py:64-79`, `stages.py:340-346`, `:218-223` |

- `no_visible_floor` · `low_bump` · `floor_two_levels` 는 `build/visual.ply` 가 있을 때만 찾는다 (`stages.py:280`). `path_blocked` 는 Stray 이고 `trajectory_camera_optical.txt` · `transform.json` 이 있을 때만 (`stages.py:341`).
- **`double_floor` 와 `floor_two_levels` 는 다른 것이다.**
  - `double_floor` 는 **충돌** 메쉬가 3 cm 안에서 겹친 것이다. 물리 엔진을 멈출 수 있어 자동으로 보정한다.
  - `floor_two_levels` 는 **보이는** 메쉬에 바닥이 3~30 cm 아래로 한 겹 더 있는 것이다. 촬영 중 자세가 위아래로 틀어진 흔적으로 보고, 재촬영 후보로 둔다.
  - `floor_two_levels` 의 `basis` 에 나오는 간격은 그 덩어리 안 표본의 간격 중앙값(cm)이다.
- 진단 단계 `metrics.floor_two_levels` = `{samples, pct, gap_cm_median}` (`stages.py:301`). 빈 바닥 표본의 1% 를 넘으면 진단이 `warn` (`stages.py:302-304`). 보정 대상은 아니다.
- `diagnose` metrics 의 `low_bumps.runs_over_0p6m` 은 **다른 방법**(지도에서 이은 영역의 상자 대각선)으로 센 값이다 (`stages.py:321-329`). 결함 목록의 `low_bump` 개수와 다를 수 있다.
- `low_bump` 의 `basis` 문장은 11/24 에 "충돌 바닥은 평평해서" → "충돌 바닥에는 이 솟음이 없어" 로 바뀌었다 (C `3ea14ff`). 문장은 약속이 아니다.
- `no_visible_floor` 의 `basis` · `action` 은 R9 로 바뀌었다. GLB · Stray 보정 전은 "둘레 바닥 색의 판(FloorPatch)으로 덮음 …", 보정 뒤 남은 것은 "보정 단계에서 덮지 않음 — …", 맵 폴더는 전과 같이 "고치지 않음 — …" (`stages.py:202-204`).

**걸어간 길과 판정 "쓸 수 없음"** (C `105591c`, `stages.s_diagnose` `stages.py:336-360`, 수치 `drivable` 은 3-5)

- 막힌 길이 걸어간 길의 **10% 넘음**, 또는 한 덩어리로 이어진 길이 **80% 미만** → 이유를 `ctx["unusable"]` 에 남기고 진단 주의로 보인다. 판정(`verdict`)은 `fail` 이 된다 (3-4).
- 막힌 길 2~10% → 주의만. 빈 바닥의 가장 큰 덩어리가 60% 미만 → 주의만 (모든 입력).
- 제자리 회전 가능 넓이 `turn_area_m2` 가 **5 m² 미만** → 주의만 (모든 입력, C `13d1c4d`, `stages.py:363-365`): `"로봇이 제자리에서 돌 수 있는 넓이가 N m² 뿐임 (5 m² 미만) — 주행 공간이 좁아 주행 실패가 예상됨 (sim 쪽 맵 판정의 '실패 예상'과 같은 기준). …"`. 5 m² 는 `sim/add_site.py` 맵 판정의 "실패 예상" 문턱이다. **판정(`verdict`)은 바꾸지 않는다** — 판정은 맵 상태만 말하고 주행 공간은 이 주의 한 줄로 따로 보인다 (11/26 C · D 결정, `C_TRACK.md` C13 닫음. D 는 `add_site.py` 의 "실패 예상" 문구를 주행 공간 뜻으로 맞추기로 함). [C 측정] 방 촬영본 `6bc1ae021f` 1.35 m² → 주의.
- **되돌아온 자리 어긋남** (Stray, C `13d1c4d`, `stages.py:397-411`, 수치 `revisit` 은 3-5): `stray_revisit` 의 `reasons` 를 모두 진단 주의로 내고, `verdict` 가 `fail` 이면 `"되돌아온 자리가 크게 어긋나 있음 — 같은 곳이 두 번 겹쳐 찍혔을 수 있음. 다시 찍기를 권함 (긴 복도는 한 방향으로만)"` 주의를 더한다. **판정은 주의까지만** — `verdict` `fail` 은 지금처럼 걸어간 길이 막혔을 때만. `report.md` 에 "되돌아온 자리 어긋남 (아이폰 원본)" 절 (`stages.py:680-690`). [C 측정] 복도 왕복 `922acd4755`: 수평 123 → 124.6 cm(남음) · 높이 44 → 1.2 cm → `fail` → 다시 찍기 권함 주의, 판정 주의.
- 막힘은 자세 보정 전 경로로 잰다. 그래서 0.15 m 를 봐준다 (`checks.py:53`).
- 문턱(10% · 80% · 60%)은 촬영본 두 건으로 잡은 잠정값이다 (`stages.py:350`). [C 측정, `C_TRACK.md` 2-6] 복도 왕복 `922acd4755`: 막힌 길 36.5% (61.7 m) · 이어진 길 62.4% · 가장 큰 덩어리 53.2% → "쓸 수 없음", `path_blocked` 4곳. 방 한 바퀴 `6bc1ae021f`: 0.2% · 100% · 89.5% → 판정 그대로.

**보정 (`s_correct`, `stages.py:450-495`)**

- 진단에서 `double_floor` 가 하나라도 나오거나, GLB · Stray 에서 `no_visible_floor` 가 하나라도 나오면 돈다 (`ctx["needs"]`, `stages.py:274`, `:391-392`). 다른 종류로는 돌지 않는다.
- 두 겹 바닥 (`double_floor` 가 있을 때)
  - GLB · **Stray**: 충돌 메쉬에서 바닥 위 5 cm 아래를 지움 (`collision.low_h_m: 0.05`, `stages.py:462-464`). Stray 는 11/24 부터 이쪽이다 (GLB 길로 가므로).
  - 맵 폴더: 바닥 위 10 cm 안의 낮은 조각과 6 cm 안의 수평 면을 지움 (`single_floor.collision_low_h_m: 0.10`, `collision_flat_h_m: 0.06`, `stages.py:465-467`)
- 보이는 바닥 구멍 덮기 (R9, C `be8310e`. GLB · Stray 만, `stages._patch_mask` `stages.py:397-447`, `:468-477`)
  - Nav2 지도의 빈 칸마다 바닥 + 10 cm 에서 `visual.ply` 로 아래로 쏜다. 면이 없거나 바닥보다 band(평평한 바닥 5 cm · 아니면 8 cm) 넘게 아래에서 맞은 칸을 8방향으로 이어 구멍으로 본다. 보정 전 `no_visible_floor` 결함의 원(`center_scene` · `radius_m`)에 걸친 구멍만 본다.
  - 덮지 않는 구멍: 바닥 아래에서 맞은 칸이 10% 넘음(꺼진 자리 · 계단) · 둘레 약 20 cm 띠의 보이는 바닥이 50% 미만 · 4 m² 넘음. 그 이유는 `correct` 의 `messages` 와 `metrics.floor_patch.regions[].reason`.
  - 덮을 구멍은 한 칸 넓혀(빈 칸 안에서만) `build/floor_patch_mask.npz` 에 쓰고, 보정 설정 `floor_patch: {mask_file, depth_m: 0.005}` 로 다시 빌드한다.
  - 빌드가 보이는 prim **`FloorPatch`** 와 `build/floor_patch.ply` 를 만든다 (`pipeline/glb_to_usd.py:399-422` `floor_patch_from_mask`, `:642-652`). 높이 = 바닥 − 0.5 cm, 색 = 둘레 스캔 바닥 정점의 색. 충돌 · Nav2 지도에는 넣지 않는다.
  - 다시 진단할 때 판에 맞은 표본은 보이는 바닥으로 세지만, `visible_minus_collision_cm` 에서는 빼고 `floor_patch_samples` 로 센다. `floor_two_levels` 는 판을 두 번째 면으로 세지 않는다 (`stages.py:284-309`).
  - 구멍 찾기는 진단이 만든 광선 구조(판이 없는 보정 전 장면)를 다시 쓴다 — `visual.ply` 의 경로 · 수정 시각이 같을 때만 (`ctx["_vis_scene"]`, `stages.py:286`, `:414-419`).
  - 덮은 자리는 `report.md` 의 "이 맵이 가정한 것과 바꾼 것" 에 위치와 함께 적힌다 (`stages.py:671-673`).
  - 두 겹 바닥도 없고 덮을 구멍도 없으면 `correct` 는 `skip` ("고칠 수 있는 자리가 없어 보정하지 않음", `stages.py:478-479`). 이때 `defects_after.json` 은 없다.
  - [C 측정, `C_TRACK.md` 2-5] 방 촬영본 `6bc1ae021f`: 후보 2곳 모두 "둘레에 보이는 바닥이 적음" 으로 덮지 않음.
- 그 뒤 `convert` → `nav2` → `diagnose` 를 다시 돌린다 (`stages.py:509-524`). 이때 `defects_after.json` 을 쓴다.
  - **판만 더하기 (R16, C `29a05ba`):** 고칠 것이 `no_visible_floor` 하나뿐이고 GLB 길(GLB · Stray)이면, `pipeline/glb_to_usd.py --config <보정 설정> --patch-only` 로 이미 만든 USD 메쉬에 `FloorPatch` prim 만 더하고 `convert` · `nav2` 는 건너뛴 채 `diagnose` 만 다시 한다 (`stages.py:510-519`). 판은 보이는 부분이라 충돌 · Nav2 지도 · 바닥 높이가 그대로이기 때문이다. 결과 USD 는 처음부터 다시 만든 것과 prim 순서만 다르다 [C 확인]. [C 측정] 복도 왕복 촬영본 보정 단계 209 → 10 s.
  - 판을 더할 수 없는 빌드 — `build/report.json` · `build/floor_samples.npz`(보이는 바닥 정점 xy · 색, 11/26 부터 빌드가 씀) · 보정 설정의 `floor_patch.mask_file` 중 없는 것이 있음 — 면 `glb_to_usd.py` 가 종료 코드 2 로 끝나고, 전처럼 처음부터 다시 만든다.
  - `correct` 의 metrics 에 `patch_only` (참 = 판만 더함). 판만 더했으면 `messages` 에 "판만 기존 USD 에 더함 — 변환 · Nav2 지도는 다시 하지 않음 …". 판만 더한 빌드의 `build/report.json` 에는 `floor_patch.patch_only: true` · `patch_seconds`.
- **`defects_after.json` 은 보정이 돌았을 때만 생긴다.** 보정 단계가 `skip` 이면 없고, 상태 응답의 `defects_after` 는 `null`.
- 보정 뒤에도 두 겹 바닥이 남으면 `correct` 단계가 `fail` (`stages.py:269-270`). 그러면 작업 상태는 `failed` 다 (4-3).

---

## 6. Stray Scanner 입력 (`stray.py`)

### 6-1. 받는 것

- 촬영 폴더에 `rgb.mp4` · `odometry.csv` · `camera_matrix.csv` · `depth/` 가 있어야 한다 (`stray.NEED`). 넷 중 둘 이상이 있으면 촬영 폴더로 보고(`stray._looks_stray`), 빠진 것을 이름으로 알려 주며 실패한다.
- zip 은 풀지 않고 이름만 보고 촬영 폴더를 찾는다 (`stray._zip_captures`). macOS 가 넣는 `__MACOSX/` · `._*` 는 무시한다.
  - zip 안 이름은 `stray._zip_parts` 로 나눈다. `\` 는 `/` 로 본다 (윈도우 PowerShell 5 의 `Compress-Archive` 등이 `\` 로 씀). 빈 조각과 `.` 은 뺀다 — `./rgb.mp4` = `rgb.mp4`, `./` 항목은 무시.
- 처리할 때는 `out/input_map/_unzip/` 에 풀고 그 안의 촬영 폴더를 쓴다 (`stages.s_input`). 폴더가 몇 겹 아래에 있어도 된다. 풀 때도 `\` 를 `/` 로 고쳐 폴더를 만든다 (`stray._unzip`).
- 풀기 전에 막는 zip (`stray.find_capture`): 깨진 zip, 촬영 폴더가 여러 개, 파일 10만 개 또는 풀면 6 GB 넘음(`stray.ZIP_MAX_FILES` · `ZIP_MAX_BYTES`), 폴더 밖을 가리키는 경로.
  - 폴더 밖 경로: 모든 항목을 먼저 확인한 뒤 푼다. `../` 로 밖으로 나가거나 절대 경로인 항목이 하나라도 있으면 `CaptureError` 이고 **아무것도 풀지 않는다** (`os.path.commonpath` 로 확인, `stray._unzip`).
  - 푸는 중에 압축이 깨졌거나 자리가 모자라면 `CaptureError` "zip 을 풀지 못함 — …".
  - 입력 판별(`detect.kind_of`, `detect.py:16`)도 이 함수를 부르므로, 깨진 zip 은 판별 때 이미 `CaptureError` 가 난다.
- **깊이 · 신뢰도 파일 짝짓기** (`stray.frame_ids`)
  - `odometry.csv` 의 `frame` 열이 0 이상 정수이고 차례대로 커지면, 줄마다 그 번호의 파일(`depth/<frame 6자리>.png` · `confidence/…`)과 짝짓는다. 자세 줄이 빠진 촬영에서 줄 순서로 짝지으면 그 뒤 깊이가 모두 밀리기 때문이다.
  - 그렇지 않거나, frame 번호로 짝지을 때 없는 파일이 줄 순서로 짝지을 때보다 많으면 줄 순서로 짝짓는다.
  - 영상(`rgb.mp4`)은 늘 줄 순서다.
  - 어느 쪽인지는 `stray_meta.json` 의 `depth_pairing` (`"frame"` · `"row"`) 과 `checks.info.depth_pairing` (6-3). frame 열이 0, 1, 2 … 그대로면 `"frame"` 이다 (줄 순서와 같음).
  - `stray_quality` 도 같은 함수로 짝짓는다.
- **입력 검사** (`stray.validate`, `convert` 첫머리). 실패면 `stray.CaptureError` 를 낸다.
  - `input` 단계가 이 예외를 받아 `fail` 로 끝내고, **요약은 그 문장 하나**, `messages` 는 비어 있다 (`stages.s_input`, 통합 기록 R4 — C `620a2b0`). 종류 판별(`detect.kind_of`) 때 난 것도 같다.
  - [C 측정] 깨진 zip → 요약 "zip 파일을 열 수 없음 — …" 한 문장.

| 검사 | 실패 | 주의만 |
|---|---|---|
| 촬영 길이 | 2 초 또는 100 장보다 짧음 (`MIN_S` · `MIN_ROWS`) | — |
| `odometry.csv` | 열이 13개보다 적음 · 숫자 아님 · NaN · 사원수 길이가 1 에서 1% 넘게 벗어남 · 시각이 커지지 않음 (줄 번호와 함께) | — |
| `depth/` · `confidence/` 장 수 | 자세 줄 수와 3장 넘게 다름. frame 번호가 건너뛰는 촬영이면 그 번호의 파일이 3개 넘게 없음 | 1~2장 다름 (`COUNT_TOL`) · frame 번호가 0, 1, 2 … 로 이어지지 않음(어느 쪽으로 짝지었는지 적음) · `confidence/` 없음 또는 빔 |
| 깊이 파일 | `depth/` 에 `.png` 가 없음 · 첫 장(짝지은 첫 번호)을 못 읽음 · 16비트 한 장이 아님 | — |
| `rgb.mp4` | 열 수 없음 · 장 수가 자세 줄 수의 90% 미만 (`VIDEO_MIN`) · 4:3 이 아님 | — |
| 내부 파라미터 | `camera_matrix.csv` 가 3×3 숫자가 아님 · fx · fy ≤ 0 · 주점이 영상 밖 | — |
| 키프레임 수 | — | 2,000장을 넘어 간격을 넓힘 (6-2) |
| 깊이 규칙 | — | 바닥 높이를 찾지 못해 `dist` 규칙으로 물러섬 (6-2) |

- 변환 중에 걸리는 것 (`stray.convert`)
  - 첫 깊이 파일(이름순)을 못 읽거나 16비트 한 장이 아님 → `CaptureError`.
  - 키프레임 깊이의 크기 · 형식이 첫 장과 다름, 신뢰도 그림 크기가 깊이와 다름 → `CaptureError` ("… 깨졌거나 다른 촬영과 섞였음").
  - 키프레임 깊이 파일 하나가 있는데 못 읽음 → 그 키프레임만 빼고 주의 ("깊이 파일 n개를 읽지 못해 그 키프레임을 뺌 …").
  - 쓸 수 있는 장이 2장 미만 → `CaptureError`.
- 주의는 `stray_meta.json` 의 `checks.warnings` 에 남고 (6-3), `input` 단계의 주의로도 나간다 (R4, C `620a2b0`).

### 6-2. 변환 값 (`stray.convert` 기본값; 엔진은 기본값만 씀)

| 항목 | 값 | 뜻 |
|---|---|---|
| `width` | 없음 = 깊이 원래 크기 | 키프레임을 깊이 원래 크기(아이폰 256×192)로 저장. 색은 1920 → 960 → 256 두 번에 나눠 줄임. 내부 파라미터는 `camera_matrix.csv` 를 화소 중심 기준으로 줄임: `c' = (c + 0.5)·s − 0.5`, `f' = f·s` (`stray_quality._intr` 와 같음). `--width 960` 을 주면 전처럼 960×720 (깊이는 최근접으로 키움) — 사진 텍스처를 구울 때만 필요. 11/24 B 측정: 960 과 메쉬 차이 중앙 0.4 mm, 합치기 약 2배 빠름 |
| `conf_rule` | `"hf"` | 깊이 고르기 규칙 (아래). `"min"` = 예전 규칙. CLI `--conf-rule hf\|min` |
| `max_depth_m` | 5.0 | 신뢰도 2 깊이를 쓰는 한계. 넘는 깊이는 0 으로. CLI `--max-depth` |
| `near_conf1_m` | 3.0 | 신뢰도 1 깊이를 쓰는 한계 (`hf` · `dist`). 함수 인자만 (CLI 없음) |
| `floor_margin_m` | 0.10 | `hf`: 신뢰도 1 은 세계 높이가 바닥 + 10 cm 보다 높을 때만. 함수 인자만 |
| `min_conf` | 1 | `conf_rule="min"` 일 때만: 신뢰도(0 · 1 · 2)가 이보다 낮은 깊이는 0 으로. CLI `--min-conf` |
| `move_m` · `turn_deg` | 0.05 · 5.0 | 앞에 고른 장에서 5 cm 넘게 움직였거나 5° 넘게 돌았을 때 키프레임으로 고름 (`stray.select`) |
| `max_keyframes` | 2000 (`stray.MAX_KEYFRAMES`) | 고른 장이 이보다 많으면 `move_m` · `turn_deg` 를 함께 넓혀 상한에 가장 가깝게 다시 고름 (이분 탐색, 최대 32배). 넓히면 `checks.warnings` 에 주의. 11/24 전에는 800장 · 1.2배씩이었음 |
| `max_speed` | 1.5 m/s | 5프레임 평균 속력이 이보다 빠른 장은 뺌 (흐림) |
| `skip_s` | 1.0 | 촬영 첫 1초는 건너뜀 (시작 직후 자세가 튐) |
| 색 저장 | JPEG 품질 92, 영역 평균 축소 | |
| 깊이 저장 | 16비트 PNG(mm) | 규칙으로 버린 화소는 0. 기본은 원래 크기 그대로. `--width` 로 키울 때만 **최근접** 확대 (섞으면 가장자리에 없는 점이 생김) |
| 자세 | `Y_UP_TO_Z_UP · T_stray` | 1-3 (`stray.convert` 안의 `pose`) |

**깊이 고르기** (`stray.depth_rule` 이 규칙을 정하고, 키프레임마다 `stray.depth_keep` 이 화소를 고름)

- `hf` (기본): 신뢰도 2 는 `max_depth_m`(5 m)까지 쓴다. 신뢰도 1 은 `near_conf1_m`(3 m) 안이고, 그 화소의 세계 높이가 바닥 + `floor_margin_m`(10 cm) 보다 높을 때만 쓴다. 신뢰도 0 은 버린다.
  - 까닭: 멀리 · 비스듬히 본 바닥의 신뢰도 1 깊이가 바닥을 두 겹으로 만들었다 (1.5 m 넘는 곳부터). 신뢰도 2 만 쓰면 어두운 물체가 빠져서, 바닥보다 높은 신뢰도 1 은 남긴다 (`stray.py` 머리말 "깊이 고르기").
  - 높이는 보정 전 자세(자세 보정보다 먼저 돎)와 `camera_matrix.csv` 를 깊이 크기로 줄인 내부 파라미터(화소 중심 기준)로 잰다.
  - 바닥 높이는 촬영 전체에 하나다 (`depth_rule.floor_z_m`, 키프레임마다 따로 정하지 않음).
- **바닥 높이 먼저 훑기** (`stray.depth_rule`)
  - 고른 키프레임 10장마다 1장 (`FLOOR_EVERY`. 짧은 촬영은 30장 넘게 되게 촘촘히, `FLOOR_MIN_FRAMES`).
  - 그 장들의 신뢰도 2 · `max_depth_m` 안 화소 중, 위를 향한 면(세계 법선 z > 0.9, `FLOOR_UP_NZ` — 높이 보정과 같은 법선 `stray_fix.frame_points`)이고 카메라 높이 중앙값보다 0.6~2.2 m 아래(`FLOOR_SEARCH_M`)인 것의 세계 높이를 2 cm 칸으로 센다.
  - 가장 많은 칸의 30% 이상(`FLOOR_PEAK_FRAC`)인 가장 낮은 칸 → 그 위 10 cm 안의 봉우리 = 바닥 높이 (`pipeline/glb_to_usd.find_floor` 와 같은 생각).
  - 바닥을 못 찾은 것으로 보는 경우 (하나라도): 신뢰도 2 깊이가 없음 · 그 띠 안에 위를 향한 신뢰도 2 면이 없음 · 바닥 ±5 cm 의 신뢰도 2 화소가 5,000개 미만(`FLOOR_MIN_PX`) · 띠 안 위를 향한 신뢰도 2 면 중 바닥 ±5 cm 몫이 30% 미만(`FLOOR_MIN_SHARE`) · 바닥이 카메라 높이 중앙값보다 0.6 m 넘게 낮지 않음(`FLOOR_BELOW_CAM_M`).
- **물러서는 규칙** — 이유는 `checks.warnings` 에 남고 입력 단계 주의로도 나간다 (6-1).
  - `dist`: `hf` 에서 바닥을 못 찾으면. 신뢰도 1 은 3 m 안이면 높이와 상관없이 쓴다. 이유는 `depth_rule.fallback_reason`. 주의 문장은 "바닥 높이를 찾지 못함 (…) — 신뢰도 1 깊이를 높이와 상관없이 3 m 안에서 씀. 멀리 본 바닥이 두 겹으로 찍힐 수 있음".
  - `no_confidence`: `confidence/` 폴더가 없으면. 3 m(`near_conf1_m`) 안의 깊이를 모두 쓴다. `fallback_reason` 은 없고, 주의는 입력 검사의 "`confidence/` 가 없음 — 신뢰도로 거르지 않고 깊이를 모두 씀" (실제로는 3 m 까지).
    - `confidence/` 가 있는데 비었으면 `no_confidence` 가 아니라 바닥을 못 찾아 `dist` 가 된다. 신뢰도 그림이 없는 장은 `min` 이 아닌 규칙이면 3 m 안 깊이를 모두 쓴다 (`min` 은 `max_depth_m` 까지 모두).
  - `min`: `--conf-rule min` · `convert(conf_rule="min")` 로만. 예전 규칙 = 신뢰도 `min_conf` 이상 · `max_depth_m` 까지.
- **엔진에서:** `stages.s_input` 이 `stray.convert` 와 `stray_mesh.build` 를 기본값으로 부르므로 **`hf` 규칙 · 5 m · 수평 자세 보정이 엔진에서 쓰인다** (R10, C `be8310e`). 11/24 까지는 형상 메쉬가 `pipeline/tsdf.py` (깊이 3 m)라 3~5 m 의 신뢰도 2 깊이(천장 · 먼 벽)가 메쉬에 들어가지 않았다.
- B 측정 (기준 촬영본, R10 을 넣은 복사본 엔진, 통합 기록 11/24 저녁): 예전 규칙(신뢰도 1 이상) · 3 m 대비 `floor_two_levels` 15.8 → 3.2% (자세 보정 없이), 천장 덮음 28 → 56% (`has_ceiling` 참), 보이는 바닥 없음 0.8 → 6.9%.

- 형상 메쉬 — 지금 엔진 (`stray_mesh.build` 기본값, `stages.py:64-70`, R10 · C `be8310e`)
  - 칸 2 cm · sdf 폭 칸 × 4 · 깊이 5 m · 키프레임 모두, 합치기 전에 자세 보정 (6-4). 그림 줄여 읽기(`image_step`)는 기본 1 그대로.
  - 결과는 `build/mesh_raw.ply` · `build/tsdf_meta.json` · `build/pose_fix.json`. 메쉬 로그는 `out/log.txt` 에 붙는다.
  - 못 만들면 (`stray.CaptureError` — `stray_mesh.MeshError` 도 그 한 종류) 입력 단계 `fail` · 요약 `"촬영본에서 형상을 만들지 못함"` · `messages` = 그 문장 하나 · metrics `{stray}`.
  - `build/tsdf_config.yaml` 은 더 생기지 않는다 (`detect.config_tsdf` 지움). 11/24 까지는 `pipeline/tsdf.py` (칸 2 cm · 깊이 3 m · 보정 없음)였다.

### 6-3. `input_map/stray_meta.json` (`stray.convert` 끝)

- **변환을 마친 표시다.** `stray.convert` 가 맨 끝에 쓴다. 엔진의 `--resume` 은 이 파일이 있으면 변환을 건너뛴다 (3-1).
- `convert` 는 시작할 때 맵 폴더의 예전 `stray_meta.json` · `trajectory_camera_optical.txt` · `pose_fix.json` · `floor_fix.json` · `stray_quality.json` · `stray_revisit.json` 을 지운다 (`stray.STALE`). 중간에 멈춘 변환이 끝난 것처럼 보이지 않게 하려는 것이다. `keyframes/rgb` · `depth` · `calib` 도 비우고 새로 쓴다.
- `trajectory_camera_optical.txt` 와 `stray_meta.json` 은 임시 이름(`.writing`)으로 쓰고 바꿔 놓는다 (`stray._write_text`). `stray_meta.json` 이 마지막이다.

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
| `confidence_pct` | [3] 수 | 고른 키프레임 깊이 화소의 신뢰도 0 · 1 · 2 비율 (%) |
| `confidence_used_pct` | [3] 수 | 신뢰도 0 · 1 · 2 화소 중 규칙으로 맵에 쓴 비율 (%) |
| `min_confidence` | 정수 | 예전 키. `min` 규칙이면 `min_conf`, 그 밖은 1. 규칙은 `depth_rule` 을 볼 것 |
| `depth_rule` | 객체 | 쓴 깊이 규칙 (6-2). `name` = `hf` · `dist` · `no_confidence` · `min`. `hf` · `dist` · `no_confidence` 는 `max_depth_m` · `near_conf1_m` · `floor_margin_m` · `floor_z_m` (`hf` 일 때만 값, 그 밖은 null). `min` 은 `min_confidence` · `max_depth_m`. `hf` · `dist` 는 바닥 훑기 결과 `floor` = `{frames, step, conf2_px, floor_px, floor_px_pct, floor_z_est_m, camera_above_floor_m, camera_z_median_m, search_below_camera_m, up_px, floor_up_px, floor_up_pct}` (`up_px` = 띠 안 위를 향한 신뢰도 2 화소, `floor_up_*` = 그중 바닥 ±5 cm). `dist` 는 `fallback_reason` (문장). `stray_quality` 가 안 쓴 프레임에 같은 규칙을 쓴다 |
| `depth_pairing` | 문자열 | `"frame"` · `"row"` — 깊이 · 신뢰도 파일을 frame 번호로 짝지었나 (6-1, `stray.frame_ids`) |
| `valid_depth_pct` | 수 | 쓸 수 있는 깊이 화소(깊이 > 0 · `max_depth_m` 안 · 신뢰도 1 이상 — 규칙과 상관없는 촬영 상태) 비율의 키프레임 평균 (%). `595d439` 전에는 "걸러낸 뒤 남은 비율" 이었다 (지금의 `used_depth_pct` 쪽) |
| `used_depth_pct` | 수 | 규칙으로 실제로 남긴 화소 비율의 키프레임 평균 (%) |
| `select` | 객체 | `{move_m, turn_deg, max_speed_mps, max_depth_m, skip_s, max_keyframes, widen, move_m_used, turn_deg_used}`. `widen` = 키프레임 상한 때문에 넓힌 배율 (1 = 안 넓힘), `*_used` = 실제로 쓴 간격 |
| `checks` | 객체 | `{warnings: [문장], info: {odometry_rows, duration_s, depth_pairing, depth_files, confidence_files, video_size, video_frames, video_fps}}` — 입력 검사 결과 + 변환 중 주의 (키프레임 간격 넓힘 · 깊이 규칙 물러섬 · 못 읽은 깊이 파일). `info` 의 키는 검사가 어디까지 갔는지에 따라 빠질 수 있음 |

실제 파일 (`ref` — 지금 코드, `room` 과 같은 촬영본. 배열은 한 줄로 모음):

```json
{"source": "6bc1ae021f", "format": "stray_scanner", "frames_total": 6611, "frames_read": 6610, "keyframes": 1133,
 "dropped_fast": 4, "duration_s": 118.3, "path_length_m": 43.83, "extent_m": [9.41, 5.29, 0.9],
 "camera_height_range_m": [-0.4, 0.5], "speed_median_mps": 0.37, "speed_max_mps": 3.41,
 "color_size": [1920, 1440], "depth_size_in": [256, 192], "keyframe_size": [256, 192],
 "intrinsics": {"fx": 177.326, "fy": 177.326, "cx": 127.306, "cy": 96.297}, "focal_variation_pct": 1.56,
 "confidence_pct": [28.9, 26.8, 44.3], "confidence_used_pct": [0.0, 52.9, 92.1], "min_confidence": 1,
 "depth_rule": {"name": "hf", "max_depth_m": 5.0, "near_conf1_m": 3.0, "floor_margin_m": 0.1, "floor_z_m": -1.547,
                "floor": {"frames": 114, "step": 10, "conf2_px": 2311187, "floor_px": 893480, "floor_px_pct": 38.7,
                          "floor_z_est_m": -1.547, "camera_above_floor_m": 1.44, "camera_z_median_m": -0.11,
                          "search_below_camera_m": [0.6, 2.2], "up_px": 958621, "floor_up_px": 843768, "floor_up_pct": 88.0}},
 "depth_pairing": "frame", "valid_depth_pct": 66.1, "used_depth_pct": 55.0,
 "select": {"move_m": 0.05, "turn_deg": 5.0, "max_speed_mps": 1.5, "max_depth_m": 5.0, "skip_s": 1.0,
            "max_keyframes": 2000, "widen": 1.0, "move_m_used": 0.05, "turn_deg_used": 5.0},
 "checks": {"warnings": [],
            "info": {"odometry_rows": 6611, "duration_s": 118.33, "depth_pairing": "frame", "depth_files": 6611, "confidence_files": 6611,
                     "video_size": [1920, 1440], "video_frames": 6611, "video_fps": 55.93}}}
```

- 바닥 높이 −1.547 m 는 입력 좌표(촬영을 시작한 폰 자리가 원점, 1-3)다. `room` 의 `floor_point_input` z (−1.5549, 5-2)와 1 cm 안이다.
- 11/24 전의 `room` 은 키프레임 681장 · `keyframe_size` [960, 720] · 상한 800 에 걸려 간격 1.73배였다 (`checks.warnings` 에 주의 하나). 지금 코드로는 상한 2,000 안이라 넓히지 않는다.

**입력 단계가 Stray 에 대해 내는 판정** (`stages.s_input`)

- Stray 만 붙는 주의
  - 촬영 범위가 좁음: `path_length_m < 5` 또는 `max(extent_m[:2]) < 3`
  - 깊이가 적음: `valid_depth_pct < 50` (규칙과 상관없는 촬영 상태로 봄, 위 표)
  - 촬영이 빠름: `speed_median_mps > 0.8`
  - `checks.warnings` 의 문장 모두 (R4, C `620a2b0`)
- 자세 보정 문장 (R11, C `be8310e`, `stages.py:98-109`) — `build/tsdf_meta.json` 의 `pose_fix` (6-4)를 읽는다. 수치는 `metrics.pose_fix` (3-5).
  - `applied` 가 참: **알림만** (`messages` 에 넣고 상태는 그대로) `"촬영 중 틀어진 자세를 고쳐서 합침"` (+ `reasons` 가 있으면 괄호 안에 ` · ` 로 이어 붙임).
  - `pose_fix` 가 있는데 `applied` 가 거짓 (`skipped` · `refused` · `error` · `off`): 주의 `"촬영 중 틀어진 자세를 고치지 않음 — 긴 구간은 틀어진 채일 수 있음"` (+ `reasons`).
  - `pose_fix` 가 없음: 주의 `"아이폰 자세에는 되돌아온 자리를 맞추는 보정이 없음 — …"`. `stray_mesh.build` 는 늘 `pose_fix` 를 쓰므로(끄면 status `off`) 지금은 `--resume` 으로 11/24 전 빌드(`pipeline/tsdf.py` 의 `tsdf_meta.json`)를 그대로 쓸 때만 나온다.
  - `flags` 의 문장은 하나씩 주의 (예: `"수평: u 축(…): 벽을 못 본 구간이 최대 16 초 — …"`).
  - 11/24 까지는 "되돌아온 자리 보정 없음" 주의가 늘 붙어 Stray 입력의 `input` 단계가 언제나 `warn` 이상이었다. 지금은 보정이 됐고 다른 주의가 없으면 `pass` 일 수 있다.
- GLB 와 같이 형상 메쉬를 보고 내는 판정 (`stages.py:78-85`, 11/24 부터 Stray 에도)
  - 실패: 범위가 1 m 미만 · 300 m 초과, 바닥(위 향한 면) 2 m² 미만.
  - 주의: 바닥이 평평하지 않음. 바닥 대부분(75% 이상)이 한 높이면 "바닥의 n% 가 다른 높이에 찍힘" 으로 적는다.
- "사진 텍스처가 없음" 주의는 Stray 에는 붙지 않는다 (`stages.py:86`).
- 요약은 `"아이폰 원본 → 형상 메쉬 · 천장 … · 바닥 … · 범위 …"` (`stages.py:77`). 11/23 까지는 `"아이폰 원본 → 맵 폴더 · …"` 였다.

`input_map/trajectory_camera_optical.txt` 의 형식: `시각 x y z qx qy qz qw id` (카메라 광학 → 입력 세계, 첫 줄은 `#` 주석). **자세 보정 전** 자세다 (6-4).

### 6-4. 자세 보정과 그 결과 파일 (B `stray_fix.py` · `stray_mesh.py`)

**엔진은 11/25 (R10, C `be8310e`) 부터 이것을 쓴다.** `stages.s_input` 이 Stray 형상 메쉬를 `stray_mesh.build` 기본값(11/25 저녁부터 `fix "auto"` — 아래 "자동 고르기", 그 전은 `"xy"`)으로 만든다. 그래서 엔진 출력에 `build/pose_fix.json` 이 있고, `build/tsdf_meta.json` 에 `pose_fix` 가 있으며, `stray_quality` 는 보정한 자세로 잰다. 입력 단계는 그 요약을 문장(6-3)과 `metrics.pose_fix`(3-5)로 내보낸다. 11/24 까지의 엔진(`pipeline/tsdf.py`)은 이 파일들이 없었다.

**하는 일** — `stray_mesh.build(map_root, build_dir, …, depth_trunc=5.0, fix=…)` 가 깊이를 합치기 전에 키프레임 자세를 고친다 (`stray_fix.fix_poses`).

- `fix` 기본 `"auto"` (`stray_mesh.AUTO`) = 아래 "자동 고르기"로 `"xy"` 나 `"floor+xy"` 를 고름. `"xy"` (`stray_fix.DEFAULT_FIX`, `stray_fix` CLI 기본) = 수평만. `"floor+xy"` = 높이 → 수평 (수평 단계는 높이를 고친 자세를 받음). `"floor"` = 높이만. `None` = 끔 (자세 그대로 — 같은 설정이면 `pipeline/tsdf.py` 와 바이트까지 같은 메쉬, 통합 기록 11/24 저녁). 관문에서 모든 단계를 건너뛰어도 같다.
- `depth_trunc` 기본 5.0 m (`stray.py` 가 키프레임 깊이를 이미 규칙대로 골라 둠, 6-2). 칸 2 cm · sdf 폭 칸 × 4 · 키프레임 모두는 `pipeline/tsdf.py` 와 같다.
- CLI: `python -m scan2sim.stray_mesh <맵 폴더> --out <빌드 폴더> [--fix auto|xy|floor+xy|floor|none] [--capture <촬영 폴더 · zip>] [--no-fix] [--tilt | --no-tilt] [--depth-trunc 5.0]`
  - `--tilt` / `--no-tilt` 는 높이 단계의 설정이다. `--fix` 없이 주면 재지 않고 높이 단계를 더해 `floor+xy` 로 돌고 한 줄 남긴다(예전과 같음). `--fix auto` 와 같이 주면 높이 단계를 켤 때만 쓴다. `--fix xy` 나 `--no-fix` (`--fix none`)와 같이 주면 오류로 멈춘다 (`stray_fix.tilt_fix`).
  - `--capture` 는 auto 가 잴 촬영 원본(auto 에서만, 아니면 종료 코드 2).
- **자동 고르기 (11/25 B, 사용자 결정 — `fix "auto"`, 엔진 기본)**: 합치기 전에 촬영 원본의 원래 자세(ARKit)로 되돌아온 자리 어긋남을 잰다(`stray_revisit.measure` 의 `raw`, 6-5 — 4~8 초 · 0.9 GB 이하). 믿을 쌍이 있고 높이 어긋남 최대 `z_max_cm` 가 15 cm(`stray_mesh.AUTO_FLOOR_Z_CM` = `stray_revisit.FAIL_Z_CM`)를 넘으면 `"floor+xy"`, 아니면 `"xy"`. `"xy"` 를 고르면 메쉬는 `fix "xy"` 와 바이트까지 같다.
  - 왜: 높이 단계는 방 촬영본에서 천장을 나쁘게 해 기본에서 뺐지만(아래 "B 측정"), 긴 왕복 촬영에서 아이폰 높이가 고르게 틀어지면 같은 바닥이 두 높이로 찍혀 걸어간 길이 막힌다. 근거(11/25 엔진): `922acd4755` 원래 자세 높이 어긋남 44 cm → `floor+xy`, 막힌 길 35.8 → 0.0% · 판정 쓸 수 없음 → 주의 · 안 쓴 프레임 중앙 6.13 → 3.49 cm · 천장 그대로. `6bc1ae021f` 11.1 cm → `"xy"`, 메쉬 md5 가 전과 같음.
  - 촬영 원본 찾기(`stray_mesh.find_capture`, 풀지 않음): `capture=` 를 주면 그것만. 아니면 `input_map/_unzip/<stray_meta.json 의 source>` → `_unzip` 아래 3겹 안의 `<source>` 폴더(macOS 가 묶은 zip 의 `outer/<source>` 등, `__MACOSX` · `.` 로 시작하는 이름은 건너뜀) → `input_map/_unzip` 그 자체. 폴더 이름 · `odometry.csv` 줄 수가 맞아야 함. (`stray_quality._capture` 와 달리 `capture=` 가 맨 앞이다.)
  - 크기 한도: 재는 프레임 간격 = max(0.1 초, 촬영 초 / 3000) — 아주 긴 촬영도 3,000장 안(약 1.4 GB · 20 초 [추정]). 쓴 값은 `auto.every_s`. 못 찾거나 재다 오류가 나면 `"xy"` 와 이유를 남기고 빌드는 계속한다. 엔진은 입력 단계가 찾은 촬영 원본을 `capture=` 로 넘긴다 (`stages.py:70`, C `13d1c4d` — 통합 기록 R20) — 그래서 촬영 폴더를 zip 이 아니라 폴더로 넣은 실행도 잰다(11/25 까지는 `_unzip` 이 없어 재지 않고 `"xy"`). `--resume` 으로 변환을 건너뛴 실행은 `capture=<입력>` 이다. 서버는 zip 만 받으므로 웹앱에서는 늘 잰다.
  - 기록: `build/pose_fix.json` 과 `tsdf_meta.json` 의 `pose_fix` 에 `auto = {chosen, threshold_cm, measured, source, every_s, verdict, pairs, reliable, z_max_cm, h_max_cm, reason, seconds (, unused_opts)}` (auto 일 때만). `fix` · `name` 은 고른 이름이라 `stray_quality` · sha256 은 그대로. `tsdf_meta.json` 의 `timing.auto_s`.
  - 높이 단계를 켰으면 `flags` 맨 앞에 한 줄 — 입력 단계 주의로 보인다(6-3): "되돌아온 자리의 높이 어긋남 44 cm (> 15 cm) — 높이 보정을 켜서 합침. 옆으로 어긋난 벽(최대 123 cm)은 남을 수 있음"(뒷문장은 수평 어긋남이 10 cm 넘을 때만). 높이 단계가 거부 · 오류면 "… 켰지만 고치지 못함 (…) — 바닥이 두 높이로 찍혔을 수 있음", 건너뛰면 "… 고칠 것이 없다고 나옴 — 바닥이 두 높이로 찍혔는지 눈으로 확인".
  - `python -m scan2sim.stray_fix <맵 폴더> [--fix …] [--tilt] [--yaw] [--json <경로>]` 는 재기만 한다 (맵 · 메쉬는 그대로).
- 단계마다 관문이 따로다. 관문에 걸린 단계는 자세를 그대로 두고 이유를 남긴다. 보정 중 오류가 나도 메쉬는 만든다 (그 단계 status `"error"`).
- 같은 입력이면 같은 결과다 (난수 없음, 스레드 결과는 순서대로 모음).

**수평 보정** (`stray_fix.xy_fix`, 단계 `kind "xy"` · `method "wall_cells"`) — 키프레임 자세의 수평 위치만 고친다 (높이 · 방향은 그대로).

- 방의 벽 방향: 키프레임마다 수직 면(|법선 z| < 0.15 · 깊이 3 m 안)의 법선 각을 90° 로 접은 넓이 가중 막대그림 → 서로 직각인 두 축 u · v. 확신 값: `r4` (맨해튼이면 1 근처) · `aligned` (축 ±6° 안 넓이 비율) · `peak2` (축 ±10° 밖 가장 큰 봉우리 / 축 봉우리) · 축마다 넓이 몫 `share_u` · `share_v` (`stray_fix.manhattan`).
- 벽 칸: 법선이 ±u (또는 ±v)에서 16° 안인 점을 (벽을 따라 20 cm · 높이 20 cm · 벽 앞뒤 25 cm) 칸으로 묶어 키프레임마다 벽 자리를 잰다 (`stray_fix.wall_obs`).
- `벽 자리 = 키프레임 오차 + 칸의 실제 자리` 를 u · v 따로 푼다 (Huber 2 cm · 시간 이음 0.02 m/√s · 근거 있는 키프레임의 중앙 = 0). 고친 자세 = 카메라 위치 − (x_u·u + x_v·v).
- 방향(yaw) 오차는 기본으로 풀지 않는다 (`yaw=False`, 기준 촬영본에서 이득 없음).
- 관문 (`stray_fix.XY_GATES`)
  - 전부 거부(`refused`): 벽이 직각인 두 방향으로 뚜렷하지 않음 — `r4 < 0.5` · `aligned < 0.5` · `peak2 > 0.5` 중 하나 (둥근 · 육각형 · 어수선한 방).
  - 그 축만 그대로: 그 축 벽 넓이가 수직 면의 8% 미만 · 벽을 다른 키프레임과 10칸 넘게 같이 본 키프레임이 20% 미만 · 보정이 40 cm 넘음 · 고친 뒤 벽 자리 잔차 RMS 가 3 cm 넘음. 두 축 모두 걸리면 `refused`.
  - 건너뜀(`skipped`): 쓴 축 모두 보정 폭(2~98%)이 2.5 cm 미만 — 고칠 것 없음.
  - 표시(`flags`, 고치지는 않음): 벽을 못 본 구간 10 초 넘음 · **수평 보정 값**이 앞뒤와 10 cm 넘게 다르거나 1 초 안에 10 cm 넘게 바뀐 키프레임(원래 자세가 튄 것이 아님 — 원래 자세가 고르게 틀어진 `922acd4755` 에서도 29장. 11/25 문구 고침: "추적이 깨졌다 이어진 것" → "추적이 깨졌거나 벽 관측이 흔들린 자리일 수 있음"). 원래 자세가 튄 곳은 `stray_revisit` 의 `jumps`.
  - 보정이 40 cm 를 넘어 거부할 때의 문장도 "틀어짐이 아니라 추적이 깨진 것으로 봄" → "이 보정이 맡기엔 큼 (긴 왕복 촬영에 쌓인 틀어짐이거나 추적이 깨진 것)" 으로 고침(높이 단계도 같음).

**높이 보정** (`stray_fix.floor_fix`, 단계 `kind "floor"` · `method "floor_cells"`) — `fix` 에 `floor` 가 있을 때만. 키프레임 자세의 높이만 고친다.

- 키프레임마다 위를 향한 면(법선 z > 0.9 · 카메라보다 0.3~2.6 m 아래 · 깊이 3 m 안)을 20 cm 칸으로 나눠, `칸 높이 = 자세 높이 오차 + 칸의 실제 높이` 로 푼다 (Huber 2 cm · 시간 이음 0.02 m/√s). 고친 카메라 z = z − dz.
- 기울기(`tilt`): 높이 단계를 쓸 때 기본 켬. 키프레임마다 바닥 기울기 오차 (gx, gy) 도 풀어 카메라 중심 둘레로 돌린다. 사전 σ 1° · 시간 이음 0.5°/√s — 키프레임 사이 시간의 제곱근에 비례한다. `595d439` 까지는 키프레임 사이마다 0.5° 라 키프레임이 촘촘하면 사전이 약했다.
- 관문 (`stray_fix.GATES`): 거부 = 바닥을 다른 키프레임과 10칸 넘게 같이 본 키프레임이 30% 미만 · 보정이 40 cm 넘음 · 고친 뒤 잔차 RMS 3 cm 넘음. 건너뜀 = 보정 폭(2~98%) 1.5 cm 미만.
- 기본에서 뺀 까닭: `hf` 규칙 · 5 m 로 다시 재 보니 바닥 이득이 작고 천장을 나쁘게 했다 (`stray_fix.py` 머리말 "측정"). 천장이 없거나 바닥 턱이 중요한 촬영은 `--fix floor+xy`.

**결과 파일**

- **`build/pose_fix.json`** — 주 info 파일. `stray_mesh.build` 가 늘 쓴다 (보정을 꺼도 status `"off"`). 묶음 info (`stray_fix.chain_info`):

| 필드 | 뜻 |
|---|---|
| `method` · `version` | `"chain"` · 1 |
| `fix` | `"xy"` · `"floor+xy"` · `"floor"` · 끄면 `"none"` |
| `order` | 돈 단계 차례 (예: `["xy"]` · `["floor", "xy"]`, 끄면 `[]`) |
| `applied` | 한 단계라도 자세를 고쳤나 |
| `status` | `applied` · `skipped` · `refused` · `error` · `off`. 아무 단계도 고치지 않았으면 단계 status 중 error > refused > skipped > off 차례로 |
| `reasons` · `flags` | 단계 이름("높이: " · "수평: ")을 앞에 붙여 모은 문장 |
| `keyframes` | 키프레임 수 |
| `seconds` | `{floor, xy, total}` (돈 단계만) |
| `steps` | 단계 info 목록 (차례대로) |

  - 단계 info 공통: `method` · `version` · `kind` · `params` · `keyframes` · `applied` · `status` · `reasons` · `flags` · `t` (키프레임 시각) · `seconds{observe, solve, total}`.
  - 수평 (`kind "xy"`): `axes_applied` · `manhattan{yaw_deg, r4, aligned, peak2, share_u, share_v, vertical_area_m2, frame_yaw_deg_p5_p95, frame_yaw_noise_deg, frames_with_walls_pct}` · `axes{u, v: applied, share, supported_pct, cells, observations(_dropped), spread_before_cm, resid_rms_cm, resid_med_cm, range_cm, max_abs_cm, p5_p95_cm, max_unsupported_s, jump_frames, reasons}` · `dx` · `dy` (세계 m, 카메라 위치에서 뺀 값 — 고치지 않았으면 0) · `du` · `dv` (축 성분) · `du_estimate` · `dv_estimate` (풀었지만 고치지 않았을 때만) · `dyaw_deg` (yaw 를 쓸 때만, 아니면 null) · `yaw_fit` (yaw 를 쓸 때만) · `shift_cm_p50_p95`.
  - 높이 (`kind "floor"`): `dz` (고치지 않았으면 0) · `dz_estimate` (고치지 않았을 때만) · `tilt` (키프레임마다 (gx, gy), 끄거나 고치지 않았으면 null) · `tilt_deg_p50_p95` · `tilt_max_deg` · `tilt_big_frames` · `frames_supported(_pct)` · `cells` · `observations(_dropped)` · `resid_rms_cm` · `resid_med_cm` · `dz_range_cm` · `dz_p5_p95_cm` · `dz_max_abs_cm` · `max_unsupported_s` · `jump_frames` · `jump_times` · `levels{…}`.
  - 관문 전에 멈춘 단계(키프레임 2장 미만 · 시각이 차례대로가 아님 · 근거가 없음)는 수치 키 일부가 없다. 오류 단계는 `method · version · kind · applied false · status "error" · reasons · flags` 만 있다 (`stray_fix.not_applied`).
- **`input_map/pose_fix.json`** — 같은 바이트의 복사본. 있어도 없어도 된다 (예전에 맵 폴더에서 읽던 곳을 위한 것). 맵 폴더에 쓸 수 없으면 건너뛰고 로그에 한 줄 남긴다. 쓸 때 맵 폴더의 `floor_fix.json` 을 지운다 (`stray_mesh._copy_to_map`).
- **`input_map/floor_fix.json`** — `af65f1b` 판(높이 단계 하나의 info)으로 만든 예전 맵에만 있다. 지금 코드는 쓰지 않고, `stray_quality` 가 예전 빌드에서만 읽는다.
- **쓰는 차례** (`stray_mesh.build`): 합치기 → `tsdf_meta.json` → `mesh_raw.ply` → `build/pose_fix.json` → (되면) `input_map/pose_fix.json`. 모두 임시 이름으로 쓰고 바꿔 놓는다. 합치기 전에는 아무것도 쓰지 않는다.
  - 그래서 `mesh_raw.ply` 가 있으면 `tsdf_meta.json` 도 있다. 중간에 멈춰 `pose_fix.json` 이 예전 것으로 남으면 sha256 이 달라 `stray_quality` 가 적용하지 않는다.
  - 예전 `build/mesh_far_raw.ply` 가 있으면 지운다 (원거리 볼륨 없음).

**`build/tsdf_meta.json`** (`stray_mesh.build`) — `pipeline/tsdf.py` 와 같은 키 (`tsdf` 설정 · `frames_total` · `frames_used` · `seconds` · `vertices` · `triangles` · `far_triangles` · `up_hint_camera`, `config` 는 null) + `builder` (`"scan2sim.stray_mesh"`) · `map_root` · `poses_rows` · `frames_skipped` · `frame` · `timing{read_map_s, auto_s (auto 가 아니면 0), pose_fix_s, integrate_s, read_wait_s, extract_s, write_s, total_s}` · `peak_rss_mb` · `pose_fix`.

- `tsdf.depth_trunc` = 5.0 (`stray_quality` 가 읽음).
- `pose_fix` = 요약 (`stray_fix.summary` + 옮긴 양):
  - `name` (보정 이름, 끄면 null) · `file` (`"pose_fix.json"` — 빌드 폴더 기준) · `sha256` (그 파일 바이트의 sha256) · `method` · `fix` · `order` · `applied` · `status` · `reasons` · `flags` · `keyframes` · `seconds` · `steps` (단계마다 요약 — 키프레임별 목록은 뺌).
  - `moved_m` · `moved_xy_m` · `moved_z_m` = 카메라 위치가 옮겨진 거리 `{median, max}` (m), `turned_deg` = 방향이 돈 각 `{median, max}`.
  - 높이 단계 요약: `method` · `kind` · `applied` · `status` · `reasons` · `flags` · `keyframes` · `frames_supported_pct` · `dz_p5_p95_cm` · `dz_max_abs_cm` · `resid_rms_cm` · `max_unsupported_s` · `stand_other_pct` · `jump_frames` · `tilt_deg_p50_p95` · `tilt_max_deg` · `tilt_big_frames` · `seconds`.
  - 수평 단계 요약: `method` · `kind` · `applied` · `status` · `reasons` · `flags` · `keyframes` · `axes_applied` · `shift_cm_p50_p95` · `yaw_fit` · `seconds` · `manhattan{yaw_deg, r4, aligned, peak2, share_u, share_v}` · `axes{u, v: applied, share, supported_pct, p5_p95_cm, max_abs_cm, resid_rms_cm, max_unsupported_s, jumps}`.

실제 값 (`ref` 의 `build/tsdf_meta.json` 의 `pose_fix`, 문장 · 단계 키 줄임):

```json
{"name": "xy", "file": "pose_fix.json", "sha256": "f86f42f6…", "method": "chain", "fix": "xy", "order": ["xy"],
 "applied": true, "status": "applied", "reasons": [],
 "flags": ["수평: u 축(벽 법선 90°): 벽을 못 본 구간이 최대 16 초 — …", "수평: v 축(벽 법선 180°): 자세가 앞뒤와 10 cm 넘게 튀거나 … 키프레임 5장 (첫 장 51.2 초, …) — …"],
 "keyframes": 1133, "seconds": {"xy": 2.34, "total": 2.34},
 "steps": [{"method": "wall_cells", "kind": "xy", "applied": true, "status": "applied", "axes_applied": ["u", "v"], "shift_cm_p50_p95": [5.5, 14.0],
            "manhattan": {"yaw_deg": 89.551, "r4": 0.876, "aligned": 0.844, "peak2": 0.031, "share_u": 0.282, "share_v": 0.561},
            "axes": {"u": {"applied": true, "supported_pct": 31.0, "p5_p95_cm": [-9.4, 3.8], "max_abs_cm": 15.3, "resid_rms_cm": 1.78, "…": "…"},
                     "v": {"applied": true, "supported_pct": 50.6, "p5_p95_cm": [-13.1, 5.5], "max_abs_cm": 19.8, "resid_rms_cm": 1.35, "jumps": 5, "…": "…"}},
            "…": "…"}],
 "moved_m": {"median": 0.0547, "max": 0.224}, "moved_xy_m": {"median": 0.0547, "max": 0.224}, "moved_z_m": {"median": 0.0, "max": 0.0},
 "turned_deg": {"median": 0.0, "max": 0.0}}
```

**보정 전 자세와 다시 적용하기**

- `trajectory_camera_optical.txt` 는 **보정 전** 자세다. 고친 자세가 필요하면 `stray_fix.apply_fix(times, poses, info)` (info = `build/pose_fix.json` 내용).
  - 단계 차례대로, applied 인 단계만 적용한다. 키프레임 시각 사이는 선형으로 이으므로 키프레임이 아닌 프레임에도 쓸 수 있다 (키프레임 시각 밖은 처음 · 끝 값).
  - 예전 `floor_fix.json` (단계 하나의 info)도 받는다. applied 인데 어떤 보정인지 모르는 info 면 `ValueError`.

**`stray_quality` 와의 관계** (`stray_quality._pose_fix` · `_rule` · `_depth`, `measure` · `drift` 같음)

- **진단의 광선 구조 넘겨받기 (C5, 11/25 B)**: `stray_quality.measure(out_dir, …, visual_scene=None)`.
  - `visual_scene` = `(build/visual.ply 경로, 그 파일의 mtime, RaycastingScene)` 세 개 묶음 — 진단 단계의 `ctx["_vis_scene"]` 모양 그대로.
  - 조건: 그 광선 구조에는 `build/visual.ply` 하나만 씬 좌표 그대로 넣었어야 한다. 보정 판(`floor_patch.ply`) 등 다른 형상을 더했으면 `None` 을 넘긴다. 지금 진단은 판을 더한 구조면 `_vis_scene` 을 `None` 으로 두므로 그대로 넘기면 된다.
  - `measure` 는 경로가 같은 파일이고 mtime 이 같을 때만 그 구조로 `meshes.visual` 을 재고, 아니면 전처럼 파일을 읽어 새로 만든다. 결과 dict 는 넘기든 안 넘기든 같다 (`seconds` 만 다름).
  - 주 수치(`mesh_raw` — 진단 수치 `stray_quality` 의 값들)는 늘 `build/mesh_raw.ply` 로 새로 만든 구조로 잰다. `visual.ply` 는 씬 좌표로 옮기고 (천장이 없으면) 떠 있는 조각을 지운 메쉬라 형상이 달라, 진단 구조로 재면 "복원 정확도"의 뜻이 바뀐다.
  - B 측정(11/25, B 노트북): `measure` 융복 2층 2.69 → 1.49 s · 인문대 2층 12.85 → 6.14 s, 두 촬영본 모두 결과 값 503개가 같음. 남은 시간은 거의 `mesh_raw` 의 PLY 읽기와 BVH 만들기(약 1.0 s / 4.7 s).
  - 엔진이 넘긴다: `stray_quality.measure(…, visual_scene=ctx.get("_vis_scene"))` (`stages.py:389`, C `13d1c4d` — 통합 기록 R14). `_vis_scene` 을 만드는 자리(`stages.py:289-290`)에 "visual.ply 말고 넣지 말 것" 주석이 있다.

- 보정 info 찾기
  - 새 빌드 (`tsdf_meta.json` 의 `pose_fix` 에 `sha256` 이 있음): `pose_fix.applied` 가 거짓이면 읽지 않는다. 참이면 `build/<file>` → `input_map/<같은 이름>` 차례로 찾아 **파일 sha256 이 같은 것만** 적용한다. 같은 것이 없으면 적용하지 않고 `notes` 에 문장을 남긴다.
  - `sha256` 이 없는 예전 빌드 (`595d439` 까지의 `stray_mesh` 와 `af65f1b` 판): `input_map/pose_fix.json` → `input_map/floor_fix.json` 차례. `tsdf_meta.json` 이 보정 없이 만든 맵이면 (`pipeline/tsdf.py` · `--no-fix`) 적용하지 않는다.
  - 적용하면 안 쓴 프레임 자세에도 같은 보정을 건다 (`apply_fix`).
- 안 쓴 프레임의 깊이는 맵의 `depth_rule` 을 그대로 쓴다 (`stray.depth_keep`). `hf` 의 높이 조건은 **보정 전 자세**와 `camera_matrix.csv` 한 값을 깊이 크기로 줄인 내부 파라미터로 잰다 (`stray.py` 와 같게 — `--k-source` 와 상관없음). `no_confidence` 맵은 3 m 까지. `depth_rule` 이 없는 예전 맵이나 `min_conf` 를 주면 신뢰도 기준만 쓴다.
  - 깊이 한계 = min(`tsdf_meta.json` 의 `tsdf.depth_trunc`, `select.max_depth_m`). `tsdf_meta.json` 이 없으면 5 m (`depth_rule` 이 없는 예전 맵은 3 m). 지금 엔진(`stray_mesh.build`)은 5 m (11/24 까지의 `pipeline/tsdf.py` 는 3 m).
- 안 쓴 프레임 고르기 (`held_out.method "time_grid"`): 촬영 시간(처음 1 초 뒤 ~ 끝)을 n 구간으로 나눠 구간마다 목표 시각(seed 0 = 가운데)에 가장 가까운, 가장 가까운 키프레임에서 0.1 초 넘게 떨어진 장. 그런 장이 없는 구간만 기준을 낮춘다 (`held_out.relaxed_cells`, `notes`). 키프레임 수가 바뀌어도 같은 구간 · 거의 같은 시각을 잰다.
- **`input_map/stray_quality.json`** 의 관련 키
  - `pose_fix`: `{applied, status, file, steps: [{kind, applied, status}], (높이) dz_p5_p95_cm · dz_max_abs_cm, (수평) xy_axes · xy_shift_cm_p50_p95 · xy_shift_max_cm, (적용 못 했으면) note}`. 새 빌드면 `file` = `"build/pose_fix.json"`.
  - `depth_rule` (규칙 이름, 규칙 없이 재면 `"min_conf N"`) · `depth_limit_m` · `depth_trunc_m` · `max_depth_m` · `min_conf` · `min_conf_build` · `intrinsics` · `intrinsics_build`.
  - `held_out`: `{method "time_grid", min_gap_s, min_gap_s_used, relaxed, relaxed_cells, candidates_gap_s, gap_s, gap_frames, target_offset_s, nearest_keyframe_cm}`.
  - `drift` 결과(따로 부를 때만)도 `pose_fix` · `depth_rule` 을 가진다.
  - 실제 (`ref`): `"pose_fix": {"applied": true, "status": "applied", "file": "build/pose_fix.json", "steps": [{"kind": "xy", "applied": true, "status": "applied"}], "xy_axes": ["u", "v"], "xy_shift_cm_p50_p95": [5.5, 14.0], "xy_shift_max_cm": 22.4}` · 안 쓴 프레임 30장 중앙 1.78 cm · 90% 8.21 cm · 맵에 닿음 99.2% → `pass`.

B 측정 (기준 촬영본, 통합 기록 11/24 저녁 · R10): 자세 보정 조합 비교(공통 안 쓴 프레임 60장)에서 수평만 = 바닥 두 겹 3.97% · 천장 두 겹 4.0% · 안 쓴 프레임 중앙 / 90% 1.64 / 8.01 cm, 높이(기울기, `595d439` 의 사전) + 수평 = 6.01% · 14.6% · 1.33 / 6.92 cm → 기본은 수평만. R10 을 넣은 복사본 엔진 전체: `floor_two_levels` 15.8 → 2.7% · 결함 6 → 1 · 복원 정확도 중앙 2.69 → 1.78 cm.

### 6-5. 되돌아온 자리 어긋남 (B `stray_revisit.py`, C4 · 11/25)

같은 곳을 다른 때 다시 본 깊이(벽 · 바닥)끼리 맞춰, 그 사이에 쌓인 자세 틀어짐을 직접 잰다. 안 쓴 프레임 정확도(`stray_quality.measure`)는 키프레임 바로 옆 장과 견주어 이것을 못 본다(복도 왕복 `922acd4755` 중앙 6.13 cm — 주의에 그침). **엔진은 진단 단계에서 부른다** (C `13d1c4d`, 통합 기록 R19 — 진단 수치 `revisit` 은 3-5, 주의 문장 · 판정은 5-4 "되돌아온 자리 어긋남"). 입력 단계의 자동 고르기(6-4)도 `raw` 만 따로 잰다.

- 부르기: `stray_revisit.measure(path, fix_info=None, write=True, every_s=0.1, capture=None)` → dict. `path` = 촬영 폴더 · zip · scan2sim 결과 폴더(`input_map/` · `build/`). 결과 폴더에 `input_map/_unzip` 이 없으면(서버가 끝난 작업에서 지움 · 폴더 입력) `capture=` 로 촬영 폴더나 zip 을 준다 — `stray_quality.measure` 와 같이 `stray_quality._capture` 로 찾고, zip 이면 `input_map/_unzip` 에 푼다. CLI `python -m scan2sim.stray_revisit <path> [--capture] [--fix-json] [--json] [--pairs] [--no-write]`.
- 촬영 자료가 잘못됐으면 `stray.validate` 의 문장으로 `stray.CaptureError`, 직접 준 보정 info 가 잘못됐으면 `ValueError`, 촬영본을 못 찾으면 `SystemExit`(문장). CLI 는 "[되돌아온 자리] 실패: …". zip 은 임시 폴더에 풀고 실패해도 지운다.
- 결과 폴더면 맵을 만든 보정을 `stray_quality._pose_fix`(sha256 이 맞을 때만)로 읽어 **원래 자세(`raw`)와 고친 자세(`fixed`)를 둘 다** 재고, `input_map/stray_revisit.json` 에 쓴다(임시 이름 → 바꾸기. `stray.STALE` 에 들어 있어 다시 변환하면 지워짐). 판정은 `fixed` 가 있으면 그것.
- 결과 dict: `{source, frames, every_s, params, jumps, jumps_total, raw, fixed?, pose_fix?, verdict, reasons, seconds}`. `raw` · `fixed` = `{summary, verdict, reasons, pairs, floor_track}`.
  - `pose_fix`(결과 폴더면 늘): `_pose_fix` 요약 + `sha256_expected`(`tsdf_meta.json` 의 값) · `sha256_used`(맞은 파일의 값, 안 맞으면 `null` — 그때는 `fixed` 가 없고 까닭이 `reasons` 에 "자세 보정: …"으로 들어감). `--fix-json` 으로 줬으면 `file` = "(fix_info 로 받음)".
  - `summary`: `pairs` · `reliable`(믿을 쌍 수) · `duration_s` · `h_max_cm` · `h_p50_cm` · `h_worst {t_a, t_b, path_between_m, h_axis}` · `z_max_cm` · `z_p50_cm` · `z_worst {dz_cm, t_a, t_b, path_between_m}` · `yaw_max_deg` · `path_between_max_m` · `unreliable_path_max_m` · (10 m 넘게 떨어진 쌍이 있으면) `h_rate_cm_per_10m` · `z_rate_cm_per_10m`.
  - `pairs[]`: 뒤 방문 − 앞 방문(맵 세계 Z 위) `dx_cm · dy_cm · dz_cm · h_cm · dyaw_deg`, `h_axis` (`"2d"` 또는 긴 복도처럼 길 방향을 못 잰 `"1d"` — 옆 성분만), `fit`, `reliable`, `unreliable_why`, `t_a` · `t_b` · `path_between_m` · `cam_b` · `frames`(실제로 쓴 장 수).
  - `jumps`: 원래 자세가 튄 곳(한 장 사이 간격으로 나눠 10 cm · 5° 넘음, 앞 20곳) · `jumps_total`. 판정에 안 씀. `floor_track`: 카메라 아래 바닥 높이의 10 초 중앙(판정에 안 씀).
  - 좌표: `at_xy` · `cam_b` · `floor_track` 은 **맵 폴더 세계**(입력 좌표, `trajectory_camera_optical.txt` 와 같음)다. 장면 좌표가 아니므로 지도 · 3D 에 그리려면 `build/transform.json` 의 `T_input_to_scene` 을 곱한다.
- 판정(믿을 쌍 중 가장 큰 값, 나쁜 쪽을 따름 — 촬영본 두 건 + 가상 방으로 잡은 잠정값): 높이 |dz| 3 cm 이하 `pass` · 15 cm 넘으면 `fail` · 그 사이 `warn`. 수평 10 cm 이하 `pass` · 30 cm 넘으면 `fail`. 다시 지나간 곳이 없으면 `skip`(점이 있는 장이 적음 · 촬영이 20 초 이하 · 20 초 넘게 떨어져 다시 본 구간이 없음 — 까닭을 나눠 적음), 믿을 쌍이 없으면 `warn`. `pass` 라도 믿지 못한 쌍이 믿을 쌍 중 가장 먼 것보다 3 m 넘게 멀리 떨어져 있거나 높이를 견준 믿을 쌍이 없으면 `warn` 으로 낮춤(먼 쌍을 못 맞춰 작은 값만 남는 것을 막음).
- 시간 · 메모리: 2~17 초 · 0.9 GB (B 노트북, 두 촬영본).
- B 측정(11/25, 믿을 쌍 중 최대 — 원래 자세 → 기본 보정 xy → floor+xy): `922acd4755` 수평 123 → 125 → 125 cm · 높이 44 → 44 → 1.2 cm(모두 `fail`), `6bc1ae021f` 수평 24 → 10 cm · 높이 11 → 11 cm(`warn`). 알려진 어긋남을 넣어 되찾음: 높이 ±0.7 cm · 옆 ±1.4 cm 안.

---

## 7. 한도 (구현된 값)

| 항목 | 값 | 위치 |
|---|---|---|
| 받는 확장자 | `.zip` · `.glb` (대소문자 무시). 화면도 같은 검사. 예시 파일도 같음 | `web/server.py:51`, `:301-303`, `:236` · `index.html:95`, `:162` |
| 파일 크기 | 1 바이트 ~ `MAX_BYTES` = 4 × 1024³ = 4,294,967,296 바이트. `/api/info` 의 `max_bytes` | `web/server.py:52`, `:304-305`, `:254` |
| 조각 크기 | 화면이 8 MiB(8,388,608 바이트)로 나눔. 서버는 조각 크기를 제한하지 않고 1 MiB 씩 읽음. `waitress` 의 요청 본문 한도도 4 GiB | `index.html:144` · `web/server.py:328`, `:582` |
| 조각 다시 보내기 (화면) | 8번까지, 1 · 2 · 4 … 초 (최대 30초) 기다림 | `index.html:144`, `:187-189` |
| 작업 번호 | `^[0-9a-f]{12}$` | `web/server.py:64` · `index.html:372` |
| 파일 이름 저장 길이 | 앞 120자 | `web/server.py:311` |
| 예시 파일 이름 | `[A-Za-z0-9][A-Za-z0-9._-]{0,79}` (80자까지). 맞지 않는 파일은 목록에서 빠짐 | `web/server.py:54`, `:236` |
| 동시 처리 | 처리 스레드 **하나** (한 번에 한 작업, 나머지는 줄). 요청 받기는 `waitress` 스레드 16 · 쉬는 연결 300초 (`channel_timeout`), 없거나 `--dev` 면 Flask 내장 서버 `threaded=True` | `web/server.py:572`, `:582`, `:584` |
| 기다리는 작업 (`--max-wait`) | `queued` 가 5건이면 새 작업을 429 로 거절. 처리 중 1건은 세지 않음. 0 이면 끔 | `web/server.py:57`, `:182-184`, `:556` |
| 디스크 여유 (`--min-free-gb`) | 작업 폴더 디스크 여유 − 알린 크기 < 20 GiB 면 새 작업을 507 로 거절. 0 이면 끔. 배포 스크립트도 20 | `web/server.py:57`, `:185-186`, `:557` · `deploy/common.sh:14` |
| 서버의 zip 입력 검사 | 항목 10만 개 · 풀린 크기 6 GiB (`stray.py` 와 같은 값). GLB 는 앞 4바이트 `glTF` | `web/server.py:53`, `:190-208` |
| 처리 제한 시간 (`--timeout-min`) | 20분. 넘으면 엔진과 하위 프로세스를 죽이고 `failed` · `timed_out`. 0 이면 끝날 때까지 기다림 | `web/server.py:57`, `:390-403`, `:555` |
| 서버를 다시 켤 때 다시 넣기 | 작업마다 한 번 (`restarts` 1). 두 번째는 `failed` | `web/server.py:97-101` |
| 작업 폴더 정리 | 끝난 작업마다 `out/input_map/_unzip` 을 지움. 최근 `--keep` 건(기본 5)보다 오래된 끝난 작업은 `input/` · `out/input_map` · `out/build` 도 (2-1) | `web/server.py:109-129`, `:553` |
| 청소 (`--ttl-hours`) | 끝난 지 24시간 · 마지막 조각 뒤 24시간 멈춘 올리다 만 작업의 폴더를 통째로 지움. 켤 때 · 작업 끝 · 10분마다. 0 이면 끔 (2-1) | `web/server.py:57`, `:132-164`, `:426`, `:558` |
| 기본 주소 | `127.0.0.1:8000`. 밖에 열 때는 `--code` 를 줄 것. 배포 스크립트는 늘 127.0.0.1 + 터널 + 접속 코드 (4장) | `web/server.py:549-550` · `deploy/start.sh:45` |
| 결함 수 | 파일마다 최대 30 | `stages.py:184` |
| 화면 폴링 | 1.5초 | `index.html:216` |
| 화면용 GLB | 정점 색 삼각형 30만 · 텍스처 긴 변 2048 화소 | `preview.py:46` |
| Stray 키프레임 | 최대 2,000장 (넘으면 간격을 넓힘) | `stray.MAX_KEYFRAMES` |
| Stray 깊이 거리 | 신뢰도 2 는 5 m · 신뢰도 1 은 3 m (`hf`). 엔진 메쉬도 5 m 까지 (R10) | `stray.convert` 기본값 · `stray_mesh.build` 기본값 |
| Stray 자세 보정 | 키프레임 보정이 40 cm 넘으면 그 단계 · 축을 거부 | `stray_fix.GATES` · `XY_GATES` |
| 보이는 바닥 구멍 덮기 (R9) | 구멍 하나 4 m² 까지. 바닥 아래에서 맞은 칸 10% 이하 · 둘레 띠의 보이는 바닥 50% 이상 | `stages.py:397` (`_patch_mask` 인자) |
| 걸어간 길 (Stray) | 막힌 길 10% 초과 또는 이어진 길 80% 미만이면 판정 `fail`. 막힌 길 2% 초과 · 빈 바닥 가장 큰 덩어리 60% 미만은 주의. 막힘 봐주기 0.15 m | `stages.py:351-357` · `checks.py:48` |
| Stray zip | 파일 10만 개 · 풀면 6 GB | `stray.ZIP_MAX_FILES` · `ZIP_MAX_BYTES` |
| 충돌 메쉬 물체 삼각형 (GLB · Stray) | 15만 (11/26 C `ecaae53`, 전 10만 — 통합 기록 R22). 스캔 바닥은 줄이기 전에 빼고 평평한 바닥 한 장을 따로 깔아 바닥은 이 수에 안 듦. 목표의 5배를 넘으면 먼저 3 cm 정점 묶기 | `detect.config_glb` (`collision.tris`) · `pipeline/glb_to_usd.py` 충돌 단계 |
| 제자리 회전 가능 넓이 | 5 m² 미만이면 주의 (판정은 그대로) | `stages.py:363` |
| 엔진 쪽 실패 기준 (참고) | GLB · Stray 범위 1 m 미만 · 300 m 초과, 바닥 2 m² 미만 · Nav2 도구 없음 · 빈 바닥 4 m² 미만 · 진단 표본 50개 미만 | `stages.py:78-81`, `:154-156`, `:168-169`, `:259-260` |

---

## 8. 계획과 다른 점

계획 = `docs/implementation.md` 5장 (2판, 11/23). 위가 코드에서 확인한 것이다. 11/24 · 11/25 에 C · D 가 고친 것은 **고침** 으로 적고 커밋을 붙였다.

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
   - 서버는 작업이 끝나면 `_unzip/` 을 지우고, 최근 5건보다 오래된 작업은 `input/` · `input_map/` 도 지운다 (D `533bffe`, 2-1). 최근 작업의 `keyframes/` 와 올린 zip(`input/`)은 남는다. CLI 는 지우지 않는다.
   - 11/25 저녁 (D `9b2c95f`) 부터 끝난 지 24시간이 지난 작업은 폴더째 지운다 (청소, 2-1). 계획 10장의 "24시간 뒤 지움" 이 서버에 들어갔다. 화면의 안내 문구(계획 6-A 1번)는 아직 없다.
7. **[고침 — D `98da703`, 통합 기록 R2]** 상태 `done` 이 "엔진이 성공함" 이 아니었다.
   - 지금은 `report.md` 가 있고 종료 코드 0 이고 실패 단계가 없어야 `done` 이다 (4-3).
   - 11/23 의 `e2e_glb` 는 `done` · `exit_code 1` · `verdict fail` 이었다. 지금 규칙으로는 `failed`.
8. **404 응답은 `{error}` JSON 이 아니라 HTML 이다.** 계획 5-6 은 "오류는 `{error}`" 라고 적었다.
9. 상태 응답에 계획 예시에 없는 필드가 있다: `file` · `queued_at` · `error` · `restarts` (2-2).
10. **맵 폴더 zip 은 웹으로 올릴 수 없다.**
    - `web/server.py:7` 의 설명은 "맵 폴더를 묶은 zip" 을 받는다고 적었다.
    - 그러나 엔진의 `detect.kind_of` 는 Stray 가 아닌 zip 을 알아보지 못한다 (`detect.py:13-20`). 입력 단계에서 `"입력을 알아볼 수 없음"` 으로 실패한다. 서버의 입력 검사(4-2)는 zip 목록만 봐서 통과시킨다.
11. **웹으로 올린 작업은 장면 이름이 모두 `input` 이다** (2-4).
    - USD 가 `input_mesh.usdc` 가 되고, `scene_root.usda` 가 `data/usd/input/…` 을 가리키고, `bundle.json` 의 `name` 도 `input` 이다.
    - 여러 장소를 `sim/` 에 들이면 이름이 겹친다. 계획 5-2 는 "이름은 서버가 정함" 까지만 적었다.
12. **Stray 결과의 `bundle.json` `kind` 가 `glb` 다** (11/24, C `0534d95`). 묶음만 보고는 Stray 인지 GLB 인지 `input` 확장자로만 가린다 (2-4).

### 8-2. 계획에는 있고 아직 없는 것

- ~~`GET /api/samples`, `POST /api/samples/<name>/jobs`, `GET /api/health` (계획 5-6 "추가하기로 한 것", D 담당)~~ — 됨 (D `9b2c95f`, 4-2). 모양은 계획 5-6 그대로이고, `/api/health` 에 `running` · `free_gb` 가 더 있다. **화면의 예시 파일 단추는 아직** (A, 계획 6-A 2번).
- ~~서버 보호 (계획 6-D 서버 1 · 2 · 3 · 4 · 5 · 7번)~~ — 됨 (D `9b2c95f`, 4장 · 7장): 처리 제한 시간 20분 · 입력 검사 · 줄 길이 429 · 디스크 여유 507 · 24시간 청소 · 끝날 때 `bundle.zip` 미리 만들기 · `waitress`. 계획과 다른 것:
  - zip 풀린 크기 한도가 계획 4 GB 가 아니라 6 GB (`stray.py` 의 한도와 맞춤, `9b2c95f` 커밋 설명).
  - 줄 길이: 계획은 "기다리는 작업이 5건을 넘으면", 코드는 "5건이면" 거절 (`0 < max_wait <= n`, `web/server.py:183`).
  - 진행률(계획 6-D 8번, C 의 `out/progress.json`)은 아직 없다.
  - [D 측정, `9b2c95f` 커밋 설명] 서버를 네 번 띄워 37항목 통과. `tests/run.sh` 전체 · 브라우저 화면 · 터널 · Windows 의 제한 시간 처리는 돌려 보지 않았다. 그 뒤 `bef9cda` 에서 `tests/run.sh` 전체 `TESTS_OK` (117초, D 서버). B 도 main `bef9cda` 위에서 `TESTS_OK` (194초, B 노트북, `5db0573` 커밋 설명).
- ~~화면의 이어 올리기~~ — 됨 (D `533bffe`). 화면이 끊긴 조각을 서버가 받은 자리부터 다시 보내고, 서버를 다시 켜도 받은 자리를 파일 크기로 되찾는다 (4-2). 페이지를 다시 연 뒤에는 이어 올리지 않는다.
- `bundle/` 을 D 의 `sim/add_site.py` 형식에 맞추기 (계획 3-2 · 5-2). **zip 구조는 됨** (D `bef9cda`, 2-4 · 4-2) — 맨 위가 `bundle/` 하나라 `add_site.py` 가 내려받은 zip 을 그대로 받는다. 남은 것 (`docs/isaac_check.md` 2장, C · D 가 정할 것):
  - ~~씬 루트~~ — 됨 (C12, D `6e88feb`): `add_site.py` 가 묶음의 `scene_root_sim.usda` 를 받아 맵 경로 줄만 바꿔 쓴다 (2-4).
  - 이름: 묶음 안은 모두 `input` (8-1 의 11번). 들이는 데는 문제없고 놓인 폴더 이름은 `<장소>_<판>` 이 된다 (D 확인).
  - 시작 자세: 경로 없이 들이면 `add_site.py` 는 묶음의 `start_pose.json` · `route.csv` 를 쓰지 않고 지도에서 다시 잡는다. 실제 촬영본에서 위치는 같고 방향만 다름 (묶음 126.1° · `add_site.py` 90.0°, D 측정).
  - ~~두 판정~~ — 정함 (C13): 웹앱도 같은 수치(`drivable.turn_area_m2`)를 보고 5 m² 미만이면 주의 한 줄. 판정은 맵 상태만 (5-4).
  - Isaac Sim 에서 들인 장소로 주행해 본 것은 아직 없다 (`C_TRACK.md` C8).
- ~~B 의 Stray 형상 메쉬 단계가 엔진에 아직 없다~~ — 들어감 (R10 · R11, C `be8310e`). 엔진이 Stray 형상 메쉬를 `stray_mesh.build` 기본값(깊이 5 m · 수평 자세 보정)으로 만들고, 입력 단계가 `pose_fix` 를 문장 · 수치로 보여 준다 (6-2 ~ 6-4). `build/tsdf_config.yaml` 은 더 생기지 않는다. 이 문서의 "R10 뒤" 표시는 정리했다.
- ~~Stray 복원 정확도를 진단 · 보고서에 붙이기~~ — 붙음 (R3, C `620a2b0`). 진단 수치 `stray_quality` = `{frames, median_cm, p90_cm, within_2cm_pct, hit_pct, verdict}`, 요약 문장은 `messages`, 통과가 아니면 이유가 주의(실패여도 주의). `report.md` 에 "복원 정확도 (아이폰 원본)" 절. [C 측정] 방 촬영본 중앙 2.17 cm · 2 cm 안 47.2% · 맵에 닿음 98.8% → 통과
- **[고침 — C `736eb68`]** "같은 입력에 같은 결과" (계획 5-3). 11/23 판에는 "아직 보장하지 않음" 이었다.
  - GLB · Stray 의 바닥 평면 찾기에서 무작위 표본(RANSAC)을 없앴다. 화면용 GLB 도 무작위를 쓰지 않는다.
  - C 측정 (Windows): 복도 GLB 4번 · 가상의 방 Stray 2번에서 변환 행렬 차이 0 · 결함 목록 같음 (`C_TRACK.md` 1장).
  - D 측정 (Ubuntu 24.04 서버 · 32 코어, main `533bffe` = R10 포함 · `105591c` 전, `6afb822` 커밋 설명): 실제 촬영본 540 MB(융복 2층 `6bc1ae021f` 에 D 가 풀어 둔 `rgb/` 1,137장을 더 넣은 zip — 11/26 D 답. 엔진은 `rgb.mp4` 만 읽어 덧붙은 `rgb/` 는 결과에 닿지 않음)를 서버로 두 번 — 각 약 40초(처음부터 끝까지). `mesh_raw.ply` · USD · `map.pgm` · 결함 목록 · `preview.glb` · 시작 자세의 md5 가 같음. 11/24 오전 엔진 `98da703` 에서는 `transform.json` 부터 달랐다.
  - `tests/repeat.py` 가 계속 검사한다.

### 8-3. 버그로 보이는 것 (고칠 사람이 정할 것)

| # | 위치 | 내용 | 상태 |
|---|---|---|---|
| 1 | `core.py:53` + `pipeline/glb_to_usd.py:66`, `:88` (`detect.profile_glb` 경유, `detect.py:42-44`) | 단계가 `except Exception` 만 잡아서 `SystemExit` 이 새어 나간다. 이때 단계 파일도 `report.md` 도 없이 엔진이 끝난다. 그러면 서버 상태는 `failed`, 그 단계부터는 모두 `pending`, `verdict` 는 `"pending"` (#2) 이 된다. 예: 바닥을 못 찾는 GLB. **11/24 부터 Stray 도 해당한다** — 입력 단계가 형상 메쉬에 `profile_glb` 를 부르므로, 형상 메쉬에 위 향한 면이 거의 없으면 같은 길로 죽는다 [추정: 그런 촬영본으로 돌려 보지 않음]. (`stray.py` 쪽은 B 가 `CaptureError` 로 바꿔 고침, `ba960b4`) | **고침** C `6ec3237`: `core.run` 이 `SystemExit` 도 받아 그 문장을 요약으로 하는 `fail` 단계로 남긴다 (`core.py:53-56`, 3-1). 뒤 단계 `skip` · `report.md` · `report.json` 있음. [C 측정] 세로 벽 한 장뿐인 GLB: 입력 단계 실패 · 종료 코드 1 · 단계 파일 8개 |
| 2 | `web/server.py:468` | `report.json` 이 없으면 `verdict` 를 `report` 단계 상태로 대신하는데, 그 단계가 없으면 `"pending"` 이 들어간다. 화면은 그 글자를 그대로 보인다 (`job.verdict \|\| "fail"` 이라 `"pending"` 은 걸러지지 않음, `index.html:241-243`). 1번이 고쳐져 한때 엔진 프로세스가 죽은 경우에만 생겼다. **11/25 저녁 (D `9b2c95f`) 부터 다시 흔해졌다:** 입력 검사에 걸린 작업 · 처리 제한 시간에 걸린 작업 · 서버가 다시 켜져 끊긴 작업도 `"pending"` 이다 (4-3 의 ②~⑥). 이유는 `error` 에 있지만 **화면은 `job.error` 를 결과에 쓰지 않는다** (`index.html` 에서 `error` 를 읽는 곳은 업로드 다시 시도 `:192` 뿐). [B 확인, Flask 시험 클라이언트] 깨진 zip 예시 파일 작업 → `failed` · `verdict "pending"` · `error` 있음. 제안: 서버는 `report.json` 이 없는 `failed` 의 `verdict` 를 `"fail"` 로 (D), 화면은 `failed` 면 `error` 를 보이기 (A) | **서버는 고침** D `6ad7114` (`:471-474`, 4-4 `verdict`). **화면은 열림 (A)**: `error` 를 결과에 보이기 |
| 3 | `web/server.py:318-336` | 조각이 알린 크기를 넘으면 400 을 돌려주지만, 그 전까지 읽은 부분은 이미 파일에 붙은 채이고 `received` 는 그대로다. 같은 `offset` 으로 다시 보내면 파일이 어긋난다 | **고침** D `533bffe`: 조각마다 먼저 파일을 `received` 로 자른다 (`:326`). 같은 `offset` 으로 다시 보내면 남은 바이트가 지워진다 |
| 4 | `web/server.py:525` | `bundle.zip` 을 한 번 만들면 다시 만들지 않는다. 처리 중(`bundle` 단계 뒤 · `report` 전)에 요청하면 `report.md` 가 빠진 zip 이 남는다 | **줄어듦** D `9b2c95f`: 작업이 끝날 때 서버가 미리 만들므로(`:411-414`), 끝난 뒤 받는 흐름(화면 · `deploy/check.py` · `tests/e2e.py`)에서는 생기지 않는다. 처리 중에 API 로 직접 요청하면 여전히 생긴다 — 열림 (D, 낮음) |
| 5 | `web/server.py:298`, `:321` | `size` · `offset` 이 숫자가 아니면 `int()` 예외로 500 이 난다 (400 이 아님) | **`size` 는 고침** D `9b2c95f`: 400 `"name · size 를 읽을 수 없음"` (`:297-300`). **`offset` 도 고침** D `6ad7114`: 400 `"offset 은 숫자여야 함"` (`:321-324`) · `tests/e2e.py` 에 검사 한 줄 |
| 6 | `web/server.py:7` vs `detect.py:13-20` | 맵 폴더 zip 을 받는다는 설명과 실제 동작이 다르다 (8-1 의 10번) | 열림 |
| 7 | `recon/requirements.txt` | GLB 빌드가 쓰는 `scikit-image`, Stray 의 `opencv`, 서버의 `flask` · `pillow` 가 목록에 없었다 (`e2e_glb` 가 `ModuleNotFoundError: No module named 'skimage'` 로 `convert` 실패) | **고침** (D `98da703`, R1): `opencv-python-headless==5.1.0.95` · `scikit-image==0.26.0` · `Flask==3.1.3` · `pillow==13.0.0` |
| 8 | 상태 응답의 `status` (`web/server.py:405-407`) | 엔진 단계가 실패해도 `done` 이었다 (8-1 의 7번) | **고침** (D `98da703`, R2) |
| 9 | `stages.py:52` · `core.py:53-54` | Stray 촬영본 오류가 `"예외: CaptureError: <문장>"` 요약과 traceback 6줄로 나간다. 사용자 문장만 보이고, `checks.warnings` 도 입력 단계 주의로 보이게 해 달라고 요청함 (통합 기록 R4, C) | **고침** C `620a2b0` |
| 10 | `web/static/index.html:325-326` | 화면이 "천장이 있는가" 를 진단 `messages` 의 문장(`"천장이 있음"`)으로 판단해 3D 위쪽을 자른다. 문장이 바뀌면 깨지고, 지금도 천장이 일부만 찍힌 경우(`room`: 덮음 27%, 문장 "천장이 일부만 찍힘")는 자르지 않는다. 상태 응답의 `metrics.before.has_ceiling` · `shadow_fix` 를 보는 편이 약속에 맞다 [추정: 화면에서 직접 확인하지 않음] | 열림 (A) |
| 11 | `stages.s_input` (Stray 분기, `--resume`) | `--resume` 이면 `input_map/stray_meta.json` 이 있을 때 변환을, `build/mesh_raw.ply` 가 **있기만 하면** 형상 메쉬를 건너뛴다. 앞 실행이 변환 중에 멈췄다면 `stray.convert` 가 `stray_meta.json` 을 지운 채라 다시 변환하지만, 예전 `mesh_raw.ply` 는 남아 있어 새 키프레임 · 자세와 맞지 않는 메쉬를 그대로 쓸 수 있다 [추정: 그렇게 멈춰 본 적은 없음, 코드로만 봄]. 제안: 이번 실행에서 변환을 했으면 메쉬도 다시 만들기 (또는 `mesh_raw.ply` 가 `stray_meta.json` 보다 새것일 때만 건너뛰기). 서버는 `--resume` 을 쓰지 않아 웹 작업에는 닿지 않는다 | **메쉬는 고침** C `be8310e` (제안 두 가지 모두, `stages.py:53`, `:62-63`). **남은 것도 고침** C `13d1c4d` (통합 기록 R15): 메쉬를 다시 만들었으면 `convert` 도 건너뛰지 않는다 (`stages.py:73`, `:131`). 아래는 그 전 설명 — 메쉬를 다시 만들어도 `convert` 는 `build/report.json` 이 있으면 건너뛰었다 (`stages.py:128-129`). 그래서 USD · `visual.ply` · `transform.json` 이 예전 메쉬로 만든 것일 수 있다. 같은 입력 · 같은 코드면 메쉬가 같아(8-2 의 md5) 차이가 없고, 코드나 입력이 바뀐 뒤 `--resume` 을 쓸 때만 닿는다 [추정: 돌려 보지 않음, 코드로만 봄]. 제안: 이번 실행에서 메쉬를 다시 만들었으면 `convert` 도 건너뛰지 않기 |
| 12 | `web/static/index.html:241-243` | 판정 `fail` 의 문구가 `"쓸 수 없음 — 실패한 단계를 볼 것"` 으로 고정돼 있다. `105591c` 부터는 실패한 단계 없이 `verdict` 만 `fail` 인 경우가 있다 (3-4). 이유는 `report` 단계 요약 · 진단 주의 · 결함 `path_blocked` 에 있다 (`C_TRACK.md` C11 ①) | 열림 (A) |
| 13 | `web/server.py:405-415` (`worker`) · `index.html:240` | 서버가 `status` 를 `done` · `failed` 로 바꾼 뒤 `bundle.zip` 을 만들고 나서야 `finished` 를 쓴다 (D `9b2c95f`, 4-3). 그 사이에 화면이 상태를 받으면 조회를 멈추고 `job.finished - job.started` 를 적어 `"끝남 (NaN초)"` · `"실패 (NaN초)"` 가 된다. 입력 검사에 걸린 작업을 `#<작업 번호>` 로 열어도 `started` 가 없어 `"실패 (NaN초)"` 다 [추정: 코드로만 봄, 화면에서 보지 않음]. 제안: `finished` 를 `status` 와 함께 쓰고 `bundle.zip` 은 그 뒤에 (D), 또는 화면이 없는 값을 거르기 (A) | **서버 쪽 고침** D `6ad7114` — `status` 와 `finished` 를 `bundle.zip` 뒤에 같이 씀 (`:418`, 4-3). 입력 검사에 걸린 작업은 `started` 가 없어 화면의 `"실패 (NaN초)"` 는 남을 수 있음 [추정] — 화면이 없는 값을 거르기 (A) |
| 14 | `web/server.py:553` (`--keep` 도움말) | 도움말은 "-1 이면 지우지 않음" 인데, `_unzip/` 은 늘 지우고 (`tidy`, 통합 기록 R17 ②), 11/25 저녁부터는 `--keep` 과 상관없이 24시간 뒤 폴더째 지운다 (`purge`, 2-1). 모두 남기려면 `--keep -1 --ttl-hours 0` | **고침** D `6ad7114` — 도움말이 `_unzip` 과 `--ttl-hours` 를 적음 (`:559`), `S2S_KEEP` 을 `start.sh` 가 넘김 |
| 15 | `tests/e2e.py:101` | 시험 서버를 기본값으로 띄워 `--min-free-gb 20` 이 걸린다. 임시 폴더가 있는 디스크의 여유가 20 GiB + 시험 파일 크기보다 작으면 `POST /api/jobs` 가 507 이라 e2e 가 실패한다 (디스크 문제를 시험 실패로 보임). 제한 시간 20분 · 청소 24시간도 같은 값이다 (시험에는 해가 없음) [추정: 코드로만 봄, B 노트북은 여유가 커서 `TESTS_OK`]. 제안: e2e 가 `--min-free-gb 0` 을 주기 | **고침** B `7877487` — e2e 가 서버를 `--min-free-gb 0` 으로 띄움 |
