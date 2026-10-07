# recon/ — Mesh / USD

- **담당**: C · 이진우
- **목적**: Point Cloud 를 정제해 Open3D 메쉬로 만들고, 경량화·Collider 를 거쳐 Isaac Sim 용 USD 로 변환한다.

## 간트 작업

| 작업 | 기간 |
| --- | --- |
| Point Cloud 정제 · Mesh v0 | 10.05–10.23 |
| 경량화 · Collider · USD 최종 | 10.24–11.16 |

## 입력 · 출력

- **받는 것**: B 의 포인트클라우드 · [`b2c_map.md`](../docs/interfaces/b2c_map.md)
- **넘기는 것**: USD → D · [`docs/interfaces/c2d_usd.md`](../docs/interfaces/c2d_usd.md) (**드래프트 USD 중간 납기 포함**)

## 환경

공용 노트북 · Python 3.12 venv `~/o3d_env` — 버전은 [`requirements.txt`](requirements.txt) 로 고정.

```bash
python3 -m venv ~/o3d_env && ~/o3d_env/bin/pip install -r recon/requirements.txt
```

Windows 에서는 `py -3.12 -m venv ~/o3d_env` 로 만들고, 아래 명령의 `~/o3d_env/bin/python` 을 `~/o3d_env/Scripts/python.exe` 로 바꾼다 (Git Bash 기준. PowerShell 은 `$OUT=...` · `$OUT\...`).

> 메쉬·USD 바이너리(`*.ply` `*.obj` `*.usdc` …)는 커밋하지 않습니다 — [`MANIFEST.md`](../MANIFEST.md) 에 기록.

## 파이프라인 — RGB-D + pose → USD

```
recon/configs/<이름>.yaml         설정 (입력 형식, voxel, 깊이 한계, 목표 삼각형 수 …)
recon/pipeline/io_rgbd.py         입력 읽기: open3d_lounge(시험용) · tum · rtabmap_keyframes(B 맵 v0.2)
recon/pipeline/color_utils.py     키프레임 RGB → 메쉬 정점 색 투영 (가림 처리) + 키프레임 정합 점검
recon/pipeline/tsdf.py            RGB-D 프레임 + pose → TSDF → mesh_raw.ply (근거리) + mesh_far_raw.ply (원거리, 배경용)
recon/pipeline/build_usd.py       정리 → 바닥 평면으로 Z-up·원점 → 구멍 메우기 → 경량화 → 충돌 메쉬 → 구역(바깥·배경·경계) → USD → 검증
recon/pipeline/mesh_utils.py      공용: 지도 범위(footprint)와 평면·벽, 작은 구멍 메우기, 바닥 채우기
recon/pipeline/usd_utils.py       USD 쓰기·검증 (씬 .usda + 메쉬 .usdc payload)
recon/pipeline/check_visibility.py  주행 통로에서 보이는 구멍 점검 (가상 카메라 광선) → 다시 찍을 곳 목록
```

### 실행

레포 루트에서. 결과 폴더는 config 의 `out`(기본 `~/recon_out/<이름>`)이고, `--out` 으로 아무 곳이나 지정할 수 있다 (두 명령에 같은 폴더). 레포 안은 피한다.

```bash
OUT=~/recon_out/lounge        # 결과를 둘 폴더 — 원하는 곳으로
~/o3d_env/bin/python recon/pipeline/tsdf.py      --config recon/configs/lounge.yaml --out $OUT   # 약 20초
~/o3d_env/bin/python recon/pipeline/build_usd.py --config recon/configs/lounge.yaml --out $OUT   # 약 30초
~/o3d_env/bin/python recon/pipeline/check_visibility.py --config recon/configs/lounge.yaml --out $OUT   # 약 15초 (선택)
```

`build_usd.py` 가 `[1/8]`~`[8/8]` 진행을 찍고, 마지막 줄이 **`검증 통과`** 면 성공 (문제가 있으면 목록을 찍고 종료 코드 1).

- **B 데이터**: config 를 복사해 `input.format: tum` 과 경로·intrinsics 를 채운다 (`lounge.yaml` 주석 참고). pose 는 **카메라 광학 좌표계 → map** 이어야 한다 (`rtabmap-export --poses_camera`)
- **구멍 메우기** (config `fill:`): ① 작은 구멍 — 테두리를 평평한 면으로 이음 (새 면 방향은 주변 면에 맞춤) ② 바닥 — 지도 범위 안에서 기존 바닥 면이 덮지 않는 5 cm 칸을 z=0 평면 조각으로 (기존 바닥보다 2 mm 낮게, 색은 가까운 바닥 색)
- **구역** (config `zones:`, 없애면 근거리만): 정확한 근거리 데이터는 주행 구역에, 원거리 데이터는 그 바깥 배경에만 쓴다.

  | USD prim (`/World/<이름>/…`) | 내용 | 보임 | 충돌 |
  | --- | --- | --- | --- |
  | `Visual` | 근거리 메쉬 (깊이 `depth_trunc`, 구멍 메우기 포함) | O | |
  | `VisualFar` | 원거리 메쉬 중 근거리 지도 밖 부분 — 잡음이 있어 충돌에는 안 넣음 | O | |
  | `GroundFar` | 근거리 지도 밖 바닥 평면 (가까운 바닥 색) — 바깥 구역 바닥 구멍 가림 | O | |
  | `Backdrop` | 전체 지도 + `ground_margin_m` 테두리의 배경 벽 (단색) | O | |
  | `Collision` | 근거리 충돌 메쉬 + 전체 바닥 평면 | | O |
  | `Boundary` | 같은 테두리의 경계벽 — 로봇이 지도 밖으로 못 나감 | | O |

  부분마다 prim 이 따로라 Isaac Sim 에서 `VisualFar`·`Backdrop` 만 끄고 켜는 비교가 가능하다.
- **충돌**: 정적 배경이라 삼각형 메쉬 그대로 (`MeshCollisionAPI approximation = none`). 볼록 분해는 움직이는 물체에만 필요
- **파라미터 실험**: config 를 레포 밖으로 복사해 값을 바꾸고 `--out` 을 다른 폴더로 → 두 `report.json` 비교

### 실제 주차장 — B 맵 v0.2 (`configs/map_v02.yaml`)

B 가 형상 메쉬(원본 depth TSDF, 색 없음)와 키프레임을 넘기므로 `tsdf.py` 없이 바로 `build_usd.py` → `check_visibility.py`. 받은 폴더는 영문 경로로 복사 (Open3D 는 한글 경로를 못 읽음).

- **색**: 키프레임 RGB 를 메쉬에 투영 (`color_utils.py`) — 가림은 메쉬 깊이 + 키프레임 depth 로 거름. 키프레임별 depth ↔ 메쉬 차이는 `color_qc.csv` (B 피드백용)
- **바닥 높이 지도** (`ground:`, `ground.npz`): 실외 바닥은 평평하지 않아 z=0 대신 카메라 경로 아래 지면 기준으로 칸마다 바닥 높이. 바닥 채우기·충돌 바닥 면이 이 높이를 따라감
- **10.7 미팅 반영**: 주행 중 구멍만 안 보이게. 배경 벽·지도 밖 바닥 면은 만들지 않고(배경은 보이는 대로), 경로에서 `max_path_dist_m`(6 m) 넘는 촬영 안 된 곳은 메우지 않음 — 외곽은 점군 보강으로 완성
- 시각용은 경량화하지 않음 (정점 색이라 줄이면 주차선이 뭉개짐)

### 결과 확인 — `$OUT/`

| 파일 | 내용 | 보는 법 |
|---|---|---|
| `report.json` | **단계별 수치**: TSDF 프레임 수·시간, 지운 조각 수, 방 범위(m)·바닥 정점 비율, 시각용 오차, 충돌 메쉬 바닥 오차, USD 검증 결과 | 텍스트 편집기 |
| `usd/<이름>_scene.usda` | D 에게 넘기는 씬 (Z-up · m, 메쉬 파일을 payload 로) | 텍스트 편집기 |
| `usd/<이름>_mesh.usdc` | 부분별 메쉬 prim (아래 구역 표) | Blender → Import → USD, Isaac Sim |
| `visual.ply` · `collider.ply` | 보이는 부분 전부 · 충돌 메쉬(근거리 + 바닥 평면) — USD 와 같은 것 | 아래 뷰어, CloudCompare, MeshLab |
| `mesh_raw.ply` · `mesh_far_raw.ply` | 정리·좌표 변환 전 TSDF 원본 (근거리 · 원거리) | 〃 |
| `visual_far.ply` · `backdrop.ply` · `boundary.ply` | 바깥 구역 · 배경 벽 · 경계벽 (아래 구역 설명) | 〃 |
| `floor_patch.ply` | 바닥 채우기로 넣은 평면 조각 (의자·테이블 밑 등 바닥이 빈 곳) | 〃 |
| `transform.json` | `mesh_raw` → 씬 좌표 4×4 변환 (c2d_usd.md 변경 이력용) | 텍스트 편집기 |
| `tsdf_meta.json` | TSDF 설정·프레임 수·시간·대략적인 위쪽 방향 | 텍스트 편집기 |
| `visibility/summary.json` | 통로에서 보이는 구멍 비율 (`holes_visible_pct`), 분류별 비율, 바닥 구멍 중 지도 안/가장자리 너머 | 텍스트 편집기 |
| `visibility/holes_list.csv` | **다시 찍을 곳 후보**: 구멍 위치(25 cm 칸)와 그 구멍이 보인 가상 카메라 수 | 엑셀, 텍스트 편집기 |
| `visibility/hole_map.png` | 위에서 본 구멍 지도 (바닥 · 물체, 초록 = 메쉬 범위, 파랑 = 경로) | 이미지 뷰어 |
| `visibility/holes_seen.ply` | 구멍이 보인 지점 (주황 = 바닥, 빨강 = 물체) — `visual.ply` 와 겹쳐 보기 | 아래 뷰어 |

**`check_visibility.py` 광선 분류** — 경로에서 좌우로 옮기고 고개를 돌린 가상 카메라에서 픽셀마다 광선을 쏴서: 면 앞쪽에 맞음 = 정상 · 면 **뒤쪽**에 맞음 = 물체 구멍 · **아래로** 빠짐 = 바닥 구멍 · **옆으로** 빠짐 = 벽 구멍/먼 배경 없음 · **위로** 빠짐 = 실외면 하늘(정상). 바닥 구멍은 메쉬 범위 안(진짜 구멍)과 가장자리 너머(촬영 범위 밖)로 나눈다. 설정은 config 의 `visibility:`.

메쉬를 축(빨강 X · 초록 Y · 파랑 Z, 0.5 m)과 함께 창으로 보기 — 파랑이 위, 원점이 바닥 모서리면 정상:

```bash
~/o3d_env/bin/python -c "import open3d as o3d, sys; m = o3d.io.read_triangle_mesh(sys.argv[1]); m.compute_vertex_normals(); o3d.visualization.draw_geometries([m, o3d.geometry.TriangleMesh.create_coordinate_frame(0.5)])" $OUT/visual.ply
```

> 창 뷰어는 화면이 있는 PC 에서만 뜬다 (SSH 로는 안 됨 — 파일을 내려받아 보거나 노트북에서 `draw_plotly`).

## 리허설 (9.21, 점군 → Poisson)

결과 폴더는 `--out` 으로 지정한다 (기본 `~/recon_out/rehearsal`, 두 명령에 같은 폴더).

```bash
OUT=~/recon_out/rehearsal     # 결과를 둘 폴더 — 원하는 곳으로
# 1) 샘플 점군(Open3D PLYPointCloud)으로 전체 리허설: 정제 → Poisson → 경량화 → Collider → OBJ  (약 4초)
~/o3d_env/bin/python recon/rehearsal/rehearsal.py --out $OUT
# 2) OBJ → USD (/World/VisualMesh + /World/ColliderMesh, Z-up · m)
~/o3d_env/bin/python recon/rehearsal/to_usd.py --out $OUT
```

### 결과 확인 — `$OUT/`

| 파일 | 내용 |
|---|---|
| `visual_mesh.obj` | 시각용 메쉬 (Poisson → 5,000 삼각형) |
| `collider_hull.obj` | 충돌용 볼록 껍질 하나 (아래 알려진 한계) |
| `campus_scene.usd` | 둘을 담은 USD (바이너리) |

`.usd` 는 바이너리라 편집기로 안 열린다. 텍스트로 풀어서 보려면 (`.usda` 는 레포 안에 두지 말 것 — git 이 추적함):

```bash
~/o3d_env/bin/python -c "from pxr import Sdf; import sys; Sdf.Layer.FindOrOpen(sys.argv[1]).Export(sys.argv[2])" $OUT/campus_scene.usd $OUT/campus_scene.usda
```

## 알려진 한계

- `rehearsal.py` 의 Collider 는 점군 **전체의 convex hull 하나**라 실내 샘플 검증용이다. 실제 씬에서는 로봇이 볼록 덩어리 안에 갇힌다 → `pipeline/` 은 삼각형 메쉬 충돌로 바꿈. Collider 방식은 [`c2d_usd.md`](../docs/interfaces/c2d_usd.md) 에서 D 와 합의 필요.
- `pipeline/` 의 원점은 아직 바닥 x·y 최솟값 모서리다. 주차장은 ENU 방향 + 루프 남서 꼭짓점 원점(c2d_usd.md)으로 맞추는 단계가 더 필요하다.
- 겉모습은 정점 색(displayColor)이라 경량화하면 색 해상도도 같이 떨어진다. 텍스처 방식은 D·E 와 논의.
- `Backdrop` 은 단색 벽이다. 실외에서 어떻게 보일지(단색 · 사진 · 단순한 건물 상자, 거리·높이)는 E 의 카메라 입력에 영향을 주므로 D·E 와 정한다.
