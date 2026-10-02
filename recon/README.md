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
recon/pipeline/io_rgbd.py         입력 읽기: open3d_lounge(시험용) · tum(B 인계)
recon/pipeline/tsdf.py            RGB-D 프레임 + pose → TSDF → mesh_raw.ply
recon/pipeline/build_usd.py       정리 → 바닥 평면으로 Z-up·원점 → 경량화 → 충돌 메쉬 → USD → 검증
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

`build_usd.py` 가 `[1/6]`~`[6/6]` 진행을 찍고, 마지막 줄이 **`검증 통과`** 면 성공 (문제가 있으면 목록을 찍고 종료 코드 1).

- **B 데이터**: config 를 복사해 `input.format: tum` 과 경로·intrinsics 를 채운다 (`lounge.yaml` 주석 참고). pose 는 **카메라 광학 좌표계 → map** 이어야 한다 (`rtabmap-export --poses_camera`)
- **충돌**: 정적 배경이라 삼각형 메쉬 그대로 (`MeshCollisionAPI approximation = none`). 볼록 분해는 움직이는 물체에만 필요
- **파라미터 실험**: config 를 레포 밖으로 복사해 값을 바꾸고 `--out` 을 다른 폴더로 → 두 `report.json` 비교

### 결과 확인 — `$OUT/`

| 파일 | 내용 | 보는 법 |
|---|---|---|
| `report.json` | **단계별 수치**: TSDF 프레임 수·시간, 지운 조각 수, 방 범위(m)·바닥 정점 비율, 시각용 오차, 충돌 메쉬 바닥 오차, USD 검증 결과 | 텍스트 편집기 |
| `usd/<이름>_scene.usda` | D 에게 넘기는 씬 (Z-up · m, 메쉬 파일을 payload 로) | 텍스트 편집기 |
| `usd/<이름>_mesh.usdc` | 시각용(`Visual`, 정점 색) + 충돌용(`Collision`, 안 보임) 메쉬 | Blender → Import → USD, Isaac Sim |
| `visual.ply` · `collider.ply` | USD 에 들어간 메쉬와 같은 것 | 아래 뷰어, CloudCompare, MeshLab |
| `mesh_raw.ply` | 정리·좌표 변환 전 TSDF 원본 | 〃 |
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
