# 실내 복도 맵 v0.2 — RealSense (B → C)

작성 2026-11-13 · 입력: 11/8 실내 촬영 5회차 중 4개 (D435i 손 촬영) · v0.1을 대신합니다

v0.1(왕복 1회차)에 정방향 회차와 오른쪽 벽면 회차를 더해 **벽을 더 높이 채웠고**(벽 꼭대기 중앙 1.1~1.25 m → 1.6~1.9 m), 맵에 쓰지 않은 **검증용 회차**를 넣었습니다. C의 실내 틀(`recon/configs/templates/indoor_rtabmap.yaml`) 입력 형식입니다.

## 쓰는 법

`recon/configs/corridor.yaml`의 ★ 줄을 채웁니다. 주 묶음만 써도 돌아갑니다.

```yaml
input:
  root: ~/recon_data/corridor_v0.2
  keyframes: keyframes
  poses: trajectory_camera_optical.txt
geometry:                               # B 형상 메쉬(세 묶음을 합친 것). 끄면 C의 tsdf.py가 주 묶음만으로 만듦(벽이 더 낮음)
  mesh: geometry_tsdf_2cm.ply
colorize:
  extra_inputs:                         # 보조 묶음 — 벽 위쪽은 rightwall 묶음에만 찍혀 있음
    - name: rightwall
      input: {format: rtabmap_keyframes, root: ~/recon_data/corridor_v0.2, keyframes: keyframes_rightwall, poses: trajectory_rightwall_camera_optical.txt, depth_scale: 1000.0}
    - name: in1
      input: {format: rtabmap_keyframes, root: ~/recon_data/corridor_v0.2, keyframes: keyframes_in1, poses: trajectory_in1_camera_optical.txt, depth_scale: 1000.0}
```

메쉬에는 이미 꼭짓점 색이 있으므로 `colorize: null`로 두고 그대로 써도 됩니다. 위 설정으로 `recon/pipeline/check_input.py`를 돌린 결과는 오류 0 · 경고 0입니다(11/13, main의 recon).

## 들어 있는 것

| 파일 | 내용 | 품질 |
|---|---|---|
| `keyframes/` + `trajectory_camera_optical.txt` | **주 묶음** 281장. `inside_2` 복도 왕복 | 가는 길 · 오는 길 벽 어긋남 중앙 3 cm |
| `keyframes_in1/` + `trajectory_in1_camera_optical.txt` | 보조 133장. `inside_1` 정방향 | 벽 점의 85%가 주 묶음 벽 5 cm 안 |
| `keyframes_rightwall/` + `trajectory_rightwall_camera_optical.txt` | 보조 177장. `inside_rightwall` — 벽을 45°로 보며 복도와 로비를 한 바퀴. 카메라 기울기 8~11°라 벽 위쪽이 찍힘 | 벽 점의 68%가 주 묶음 벽 5 cm 안. **복도 방향으로 0.2~0.3 m 어긋난 곳이 있음**. 빼낸 키프레임 자리에서 자세가 건너뜀(3곳) |
| `validation/keyframes` + `validation/trajectory_camera_optical.txt` | **검증용** 142장. `inside_1_reverse` — 맵에 쓰지 않음 | 맵 좌표 자세. 주 묶음 벽에 맞춰 넣음 |
| `geometry_tsdf_2cm.ply` | 주 + 보조 두 묶음을 합친 2 cm TSDF 메쉬(색 있음, depth 3 m 안, 바닥 아래는 잘라냄) | 아래 표 |
| `trajectory_camera_link.txt` | 주 묶음의 camera_link 자세 | |
| `topview.png`, `measurements.md`, `qa/` | 위에서 본 그림, 실측 대조용 수치, 점검 그림 | |

모든 묶음은 640×480, depth mm · RGB 정렬 · 거리 보정(c=0.007) 완료, 내부 파라미터 한 종류입니다. 묶음마다 키프레임 번호가 따로라서 번호가 겹칩니다.

## 좌표계

**건물 좌표**: 복도 축이 x, `inside_2` 출발점이 원점, 바닥이 z = 0, 단위 m, Z 위. 로비는 +y 쪽입니다. 주차장 맵과는 이어져 있지 않습니다.

## 만든 방법

1. **주 묶음**(v0.1과 같음): RTAB-Map → 바닥 수평 → 벽 방향으로 회전 오차 보정 → 복도 벽 직선 두 줄에 좌우 맞춤.
2. **보조 · 검증 묶음**: 이 회차들은 RTAB-Map 맵이 조각나거나 높이가 흘러서, 키프레임을 주 묶음 맵에 하나씩 맞춰 넣었습니다.
   - 기울기와 높이: 키프레임 depth의 바닥 평면으로 정했습니다. 벽을 정면으로 보는 키프레임은 "벽은 수직"이라는 조건을 함께 썼습니다.
   - 회전: 벽 방향으로 정했습니다.
   - 위치: 주 묶음의 벽 거리 지도에 맞췄습니다(6 cm 격자). 복도 방향(앞뒤)은 문틀 같은 구조가 있을 때만 정해지고, 없는 구간은 오도메트리 값입니다.
   - 주 묶음 벽과 10 cm 안에서 맞는 점이 절반이 안 되는 키프레임은 뺐습니다(로비 안쪽 등).

## 품질 — 맵 구성별 비교

검증 회차(`validation/`, 48장 표본)의 실제 depth와 같은 자세에서 메쉬를 렌더한 depth를 비교했습니다. 로봇 시점은 높이 0.45 m, 수평 시선으로 복도 30곳에서 광선이 12 m 안에 메쉬에 닿는 비율입니다. 모두 계산값이고 줄자 실측과는 아직 대조하지 않았습니다.

| 메쉬 구성 | depth 차 중앙 | 5 cm 안 | 검증 화소를 채운 비율 | 로봇 시점 채움 | 화면 위 1/3 채움 |
|---|---|---|---|---|---|
| 주 묶음만 (v0.1) | 3.6 cm | 57% | 78% | 64% | 22% |
| + in1 | 3.5 cm | 58% | 84% | 66% | 25% |
| **+ in1 + rightwall (이 판)** | 3.6 cm | 57% | 87% | **76%** | **47%** |
| + leftwall까지 | 3.8 cm | 55% | 88% | 78% | 52% |

- 벽면 회차를 넣어도 검증 depth 차는 그대로이고, 로봇 화면 위쪽 채움이 22% → 47%로 늘었습니다.
- `inside_leftwall`은 넣지 않았습니다. 흰 벽에서 위치 추적이 자주 끊겨 문 주변이 번지고, 채움은 5%p만 늘었습니다.
- depth 차 3.6 cm에는 D435i depth 잡음(3 m에서 수 cm)과 검증 자세 오차가 들어 있습니다.

**벽 높이** (복도 벽을 0.5 m 칸으로 나눠 칸마다 잰 벽 꼭대기의 중앙값)

| | -y 벽 | +y 벽 |
|---|---|---|
| 주 묶음만 (v0.1) | 1.25 m | 1.07 m |
| 이 판 | 1.89 m (1.8 m 넘는 구간 69%) | 1.61 m (1.8 m 넘는 구간 38%) |

2.2 m를 넘는 구간은 -y 벽 4%, +y 벽 0%입니다. 메쉬 최고점은 2.47 m입니다.

**복도 폭** (높이 0.4~1.4 m 벽면 사이)

| 메쉬 구성 | 폭 | 5 m 구간별 |
|---|---|---|
| 주 묶음만 | 2.54 m | 2.49~2.57 m |
| + in1 | 2.55 m | 2.50~2.59 m |
| 이 판 | 2.60 m | 2.50~2.63 m |

rightwall 묶음의 벽면이 주 묶음 벽면보다 양쪽으로 2~3 cm씩 바깥에 놓여서 합친 메쉬의 폭이 6 cm 넓게 나옵니다. 어느 쪽이 맞는지는 줄자 실측이 있어야 압니다. 그때까지 복도 폭은 **2.54~2.60 m**로 봐 주세요.

## 한계

- **천장이 없고 벽 위쪽이 비어 있습니다.** 벽은 대체로 1.6~1.9 m까지이고, 로봇 시점에서 화면 위쪽의 절반은 여전히 비어 있습니다. 카메라를 수평이나 약간 위로 든 추가 촬영이 있으면 채울 수 있습니다.
- **rightwall 묶음의 앞뒤 어긋남**: 문과 표지판이 0.2~0.3 m 번져 보이는 곳이 있습니다. 벽 사진을 구울 때 이 묶음을 쓰면 문틀이 겹쳐 보일 수 있으니, 결과를 보고 묶음을 빼거나 합의 거르기(`walls.consensus`)를 조이면 됩니다.
- **직선 복도 가정**: 복도 벽이 직선이고 폭이 일정하다고 보고 맞췄습니다. 평면도와는 대조하지 않았습니다.
- **로비**: 유리벽 구간은 형상이 지저분하고, 로비 안쪽 키프레임은 대부분 뺐습니다.
