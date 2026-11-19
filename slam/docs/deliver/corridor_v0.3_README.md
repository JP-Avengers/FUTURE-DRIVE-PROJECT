# 실내 복도 맵 v0.3 — RealSense (B → C)

작성 2026-11-20 · 입력: 11/8 실내 촬영 4회차 + 11/18 추가 촬영 1회차 (D435i 손 촬영) · v0.2를 대신합니다

v0.2에 **카메라를 30° 위로 들고 복도를 왕복한 회차(`30_ceiling`)**를 더해 **천장과 벽 위쪽을 채웠습니다**.
- 복도 천장: 0% → 99%
- 벽 꼭대기 중앙: 1.6~1.9 m → 2.7 m
- 로봇 화면 위 1/3 채움: 47% → 74%

나머지 묶음과 좌표계는 v0.2와 같습니다. C의 실내 틀(`recon/configs/templates/indoor_rtabmap.yaml`) 입력 형식입니다.

## 쓰는 법

v0.2 설정(`recon/configs/corridor.yaml`)에서 `root`를 바꾸고 보조 묶음 `ceil30`을 더합니다.

```yaml
input:
  root: ~/recon_data/corridor_v0.3
  keyframes: keyframes
  poses: trajectory_camera_optical.txt
geometry:                               # B 형상 메쉬(네 묶음을 합친 것, 천장 포함)
  mesh: geometry_tsdf_2cm.ply
colorize:
  extra_inputs:                         # 보조 묶음 — 벽 위쪽 · 천장은 ceil30 묶음이 가장 많이 찍음
    - name: ceil30
      input: {format: rtabmap_keyframes, root: ~/recon_data/corridor_v0.3, keyframes: keyframes_ceil30, poses: trajectory_ceil30_camera_optical.txt, depth_scale: 1000.0}
    - name: rightwall
      input: {format: rtabmap_keyframes, root: ~/recon_data/corridor_v0.3, keyframes: keyframes_rightwall, poses: trajectory_rightwall_camera_optical.txt, depth_scale: 1000.0}
    - name: in1
      input: {format: rtabmap_keyframes, root: ~/recon_data/corridor_v0.3, keyframes: keyframes_in1, poses: trajectory_in1_camera_optical.txt, depth_scale: 1000.0}
```

위 설정으로 main의 `recon/pipeline/check_input.py`를 돌린 결과는 아래 "점검" 절에 있습니다.

천장이 새로 생겼으므로 `walls`의 벽 찾기가 천장을 벽으로 잡지 않는지 한 번 봐 주세요. 천장 법선은 위아래라 `min_normal_cos: 0.5` 조건이면 걸러질 것으로 봅니다(확인하지 않음).

## 들어 있는 것

| 파일 | 내용 | 품질 |
|---|---|---|
| `keyframes/` + `trajectory_camera_optical.txt` | **주 묶음** 281장. `inside_2` 복도 왕복 (v0.2와 같음) | 가는 길 · 오는 길 벽 어긋남 중앙 3 cm |
| `keyframes_in1/` + `trajectory_in1_camera_optical.txt` | 보조 133장. `inside_1` 정방향 (v0.2와 같음) | 벽 점의 85%가 주 묶음 벽 5 cm 안 |
| `keyframes_rightwall/` + `trajectory_rightwall_camera_optical.txt` | 보조 177장. `inside_rightwall` (v0.2와 같음) | 벽 점의 68%가 5 cm 안. 복도 방향 0.2~0.3 m 어긋난 곳 있음 |
| **`keyframes_ceil30/` + `trajectory_ceil30_camera_optical.txt`** | **새 보조 199장.** `30_ceiling`(11/18) — 카메라를 약 32° 위로 들고 복도 · 로비 왕복 | 벽 점의 63%가 5 cm 안, 키프레임별 10 cm 안 비율 중앙 87%. **높이는 천장 높이 가정에 기댐(아래 한계)** |
| `validation/keyframes` + `validation/trajectory_camera_optical.txt` | **검증용** 142장. `inside_1_reverse` — 맵에 쓰지 않음 (v0.2와 같음) | |
| `geometry_tsdf_2cm.ply` | 네 묶음을 합친 2 cm TSDF 메쉬(색 있음, depth 0.4~3 m, 바닥 아래와 복도 천장 위로 튄 조각은 잘라냄) | 아래 표 |
| `trajectory_camera_link.txt` | 주 묶음의 camera_link 자세 | |
| `topview.png`, `measurements.md`, `qa/` | 위에서 본 그림(천장이 덮여 보임), 실측 대조용 수치, 점검 그림 | |

- 모든 묶음은 640×480, depth mm · RGB 정렬 · 거리 보정(c=0.007) 완료, 내부 파라미터 한 종류입니다(11/18 촬영도 같은 카메라).
- 묶음마다 키프레임 번호가 따로라서 번호가 겹칩니다.
- `keyframes_ceil30`에서는 사람 얼굴 · 유리문에 비친 사람 · 노트북 화면이 찍힌 키프레임과 그 앞뒤(원래 번호 213~217, 258~264)를 뺐습니다.

## 좌표계

v0.2와 같습니다. **건물 좌표**: 복도 축이 x, `inside_2` 출발점이 원점, 바닥이 z = 0, 단위 m, Z 위. 로비는 +y 쪽입니다.

## `ceil30` 묶음을 맞춰 넣은 방법

이 회차는 단독으로는 RTAB-Map 맵이 되지 않았습니다. 기본 설정에서는 추적 끊김 96%였고, 재시도 설정에서는 41조각으로 나뉘었습니다. 그래서 키프레임을 주 묶음 맵에 하나씩 맞춰 넣었습니다. v0.2의 방법과 다른 점은 셋입니다.

1. **기울기**: 바닥이 거의 안 보여서 키프레임 depth의 **천장 평면**으로 정했습니다. IMU 가속도의 위 방향과 8° 안으로 맞는 평면만 쓰고, 천장이 안 잡히면 IMU 값을 썼습니다(천장 198장, 바닥 4장, IMU 39장).
2. **높이**: 카메라 높이 = 천장 높이 − 카메라에서 천장까지 거리. 천장 높이는 **2.87 m로 가정**했습니다. 바닥과 천장을 이웃한 키프레임에서 함께 본 16쌍의 중앙값이고, 25~75% 범위는 2.81~2.93 m입니다.
3. **조각 사이 방향**: 추적이 끊겨 다시 시작할 때 오도메트리 방향이 틀어졌습니다(반환점에서 실제 162° 회전을 약 90°로 기록). 조각 안에서는 오도메트리 방향을 쓰고, 조각과 조각 사이는 **IMU 자이로**를 중력 축으로 적분한 방향으로 이었습니다. 조각 안에서 자이로와 오도메트리 방향 차이는 최대값 중앙 1.9°였습니다.

위치는 v0.2와 같이 주 묶음의 벽 거리 지도에 맞췄습니다. 새로 "카메라는 통로 안에 있다"는 조건(기준 궤적에서 2 m 안, 벽에서 25 cm 밖)을 벌점으로 넣었습니다. 벽 일치가 50% 미만인 키프레임 33장은 뺐습니다.

## 품질 — 맵 구성별 비교

검증 회차(48장 표본)의 실제 depth와 같은 자세에서 메쉬를 렌더한 depth를 비교했습니다. 로봇 시점은 높이 0.45 m, 수평 시선으로 복도 30곳에서 광선이 12 m 안에 메쉬에 닿는 비율입니다. "통로 안에 떠 있는 면"은 벽에서 25 cm 넘게 안쪽 · 바닥과 천장 사이에 있는 메쉬 면적(번짐 · 잡음)입니다. 모두 계산값입니다.

| 메쉬 구성 | depth 차 중앙 | 5 cm 안 | 로봇 시점 채움 | 화면 위 1/3 | 복도 천장 덮음 | 벽 꼭대기 중앙 | 통로 안에 떠 있는 면 |
|---|---|---|---|---|---|---|---|
| v0.2 (주 + in1 + rightwall) | 3.6 cm | 57% | 76% | 47% | 0% | 1.89 / 1.63 m | 0.72 ㎡ |
| **v0.3 (+ ceil30)** | 3.7 cm | 56% | **85%** | **74%** | **99%** | **2.72 / 2.73 m** | 0.87 ㎡ |
| + 벽 45° 두 회차까지 | 3.7 cm | 56% | 88% | 81% | 100% | 2.72 / 2.71 m | 4.58 ㎡ |

- 검증 depth 차는 그대로이고, 천장과 벽 위쪽이 채워졌습니다.
- 11/18의 벽 45° 두 회차(`wall_45`, `wall_45_2`)는 넣지 않았습니다. 화면 위쪽 채움은 7%p 더 늘지만, 통로 안에 떠 있는 조각이 5배가 됩니다. `wall_45_2`는 기준 맵 밖(계단 쪽)을 걸은 구간도 있습니다.
- 천장만 찍은 두 회차(`ceiling`, `60_ceiling`)는 넣지 않았습니다. 벽이 보이는 키프레임이 13장 · 25장뿐이라 복도 방향 위치를 정할 수 없었습니다. 천장은 `ceil30`만으로 99%가 덮였습니다.
- 로비 쪽 검증 키프레임에서 렌더가 사진과 다른 것은 v0.2 메쉬에서도 같습니다(검증 회차의 로비 구간 자세 문제).

## 한계

- **천장 높이 2.87 m는 추정값입니다.** `ceil30` 묶음의 높이 전체가 이 값에 기대므로, 실제와 다르면 벽 위쪽 · 천장이 그만큼 위아래로 어긋납니다(25~75% 범위로 ±6 cm). 벽면 요철로 기존 맵과 높이를 대조하는 방법은 시험했지만 판별력이 없어 쓰지 않았습니다. **줄자로 바닥~천장 높이를 재 주시면** 바로 고칠 수 있습니다.
- **천장 조명**: 조명 패널은 depth가 튀어서 천장 위로 솟은 조각이 생겼습니다. 복도 구간에서 3.27 m 위는 잘라냈고, 로비는 천장 높이가 달라 그대로 두었습니다.
- **벽 2.7 m 위 ~ 천장 사이**: 벽과 천장이 만나는 모서리 근처는 아직 군데군데 비어 있습니다.
- v0.2의 한계(rightwall 묶음 복도 방향 어긋남, 직선 복도 가정, 로비 유리벽 잡음, 줄자 미대조)는 그대로입니다.

## 점검

- C `check_input.py`(11/20 main, 위 설정): **오류 0 · 경고 1.** 경고는 `ceil30` 묶음의 자세 연속성(5.1 m 넘게 튀거나 45° 넘게 도는 곳 6곳)입니다. 6곳 모두 키프레임을 뺀 자리입니다: 로비 안 16장(벽 일치 낮음, 8.2 m), 로비 출구 8장(5.0 m), 반환점 4장(137°), 출발 · 도착 제자리 회전(62~99°).
- 체크섬: `checksums.sha256`
