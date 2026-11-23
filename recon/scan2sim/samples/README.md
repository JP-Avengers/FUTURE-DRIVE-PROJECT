# scan2sim 시험 자료 만들기

웹앱 엔진(`recon/scan2sim`)이 결함을 제자리에서 짚는지 보려고, **정답을 아는 가짜 입력**을 만드는 스크립트 두 개.
실제 스캔이 없어도 GLB 경로와 아이폰 원본(Stray Scanner) 경로를 둘 다 시험할 수 있다.

| 스크립트 | 만드는 것 |
| --- | --- |
| `make_room.py` | 가짜 실내 방 `room.glb` (정점 색) + 정답 `room_truth.json` |
| `make_stray.py` | 그 방 안을 걸으며 찍은 것처럼 만든 Stray Scanner 촬영 zip + `stray_truth.json` |

**만든 파일은 커밋하지 않는다.** 출력은 레포 밖(예: `~/webapp_runs/samples`)에 둔다.
`.gitignore` 가 `*.glb` 는 걸러 주지만 zip · mp4 · png 는 거르지 않는다.

## 명령

```bash
cd recon
python scan2sim/samples/make_room.py  --out ~/webapp_runs/samples            # [--seed 0] [--no-defects]
python scan2sim/samples/make_stray.py --room ~/webapp_runs/samples --out ~/webapp_runs/samples   # [--seconds 25] [--fps 30] [--seed 0]

python -m scan2sim ~/webapp_runs/samples/room.glb          --out ~/webapp_runs/samples/run_glb
python -m scan2sim ~/webapp_runs/samples/synth_room_s0.zip --out ~/webapp_runs/samples/run_stray
```

같은 `--seed` 면 같은 파일이 나온다 (색 흔들림 · 깊이 잡음만 seed 를 따르고, 방 배치와 결함 자리는 고정).

## 방과 결함

씬 좌표(Z 위, m, 방 가운데 = 원점, 바닥 z 0). 안쪽 8 × 6 m, 천장 2.8 m, 북쪽 벽(y = +3)에 문 구멍, 가구 상자 4개.

| 결함 (`type`) | 자리 (씬 x · y) | 무엇을 넣었나 | GLB 실행에서 걸리는 이유 |
| --- | --- | --- | --- |
| `double_floor` | x −2.5~−1.5 · y 1.1~2.1 (가운데 −2.0, 1.6) | 바닥 위 1.0~1.5 cm 에 뜬 얇은 판 | 충돌 메쉬는 바닥 ±5 cm 의 **위를 향한** 면만 빼므로 판 밑면(1.0 cm, 아래를 향함)이 남아, 평평한 충돌 바닥과 1 cm 간격으로 두 겹 |
| `no_visible_floor` | x 1.5~2.5 · y −2.1~−1.1 (가운데 2.0, −1.6) | 보이는 바닥 삼각형을 지운 1 m² 구멍 | 위에서 쏜 광선이 보이는 바닥을 못 만남. 충돌 바닥은 받침 바닥으로 메워져 빈 바닥으로 잡힘 |
| `low_bump` | x 0.0~1.6 · y 1.55~1.85 (가운데 0.8, 1.7) | 높이 4.5 cm · 길이 1.6 m 띠 (문턱 · 전선 덮개) | 윗면이 바닥 ±5 cm 안이라 충돌에서는 빠지고, 보이는 바닥만 3~12 cm 솟음 |

`--no-defects` 면 셋 다 빼고 만든다 (결함이 없을 때 아무것도 안 걸리는지 보는 대조군).
`--bump-cm` 으로 턱 높이를 바꾼다 (기본 4.5, 3~12). 5 cm 를 넘기면 턱 윗면이 GLB 충돌 메쉬에도 남는다.

### 어느 경로에서 무엇이 걸리나 (11.23 실행 결과, seed 0)

| 결함 | GLB (`room.glb`) | 아이폰 원본 (`synth_room_s0.zip`) |
| --- | --- | --- |
| `double_floor` | 걸림 (가운데 1 cm 안) → 보정 뒤 사라짐 | 안 걸림 — 자세가 정확해 바닥이 한 겹으로 복원되고, 1.5 cm 판은 바닥 높이 지도에 섞임 |
| `no_visible_floor` | 걸림 (2 cm 안) | 안 걸림 — 맵 폴더 경로는 구멍 위에도 바닥 한 장(보이는 바닥)을 새로 깜 |
| `low_bump` 4.5 cm | 걸림 (1 cm 안) | 안 걸림 — 바닥 높이 지도(띠 5 cm)가 턱을 바닥으로 봄 |
| `low_bump` 8 cm (`--bump-cm 8`) | 걸림 | 걸림 (3 cm 안). 다만 턱 가장자리에 `double_floor` 표본 2개가 덤으로 걸림 (보정 뒤 사라짐) |

결함 없는 방(`--no-defects`)의 GLB 실행에서는 아무 결함도 안 걸렸다.
같은 입력을 두 번 돌린 결과(`defects_*.json`)는 GLB · 아이폰 원본 모두 똑같았다.

## 정답 파일과 좌표

- `room_truth.json` — 결함마다 `center_input`(glTF 좌표, Y 위) 과 `center_scene_expected`(씬 좌표 예상값).
  둘 다 결함 가운데의 **바닥 높이 점**이라 scan2sim 의 `center_input` · `center_scene` 과 같은 뜻이다.
  - `center_scene_expected` 는 `glb_to_usd.py` 의 변환(Y 위 → Z 위, `origin: center`)을 정점 범위로 계산한 값이고,
    바닥 평면 맞춤이 정확히 기울기 0 · z 0 을 준다고 **가정**했다 (실제로는 0.03° 안팎 기울어짐).
  - 실행 결과와 견줄 때는 `<run>/build/transform.json` 의 `T_input_to_scene` 으로 `center_input` 을 옮기는 것이 가장 정확하다.
- `stray_truth.json` — 촬영의 세계 좌표 = `room.glb` 의 glTF 좌표 그대로. 견줄 때는
  세계 점 → `(x, −z, y)` (Y 위 → Z 위, `stray.py` 와 같음) → `T_input_to_scene`.

## Stray 촬영의 모양

- `rgb.mp4` 1920×1440 (mp4v) · `depth/NNNNNN.png` 256×192 16비트 mm · `confidence/NNNNNN.png` 0/1/2 ·
  `odometry.csv` · `camera_matrix.csv` · `imu.csv` — zip 안에 촬영 폴더 하나 (`synth_room_s<seed>/`).
- 자세 = 카메라 → 세계, 카메라 축 OpenCV (x 오른쪽 · y 아래 · z 앞), 세계 +Y 위, 쿼터니언 `qx, qy, qz, qw` (실제 자료와 같은 규약).
- 방 가운데를 도는 길(x ±1.9 · y ±0.9 m)을 0.42 m/s 로 한 바퀴, 카메라 높이 1.35 m · 아래로 약 32° · 좌우 ±50° 둘러봄.
- 처음 0.3 초는 실제 촬영처럼 기록된 자세가 튐 (`stray.py` 는 처음 1 초를 건너뜀).
- 깊이는 광선으로 정확히 구한 뒤 잡음을 조금 더함. 영상은 삼각형 색 × 간단한 음영 (사진 같지는 않음).
- 기본값(25 초 · 30 fps)으로 약 43 MB, 만드는 데 1.5~2.5 분 (다른 작업이 CPU 를 쓰는 정도에 따라).
- 자세에 오차가 없으므로 복원 형상은 정답 방과 거의 같다 (TSDF 메쉬 → 정답 거리 중앙 1 mm · 90% 3 mm).
  실제 아이폰처럼 자세가 틀어지는 경우는 흉내 내지 않았다.
