# 통합 기록 (B)

매일 main 최신(또는 B 작업 브랜치)으로 자동 시험과 실제 촬영본을 돌린 결과. 전날과 달라진 것과 다른 트랙에 넘길 것을 적는다. 촬영본 이름만 적고 파일은 저장소에 넣지 않는다.

## 다른 트랙에 넘길 것 (열린 것)

| 번호 | 받는 사람 | 내용 | 근거 | 상태 |
|---|---|---|---|---|
| R1 | D | `recon/requirements.txt`에 `opencv-python-headless`, `scikit-image`, `flask`, `Pillow`를 버전 고정해 더하기. 없으면 새 환경에서 Stray 입력은 `input` 단계(`stray.py`의 `import cv2`), GLB 입력은 `convert` 단계(`pipeline/glb_to_usd.py`의 `skeletonize`)에서 실패함. `flask` · `Pillow`는 지금 open3d에 딸려 우연히 설치됨 | 11/23 새 환경 시험 | 열림 |
| R2 | D | 서버가 엔진의 단계 실패를 `done`으로 표시함. `web/server.py` worker가 `report.md`가 있으면 `done`으로 두는데, `report` 단계는 앞 단계가 실패해도 항상 씀. 화면이 판정(`verdict`)으로 실패를 보여 주는지 A와 확인하거나, 실패 단계가 있으면 `failed`로 두기 | 11/23 복도 GLB e2e | 열림 |
| R3 | C | `stages.py`에서 B의 `stray_quality.measure`를 불러 복원 정확도를 진단 결과와 보고서에 넣기. 아래 "R3 붙이는 자리" | 6-B4의 1번 | 열림 |
| R4 | C | `s_input`에서 `stray.CaptureError`를 받아 그 문장만 실패 이유로 보여 주기(`except stray.CaptureError as e: return Result(FAIL, str(e))`). 지금은 `core.run`이 예외로 잡아 요약이 "예외: CaptureError: …"이고 보고서에 traceback 6줄이 나감. 그리고 `stray_meta.json`의 `checks.warnings`를 `r.warn`으로 보여 주기(예: 깊이 파일 1장 모자람, confidence/ 없음) | 11/23 B의 `stray.py` 입력 검사 | 열림 |
| R5 | C · D | 화면용 `preview.glb`가 너무 큼: 복도 GLB(삼각형 619만 개)에서 129.7 MB. 폰에서 못 열 수 있고 터널로 받는 데 오래 걸림. 삼각형 수를 줄이고(예: 30만 개), 작업이 끝날 때 미리 만들기(implementation 6-C 3번 · 6-D 4번) | 11/23 복도 GLB e2e | 열림 |
| R6 | C | 손에 든 폰 촬영에서 바닥 높이 지도가 깨질 수 있음. `pipeline/build_usd.py:113-117`이 카메라 높이를 "아래로 쏜 광선이 처음 닿는 거리의 중앙값"으로 잡는데, 폰 아래에 책상 · 의자가 있으면 값이 흔들림(예시 촬영 136장: 0.285 m). 같은 촬영을 키프레임 66장으로 줄이자 바닥 후보가 0개가 되어 `Ground.from_mesh`에서 `ValueError: No points given`으로 convert 실패. 제안: Stray 입력은 바닥 평면(RANSAC) 높이를 쓰거나, 후보가 없으면 band를 넓혀 다시 찾기 | 11/23 키프레임 상한 시험 | 열림 |
| R7 | C | Stray 입력의 처리 시간 대부분이 `build_usd`의 텍스처 굽기임(아래 "처리 시간"). 바닥과 벽마다 모든 키프레임을 따로 한 바퀴씩 돌며 960×720 깊이 버퍼를 다시 만듦. 키프레임마다 깊이 버퍼를 한 번만 만들어 바닥 · 모든 벽에 함께 쓰면 5분 촬영 기준 build 약 −50%(추정). 설정만으로는 `walls.max_walls` · `walls.max_dist_m`을 줄이는 방법 | 11/23 시간 측정 | 열림 |

### R3 붙이는 자리 (제안, 약 1초 더 걸림)

`s_diagnose`의 `r.summary = …` 바로 앞:

```python
    if ctx.get("source") == "stray":                      # 아이폰 원본: 안 쓴 프레임 깊이 ↔ 맵 (B 의 stray_quality)
        from . import stray_quality
        try:
            q = ctx.get("stray_quality") or stray_quality.measure(ctx["out"], capture=ctx["input"])
            ctx["stray_quality"] = q
            m["stray_quality"] = {k: q[k] for k in ("frames", "median_cm", "p90_cm", "within_2cm_pct", "hit_pct", "verdict")}
            r.messages.append(q["summary"])
            for w in q["reasons"]:
                r.warn(w)                                 # fail 이어도 주의로만 (맵은 만들어짐)
        except (SystemExit, Exception) as e:              # measure 는 SystemExit 을 냄
            r.messages.append(f"복원 정확도를 재지 못함: {e}")
```

`s_report`의 `if ctx.get("bundle"):` 앞:

```python
    sq = ctx.get("stray_quality")
    if sq:
        L += ["", "## 복원 정확도 (아이폰 원본)", "", f"- {sq['summary']} · 90% {sq['p90_cm']} cm → {MARK[sq['verdict']]} (기준: 중앙 3 cm 이하)"]
```

## 11/23 (B 노트북, WSL Ubuntu 24.04, 첫 확인)

- 코드: `b/yunseong` = main `a069f09` + B의 시험 도구.
- 환경: `python3 -m venv` + `pip install -r recon/requirements.txt`만으로 새로 만듦(41초, 2.0 GB). 그 뒤 `opencv-python-headless 5.1.0.95`, `scikit-image 0.26.0`을 따로 넣고 시험(R1).

| 입력 | 방법 | 결과 | 걸린 시간 | 메모 |
|---|---|---|---|---|
| 아이폰 예시 zip(17초, 50 MB, 사람이 찍혀 있음) | 엔진 직접 | 주의 · 결함 후보 4곳(보이는 바닥 없음) · 보정 안 함 | 52초(convert 45초), 최대 메모리 0.7 GB | 키프레임 839장 중 136장. 묶음의 메쉬 이름 `m_9a5813208f_mesh.usdc` |
| 같은 zip | `tests/e2e.py`(서버 + API) | 13/13 통과 | 55초 | 묶음의 메쉬 이름이 `input_mesh.usdc` — 서버가 올린 파일을 `input.zip`으로 저장해서 |
| 복도 v0.4 메쉬로 만든 GLB(삼각형 619만 개, 169 MB, 꼭짓점 색) | `tests/e2e.py` | **convert 실패**(`skimage` 없음) · 상태는 `done` | 108초 | R1 · R2 |
| 같은 GLB(`scikit-image 0.26.0`을 넣은 뒤) | `tests/e2e.py` | 14/14 통과 · 주의 · 결함 후보 30곳 → 보정 뒤 12곳 | 5분 23초(convert 2분 16초, correct 2분 43초) | `preview.glb` 129.7 MB(R5). 묶음의 메쉬 이름 `input_mesh.usdc` |
| 망가진 촬영본 25가지(파일 빠짐 · 짧음 · 자세 파일 깨짐 · 장 수 불일치 · zip 폴더 여러 개 · 폴더 밖 경로 등, 실제 촬영본을 잘라 만듦) | `stray.py` 입력 검사 | 모두 사용자 문장으로 멈춤. 1~2장 차이와 confidence/ 없음은 주의만 | — | 좋은 촬영본은 키프레임 파일까지 고치기 전과 같음(md5) |
| 아이폰 예시 zip의 엔진 결과 | `stray_quality.py`(검토 반영 뒤) | 안 쓴 프레임 30장(가장 가까운 키프레임에서 0.1초 이상, 중앙 0.117초 · 1.7 cm 떨어짐) 깊이 ↔ 맵: 중앙 1.23 cm · 90% 4.72 cm · 2 cm 안 68.5% · 맵에 닿음 95.8% → 통과. 처음 판(키프레임 바로 옆 장 포함)은 1.08 cm, 키프레임 자체로 재면 1.10 cm — 독립 검증이 아니라 자세 흔들림 · 깊이 잡음 수준만 봄 | 4.5초(측정 본체 1.4초) | 신뢰도 2 화소만 쓰면 중앙 0.82 cm · 2 cm 안 79%. 화면용 메쉬(`visual.ply`, 정리 · 줄임 뒤)는 중앙 2.74 cm. 자세를 일부러 틀리게 넣으면 38~65 cm로 커져 `fail` |

## 처리 시간 (11/23, Stray 예시 촬영으로 측정 · 5분 촬영은 추정)

부하가 높은 상태(load 8~14 / 8스레드)에서 잰 값이라 한가할 때보다 약 1.25배 느림. 비율과 기울기가 믿을 만한 부분.

**예시(17초, 키프레임 136장)의 단계별 시간 — 측정**

| 단계 | 초 | 무엇에 비례 |
|---|---|---|
| 입력(zip 풀기 + `stray.convert`) | 3.5 | 영상 전체 장 수(장당 약 3 ms) + 키프레임 |
| `check_input.py` | 3~5 | 거의 고정 |
| `tsdf.py` | 4.5~6.5 | 키프레임(장당 약 23 ms) |
| **`build_usd.py`** | **25~34** | 바닥 굽기 8.5 · 벽 굽기(벽 2개) 13.7 · 정점 색 5.2 · 나머지 약 4 · import 약 3 |
| 보이는 구멍 · nav2 · diagnose | 약 8 | 넓이 |
| correct | 0 또는 약 40 | 진단에 걸리면 빌드를 한 번 더 함 |

**키프레임 수에 따른 build 시간 — 측정(같은 촬영, 간격 기준만 바꿈)**: 69장 29 s · 136장 34~42 s · 296장 74 s · 501장 119 s → 약 15 s + 0.21 s × 키프레임(벽 2개). 넓이를 2배 · 4배로 늘린 시험(키프레임 272 · 544장, 벽 4 · 8개)은 102 s · 266 s. 바닥 · 벽 텍스처가 칠해진 비율은 간격을 바꿔도 거의 같았음(바닥 60~63%).

**5분 촬영 — 추정(실제 긴 촬영본은 아직 없음)**: 예시의 키프레임 비율(초당 8장 · m당 21장)로 1,900~3,200장, 방 100~150 m², 벽 3~10개로 놓으면 전체 약 12 / 20 / 35분(한가한 노트북 약 9 / 16 / 28분). **목표 10분을 넘길 가능성이 큼.** E의 5분 촬영본으로 실제로 잴 것.

**줄이는 방법**

1. 키프레임 상한(B, 적용함): `stray.py`의 `MAX_KEYFRAMES = 800`. 넘으면 움직임 · 회전 간격을 같이 넓혀 800장 안으로(`stray_meta.json`의 `select.widen`, 입력 단계 주의). 800장보다 적은 촬영은 그대로(예시는 키프레임 파일까지 같음). 예시를 70장 상한으로 돌리면 품질 수치 중앙 1.19 cm(상한 없이 1.23 cm)로 차이가 없었지만, R6 때문에 convert가 실패함.
2. 텍스처 굽기를 키프레임당 한 번으로(C, R7).
3. 색 크기 640×480: 예시 build 34 → 30 s. 바닥 사진 한 화소가 1.9 → 2.9 cm로 커짐. 2번이 같은 효과를 손실 없이 냄.
