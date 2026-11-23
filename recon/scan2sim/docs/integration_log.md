# 통합 기록 (B)

매일 main 최신(또는 B 작업 브랜치)으로 자동 시험과 실제 촬영본을 돌린 결과. 전날과 달라진 것과 다른 트랙에 넘길 것을 적는다. 촬영본 이름만 적고 파일은 저장소에 넣지 않는다.

## 다른 트랙에 넘길 것 (열린 것)

| 번호 | 받는 사람 | 내용 | 근거 | 상태 |
|---|---|---|---|---|
| R1 | D | `recon/requirements.txt`에 `opencv-python-headless`, `scikit-image`, `flask`, `Pillow`를 버전 고정해 더하기. 없으면 새 환경에서 Stray 입력은 `input` 단계(`stray.py`의 `import cv2`), GLB 입력은 `convert` 단계(`pipeline/glb_to_usd.py`의 `skeletonize`)에서 실패함. `flask` · `Pillow`는 지금 open3d에 딸려 우연히 설치됨 | 11/23 새 환경 시험 | 열림 |
| R2 | D | 서버가 엔진의 단계 실패를 `done`으로 표시함. `web/server.py` worker가 `report.md`가 있으면 `done`으로 두는데, `report` 단계는 앞 단계가 실패해도 항상 씀. 화면이 판정(`verdict`)으로 실패를 보여 주는지 A와 확인하거나, 실패 단계가 있으면 `failed`로 두기 | 11/23 복도 GLB e2e | 열림 |
| R3 | C | `stages.py`에서 B의 `stray_quality.measure`를 불러 복원 정확도를 진단 결과와 보고서에 넣기. 아래 "R3 붙이는 자리" | 6-B4의 1번 | 열림 |

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
| 아이폰 예시 zip의 엔진 결과 | `stray_quality.py` | 안 쓴 프레임 30장 깊이 ↔ 맵: 중앙 1.08 cm · 90% 4.39 cm · 2 cm 안 70.2% · 맵에 닿음 97.8% → 통과 | 4.5초(측정 본체 1초) | 신뢰도 2 화소만 쓰면 중앙 0.82 cm · 2 cm 안 79%. 화면용 메쉬(`visual.ply`, 정리 · 줄임 뒤)는 중앙 2.74 cm. 자세를 일부러 틀리게 넣으면 38~65 cm로 커져 `fail` |
