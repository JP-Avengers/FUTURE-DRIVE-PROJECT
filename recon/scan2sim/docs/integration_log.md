# 통합 기록 (B)

매일 main 최신(또는 B 작업 브랜치)으로 자동 시험과 실제 촬영본을 돌린 결과. 전날과 달라진 것과 다른 트랙에 넘길 것을 적는다. 촬영본 이름만 적고 파일은 저장소에 넣지 않는다.

## 다른 트랙에 넘길 것 (열린 것)

| 번호 | 받는 사람 | 내용 | 근거 | 상태 |
|---|---|---|---|---|
| R1 | D | `recon/requirements.txt`에 `opencv-python-headless`, `scikit-image`, `flask`, `Pillow`를 버전 고정해 더하기. 없으면 새 환경에서 Stray 입력은 `input` 단계(`stray.py`의 `import cv2`), GLB 입력은 `convert` 단계(`pipeline/glb_to_usd.py`의 `skeletonize`)에서 실패함. `flask` · `Pillow`는 지금 open3d에 딸려 우연히 설치됨 | 11/23 새 환경 시험 | 고침 `98da703`(D) |
| R2 | D | 서버가 엔진의 단계 실패를 `done`으로 표시함. `web/server.py` worker가 `report.md`가 있으면 `done`으로 두는데, `report` 단계는 앞 단계가 실패해도 항상 씀. 화면이 판정(`verdict`)으로 실패를 보여 주는지 A와 확인하거나, 실패 단계가 있으면 `failed`로 두기 | 11/23 복도 GLB e2e | 고침 `98da703`(D) |
| R3 | C | `stages.py`에서 B의 `stray_quality.measure`를 불러 복원 정확도를 진단 결과와 보고서에 넣기. 아래 "R3 붙이는 자리" | 6-B4의 1번 | 대기 — B 가 main 에 올리면 C 가 붙임 |
| R4 | C | `s_input`에서 `stray.CaptureError`를 받아 그 문장만 실패 이유로 보여 주기(`except stray.CaptureError as e: return Result(FAIL, str(e))`). 지금은 `core.run`이 예외로 잡아 요약이 "예외: CaptureError: …"이고 보고서에 traceback 6줄이 나감. 그리고 `stray_meta.json`의 `checks.warnings`를 `r.warn`으로 보여 주기(예: 깊이 파일 1장 모자람, confidence/ 없음) | 11/23 B의 `stray.py` 입력 검사 | 대기 — B 가 main 에 올리면 C 가 붙임 |
| R5 | C · D | 화면용 `preview.glb`가 너무 큼: 복도 GLB(삼각형 619만 개)에서 129.7 MB. 폰에서 못 열 수 있고 터널로 받는 데 오래 걸림. 삼각형 수를 줄이고(예: 30만 개), 작업이 끝날 때 미리 만들기(implementation 6-C 3번 · 6-D 4번) | 11/23 복도 GLB e2e | 고침 `2a6e89f`(C) · `ed7220e`(D) |
| R6 | C | 손에 든 폰 촬영에서 바닥 높이 지도가 깨질 수 있음. `pipeline/build_usd.py:113-117`이 카메라 높이를 "아래로 쏜 광선이 처음 닿는 거리의 중앙값"으로 잡는데, 폰 아래에 책상 · 의자가 있으면 값이 흔들림(예시 촬영 136장: 0.285 m). 같은 촬영을 키프레임 66장으로 줄이자 바닥 후보가 0개가 되어 `Ground.from_mesh`에서 `ValueError: No points given`으로 convert 실패. 제안: Stray 입력은 바닥 평면(RANSAC) 높이를 쓰거나, 후보가 없으면 band를 넓혀 다시 찾기 | 11/23 키프레임 상한 시험 | 해당 코드를 더 타지 않음 `0534d95`(C) |
| R7 | C | Stray 입력의 처리 시간 대부분이 `build_usd`의 텍스처 굽기임(아래 "처리 시간"). 바닥과 벽마다 모든 키프레임을 따로 한 바퀴씩 돌며 960×720 깊이 버퍼를 다시 만듦. 키프레임마다 깊이 버퍼를 한 번만 만들어 바닥 · 모든 벽에 함께 쓰면 5분 촬영 기준 build 약 −50%(추정). 설정만으로는 `walls.max_walls` · `walls.max_dist_m`을 줄이는 방법 | 11/23 시간 측정 | 굽지 않음 `0534d95`(C) |
| R8 | C | **Stray(맵 폴더) 입력에서는 바닥 결함 진단이 사실상 동작하지 않음.** 가상의 방에 넣은 두 겹 바닥(1.5 cm 판) · 보이는 바닥 구멍(1 m²) · 낮은 턱(4.5 cm)이 TSDF 메쉬(`mesh_raw.ply`)에는 모두 제대로 있는데, 진단 결과는 0곳. `ground.npz`(띠 5 cm)가 턱과 판을 따라가 바닥을 그 높이로 다시 깔고, `build_usd.py:312-325`가 구멍 위에 보이는 바닥을 새로 깔아서, 진단이 엔진이 다시 깐 바닥끼리 견주게 됨. 같은 방을 GLB로 넣으면 3곳 모두 2 cm 안에서 잡힘. 주 입력이 Stray라 시연 핵심 기능에 걸림. 턱 8 cm 판에서는 Stray도 턱을 잡음(2.8 cm 안). 또 Stray 결과의 `visual.ply` · `collider.ply` 바닥이 벽 밖으로 0.6~0.7 m 뻗어 나감(보이는 면의 15%, TSDF 메쉬에는 없음) — Nav2 빈 바닥이 GLB 45 m² vs Stray 58 m² | 11/23 가상의 방(`samples/`) | 고침 `0534d95`(C) — B 가 run.sh 정답 비교로 확인(11/24) |
| R9 | C | 보이는 바닥 구멍(`no_visible_floor`)도 보정 단계에서 고치기. 결함 자리의 구멍만 둘레 바닥 색의 판(`FloorPatch` prim, 바닥 0.5 cm 아래)으로 덮고 다시 진단. 충돌 · Nav2 지도는 그대로. 아래 "R9 제안" | 11/24 B 시제품(저장소 밖 복사본): 가상의 방 GLB · Stray 1 → 0곳, 아이폰 예시 4 → 3곳, 기준 장소 촬영본은 할 일 없음. `tests/run.sh` TESTS_OK | 대기 — C 검토 |
| R10 | C | **Stray 형상 메쉬를 B 의 `stray_mesh.build` 로** (11/24 팀 결정: 형상 메쉬는 B 가 만들어 넘김). `stages.s_input` 의 Stray 분기에서 `pipeline/tsdf.py` 를 부르는 4줄을 아래 "R10 바꿀 자리"로. 결과 파일 이름 · 좌표는 그대로. **이것이 들어가야 B 의 개선(깊이 규칙의 3~5 m 깊이 · 가로 자세 보정)이 웹앱에 실제로 쓰임** — 지금 엔진은 tsdf.py 를 3 m 로 부름. B 복사본 측정(기준 촬영본, 11/24 밤): floor_two_levels 15.8 → 2.7%, 천장 있음(덮음 28 → 55%), 결함 6 → 1(보이는 바닥 없음), 복원 정확도 2.69 → 1.78 cm, e2e 65 s. 넣을 때 B 가 `CONTRACT.md` 의 `build/tsdf_config.yaml` 을 지움 | 11/24 B `stray_mesh.py` | 열림 |

| R11 | C | (R10 을 바로 못 넣을 때만) `detect.config_tsdf` 의 `depth_trunc: 3.0` → `5.0`. B 의 `stray.py` 가 이제 키프레임 깊이를 규칙대로 골라 두어(신뢰도 2 는 5 m 까지) 3 m 에서 자르면 천장 · 먼 벽을 버림. 그리고 `s_input` 의 "아이폰 자세에는 되돌아온 자리를 맞추는 보정이 없음 (얼마나인지는 재지 않음)" 주의는 R10 뒤로는 맞지 않음 — `build/tsdf_meta.json` 의 `pose_fix`(status · reasons · flags)를 읽어 보여 주기를 제안 | 11/24 밤 B 깊이 규칙 · 자세 보정 | 열림 |

### R9 제안 — 보이는 바닥 구멍을 보정 단계에서 덮기

기준 코드 `d43c952`(11/24 저녁 `99fa36e` 까지 이 파일들은 같음). 시제품은 B 노트북의 저장소 밖 복사본에서만 돌렸다. 저장소는 고치지 않았다. diff 전문: `docs/requests/R9_stages.diff` · `docs/requests/R9_glb_to_usd.diff` (`git apply` 로 바로 넣을 수 있음 — B 가 `0aa1026` 위에서 `git apply --check` 확인).

**지금 왜 남나** [코드 · 측정]

- GLB · Stray 길은 이미 스캔 바닥 아래에 받침 바닥 `FloorFill` 을 깐다 (`pipeline/glb_to_usd.py:603-606`). 범위는 충돌 바닥과 같은 `floor_fp` 다. 그래서 방 안 구멍도 덮인다 (`pipeline/mesh_utils.py:114` `binary_fill_holes`). `C_TRACK.md` 2-4 와 같다.
- 그런데 받침 바닥은 맵 전체 바닥 색의 중앙값 한 가지 색이다 (`glb_to_usd.py:605`, `floor_color` `:373`).
- 높이는 스캔 바닥보다 2 cm 아래다 (`configs/templates/polycam_glb.yaml:28`). 바닥 높이 지도를 쓰는 입력(실제 촬영본)은 5 cm 아래다 (`scan2sim/detect.py:105`).
- 진단은 `build/visual.ply`(스캔 메쉬만)에 광선을 쏜다 (`stages.py:249-258`). 받침 바닥은 세지 않는다. 그래서 받침 바닥이 있어도 `no_visible_floor` 는 늘 남는다.
- 보정 단계는 `double_floor` 일 때만 돈다 (`stages.py:245`). `no_visible_floor` 는 `fixable: false` 다 (`stages.py:181`).
- [측정] 결함 가운데에서 위로부터 쏘면 맞는 면은 가상의 방 GLB · Stray 모두 `FloorFill`(−2.0 cm)이다. 아이폰 예시 4곳도 모두 `FloorFill`(−4.8 ~ −8.1 cm)이다.
- 즉 카메라에 보이는 것은 빈 구멍이 아니라 **무늬 없는 한 가지 색이 2~8 cm 꺼진 자리**다. 결함 문구(`basis`)도 이렇게 고치기를 제안한다.

**하는 일**

1. 진단: GLB 길(GLB · Stray)에서 `no_visible_floor` 가 있으면 `needs` 에 넣는다. 보정 전 목록의 `fixable` 은 `true`.
2. 보정: 새 함수 `_patch_mask(ctx, defects)` 가 덮을 칸을 고른다 (Nav2 지도 격자 5 cm).
   - Nav2 빈 칸마다 바닥 위 10 cm 에서 `visual.ply` 로 아래로 쏜다. 바닥 높이 ± 띠(평평 5 cm · 높이 지도 8 cm) 안에 면이 없으면 구멍 칸이다. 1 m 에서 쏘는 진단과 달리 탁자 · 선반 밑에서도 바닥을 본다.
   - 이은 구멍(8방향) 중 보정 전 결함 원(`center_scene` · `radius_m`)에 걸친 것만 본다.
   - 덮지 않는 것: ① 바닥보다 띠 넘게 아래에 면이 보이는 칸이 10% 넘음(꺼진 자리 · 내려가는 계단 · 두 높이 바닥의 아래층) ② 둘레 20 cm 띠에 보이는 바닥이 50% 미만 ③ 4 m² 넘음(다시 찍을 것).
   - 덮는 구멍은 한 칸 넓혀 스캔 바닥 테두리 밑까지 간다. Nav2 빈 칸 안에서만 깐다. 장애물 · 벽 · 미지 칸에는 깔지 않는다.
   - `build/floor_patch_mask.npz` 를 쓰고, 보정 설정에 `floor_patch.mask_file` 을 넣어 convert → nav2 → diagnose 를 다시 돈다. 두 겹 바닥 보정과 같은 길이다. 둘 다 걸리면 한 번에 한다.
   - 덮을 칸이 없으면 보정을 건너뛴다(skip). 다시 만들지 않는다.
3. `glb_to_usd.py`: `floor_patch.mask_file` 이 있으면 보이는 prim `FloorPatch` 를 더한다. 충돌에는 넣지 않는다.
   - 높이 = 바닥 − 0.5 cm. 받침 바닥(−2 · −5 cm)보다 위, 스캔 바닥보다 아래다. 테두리에서 겹친 곳은 스캔 바닥이 보인다.
   - 색: 스캔 바닥에서 15 cm 안은 가까운 바닥 정점 64개 색의 중앙값이다(이음매가 안 보이게). 더 안쪽은 1,024개 중앙값(둘레 평균 색)으로 넘어간다. 가까운 색만 끌어온 첫 판은 타일 바닥에서 옆 칸 색이 구멍 안으로 번졌다 [측정, 아래 색 표].
   - 정점 색(`displayColor`) prim 이다. `FloorFill` 과 같은 방식이다.
   - `build/floor_patch.ply` 도 쓴다(진단용). 설정이 없으면 지난 것을 지운다.
4. 다시 진단: `floor_patch.ply` 도 보이는 바닥으로 센다. 높이 차 통계(`visible_minus_collision_cm`)와 두 겹 검사에서는 판을 뺀다. 판에 맞은 표본 수는 `metrics.floor_patch_samples` 에 둔다.
5. 보고:
   - 보정 단계 요약: "보이는 바닥 없음 1 → 0곳 (판 1.2 m²)".
   - 메시지: 덮은 곳, 덮지 않은 곳과 이유.
   - `metrics.floor_patch` = `{cells, area_m2, regions[center, area_m2, low_pct, ring_floor_pct, reason, patched]}`.
   - 보고서 "이 맵이 가정한 것과 바꾼 것" 에 덮은 곳 한 줄.
   - 보정 뒤 목록에 남은 구멍은 `fixable: false` 와 "보정 단계에서 덮지 않음" 문구.

**어디 — diff 요약** (stages.py +96 −14줄 · glb_to_usd.py +50 −4줄)

```diff
# recon/scan2sim/stages.py
@@ _defects :181  no_visible_floor
-   fixable=False, action="고치지 않음 — …"
+   fixable=(kind == "glb" and not corrected), basis 에 받침 바닥 설명, action 세 갈래 (덮음 · 보정 뒤 덮지 않음 · 맵 폴더는 그대로)
@@ s_diagnose :254  보이는 바닥 광선
+   pp, pid = build/floor_patch.ply, -1
+   if os.path.exists(pp): pid = sc.add_triangles(…)                     # 판도 보이는 바닥
-   h = sc.cast_rays(…)["t_hit"].numpy()
+   a1 = sc.cast_rays(…); h, fake = a1["t_hit"], a1["geometry_ids"] == pid
+   two = … & (a2["geometry_ids"] != pid)                                 # 판은 테두리 밑에 깔려 있어 두 겹으로 안 셈
+   visible_minus_collision_cm 는 ok & ~fake 로 · metrics.floor_patch_samples
@@ s_diagnose :327 뒤
+   if m["defects"].get("no_visible_floor") and ctx.get("kind") == "glb" and not ctx.get("corrected") and not ctx.get("diagnose_only"):
+       ctx.setdefault("needs", {})["no_visible_floor"] = True
+def _patch_mask(ctx, defects, top=0.10, max_m2=4.0, low_share=0.1, ring_share=0.5):   # 약 45줄
+   빈 칸 → 바닥 위 10 cm 에서 아래로 → 바닥 · 아래 면 · 구멍 격자 → ndimage.label(8방향)
+   → 결함 원에 걸친 덩어리만 → 거르기 3가지 → binary_dilation(1칸) & free
@@ s_correct :332
-   if glb: lines += ["collision:", …]           (지금은 늘 충돌 줄을 넣음)
+   if needs.get("double_floor") and glb: …   elif needs.get("double_floor"): …
+   if needs.get("no_visible_floor"):
+       D = defects_before.json 의 no_visible_floor
+       mask, regions = _patch_mask(ctx, D)
+       if mask.any(): savez(build/floor_patch_mask.npz); lines += ["floor_patch:", "  mask_file: …", "  depth_m: 0.005"]
+   if len(lines) == 2: return Result(SKIP, "고칠 수 있는 자리가 없어 보정하지 않음", …)
+   요약 · metrics.floor_patch · ctx["floor_patch"]
@@ s_report :528 뒤
+   덮은 곳 한 줄
# recon/pipeline/glb_to_usd.py
+def floor_samples(visuals, materials, band, ground)    # floor_color 앞부분을 떼어 xy 도 돌려줌. floor_color 결과는 같음 (FloorFill 해시 같음)
+def floor_patch_from_mask(path, ground, depth, pts, cols, k=64, wide=1024, blend_m=0.15)   # 약 25줄
@@ main :606 뒤
+   floor_patch.ply 지움 → mask_file 이 있으면 FloorPatch 를 visuals 에 · floor_patch.ply 씀 · report["floor_patch"]
```

**잰 것** [측정] — B 노트북 WSL. 다른 작업 3~4개와 함께 돌려서 걸린 시간은 ±20% 흔들린다 (코드가 같은 input 단계도 41 → 49 s).

| 입력 | `no_visible_floor` 보정 전 → 뒤 | 판 넓이 | 빈 바닥 중 보이는 바닥 없음 | 다른 결함 | 충돌 · 경계벽 · Visual · FloorFill · `map.pgm` | 보정 단계 |
|---|---|---|---|---|---|---|
| 가상의 방 GLB (구멍 1 m²) | 1 → 0 | 1.20 m² (구멍 1.00 + 테두리 한 칸) | 2.9 → 0.0% | 같음 (턱 · 두 높이) | 모두 바이트 같음. `FloorPatch` 960 삼각형 · USD +0.02 MB | 두 겹 바닥 보정과 함께 돎. 7.1 → 4.0 s (흔들림 안) |
| 가상의 방 Stray | 1 → 0 | 1.31 m² (구멍 1.10) | 3.6 → 0.3% | 같음 | 같음. 1,046 삼각형 | 7.7 → 9.4 s (흔들림 안) |
| 기준 장소 `6bc1ae021f` | 0 → 보정 안 함 | 0 | 2.4% | 같음 | 모두 같음 (USD 50.15 MB 그대로) | 건너뜀 — 할 일 없음 |
| 아이폰 예시 `9a5813208f` (17초 촬영) | 4 → 3 | 1.91 m² (1.63 m² 한 곳 + 아주 작은 것 2곳) | 79.5 → 63.2% | 없음 | 같음. 1,526 삼각형 | 새로 돎 8.3 s (전체 26.0 → 28.0 s) |

- 보정 전 결함 목록(종류 · 위치 · 표본 수)은 네 입력 모두 전과 같다.
- `map.yaml` 은 첫 줄 주석의 실행 폴더 경로만 다르다.
- 결함 가운데의 맨 위 보이는 면: 고친 뒤 `FloorPatch`(−0.5 cm, 예시는 −0.7 cm). 덮지 않은 예시 3곳은 그대로 `FloorFill`.
- 예시에서 덮지 않은 것: 9.83 m²(4 m² 넘음), 0.44 m²(둘레 보이는 바닥 0%). 이 촬영은 빈 바닥의 80% 에 보이는 바닥이 없다.
- 판 계산 자체: 마스크 0.02 / 0.17 / 1.1 / 0.13 s, 색 0.07~0.12 s. 최대 메모리는 그대로 (0.47 / 0.87 / 2.23 / 0.75 GB).
- 같은 입력 두 번(가상의 방 GLB): `floor_patch.ply` · `floor_patch_mask.npz` · `defects_after.json` · `room_mesh.usdc` · `map.pgm` 이 바이트까지 같다.
- `tests/run.sh`(전체, 시제품 복사본에서): **TESTS_OK** · 133초. 정답 결함 3개가 보정 전 목록에서 그대로 잡힘 (구멍: GLB 0.015 m · Stray 0.007 m 안).

색 · 높이 — 판 vs 받침 바닥 (0~255, 채널 평균 절댓값)

| | 가상의 방 GLB | 가상의 방 Stray | 아이폰 예시 |
|---|---|---|---|
| 높이 (바닥 기준) 판 / 받침 바닥 | −0.5 / −2.0 cm | −0.5 / −2.0 cm | −0.5 / −5.0 cm |
| 테두리 이음매: 판 정점과 10 cm 안 스캔 바닥 정점의 색 차. 판 / 받침 바닥 | 8.0 / 9.1 | 11.7 / 15.3 | 17.6 / 35.7 |
| 구멍 안 정답 바둑판 색과의 차. 판 / 받침 바닥 | 10.0 / 9.8 (가까운 색만 쓴 첫 판 13.2) | — | — |

- 판이 나아지는 곳은 테두리(색 · 높이)다. 구멍 안은 받침 바닥과 비슷하다. 무늬는 만들지 못한다.
- 예시의 테두리 스캔 바닥은 바닥 높이 지도보다 −4.3 ~ +5.3 cm(5~95%)로 출렁인다. 그래서 실제 촬영본에서는 높이 이음매가 남는다. 판이 스캔 바닥 위로 보이는 곳이 마스크의 4.7% 다 (판을 1.5 cm 아래로 두어도 2.9%).

**거르기 규칙이 실제 촬영본에서 어떻게 판단하나** [측정] — 결함 원 제한 없이, 0.05 m² 넘는 모든 구멍에 돌려 봄

| 입력 | 구멍 | 덮음 | 아래에 면 (꺼진 자리) | 둘레가 바닥 아님 | 4 m² 넘음 |
|---|---|---|---|---|---|
| 기준 장소 | 33 | 22 (9.25 m²) | 6 | 5 | 0 |
| 아이폰 예시 | 8 | 4 (1.95 m²) | 1 | 2 | 1 (9.83 m²) |
| 가상의 방 GLB | 8 | 6 | 0 | 2 | 0 |
| 가상의 방 Stray | 7 | 7 | 0 | 0 | 0 |

- 기준 장소에서는 결함이 없어 실제 흐름은 하나도 덮지 않는다.
- 기준 장소의 "아래에 면" 6곳은 바닥이 두 높이로 찍힌 자리(빈 바닥의 14.6%)의 아래층으로 보인다 [추정]. 덮지 않는 쪽이 맞다.
- 가상의 방 GLB 의 "덮음" 6곳 중 4곳은 벽 밑 한 칸 띠(0.3~0.4 m²)다. 벽 안쪽 칸이 Nav2 에서 빈 칸이라 잡힌다. 실제 흐름은 결함 원에 걸친 구멍만 덮으므로 이런 자리는 건드리지 않는다.

**위험 · 정할 것**

1. **찍힌 바닥이 아니다.**
   - 판은 둘레 색이라 타일 무늬나 그 자리의 낮은 물건(케이블 · 매트)이 없다.
   - 검은 매트 · 광택 바닥처럼 LiDAR 가 못 찍은 자리는 실제와 다른 색이 된다.
   - 보정 뒤 목록에서는 사라진다. 그래서 보정 메시지와 보고서 "바꾼 것" 에 위치를 남겼다.
   - 화면에도 보이게 할지(예: 보정 뒤 목록에 덮은 자리를 따로 남김)는 C · A 가 정할 것.
2. **꺼진 자리.** 아래에 면이 보이면 덮지 않는다. 아래가 아예 안 찍힌 진짜 구멍(어두운 계단참 등)은 가려 낼 수 없다 [추정]. 다만 충돌 바닥은 지금도 평평한 한 장이라 주행은 전과 같다. 바뀌는 것은 카메라 그림뿐이다.
3. **시간.**
   - 두 겹 바닥 없이 구멍만 있으면 빌드를 한 번 더 한다.
   - 예시는 +8.3 s [측정]. 기준 장소 크기면 convert 32 s + diagnose 4~5 s 로 약 +40 s [추정].
   - 빌드 없이 USD 에 prim 만 더하는 길도 있다. 다만 진단 · 미리보기 · 묶음을 따로 맞춰야 해서 이번 제안에서는 뺐다.
4. **잠정값.** 4 m² · 10% · 50% 와 판 깊이 0.5 cm 는 자료 4개로 잡은 값이다. `wide=1024` 는 정점 수라 촘촘한 TSDF 메쉬에서는 닿는 반경이 좁아진다 [추정]. 5 cm 칸으로 솎은 뒤 고르면 메쉬마다 같아진다.
5. **약속 문서.**
   - `fixable` 이 `no_visible_floor` 에서도 참이 된다 (`CONTRACT.md` 5-2 표 "`double_floor` 만", 5-4 표).
   - 보정이 `no_visible_floor` 로도 돈다 (3-3 단계 목록의 `correct` 줄, 5-4 "보정" 문단 "다른 종류로는 돌지 않는다").
   - prim `FloorPatch` 가 더해진다 (3-5 prim 목록, 2-4 그림자 설정 prim). `build/` 에 `floor_patch_mask.npz` · `floor_patch.ply` 가 생긴다.
   - 필드는 더하기만 하고 이름은 그대로다. `CONTRACT.md` 는 B 파일이라, C 가 넣으면 B 가 같은 날 고친다.
6. **Isaac Sim 에서 열어 보지 않았다.** `FloorFill` 과 같은 정점 색 prim 이라 같은 방식으로 보일 것이다 [추정]. 그림자 설정이 켜진 장면은 `scene_root.usda` 의 `doNotCastShadows` 목록에 `FloorPatch` 가 저절로 들어간다 [측정]. 화면용 `preview.glb` 에도 들어간다 (+0.02~0.03 MB).

### R10 바꿀 자리 (`stages.py`, `ccaca39` 기준 61~65행)

```diff
         if not (ctx.get("resume") and os.path.exists(path)):
-            tc = os.path.join(build, "tsdf_config.yaml")
-            open(tc, "w", encoding="utf-8", newline=chr(10)).write(detect.config_tsdf(mp, ctx["name"], build))
-            if _call([sys.executable, os.path.join(PIPE, "tsdf.py"), "--config", tc, "--out", build], ctx.path("log.txt")) or not os.path.exists(path):
-                return Result(FAIL, "촬영본에서 형상을 만들지 못함", metrics=dict(stray=sm), messages=["자세한 내용: log.txt"])
+            from . import stray_mesh                            # B 의 형상 메쉬 (pipeline/tsdf.py 와 같은 설정 · 같은 파일 이름 · 같은 좌표)
+            try:
+                with open(ctx.path("log.txt"), "a", encoding="utf-8") as fh:
+                    stray_mesh.build(mp, build, log=lambda s: print(s, file=fh, flush=True))
+            except stray.CaptureError as e:                     # stray_mesh.MeshError 도 여기로 — 쓸 장 없음 · 해상도 불일치 · 메쉬 빔
+                return Result(FAIL, "촬영본에서 형상을 만들지 못함", metrics=dict(stray=sm), messages=[str(e)])
```

- B 가 저장소 복사본에 이 패치를 넣어 가상 Stray 촬영본을 끝까지 돌림: 8단계 모두 돌고 `mesh_raw.ply` sha 가 tsdf.py 결과와 같음, 복원 정확도 0.39 cm, 보정 두 겹 18 → 0.
- `detect.config_tsdf` 는 부르는 곳이 없어짐 — 지울지는 C 가 정함. `pipeline/tsdf.py` 는 맵 폴더 길(run.py)이 계속 씀.
- 앞으로 B 가 이 단계에 넣을 것: 바닥 높이 틀어짐 보정(`pose_fix`), 처리 시간 줄이기, 깊이 범위 · 신뢰도 기준 — 모두 `stray_mesh.build` 안에서 바뀌므로 `stages.py` 를 다시 고칠 일은 없음.

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
| 가상의 방 GLB(`samples/make_room.py`, 넣은 결함 3개) | `tests/e2e.py --truth` | 18/18 통과. 두 겹 바닥 0.01 m · 보이는 바닥 없음 0.02 m · 낮은 턱 0.00 m 안에서 잡힘. 보정 뒤 두 겹 0 | 16초 | 결함 없는 방은 0곳. 두 번 돌려 결함 목록 같음 |
| 가상의 방 Stray 촬영본(`samples/make_stray.py`, 25초) | `tests/e2e.py --truth` | 흐름은 통과, **넣은 결함 3개 모두 못 잡음**(R8) | 86초 | 복원 메쉬는 실제 방과 중앙 0.9 mm · 90% 3.1 mm(자세가 정확한 가상 자료). 두 번 돌려 메쉬 · 결함 목록 같음 |
| `tests/run.sh` 전체(자료를 처음부터 만듦) | — | **TESTS_OK** (e2e_glb · repeat_glb · e2e_stray · repeat_stray · quality_stray). 가상 Stray 복원 정확도 중앙 0.39 cm · 2 cm 안 99.5% | 399초(자료 만들기 97초 포함) | |
| 끝난 작업 폴더 2개(가상의 방 GLB · Stray)를 `flask` · `pillow`만 든 환경의 서버로 열기 | curl | 상태 · preview.glb · map.png · report.md · bundle.zip 모두 200 | — | 화면(A) 개발용 묶음 23 MB(팀 공유 드라이브에 올릴 것). 입력 · 중간 파일은 뺌 |
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

## 11/24 (B 노트북, WSL Ubuntu 24.04)

- 코드: `b/yunseong` = main `ed7220e`(C · D 의 11/24 변경) + B 의 시험 도구.
- C · D 가 R1 · R2 · R5 · R6 · R7 · R8 을 처리함(위 표). R3 · R4 는 B 의 파일이 main 에 들어가면 C 가 붙임.

| 입력 | 방법 | 결과 | 걸린 시간 | 메모 |
|---|---|---|---|---|
| `tests/run.sh` 전체(Stray 도 정답 비교 켬) | — | **TESTS_OK**. Stray 에서 넣은 결함 3개를 두 겹 0.33 m · 구멍 0.01 m · 턱 0.00 m 안에서 잡음(R8 고쳐짐 확인). 복원 정확도 중앙 0.39 cm | 329초 | |
| 기준 장소 촬영본 `6bc1ae021f`(118초 · 6,611장 · 379 MB, E 가 찍음) | `tests/e2e.py`(서버 + API) | 15/15 통과 · 주의 · 결함 후보 6곳 · 보정 안 함 | 78~85초 · 최대 메모리 2.2 GB | C 의 Windows(바쁜 PC) 측정 334초와 큰 차이 — 같은 코드 기준인지는 확인 필요 |

### 11/24 저녁

| 입력 | 방법 | 결과 | 걸린 시간 | 메모 |
|---|---|---|---|---|
| B 의 형상 메쉬 `stray_mesh.py` vs `pipeline/tsdf.py` (같은 설정 2 cm · 3 m) | 가상의 방(193장) · 기준 촬영본(681장) 각 2번 | `mesh_raw.ply` **바이트까지 같음**(정점 · 삼각형 · 색 모두) | 기준 촬영본 22~23 s → 16~18 s, 가상의 방 7.7 s → 6.1 s · 최대 메모리 +1.5% | 그림을 cv2 로 읽고 스레드 2개로 미리 읽음. 다른 에이전트와 CPU 를 나눠 쓴 상태의 측정 |
| Stray 키프레임을 깊이 원래 크기(256×192)로 · 상한 2,000 · 이분 탐색 (B `stray.py`) | 기준 촬영본 | 키프레임 681(상한 800 · 간격 8.6 cm 로 넓힘) → **1,133**(안 넓힘). 변환 22 s · 메쉬 10.4 s (전: 681장을 960 으로 메쉬 16~18 s) | — | 960 과 메쉬 차이 중앙 0.4 mm(처리 시간 에이전트 측정). 공통 안 쓴 프레임 60장 기준 1,133장 쪽이 정확도 2.69 → 2.57 cm |
| 같은 변경으로 `tests/run.sh` 전체 | — | **TESTS_OK** · Stray 정답 결함 0.12 / 0.01 / 0.02 m | 132 s (전 329 s) | |
| 기준 촬영본 바닥 높이 틀어짐 (B 조사) | 키프레임마다 바닥 칸 높이 = 자세 높이 오류 + 칸의 실제 높이로 풂 | 처음 10 s 에 −20~+7 cm, 끝(출발 자리로 돌아옴) +8~+16 cm, 다시 지난 자리에서 중앙 5.3 cm 차이. **가로(벽)도 ±10~17 cm 틀어짐** | 보정 계산 약 8 s | 높이 보정만으로 C 의 두 높이 구간 14.6 → 6.6%, 바닥 두 겹 칸 22.9 → 11.8%. 가로까지 고치면 안 쓴 프레임 p90 24 → 8 cm (시험). 가상의 방은 보정해도 턱 · 판 그대로 |
| B 형상 메쉬에 바닥 높이 보정 (`stray_fix.py`, `stray_mesh.build` 기본 켬) | 기준 촬영본 1,133장 | 고침: 보정 5~95% −7.4 ~ +4.8 cm · 잔차 1.96 cm. C 기준 띠 14.55 → 7.77% · 바닥 두 겹 칸 19.8 → 12.9% · 안 쓴 프레임 중앙 2.03 → 1.89 cm. **천장 두 겹 칸 5.2 → 10.3% 로 나빠짐(원인 모름)** | 보정 6~7.5 s | 엔진 전체(R10 넣은 복사본): 결함 6 → 3, floor_two_levels 15.8 → 12.0%, 복원 정확도 p90 34 → 12 cm. 가상의 방은 건너뜀(메쉬 sha 같음). 새 깊이 규칙과 함께 쓸지는 다시 비교 중 |
| 복원 정확도의 안 쓴 프레임 고르기를 시간 격자로 (`stray_quality`) | 1,133장 vs 681장 맵 | 같은 장으로 재면 두 맵 차이 0.04 cm 안. 고른 장의 시각 차 4.5~6.6 s → 0.15~0.5 s, 맵끼리 수치 차 0.3~0.7 → 0.2~0.26 cm | — | 키프레임 설정을 바꿔도 보고서 수치가 덜 흔들림. 같은 입력이어도 숫자가 전과 다름 |
| Stray 깊이 규칙 hf(신뢰도 2 는 5 m · 신뢰도 1 은 3 m 안 바닥 + 10 cm 위만) + 깊이 한계 5 m (B `stray.py` · `stray_mesh.py`) | 기준 촬영본, 엔진(R10 넣은 복사본) | floor_two_levels 15.8 → 3.2% (보정 없이), 천장 덮음 28 → 56% · has_ceiling 참, 보이는 바닥 없음 0.8 → 6.9% | 키프레임 변환 +2~4 s | 신뢰도 2 만 쓰면 어두운 장애물 19 m² 가 빠져서 높이 조건을 둠. 가까운 거리 정확도 +0.2 cm 나빠짐 |
| 자세 보정 조합 비교 (새 규칙 · 5 m, 공통 안 쓴 프레임 60장) | 기준 촬영본 | 가로만: 바닥 두 겹 3.97 · 천장 두 겹 4.0 · 천장 키프레임 쌍 >5 cm 21.7% · 1.64/8.01 cm. 높이(기울기)+가로: 6.01 · 14.6 · 55.4% · 1.33/6.92 cm | 가로 2.5 s · 높이 + 가로 9.2 s | **기본값 = 가로만** (높이 단계는 천장을 나쁘게 하고, 엔진에서 floor_under 주의를 새로 만듦) |
| 코드 검토(B 파일 4개) | 다른 에이전트 | 높음 2(바닥 첫 훑기가 천장 많은 촬영본에서 엉뚱한 면을 고를 수 있음 — 17초 예시에서 확인 · 복원 정확도가 새 규칙 맵을 옛 규칙으로 잼 — 고침), 중간 2, 낮음 5 | — | 좌표 · 내부 파라미터 · 보정 적용 · 재현성은 돌려서 맞음 |
