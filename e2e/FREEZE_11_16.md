# 11.16 기능 동결 · 11.20 후보 전달 · 11.23 모델 동결 — 체크리스트 (E, 11.11 초안)

후보가 정해지면 `[ ]` 와 `‹…›` 만 채운다. 근거 · 판정은 `PLAN.md`(11.9 계획 · 11.11 절), 숫자는 `results/`.

## 1. 후보 표 (11.15 결정 2 뒤 채움)

| 후보 | 데이터 | 설정(finetune 인자) | best ckpt 파일 · sha12 | 노드 파라미터(기본값과 다른 것만) | 오프라인 근거 |
|---|---|---|---|---|---|
| 제로샷(비교군) | — | 공개 `x_mobility-nav2-semantic_action_path.ckpt` | `339c68f9e027` | `FD_METHOD=xmobility_zeroshot` | `obstacle_diag` 중앙 0.10 |
| aw30(백업) | xm_v1 | `--gin XMobilityLoss.action_weight=30` | `xm_ft_v0.1_aw30_e8.ckpt` · `4fdf249cda0a` | 없음 | v1.4 test w_mae 0.082 · 중앙 0.18 |
| A | xm_v1_1_p3 | aw30 + 경로 ×1/3 데이터 | `xm_ft_v0.1_aw30_p3_e8.ckpt` · `114284109957` | `path_scale:=0.3333`(`~/path` 만) | 11.9 교차 평가 1위 |
| ‹E4 · E5 · E6 중› | v1.4 · v1.5 | aw30 + `--prev-action-source exec` [+ `--prev-action-dropout 0.5`] | ‹› · ‹› | **`feed_clamped_action:=true`** | ‹중앙 크기 비 · w_mae› |

- **실행 명령 입력판(`exec`) 은 `feed_clamped_action:=true`**: 학습 입력 `exec_command` = 실제로 보낸 명령(상한 v 0–0.8 · |ω| ≤ 0.7 안). 노드 기본(false)은 자르기 전 예측을 넣음. 하네스에서는 `FD_XM_ARGS="--node-param feed_clamped_action:=true"`.
- p3 계열이 아니면 `path_scale` 기본 1.0. 드롭아웃은 학습 때만 — 노드 설정 없음.

## 2. 11.16 기능 동결 (코드 · 문서)

- [ ] 노드 `fd_xm_infer` 파라미터 고정 — 위 표의 칸 외에는 기본값. 바꿀 것이 생기면 11.16 전에만.
- [ ] `docs/interfaces/e2d_model.md` 버전 칸: 후보 ckpt 이름 · sha12 · 크기 · 노드 파라미터, 변경 이력 한 줄.
- [ ] `eval/XM_HARNESS.md` §2: 후보별 `FD_CKPT` · `FD_XM_ARGS` 한 줄씩(exec 판 `feed_clamped_action`).
- [ ] `e2e/README.md`: 학습 · 평가 명령을 최종 설정으로(v1.4 · v1.5, `--prev-action-source exec`).
- [ ] MANIFEST: 학습 데이터 행(v1.4 · v1.5) · 후보 ckpt 행(아래 형식).
- [ ] 레포 커밋 해시를 이 파일 끝에 — 그 뒤 `e2e/` · `ros2_ws/src/fd_xm_infer/` 기능 변경 없음(버그 수정만, 커밋 메시지에 '동결 뒤 수정').

MANIFEST ckpt 행 형식(기존 11.9 A 행과 같게):
```
| 11.‹›| `checkpoint` | `xm_ft` | ‹v1.4_aw30_exec› | `data/checkpoints/‹run›/` | 987M | ‹sha12› | `xm_parking_r3 (‹데이터›) + X-Mobility 사전학습 ckpt(sha 339c68f9e027) → e2e/finetune.py(레포 ‹rev›, X-Mobility 5837aead8902), ‹N› 에폭 · 16-mixed · ‹인자›` | E | `best = validation/losses/action 최소(‹값›), global_step ‹›, world_size ‹›. 노드 feed_clamped_action:=true. 실물: E 로컬 ‹바이트› B · Drive ‹날짜›` |
```

## 3. 11.20 후보 전달 (D, 짧은 폐루프 2–3회)

- [ ] ckpt 를 Drive E 폴더에(링크는 메시지로만 — 레포는 공개) · 로컬 크기 = Drive 크기 확인.
- [ ] D 에 보낼 것: 후보마다 파일 이름 · sha12 · `FD_CKPT` · `FD_XM_ARGS` 한 줄, 같은 시나리오로 제로샷 · aw30 도(비교군).
- [ ] 시나리오: 60회 씬(‹v0.8 r3 / r3›)의 바퀴 cw · ccw + 중앙 장애물 1–2개(STALL 이 났던 것). 판정은 2 m 규칙(`max_dev`).
- [ ] 받은 `results.csv` → `eval/analyze_runs.py` → 규칙 1(통과율 → 평균 cte → 충돌).

## 4. 11.23 모델 동결

- [ ] 최종 ckpt sha · `run_meta.json` · MANIFEST 행 · `results/xm_ft_v1.0/`(오프라인 표 · 장애물 진단 · 짧은 폐루프).
- [ ] 폐루프에서 제로샷보다 나은 후보가 없으면 그것도 결과로 보고(동결 후보에 제로샷 포함, PLAN 11.10).

동결 커밋: ‹›
