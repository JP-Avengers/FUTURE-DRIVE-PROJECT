# xm_ft v0.1 — 첫 파인튜닝 결과 (11.6, Kaggle T4 × 2)

- 체크포인트: `epoch=8-step=10719.ckpt`(best, `validation/losses/action` 0.3224) · 0.96 GB · sha12 `d86970ce1faf` — git 밖(E 보관, 서버 `data/checkpoints/xm_ft_v0.1/` 로 전달 예정)
- 데이터 `xm_parking_r3` v1.0(xm_v1) · 시작 ckpt `nvidia/X-Mobility` sha12 `339c68f9e027` · 배치 2 × 누적 2 × GPU 2 = 실효 8 · 10 에폭 · 4 h 39 min
- 오프라인 test(4,753프레임) 제로샷 → FT: v MAE 0.100 → 0.011 · ω MAE 0.123 → 0.050 · ω 부호 일치 84.8 → 95.8 %. Nav2 장애물 ω MAE 0.196 → 0.177(약점)

| 파일 | 내용 |
| --- | --- |
| `report_tables.md` | 표 E-14 · E-15 · MANIFEST 행 초안(`e2e/fill_report.py`) |
| `curves.md` · `curves.png` | 에폭별 검증 곡선(`e2e/plot_training.py`) |
| `weak_segments.md` | 제로샷 → FT 2 m 구간별 약한 곳(`e2e/weak_segments.py`) |
| `eval_*_summary.json` | `e2e/eval_offline.py` 묶음별 지표 |
| `run_meta.json` | 학습 기록(`e2e/finetune.py`) — 경로는 Kaggle 기준 |
