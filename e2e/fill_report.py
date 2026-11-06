#!/usr/bin/env python3
"""학습 · 평가 결과 → 결과보고서 표(REPORT_DRAFT.md 표 E-14 · E-15) · MANIFEST checkpoint 행 초안.

  --train  e2e/finetune.py 의 -o 폴더(run_meta.json, 있으면 curves.md)            → 표 E-14 · MANIFEST 행
  --zs     e2e/eval_offline.py 의 -o 폴더, 사전학습 ckpt 그대로(제로샷)            → 표 E-15 왼쪽 값
  --ft     e2e/eval_offline.py 의 -o 폴더, 파인튜닝 ckpt                           → 표 E-15 오른쪽 값
  --ckpt   (선택) 내려받은 best ckpt 파일 — 크기 · sha256 12자리를 MANIFEST 행에 넣음
셋 중 있는 것만 씀. 출력: 화면 + --out(기본 report_tables.md). 숫자를 보고서에 옮기는 일만 하고 해석은 하지 않는다.

사용: python3 e2e/fill_report.py --train <학습 폴더> --zs <제로샷 평가> --ft <FT 평가> [--ckpt <best.ckpt>] [--version xm_ft_v0.1]
필요: 표준 라이브러리만.
"""
import argparse
import datetime
import hashlib
import json
import os
import sys

# Windows 에서 출력을 파이프 · 파일로 돌리면 stdout 이 cp949 라 '—' 등에서 UnicodeEncodeError — 못 쓰는 글자만 ? 로
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")

ROUTE_KO = {"0_straight": "지시 직진", "1_left": "지시 좌회전", "2_right": "지시 우회전"}


def load(d, name):
    p = os.path.join(d, name) if d else None
    if not p or not os.path.isfile(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def f3(x):
    return "—" if x is None else f"{x:.3f}"


def pct(x):
    return "—" if x is None else f"{100 * x:.1f} %"


def pair(a, b, key, fmt):
    va = fmt(a.get(key)) if a else "—"
    vb = fmt(b.get(key)) if b else "—"
    return f"{va} · {vb}"


def table_e15(zs, ft):
    """제로샷 · FT summary.json → 표 E-15 (묶음 = 전체 · 시나리오 · 경로 지시 · 장애물 근처)."""
    base = ft or zs

    def keys(sect):   # 제로샷 · FT 묶음의 합집합(한쪽에만 있는 시나리오 · 지시도 행으로 남김)
        return sorted(set((zs or {}).get(sect, {})) | set((ft or {}).get(sect, {})))
    rows = [("전체", "overall")]
    rows += [(f"{k}", ("by_scenario", k)) for k in keys("by_scenario")]
    rows += [(ROUTE_KO.get(k, k), ("by_route_command", k)) for k in keys("by_route_command")]
    rows.append((f"장애물 {base['near_obstacle']['threshold_m']} m 안", "near_obstacle"))

    def get(s, key):
        if s is None:
            return None
        if isinstance(key, tuple):
            return s.get(key[0], {}).get(key[1])
        return s.get(key)

    lines = ["| 묶음 | 프레임 | v MAE (m/s) 제로샷 · FT | ω MAE (rad/s) 제로샷 · FT | ω 부호 일치 제로샷 · FT | 경로 ADE (m) 제로샷 · FT | FDE (m) 제로샷 · FT |",
             "| --- | --- | --- | --- | --- | --- | --- |"]
    for name, key in rows:
        a, b = get(zs, key), get(ft, key)
        na, nb = (a or {}).get("n"), (b or {}).get("n")
        n = nb if na is None or na == nb else (na if nb is None else f"{na} · {nb}")   # 다르면 제로샷 · FT 둘 다
        lines.append(f"| {name} | {n if n is not None else '—'} | {pair(a, b, 'v_mae', f3)} | {pair(a, b, 'w_mae', f3)} | "
                     f"{pair(a, b, 'w_sign_agree', pct)} | {pair(a, b, 'ade_m', f3)} | {pair(a, b, 'fde_m', f3)} |")
    if "2_right" in base["by_route_command"] and "1_left" not in base["by_route_command"]:
        lines.append("| 지시 좌회전 | 이 split 에 없음 (E.3.5-1) | — | — | — | — | — |")
    setup = (ft or zs).get("setup", {})
    note = f"split {setup.get('split', '?')} · 파일 {setup.get('files', '?')} · 프레임 {setup.get('frames', '?')}" \
           f" · teacher_forcing {setup.get('teacher_forcing', '?')}"
    warn = ""
    if zs and ft:
        sa, sb = zs.get("setup", {}), ft.get("setup", {})
        diff = [f"{k} 제로샷 {sa.get(k)} · FT {sb.get(k)}" for k in ("split", "files", "frames", "teacher_forcing")
                if sa.get(k) != sb.get(k)]
        if diff:
            warn = "\n\n**경고: 제로샷 · FT 평가 조건이 다름 — 같은 행끼리 비교 불가** (" + " · ".join(diff) + ")"
    return "\n".join(lines) + f"\n\n({note})" + warn


def table_e14(rm, curves, version, ckpt_info):
    k = rm.get("keys") or {}
    test = rm.get("test") or {}
    rows = [
        ("모델 버전 · MANIFEST 행", f"`{version}`" + (f", sha256 `{ckpt_info['sha12']}`" if ckpt_info else "")),
        ("학습 데이터", f"`{rm.get('dataset')}`"),
        ("환경 · GPU 수", f"world_size {rm.get('world_size')} · torch {rm.get('torch')} · lightning {rm.get('lightning')} · DDP {rm.get('ddp_backend')}"),
        ("에폭 · 총 스텝", f"{rm.get('epochs')} 에폭 설정 · global_step {rm.get('global_step')} · 에폭당 {rm.get('steps_per_epoch')} 스텝"),
        ("스텝 시간 · GPU 최대 메모리", f"{rm.get('step_time_s')} s · {rm.get('peak_gpu_mem_gb')} GB"),
        ("학습 시간", f"{rm.get('started')} → {rm.get('finished')}"),
        ("키 대조 (넣은 키 · missing · unexpected)", f"{k.get('loaded')} · {len(k.get('missing') or [])} · 예상분 {k.get('unexpected_expected')} + 예상 밖 {len(k.get('unexpected_other') or [])}"),
        ("ckpt 하이퍼파라미터 → 적용", f"{k.get('ckpt_hparams')} → gin 값"),
        (f"best ({rm.get('monitor')})", f"{rm.get('best_score')} — `{os.path.basename(rm.get('best_model_path') or '')}`"),
        ("마지막 val_loss", f"{rm.get('val_loss')}"),
        ("test 지표 (X-Mobility 내장)", ", ".join(f"{m.split('/')[-1]} {v:.4f}" for m, v in (test.get("metrics") or {}).items()
                                              if isinstance(v, (int, float)) and "metrics/action" in m) or "—"),
        ("repo · X-Mobility 커밋", f"`{rm.get('repo_rev')}` · `{rm.get('xm_rev')}`"),
    ]
    out = "| 항목 | 값 |\n| --- | --- |\n" + "\n".join(f"| {a} | {b} |" for a, b in rows)
    if not rm.get("finished"):   # finetune.py 는 학습 전 · 끝난 뒤 두 번 씀 — finished 없으면 학습이 끝나지 않은 run
        out = ("**경고: run_meta.json 에 finished 가 없음 — 학습이 끝나지 않은(중단 · OOM) run. best · global_step · "
               "world_size 등은 None**\n\n" + out)
    if curves:
        out += "\n\n학습 곡선(에폭별 검증, plot_training.py):\n\n" + curves
    return out


def sha12_size(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for c in iter(lambda: f.read(1 << 20), b""):
            h.update(c)
    return dict(sha12=h.hexdigest()[:12], size_gb=os.path.getsize(path) / 2**30)


def manifest_row(rm, version, ckpt_info):
    day = datetime.date.today()
    size = f"{ckpt_info['size_gb']:.2f}G" if ckpt_info else "【크기】"
    sha = ckpt_info["sha12"] if ckpt_info else "【sha12】"
    src = (f"`xm_parking_r3 ({os.path.basename(str(rm.get('dataset')).rstrip('/'))}) + X-Mobility 사전학습 ckpt(sha {rm.get('checkpoint_sha12')})"
           f" → e2e/finetune.py(레포 {rm.get('repo_rev')}, X-Mobility {rm.get('xm_rev')}), {rm.get('epochs')} 에폭 · {rm.get('precision')}`")
    note = (f"`best = {rm.get('monitor')} 최소({rm.get('best_score')}), global_step {rm.get('global_step')}, world_size {rm.get('world_size')},"
            f" 키 missing {len((rm.get('keys') or {}).get('missing') or [])}`")
    return (f"| {day.month}.{day.day} | `checkpoint` | `xm_ft` | {version.replace('xm_ft_', '')} | `data/checkpoints/{version}/` | {size} | {sha} | "
            f"{src} | E | {note} |")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--train")
    ap.add_argument("--zs")
    ap.add_argument("--ft")
    ap.add_argument("--ckpt")
    ap.add_argument("--version", default="xm_ft_v0.1")
    ap.add_argument("--out", default="report_tables.md")
    a = ap.parse_args()
    rm = load(a.train, "run_meta.json")
    zs, ft = load(a.zs, "summary.json"), load(a.ft, "summary.json")
    if not (rm or zs or ft):
        sys.exit("읽을 것이 없음 — --train · --zs · --ft 폴더에 run_meta.json / summary.json 이 있는지 확인")
    ckpt_info = sha12_size(a.ckpt) if a.ckpt and os.path.isfile(a.ckpt) else None
    parts = []
    if rm:
        curves = None
        cp = os.path.join(a.train, "curves.md")
        if os.path.isfile(cp):
            with open(cp, encoding="utf-8") as f:
                curves = f.read().strip()
        parts += ["## 표 E-14. 파인튜닝 결과", table_e14(rm, curves, a.version, ckpt_info),
                  "## MANIFEST checkpoint 행 초안", manifest_row(rm, a.version, ckpt_info)]
    if zs or ft:
        parts += ["## 표 E-15. test split 개루프 지표 (제로샷 · 파인튜닝)", table_e15(zs, ft)]
    text = "\n\n".join(parts) + "\n"
    with open(a.out, "w", encoding="utf-8") as f:   # 화면 출력이 실패해도 표 파일은 남게 먼저 씀
        f.write(text)
    print(text)
    print(f"→ {a.out}")


if __name__ == "__main__":
    main()
