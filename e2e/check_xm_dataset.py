#!/usr/bin/env python3
"""X-Mobility 학습 형식(xm_v1 등) 점검 — 학습을 돌리기 전에 X-Mobility IsaacSimDataset 로더 계약을 CPU 로 확인한다.

로더(X-Mobility 5837aea model/dataset/isaac_sim_dataset.py)가 실제로 가정하는 것만 검사:
  구조   <root>/{train,val,test}/<scenario>/*.pqt — split 폴더 안에 파일이 하나라도 있으면 로더가 os.listdir(파일)로 죽음
  열     driving_command(6) · ego_speed(스칼라) · path(10 = 5점×2) · camera_image(JPEG) · route_poses(40 = 20점×2)
  이미지 JPEG RGB 512×320 — 로더가 INPUT_IMAGE_SIZE (320, 512) 로 다시 줄이므로 크기가 다르면 비율이 틀어짐
  시퀀스 한 파일 안 연속 행을 겹치지 않게 sequence_length 개씩 자름 → 파일마다 n % L 행은 버려짐
그 밖에 이 프로젝트 쪽 약속:
  driving_command = [v, 0, 0, 0, 0, ω] (가운데 4칸 0) · |v| ≤ 0.8 · |ω| ≤ 0.7 (topics.md 상한, 넘으면 경고)
  route_poses · path 첫 점 ≈ (0, 0) (로봇 좌표계 원점 = 현재 위치, sim/export_xmobility.py)
  같은 녹화 묶음이 둘 이상의 split 에 들어가지 않음(파일 이름 run_<묶음>_<NN>.pqt)

사용: python3 e2e/check_xm_dataset.py <root> [--seq-len 4] [--image-stride 1] [--json out.json]
필요: numpy · pandas · pyarrow · pillow (torch 불필요). 오류가 있으면 종료 코드 1.
"""
import argparse
import io
import json
import math
import os
import re
import sys
from collections import Counter, defaultdict

import numpy as np
import pandas as pd
from PIL import Image

SPLITS = ("train", "val", "test")
REQUIRED = ["driving_command", "ego_speed", "path", "camera_image", "route_poses"]
LEN = {"driving_command": 6, "path": 10, "route_poses": 40}
IMG_W, IMG_H = 512, 320
V_MAX, W_MAX, EPS = 0.8, 0.7, 1e-3
ORIGIN_TOL = 0.05          # m — 첫 점이 원점에서 이보다 멀면 경고
RUN_RE = re.compile(r"^run_(.+)_(\d+)\.pqt$")


class Report:
    def __init__(self):
        self.errors, self.warnings = [], []

    def err(self, msg):
        self.errors.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)


def stack(col, n, name, where, rep):
    """object 열(배열 리스트)을 (rows, n) 로. 길이가 틀린 행이 있으면 오류 후 None."""
    lens = col.map(lambda a: len(a) if a is not None else -1)
    bad = lens != n
    if bad.any():
        rep.err(f"{where}: {name} 길이 {n} 아닌 행 {int(bad.sum())}개 (예: {sorted(set(lens[bad].tolist()))[:5]})")
        return None
    return np.stack(col.to_numpy()).astype(np.float64)


def check_file(path, where, a, rep):
    try:
        df = pd.read_parquet(path, engine="pyarrow")
    except Exception as e:  # noqa: BLE001 — 깨진 파일 이름과 원인을 그대로 보고
        rep.err(f"{where}: 읽기 실패 {e}")
        return None
    miss = [c for c in REQUIRED if c not in df.columns]
    if miss:
        rep.err(f"{where}: 필수 열 없음 {miss}")
        return None
    n = len(df)
    st = {"rows": n, "usable_seq": n // a.seq_len, "dropped_rows": n % a.seq_len}
    if n < a.seq_len:
        rep.warn(f"{where}: 행 {n} < sequence_length {a.seq_len} — 이 파일은 학습에 안 쓰임")

    cmd = stack(df["driving_command"], LEN["driving_command"], "driving_command", where, rep)
    rp = stack(df["route_poses"], LEN["route_poses"], "route_poses", where, rep)
    pa = stack(df["path"], LEN["path"], "path", where, rep)
    spd = pd.to_numeric(df["ego_speed"], errors="coerce").to_numpy(np.float64)

    for name, arr in (("driving_command", cmd), ("route_poses", rp), ("path", pa), ("ego_speed", spd)):
        if arr is not None and not np.isfinite(arr).all():
            rep.err(f"{where}: {name} 에 NaN/Inf {int((~np.isfinite(arr)).sum())}개")

    if cmd is not None:
        mid = np.abs(cmd[:, 1:5]).max() if n else 0.0
        if mid > 1e-9:
            rep.err(f"{where}: driving_command 가운데 4칸(vy, vz, wx, wy)이 0 이 아님 (최대 {mid:.3g})")
        v, w = cmd[:, 0], cmd[:, 5]
        nv, nw = int((np.abs(v) > V_MAX + EPS).sum()), int((np.abs(w) > W_MAX + EPS).sum())
        if nv or nw:
            rep.warn(f"{where}: 상한 초과 |v|>{V_MAX} {nv}행 · |ω|>{W_MAX} {nw}행")
        st.update(v=v, w=w)
    for name, arr in (("route_poses", rp), ("path", pa)):
        if arr is not None and n:
            d0 = np.hypot(arr[:, 0], arr[:, 1])
            k = int((d0 > ORIGIN_TOL).sum())
            if k:
                rep.warn(f"{where}: {name} 첫 점이 원점에서 {ORIGIN_TOL} m 넘게 떨어진 행 {k}개 (최대 {d0.max():.3f} m)")
    st["speed"] = spd

    bad_img = Counter()
    for i in range(0, n, a.image_stride):
        b = df["camera_image"].iat[i]
        try:
            im = Image.open(io.BytesIO(b))
            if im.format != "JPEG":
                bad_img[f"형식 {im.format}"] += 1
            if im.size != (IMG_W, IMG_H):
                bad_img[f"크기 {im.size[0]}×{im.size[1]}"] += 1
            if im.mode != "RGB":
                bad_img[f"모드 {im.mode}"] += 1
            im.load()
        except Exception as e:  # noqa: BLE001
            bad_img[f"디코드 실패 {type(e).__name__}"] += 1
    if bad_img:
        rep.err(f"{where}: camera_image 문제 {dict(bad_img)}")

    for opt in ("route_command", "obstacle_dist", "teacher_noise", "cte"):
        if opt in df.columns:
            st[opt] = pd.to_numeric(df[opt], errors="coerce").to_numpy(np.float64)
    return st


def summarize(sts):
    out = {"files": len(sts), "rows": sum(s["rows"] for s in sts),
           "usable_seq": sum(s["usable_seq"] for s in sts), "dropped_rows": sum(s["dropped_rows"] for s in sts)}
    cat = lambda k: np.concatenate([s[k] for s in sts if k in s]) if any(k in s for s in sts) else None  # noqa: E731
    v, w, spd = cat("v"), cat("w"), cat("speed")
    if v is not None and len(v):
        out.update(v_mean=float(v.mean()), v_p95=float(np.percentile(v, 95)),
                   w_abs_mean=float(np.abs(w).mean()),
                   w_left=float((w > 0.1).mean()), w_right=float((w < -0.1).mean()),
                   stopped=float((np.abs(v) < 0.05).mean()))
    if spd is not None and len(spd):
        out["speed_mean"] = float(np.nanmean(spd))
    rc = cat("route_command")
    if rc is not None:
        c = Counter(int(x) for x in rc[np.isfinite(rc)])
        out["route_command"] = {str(k): c[k] for k in sorted(c)}
    od = cat("obstacle_dist")
    if od is not None:
        fin = od[np.isfinite(od)]
        out["obstacle_near_1p5m"] = int((fin < 1.5).sum())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("root")
    ap.add_argument("--seq-len", type=int, default=4, help="gin SEQUENCE_LENGTH (기본 4)")
    ap.add_argument("--image-stride", type=int, default=1, help="N 행마다 JPEG 디코드 검사(기본 전부)")
    ap.add_argument("--json", help="요약을 JSON 으로 저장")
    a = ap.parse_args()
    rep = Report()

    if not os.path.isdir(a.root):
        sys.exit(f"폴더 없음: {a.root}")
    per = defaultdict(list)              # (split, scenario) → [stats]
    bundle_split = defaultdict(set)
    for sp in SPLITS:
        spath = os.path.join(a.root, sp)
        if not os.path.isdir(spath):
            rep.err(f"{sp}/ 없음 — 로더가 {sp} 단계에서 실패")
            continue
        for ent in sorted(os.listdir(spath)):
            p = os.path.join(spath, ent)
            if not os.path.isdir(p):
                rep.err(f"{sp}/{ent}: split 폴더 안의 파일 — 로더가 os.listdir 로 죽음, 옮기거나 지우세요")
                continue
            files = sorted(f for f in os.listdir(p) if f.endswith("pqt"))
            if not files:
                rep.warn(f"{sp}/{ent}/: .pqt 없음")
            for f in files:
                m = RUN_RE.match(f)
                if m:
                    bundle_split[m.group(1)].add(sp)
                st = check_file(os.path.join(p, f), f"{sp}/{ent}/{f}", a, rep)
                if st is not None:
                    per[(sp, ent)].append(st)

    for b, sps in sorted(bundle_split.items()):
        if len(sps) > 1:
            rep.err(f"녹화 묶음 {b} 가 여러 split 에 있음 {sorted(sps)} — 평가 누수")

    summary = {"root": os.path.abspath(a.root), "seq_len": a.seq_len,
               "groups": {f"{sp}/{sc}": summarize(v) for (sp, sc), v in sorted(per.items())},
               "splits": {sp: summarize([s for (k, _), v in per.items() if k == sp for s in v]) for sp in SPLITS},
               "errors": rep.errors, "warnings": rep.warnings}

    print(f"점검: {summary['root']}  (sequence_length {a.seq_len})")
    for sp in SPLITS:
        s = summary["splits"][sp]
        if not s["files"]:
            continue
        print(f"  {sp:5s} 파일 {s['files']:4d} · 행 {s['rows']:6d} · 시퀀스 {s['usable_seq']:6d} · 버려지는 행 {s['dropped_rows']}")
    for g, s in summary["groups"].items():
        extra = ""
        if "v_mean" in s:
            extra = (f" · v 평균 {s['v_mean']:.2f} · |ω| 평균 {s['w_abs_mean']:.2f}"
                     f" · 좌 {s['w_left']:.0%} 우 {s['w_right']:.0%} · 정지 {s['stopped']:.0%}")
        if "route_command" in s:
            extra += f" · 지시 {s['route_command']}"
        print(f"    {g:28s} 행 {s['rows']:6d}{extra}")
    for w in rep.warnings:
        print(f"  경고: {w}")
    for e in rep.errors:
        print(f"  오류: {e}")
    print(f"결과: 오류 {len(rep.errors)} · 경고 {len(rep.warnings)}")
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=1, default=lambda x: None if isinstance(x, float) and math.isnan(x) else x)
    sys.exit(1 if rep.errors else 0)


if __name__ == "__main__":
    main()
