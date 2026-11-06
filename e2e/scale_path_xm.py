"""경로 정답 축척 — 데이터 폴더를 복사하고 모든 split 의 `path` 열(5점 × (x, y), m)에 배수를 곱함 (E)

  왜    X-Mobility 경로 머리는 Tanh 라 ±1 m 를 못 넘는데(finetune.py --monitor 설명), 우리 path 정답(3.36 s 앞, 5점)은
        끝점이 중앙값 1.9 m · 최대 2.6 m 라 좌표의 25 % 가 |1| 을 넘음(xm_v1_1 r3_auto 한 파일 기준). 그래서 경로 손실(가중치 5)이
        줄일 수 없는 바닥(val 6.9)에 붙어 잠재 상태를 흔들기만 함. 1/3 로 줄이면 전부 ±1 안 → 경로 머리가 배울 수 있고,
        '경로 공간 감독이 폐루프에 좋다'는 보고(TCP · Waypoint–Action gap)에 맞춤. 구조 변경 없음(데이터만).
  방법  train · val · test 모두 같은 배수(평가도 같은 축척이어야 ADE 가 맞음). 원본은 읽기만. index/ 등 나머지는 그대로 복사.
  기록  <대상>/PATH_SCALE.json — scale · 원본 경로. eval_offline.py 는 이 파일이 있으면 ADE · FDE 를 scale 로 나눠 m 로 보고.
  주의  이 데이터로 학습한 ckpt 의 경로 출력은 ×(1/scale) 해야 m. 노드의 ~/path(디버그 발행)는 축척 그대로 나감.

  python e2e/scale_path_xm.py C:/Users/hennr/Downloads/xm_v1_2 C:/Users/hennr/Downloads/xm_v1_2_p3 --scale 0.3333333
"""
import argparse
import glob
import json
import os
import shutil
import sys

import numpy as np
import pandas as pd

SPLITS = ("train", "val", "test")


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src")
    ap.add_argument("dst", help="새 폴더(없어야 함)")
    ap.add_argument("--scale", type=float, default=1.0 / 3.0, help="path 에 곱할 배수(기본 1/3)")
    a = ap.parse_args()
    if os.path.exists(a.dst):
        sys.exit(f"대상이 이미 있음: {a.dst}")
    if not (0 < a.scale <= 1):
        sys.exit("--scale 은 0 < s <= 1")
    shutil.copytree(a.src, a.dst)
    files = [f for sp in SPLITS for f in sorted(glob.glob(os.path.join(a.dst, sp, "*", "*.pqt")))]
    if not files:
        sys.exit(f"{a.dst} 아래 <split>/<시나리오>/*.pqt 없음")
    n_rows, mx_before, mx_after = 0, 0.0, 0.0
    for f in files:
        df = pd.read_parquet(f)
        p = np.stack(df["path"].to_numpy()).astype(np.float64)
        mx_before = max(mx_before, float(np.abs(p).max()))
        q = p * a.scale
        mx_after = max(mx_after, float(np.abs(q).max()))
        df["path"] = list(q.astype(np.float32 if np.stack(df["path"].to_numpy()).dtype == np.float32 else np.float64))
        df.to_parquet(f, index=False)
        n_rows += len(df)
    rec = dict(src=os.path.abspath(a.src), scale=a.scale, files=len(files), rows=n_rows,
               path_abs_max_before=round(mx_before, 3), path_abs_max_after=round(mx_after, 3),
               note="path = 원본 × scale (train · val · test 모두). ADE · FDE(m) = 측정값 / scale")
    with open(os.path.join(a.dst, "PATH_SCALE.json"), "w", encoding="utf-8") as fh:
        json.dump(rec, fh, ensure_ascii=False, indent=1)
    print(f"{len(files)}개 파일 · {n_rows}행 · path × {a.scale:.4f} · |path| 최대 {mx_before:.2f} → {mx_after:.2f} m"
          + (" (아직 1 을 넘는 좌표가 있음)" if mx_after > 1 else "") + f" → {a.dst}")
    print(f"확인: python e2e/check_xm_dataset.py {a.dst} --image-stride 20")


if __name__ == "__main__":
    main()
