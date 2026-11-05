#!/usr/bin/env python3
"""학습 곡선 — e2e/finetune.py 의 <out>/csv/metrics.csv(Lightning CSVLogger) → 에폭별 검증 표 + 그림.

  표(화면 · <out>/curves.md): 에폭마다 val_loss · 명령 손실(best ckpt 기준) · 경로 · KL 손실, 선속도 · 각속도 · 경로 MAE
  그림(<out>/curves.png, matplotlib 있을 때만): 학습 손실(스텝, 이동 평균) · 검증 손실(에폭) · 검증 MAE(에폭)

열 이름은 X-Mobility trainer 가 남기는 것(`{train,validation}/losses/{action,path,kl}`, `validation/metrics/action/*`) —
없는 열은 건너뜀. 사용: python3 e2e/plot_training.py <finetune 출력 폴더> [--smooth 50]
필요: pandas (그림은 matplotlib 선택).
"""
import argparse
import os
import sys

import pandas as pd

# 그림 글자는 영어(Kaggle · 서버 기본 글꼴에 한글이 없음)
PLOT_NAME = {"val_loss": "val_loss", "명령": "action", "경로": "path", "KL": "kl", "v MAE": "v MAE",
             "ω MAE": "w MAE", "경로 MAE": "path MAE"}
VAL_COLS = [("val_loss", "val_loss"), ("validation/losses/action", "명령"), ("validation/losses/path", "경로"),
            ("validation/losses/kl", "KL"), ("validation/metrics/action/linear_mae", "v MAE"),
            ("validation/metrics/action/angular_mae", "ω MAE"), ("validation/metrics/action/path_mae", "경로 MAE")]
TRAIN_COLS = ["train/losses/action", "train/losses/path", "train/losses/kl"]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out", help="finetune.py 의 -o 폴더")
    ap.add_argument("--smooth", type=int, default=50, help="학습 손실 이동 평균 창(스텝)")
    a = ap.parse_args()
    path = os.path.join(a.out, "csv", "metrics.csv")
    if not os.path.isfile(path):
        sys.exit(f"없음: {path} — finetune.py(CSVLogger 들어간 판)로 학습한 폴더인지 확인")
    df = pd.read_csv(path)

    cols = [(c, n) for c, n in VAL_COLS if c in df.columns]
    if "epoch" not in df.columns or not cols:
        sys.exit(f"검증 열이 없음 — 열: {list(df.columns)[:20]}")
    val = df[["epoch"] + [c for c, _ in cols]].dropna(subset=[c for c, _ in cols], how="all")
    val = val.groupby("epoch", as_index=False).last()
    lines = ["| 에폭 | " + " | ".join(n for _, n in cols) + " |", "| --- |" + " --- |" * len(cols)]
    for _, r in val.iterrows():
        lines.append(f"| {int(r['epoch'])} | " + " | ".join("" if pd.isna(r[c]) else f"{r[c]:.4f}" for c, _ in cols) + " |")
    best = None
    if "validation/losses/action" in val.columns and val["validation/losses/action"].notna().any():
        best = val.loc[val["validation/losses/action"].idxmin()]
        lines.append(f"\n명령 손실 최소(best ckpt 기준): 에폭 {int(best['epoch'])} · {best['validation/losses/action']:.4f}")
    text = "\n".join(lines)
    print(text)
    with open(os.path.join(a.out, "curves.md"), "w", encoding="utf-8") as f:
        f.write(text + "\n")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib 없음 — 그림 건너뜀")
        return
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))
    for c in TRAIN_COLS:
        if c in df.columns and "step" in df.columns:
            t = df[["step", c]].dropna()
            ax[0].plot(t["step"], t[c].rolling(a.smooth, min_periods=1).mean(), label=c.split("/")[-1])
    ax[0].set_title(f"train loss (step, moving avg {a.smooth})")
    ax[0].set_xlabel("step")
    for c, n in cols:
        if c.startswith("validation/losses") or c == "val_loss":
            ax[1].plot(val["epoch"], val[c], marker="o", label=PLOT_NAME[n])
        elif c.startswith("validation/metrics"):
            ax[2].plot(val["epoch"], val[c], marker="o", label=PLOT_NAME[n])
    ax[1].set_title("validation loss (epoch)")
    ax[2].set_title("validation MAE (epoch)")
    for x in ax:
        x.legend()
        x.grid(alpha=0.3)
    ax[1].set_xlabel("epoch")
    ax[2].set_xlabel("epoch")
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "curves.png"), dpi=120)
    print(f"그림: {os.path.join(a.out, 'curves.png')}")


if __name__ == "__main__":
    main()
