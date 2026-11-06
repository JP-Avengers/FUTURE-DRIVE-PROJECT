"""교사 조향 비교 — 같은 자리(경로 s 칸 · 방향)에서 시나리오(교사)마다 ω 를 얼마나 다르게 주는지 (E)

  왜    v0.1 이 Nav2 녹화에서 '덜 꺾는' 원인 확인용. 장애물 회피 방향이 섞여서인지(장애물 근처), 교사 둘이
        같은 코너를 다르게 돌아서인지(장애물 먼 곳)를 데이터만으로 가름. 모델은 어느 교사인지 모르므로
        같은 자리에서 교사끼리 ω 가 다르면 둘의 평균을 배움.
  출력  ① 파일별 중앙값: 급회전(|ω|>0.3) 비율 · 초당 ω 부호 바뀜 · 평균 |dω/dt| · 최대 |ω|
        ② 같은 자리 비교: route_s 칸(기본 2 m) × 방향(cw/ccw)마다 시나리오별 평균 · 표준편차 ω (회전 큰 칸만)
        ③ 급회전 프레임 중 장애물 NEAR m 밖 비율
  ω     driving_command[5](sim/export_xmobility.py), 프레임 간격 0.2 s

  python e2e/teacher_style.py C:/Users/hennr/Downloads/xm_v1_1 -o e2e/results/teacher_style_xm_v1_1.md
"""
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd

W_IDX, DT, BIG, NEAR = 5, 0.2, 0.3, 3.0


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("root", help="데이터 폴더(<root>/<split>/<시나리오>/*.pqt, split 모두 사용)")
    ap.add_argument("-s", "--scenario", action="append", help="비교할 시나리오(기본: 전부 · r3_nav2_seg 같은 구간 녹화는 빼는 게 나음)")
    ap.add_argument("--bin", type=float, default=2.0, help="route_s 칸 크기 m")
    ap.add_argument("--min-n", type=int, default=15, help="칸마다 시나리오별 최소 프레임")
    ap.add_argument("-o", "--out", help="결과 .md")
    a = ap.parse_args()
    scen = a.scenario or sorted({os.path.basename(p) for p in glob.glob(os.path.join(a.root, "*", "*")) if os.path.isdir(p)})
    per_file, frames = [], []
    for sc in scen:
        for p in sorted(glob.glob(os.path.join(a.root, "*", sc, "*.pqt"))):
            x = pd.read_parquet(p, columns=["driving_command", "route_s", "obstacle_dist"])
            w = np.stack(x.driving_command.values)[:, W_IDX]
            per_file.append(dict(scenario=sc, big=(np.abs(w) > BIG).mean(),
                                 flips_per_s=(np.diff(np.sign(w[np.abs(w) > 0.05])) != 0).sum() / (len(w) * DT),
                                 dwdt=np.abs(np.diff(w)).mean() / DT, max_w=np.abs(w).max()))
            frames.append(pd.DataFrame(dict(scenario=sc, dir="ccw" if "_ccw" in os.path.basename(p) else "cw",
                                            s=np.floor(x.route_s.values / a.bin) * a.bin, w=w,
                                            od=pd.to_numeric(x.obstacle_dist, errors="coerce").values)))
    f = pd.DataFrame(per_file)
    d = pd.concat(frames)
    out = [f"# 교사 조향 비교 — {a.root}", "", "## ① 파일별 중앙값", "",
           "| 시나리오 | 파일 | 급회전 비율 | 초당 부호 바뀜 | 평균 dω/dt | 최대 ω |", "|---|---|---|---|---|---|"]
    for sc, g in f.groupby("scenario"):
        m = g.median(numeric_only=True)
        out.append(f"| {sc} | {len(g)} | {m.big:.3f} | {m.flips_per_s:.2f} | {m.dwdt:.3f} | {m.max_w:.2f} |")
    g = d.groupby(["dir", "s", "scenario"]).w.agg(["mean", "std", "count"]).unstack("scenario")
    g = g[(g["count"] >= a.min_n).all(axis=1)]
    g = g[(g["mean"].abs() > 0.2).any(axis=1)]
    out += ["", f"## ② 같은 자리 평균 ω (± 표준편차) — {a.bin:g} m 칸 · 방향별, 어느 교사든 |평균|>0.2 인 칸", "",
            "| 방향 | s (m) | " + " | ".join(scen) + " |", "|---|---|" + "---|" * len(scen)]
    for (dr, s), r in g.iterrows():
        out.append(f"| {dr} | {s:g} | " + " | ".join(f"{r['mean'][sc]:+.2f} ± {r['std'][sc]:.2f}" for sc in scen) + " |")
    out += ["", f"## ③ 급회전(|ω|>{BIG}) 중 장애물 {NEAR:g} m 밖", ""]
    for sc in scen:
        b = d[(d.scenario == sc) & (d.w.abs() > BIG)]
        known = b.od.notna()
        if known.any():
            out.append(f"- {sc}: {(b.od[known] >= NEAR).sum()} / {known.sum()} ({100 * (b.od[known] >= NEAR).mean():.0f} %)")
        else:
            out.append(f"- {sc}: obstacle_dist 없음")
    txt = "\n".join(out)
    print(txt)
    if a.out:
        with open(a.out, "w", encoding="utf-8") as fh:
            fh.write(txt + "\n")


if __name__ == "__main__":
    main()
