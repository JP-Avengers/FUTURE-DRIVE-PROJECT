"""make_xm_v3.py — [11.05 D] xm_v2 에서 Nav2 교사 녹화 앞머리의 '제자리 돌아서기' 구간을 뺀 xm_v3 만들기.
Nav2 반시계 녹화는 시계 방향 자세(−96°)로 출발해 첫 ~3 s 를 제자리 180° 회전에 씀 → 같은 출발 장면에 '출발' · '제자리 회전' 두 정답이 생겨
2차 모델이 출발 직후 제자리에서 좌우로 돌다 멈춤(11.05 평가 3회). 규칙: 각 Nav2 런의 앞에서부터, 경로 5번째 점(로봇 기준)의 방향이
정면 ±30° 안에 처음 들어오는 행 전까지를 버림. 그 밖 파일은 하드링크.
사용: python make_xm_v3.py [--dry]"""
import glob, math, os, shutil, sys
import numpy as np, pandas as pd
SRC = os.path.expanduser("~/Capstone_Design_1/data/teacher/xm_v2"); DST = SRC.replace("xm_v2", "xm_v3")
DRY = "--dry" in sys.argv
tot_drop = tot = 0
for f in sorted(glob.glob(f"{SRC}/*/*/*.pqt")):
    rel = os.path.relpath(f, SRC); out = f"{DST}/{rel}"
    scen = rel.split("/")[1]
    if not DRY: os.makedirs(os.path.dirname(out), exist_ok=True)
    k = 0
    if scen.startswith("r3_nav2"):
        rp = pd.read_parquet(f, columns=["route_poses"])["route_poses"].to_numpy()
        for i, r in enumerate(rp):
            p = np.asarray(list(r), dtype=float).reshape(20, -1)[5]
            if abs(math.degrees(math.atan2(p[1], p[0]))) < 30: k = i; break
        else: k = 0
    n = len(pd.read_parquet(f, columns=["acqtime"]))
    tot += n; tot_drop += k
    if k: print(f"  {rel}: 앞 {k}행({k/5:.1f} s) 뺌 / {n}")
    if DRY: continue
    if k:
        d = pd.read_parquet(f); d.iloc[k:].reset_index(drop=True).to_parquet(out, index=False)
    else:
        if os.path.exists(out): os.remove(out)
        os.link(f, out)
if not DRY:
    for g in glob.glob(f"{SRC}/index/*.json"):
        os.makedirs(f"{DST}/index", exist_ok=True); shutil.copy(g, f"{DST}/index/")
print(f"TOTAL 뺀 행 {tot_drop} / {tot} ({100*tot_drop/tot:.2f} %)", "DRY" if DRY else f"→ {DST}")
