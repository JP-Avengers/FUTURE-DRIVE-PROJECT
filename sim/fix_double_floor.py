"""충돌 메쉬의 '바닥 위 납작한 조각'을 지정한 자리에서만 지우는 덮어쓰기 레이어 — D 임시 보정(Isaac Sim 없이 pxr 만).

r3 는 C 가 바닥을 한 겹으로 정리했지만(촬영 궤적 바퀴 자리 기준), 그 밖에 바닥 면과 따로 떨어진 작은 조각이 바닥 높이에
겹쳐 남은 곳이 있다. 잡음을 키운 자동 교사가 서쪽 끝 (−36.8, −7.35)의 6삼각형짜리 조각을 밟자 PhysX 가 끝나지 않는 계산에
빠져 시뮬이 멈췄다(10.13 — 처음 판 v0.4b 의 겹친 바닥 멈춤과 같은 증상).
바닥 면(지도 전체를 덮는 가장 큰 연결 성분)은 그대로 두고, 지정한 자리 반경 안에 있는 다른 성분 중 바닥 위 --th 이하로 납작하고
작은(--max-tris 이하) 것만 뺀다. 점은 그대로, /World/Parking/Collision 의 faceVertexIndices/Counts 만 덮어쓴다.
C 가 원본에서 고치면 이 레이어를 씬 루트 subLayers 에서 뺀다.

레포 루트(= $CD1)에서:
    envs/isaacsim_env/bin/python sim/fix_double_floor.py <parking_mesh.usdc> --list                  # 지도 전체의 납작한 조각 위치만
    envs/isaacsim_env/bin/python sim/fix_double_floor.py <parking_mesh.usdc> <out.usdc> [--th 0.08] -- x,y[,r] [...]  # 그 자리 조각을 뺀 레이어
(자리 앞의 -- 는 음수 좌표를 옵션으로 읽지 않게 하려는 것)
끝에 지운 성분 · 자리별 겹친 바닥 칸('=') 수 전후를 찍는다(column_map.py 와 같은 기준: 맨 위 면 아래 3 cm 안에 면이 한 겹 더).
"""
import argparse
import math
import os
from collections import defaultdict

import numpy as np
from pxr import Sdf, Usd, UsdGeom, Vt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("out", nargs="?")
    ap.add_argument("spots", nargs="*", help="x,y[,r] — 반경 r 기본 0.8 m")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--th", type=float, default=0.05, help="바닥 위 이 높이(m) 이하면 납작한 조각")
    ap.add_argument("--max-tris", type=int, default=200, help="이보다 큰 성분은 건드리지 않음")
    a = ap.parse_intermixed_args()  # 옵션이 자리 인자 사이에 와도 되게

    st = Usd.Stage.Open(a.src)
    m = UsdGeom.Mesh(st.GetDefaultPrim().GetChild("Collision"))
    P = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
    F = np.array(m.GetFaceVertexIndicesAttr().Get(), dtype=np.int64).reshape(-1, 3)
    if np.any(np.array(m.GetFaceVertexCountsAttr().Get()) != 3):
        raise SystemExit("삼각형 메쉬만 지원")

    # 연결 성분 — 가장 큰 성분 = 바닥 면
    par = np.arange(len(P))

    def find(x):
        r = x
        while par[r] != r:
            r = par[r]
        while par[x] != r:
            par[x], x = r, par[x]
        return r
    for i0, i1, i2 in F:
        r0, r1 = find(i0), find(i1)
        if r0 != r1:
            par[r1] = r0
        r0, r2 = find(i0), find(i2)
        if r0 != r2:
            par[r2] = r0
    froot = np.array([find(v) for v in F[:, 0]])
    roots, sizes = np.unique(froot, return_counts=True)
    floor_root = roots[np.argmax(sizes)]
    floor = froot == floor_root
    size_of = dict(zip(roots, sizes))

    # 바닥 높이 — 바닥 면 삼각형을 0.5 m 칸에 넣고 무게중심 좌표로
    CELL = 0.5
    T = P[F]
    mn, mx = T[:, :, :2].min(1), T[:, :, :2].max(1)
    grid = defaultdict(list)
    for i in np.where(floor)[0]:
        for gx in range(int(math.floor(mn[i, 0] / CELL)), int(math.floor(mx[i, 0] / CELL)) + 1):
            for gy in range(int(math.floor(mn[i, 1] / CELL)), int(math.floor(mx[i, 1] / CELL)) + 1):
                grid[(gx, gy)].append(i)

    def bary_z(i, x, y):
        p0, p1, p2 = T[i]
        d = (p1[1] - p2[1]) * (p0[0] - p2[0]) + (p2[0] - p1[0]) * (p0[1] - p2[1])
        if abs(d) < 1e-14:
            return None
        l0 = ((p1[1] - p2[1]) * (x - p2[0]) + (p2[0] - p1[0]) * (y - p2[1])) / d
        l1 = ((p2[1] - p0[1]) * (x - p2[0]) + (p0[0] - p2[0]) * (y - p2[1])) / d
        l2 = 1 - l0 - l1
        if min(l0, l1, l2) < -1e-9:
            return None
        return l0 * p0[2] + l1 * p1[2] + l2 * p2[2]

    def floor_z(x, y):
        zs = [z for i in grid.get((int(math.floor(x / CELL)), int(math.floor(y / CELL))), [])
              if (z := bary_z(i, x, y)) is not None]
        return max(zs) if zs else None

    # 납작한 작은 성분: 모든 꼭짓점이 바닥 위 --th 이하(바닥 밖 꼭짓점이 하나라도 있으면 제외)
    flat = {}
    for r in roots:
        if r == floor_root or size_of[r] > a.max_tris:
            continue
        idx = np.where(froot == r)[0]
        verts = np.unique(F[idx].reshape(-1))
        hmax, ok = -1e9, True
        for v in verts:
            fz = floor_z(P[v, 0], P[v, 1])
            if fz is None:
                ok = False
                break
            hmax = max(hmax, P[v, 2] - fz)
        if ok and hmax <= a.th:
            c = P[verts].mean(0)
            flat[r] = (idx, c, hmax)

    if a.list or not a.out:
        print(f"바닥 면 {size_of[floor_root]} 삼각형 · 다른 성분 {len(roots) - 1}개 · 바닥 위 {a.th} m 이하로 납작한 작은 성분 {len(flat)}개")
        for r, (idx, c, h) in sorted(flat.items(), key=lambda kv: (kv[1][1][0], kv[1][1][1])):
            print(f"  ({c[0]:.2f}, {c[1]:.2f}) · 삼각형 {len(idx)} · 바닥 위 최대 {h:.3f} m")
        return

    spots = []
    for s in a.spots:
        v = [float(t) for t in s.split(",")]
        spots.append((v[0], v[1], v[2] if len(v) > 2 else 0.8))
    if not spots:
        raise SystemExit("지울 자리(x,y[,r])를 하나 이상 주세요 — 지도 전체 목록은 --list")

    remove = np.zeros(len(F), dtype=bool)
    for r, (idx, c, h) in flat.items():
        for sx, sy, sr in spots:
            if math.hypot(c[0] - sx, c[1] - sy) <= sr:
                remove[idx] = True
                print(f"지움: 성분 중심 ({c[0]:.2f}, {c[1]:.2f}) · 삼각형 {len(idx)} · 바닥 위 최대 {h:.3f} m — 자리 ({sx}, {sy}, r {sr})")
    if not remove.any():
        raise SystemExit("지정한 자리에서 지울 납작한 조각을 못 찾음 — 아무것도 쓰지 않음")

    # 자리별 겹친 바닥 칸('=') 전후 — 0.05 m 격자
    def double_cells(keep):
        out = {}
        sel_all = np.where(keep)[0]
        for sx, sy, sr in spots:
            near = sel_all[(np.abs(T[sel_all, :, 0].mean(1) - sx) < sr + 0.5) & (np.abs(T[sel_all, :, 1].mean(1) - sy) < sr + 0.5)]
            nd = 0
            for x in np.arange(sx - sr, sx + sr + 1e-9, 0.05):
                for y in np.arange(sy - sr, sy + sr + 1e-9, 0.05):
                    zs = sorted({round(z, 4) for i in near if (z := bary_z(i, x, y)) is not None}, reverse=True)
                    if len(zs) > 1 and zs[0] - zs[1] < 0.03:
                        nd += 1
            out[(sx, sy)] = nd
        return out
    before = double_cells(np.ones(len(F), dtype=bool))
    after = double_cells(~remove)
    for k in before:
        print(f"자리 {k}: 겹친 바닥 칸(0.05 m 격자) {before[k]} → {after[k]}")

    if os.path.exists(a.out):
        raise SystemExit(f"이미 있음 — 덮어쓰지 않음: {a.out}")
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    keep = np.where(~remove)[0]
    lyr = Sdf.Layer.CreateNew(a.out)
    lyr.customLayerData = {"generated_by": "sim/fix_double_floor.py", "source": os.path.relpath(a.src),
                           "spots": ";".join(f"{x},{y},{r}" for x, y, r in spots), "thresh_m": a.th,
                           "removed_tris": int(remove.sum())}
    s2 = Usd.Stage.Open(lyr)
    o = UsdGeom.Mesh(s2.OverridePrim("/World/Parking/Collision"))
    o.CreateFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(keep)))
    o.CreateFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(F[keep].reshape(-1).astype(np.int32)))
    lyr.Save()
    print(f"저장 {a.out} — 삼각형 {len(F)} → {len(keep)} (−{int(remove.sum())})")


if __name__ == "__main__":
    main()
