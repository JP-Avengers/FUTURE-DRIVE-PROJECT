"""충돌 바닥 층수 점검 — 로봇이 지날 수 있는 띠에서 바닥이 한 겹인지 수직 광선으로 셈 (논문 '시뮬레이션 적합성 진단'의 정적 진단과 같은 생각).

    python recon/tools/check_floor_layers.py <mesh.usdc> [--prim Collision] [--route route.csv --band 2.35] [--box x0 x1 y0 y1] [--ray-top 1.0] [--out layers.json]

표본점: --route 가 있으면 경로 양옆 --band m 의 띠(진행 0.25 m · 옆 0.1 m 간격), --box 면 그 사각형(0.1 m 간격), 둘 다 없으면 충돌 메쉬 범위 전체(0.25 m 간격).
각 표본에서 --ray-top 높이(천장이 있는 실내는 천장 아래, 기본 1.0 m — 경사진 실외는 --route 의 z 가 없으므로 넉넉히 높게)에서 아래로 광선을 쏴 맨 위 면을 찾고,
그 면 바로 아래에 면이 더 있는지 봄:  3 cm 안 = 두 겹 바닥(PhysX 멈춤의 원인) · 3~30 cm 아래 = 아래 면(뒤집힌 면 · 뜬 조각의 밑).
합격 = 두 겹 표본 0. 종료 코드: 두 겹이 있으면 1.
"""
import argparse
import csv
import json
import sys

import numpy as np
import open3d as o3d
from pxr import Usd, UsdGeom


def samples(a, pts):
    if a.route:
        xy = np.array([(float(r["x"]), float(r["y"])) for r in csv.DictReader(open(a.route, encoding="utf-8"))])
        d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        s = np.arange(0, d[-1], 0.25)
        c = np.c_[np.interp(s, d, xy[:, 0]), np.interp(s, d, xy[:, 1])]
        t = np.gradient(c, axis=0)
        t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
        n = np.c_[-t[:, 1], t[:, 0]]
        off = np.arange(-a.band, a.band + 1e-6, 0.1)
        return (c[:, None, :] + off[None, :, None] * n[:, None, :]).reshape(-1, 2)
    if a.box:
        x0, x1, y0, y1 = a.box
        X, Y = np.meshgrid(np.arange(x0, x1, 0.1), np.arange(y0, y1, 0.1))
    else:
        lo, hi = pts[:, :2].min(0), pts[:, :2].max(0)
        X, Y = np.meshgrid(np.arange(lo[0], hi[0], 0.25), np.arange(lo[1], hi[1], 0.25))
    return np.c_[X.ravel(), Y.ravel()]


def measure(mesh, prim, xy, ztop):
    """충돌 메쉬 prim 에서 표본 xy 마다 ztop(숫자 또는 표본별 배열) 높이에서 아래로 쏜 광선으로 바닥 층수를 셈. → (결과 dict, 두 겹 표본 xy)"""
    st = Usd.Stage.Open(mesh)
    m = UsdGeom.Mesh(st.GetPrimAtPath(st.GetDefaultPrim().GetPath().AppendChild(prim)))
    pts = np.array(m.GetPointsAttr().Get(), np.float32)
    tri = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3).astype(np.uint32)
    sc = o3d.t.geometry.RaycastingScene()
    sc.add_triangles(o3d.core.Tensor(pts), o3d.core.Tensor(tri))
    if xy is None:
        lo, hi = pts[:, :2].min(0), pts[:, :2].max(0)
        X, Y = np.meshgrid(np.arange(lo[0], hi[0], 0.25), np.arange(lo[1], hi[1], 0.25))
        xy = np.c_[X.ravel(), Y.ravel()]
    ztop = np.broadcast_to(np.asarray(ztop(xy) if callable(ztop) else ztop, float), (len(xy),))

    def cast(z):
        r = np.c_[xy, z, np.zeros(len(xy)), np.zeros(len(xy)), -np.ones(len(xy))].astype(np.float32)
        return sc.cast_rays(o3d.core.Tensor(r))["t_hit"].numpy()
    h1 = cast(ztop)
    h2 = cast(ztop - h1 - 1e-4)                                 # 맨 위 면 바로 아래에서 다시
    ok = np.isfinite(h1)
    two = ok & np.isfinite(h2) & (h2 < 0.03)
    under = ok & np.isfinite(h2) & (h2 >= 0.03) & (h2 < 0.30)
    n = int(ok.sum())
    res = dict(mesh=mesh, prim=prim, samples=int(len(xy)), on_floor=n, single_pct=round(100 * float((ok & ~two & ~under).sum()) / max(n, 1), 2),
               double_within_3cm=int(two.sum()), double_pct=round(100 * float(two.sum()) / max(n, 1), 2), double_gap_mm_median=round(float(np.median(h2[two]) * 1000), 1) if two.any() else None,
               under_3_to_30cm=int(under.sum()), under_pct=round(100 * float(under.sum()) / max(n, 1), 2), passed=not bool(two.any()))
    return res, xy[two]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mesh")
    ap.add_argument("--prim", default="Collision")
    ap.add_argument("--route")
    ap.add_argument("--band", type=float, default=2.35)
    ap.add_argument("--box", type=float, nargs=4)
    ap.add_argument("--ray-top", type=float, default=1.0)
    ap.add_argument("--out")
    a = ap.parse_args()
    res, w = measure(a.mesh, a.prim, samples(a, None) if (a.route or a.box) else None, a.ray_top)
    print(f"[바닥 층수] 표본 {res['samples']:,} 중 바닥 위 {res['on_floor']:,} · 한 겹 {res['single_pct']}% · 두 겹(3 cm 안) {res['double_within_3cm']} ({res['double_pct']}%"
          + (f", 간격 중앙 {res['double_gap_mm_median']} mm" if len(w) else "") + f") · 아래 면(3~30 cm) {res['under_3_to_30cm']} ({res['under_pct']}%) → {'합격' if res['passed'] else '불합격'}")
    if len(w):
        print("   두 겹 자리 (앞 5개):", np.round(w[:5], 2).tolist())
    if a.out:
        json.dump(res, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    sys.exit(0 if res["passed"] else 1)


if __name__ == "__main__":
    main()
