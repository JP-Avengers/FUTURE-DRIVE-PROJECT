"""충돌 메쉬의 겹친 바닥을 한 겹으로 — C 주차장 USD 의 Collision 을 덮어쓰는 레이어를 만든다(D 임시 보정).

C 의 Collision 은 지형 메쉬(TSDF 를 줄인 것) + 지도 전체 바닥 면 + 부분 바닥 면을 합친 것이라, 바닥이 몇 mm~몇 cm 간격으로
여러 겹이다(맵 v0.4b 회차 0 경로 바퀴 자리 표본의 85 % 이상). 이 상태로 로봇을 몰면 Isaac Sim 6.0.1 의 PhysX(CPU)가
특정 지점에서 끝나지 않는 계산에 빠져 시뮬이 멈췄다(10.12, 두 곳 재현 · 위치 고정 확인).
여기서는 '지도 전체를 덮고 모두 위를 보는 성분'(바닥 면)을 바닥으로 쓰고, 나머지는 그 바닥보다 THRESH 넘게 솟은 삼각형
(차 · 연석 · 기둥 등)과 바닥 면 밖의 삼각형만 남긴다. 점(points)은 그대로, faceVertexIndices/Counts 만 덮어쓴다.
C 가 한 겹 바닥으로 내보내면 이 레이어는 뺀다.

Isaac Sim 을 띄우지 않고 pxr 만 쓴다(1분 안팎). 레포 루트(= $CD1)에서:
    envs/isaacsim_env/bin/python sim/make_single_floor.py data/usd/parking_v0.4b/usd/parking_mesh.usdc \
        data/usd/parking_v0.4b_dfix/collision_onefloor.usdc [THRESH=0.05]
"""
import math
import os
import sys

import numpy as np
from pxr import Sdf, Usd, UsdGeom, Vt

SRC, OUT = sys.argv[1], sys.argv[2]
TH = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
st = Usd.Stage.Open(SRC)
m = UsdGeom.Mesh(st.GetDefaultPrim().GetChild("Collision"))
P = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
F = np.array(m.GetFaceVertexIndicesAttr().Get(), dtype=np.int64).reshape(-1, 3)

# 연결 성분 — 지도 전체를 덮고 전부 위를 보는 성분을 바닥 면으로
par = np.arange(len(P))


def find(a):
    r = a
    while par[r] != r:
        r = par[r]
    while par[a] != r:
        par[a], a = r, par[a]
    return r


for a, b, c in F:
    ra, rb = find(a), find(b)
    if ra != rb:
        par[rb] = ra
    ra, rc = find(a), find(c)
    if ra != rc:
        par[rc] = ra
froot = np.array([find(v) for v in F[:, 0]])
N = np.cross(P[F[:, 1]] - P[F[:, 0]], P[F[:, 2]] - P[F[:, 0]])
NZ = N[:, 2] / np.maximum(np.linalg.norm(N, axis=1), 1e-30)
xmin, ymin = P[:, 0].min(), P[:, 1].min()
xmax, ymax = P[:, 0].max(), P[:, 1].max()
best = None
for r in np.unique(froot):
    sel = froot == r
    T = P[F[sel]]
    cover = (T[:, :, 0].max() - T[:, :, 0].min()) * (T[:, :, 1].max() - T[:, :, 1].min()) / ((xmax - xmin) * (ymax - ymin))
    up = (NZ[sel] > 0.5).mean()
    if cover > 0.9 and up > 0.99 and (best is None or sel.sum() > best[1]):
        best = (r, sel.sum())
floor = froot == best[0]
print(f"floor component: {floor.sum()} tris, others {(~floor).sum()}")

# 바닥 높이 격자(0.1 m) — 바닥 면 삼각형을 래스터화
S = 0.1
nx, ny = int(math.ceil((xmax - xmin) / S)) + 1, int(math.ceil((ymax - ymin) / S)) + 1
H = np.full((nx, ny), np.nan)
for f in F[floor]:
    p0, p1, p2 = P[f]
    i0, i1 = int(math.floor((min(p0[0], p1[0], p2[0]) - xmin) / S)), int(math.ceil((max(p0[0], p1[0], p2[0]) - xmin) / S))
    j0, j1 = int(math.floor((min(p0[1], p1[1], p2[1]) - ymin) / S)), int(math.ceil((max(p0[1], p1[1], p2[1]) - ymin) / S))
    gi, gj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij")
    x, y = xmin + gi * S, ymin + gj * S
    d = (p1[1] - p2[1]) * (p0[0] - p2[0]) + (p2[0] - p1[0]) * (p0[1] - p2[1])
    if abs(d) < 1e-14:
        continue
    l0 = ((p1[1] - p2[1]) * (x - p2[0]) + (p2[0] - p1[0]) * (y - p2[1])) / d
    l1 = ((p2[1] - p0[1]) * (x - p2[0]) + (p0[0] - p2[0]) * (y - p2[1])) / d
    l2 = 1 - l0 - l1
    ok = (l0 >= -1e-9) & (l1 >= -1e-9) & (l2 >= -1e-9) & (gi < nx) & (gj < ny)
    H[gi[ok], gj[ok]] = (l0 * p0[2] + l1 * p1[2] + l2 * p2[2])[ok]
print(f"floor grid {nx}x{ny}, filled {np.isfinite(H).mean() * 100:.1f}%")


def floor_at(x, y):
    i = np.clip(np.round((x - xmin) / S).astype(int), 0, nx - 1)
    j = np.clip(np.round((y - ymin) / S).astype(int), 0, ny - 1)
    return H[i, j]


others = np.where(~floor)[0]
V = P[F[others]]                       # (n,3,3)
fz = floor_at(V[:, :, 0], V[:, :, 1])  # (n,3)
above = np.nanmax(V[:, :, 2] - fz, axis=1)
no_floor = np.all(np.isnan(fz), axis=1)  # 바닥 면 밖(가장자리) — 남긴다
keep_o = others[(above > TH) | no_floor]
keep = np.sort(np.concatenate([np.where(floor)[0], keep_o]))
print(f"others kept {len(keep_o)} / {len(others)} (> {TH} m above floor or off-floor); total {len(keep)} / {len(F)}")
if os.path.exists(OUT):
    sys.exit(f"이미 있음 — 덮어쓰지 않음: {OUT}")
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
lyr = Sdf.Layer.CreateNew(OUT)
lyr.customLayerData = {"generated_by": "sim/make_single_floor.py", "source": os.path.relpath(SRC), "thresh_m": TH}
s2 = Usd.Stage.Open(lyr)
o = UsdGeom.Mesh(s2.OverridePrim("/World/Parking/Collision"))
o.CreateFaceVertexCountsAttr().Set(Vt.IntArray([3] * len(keep)))
o.CreateFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(F[keep].reshape(-1).astype(np.int32)))
lyr.Save()
print("saved", OUT)
