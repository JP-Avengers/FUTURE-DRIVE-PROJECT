"""충돌 메쉬로 Nav2 지도(PGM + YAML)를 만든다 — 시뮬과 같은 형상이라 지도 오차가 없는 '정답 지도'.

장애물 = 바닥에서 --h-lo ~ --h-hi(기본 0.12 ~ 1.0 m) 높이에 있는 면. 1.0 m 는 Nova Carter 키(약 0.75 m)에 여유를 둔 값이라
그보다 높이 떠 있는 나뭇가지 · 잔상(남서 모서리 물체는 바닥 위 0.85 m 부터)은 장애물로 안 친다. 연석(18 cm)은 장애물.
바닥 높이 = 위를 보는 면(nz > 0.9)의 최저 높이(0.5 m 격자) — logs/refline/analyze_corridor.py 와 같은 기준.
경계벽(Boundary prim)은 높이와 상관없이 장애물. 바닥이 없는 칸은 미지(unknown).
좌표는 씬 월드 = B 맵 좌표(x · y) — Nav2 의 map 프레임이 이 좌표가 된다(기준선 eval/routes/parking_r3_*.csv 와 같음).

레포 루트(= $CD1)에서:
    envs/isaacsim_env/bin/python sim/nav2/make_map.py data/usd/parking_v0.4b_r3/usd/parking_mesh.usdc data/nav2/parking_v0.4b_r3
→ <out>/map.pgm · map.yaml(Nav2 map_server 형식 — 형상 그대로, 위치 추정(AMCL)용) · preview.png(기준선 겹침, 확인용)
--keepout eval/routes/parking_r3_island.yaml 이면 가운데 화단 윤곽(island 꼭짓점)을 장애물로 채운 map_keepout.pgm · .yaml 도 쓴다
(경로 계획 · 비용 지도용) — 화단 안은 메쉬로는 거의 비어 있어(북쪽 경계석이 낮음) 경로가 화단을 가로지를 수 있으므로.
AMCL 에는 채우지 않은 map.yaml 을 준다: LiDAR 가 못 보는 큰 장애물 덩어리가 있으면 엉뚱한 자세의 스캔도 거기 걸려 점수가 높아져
먼 곳의 가짜 무리가 살아남고 위치가 튐(10.14 — 10 m 어긋난 자세 점수 비 0.56 → 0.19).
"""
import argparse
import csv
import os
import struct
import zlib

import numpy as np
import yaml
from pxr import Usd, UsdGeom

ap = argparse.ArgumentParser()
ap.add_argument("mesh")
ap.add_argument("out")
ap.add_argument("--cell", type=float, default=0.05)
ap.add_argument("--h-lo", type=float, default=0.12)
ap.add_argument("--h-hi", type=float, default=1.0)
ap.add_argument("--margin", type=float, default=2.0)
ap.add_argument("--route", default="eval/routes/parking_r3_cw.csv", help="미리보기에 겹칠 기준선(없으면 생략)")
ap.add_argument("--keepout", help="island 꼭짓점이 든 기준선 정의 yaml — 그 다각형 안을 장애물로")
ap.add_argument("--prim", default="Collision", help="장애물을 뽑을 메쉬 prim — Collision(기본) 또는 Visual(LiDAR 가 보는 형상)")
a = ap.parse_args()
FCELL = 0.5


def tris(prim):
    m = UsdGeom.Mesh(prim)
    P = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
    cnt = np.array(m.GetFaceVertexCountsAttr().Get())
    assert (cnt == 3).all(), "삼각형만 가정"
    F = np.array(m.GetFaceVertexIndicesAttr().Get(), dtype=np.int64).reshape(-1, 3)
    M = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=np.float64)
    P = (np.hstack([P, np.ones((len(P), 1))]) @ M)[:, :3]   # 행 벡터 규약
    return P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]


def samples(A, B, C, step):
    """삼각형마다 step 간격 무게중심 좌표 점(세운 면도 x · y 자취가 남음)."""
    edge = np.maximum.reduce([np.linalg.norm(B - A, axis=1), np.linalg.norm(C - B, axis=1), np.linalg.norm(A - C, axis=1)])
    nsub = np.clip(np.ceil(edge / step).astype(int), 1, 80)
    for k in np.unique(nsub):
        sel = nsub == k
        uv = np.array([(i / k, j / k) for i in range(k + 1) for j in range(k + 1 - i)])
        u, v = uv[:, 0], uv[:, 1]
        yield (A[sel][:, None] * (1 - u - v)[None, :, None] + B[sel][:, None] * u[None, :, None]
               + C[sel][:, None] * v[None, :, None]).reshape(-1, 3)


st = Usd.Stage.Open(a.mesh)
root = st.GetDefaultPrim()
coll = next(p for p in Usd.PrimRange(root) if p.GetName() == a.prim)
bnd = next((p for p in Usd.PrimRange(root) if p.GetName() == "Boundary"), None)
A, B, C = tris(coll)
x0, y0 = np.minimum.reduce([A, B, C]).min(0)[:2] - a.margin
x1, y1 = np.maximum.reduce([A, B, C]).max(0)[:2] + a.margin
nx, ny = int(np.ceil((x1 - x0) / a.cell)), int(np.ceil((y1 - y0) / a.cell))

# 바닥 높이(0.5 m 격자) — 위를 보는 면의 꼭짓점 최저 높이, 빈 칸은 이웃 값으로 몇 번 메움
N = np.cross(B - A, C - A)
up = N[:, 2] / np.maximum(np.linalg.norm(N, axis=1), 1e-30) > 0.9
fl = np.full((int((x1 - x0) / FCELL) + 1, int((y1 - y0) / FCELL) + 1), np.inf)
V = np.vstack([A[up], B[up], C[up]])
np.minimum.at(fl, (((V[:, 0] - x0) / FCELL).astype(int), ((V[:, 1] - y0) / FCELL).astype(int)), V[:, 2])
for _ in range(6):
    pad = np.pad(fl, 1, constant_values=np.inf)
    nb = np.min(np.stack([pad[:-2, 1:-1], pad[2:, 1:-1], pad[1:-1, :-2], pad[1:-1, 2:]]), 0)
    fl = np.where(np.isfinite(fl), fl, nb)


def floor_at(x, y):
    return fl[np.clip(((x - x0) / FCELL).astype(int), 0, fl.shape[0] - 1),
              np.clip(((y - y0) / FCELL).astype(int), 0, fl.shape[1] - 1)]


def cell_of(p):
    return np.clip(((p[:, 0] - x0) / a.cell).astype(int), 0, nx - 1), np.clip(((p[:, 1] - y0) / a.cell).astype(int), 0, ny - 1)


# 바닥이 있는 칸 = 위를 보는 면의 표본이 떨어진 칸(바닥 높이 ± 0.3 m 안)
floor = np.zeros((nx, ny), bool)
for p in samples(A[up], B[up], C[up], a.cell * 0.7):
    p = p[np.abs(p[:, 2] - floor_at(p[:, 0], p[:, 1])) < 0.3]
    floor[cell_of(p)] = True
# 장애물 = 바닥 위 h_lo ~ h_hi 의 면 표본이 2개 이상인 칸(외톨이 잡음 거름)
hits = np.zeros((nx, ny), np.int32)
for p in samples(A, B, C, a.cell * 0.7):
    h = p[:, 2] - floor_at(p[:, 0], p[:, 1])
    p = p[(h >= a.h_lo) & (h <= a.h_hi)]
    np.add.at(hits, cell_of(p), 1)
occ = hits >= 2
if bnd:
    for p in samples(*tris(bnd), a.cell * 0.7):
        occ[cell_of(p)] = True

inside = np.zeros((nx, ny), bool)
if a.keepout:
    poly = np.array([(float(v[1]), float(v[2])) for v in yaml.safe_load(open(a.keepout))["island"]])
    gx, gy = np.meshgrid(x0 + (np.arange(nx) + 0.5) * a.cell, y0 + (np.arange(ny) + 0.5) * a.cell, indexing="ij")
    for (xa, ya), (xb, yb) in zip(poly, np.roll(poly, -1, 0)):   # 짝홀 규칙
        cross = ((ya > gy) != (yb > gy)) & (gx < (xb - xa) * (gy - ya) / (yb - ya + 1e-12) + xa)
        inside ^= cross
keep = int(inside.sum())
os.makedirs(a.out, exist_ok=True)


def write_map(name, occ_cells, note):
    """PGM: 0 = 장애물, 254 = 빈 칸, 205 = 미지 / 행 0 = 위쪽(y 최대)."""
    im = np.full((nx, ny), 205, np.uint8)
    im[floor] = 254
    im[occ_cells] = 0
    im = im.T[::-1]
    with open(os.path.join(a.out, name + ".pgm"), "wb") as f:
        f.write(f"P5\n{nx} {ny}\n255\n".encode())
        f.write(im.tobytes())
    with open(os.path.join(a.out, name + ".yaml"), "w") as f:
        f.write(f"# sim/nav2/make_map.py — {os.path.relpath(a.mesh)} · 장애물 바닥 위 {a.h_lo}~{a.h_hi} m · 경계벽{note}\n"
                f"image: {name}.pgm\nmode: trinary\nresolution: {a.cell}\norigin: [{x0:.3f}, {y0:.3f}, 0.0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return im


img = write_map("map", occ, " (위치 추정용 — 화단 안은 채우지 않음)")
if a.keepout:
    img = write_map("map_keepout", occ | inside, f" · 화단 채움 {a.keepout} (경로 계획 · 비용 지도용)")

# 미리보기 PNG(2배 축소 없이 0.05 m/px) — 기준선을 빨간 선으로
rgb = np.stack([img] * 3, -1).copy()
rgb[img == 205] = (200, 205, 215)
if a.route and os.path.exists(a.route):
    pts = np.array([(float(r["x"]), float(r["y"])) for r in csv.DictReader(open(a.route)) if r.get("run", "0") == "0"])
    for (xa, ya), (xb, yb) in zip(pts, np.roll(pts, -1, 0)):
        for t in np.linspace(0, 1, 12):
            i, j = int((xa + (xb - xa) * t - x0) / a.cell), int((ya + (yb - ya) * t - y0) / a.cell)
            if 0 <= i < nx and 0 <= j < ny:
                rgb[ny - 1 - j, i] = (220, 40, 40)


def png(path, arr):
    h, w, _ = arr.shape
    raw = b"".join(b"\x00" + arr[r].tobytes() for r in range(h))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


png(os.path.join(a.out, "preview.png"), rgb)
print(f"MAP {a.out} — {nx}×{ny} 칸({a.cell} m) · 원점 ({x0:.2f}, {y0:.2f}) · 빈 칸 {int((img == 254).sum())} · "
      f"장애물 {int((img == 0).sum())}(화단 채움 {keep}) · 미지 {int((img == 205).sum())}")
