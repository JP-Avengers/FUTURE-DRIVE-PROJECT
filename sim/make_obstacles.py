"""시나리오 파일(eval/scenarios/*.yaml)의 장애물을 USD 레이어로 — 카메라 · LiDAR 에 보이고 충돌하는 정적 상자.

장애물 자리는 기준선 CSV 의 길이 s · 옆 오프셋(왼쪽 +)으로 받아 월드 좌표로 바꾸고, 바닥 높이는 씬 충돌 메쉬의 위를 보는 면에서 잰다.
만든 레이어는 씬 파일을 고치지 않고 실행 때 얹는다:
    FD_LAYERS=<out.usda> FD_LIDAR=1 FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/run_proxy.sh headless
레포 루트(= $CD1)에서:
    envs/isaacsim_env/bin/python sim/make_obstacles.py eval/scenarios/parking_r3_obstacle.yaml sim/scenes/parking/obstacles/parking_r3_obstacle.usda \
        [--mesh data/usd/parking_v0.4b_r3/usd/parking_mesh.usdc]
끝에 장애물마다 월드 좌표 · 바닥 높이를 찍는다(평가 · 교사 스크립트가 같은 값을 쓰도록 레이어 customLayerData 에도 남김).
"""
import argparse
import csv
import json
import math
import os

import numpy as np
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ap = argparse.ArgumentParser()
ap.add_argument("scenario")
ap.add_argument("out")
ap.add_argument("--mesh", default="data/usd/parking_v0.4b_r3/usd/parking_mesh.usdc", help="바닥 높이를 잴 충돌 메쉬")
a = ap.parse_args()
sc = yaml.safe_load(open(a.scenario))

R = np.array([(float(r["s"]), float(r["x"]), float(r["y"])) for r in csv.DictReader(open(sc["route"])) if r.get("run", "0") == "0"])


def on_route(s, lateral):
    i = int(np.clip(np.searchsorted(R[:, 0], s), 1, len(R) - 1))
    (s0, x0, y0), (s1, x1, y1) = R[i - 1], R[i]
    f = (s - s0) / max(s1 - s0, 1e-9)
    x, y, th = x0 + f * (x1 - x0), y0 + f * (y1 - y0), math.atan2(y1 - y0, x1 - x0)
    return x - lateral * math.sin(th), y + lateral * math.cos(th), th   # 왼쪽 + = 진행 방향 왼쪽


st = Usd.Stage.Open(a.mesh)
coll = next(p for p in Usd.PrimRange(st.GetDefaultPrim()) if p.GetName() == "Collision")
m = UsdGeom.Mesh(coll)
P = np.array(m.GetPointsAttr().Get(), dtype=np.float64)
F = np.array(m.GetFaceVertexIndicesAttr().Get(), dtype=np.int64).reshape(-1, 3)
M = np.array(UsdGeom.Xformable(coll).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=np.float64)
P = (np.hstack([P, np.ones((len(P), 1))]) @ M)[:, :3]
A, B, C = P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]
N = np.cross(B - A, C - A)
up = N[:, 2] / np.maximum(np.linalg.norm(N, axis=1), 1e-30) > 0.9


def floor_z(x, y):
    """(x, y) 를 덮는 위를 보는 삼각형들의 높이 중 가장 낮은 것(바닥) — 없으면 None."""
    sel = up & (np.minimum.reduce([A[:, 0], B[:, 0], C[:, 0]]) <= x) & (np.maximum.reduce([A[:, 0], B[:, 0], C[:, 0]]) >= x) \
        & (np.minimum.reduce([A[:, 1], B[:, 1], C[:, 1]]) <= y) & (np.maximum.reduce([A[:, 1], B[:, 1], C[:, 1]]) >= y)
    zs = []
    for a_, b_, c_ in zip(A[sel], B[sel], C[sel]):
        d = (b_[1] - c_[1]) * (a_[0] - c_[0]) + (c_[0] - b_[0]) * (a_[1] - c_[1])
        if abs(d) < 1e-12:
            continue
        l0 = ((b_[1] - c_[1]) * (x - c_[0]) + (c_[0] - b_[0]) * (y - c_[1])) / d
        l1 = ((c_[1] - a_[1]) * (x - c_[0]) + (a_[0] - c_[0]) * (y - c_[1])) / d
        if min(l0, l1, 1 - l0 - l1) >= -1e-9:
            zs.append(l0 * a_[2] + l1 * b_[2] + (1 - l0 - l1) * c_[2])
    return min(zs) if zs else None


os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
lyr = Sdf.Layer.CreateNew(a.out) if not os.path.exists(a.out) else Sdf.Layer.FindOrOpen(a.out)
lyr.Clear()
out = Usd.Stage.Open(lyr)
UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(out, 1.0)
world = out.OverridePrim("/World")
root = UsdGeom.Xform.Define(out, "/World/Obstacles")
placed = []
for o in sc.get("obstacles", []):
    assert o.get("type", "box") == "box", "지금은 상자만"
    x, y, th = on_route(float(o["s"]), float(o.get("lateral", 0.0)))
    z0 = floor_z(x, y)
    assert z0 is not None, f"{o['name']}: 바닥을 못 찾음 ({x:.2f}, {y:.2f})"
    sx, sy, sz = (float(v) for v in o["size"])
    yaw = math.degrees(th) + float(o.get("yaw_deg", 0.0))   # 상자 방향 = 기준선 방향 + yaw_deg
    path = f"/World/Obstacles/{o['name']}"
    cube = UsdGeom.Cube.Define(out, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.XformCommonAPI(cube)
    xf.SetTranslate(Gf.Vec3d(x, y, z0 + sz / 2))
    xf.SetRotate(Gf.Vec3f(0, 0, yaw))
    xf.SetScale(Gf.Vec3f(sx, sy, sz))
    col = o.get("color", [0.55, 0.38, 0.22])
    cube.CreateDisplayColorAttr([Gf.Vec3f(*col)])
    mat = UsdShade.Material.Define(out, path + "_mat")
    sh = UsdShade.Shader.Define(out, path + "_mat/shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*col))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())        # 정적 충돌체(강체 아님 — 밀리지 않음)
    placed.append({"name": o["name"], "x": round(x, 3), "y": round(y, 3), "z_floor": round(z0, 3), "yaw_deg": round(yaw, 2),
                   "size": [sx, sy, sz], "s": float(o["s"]), "lateral": float(o.get("lateral", 0.0))})
    print(f"OBSTACLE {o['name']} s={o['s']} lateral={o.get('lateral', 0.0)} → ({x:.3f}, {y:.3f}) 바닥 {z0:.3f} m · 방향 {yaw:.1f}° · 크기 {sx}×{sy}×{sz} m")
lyr.customLayerData = {"generated_by": "sim/make_obstacles.py", "scenario": a.scenario, "obstacles_json": json.dumps(placed)}
lyr.documentation = f"시나리오 장애물 — {sc['name']} ({a.scenario}). FD_LAYERS 로 씬 위에 얹음"
lyr.Save()
print(f"SAVED {a.out} — 장애물 {len(placed)}개")
