"""rehearsal.py 가 만든 OBJ 두 개를 USD 하나로: /World/VisualMesh + /World/ColliderMesh(CollisionAPI).

Z-up, 1 unit = 1 m. Isaac Sim 없이 usd-core(pxr)만으로 작성한다.
"""
import argparse, os
from pxr import Usd, UsdGeom, UsdPhysics
import numpy as np
import open3d as o3d

parser = argparse.ArgumentParser()
parser.add_argument("--out", default="~/jinwoo/out", help="rehearsal.py 출력 폴더 (USD 도 여기에 씀)")
args = parser.parse_args()

out_dir = os.path.expanduser(args.out)
usd_path = os.path.join(out_dir, "campus_scene.usd")

visual = o3d.io.read_triangle_mesh(os.path.join(out_dir, "visual_mesh.obj"))
collider = o3d.io.read_triangle_mesh(os.path.join(out_dir, "collider_hull.obj"))

stage = Usd.Stage.CreateNew(usd_path)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

# 시각용 메쉬
vis_prim = UsdGeom.Mesh.Define(stage, "/World/VisualMesh")
v = np.asarray(visual.vertices)
f = np.asarray(visual.triangles)
vis_prim.CreatePointsAttr(v.tolist())
vis_prim.CreateFaceVertexCountsAttr([3]*len(f))
vis_prim.CreateFaceVertexIndicesAttr(f.flatten().tolist())

# 충돌용 형상 (별도 prim + Collision API)
col_prim = UsdGeom.Mesh.Define(stage, "/World/ColliderMesh")
cv = np.asarray(collider.vertices)
cf = np.asarray(collider.triangles)
col_prim.CreatePointsAttr(cv.tolist())
col_prim.CreateFaceVertexCountsAttr([3]*len(cf))
col_prim.CreateFaceVertexIndicesAttr(cf.flatten().tolist())
col_prim.CreateVisibilityAttr("invisible")
UsdPhysics.CollisionAPI.Apply(col_prim.GetPrim())

stage.GetRootLayer().Save()
print("USD_WRITE_OK ->", usd_path)
print("파일 크기:", os.path.getsize(usd_path), "bytes")
