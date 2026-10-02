"""rehearsal.py 가 만든 OBJ 두 개를 USD 하나로: /World/VisualMesh + /World/ColliderMesh(CollisionAPI).

Z-up, 1 unit = 1 m. Isaac Sim 없이 usd-core(pxr)만으로 작성한다.
"""
import argparse, os
from pxr import Usd, UsdGeom, UsdPhysics, Vt, Gf
import numpy as np
import open3d as o3d

parser = argparse.ArgumentParser()
parser.add_argument("--out", default="~/jinwoo/out", help="rehearsal.py 출력 폴더 (USD 도 여기에 씀)")
args = parser.parse_args()

out_dir = os.path.expanduser(args.out)
usd_path = os.path.join(out_dir, "campus_scene.usd")

visual = o3d.io.read_triangle_mesh(os.path.join(out_dir, "visual_mesh.obj"))
collider = o3d.io.read_triangle_mesh(os.path.join(out_dir, "collider_hull.obj"))


def add_mesh(stage, path, mesh, with_color=True):
    """삼각형 메쉬를 UsdGeom.Mesh 로. 법선·범위를 함께 쓰고, 색은 정점 색(displayColor)으로."""
    v = np.asarray(mesh.vertices, dtype=np.float32)
    f = np.asarray(mesh.triangles, dtype=np.int32)
    prim = UsdGeom.Mesh.Define(stage, path)
    prim.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v))
    prim.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(f), 3, dtype=np.int32)))
    prim.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(f.ravel()))
    prim.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, v.min(0))), Gf.Vec3f(*map(float, v.max(0)))]))
    # USD 기본값은 catmullClark — 렌더러가 삼각형을 곡면으로 다시 쪼갠다. 스캔 메쉬는 그대로 그려야 함
    prim.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.compute_vertex_normals()
    prim.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(mesh.vertex_normals, dtype=np.float32)))
    prim.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    if with_color and mesh.has_vertex_colors():
        prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(
            Vt.Vec3fArray.FromNumpy(np.asarray(mesh.vertex_colors, dtype=np.float32)))
    return prim


stage = Usd.Stage.CreateNew(usd_path)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

# 시각용 메쉬
add_mesh(stage, "/World/VisualMesh", visual)

# 충돌용 형상 (별도 prim + Collision API, 색 없음)
col_prim = add_mesh(stage, "/World/ColliderMesh", collider, with_color=False)
col_prim.CreateVisibilityAttr("invisible")
UsdPhysics.CollisionAPI.Apply(col_prim.GetPrim())

stage.GetRootLayer().Save()
print("USD_WRITE_OK ->", usd_path)
print("파일 크기:", os.path.getsize(usd_path), "bytes")
