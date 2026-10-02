"""Open3D 메쉬 → USD. c2d_usd.md 초안 규격: m · Z-up, 씬 .usda 레이어 + 메쉬 .usdc payload."""
import os

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt


def add_mesh(stage, path, mesh, with_color=True, double_sided=False):
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
    if double_sided:
        prim.CreateDoubleSidedAttr(True)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    prim.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(mesh.vertex_normals, dtype=np.float32)))
    prim.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    if with_color and mesh.has_vertex_colors():
        prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(
            Vt.Vec3fArray.FromNumpy(np.asarray(mesh.vertex_colors, dtype=np.float32)))
    return prim


def _new_stage():
    # 메모리에 만들고 Export → 같은 프로세스에서 다시 써도 덮어쓰기. LoadNone = payload 를 미리 열지 않음
    stage = Usd.Stage.CreateInMemory(load=Usd.Stage.LoadNone)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    return stage


def write_scene(usd_dir, name, visuals, colliders, info=None):
    """<name>_mesh.usdc + <name>_scene.usda (/World/<Name> 가 메쉬 파일을 payload 로 참조).

    visuals   : {"Visual": 메쉬, "VisualFar": …, "Backdrop": …}  — 보이는 부분 (정점 색)
    colliders : {"Collision": 메쉬, "Boundary": …}               — 안 보이고 충돌만 (삼각형 메쉬 그대로)
    부분마다 prim 을 따로 둬서 D 가 배경·바깥 구역만 끄고 켤 수 있다."""
    os.makedirs(usd_dir, exist_ok=True)
    prim_name = name[:1].upper() + name[1:]
    mesh_path = os.path.join(usd_dir, f"{name}_mesh.usdc")
    scene_path = os.path.join(usd_dir, f"{name}_scene.usda")

    st = _new_stage()
    st.SetDefaultPrim(UsdGeom.Xform.Define(st, f"/{prim_name}").GetPrim())
    for part, mesh in visuals.items():
        add_mesh(st, f"/{prim_name}/{part}", mesh, double_sided=(part == "Backdrop"))
    for part, mesh in colliders.items():
        col = add_mesh(st, f"/{prim_name}/{part}", mesh, with_color=False)
        col.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(col.GetPrim())
        # 정적 배경 → PhysX 삼각형 메쉬 충돌. 볼록 분해는 움직이는 물체에만 필요
        UsdPhysics.MeshCollisionAPI.Apply(col.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.none)
    st.GetRootLayer().Export(mesh_path)

    sc = _new_stage()
    sc.SetDefaultPrim(UsdGeom.Xform.Define(sc, "/World").GetPrim())
    UsdGeom.Xform.Define(sc, f"/World/{prim_name}").GetPrim().GetPayloads().AddPayload(f"./{os.path.basename(mesh_path)}")
    if info:
        sc.GetRootLayer().customLayerData = info
    sc.GetRootLayer().Export(scene_path)
    return scene_path, mesh_path


def verify_scene(scene_path, mesh_path):
    """D 에게 넘기기 전 점검. (문제 목록, 메쉬별 삼각형 수) — 문제 목록이 비어 있으면 통과."""
    for p in (scene_path, mesh_path):   # 같은 프로세스에서 다시 쓴 파일이면 캐시 대신 새로 읽기
        layer = Sdf.Layer.Find(p)
        if layer:
            layer.Reload(force=True)
    stage = Usd.Stage.Open(scene_path)
    problems = []
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        problems.append("upAxis 가 Z 가 아님")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
        problems.append("metersPerUnit 이 1 이 아님")
    if not stage.GetDefaultPrim():
        problems.append("defaultPrim 없음")
    meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    visual = [p for p in meshes if not p.HasAPI(UsdPhysics.CollisionAPI)]
    collision = [p for p in meshes if p.HasAPI(UsdPhysics.CollisionAPI)]
    if not visual:
        problems.append("시각용 메쉬 없음")
    if not collision:
        problems.append("CollisionAPI 메쉬 없음")
    for p in meshes:
        g = UsdGeom.Mesh(p)
        if g.GetSubdivisionSchemeAttr().Get() != UsdGeom.Tokens.none:
            problems.append(f"{p.GetPath()}: subdivisionScheme 이 none 이 아님")
        if not g.GetExtentAttr().HasAuthoredValue():
            problems.append(f"{p.GetPath()}: extent 없음")
    for p in collision:
        if UsdGeom.Mesh(p).GetVisibilityAttr().Get() != UsdGeom.Tokens.invisible:
            problems.append(f"{p.GetPath()}: 충돌용이 보이게 되어 있음")
    summary = {str(p.GetPath()): len(UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()) for p in meshes}
    return problems, summary
