"""Open3D 메쉬 → USD. c2d_usd.md 초안 규격: m · Z-up, 씬 .usda 레이어 + 메쉬 .usdc payload."""
import os
import shutil

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt


def srgb_to_linear(c):
    """사진값(sRGB, 0~1) → 선형. USD displayColor 는 선형 값이라 사진값을 그대로 넣으면 Isaac Sim 에서 2~3배 밝게 나옴."""
    c = np.asarray(c, dtype=np.float64)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def add_mesh(stage, path, mesh, with_color=True, double_sided=False, linear_color=True):
    """삼각형 메쉬를 UsdGeom.Mesh 로. 법선·범위를 함께 쓰고, 색은 정점 색(displayColor, 선형으로 바꿔서)으로."""
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
        col = np.asarray(mesh.vertex_colors)
        prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(
            Vt.Vec3fArray.FromNumpy((srgb_to_linear(col) if linear_color else col).astype(np.float32)))
    return prim


def add_texture_material(stage, mesh_prim, mat_path, mesh, tex, usd_dir):
    """텍스처 재질 (UsdPreviewSurface). tex = floor_texture.run · edge_finish.run 의 dict:
    region 이 있으면 color (구운 사진, sRGB — 렌더러가 선형으로 바꿈) 는 st = 영역 안 위치 (0~1) — 바닥,
    없으면 color 도 반복 재질 — 연석 · 인도 · 잔디.
    normal · roughness (CC0 재질, raw) 는 st_tile = 세계 xy / tile_m 로 반복. 텍스처는 usd/textures/ 로 복사."""
    os.makedirs(os.path.join(usd_dir, "textures"), exist_ok=True)
    v = np.asarray(mesh.vertices)
    pv = UsdGeom.PrimvarsAPI(mesh_prim)
    if tex.get("region"):
        x0, x1, y0, y1 = tex["region"]
        pv.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
            Vt.Vec2fArray.FromNumpy(np.c_[(v[:, 0] - x0) / (x1 - x0), (v[:, 1] - y0) / (y1 - y0)].astype(np.float32)))
    pv.CreatePrimvar("st_tile", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
        Vt.Vec2fArray.FromNumpy((v[:, :2] / tex["tile_m"]).astype(np.float32)))
    mat = UsdShade.Material.Define(stage, mat_path)
    surf = UsdShade.Shader.Define(stage, f"{mat_path}/Surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")

    def reader(name, var):
        r = UsdShade.Shader.Define(stage, f"{mat_path}/{name}")
        r.CreateIdAttr("UsdPrimvarReader_float2")
        r.CreateInput("varname", Sdf.ValueTypeNames.Token).Set(var)
        return r.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    def texture(name, src, st, color_space, wrap, out, out_type, scale=None, bias=None, convert=None, tag=""):
        base, ext = os.path.splitext(os.path.basename(src))
        dst = f"textures/{base}{tag}{ext}"
        if convert is None:
            shutil.copyfile(src, os.path.join(usd_dir, dst))
        else:                      # 값을 바꾼 사본 (렌더러마다 scale · bias 처리가 달라서 텍스처 자체에 반영 — Blender 미리보기 = Isaac Sim)
            from PIL import Image
            a = np.asarray(Image.open(src).convert("RGB" if out == "rgb" else "L")).astype(np.float32) / 255
            Image.fromarray((np.clip(convert(a), 0, 1) * 255 + 0.5).astype(np.uint8)).save(os.path.join(usd_dir, dst), quality=95)
        t = UsdShade.Shader.Define(stage, f"{mat_path}/{name}")
        t.CreateIdAttr("UsdUVTexture")
        t.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./{dst}")
        t.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st)
        t.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(color_space)
        t.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set(wrap)
        t.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set(wrap)
        if scale is not None:
            t.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*scale))
            t.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*bias))
        return t.CreateOutput(out, out_type)

    st_tile = reader("ReadSTTile", "st_tile")
    if tex.get("region"):
        surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            texture("ColorTex", tex["color"], reader("ReadST", "st"), "sRGB", "clamp", "rgb", Sdf.ValueTypeNames.Float3))
    else:
        surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            texture("ColorTex", tex["color"], st_tile, "sRGB", "repeat", "rgb", Sdf.ValueTypeNames.Float3))
    if tex.get("normal") or tex.get("roughness"):
        if tex.get("normal"):      # 노멀 맵 0~1 → −1~1 (OpenGL 방식 NormalGL). strength 배로 기울기(x · y)를 키운 사본
            k = tex.get("normal_strength", 1.0)

            def strengthen(a):
                n = a * 2 - 1
                n[..., :2] *= k
                n /= np.maximum(np.linalg.norm(n, axis=-1, keepdims=True), 1e-6)
                return (n + 1) / 2
            surf.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(
                texture("NormalTex", tex["normal"], st_tile, "raw", "repeat", "rgb", Sdf.ValueTypeNames.Float3, (2, 2, 2, 1), (-1, -1, -1, 0),
                        convert=strengthen if k != 1.0 else None, tag=f"_x{k:g}" if k != 1.0 else ""))
        if tex.get("roughness"):   # roughness_range [lo, hi] 로 다시 맞춘 사본 (CC0 아스팔트 거칠기가 낮아 햇빛에 젖은 노면처럼 번들거림, 11.01)
            rr = tex.get("roughness_range")
            surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(
                texture("RoughTex", tex["roughness"], st_tile, "raw", "repeat", "r", Sdf.ValueTypeNames.Float,
                        convert=(lambda a: rr[0] + (rr[1] - rr[0]) * a) if rr else None, tag=f"_r{rr[0]:g}-{rr[1]:g}" if rr else ""))
    UsdShade.MaterialBindingAPI.Apply(mesh_prim.GetPrim()).Bind(mat)
    return mat


def _new_stage():
    # 메모리에 만들고 Export → 같은 프로세스에서 다시 써도 덮어쓰기. LoadNone = payload 를 미리 열지 않음
    stage = Usd.Stage.CreateInMemory(load=Usd.Stage.LoadNone)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    return stage


def write_scene(usd_dir, name, visuals, colliders, info=None, double_sided=("Backdrop",), materials=None):
    """<name>_mesh.usdc + <name>_scene.usda (/World/<Name> 가 메쉬 파일을 payload 로 참조).

    visuals   : {"Floor": …, "Visual": 메쉬, "VisualFar": …, "Backdrop": …}  — 보이는 부분 (정점 색)
    colliders : {"Collision": 메쉬, "Boundary": …}               — 안 보이고 충돌만 (삼각형 메쉬 그대로)
    materials : {"Floor": floor_texture.run 의 텍스처 dict} — 그 부분에 텍스처 재질 (정점 색도 남겨 둠 — 텍스처를 못 읽을 때 대신)
    부분마다 prim 을 따로 둬서 D 가 배경·바깥 구역만 끄고 켤 수 있다."""
    materials = materials or {}
    os.makedirs(usd_dir, exist_ok=True)
    prim_name = name[:1].upper() + name[1:]
    mesh_path = os.path.join(usd_dir, f"{name}_mesh.usdc")
    scene_path = os.path.join(usd_dir, f"{name}_scene.usda")

    st = _new_stage()
    st.SetDefaultPrim(UsdGeom.Xform.Define(st, f"/{prim_name}").GetPrim())
    for part, mesh in visuals.items():
        prim = add_mesh(st, f"/{prim_name}/{part}", mesh, double_sided=(part in double_sided))   # 양면 = 뒤에서 봐도 그림 (비쳐 보이지 않게)
        if part in materials:
            add_texture_material(st, prim, f"/{prim_name}/Looks/{part}Mat", mesh, materials[part], usd_dir)
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
    for p in stage.Traverse():      # 텍스처 파일이 실제로 있나 (상대 경로는 메쉬 .usdc 기준)
        if p.IsA(UsdShade.Shader) and UsdShade.Shader(p).GetIdAttr().Get() == "UsdUVTexture":
            f = UsdShade.Shader(p).GetInput("file").Get()
            if f is None or not f.resolvedPath:
                problems.append(f"{p.GetPath()}: 텍스처 파일 없음 ({f})")
    summary = {str(p.GetPath()): len(UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()) for p in meshes}
    return problems, summary
