"""C 가 만든 씬 USD 를 Blender 로 열어 로봇 카메라 높이에서 렌더 — Isaac Sim 없이 재질(색 · 요철 · 거칠기)을 직접 확인.

Blender 의 USD 가져오기는 UsdPreviewSurface · 텍스처 · UV(primvar st · st_tile)를 그대로 읽는다. 렌더러(EEVEE)는 Isaac Sim(RTX)과
다르지만 같은 물리 기반 재질 규칙이라 재질 느낌은 거의 같다. 충돌 · 경계벽(invisible)은 안 가져온다.

    blender -b -P recon/tools/preview_blender.py -- --usd <out>/usd/parking_scene.usda --route eval/routes/parking_r3_cw.csv --out <폴더>

옵션: --views 6 (기준선 위 몇 곳) · --light both|overcast|sun (흐린 날 = 돔만, D 씬 기본과 비슷 · 해 = 비스듬한 햇빛)
      --height 0.35 (로봇 카메라 높이, m) · --hfov 90 · --res 960x540 · --samples 32 · --overview (위에서 비스듬히 한 장 더)
결과: <out>/view<k>_<light>.png (+ overview_<light>.png)
"""
import argparse, csv, math, os, sys

import bpy
from mathutils import Vector


def args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True)
    ap.add_argument("--route", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--views", type=int, default=6)
    ap.add_argument("--light", default="both", choices=["both", "overcast", "sun"])
    ap.add_argument("--height", type=float, default=0.35)
    ap.add_argument("--hfov", type=float, default=90.0)
    ap.add_argument("--pitch", type=float, default=5.0, help="아래로 몇 도")
    ap.add_argument("--res", default="960x540")
    ap.add_argument("--samples", type=int, default=32)
    ap.add_argument("--overview", action="store_true")
    return ap.parse_args(argv)


def setup_world(kind):
    sc = bpy.context.scene
    world = sc.world or bpy.data.worlds.new("World")
    sc.world = world
    world.use_nodes = True
    nt = world.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    nt.links.new(bg.outputs[0], out.inputs[0])
    for o in [o for o in bpy.data.objects if o.name.startswith("PreviewSun")]:
        bpy.data.objects.remove(o)
    if kind == "overcast":          # 흐린 날: 하늘에서 고르게 (D 씬 기본 = 돔 조명만, 해 0)
        bg.inputs["Color"].default_value = (0.78, 0.82, 0.88, 1)
        bg.inputs["Strength"].default_value = 1.0
    else:                           # 해: 비스듬한 햇빛 + 하늘 (요철 · 거칠기가 잘 드러남)
        sky = nt.nodes.new("ShaderNodeTexSky")
        sky.sky_type = "NISHITA"
        sky.sun_elevation = math.radians(35)
        sky.sun_rotation = math.radians(140)
        nt.links.new(sky.outputs[0], bg.inputs["Color"])
        bg.inputs["Strength"].default_value = 0.25
        sun = bpy.data.lights.new("PreviewSun", "SUN")
        sun.energy = 3.0
        sun.angle = math.radians(1.0)
        ob = bpy.data.objects.new("PreviewSun", sun)
        ob.rotation_euler = (math.radians(90 - 35), 0, math.radians(140 + 90))
        sc.collection.objects.link(ob)


def vertex_color_materials():
    """재질이 없는 메쉬(스캔 물체 · 정점 색) 는 displayColor(선형) 를 기본색으로 쓰는 재질 — Isaac Sim 이 정점 색을 그리는 것처럼.
    Blender 는 재질 없는 메쉬를 흰색으로 그림."""
    mat = bpy.data.materials.new("DisplayColor")
    mat.use_nodes = True
    nt = mat.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    attr = nt.nodes.new("ShaderNodeAttribute")
    attr.attribute_name = "displayColor"
    nt.links.new(attr.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 0.8
    for ob in bpy.data.objects:
        if ob.type == "MESH" and not ob.data.materials and "displayColor" in ob.data.attributes:
            ob.data.materials.append(mat)


def ground_z(x, y):
    dg = bpy.context.evaluated_depsgraph_get()
    hit, loc, *_ = bpy.context.scene.ray_cast(dg, Vector((x, y, 50.0)), Vector((0, 0, -1)))
    return loc.z if hit else 0.0


def look(cam, eye, target):
    d = (Vector(target) - Vector(eye)).normalized()
    cam.location = eye
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def main():
    a = args()
    os.makedirs(a.out, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=os.path.abspath(a.usd), import_materials=True, import_usd_preview=True,
                          import_visible_only=True, read_mesh_uvs=True, read_mesh_colors=True, import_lights=False,
                          import_cameras=False, create_world_material=False)
    vertex_color_materials()
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_EEVEE_NEXT"
    try:
        sc.eevee.taa_render_samples = a.samples
    except AttributeError:
        pass
    w, h = map(int, a.res.split("x"))
    sc.render.resolution_x, sc.render.resolution_y = w, h
    sc.view_settings.view_transform = "Standard"     # Isaac Sim 화면과 비슷하게 (Filmic/AgX 는 대비가 낮아 보임)
    cam_data = bpy.data.cameras.new("RobotCam")
    cam_data.angle = math.radians(a.hfov)
    cam_data.clip_start, cam_data.clip_end = 0.05, 500
    cam = bpy.data.objects.new("RobotCam", cam_data)
    sc.collection.objects.link(cam)
    sc.camera = cam

    pts = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(open(a.route, encoding="utf-8"))]
    step = max(1, len(pts) // a.views)
    shots = []
    for k in range(a.views):
        i = min(k * step, len(pts) - 2)
        j = min(i + 8, len(pts) - 1)
        (x, y), (x2, y2) = pts[i], pts[j]
        z = ground_z(x, y) + a.height
        d = Vector((x2 - x, y2 - y, 0)).normalized()
        tgt = (x + d.x * 10, y + d.y * 10, z - 10 * math.tan(math.radians(a.pitch)))
        shots.append((f"view{k}", (x, y, z), tgt))
    if a.overview:
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        shots.append(("overview", (cx + 20, cy - 45, 35), (cx, cy, 0)))

    for kind in (["overcast", "sun"] if a.light == "both" else [a.light]):
        setup_world(kind)
        for name, eye, tgt in shots:
            cam_data.angle = math.radians(a.hfov if name != "overview" else 50)
            look(cam, eye, tgt)
            sc.render.filepath = os.path.join(os.path.abspath(a.out), f"{name}_{kind}.png")
            bpy.ops.render.render(write_still=True)
            print("렌더:", sc.render.filepath)


main()
