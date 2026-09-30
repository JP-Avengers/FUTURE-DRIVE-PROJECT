"""Proxy 씬 생성 — sim/scenes/proxy/params.yaml 을 읽어 같은 폴더의 .usda 레이어를 만든다.

Isaac Sim 을 띄우지 않고 pxr 만 쓴다(몇 초). 레포 루트(= $CD1)에서:
    envs/isaacsim_env/bin/python sim/build_proxy_scene.py

만드는 레이어(실행할 때마다 새로 씀 — 손으로 고치지 말 것):
    proxy_parking.usda  루트. 아래를 subLayers 로 묶는다. 이 파일을 연다
    world.usda          물리 장면(PhysX 설정은 NVIDIA 샘플과 같게) · 조명 · 재질(/World/Looks)
    env_proxy.usda      노면(화면용 상자 + 충돌용 무한 평면) · 중앙 섬 · 주차칸 선 · 주차 차량   ← 캠퍼스 씬에서는 이 레이어만 바뀜
    background.usda     방위별 건물 박스 · 경사면 (캠퍼스 씬과 공용)
    robot.usda          Nova_Carter_ROS.usd 참조 · 시작 자세 · 운전자용 3인칭 카메라
GUI 로 고치는 레이어(sensors.usda · ros.usda)는 없을 때 빈 틀만 만들고 덮어쓰지 않는다.

좌표: 원점 = 루프 남서 꼭짓점 노면, +X 동 · +Y 북 · +Z 위(ENU), m. 같은 params 로 다시 만들면
같은 파일이 나온다(난수 시드 고정, 시각 기록 없음).
"""
import math
import os
import random

import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "scenes", "proxy")
GENERATED = "sim/build_proxy_scene.py 가 만든 레이어 — 손으로 고치지 말고 params.yaml 을 고쳐 다시 만들 것"
# USD 는 앞쪽 레이어가 강하다. sensors 가 robot(에셋) 값을 덮어써야 하므로 이 순서를 지킨다.
SUBLAYERS = ["sensors.usda", "ros.usda", "robot.usda", "env_proxy.usda", "background.usda", "world.usda"]
# 시간 단위 — NVIDIA 샘플 씬(carter_warehouse_navigation.usd)과 같은 60. 쓰지 않으면 USD 기본값 24 로 열려
# 타임라인(24 Hz)과 물리(60 Hz)가 어긋나고, 차동 제어기가 deltaTime 0 스텝을 건너뛴다(9.29: 경고 1,559회, odom 84 Hz).
TIME_CODES_PER_SECOND = 60
# PhysX 장면 설정 — NVIDIA 샘플 씬과 같게. 비워 두면 기본값(GPU 물리)으로 돌아 직진 출발 때 로봇이 회차마다
# 다르게 흔들렸다(9.30: 빈 바닥에서도 56~79 %, 이 설정으로 0 %). pxr 만으로는 PhysxSchema 를 못 불러서 속성을 직접 쓴다.
PHYSX_SCENE = {  # 이름: (값, 타입, uniform 여부)
    "physxScene:solverType": ("TGS", Sdf.ValueTypeNames.Token, True),
    "physxScene:enableGPUDynamics": (False, Sdf.ValueTypeNames.Bool, False),
    "physxScene:enableStabilization": (True, Sdf.ValueTypeNames.Bool, False),
    "physxScene:enableCCD": (True, Sdf.ValueTypeNames.Bool, False),
    "physxScene:broadphaseType": ("MBP", Sdf.ValueTypeNames.Token, True),
}

LOOKS = {  # 이름: (색 RGB, 거칠기, 금속성). 텍스처는 도메인 랜덤화 단계에서
    "asphalt": ((0.18, 0.18, 0.19), 0.9, 0.0),
    "line_white": ((0.92, 0.92, 0.90), 0.7, 0.0),
    "island": ((0.50, 0.50, 0.48), 0.9, 0.0),
    "wall_white": ((0.85, 0.85, 0.82), 0.8, 0.0),
    "glass": ((0.32, 0.46, 0.56), 0.15, 0.4),
    "brick": ((0.55, 0.27, 0.20), 0.9, 0.0),
    "grass": ((0.24, 0.38, 0.17), 1.0, 0.0),
}


# ── 평면 기하 ──────────────────────────────────────────────
def sub(a, b): return (a[0] - b[0], a[1] - b[1])
def add(a, b): return (a[0] + b[0], a[1] + b[1])
def mul(a, k): return (a[0] * k, a[1] * k)
def cross(a, b): return a[0] * b[1] - a[1] * b[0]


def unit(v):
    n = math.hypot(*v)
    return (v[0] / n, v[1] / n)


def right(d):  # 진행 방향의 오른쪽 — 시계 방향 루프에서는 안쪽
    return (d[1], -d[0])


def offset_polygon(pts, dist):
    """시계 방향 볼록 다각형을 안쪽으로 dist 만큼 줄인다(변을 평행 이동해 이웃 변과 교차)."""
    n, lines = len(pts), []
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        shift = mul(right(unit(sub(b, a))), dist)
        lines.append((add(a, shift), sub(b, a)))
    out = []
    for i in range(n):
        (p, r), (q, s) = lines[i - 1], lines[i]
        t = cross(sub(q, p), s) / cross(r, s)
        out.append(add(p, mul(r, t)))
    return out


def inside(poly, p, margin=0.0):
    """시계 방향 볼록 다각형 안에 있는지(변에서 margin 이상 떨어져야 안)."""
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        d = unit(sub(b, a))
        if cross(d, sub(p, a)) > -margin:  # 오른쪽(안쪽)이면 음수
            return False
    return True


# ── USD 도우미 ────────────────────────────────────────────
def new_stage(name, doc):
    path = os.path.join(OUT, name)
    if os.path.exists(path):
        os.remove(path)
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    layer = stage.GetRootLayer()
    layer.documentation = doc
    return stage


def stamp(layer, P):
    layer.customLayerData = {"proxy_version": P["version"], "generated_by": "sim/build_proxy_scene.py",
                             "params": "sim/scenes/proxy/params.yaml"}


def bind(prim, look):
    UsdShade.MaterialBindingAPI.Apply(prim)
    prim.CreateRelationship("material:binding", False).SetTargets([Sdf.Path("/World/Looks/" + look)])


def box(stage, path, center, size, look, yaw=0.0, rot_x=0.0, rot_y=0.0, collide=True):
    """1 m 큐브를 크기·자세로 늘인 상자. center 는 상자 중심(x, y, z)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(cube)
    x.AddTranslateOp().Set(Gf.Vec3d(*center))
    if rot_x or rot_y:
        x.AddRotateXYZOp().Set(Gf.Vec3f(rot_x, rot_y, yaw))
    elif yaw:
        x.AddRotateZOp().Set(yaw)
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    bind(cube.GetPrim(), look)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def look_at_matrix(eye, target):
    """-Z 가 eye→target 을 보고 +Z 가 위쪽인 prim 변환(카메라·원거리 조명 공용)."""
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
    return view.GetInverse()


# ── 레이어 ────────────────────────────────────────────────
def build_world(P):
    st = new_stage("world.usda", GENERATED)
    UsdGeom.Xform.Define(st, "/World")
    scene = UsdPhysics.Scene.Define(st, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    prim = scene.GetPrim()
    prim.AddAppliedSchema("PhysxSceneAPI")
    for name, (value, vtype, uniform) in PHYSX_SCENE.items():
        prim.CreateAttribute(name, vtype, custom=False,
                             variability=Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying).Set(value)

    L = P["lights"]
    UsdGeom.Xform.Define(st, "/World/Lights")
    UsdLux.DomeLight.Define(st, "/World/Lights/Dome").CreateIntensityAttr(L["dome_intensity"])
    sun = UsdLux.DistantLight.Define(st, "/World/Lights/Sun")
    sun.CreateIntensityAttr(L["sun_intensity"])
    sun.CreateAngleAttr(1.0)
    az, el = math.radians(L["sun_azimuth_deg"]), math.radians(L["sun_elevation_deg"])
    sun_dir = (math.cos(el) * math.sin(az), math.cos(el) * math.cos(az), math.sin(el))
    UsdGeom.Xformable(sun).AddTransformOp().Set(look_at_matrix(sun_dir, (0, 0, 0)))

    UsdGeom.Scope.Define(st, "/World/Looks")
    colors = {**LOOKS, **{f"car_{i}": (tuple(c), 0.35, 0.3) for i, c in enumerate(P["cars"]["colors"])}}
    for name, (rgb, rough, metal) in colors.items():
        mat = UsdShade.Material.Define(st, f"/World/Looks/{name}")
        sh = UsdShade.Shader.Define(st, f"/World/Looks/{name}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    stamp(st.GetRootLayer(), P)
    st.GetRootLayer().Save()


def stall_row(st, parent, rng, P, origin, along, normal, count, base_z, stats):
    """along 방향으로 count 칸, normal 방향으로 깊이 depth 인 직각 주차 줄. origin = 첫 칸 앞변 중앙."""
    S, C = P["stalls"], P["cars"]
    w, dep = S["width"], S["depth"]
    yaw = math.degrees(math.atan2(normal[1], normal[0]))
    lz = base_z + S["line_height"] / 2
    for k in range(count + 1):  # 칸 경계선
        c = add(add(origin, mul(along, (k - 0.5) * w)), mul(normal, dep / 2))
        box(st, f"{parent}/Line_{k:02d}", (c[0], c[1], lz), (dep, S["line_width"], S["line_height"]),
            "line_white", yaw=yaw, collide=False)
        stats["lines"] += 1
    for k in range(count):
        stats["slots"] += 1
        if rng.random() < C["fill"]:
            c = add(add(origin, mul(along, k * w)), mul(normal, dep / 2))
            cl, cw, ch = C["size"]
            box(st, f"{parent}/Car_{k:02d}", (c[0], c[1], base_z + ch / 2), (cl, cw, ch),
                f"car_{rng.randrange(len(C['colors']))}", yaw=yaw)
            stats["cars"] += 1


def build_env(P):
    st = new_stage("env_proxy.usda", GENERATED)
    UsdGeom.Xform.Define(st, "/World")
    UsdGeom.Xform.Define(st, "/World/Env")
    rng = random.Random(P["cars"]["seed"])
    stats = {"slots": 0, "cars": 0, "lines": 0}
    g = P["ground"]
    # 노면 상자는 화면용이고 충돌은 무한 평면이 맡는다. 큰 상자를 충돌체로 쓰면 로봇이 상자 중심에서 멀 때
    # 직진 출발이 흔들렸다(9.30: 중심에서 15 m 넘게 떨어지면 8~61 %, 평면으로 바꾸면 0 %).
    box(st, "/World/Env/Ground", ((g["x_min"] + g["x_max"]) / 2, (g["y_min"] + g["y_max"]) / 2, -0.1),
        (g["x_max"] - g["x_min"], g["y_max"] - g["y_min"], 0.2), "asphalt", collide=False)
    plane = UsdGeom.Plane.Define(st, "/World/Env/GroundPlane")  # z = 0 노면 — 물리에서는 무한 평면
    plane.CreateAxisAttr(UsdGeom.Tokens.z)
    plane.CreateWidthAttr(g["x_max"] - g["x_min"])
    plane.CreateLengthAttr(g["y_max"] - g["y_min"])
    plane.CreatePurposeAttr(UsdGeom.Tokens.guide)  # 렌더에는 안 보임
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())

    loop = [(v["x"], v["y"]) for v in P["loop"]]
    names = [v["name"] for v in P["loop"]]
    half = P["road"]["width"] / 2
    S = P["stalls"]

    # 중앙 섬 — 루프를 도로 반폭만큼 안쪽으로 줄인 다각형을 연석 높이로 올림
    isl = offset_polygon(loop, half)
    h = P["island"]["curb_height"]
    mesh = UsdGeom.Mesh.Define(st, "/World/Env/Island")
    n = len(isl)
    pts = [Gf.Vec3f(x, y, 0.0) for x, y in isl] + [Gf.Vec3f(x, y, h) for x, y in isl]
    counts, idx = [n], list(range(2 * n - 1, n - 1, -1))  # 윗면(바깥에서 보아 반시계)
    for i in range(n):  # 옆면 — 바깥을 향하게(시계 방향 변 i→j 의 왼쪽)
        j = (i + 1) % n
        counts.append(4)
        idx += [i, n + i, n + j, j]
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    bind(mesh.GetPrim(), "island")
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")

    # 섬 안 주차 — 북쪽 변(NW'→NE')과 나란히 등을 맞댄 줄
    ia, ib = isl[names.index("NW")], isl[names.index("NE")]
    along = unit(sub(ib, ia))
    inward = right(along)
    L = math.hypot(*sub(ib, ia))
    for r in range(P["island"]["rows"]):
        # 줄 r 의 칸 중 네 모서리가 섬 안(0.3 m 여유)인 연속 구간만 쓴다
        front = add(ia, mul(inward, r * S["depth"]))
        ok = []
        for k in range(int(L // S["width"])):
            c0 = add(front, mul(along, (k + 0.5) * S["width"]))
            corners = [add(add(c0, mul(along, sx * S["width"] / 2)), mul(inward, dz * S["depth"]))
                       for sx in (-1, 1) for dz in (0, 1)]
            if all(inside(isl, q, 0.3) for q in corners):
                ok.append(k)
        if ok:
            origin = add(front, mul(along, (ok[0] + 0.5) * S["width"]))
            UsdGeom.Xform.Define(st, f"/World/Env/IslandRow{r}")
            stall_row(st, f"/World/Env/IslandRow{r}", rng, P, origin, along, inward, len(ok), h, stats)

    # 바깥 주차 줄 — 지정한 변의 바깥쪽, 도로 가장자리부터 깊이 depth
    for e in S["outer_edges"]:
        a_name, b_name = e.split("-")
        a, b = loop[names.index(a_name)], loop[names.index(b_name)]
        d = unit(sub(b, a))
        out = mul(right(d), -1)
        L = math.hypot(*sub(b, a))
        m = S["corner_margin"]
        count = int((L - 2 * m) // S["width"])
        if count <= 0:
            continue
        start = m + ((L - 2 * m) - count * S["width"]) / 2 + S["width"] / 2
        origin = add(add(a, mul(d, start)), mul(out, half))
        path = f"/World/Env/Row_{a_name}_{b_name}"
        UsdGeom.Xform.Define(st, path)
        stall_row(st, path, rng, P, origin, d, out, count, 0.0, stats)

    stamp(st.GetRootLayer(), P)
    st.GetRootLayer().Save()
    return stats


def build_background(P):
    st = new_stage("background.usda", GENERATED + " · 캠퍼스 씬과 공용(C 가 사진 텍스처를 입힘)")
    UsdGeom.Xform.Define(st, "/World")
    UsdGeom.Xform.Define(st, "/World/Background")
    for b in P["buildings"]:
        (x0, x1), (y0, y1), hh = b["x"], b["y"], b["h"]
        box(st, f"/World/Background/{b['name']}", ((x0 + x1) / 2, (y0 + y1) / 2, hh / 2),
            (x1 - x0, y1 - y0, hh), b["look"])
    t = 0.3  # 경사판 두께
    w = P["slopes"]["west"]  # 동쪽 끝 윗면이 노면(z=0)에 닿고 서쪽으로 오른다
    a, L = math.radians(w["deg"]), abs(w["x_to"] - w["x_from"])
    cx = w["x_from"] - L / 2 * math.cos(a) - t / 2 * math.sin(a)
    cz = L / 2 * math.sin(a) - t / 2 * math.cos(a)
    box(st, "/World/Background/Slope_West", (cx, sum(w["y"]) / 2, cz), (L, w["y"][1] - w["y"][0], t),
        w["look"], rot_y=w["deg"])
    s = P["slopes"]["south"]  # 북쪽 끝 윗면이 노면에 닿고 남쪽으로 오른다
    a, L = math.radians(s["deg"]), abs(s["y_to"] - s["y_from"])
    cy = s["y_from"] - L / 2 * math.cos(a) - t / 2 * math.sin(a)
    cz = L / 2 * math.sin(a) - t / 2 * math.cos(a)
    box(st, "/World/Background/Slope_SouthExit", (sum(s["x"]) / 2, cy, cz), (s["x"][1] - s["x"][0], L, t),
        s["look"], rot_x=-s["deg"])
    stamp(st.GetRootLayer(), P)
    st.GetRootLayer().Save()


def build_robot(P):
    """로봇 참조는 Sdf 로 직접 쓴다 — Usd 로 쓰면 저장할 때 S3 에셋을 열려다 경고가 난다."""
    R = P["robot"]
    path = os.path.join(OUT, "robot.usda")
    if os.path.exists(path):
        os.remove(path)
    layer = Sdf.Layer.CreateNew(path)
    layer.documentation = GENERATED
    layer.pseudoRoot.SetInfo("upAxis", "Z")
    layer.pseudoRoot.SetInfo("metersPerUnit", 1.0)
    world = Sdf.PrimSpec(layer.pseudoRoot, "World", Sdf.SpecifierDef, "Xform")
    robot = Sdf.PrimSpec(world, R["name"], Sdf.SpecifierDef, "Xform")
    robot.referenceList.Prepend(Sdf.Reference(R["asset"]))
    s = R["start"]
    Sdf.AttributeSpec(robot, "xformOp:translate", Sdf.ValueTypeNames.Double3).default = Gf.Vec3d(s["x"], s["y"], 0.0)
    Sdf.AttributeSpec(robot, "xformOp:rotateZ", Sdf.ValueTypeNames.Float).default = float(s["yaw_deg"])
    Sdf.AttributeSpec(robot, "xformOpOrder", Sdf.ValueTypeNames.TokenArray).default = \
        ["xformOp:translate", "xformOp:rotateZ"]
    c = R["chase_cam"]  # 로봇 기준 좌표(+X 앞)
    cam = Sdf.PrimSpec(robot, "ChaseCam", Sdf.SpecifierDef, "Camera")
    Sdf.AttributeSpec(cam, "xformOp:transform", Sdf.ValueTypeNames.Matrix4d).default = \
        look_at_matrix((-c["back"], 0.0, c["up"]), (c["ahead"], 0.0, c["up"] - c["down"]))
    Sdf.AttributeSpec(cam, "xformOpOrder", Sdf.ValueTypeNames.TokenArray).default = ["xformOp:transform"]
    Sdf.AttributeSpec(cam, "focalLength", Sdf.ValueTypeNames.Float).default = 18.0
    Sdf.AttributeSpec(cam, "clippingRange", Sdf.ValueTypeNames.Float2).default = Gf.Vec2f(0.1, 500.0)
    stamp(layer, P)
    layer.Save()


def ensure_gui_layer(name, doc):
    path = os.path.join(OUT, name)
    if os.path.exists(path):
        return False
    layer = Sdf.Layer.CreateNew(path)
    layer.documentation = doc
    layer.Save()
    return True


def build_root(P):
    path = os.path.join(OUT, "proxy_parking.usda")
    if os.path.exists(path):
        os.remove(path)
    layer = Sdf.Layer.CreateNew(path)
    layer.documentation = "Proxy 주차장 루트 — 이 파일을 연다. " + GENERATED
    layer.pseudoRoot.SetInfo("upAxis", "Z")
    layer.pseudoRoot.SetInfo("metersPerUnit", 1.0)
    layer.defaultPrim = "World"
    layer.timeCodesPerSecond = TIME_CODES_PER_SECOND
    layer.subLayerPaths = ["./" + s for s in SUBLAYERS]
    stamp(layer, P)
    layer.Save()


def main():
    with open(os.path.join(OUT, "params.yaml"), encoding="utf-8") as f:
        P = yaml.safe_load(f)
    build_world(P)
    stats = build_env(P)
    build_background(P)
    build_robot(P)
    new_gui = [n for n, d in (("sensors.usda", "센서 덮어쓰기(카메라 640×480 · 발행 주기 · 끌 센서) — GUI 로 고치는 레이어"),
                              ("ros.usda", "ROS 2 그래프(/clock 등) — GUI 로 고치는 레이어"))
               if ensure_gui_layer(n, d)]
    build_root(P)
    loop = [(v["x"], v["y"]) for v in P["loop"]]
    length = sum(math.dist(loop[i], loop[(i + 1) % len(loop)]) for i in range(len(loop)))
    print(f"proxy {P['version']}: 루프 {length:.1f} m · 주차칸 {stats['slots']} · 차량 {stats['cars']} · "
          f"선 {stats['lines']} · 새 GUI 레이어 {new_gui or '없음'}")
    print("OK", os.path.relpath(os.path.join(OUT, "proxy_parking.usda"), os.path.dirname(HERE)))


if __name__ == "__main__":
    main()
