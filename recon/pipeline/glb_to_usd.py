"""Polycam 등 사진 텍스처 메쉬 (GLB · glTF) → Isaac Sim 용 USD. 키프레임 · 자세 없이 메쉬 하나만 받았을 때.

    python recon/pipeline/glb_to_usd.py --config recon/configs/<장소>.yaml [--out $OUT]
    (run.py 가 input.format: glb 이면 build 단계에서 이걸 부름)

1) 불러오기: 노드 변환 적용, Y-up → Z-up (glTF 규약), 단위 m (input.scale 로 바꿀 수 있음)
2) 수평 · 바닥 높이: 위를 향한 면 중 가장 넓은 높이 = 바닥 → 그 면에 맞춰 기울기 보정, 바닥 = z 0
3) 보이는 부분: GLB 의 부분(primitive)마다 Visual<k> prim — UV · 사진 텍스처 그대로 (정점 색만 있으면 정점 색)
4) 충돌: 합친 메쉬를 정리 · 줄이고, 스캔 바닥 대신 평평한 바닥 한 장 + 경계벽 (겹친 바닥 = PhysX 멈춤)
5) 주행 경로가 없으므로 바닥 빈 곳의 뼈대(skeleton)를 따라 가상 경로 → drive_path.json (visibility · 미리보기용)
출력: <out>/usd/<name>_scene.usda · _mesh.usdc · textures/, visual.ply (점검용), transform.json, report.json, drive_path.json
"""
import argparse
import json
import os
import time

import numpy as np
import open3d as o3d
import trimesh
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree

import config
from mesh_utils import Footprint
from usd_utils import verify_scene, write_scene

Y_UP_TO_Z_UP = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)   # (x, y, z) → (x, −z, y)


def load_parts(path, up="y", scale=1.0):
    """GLB → [(이름, o3d 메쉬 (정점 · 삼각형 · 정점 색), uv (N,2) 또는 None, PIL 텍스처 또는 None)] (Z-up, m)."""
    sc = trimesh.load(path, force="scene", process=False)
    A = (Y_UP_TO_Z_UP if up == "y" else np.eye(4)) @ np.diag([scale, scale, scale, 1.0])
    parts = []
    for node in sc.graph.nodes_geometry:
        T, gname = sc.graph[node]
        g = sc.geometry[gname]
        if not isinstance(g, trimesh.Trimesh) or len(g.faces) == 0:
            continue
        v = trimesh.transform_points(np.asarray(g.vertices, float), A @ T)
        m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v), o3d.utility.Vector3iVector(np.asarray(g.faces)))
        uv, img = None, None
        vis = g.visual
        if isinstance(vis, trimesh.visual.TextureVisuals):
            mat = vis.material
            img = getattr(mat, "baseColorTexture", None) or getattr(mat, "image", None)
            if vis.uv is not None and img is not None:
                uv = np.asarray(vis.uv, float)                   # trimesh 가 이미 v 를 뒤집어 둠 (0 = 아래) → USD st 와 같음
            else:
                col = getattr(mat, "baseColorFactor", None) or getattr(mat, "main_color", None)
                if col is not None:
                    m.paint_uniform_color(np.asarray(col[:3], float) / (255.0 if np.max(col) > 1 else 1.0))
        elif vis is not None and vis.kind == "vertex":
            m.vertex_colors = o3d.utility.Vector3dVector(np.asarray(vis.vertex_colors)[:, :3] / 255.0)
        elif vis is not None and vis.kind == "face":
            fc = np.asarray(vis.face_colors)[:, :3] / 255.0       # 면 색 → 정점 색 (정점마다 붙은 면 평균)
            acc = np.zeros((len(v), 3)); cnt = np.zeros(len(v))
            for k in range(3):
                np.add.at(acc, g.faces[:, k], fc); np.add.at(cnt, g.faces[:, k], 1)
            m.vertex_colors = o3d.utility.Vector3dVector(acc / np.maximum(cnt, 1)[:, None])
        m.compute_vertex_normals()
        parts.append((str(node), m, uv, img))
    if not parts:
        raise SystemExit(f"{path}: 삼각형 메쉬가 없음")
    return parts


def merged(parts):
    out = o3d.geometry.TriangleMesh()
    for _, m, _, _ in parts:
        mm = o3d.geometry.TriangleMesh(m.vertices, m.triangles)
        out += mm
    out.compute_triangle_normals()
    return out


def find_floor(mesh, c):
    """위를 향한 면의 넓이로 높이 히스토그램 → 가장 넓은 낮은 층 = 바닥. 그 면들로 평면을 맞춰 기울기 보정 행렬.
    → (4x4 수평 · 바닥 z0 변환, 통계)"""
    v, t = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    tn = np.asarray(mesh.triangle_normals)
    cen = v[t].mean(1)
    area = 0.5 * np.linalg.norm(np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]]), axis=1)
    up = tn[:, 2] > np.cos(np.radians(c.get("max_tilt_deg", 15)))
    if area[up].sum() < 0.5:
        raise SystemExit("바닥(위를 향한 넓은 면)을 못 찾음 — input.up 축 확인 (Polycam GLB 는 y)")
    z = cen[up, 2]
    bins = np.arange(z.min() - 0.05, z.max() + 0.05, 0.02)
    h, e = np.histogram(z, bins=bins, weights=area[up])
    big = h >= 0.3 * h.max()                                   # 넓은 층 중 가장 낮은 것 = 바닥 (탁자 위 · 계단 위가 더 넓어도)
    k = int(np.argmax(big))
    k = k + int(np.argmax(h[k:k + 5]))                          # 그 근처 봉우리
    z0 = 0.5 * (e[k] + e[k + 1])
    sel = up & (np.abs(cen[:, 2] - z0) < 0.05)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cen[sel]))
    (a, b, cc, d), inl = pc.segment_plane(0.02, 3, 1000)
    n = np.array([a, b, cc]) * np.sign(cc)
    d = d * np.sign(cc)
    ax = np.cross(n, [0, 0, 1.0])
    s, co = np.linalg.norm(ax), n[2]
    R = np.eye(3)
    if s > 1e-9:                                                # n → +Z 회전 (로드리게스)
        k_ = ax / s
        K = np.array([[0, -k_[2], k_[1]], [k_[2], 0, -k_[0]], [-k_[1], k_[0], 0]])
        R = np.eye(3) + s * K + (1 - co) * K @ K
    M = np.eye(4)
    M[:3, :3] = R
    p0 = -d * n                                                 # 평면 위 점
    M[:3, 3] = -(R @ p0) * [0, 0, 1]                            # 바닥 = z 0 (수평 이동은 origin 에서)
    tilt = float(np.degrees(np.arccos(np.clip(co, -1, 1))))
    return M, dict(floor_tilt_deg=round(tilt, 2), floor_area_m2=round(float(area[sel].sum()), 1),
                   floor_plane_inlier_pct=round(100 * len(inl) / max(1, sel.sum()), 1))


def floor_color(visuals, materials, band):
    """보이는 바닥 색 (중앙값): 부분마다 위를 향한 바닥 높이 정점의 정점 색, 텍스처 부분은 그 정점 UV 자리의 텍스처 색."""
    cols = []
    for name, m in visuals.items():
        v, n = np.asarray(m.vertices), np.asarray(m.vertex_normals)
        sel = (n[:, 2] > 0.9) & (np.abs(v[:, 2]) < band)
        if sel.sum() == 0:
            continue
        if name in materials:
            im = np.asarray(Image.open(materials[name]["color"]).convert("RGB")) / 255.0
            uv = np.asarray(materials[name]["uv"])[sel] % 1.0
            cols.append(im[((1 - uv[:, 1]) * (im.shape[0] - 1)).astype(int), (uv[:, 0] * (im.shape[1] - 1)).astype(int)])
        elif m.has_vertex_colors():
            cols.append(np.asarray(m.vertex_colors)[sel])
    return np.median(np.vstack(cols), axis=0) if cols else np.array([0.5, 0.5, 0.5])


def free_space(mesh, c):
    """바닥 빈 곳 격자: 바닥 범위 안에서, 로봇 높이(0.05 ~ robot_h_m) 물체로부터 clearance_m 떨어진 칸. → (Footprint, free bool)"""
    v = np.asarray(mesh.vertices)
    t = np.asarray(mesh.triangles)
    tn = np.asarray(mesh.triangle_normals)
    cen = v[t].mean(1)
    cell = c.get("cell_m", 0.05)
    floor = (tn[:, 2] > 0.9) & (np.abs(cen[:, 2]) < c.get("floor_band_m", 0.03))
    fp = Footprint(cen[floor, :2], cell)
    obst = ~floor & (cen[:, 2] > 0.05) & (cen[:, 2] < c.get("robot_h_m", 1.0))
    occ = np.zeros(fp.grid.shape, bool)
    ij = fp.index(cen[obst, :2])
    ok = (ij >= 0).all(1) & (ij < fp.grid.shape).all(1)
    occ[ij[ok, 0], ij[ok, 1]] = True
    dist = ndimage.distance_transform_edt(~occ & fp.grid) * cell   # 물체 · 바닥 밖까지 거리
    return fp, dist >= c.get("clearance_m", 0.3)


def skeleton_path(fp, free, c):
    """빈 곳의 뼈대를 따라 이어진 경로 (xy, 순서대로). 가장 큰 덩어리만, 가까운 점부터 이어 감."""
    from skimage.morphology import skeletonize
    lab, n = ndimage.label(free)
    if n == 0:
        return np.zeros((0, 2))
    big = lab == (np.argmax(np.bincount(lab.ravel())[1:]) + 1)
    sk = np.argwhere(skeletonize(big))
    if len(sk) < 2:
        sk = np.argwhere(big)[:2]
    xy = fp.lo + (sk + 0.5) * fp.cell
    step = c.get("path_step_m", 0.1)
    keep = [0]                                                  # 촘촘한 뼈대 점 → step 간격으로 솎기
    tree = cKDTree(xy)
    taken = np.zeros(len(xy), bool)
    for i in range(len(xy)):
        if not taken[i]:
            taken[tree.query_ball_point(xy[i], step * 0.9)] = True
            keep.append(i)
    pts = xy[sorted(set(keep))]
    order, left = [0], set(range(1, len(pts)))                  # 가까운 점부터 (멀리 뛰면 그 사이는 경로가 아님 — visibility 는 방향만 씀)
    while left:
        last = pts[order[-1]]
        j = min(left, key=lambda q: np.sum((pts[q] - last) ** 2))
        order.append(j)
        left.remove(j)
    return pts[order]


def path_poses(xy, h):
    """경로 xy → 카메라 자세 (광학 좌표계: x 오른쪽, y 아래, z 앞) — 진행 방향을 봄, 높이 h."""
    poses = []
    for i in range(len(xy)):
        a, b = xy[max(i - 1, 0)], xy[min(i + 1, len(xy) - 1)]
        f = np.r_[b - a, 0.0]
        f = f / np.linalg.norm(f) if np.linalg.norm(f) > 1e-6 else np.array([1.0, 0, 0])
        x = np.cross(f, [0, 0, 1.0])
        P = np.eye(4)
        P[:3, :3] = np.stack([x, np.cross(f, x), f], 1)
        P[:3, 3] = [xy[i, 0], xy[i, 1], h]
        poses.append(P)
    return poses


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out")
    args = ap.parse_args()
    cfg = config.load(args.config)
    out = os.path.expanduser(args.out or cfg["out"])
    os.makedirs(out, exist_ok=True)
    t0 = time.time()
    inp = cfg["input"]
    path = os.path.join(os.path.expanduser(inp.get("root", ".")), inp["mesh"])
    report = dict(name=cfg["name"], input=dict(format="glb", mesh=inp["mesh"]))

    parts = load_parts(path, inp.get("up", "y"), float(inp.get("scale", 1.0)))
    whole = merged(parts)
    ext = np.ptp(np.asarray(whole.vertices), axis=0)
    n_tex = sum(p[2] is not None for p in parts)
    print(f"[1/5] 불러오기: 부분 {len(parts)}개 (사진 텍스처 {n_tex}) · 삼각형 {len(whole.triangles):,} · 범위 {np.round(ext, 2).tolist()} m")
    report["load"] = dict(parts=len(parts), textured=n_tex, triangles=len(whole.triangles), extent_m=np.round(ext, 3).tolist())
    if ext.max() > 500 or ext.max() < 0.5:
        print(f"      ⚠ 범위가 이상함 ({ext.max():.2f} m) — 단위 확인 (input.scale: cm 이면 0.01)")

    o = cfg.get("orient", {})
    M, st = find_floor(whole, o)
    whole.transform(M)
    v = np.asarray(whole.vertices)
    if o.get("origin", "min_corner") == "min_corner":
        M2 = np.eye(4)
        M2[:2, 3] = -v[:, :2].min(0)
        M = M2 @ M
        whole.transform(M2)
    elif o.get("origin") == "center":
        M2 = np.eye(4)
        M2[:2, 3] = -0.5 * (v[:, :2].min(0) + v[:, :2].max(0))
        M = M2 @ M
        whole.transform(M2)
    whole.compute_triangle_normals()
    whole.compute_vertex_normals()
    report["orient"] = st
    print(f"[2/5] 수평: 바닥 기울기 {st['floor_tilt_deg']}° 보정 · 바닥 면 {st['floor_area_m2']} m² → z 0")

    usd_dir = os.path.join(out, "usd")
    tex_dir = os.path.join(out, "glb_tex")
    os.makedirs(tex_dir, exist_ok=True)
    visuals, materials = {}, {}
    for k, (name, m, uv, img) in enumerate(parts):
        m.transform(M)
        m.compute_vertex_normals()
        pname = "Visual" if len(parts) == 1 else f"Visual{k}"
        visuals[pname] = m
        if uv is not None:
            ext_ = "png" if img.mode in ("RGBA", "LA", "P") else "jpg"
            tp = os.path.join(tex_dir, f"{pname.lower()}_color.{ext_}")
            (img.convert("RGBA") if ext_ == "png" else img.convert("RGB")).save(tp, **({} if ext_ == "png" else dict(quality=95)))
            materials[pname] = dict(color=tp, uv=uv, tile_m=1.0)
    print(f"[3/5] 보이는 부분: prim {len(visuals)}개 · 사진 텍스처 {len(materials)}")

    cc = cfg.get("collision", {})
    col = o3d.geometry.TriangleMesh(whole.vertices, whole.triangles)
    col.remove_duplicated_vertices()
    col.remove_degenerate_triangles()
    tri_c, n_c, _ = col.cluster_connected_triangles()
    n_c = np.asarray(n_c)
    col.remove_triangles_by_mask(n_c[np.asarray(tri_c)] < cc.get("min_cluster_tris", 200))
    col.compute_triangle_normals()
    vv, tt = np.asarray(col.vertices), np.asarray(col.triangles)
    cen = vv[tt].mean(1)
    scan_floor = (np.asarray(col.triangle_normals)[:, 2] > 0.9) & (np.abs(cen[:, 2]) < cc.get("floor_band_m", 0.03))
    if cc.get("flat_floor", True):                              # 스캔 바닥은 빼고 평평한 한 장
        col.remove_triangles_by_mask(scan_floor)
    col.remove_unreferenced_vertices()
    if cc.get("tris") and len(col.triangles) > cc["tris"]:
        col = col.simplify_quadric_decimation(target_number_of_triangles=cc["tris"])
    colliders = {}
    fs_cfg = cfg.get("drive_path", {})
    fp, free = free_space(whole, fs_cfg)
    floor_fp = Footprint(np.asarray(whole.vertices)[:, :2], cc.get("floor_cell_m", 0.1), margin=cc.get("floor_margin_m", 0.3))
    colliders["Collision"] = col + floor_fp.plane(0.0) if cc.get("flat_floor", True) else col
    if cc.get("floor_fill", True):                              # 보이는 바닥 받침: 스캔 바닥 아래 한 장 (스캔 가장자리 · 벽 밑 틈이 하얗게 비치지 않게)
        fill = floor_fp.plane(-cc.get("floor_fill_depth_m", 0.02))   # 스캔 바닥이 몇 mm 출렁여도 뚫고 나오지 않게 2 cm 아래
        fill.paint_uniform_color(floor_color(visuals, materials, cc.get("floor_band_m", 0.03)))
        visuals["FloorFill"] = fill
    if cc.get("boundary_height_m", 2.0) > 0:
        colliders["Boundary"] = floor_fp.walls(cc.get("boundary_height_m", 2.0))
    report["collision"] = dict(object_tris=len(col.triangles), scan_floor_tris_removed=int(scan_floor.sum()) if cc.get("flat_floor", True) else 0,
                               floor_area_m2=round(floor_fp.area(), 1), boundary_height_m=cc.get("boundary_height_m", 2.0))
    print(f"[4/5] 충돌: 물체 삼각형 {len(col.triangles):,} (스캔 바닥 {report['collision']['scan_floor_tris_removed']:,} 빼고)"
          f" + 평평한 바닥 {report['collision']['floor_area_m2']} m² · 경계벽 {report['collision']['boundary_height_m']} m")

    xy = skeleton_path(fp, free, fs_cfg)
    h = fs_cfg.get("camera_height_m", 0.3)
    poses = path_poses(xy, h)
    W, H, fov = 640, 480, 90.0
    fx = W / 2 / np.tan(np.radians(fov / 2))
    json.dump(dict(note="GLB 입력 — 바닥 빈 곳 뼈대로 만든 가상 경로 (실제 촬영 경로 아님)", camera_height_m=h,
                   intrinsic=dict(width=W, height=H, fx=fx, fy=fx, cx=W / 2, cy=H / 2), poses=[P.tolist() for P in poses]),
              open(os.path.join(out, "drive_path.json"), "w", encoding="utf-8"))
    report["drive_path"] = dict(points=len(xy), free_area_m2=round(float(free.sum()) * fp.cell ** 2, 1),
                                clearance_m=fs_cfg.get("clearance_m", 0.3))
    print(f"      가상 주행 경로: {len(xy)}점 (물체에서 {fs_cfg.get('clearance_m', 0.4)} m 떨어진 빈 바닥 {report['drive_path']['free_area_m2']} m²)")

    o3d.io.write_triangle_mesh(os.path.join(out, "visual.ply"), whole)
    T = M @ (Y_UP_TO_Z_UP if inp.get("up", "y") == "y" else np.eye(4)) @ np.diag([float(inp.get("scale", 1.0))] * 3 + [1.0])
    json.dump(dict(T_input_to_scene=T.tolist(), note="GLB 원본 좌표 (glTF Y-up) → 씬 (Z-up, 바닥 z 0, m)"),
              open(os.path.join(out, "transform.json"), "w", encoding="utf-8"), indent=1)
    info = dict(source="glb", name=cfg["name"], built=time.strftime("%Y-%m-%d"))
    scene_path, mesh_path = write_scene(usd_dir, cfg["name"], visuals, colliders, info=info,
                                        double_sided=tuple(cfg.get("usd", {}).get("double_sided", ["Visual"])) + tuple(visuals), materials=materials)
    problems, counts = verify_scene(scene_path, mesh_path)
    report["usd"] = dict(scene=scene_path, mesh=mesh_path, mesh_mb=round(os.path.getsize(mesh_path) / 1e6, 2), problems=problems, prims=counts)
    report["seconds"] = round(time.time() - t0, 1)
    json.dump(report, open(os.path.join(out, "report.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=str)
    print(f"[5/5] USD: {scene_path} (+ 메쉬 {report['usd']['mesh_mb']} MB) | {'검증 통과' if not problems else '문제: ' + '; '.join(problems)}")
    print(f"      완료 {report['seconds']}s → {os.path.join(out, 'report.json')}")
    if problems:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
