"""메쉬 → 정리 → Z-up·원점 → (바닥 높이) → 구멍 메우기 → (색 투영) → 경량화 → 충돌 메쉬 → 구역 → USD → 검증.

    python recon/pipeline/build_usd.py --config recon/configs/lounge.yaml

입력: config 에 geometry 가 없으면 <out>/mesh_raw.ply (+ mesh_far_raw.ply), tsdf_meta.json  (tsdf.py 출력)
      geometry 가 있으면 그 형상 메쉬 (B 맵 v0.2~: 원본 depth 메쉬, 색 없음) — 색은 colorize 로 키프레임 투영
출력 (<out>/): visual.ply (보이는 부분 전부), collider.ply, transform.json, report.json,
               floor_patch.ply · visual_far.ply · backdrop.ply · boundary.ply (켠 경우),
               ground.npz · color_qc.csv (실외 설정일 때), usd/<name>_scene.usda + usd/<name>_mesh.usdc
"""
import argparse, csv, json, os, time

import numpy as np
import open3d as o3d
import yaml

import io_rgbd
import usd_utils
from color_utils import fill_unseen, project_colors, raycasting_scene
from mesh_utils import Footprint, Ground, fill_small_holes, floor_colors, floor_patch


def cleanup(mesh, min_tris):
    """떠다니는 작은 조각(min_tris 미만)과 깨진 삼각형 제거."""
    ids, counts, _ = mesh.cluster_connected_triangles()
    ids, counts = np.asarray(ids), np.asarray(counts)
    small = counts[ids] < min_tris
    out = o3d.geometry.TriangleMesh(mesh)
    out.remove_triangles_by_mask(small)
    out.remove_degenerate_triangles()
    out.remove_duplicated_triangles()
    out.remove_duplicated_vertices()
    out.remove_non_manifold_edges()
    out.remove_unreferenced_vertices()
    out.compute_vertex_normals()
    return out, dict(clusters=len(counts), small_clusters=int((counts < min_tris).sum()),
                     removed_tris=len(mesh.triangles) - len(out.triangles))


def find_floor(mesh, up_hint, max_tilt_deg, n=200000):
    """up_hint 에서 max_tilt_deg 안을 향한 면의 점으로 평면을 찾고, 큰 수평면 중 가장 낮은 것을 바닥으로.
    (바닥 법선, 바닥 점들) — 테이블 윗면도 수평면이라 높이로 구별한다."""
    pcd = mesh.sample_points_uniformly(n)
    horiz = np.abs(np.asarray(pcd.normals) @ up_hint) > np.cos(np.radians(max_tilt_deg))
    rest = pcd.select_by_index(np.where(horiz)[0])
    n_horiz, planes = len(rest.points), []
    for _ in range(3):
        if len(rest.points) < 1000:
            break
        model, inl = rest.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
        pts = np.asarray(rest.points)[inl]
        planes.append((float(np.median(pts @ up_hint)), len(inl), np.array(model[:3]), pts))
        rest = rest.select_by_index(inl, invert=True)
    big = [p for p in planes if p[1] > 0.1 * n_horiz]
    if not big:
        raise SystemExit("바닥 평면을 찾지 못함 — orient.up_hint / max_tilt_deg 확인")
    _, _, normal, pts = min(big, key=lambda p: p[0])
    normal /= np.linalg.norm(normal)
    return (normal if normal @ up_hint > 0 else -normal), pts


def orient(mesh, up_hint, c, crop=True):
    """바닥 법선 → +Z (level: false 면 회전 없이 입력 Z 유지), 바닥 → z=0,
    (origin=min_corner 면) x·y 최솟값 모서리 → 원점. crop = 바닥보다 crop_below_floor 아래 잘라냄 (평평한 바닥 기준)."""
    n, floor_pts = find_floor(mesh, up_hint, c["max_tilt_deg"])
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(n, z)
    s = np.linalg.norm(axis)
    level = c.get("level", True)
    R = np.eye(3) if s < 1e-8 or not level else o3d.geometry.get_rotation_matrix_from_axis_angle(axis / s * np.arctan2(s, n @ z))
    v = np.asarray(mesh.vertices) @ R.T
    shift = np.array([0.0, 0.0, np.median((floor_pts @ R.T)[:, 2])])
    if c["origin"] == "min_corner":
        shift[:2] = v[:, :2].min(0)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, -shift
    out = o3d.geometry.TriangleMesh(mesh).transform(T)
    if crop:
        bb = out.get_axis_aligned_bounding_box()
        out = out.crop(o3d.geometry.AxisAlignedBoundingBox(np.r_[bb.min_bound[:2], -c["crop_below_floor"]], bb.max_bound))
    out.compute_vertex_normals()
    zs = np.asarray(out.vertices)[:, 2]
    return out, T, dict(floor_tilt_from_hint_deg=round(float(np.degrees(np.arccos(np.clip(n @ up_hint, -1, 1)))), 2),
                        leveled=bool(level), floor_vertex_pct=round(100 * float((np.abs(zs) < 0.02).mean()), 1),
                        extent_m=np.round(out.get_axis_aligned_bounding_box().get_extent(), 3).tolist())


def distance_to(mesh, points):
    """점들에서 mesh 표면까지 정확한 거리 (m)."""
    return raycasting_scene(mesh).compute_distance(o3d.core.Tensor(np.asarray(points, dtype=np.float32))).numpy()


def surface_error(simple, ref, n=300000):
    """경량화 메쉬 품질 (cm, %): 줄인 면이 원래 면에서 벗어난 정도 + 원래 면을 1cm 안으로 덮은 비율."""
    d = distance_to(ref, simple.sample_points_uniformly(n).points)
    back = distance_to(simple, ref.sample_points_uniformly(n).points)
    return dict(median_cm=round(100 * float(np.median(d)), 2), p95_cm=round(100 * float(np.percentile(d, 95)), 2),
                covered_1cm_pct=round(100 * float((back < 0.01).mean()), 1))


def make_ground(cfg, scene, poses):
    """실외 바닥 높이 지도 (config ground). 카메라 장착 높이 auto = 카메라에서 아래로 쏜 광선이 메쉬에 닿는 거리의 중앙값."""
    g = cfg["ground"]
    cams = np.array([P[:3, 3] for P in poses])
    h = g.get("camera_height_m", "auto")
    if h == "auto":
        rays = np.c_[cams, np.tile([0, 0, -1], (len(cams), 1))].astype(np.float32)
        t = raycasting_scene(scene).cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
        h = float(np.median(t[np.isfinite(t)]))
    ground = Ground.from_mesh(scene, cams - [0, 0, h], g["cell_m"], g["band_m"])
    ground.stats["camera_height_m"] = round(h, 3)
    return ground


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", help="결과 폴더 (기본: config 의 out). geometry 가 없으면 tsdf.py 출력 폴더")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))
    out = os.path.expanduser(args.out or cfg["out"])
    os.makedirs(out, exist_ok=True)
    geo = cfg.get("geometry")
    if geo:           # 형상 메쉬를 받은 경우: TSDF 단계 없이 그 메쉬로
        root = os.path.expanduser(cfg["input"]["root"])
        mesh_path = os.path.join(root, geo["mesh"])
        far_path = os.path.join(root, geo["far_mesh"]) if geo.get("far_mesh") else None
        report = dict(name=cfg["name"], geometry=dict(mesh=mesh_path, far_mesh=far_path))
    else:
        mesh_path, far_path = os.path.join(out, "mesh_raw.ply"), os.path.join(out, "mesh_far_raw.ply")
        report = dict(name=cfg["name"], tsdf=json.load(open(os.path.join(out, "tsdf_meta.json"), encoding="utf-8")))
    t0 = time.time()
    o3d.utility.random.seed(0)        # 바닥 평면 찾기(RANSAC)·점 뽑기가 실행마다 같게
    outdoor = "ground" in cfg
    fr = io_rgbd.load(cfg["input"]) if outdoor or "colorize" in cfg else None

    raw = o3d.io.read_triangle_mesh(mesh_path)
    raw.compute_vertex_normals()
    print(f"[1/8] 불러오기: 삼각형 {len(raw.triangles):,}")

    clean, report["cleanup"] = cleanup(raw, cfg["cleanup"]["min_cluster_tris"])
    del raw
    print(f"[2/8] 정리: 조각 {report['cleanup']['small_clusters']:,}개 · 삼각형 {report['cleanup']['removed_tris']:,}개 삭제")

    hint = cfg["orient"]["up_hint"]
    up = np.array(report["tsdf"]["up_hint_camera"] if hint == "camera" else hint, dtype=float)
    scene, T, report["orient"] = orient(clean, up / np.linalg.norm(up), cfg["orient"], crop=not outdoor)
    del clean
    json.dump(dict(source=mesh_path, T_input_to_scene=T.tolist(), orient=cfg["orient"]),
              open(os.path.join(out, "transform.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    poses = [T @ P for P in fr.poses] if fr else None              # 카메라 경로도 같은 씬 좌표로
    ground = None
    if outdoor:       # 바닥이 평평하지 않음 → 바닥 높이 지도. 바닥 아래 잡음은 그 높이 기준으로 잘라냄
        ground = make_ground(cfg, scene, poses)
        v = np.asarray(scene.vertices)
        below = v[:, 2] < ground.height(v[:, :2]) - cfg["orient"]["crop_below_floor"]
        scene.remove_vertices_by_mask(below)
        scene.compute_vertex_normals()
        ground.save(os.path.join(out, "ground.npz"))
        report["ground"] = dict(**ground.stats, cropped_below_vertices=int(below.sum()))
    print(f"[3/8] Z-up: 범위 {report['orient']['extent_m']} m | 바닥 기울기 {report['orient']['floor_tilt_from_hint_deg']}°"
          + (f" | 카메라 높이 {report['ground']['camera_height_m']} m · 바닥 아래 잡음 정점 {report['ground']['cropped_below_vertices']:,} 삭제"
             if outdoor else f" | 바닥 정점 {report['orient']['floor_vertex_pct']}%"))

    f = cfg.get("fill", {})
    report["fill"] = {}
    if f.get("small_hole_size_m", 0) > 0:
        scene, report["fill"]["small_hole_tris_added"] = fill_small_holes(scene, f["small_hole_size_m"])
    if f.get("floor_patch", False):
        patch, report["fill"]["floor_cells"] = floor_patch(scene, f["floor_cell_m"], f["floor_band_m"], ground=ground,
                                                         min_floor_verts=f.get("min_floor_verts", 3),
                                                         path_xy=fill_path_xy(cfg, poses, T),
                                                         max_path_dist=f.get("max_path_dist_m"))
        report["fill"]["floor_area_m2"] = round(report["fill"]["floor_cells"] * f["floor_cell_m"] ** 2, 2)
        scene = scene + patch
        o3d.io.write_triangle_mesh(os.path.join(out, "floor_patch.ply"), patch)
    print(f"[4/8] 구멍 메우기: 작은 구멍 삼각형 +{report['fill'].get('small_hole_tris_added', 0):,}"
          f" | 바닥 채움 {report['fill'].get('floor_area_m2', 0)} m²")

    if "colorize" in cfg:
        report["colorize"] = colorize(cfg, out, scene, ground, fr, poses, T)
        print(f"      색 투영: 키프레임 {report['colorize']['keyframes']}장 | 색이 보인 정점 {report['colorize']['seen_pct']}%"
              f" | 키프레임 depth ↔ 메쉬 차이 중앙 {report['colorize']['depth_vs_mesh_median_cm']} cm")

    d = cfg["decimate"]
    if d.get("visual_tris") and len(scene.triangles) > d["visual_tris"]:
        visual = scene.simplify_quadric_decimation(target_number_of_triangles=d["visual_tris"])
        visual.compute_vertex_normals()
        report["visual"] = dict(triangles=len(visual.triangles), **surface_error(visual, scene))
        print(f"[5/8] 시각용: 삼각형 {len(visual.triangles):,} | 오차 중앙 {report['visual']['median_cm']} cm · 95% {report['visual']['p95_cm']} cm")
    else:             # 정점 색은 평평한 바닥부터 줄어 색(주차선)이 뭉개짐 → 줄이지 않음 (visual_tris: 0)
        visual = scene
        report["visual"] = dict(triangles=len(visual.triangles), decimated=False)
        print(f"[5/8] 시각용: 경량화 안 함 (삼각형 {len(visual.triangles):,})")

    collider = visual.simplify_quadric_decimation(target_number_of_triangles=d["collider_tris"])
    collider.vertex_colors = o3d.utility.Vector3dVector()
    collider.compute_vertex_normals()
    pts = np.asarray(visual.sample_points_uniformly(300000).points)
    dist = distance_to(collider, pts)
    floor = np.abs(pts[:, 2] - (ground or Ground()).height(pts[:, :2])) < 0.03
    report["collider"] = dict(triangles=len(collider.triangles),
                              median_cm=round(100 * float(np.median(dist)), 2), p95_cm=round(100 * float(np.percentile(dist, 95)), 2),
                              floor_p95_cm=round(100 * float(np.percentile(dist[floor], 95)), 2),
                              floor_max_cm=round(100 * float(dist[floor].max()), 2))
    print(f"[6/8] 충돌용: 삼각형 {len(collider.triangles):,} | 바닥 오차 95% {report['collider']['floor_p95_cm']} cm · 최대 {report['collider']['floor_max_cm']} cm")

    visuals, colliders = {"Visual": visual}, {"Collision": collider}
    report["zones"] = zones(cfg, far_path, T, scene, visual, collider, visuals, colliders, ground)
    if report["zones"]:
        zr = report["zones"]
        print(f"[7/8] 구역: 바깥 구역 삼각형 {zr['far_tris']:,} | 바닥 충돌면 {zr['ground_area_m2']} m²"
              f" | 배경 벽 {zr['backdrop_height_m']} m · 경계벽 {zr['boundary_height_m']} m")
    else:
        print("[7/8] 구역: 끔 (config 에 zones 없음)")

    # visual.ply = 보이는 부분 전부 (점검·보기용), 부분별 파일도 따로
    o3d.io.write_triangle_mesh(os.path.join(out, "visual.ply"), merge(visuals.values()))
    o3d.io.write_triangle_mesh(os.path.join(out, "collider.ply"), colliders["Collision"])
    for name, fname in (("VisualFar", "visual_far.ply"), ("Backdrop", "backdrop.ply"), ("Boundary", "boundary.ply")):
        path = os.path.join(out, fname)
        if name in visuals or name in colliders:
            o3d.io.write_triangle_mesh(path, visuals[name] if name in visuals else colliders[name])
        elif os.path.exists(path):
            os.remove(path)
    scene_usd, mesh_usd = usd_utils.write_scene(
        os.path.join(out, "usd"), cfg["name"], visuals, colliders,
        info=dict(config=os.path.basename(args.config), transform="../transform.json",
                  **{f"{k}_tris": len(m.triangles) for k, m in {**visuals, **colliders}.items()}))
    problems, summary = usd_utils.verify_scene(scene_usd, mesh_usd)
    report["usd"] = dict(scene=scene_usd, mesh=mesh_usd, mesh_mb=round(os.path.getsize(mesh_usd) / 1e6, 2),
                         prims=summary, problems=problems)
    report["seconds"] = round(time.time() - t0, 1)
    json.dump(report, open(os.path.join(out, "report.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[8/8] USD: {scene_usd} (+ 메쉬 {report['usd']['mesh_mb']} MB) | 검증 {'통과' if not problems else '문제 ' + str(problems)}")
    print(f"완료 {report['seconds']}s → {os.path.join(out, 'report.json')}")
    if problems:
        raise SystemExit(1)


def fill_path_xy(cfg, poses, T):
    """바닥 채우기 거리 제한에 쓸 주행 경로 (xy). 보조 키프레임(반시계 회차 등)이 있으면 그 경로도 포함."""
    if not poses:
        return None
    xy = [np.array([P[:2, 3] for P in poses])]
    extra = cfg.get("colorize", {}).get("extra_input")
    if extra:
        xy.append(np.array([(T @ P)[:2, 3] for P in io_rgbd.load(extra).poses]))
    return np.vstack(xy)


def colorize(cfg, out, scene, ground, fr, poses, T):
    """키프레임 RGB 를 씬 메쉬(구멍 메운 조각 포함)에 투영 → 정점 색. 키프레임별 depth ↔ 메쉬 차이는 color_qc.csv (B 피드백용)."""
    c = cfg["colorize"]
    v, n = np.asarray(scene.vertices), np.asarray(scene.vertex_normals)
    pts = v.copy()
    lift = c.get("floor_lift_m", 0.0)
    if lift and ground is not None:   # 키프레임 depth 의 바닥이 메쉬 바닥보다 높게 나옴 → 그 높이에서 투영해야 사진의 같은 자리
        on_floor = (n[:, 2] > 0.9) & (np.abs(v[:, 2] - ground.height(v[:, :2])) < cfg["ground"]["band_m"])
        pts[on_floor, 2] += lift
    occ = raycasting_scene(scene)
    colors, seen, rows = project_colors(pts, n, fr, poses, occ, c, qc=True)
    extra = {}
    if c.get("extra_input"):          # 보조 키프레임 (예: 반시계 회차) — 주 키프레임이 못 본 점에만, 제외 구역 빼고
        fr2 = io_rgbd.load(c["extra_input"])
        todo = ~seen
        for x0, x1, y0, y1 in c.get("extra_exclude_xy", []):   # 씬 좌표 (origin keep 이면 B 맵 좌표와 같음)
            todo &= ~((v[:, 0] >= x0) & (v[:, 0] <= x1) & (v[:, 1] >= y0) & (v[:, 1] <= y1))
        idx = np.where(todo)[0]
        col2, seen2, _ = project_colors(pts[idx], n[idx], fr2, [T @ P for P in fr2.poses], occ, c)
        colors[idx[seen2]] = col2[seen2]
        seen[idx[seen2]] = True
        extra = dict(extra_keyframes=len(fr2.poses), extra_colored_pct=round(100 * float(seen2.sum()) / len(seen), 1))
        colors = fill_unseen(v, n, colors, seen)
    scene.vertex_colors = o3d.utility.Vector3dVector(colors)
    with open(os.path.join(out, "color_qc.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    med = [r["median_abs_cm"] for r in rows if r["median_abs_cm"] is not None]
    bias = [r["bias_cm"] for r in rows if r["bias_cm"] is not None]
    worst = sorted((r for r in rows if r["median_abs_cm"] is not None), key=lambda r: -r["median_abs_cm"])[:5]
    return dict(keyframes=len(rows), seen_pct=round(100 * float(seen.mean()), 1), floor_lift_m=lift, **extra,
                depth_vs_mesh_median_cm=round(float(np.median(med)), 1), depth_vs_mesh_bias_cm=round(float(np.median(bias)), 1),
                worst_keyframes=[dict(id=r["id"], x=r["x"], y=r["y"], median_abs_cm=r["median_abs_cm"]) for r in worst])


def merge(meshes):
    out = o3d.geometry.TriangleMesh()
    for m in meshes:
        m = o3d.geometry.TriangleMesh(m)
        if not m.has_vertex_colors():
            m.paint_uniform_color([0.6, 0.6, 0.6])
        out += m
    out.compute_vertex_normals()
    return out


def zones(cfg, far_path, T, scene, visual, collider, visuals, colliders, ground=None):
    """주행 구역(근거리) 밖을 채운다. 결과는 visuals / colliders 에 부분별로 추가.
    - VisualFar : 원거리 메쉬 중 근거리 지도 밖 부분 (보이기만, 충돌 없음 — 잡음 바닥에 걸리지 않게)
    - GroundFar : 근거리 지도 밖의 보이는 바닥 면 (바깥 구역 바닥 구멍이 비쳐 보이지 않게, 색 = 가까운 바닥 색)
    - Collision : 근거리 충돌 메쉬 + 전체 지도보다 ground_margin_m 넓은 바닥 면
    - Backdrop  : 그 바깥 테두리의 배경 벽 (보이기만)
    - Boundary  : 같은 테두리의 경계벽 (안 보이고 충돌만 — 로봇이 지도 밖으로 못 나감)
    ground 가 있으면 바닥 면·벽은 그 높이를 따라간다 (실외)."""
    z = cfg.get("zones")
    if not z:
        return {}
    ground = ground or Ground()
    far_vis = None
    near_fp = Footprint(scene, z["near_cell_m"])
    if far_path and os.path.exists(far_path):
        far = o3d.io.read_triangle_mesh(far_path).transform(T)
        v, tri = np.asarray(far.vertices), np.asarray(far.triangles)
        cen = v[tri].mean(axis=1)
        far.remove_triangles_by_mask(near_fp.inside(cen[:, :2])
                                     | (cen[:, 2] < ground.height(cen[:, :2]) - cfg["orient"]["crop_below_floor"]))
        far.remove_unreferenced_vertices()
        far, _ = cleanup(far, cfg["cleanup"]["min_cluster_tris"])
        if len(far.triangles) > z["far_tris"]:
            far = far.simplify_quadric_decimation(target_number_of_triangles=z["far_tris"])
        far.compute_vertex_normals()
        if len(far.triangles):
            far_vis = visuals["VisualFar"] = far

    xy = np.asarray(visual.vertices)[:, :2]
    if far_vis is not None:
        xy = np.vstack([xy, np.asarray(far_vis.vertices)[:, :2]])
    outer = Footprint(xy, z["ground_cell_m"], margin=z["ground_margin_m"])
    colliders["Collision"] = collider + outer.plane(-0.002, ground=ground)
    # 근거리 메쉬 가장자리 아래로 20 cm 겹치게 (칸 단위로 피해 가면 경계에 틈이 생겨 비쳐 보임)
    ground_far = outer.plane(-0.002, exclude=near_fp.shrunk(0.2), ground=ground) if z.get("ground_far", True)         else o3d.geometry.TriangleMesh()
    if len(ground_far.triangles):
        ground_far.vertex_colors = o3d.utility.Vector3dVector(floor_colors(
            [visual, far_vis], np.asarray(ground_far.vertices), ground=ground, mode=z.get("ground_color", "nearest")))
        visuals["GroundFar"] = ground_far
    if z.get("boundary_height_m", 0) > 0:
        colliders["Boundary"] = outer.walls(z["boundary_height_m"], ground=ground)
    if z.get("backdrop_height_m", 0) > 0:
        backdrop = outer.walls(z["backdrop_height_m"], ground=ground)
        color = z.get("backdrop_color", "mean")
        if color == "mean":       # 벽처럼 옆을 향한 면들의 평균 색 (가까운 색을 늘이면 세로 줄무늬가 생김)
            n, c = np.asarray(visual.vertex_normals), np.asarray(visual.vertex_colors)
            color = c[np.abs(n[:, 2]) < 0.3].mean(axis=0) if visual.has_vertex_colors() else [0.6, 0.6, 0.6]
        backdrop.paint_uniform_color(color)
        visuals["Backdrop"] = backdrop
    return dict(far_tris=len(far_vis.triangles) if far_vis is not None else 0, ground_area_m2=round(outer.area(), 1),
                ground_margin_m=z["ground_margin_m"], backdrop_height_m=z.get("backdrop_height_m", 0),
                boundary_height_m=z.get("boundary_height_m", 0))


if __name__ == "__main__":
    main()
