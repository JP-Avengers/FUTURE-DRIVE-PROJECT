"""TSDF 메쉬 → 정리 → Z-up·원점 → 구멍 메우기 → 경량화 → 충돌 메쉬 → 구역(바깥·배경·경계) → USD → 검증.

    python recon/pipeline/build_usd.py --config recon/configs/lounge.yaml

입력 (<out>/): mesh_raw.ply (+ mesh_far_raw.ply), tsdf_meta.json  (tsdf.py 출력)
출력 (<out>/): visual.ply (보이는 부분 전부), collider.ply, transform.json, report.json,
               floor_patch.ply · visual_far.ply · backdrop.ply · boundary.ply (켠 경우),
               usd/<name>_scene.usda + usd/<name>_mesh.usdc
"""
import argparse, json, os, time

import numpy as np
import open3d as o3d
import yaml

import usd_utils
from mesh_utils import Footprint, fill_small_holes, floor_colors, floor_patch


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


def orient(mesh, up_hint, c):
    """바닥 법선 → +Z, 바닥 → z=0, (origin=min_corner 면) x·y 최솟값 모서리 → 원점."""
    n, floor_pts = find_floor(mesh, up_hint, c["max_tilt_deg"])
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(n, z)
    s = np.linalg.norm(axis)
    R = np.eye(3) if s < 1e-8 else o3d.geometry.get_rotation_matrix_from_axis_angle(axis / s * np.arctan2(s, n @ z))
    v = np.asarray(mesh.vertices) @ R.T
    shift = np.array([0.0, 0.0, np.median((floor_pts @ R.T)[:, 2])])
    if c["origin"] == "min_corner":
        shift[:2] = v[:, :2].min(0)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, -shift
    out = o3d.geometry.TriangleMesh(mesh).transform(T)
    bb = out.get_axis_aligned_bounding_box()
    out = out.crop(o3d.geometry.AxisAlignedBoundingBox(np.r_[bb.min_bound[:2], -c["crop_below_floor"]], bb.max_bound))
    out.compute_vertex_normals()
    zs = np.asarray(out.vertices)[:, 2]
    return out, T, dict(floor_tilt_from_hint_deg=round(float(np.degrees(np.arccos(np.clip(n @ up_hint, -1, 1)))), 1),
                        floor_vertex_pct=round(100 * float((np.abs(zs) < 0.02).mean()), 1),
                        extent_m=np.round(out.get_axis_aligned_bounding_box().get_extent(), 3).tolist())


def distance_to(mesh, points):
    """점들에서 mesh 표면까지 정확한 거리 (m)."""
    rs = o3d.t.geometry.RaycastingScene()
    rs.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return rs.compute_distance(o3d.core.Tensor(np.asarray(points, dtype=np.float32))).numpy()


def surface_error(simple, ref, n=300000):
    """경량화 메쉬 품질 (cm, %): 줄인 면이 원래 면에서 벗어난 정도 + 원래 면을 1cm 안으로 덮은 비율."""
    d = distance_to(ref, simple.sample_points_uniformly(n).points)
    back = distance_to(simple, ref.sample_points_uniformly(n).points)
    return dict(median_cm=round(100 * float(np.median(d)), 2), p95_cm=round(100 * float(np.percentile(d, 95)), 2),
                covered_1cm_pct=round(100 * float((back < 0.01).mean()), 1))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", help="tsdf.py 출력 폴더 (기본: config 의 out)")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))
    out = os.path.expanduser(args.out or cfg["out"])
    meta = json.load(open(os.path.join(out, "tsdf_meta.json"), encoding="utf-8"))
    report, t0 = dict(name=cfg["name"], tsdf=meta), time.time()

    raw = o3d.io.read_triangle_mesh(os.path.join(out, "mesh_raw.ply"))
    raw.compute_vertex_normals()
    print(f"[1/8] 불러오기: 삼각형 {len(raw.triangles):,}")

    clean, report["cleanup"] = cleanup(raw, cfg["cleanup"]["min_cluster_tris"])
    print(f"[2/8] 정리: 조각 {report['cleanup']['small_clusters']:,}개 · 삼각형 {report['cleanup']['removed_tris']:,}개 삭제")

    hint = cfg["orient"]["up_hint"]
    up = np.array(meta["up_hint_camera"] if hint == "camera" else hint, dtype=float)
    scene, T, report["orient"] = orient(clean, up / np.linalg.norm(up), cfg["orient"])
    json.dump(dict(source="mesh_raw.ply", T_input_to_scene=T.tolist(), orient=cfg["orient"]),
              open(os.path.join(out, "transform.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[3/8] Z-up: 범위 {report['orient']['extent_m']} m | 바닥 정점 {report['orient']['floor_vertex_pct']}%")

    f = cfg.get("fill", {})
    report["fill"] = {}
    if f.get("small_hole_size_m", 0) > 0:
        scene, report["fill"]["small_hole_tris_added"] = fill_small_holes(scene, f["small_hole_size_m"])
    if f.get("floor_patch", False):
        patch, report["fill"]["floor_cells"] = floor_patch(scene, f["floor_cell_m"], f["floor_band_m"])
        report["fill"]["floor_area_m2"] = round(report["fill"]["floor_cells"] * f["floor_cell_m"] ** 2, 2)
        scene = scene + patch
        o3d.io.write_triangle_mesh(os.path.join(out, "floor_patch.ply"), patch)
    print(f"[4/8] 구멍 메우기: 작은 구멍 삼각형 +{report['fill'].get('small_hole_tris_added', 0):,}"
          f" | 바닥 채움 {report['fill'].get('floor_area_m2', 0)} m²")

    d = cfg["decimate"]
    visual = scene.simplify_quadric_decimation(target_number_of_triangles=d["visual_tris"])
    visual.compute_vertex_normals()
    report["visual"] = dict(triangles=len(visual.triangles), **surface_error(visual, scene))
    print(f"[5/8] 시각용: 삼각형 {len(visual.triangles):,} | 오차 중앙 {report['visual']['median_cm']} cm · 95% {report['visual']['p95_cm']} cm")

    collider = visual.simplify_quadric_decimation(target_number_of_triangles=d["collider_tris"])
    collider.vertex_colors = o3d.utility.Vector3dVector()
    collider.compute_vertex_normals()
    pts = np.asarray(visual.sample_points_uniformly(300000).points)
    dist = distance_to(collider, pts)
    floor = np.abs(pts[:, 2]) < 0.03
    report["collider"] = dict(triangles=len(collider.triangles),
                              median_cm=round(100 * float(np.median(dist)), 2), p95_cm=round(100 * float(np.percentile(dist, 95)), 2),
                              floor_p95_cm=round(100 * float(np.percentile(dist[floor], 95)), 2),
                              floor_max_cm=round(100 * float(dist[floor].max()), 2))
    print(f"[6/8] 충돌용: 삼각형 {len(collider.triangles):,} | 바닥 오차 95% {report['collider']['floor_p95_cm']} cm · 최대 {report['collider']['floor_max_cm']} cm")

    visuals, colliders = {"Visual": visual}, {"Collision": collider}
    report["zones"] = zones(cfg, out, T, scene, visual, collider, visuals, colliders)
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


def merge(meshes):
    out = o3d.geometry.TriangleMesh()
    for m in meshes:
        m = o3d.geometry.TriangleMesh(m)
        if not m.has_vertex_colors():
            m.paint_uniform_color([0.6, 0.6, 0.6])
        out += m
    out.compute_vertex_normals()
    return out


def zones(cfg, out, T, scene, visual, collider, visuals, colliders):
    """주행 구역(근거리) 밖을 채운다. 결과는 visuals / colliders 에 부분별로 추가.
    - VisualFar : 원거리 메쉬 중 근거리 지도 밖 부분 (보이기만, 충돌 없음 — 잡음 바닥에 걸리지 않게)
    - GroundFar : 근거리 지도 밖의 보이는 바닥 평면 (바깥 구역 바닥 구멍이 비쳐 보이지 않게, 색 = 가까운 바닥 색)
    - Collision : 근거리 충돌 메쉬 + 전체 지도보다 ground_margin_m 넓은 바닥 평면
    - Backdrop  : 그 바깥 테두리의 배경 벽 (보이기만)
    - Boundary  : 같은 테두리의 경계벽 (안 보이고 충돌만 — 로봇이 지도 밖으로 못 나감)"""
    z = cfg.get("zones")
    if not z:
        return {}
    far_vis = None
    near_fp = Footprint(scene, z["near_cell_m"])
    far_path = os.path.join(out, "mesh_far_raw.ply")
    if os.path.exists(far_path):
        far = o3d.io.read_triangle_mesh(far_path).transform(T)
        v, tri = np.asarray(far.vertices), np.asarray(far.triangles)
        cen = v[tri].mean(axis=1)
        far.remove_triangles_by_mask(near_fp.inside(cen[:, :2]) | (cen[:, 2] < -cfg["orient"]["crop_below_floor"]))
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
    colliders["Collision"] = collider + outer.plane(-0.002)
    # 근거리 메쉬 가장자리 아래로 20 cm 겹치게 (칸 단위로 피해 가면 경계에 틈이 생겨 비쳐 보임)
    ground_far = outer.plane(-0.002, exclude=near_fp.shrunk(0.2))
    if len(ground_far.triangles):
        ground_far.vertex_colors = o3d.utility.Vector3dVector(floor_colors([visual, far_vis], np.asarray(ground_far.vertices)))
        visuals["GroundFar"] = ground_far
    if z.get("boundary_height_m", 0) > 0:
        colliders["Boundary"] = outer.walls(z["boundary_height_m"])
    if z.get("backdrop_height_m", 0) > 0:
        backdrop = outer.walls(z["backdrop_height_m"])
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
