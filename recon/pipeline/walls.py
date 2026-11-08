"""벽에 키프레임 사진 입히기 (실내 — 복도는 화면 절반이 벽). 정점 색(꼭짓점마다 색 하나)은 벽 글씨 · 문틀 · 표지판이 뭉개짐.

1) 시각용 메쉬에서 큰 수직 평면(벽)을 RANSAC 으로 찾음 (넓이 min_area_m2 이상, 기울기 max_tilt_deg 안)
2) 벽마다 좌표를 돌려 '벽 = 바닥(z=0), 위 = +y' 로 놓고 floor_texture.bake 를 그대로 씀
   (가림 · 키프레임 depth 비교 · 화소 크기 순위 · 가장자리 감쇠 · 반사 거르기가 바닥과 같음)
3) 벽 삼각형은 Visual 에서 떼어 Wall<k> prim 으로 — 형상은 스캔 그대로, 색만 텍스처 (st = 벽 좌표)
config walls: {min_area_m2, max_walls, dist_m, max_tilt_deg, cell_m, max_dist_m, top_k, power, border_px, feather_px, min_cos, specular,
              require_depth, depth_tol_m, depth_tol_per_m, consensus}  — 벽은 키프레임 depth 가 벽 거리와 맞는 화소만 (앞의 화분 · 의자가 번지지 않게)
"""
import copy
import os

import numpy as np
import open3d as o3d
from PIL import Image

import floor_texture
from mesh_utils import Ground


def frame(n, d):
    """벽 평면 n·x + d = 0 (n 은 카메라 쪽) → 4x4: 세계 → 벽 좌표 (z = 벽에서 떨어진 거리, y = 위, x = 벽 따라)."""
    z = n / np.linalg.norm(n)
    y = np.array([0.0, 0.0, 1.0]) - z[2] * z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    A = np.eye(4)
    A[:3, :3] = np.stack([x, y, z])
    A[:3, 3] = [0.0, 0.0, d]                                    # n·p + d = 벽까지 거리 → 벽 위 점은 z = 0
    return A


def find(mesh, cams, c):
    """→ [(n, d, 삼각형 bool)] 넓은 벽부터. n 은 카메라가 많은 쪽으로."""
    tri = np.asarray(mesh.triangles)
    v = np.asarray(mesh.vertices)
    cen = v[tri].mean(1)
    mesh.compute_triangle_normals()
    tn = np.asarray(mesh.triangle_normals)
    area = 0.5 * np.linalg.norm(np.cross(v[tri[:, 1]] - v[tri[:, 0]], v[tri[:, 2]] - v[tri[:, 0]]), axis=1)
    vert = np.abs(tn[:, 2]) < np.sin(np.radians(c.get("max_tilt_deg", 10)))
    free = vert.copy()
    walls = []
    rng = np.random.default_rng(0)
    o3d.utility.random.seed(0)
    for _ in range(c.get("max_walls", 12) * 3):
        if len(walls) >= c.get("max_walls", 12) or free.sum() < 100:
            break
        idx = np.where(free)[0]
        pick = rng.choice(idx, size=min(len(idx), 60000), replace=False, p=area[idx] / area[idx].sum())
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cen[pick]))
        (a, b, cc, d), _ = pc.segment_plane(c.get("dist_m", 0.03), 3, 500)
        n = np.array([a, b, cc])
        if abs(n[2]) > np.sin(np.radians(c.get("max_tilt_deg", 10))):
            free[pick] = False                                  # 수평에 가까운 평면 (탁자 등) — 그 점들은 빼고 다시
            continue
        on = free & (np.abs(cen @ n + d) < c.get("dist_m", 0.03)) & (np.abs(tn @ n) > 0.9)
        if area[on].sum() < c.get("min_area_m2", 2.0):
            free[on | np.isin(np.arange(len(free)), pick)] = False
            continue
        if np.median(cams @ n + d) < 0:                         # 카메라가 있는 쪽이 +
            n, d = -n, -d
        walls.append((n, float(d), on))
        free &= ~on
    return walls, area


def run(cfg, out, visuals, occ_mesh, sets):
    """visuals["Visual"] 에서 벽을 떼어 Wall<k> 로. sets = [(이름, Frames, 씬 좌표 poses, 바닥 올림, 제외 구역)].
    → (재질 dict {Wall<k>: 텍스처 정보}, 통계)"""
    c = dict(dict(cell_m=0.01, max_dist_m=5.0, top_k=5, power=3.0, border_px=6, feather_px=40, min_cos=0.35,
                  require_depth=True, depth_tol_m=0.05, depth_tol_per_m=0.02, consensus={"max_diff": 0.15}), **cfg["walls"])
    vis = visuals["Visual"]
    cams = np.vstack([np.array([P[:3, 3] for P in s[2]]) for s in sets])
    walls, area = find(vis, cams, c)
    if not walls:
        return {}, dict(walls=0)
    tri = np.asarray(vis.triangles)
    taken = np.zeros(len(tri), bool)
    mats, stats = {}, []
    for k, (n, d, on) in enumerate(walls):
        A = frame(n, d)
        part = copy.deepcopy(vis)
        part.triangles = o3d.utility.Vector3iVector(tri[on])
        if vis.has_triangle_normals():
            part.triangle_normals = o3d.utility.Vector3dVector(np.asarray(vis.triangle_normals)[on])
        part.remove_unreferenced_vertices()
        flat = copy.deepcopy(part).transform(A)                 # 벽 좌표: 벽 = z 0 평면
        occ = o3d.t.geometry.RaycastingScene()
        occ.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(copy.deepcopy(occ_mesh).transform(A)))
        c["label"] = f"벽 Wall{k} 텍스처"
        wsets = [(name, fr, [A @ P for P in poses], 0.0, []) for name, fr, poses, _, _ in sets]
        dk = os.path.join(out, "wall_tex", f"wall{k}")
        os.makedirs(dk, exist_ok=True)
        region, st = floor_texture.bake(wsets, flat, Ground(), occ, c, dk)
        color = os.path.join(dk, f"wall{k}_color.jpg")
        Image.open(os.path.join(dk, "bake_raw.png")).convert("RGB").save(color, quality=c.get("jpeg_quality", 92))
        name = f"Wall{k}"
        visuals[name] = part
        mats[name] = dict(color=color, region=region, uv_T=A.tolist(), tile_m=1.0)
        taken |= on
        x0, x1, y0, y1 = region
        stats.append(dict(name=name, area_m2=round(float(area[on].sum()), 1), size_m=[round(x1 - x0, 2), round(y1 - y0, 2)],
                          normal=np.round(n, 3).tolist(), seen_pct=st["seen_pct"], gsd_cm_p10_50_90=st["gsd_cm_p10_50_90"],
                          specular=st.get("specular")))
        print(f"      벽 {name}: {stats[-1]['area_m2']} m² ({x1 - x0:.1f} × {y1 - y0:.1f} m) · 사진으로 칠한 텍셀 {st['seen_pct']}%"
              f" · 화소 크기 중앙 {st['gsd_cm_p10_50_90'][1]} cm")
    rest = copy.deepcopy(vis)
    rest.triangles = o3d.utility.Vector3iVector(tri[~taken])
    if vis.has_triangle_normals():
        rest.triangle_normals = o3d.utility.Vector3dVector(np.asarray(vis.triangle_normals)[~taken])
    rest.remove_unreferenced_vertices()
    visuals["Visual"] = rest
    return mats, dict(walls=len(walls), wall_area_m2=round(float(sum(s["area_m2"] for s in stats)), 1), list=stats)
