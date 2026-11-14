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

from scipy import ndimage
from scipy.spatial import cKDTree

import floor_texture
from edge_finish import grid_mesh
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
        idx = np.where(free & (area > 0))[0]                    # 넓이 0 삼각형(촘촘한 TSDF 메쉬에 섞여 있음)은 뽑을 수 없음 — 복도 v0.2 에서 오류 (11.14)
        if len(idx) < 100:
            break
        pick = rng.choice(idx, size=min(len(idx), 60000), replace=False, p=area[idx] / area[idx].sum())
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(cen[pick]))
        (a, b, cc, d), _ = pc.segment_plane(c.get("dist_m", 0.03), 3, 500)
        n = np.array([a, b, cc])
        if abs(n[2]) > np.sin(np.radians(c.get("max_tilt_deg", 10))):
            free[pick] = False                                  # 수평에 가까운 평면 (탁자 등) — 그 점들은 빼고 다시
            continue
        on = free & (np.abs(cen @ n + d) < c.get("dist_m", 0.03)) & (np.abs(tn @ n) > c.get("min_normal_cos", 0.9))
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
    cen = np.asarray(vis.vertices)[tri].mean(1)
    vcol = np.asarray(vis.vertex_colors)
    taken = np.zeros(len(tri), bool)
    mats, stats = {}, []
    for k, (n, d, on) in enumerate(walls):
        A = frame(n, d)
        part = copy.deepcopy(vis)
        part.triangles = o3d.utility.Vector3iVector(tri[on])
        if vis.has_triangle_normals():
            part.triangle_normals = o3d.utility.Vector3dVector(np.asarray(vis.triangle_normals)[on])
        part.remove_unreferenced_vertices()
        if c.get("sheet"):   # 벽 한 장 (복도 v0.2, 11.14): 손 촬영 TSDF 벽은 깊이 잡음 · 묶음 사이 2~3 cm 어긋남으로 울퉁불퉁하고 면 방향도 들쭉날쭉함 —
            #  바닥 한 장(single_floor)과 같은 생각으로, 벽 평면 dist_m 안의 스캔 면을 모두 빼고 그 범위에 평평한 격자 한 장을 새로 깖.
            #  보완(만든 것): 벽의 위치 · 범위(문 · 끊긴 곳 포함)는 측정, 평평함은 가정. 충돌 메쉬는 스캔 그대로.
            #  (정점만 평면으로 옮기는 방식은 벽 삼각형 일부만 옮겨져 구멍이 숭숭 뚫렸음 — 보이는 구멍 7.4 → 11.3%)
            cw = cen @ A[:3, :3].T + A[:3, 3]
            sc = c.get("sheet_cell_m", 0.05)
            lo2 = cw[on, :2].min(0) - 4 * sc
            shp = np.ceil((cw[on, :2].max(0) - lo2) / sc).astype(int) + 5

            def cells(mask):
                q = ((cw[mask, :2] - lo2) / sc).astype(int)
                return q, (q >= 0).all(1) & (q < shp).all(1)
            M = np.zeros(shp, bool)
            q, ok = cells(on)
            M[q[ok, 0], q[ok, 1]] = True
            near = ~taken & ~on & (np.abs(cw[:, 2]) < c.get("dist_m", 0.03))        # 면 방향이 안 맞아 벽으로 안 잡힌 울퉁불퉁한 조각도 같은 벽 자리면 뺌
            Md = ndimage.binary_dilation(M, iterations=max(1, int(round(0.15 / sc))))
            qn, okn = cells(near)
            hit = np.zeros(len(qn), bool)
            hit[okn] = Md[qn[okn, 0], qn[okn, 1]]
            on = on.copy()
            on[np.where(near)[0][hit]] = True
            q, ok = cells(on)
            M[q[ok, 0], q[ok, 1]] = True
            M = ndimage.binary_closing(M, iterations=max(1, int(round(c.get("sheet_close_m", 0.15) / sc))))
            lab, nl = ndimage.label(M)
            if nl:
                M = np.isin(lab, 1 + np.nonzero(ndimage.sum(M, lab, range(1, nl + 1)) * sc * sc >= c.get("sheet_min_m2", 0.3))[0])
            sheet = grid_mesh(M, lo2, sc, lambda xy: np.zeros(len(xy)))
            if len(sheet.triangles):
                if vis.has_vertex_colors():                                        # 사진이 안 닿은 텍셀이 쓸 색 = 가장 가까운 스캔 벽 색
                    _, j = cKDTree(cw[on, :2]).query(np.asarray(sheet.vertices)[:, :2])
                    sheet.vertex_colors = o3d.utility.Vector3dVector(vcol[tri[on][j, 0]])
                sheet.transform(np.linalg.inv(A))
                sheet.compute_vertex_normals()
                part = sheet
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
