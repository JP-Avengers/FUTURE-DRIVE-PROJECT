"""진단에 쓰는 계산: Nav2 지도 읽기 · 빈 바닥 표본 · 보이는 물체와 충돌 물체의 일치."""
import os

import numpy as np


def load_nav2(d):
    """make_map.py 가 만든 지도 폴더 → (정보 dict, 빈 칸 bool, 장애물 칸 bool). 행 0 = 위쪽(y 큰 쪽)."""
    import yaml
    from PIL import Image
    y = yaml.safe_load(open(os.path.join(d, "map.yaml"), encoding="utf-8"))
    im = np.asarray(Image.open(os.path.join(d, "map.pgm")))
    return dict(dir=d, origin=y["origin"][:2], cell=y["resolution"], shape=im.shape), im >= 250, im < 50


def free_xy(nav2, free, clearance, step=1):
    """장애물 · 미지 칸과 clearance 넘게 떨어진 빈 칸의 중심 (세계 좌표)과 그 거리."""
    from scipy import ndimage
    dist = ndimage.distance_transform_edt(free) * nav2["cell"]
    iy, ix = np.nonzero(dist >= clearance)
    iy, ix = iy[::step], ix[::step]
    xy = np.c_[nav2["origin"][0] + (ix + 0.5) * nav2["cell"], nav2["origin"][1] + (nav2["shape"][0] - 1 - iy + 0.5) * nav2["cell"]]
    return xy, dist[iy, ix]


def clusters(xy, nav2, join_cells=2):
    """표본점 xy 를 지도 격자에서 가까운 것끼리 묶음 (join_cells 칸 안이면 한 덩어리). → [dict(center=[x, y], radius_m, extent_m, samples)] 큰 것부터"""
    from scipy import ndimage
    if len(xy) == 0:
        return []
    H, W = nav2["shape"]
    c, (ox, oy) = nav2["cell"], nav2["origin"]
    ix = np.clip(np.floor((xy[:, 0] - ox) / c).astype(int), 0, W - 1)
    iy = np.clip(H - 1 - np.floor((xy[:, 1] - oy) / c).astype(int), 0, H - 1)
    g = np.zeros((H, W), bool)
    g[iy, ix] = True
    lab, _ = ndimage.label(ndimage.binary_dilation(g, iterations=join_cells), structure=np.ones((3, 3)))
    of = lab[iy, ix]
    out = []
    for k in np.unique(of):
        p = xy[of == k]
        cen = p.mean(0)
        out.append(dict(center=np.round(cen, 3).tolist(), radius_m=round(float(np.linalg.norm(p - cen, axis=1).max() + 2 * c), 2),
                        extent_m=round(float(np.linalg.norm(np.ptp(p, axis=0)) + c), 2), samples=int(len(p))))
    return sorted(out, key=lambda d: -d["samples"])


def drivable(nav2, free, occ, path_xy=None, clearance=0.3, step_m=0.1, tol_m=0.15):
    """지도가 다닐 수 있는 모양인가 — 빈 바닥이 한 덩어리인지, 촬영자가 걸어간 길(path_xy, 씬 좌표)이 지도에서 빈 자리인지.
    촬영자가 지나간 자리는 실제로 비어 있었으므로, 그 자리가 장애물로 찍혔다면 같은 곳이 다른 높이 · 위치로 겹쳐 찍힌 것(자세 틀어짐)이다.
    → dict(main_share_pct = 장애물과 clearance 띄운 빈 바닥 중 가장 큰 덩어리의 비율, pieces,
            path = {length_m, blocked_m, blocked_pct, main_piece_pct, blocked_xy}) — path 는 path_xy 가 있을 때만.
    막힘 = 그 자리가 장애물 칸이고 가장 가까운 빈 칸까지 tol_m 넘음 (자세 보정 전 경로를 쓰므로 그만큼은 봐줌)."""
    from scipy import ndimage
    H, W = nav2["shape"]
    c, (ox, oy) = nav2["cell"], nav2["origin"]
    wide = ndimage.distance_transform_edt(free) * c >= clearance
    lab, n = ndimage.label(wide, structure=np.ones((3, 3)))
    sizes = np.bincount(lab.ravel())[1:] if n else np.zeros(0)
    out = dict(clearance_m=clearance, free_m2=round(float(free.sum()) * c * c, 1), pieces=int((sizes * c * c >= 1.0).sum()),
               main_share_pct=round(100 * float(sizes.max()) / max(float(sizes.sum()), 1.0), 1) if n else 0.0)
    # 로봇이 제자리에서 돌 수 있는 넓이 — sim/add_site.py 의 맵 판정(area_clear_ge_footprint_m2)과 같은 계산: 빈 칸이 아닌 칸까지 로봇 대각선(0.607 · 0.25 m) 넘게 떨어진 빈 칸
    foot = float(np.hypot(0.607, 0.25))
    out.update(turn_radius_m=round(foot, 3), turn_area_m2=round(float((ndimage.distance_transform_edt(free) * c >= foot).sum()) * c * c, 2))
    if path_xy is None or len(path_xy) < 2:
        return out
    seg = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    s = np.r_[0, np.cumsum(seg)]
    t = np.arange(0, s[-1], step_m)
    p = np.c_[np.interp(t, s, path_xy[:, 0]), np.interp(t, s, path_xy[:, 1])]
    ix = np.floor((p[:, 0] - ox) / c).astype(int)
    iy = H - 1 - np.floor((p[:, 1] - oy) / c).astype(int)
    ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    far = ndimage.distance_transform_edt(~free) * c > tol_m     # 빈 칸까지 tol_m 넘게 떨어진 칸
    blocked = np.zeros(len(p), bool)
    blocked[ok] = occ[iy[ok], ix[ok]] & far[iy[ok], ix[ok]]
    lab0, _ = ndimage.label(ndimage.binary_dilation(free, iterations=int(round(tol_m / c))), structure=np.ones((3, 3)))   # 길이 이어져 있는가 — 빈 칸을 tol_m 만큼 넓혀서
    on = np.zeros(len(p), int)
    on[ok] = lab0[iy[ok], ix[ok]]
    cnt = np.bincount(on[on > 0]) if (on > 0).any() else np.zeros(1)
    out["path"] = dict(length_m=round(float(s[-1]), 1), blocked_m=round(float(blocked.sum()) * step_m, 1), blocked_pct=round(100 * float(blocked.mean()), 1),
                       main_piece_pct=round(100 * float(cnt.max()) / len(p), 1), blocked_xy=p[blocked])
    return out


def obstacle_match(usd_mesh, ground, nav2, free, h_lo=0.12, h_hi=1.0, tol_m=0.10, near_m=1.5):
    """보이는 물체와 충돌 물체가 같은 자리에 있는가 — 라이다는 보이는 형상을 보고 Nav2 지도는 충돌 형상으로 만들기 때문에, 둘이 다르면 위치 추정이 흔들린다.
    바닥 위 h_lo~h_hi 높이의 면을 지도 격자에 찍어 (보이는 prim 전체 / Collision), 로봇이 다니는 빈 바닥에서 near_m 안의 칸만 견줌.
    → 보이는 칸 중 tol_m 안에 충돌 칸이 있는 비율 · 그 반대 비율 (%). 경계벽(Boundary)은 보이지 않으므로 뺌."""
    import open3d as o3d
    from pxr import Usd, UsdGeom, UsdPhysics
    from scipy import ndimage
    st = Usd.Stage.Open(usd_mesh)
    H, W = nav2["shape"]
    c, (ox, oy) = nav2["cell"], nav2["origin"]

    def grid(prims):
        g = np.zeros((H, W), bool)
        for p in prims:
            m = UsdGeom.Mesh(p)
            P = np.array(m.GetPointsAttr().Get(), float)
            F = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
            if len(F) == 0:
                continue
            cen = P[F].mean(1)
            rel = cen[:, 2] - ground.height(cen[:, :2])
            F = F[(rel > h_lo) & (rel < h_hi)]
            if len(F) == 0:
                continue
            tm = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(P), o3d.utility.Vector3iVector(F))
            n = int(min(4_000_000, max(2000, tm.get_surface_area() / (0.5 * c) ** 2)))
            o3d.utility.random.seed(0)
            q = np.asarray(tm.sample_points_uniformly(n).points)
            ix = np.floor((q[:, 0] - ox) / c).astype(int)
            iy = H - 1 - np.floor((q[:, 1] - oy) / c).astype(int)
            ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
            g[iy[ok], ix[ok]] = True
        return g
    meshes = [p for p in st.Traverse() if p.IsA(UsdGeom.Mesh)]
    vis = grid([p for p in meshes if not p.HasAPI(UsdPhysics.CollisionAPI)])
    col = grid([p for p in meshes if p.GetName() == "Collision"])
    k = int(round(tol_m / c))
    near = ndimage.binary_dilation(free, iterations=int(round(near_m / c)))
    v, cl = vis & near, col & near
    v_in_c = (v & ndimage.binary_dilation(col, iterations=k)).sum() / max(v.sum(), 1)
    c_in_v = (cl & ndimage.binary_dilation(vis, iterations=k)).sum() / max(cl.sum(), 1)
    return dict(visible_cells=int(v.sum()), collision_cells=int(cl.sum()), visible_in_collision_pct=round(100 * float(v_in_c), 1), collision_in_visible_pct=round(100 * float(c_in_v), 1),
                tol_m=tol_m, height_band_m=[h_lo, h_hi])


def ceiling_over_floor(visual_ply, ground, nav2, free, min_h=1.8):
    """빈 바닥 위에 천장(아래를 향한 면, 바닥 위 min_h 넘는 높이)이 덮인 비율 (%). 입력을 미리 잴 수 없는 맵 폴더 · 기존 맵에서 천장 유무를 알아낼 때."""
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(visual_ply)
    m.compute_triangle_normals()
    v, t, n = np.asarray(m.vertices), np.asarray(m.triangles), np.asarray(m.triangle_normals)
    cen = v[t].mean(1)
    sel = (n[:, 2] < -0.8) & (cen[:, 2] - ground.height(cen[:, :2]) > min_h)
    H, W = nav2["shape"]
    c, (ox, oy) = nav2["cell"], nav2["origin"]
    g = np.zeros((H // 5 + 1, W // 5 + 1), bool)                 # 0.25 m 칸
    ix = (np.floor((cen[sel, 0] - ox) / c).astype(int)) // 5
    iy = (H - 1 - np.floor((cen[sel, 1] - oy) / c).astype(int)) // 5
    ok = (ix >= 0) & (ix < g.shape[1]) & (iy >= 0) & (iy < g.shape[0])
    g[iy[ok], ix[ok]] = True
    fy, fx = np.nonzero(free)
    return round(100 * float(g[fy // 5, fx // 5].mean()), 1) if len(fy) else 0.0
