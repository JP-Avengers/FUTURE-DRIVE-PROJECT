"""맵 테두리 마감 (v0.7). build_usd.py 가 구역 · 바닥 텍스처 다음에 부름.

10/30 팀 결정: 원경(VisualFar, 스캔 밖 키프레임 depth TSDF)은 건물처럼 보이지 않고 잡음이라 빼고(far_tsdf 끔),
스캔 바닥 경계를 다듬어 연석 + 인도 띠 + 나무 + 바깥 잔디로 깔끔하게 마무리한다 (10/7 '벽·인위적 마감 없이' 방침을 바꿈).
  1) 경계: 바닥 범위를 smooth_m 로 다듬은 매끈한 선 (안쪽 구멍 = 가운데 섬은 그대로)
  2) 경계 밖 정리: 바닥 · 스캔 물체 중 경계 밖 조각을 지움 (B 차 목록 자리는 남김). B 나무 자리의 스캔 나무 덩어리도 지움
  3) 연석(높이 curb_h_m · 윗면 curb_top_m) 을 경계선을 따라 세우고, 그 밖에 인도 띠(sidewalk_m) · 잔디(outer_m 까지)
  4) 나무: B 나무(위치 · 수관 지름 · 높이) 자리에 줄기 + 수관 모양 메쉬 (Isaac Sim 나무 에셋으로 바꾸기 쉽게 Trees prim 하나로)
  연석 · 인도 · 잔디 · 나무는 '보완(만든 것)' — 위치는 측정(바닥 경계 · B 나무 · 연석 실측 18 cm), 모양은 만든 것.
"""
import csv, os

import numpy as np
import open3d as o3d
from scipy import ndimage


def boundary_mask(floor, smooth_m, cell=0.1, pad=60.0):
    """바닥 메쉬 범위 → 다듬은 경계 안쪽 bool 격자 [ix, iy] (+ lo, cell). pad = 바깥 잔디까지 담을 여백."""
    v = np.asarray(floor.vertices)[:, :2]
    lo = v.min(0) - pad
    shape = np.ceil((v.max(0) + pad - lo) / cell).astype(int)
    fp = np.zeros(shape, bool)
    ij = ((v - lo) / cell).astype(int)
    fp[ij[:, 0], ij[:, 1]] = True
    fp = ndimage.binary_closing(fp, iterations=3)
    m = ndimage.gaussian_filter(ndimage.binary_fill_holes(fp).astype(np.float32), smooth_m / cell) > 0.5
    lab, n = ndimage.label(m)
    if n > 1:
        m = lab == (np.argmax(ndimage.sum(m, lab, range(1, n + 1))) + 1)
    return ndimage.binary_fill_holes(m), lo, cell, fp


def outlines(mask, lo, cell, step=0.2, min_len_m=3.0):
    """격자 mask 의 테두리들 → [(닫힌 선 (N,2) 간격 step, mask 바깥 방향 법선 (N,2))]."""
    import contourpy
    f = ndimage.gaussian_filter(mask.astype(np.float32), 1.0)
    out = []
    for p in contourpy.contour_generator(z=f.T).lines(0.5):              # z[iy, ix] → 선 좌표 (ix, iy)
        if len(p) * cell >= min_len_m:
            out.append(_resample(mask, lo + (p + 0.5) * cell, lo, cell, step))
    return out


def _resample(mask, p, lo, cell, step):
    seg = np.linalg.norm(np.diff(np.vstack([p, p[:1]]), axis=0), axis=1)
    s = np.r_[0, np.cumsum(seg)]
    t = np.arange(0, s[-1], step)
    pc = np.vstack([p, p[:1]])
    p = np.c_[np.interp(t, s, pc[:, 0]), np.interp(t, s, pc[:, 1])]
    tg = np.roll(p, -1, 0) - np.roll(p, 1, 0)
    n = np.c_[tg[:, 1], -tg[:, 0]]
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
    ij = np.clip(((p + 0.3 * n - lo) / cell).astype(int), 0, np.array(mask.shape) - 1)
    if mask[ij[:, 0], ij[:, 1]].mean() > 0.5:                         # 법선이 mask 안쪽을 보면 뒤집음
        n = -n
    return p, n


def _orient(m, want):
    """삼각형 법선이 want (3,) 쪽을 대체로 보도록 감는 방향을 맞춤."""
    m.compute_triangle_normals()
    if (np.asarray(m.triangle_normals) @ want).mean() < 0:
        m.triangles = o3d.utility.Vector3iVector(np.asarray(m.triangles)[:, ::-1])
    m.compute_vertex_normals()
    return m


def curb_mesh(p, n_road, ground, h, top_w):
    """선 p 를 따라 연석: 차로 쪽 면(n_road 쪽을 봄) + 윗면(차로 반대쪽으로 top_w)."""
    zg = ground.height(p)
    face = strip(np.c_[p, zg - 0.05], np.c_[p, zg + h])
    face.compute_triangle_normals()
    tn = np.asarray(face.triangle_normals)[:, :2]
    if (tn * np.r_[n_road, n_road][:len(tn)]).sum(1).mean() < 0:      # 띠 삼각형 순서 = 점 순서 두 번
        face.triangles = o3d.utility.Vector3iVector(np.asarray(face.triangles)[:, ::-1])
    q = p - n_road * top_w
    top = _orient(strip(np.c_[p, zg + h], np.c_[q, ground.height(q) + h]), np.array([0, 0, 1.0]))
    m = face + top
    m.compute_vertex_normals()
    return m


def grid_mesh(sel, lo, cell, zfun):
    """격자 칸 sel[ix, iy] → 칸마다 사각형 2개 (모서리 높이 = zfun(xy))."""
    ii, jj = np.nonzero(sel)
    if not len(ii):
        return o3d.geometry.TriangleMesh()
    corners = np.unique(np.r_[np.c_[ii, jj], np.c_[ii + 1, jj], np.c_[ii, jj + 1], np.c_[ii + 1, jj + 1]], axis=0)
    key = corners[:, 0] * (sel.shape[1] + 2) + corners[:, 1]
    order = np.argsort(key)
    key = key[order]
    corners = corners[order]

    def vid(a, b):
        return np.searchsorted(key, a * (sel.shape[1] + 2) + b)
    v00, v10, v01, v11 = vid(ii, jj), vid(ii + 1, jj), vid(ii, jj + 1), vid(ii + 1, jj + 1)
    xy = lo + corners * cell
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.c_[xy, zfun(xy)]),
                                  o3d.utility.Vector3iVector(np.r_[np.c_[v00, v10, v11], np.c_[v00, v11, v01]]))
    m.compute_vertex_normals()
    return m


def strip(a, b):
    """같은 개수의 닫힌 선 두 줄 a · b (N,3) 사이를 삼각형 띠로."""
    N = len(a)
    i = np.arange(N)
    j = (i + 1) % N
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.r_[a, b]),
                                  o3d.utility.Vector3iVector(np.r_[np.c_[i, j, N + j], np.c_[i, N + j, N + i]]))
    m.compute_vertex_normals()
    return m


def tree_mesh(x, y, z, diameter, height, rng):
    """줄기(원기둥) + 수관(울퉁불퉁한 타원구), 정점 색. (보이는 메쉬, 충돌용 줄기)"""
    trunk_h = 0.45 * height
    r_t = float(np.clip(0.04 * diameter, 0.1, 0.25))
    trunk = o3d.geometry.TriangleMesh.create_cylinder(r_t, trunk_h + 0.3, resolution=10, split=1)
    trunk.translate((x, y, z + trunk_h / 2 - 0.15))
    trunk.paint_uniform_color([0.30, 0.24, 0.18])
    crown = o3d.geometry.TriangleMesh.create_sphere(1.0, resolution=12)
    v = np.asarray(crown.vertices)
    bump = 1 + 0.12 * np.sin(3 * v[:, 0] + rng.uniform(0, 6)) * np.sin(4 * v[:, 1] + rng.uniform(0, 6)) + 0.06 * rng.standard_normal(len(v))
    v = v * bump[:, None] * [diameter / 2, diameter / 2, 0.32 * height]
    crown.vertices = o3d.utility.Vector3dVector(v + [x, y, z + 0.66 * height])
    g = rng.uniform(0.75, 1.1, len(v))[:, None] * np.array([0.22, 0.34, 0.16])
    crown.vertex_colors = o3d.utility.Vector3dVector(np.clip(g, 0, 1))
    crown.compute_vertex_normals()
    tree = trunk + crown
    tree.compute_vertex_normals()
    col = o3d.geometry.TriangleMesh.create_cylinder(r_t, trunk_h, resolution=8, split=1).translate((x, y, z + trunk_h / 2))
    return tree, col


def _car_rects(rows, xy, margin=0.3):
    keep = np.zeros(len(xy), bool)
    for r in rows:
        if r["type"] != "car":
            continue
        yaw = np.radians(float(r["yaw_deg"]))
        d = xy - [float(r["x"]), float(r["y"])]
        a, b = d[:, 0] * np.cos(yaw) + d[:, 1] * np.sin(yaw), -d[:, 0] * np.sin(yaw) + d[:, 1] * np.cos(yaw)
        keep |= (np.abs(a) < float(r["length_m"]) / 2 + margin) & (np.abs(b) < float(r["width_m"]) / 2 + margin)
    return keep


def _cut(mesh, keep_tri):
    out = o3d.geometry.TriangleMesh(mesh.vertices, o3d.utility.Vector3iVector(np.asarray(mesh.triangles)[keep_tri]))
    if mesh.has_vertex_colors():
        out.vertex_colors = mesh.vertex_colors
    if mesh.has_vertex_normals():
        out.vertex_normals = mesh.vertex_normals
    out.remove_unreferenced_vertices()
    return out


def run(cfg, root, visuals, colliders, ground):
    """visuals · colliders 를 고쳐 씀 (Floor · Visual 자르기, Curb · Sidewalk · Grass · Trees 더하기, 충돌에 연석 · 인도 · 줄기).
    → (재질 dict {부분: 텍스처 dict}, 통계)"""
    e = cfg["edge"]
    floor = visuals["Floor"] if "Floor" in visuals else visuals["Visual"]
    mask, lo, cell, fp = boundary_mask(floor, e.get("smooth_m", 1.0), pad=e.get("outer_m", 40.0) + 5)
    dist_out = ndimage.distance_transform_edt(~mask) * cell              # 경계 밖으로 몇 m
    dist_in = ndimage.distance_transform_edt(mask) * cell                # 경계 안으로 몇 m
    shape = np.array(mask.shape)
    rows = []
    if e.get("objects") and os.path.exists(os.path.join(root, e["objects"])):
        rows = list(csv.DictReader(open(os.path.join(root, e["objects"]), encoding="utf-8")))
    trees = [r for r in rows if r["type"] == "tree"]
    # 가운데 섬 = 바닥 범위 안의 구멍(화단) — margin_m 넓혀 둘레의 사진 안 닿은 계단 바닥까지 덮음 (구멍 둘레 3 m 는 99% 안 찍힘).
    # 섬 쪽에 주차된 차 자리(+ car_gap_m)는 화단에서 뺌 — 차가 화단에 묻히지 않게
    isl = e.get("island") or {}
    island = np.zeros(mask.shape, bool)
    if isl:
        hole = ndimage.binary_fill_holes(fp) & ~fp & mask
        lab, nlab = ndimage.label(hole)
        big = np.isin(lab, 1 + np.nonzero(ndimage.sum(hole, lab, range(1, nlab + 1)) * cell * cell >= isl.get("min_area_m2", 4.0))[0])
        grow = ndimage.binary_dilation(big, iterations=int(isl.get("margin_m", 1.0) / cell))
        ii, jj = np.nonzero(grow)
        cars_g = np.zeros(mask.shape, bool)
        cars_g[ii, jj] = _car_rects(rows, lo + (np.c_[ii, jj] + 0.5) * cell, margin=isl.get("car_gap_m", 0.5))
        island = ndimage.gaussian_filter((grow & ~cars_g).astype(np.float32), isl.get("smooth_m", 0.8) / cell) > 0.5
        lab, nlab = ndimage.label(island)                                 # 차를 빼고 떨어져 나온 작은 조각은 버림
        if nlab:
            island = np.isin(lab, 1 + np.nonzero(ndimage.sum(island, lab, range(1, nlab + 1)) * cell * cell >= isl.get("min_area_m2", 4.0))[0])
    dist_isl = ndimage.distance_transform_edt(island) * cell             # 섬 안으로 몇 m

    def look(a, xy):
        ij = np.clip(((xy - lo) / cell).astype(int), 0, shape - 1)
        return a[ij[:, 0], ij[:, 1]]

    def d_out(xy):
        return look(dist_out, xy)
    st = dict(boundary_area_m2=round(float(mask.sum()) * cell * cell, 1))

    # 2) 경계 밖 · 섬 안 정리
    tol = e.get("cut_tol_m", 0.1)                                      # 연석 밑으로 조금 들어가게 (틈이 안 보이게)
    for name in [k for k in ("Floor", "Visual") if k in visuals]:
        m = visuals[name]
        c = np.asarray(m.vertices)[np.asarray(m.triangles)].mean(1)
        keep = (d_out(c[:, :2]) <= tol) & (look(dist_isl, c[:, :2]) <= tol)
        if name == "Visual":
            cars = _car_rects(rows, c[:, :2])
            keep |= cars                                               # 경계 · 섬에 걸친 주차 차는 남김
            h = c[:, 2] - ground.height(c[:, :2])
            for r in trees:                                            # 스캔 나무 덩어리 → 모델로 바꿈 (차 자리는 남김)
                d = np.hypot(c[:, 0] - float(r["x"]), c[:, 1] - float(r["y"]))
                keep &= ~((d < float(r["length_m"]) / 2 * e.get("tree_cut_scale", 1.0)) & (h > 0.3) & ~cars)
            if e.get("tall_cut_m"):                                    # 경계 가까이 높은 조각 = 나무 · 덤불 잔해 (차는 2 m 아래)
                keep &= ~((h > e["tall_cut_m"]) & (look(dist_in, c[:, :2]) < e.get("tall_band_m", 4.0)) & ~cars)
        st[f"{name.lower()}_tris_removed"] = int((~keep).sum())
        visuals[name] = _cut(m, keep)

    # 2b) 경계 안쪽 바닥 구멍 메우기: 다듬은 경계 안으로 들어온 스캔 바닥 가장자리 구멍 (주차장 140조각 · 20 m²) —
    #     안 메우면 로봇 카메라에 바닥이 뚫려 보임. 높이는 바닥 지도, 색은 가까운 바닥 정점 색
    if "Floor" in visuals:
        from scipy.spatial import cKDTree
        fc = np.asarray(visuals["Floor"].vertices)[np.asarray(visuals["Floor"].triangles)].mean(1)[:, :2]
        covered = np.zeros(mask.shape, bool)                            # 실제 바닥 삼각형이 있는 칸 (fp 는 메워서 30 cm 틈이 빠짐)
        ij = np.clip(((fc - lo) / cell).astype(int), 0, shape - 1)
        covered[ij[:, 0], ij[:, 1]] = True
        gap = ndimage.binary_dilation(mask & ~covered & ~island) & mask & ~island   # 한 칸 넓혀 원래 바닥 밑으로 2 mm 겹침 (이음새 틈)
        patch = grid_mesh(gap, lo, cell, lambda xy: ground.height(xy) - 0.002)
        if len(patch.triangles):
            fv = visuals["Floor"]
            if fv.has_vertex_colors():
                _, j = cKDTree(np.asarray(fv.vertices)[:, :2]).query(np.asarray(patch.vertices)[:, :2])
                patch.vertex_colors = o3d.utility.Vector3dVector(np.asarray(fv.vertex_colors)[j])
            merged = fv + patch
            merged.compute_vertex_normals()
            visuals["Floor"] = merged
        st["floor_gap_filled_m2"] = round(float(gap.sum()) * cell * cell, 1)

    # 3) 연석 · 인도 · 잔디 (+ 섬: 연석 + 화단)
    h = e.get("curb_h_m", 0.15)
    top_w = e.get("curb_top_m", 0.15)
    curb = o3d.geometry.TriangleMesh()
    outer = outlines(mask, lo, cell)
    p, n = max(outer, key=lambda t: len(t[0]))                         # 바깥 경계 (n = 바깥쪽)
    curb += curb_mesh(p, -n, ground, h, top_w)                         # 차로는 안쪽
    outline_m = len(p) * 0.2
    island_mesh = o3d.geometry.TriangleMesh()
    if island.any():
        for q, nq in outlines(island, lo, cell):                       # nq = 섬 바깥쪽 = 차로 쪽
            curb += curb_mesh(q, nq, ground, h, top_w)
            outline_m += len(q) * 0.2
        island_mesh = grid_mesh(ndimage.binary_erosion(island, iterations=max(1, int((top_w - 0.03) / cell))), lo, cell,
                                lambda xy: ground.height(xy) + h - 0.003)
    curb.compute_vertex_normals()
    sw = e.get("sidewalk_m", 2.0)
    side_sel = (dist_out > top_w - 0.03) & (dist_out <= sw)          # 연석 윗면 밑으로 3 cm 겹쳐 시작
    sidewalk = grid_mesh(side_sel, lo, cell, lambda xy: ground.height(xy) + h - 0.003)
    gc = e.get("grass_cell_m", 0.5)
    k = int(round(gc / cell))
    gs = (dist_out[::k, ::k] > sw - gc) & (dist_out[::k, ::k] <= e.get("outer_m", 40.0))
    grass = grid_mesh(gs, lo, cell * k, lambda xy: ground.height(xy) + h - 0.006)
    for m_, col in ((curb, [0.62, 0.62, 0.6]), (sidewalk, [0.55, 0.55, 0.53]), (grass, [0.25, 0.33, 0.17]), (island_mesh, [0.25, 0.33, 0.17])):
        if len(m_.triangles):
            m_.paint_uniform_color(col)                                # 텍스처를 못 읽을 때 대신 보이는 색
    visuals.update(Curb=curb, Sidewalk=sidewalk, Grass=grass)
    if len(island_mesh.triangles):
        visuals["Island"] = island_mesh
    st["island_m2"] = round(float(island.sum()) * cell * cell, 1)

    # 4) 나무
    rng = np.random.default_rng(0)
    tvis, tcol = o3d.geometry.TriangleMesh(), o3d.geometry.TriangleMesh()
    for r in trees:
        x, y = float(r["x"]), float(r["y"])
        xy = np.array([[x, y]])
        z = float(ground.height(xy)[0]) + (h if d_out(xy)[0] > 0 or look(island, xy)[0] else 0)
        a, b = tree_mesh(x, y, z, float(r["length_m"]), float(r["height_m"]), rng)
        tvis += a
        tcol += b
    if len(tvis.triangles):
        tvis.compute_vertex_normals()
        visuals["Trees"] = tvis

    col = colliders["Collision"] + curb + sidewalk + island_mesh + tcol   # 연석 · 인도 · 섬 윗면 · 줄기 (잔디는 경계벽 밖이라 뺌)
    col.vertex_colors = o3d.utility.Vector3dVector()
    col.compute_vertex_normals()
    colliders["Collision"] = col
    st.update(outline_m=round(float(outline_m), 1), curb_h_m=h, sidewalk_m=sw, sidewalk_tris=len(sidewalk.triangles),
              grass_tris=len(grass.triangles), trees=len(trees), tree_tris=len(tvis.triangles))

    mats = {}
    for part, key in (("Curb", "curb"), ("Sidewalk", "sidewalk"), ("Grass", "grass"), ("Island", "grass")):
        mm = (e.get("materials") or {}).get(key)
        if not mm:
            continue
        t = dict(tile_m=mm.get("tile_m", 1.0), normal_strength=mm.get("normal_strength", 1.0))
        for k_ in ("color", "normal", "roughness"):
            path = os.path.expanduser(mm[k_]) if mm.get(k_) else None
            t[k_] = path if path and os.path.exists(path) else None
        if t["color"]:
            mats[part] = t
        else:
            print(f"      ! 테두리 재질 {key} 색 파일 없음 → 단색")
    return mats, st
