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


def run(cfg, root, visuals, colliders, ground, T=None):
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
    # 가운데 섬 = 바닥 범위 안의 구멍 — margin_m 넓혀 둘레의 사진 안 닿은 계단 바닥까지 (구멍 둘레 3 m 는 99% 안 찍힘).
    #   mode floor (v0.7 r2~): 주변 도로와 같은 아스팔트 바닥으로 메움 — 구멍에 바닥을 깔고 섬 안 스캔 조각을 지움.
    #     아스팔트 색은 floor_texture fill_stalls.holes 가 같은 자리를 채움 (11.01: 연석 + 화단은 어색함)
    #   mode curb: 연석 + 화단, 섬 쪽에 주차된 차 자리(+ car_gap_m)는 화단에서 뺌
    isl = e.get("island") or {}
    island_floor = isl.get("mode", "curb") == "floor"
    island = np.zeros(mask.shape, bool)
    if isl:
        hole = ndimage.binary_fill_holes(fp) & ~fp & mask
        if isl.get("conf_map"):   # 보조 촬영이 섬 바닥을 채워 구멍이 안 남는 맵 (v0.9, B v0.5g): 주 촬영이 바닥을 못 본 안쪽 구멍 = 섬
            from mesh_utils import conf_holes, resample_mask
            ch, clo, ccell = conf_holes(os.path.join(root, isl["conf_map"]), isl.get("measured_conf", (1, 2, 3)), isl.get("min_area_m2", 4.0))
            hole |= resample_mask(ch, clo, ccell, lo[0] + (np.arange(mask.shape[0]) + 0.5) * cell, lo[1] + (np.arange(mask.shape[1]) + 0.5) * cell) & mask
        lab, nlab = ndimage.label(hole)
        big = np.isin(lab, 1 + np.nonzero(ndimage.sum(hole, lab, range(1, nlab + 1)) * cell * cell >= isl.get("min_area_m2", 4.0))[0])
        grow = ndimage.binary_dilation(big, iterations=int(isl.get("margin_m", 1.0) / cell))
        ii, jj = np.nonzero(grow)
        cars_g = np.zeros(mask.shape, bool)
        if not island_floor:
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
        keep = d_out(c[:, :2]) <= tol
        if name == "Visual" or not island_floor:                        # 섬을 바닥으로 메울 땐 바닥은 그대로 두고 스캔 조각만 지움
            keep &= look(dist_isl, c[:, :2]) <= tol
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

    # 2a) 충돌 메쉬도 같은 기준으로 (v0.8 r3, 11.10 — D: 경계 밖 스캔 조각이 Nav2 지도 테두리에 두꺼운 띠, 라이다가 보는 형상과 다름).
    #     바닥 높이 + keep_below_m 아래(충돌 바닥 · 연석 높이)는 그대로 — 로봇이 디딜 바닥은 줄이지 않음
    cc = e.get("cut_collision")
    if cc and "Collision" in colliders:
        m = colliders["Collision"]
        c = np.asarray(m.vertices)[np.asarray(m.triangles)].mean(1)
        h = c[:, 2] - ground.height(c[:, :2])
        keep = (d_out(c[:, :2]) <= tol) & (look(dist_isl, c[:, :2]) <= tol)
        for r in trees:
            d = np.hypot(c[:, 0] - float(r["x"]), c[:, 1] - float(r["y"]))
            keep &= ~((d < float(r["length_m"]) / 2 * e.get("tree_cut_scale", 1.0)) & (h > 0.3))
        if e.get("tall_cut_m"):
            keep &= ~((h > e["tall_cut_m"]) & (look(dist_in, c[:, :2]) < e.get("tall_band_m", 4.0)))
        keep |= h < (cc.get("keep_below_m", 0.1) if isinstance(cc, dict) else 0.1)
        st["collision_tris_removed"] = int((~keep).sum())
        colliders["Collision"] = _cut(m, keep)

    # 2b) 경계 안쪽 바닥 구멍 메우기: 다듬은 경계 안으로 들어온 스캔 바닥 가장자리 구멍 (주차장 140조각 · 20 m²) —
    #     안 메우면 로봇 카메라에 바닥이 뚫려 보임. 높이는 바닥 지도, 색은 가까운 바닥 정점 색
    if "Floor" in visuals:
        from scipy.spatial import cKDTree
        fc = np.asarray(visuals["Floor"].vertices)[np.asarray(visuals["Floor"].triangles)].mean(1)[:, :2]
        covered = np.zeros(mask.shape, bool)                            # 실제 바닥 삼각형이 있는 칸 (fp 는 메워서 30 cm 틈이 빠짐)
        ij = np.clip(((fc - lo) / cell).astype(int), 0, shape - 1)
        covered[ij[:, 0], ij[:, 1]] = True
        skip = np.zeros(mask.shape, bool) if island_floor else island    # 섬을 바닥으로 메우면 섬 구멍도 여기서 깖
        gap = ndimage.binary_dilation(mask & ~covered & ~skip) & mask & ~skip   # 한 칸 넓혀 원래 바닥 밑으로 2 mm 겹침 (이음새 틈)
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
    if island.any() and not island_floor:
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
    if not e.get("outer_ground", True):                                # 11.06: 초록 바깥 땅은 어색해서 끔 — 인도 너머는 비워 둠
        gs = np.zeros_like(gs)
    grass = grid_mesh(gs, lo, cell * k, lambda xy: ground.height(xy) + h - 0.006)
    for m_, col in ((curb, [0.62, 0.62, 0.6]), (sidewalk, [0.55, 0.55, 0.53]), (grass, [0.25, 0.33, 0.17]), (island_mesh, [0.25, 0.33, 0.17])):
        if len(m_.triangles):
            m_.paint_uniform_color(col)                                # 텍스처를 못 읽을 때 대신 보이는 색
    # 3a) 둘레 스캔 (v0.9, 11.11 — B v0.5g 손 촬영 둘레 실험층): 연석 밖의 '바닥 수준' 면만 보이는 메쉬로 (Perimeter prim, 충돌 없음).
    #     남기는 것 = 위를 향한 면(min_nz) 중 바닥 지도 높이 −below_m ~ +max_h_m. 솟은 것(화단 턱 · 벽 · 차 · 사람)과 깨진 가장자리(open_m 로 깎음) · 작은 조각은 버림.
    #     max_h_m 은 라이다 높이 띠(바닥 위 약 0.23 m~) 아래로 — 지도(Collision)에 없는 형상을 라이다가 보면 AMCL 이 틀어짐.
    #     위치는 지역마다 0.2~3 m 어긋나 있음 (B README) — 배경용. 둘레 스캔이 덮는 자리의 만든 인도는 보이는 메쉬에서 뺌 (충돌은 그대로)
    sidewalk_vis = sidewalk
    pm = e.get("perimeter")
    if pm and os.path.exists(os.path.join(root, pm["mesh"])):
        pmesh = o3d.io.read_triangle_mesh(os.path.join(root, pm["mesh"]))
        pv = np.asarray(pmesh.vertices).copy()
        if T is not None:
            pv = pv @ np.asarray(T)[:3, :3].T + np.asarray(T)[:3, 3]
        pmesh.vertices = o3d.utility.Vector3dVector(pv)
        pmesh.compute_triangle_normals()
        ptri = np.asarray(pmesh.triangles)
        pc = pv[ptri].mean(1)
        ph = pc[:, 2] - ground.height(pc[:, :2])
        keep = (np.asarray(pmesh.triangle_normals)[:, 2] > pm.get("min_nz", 0.75)) & (ph > -pm.get("below_m", 0.3)) & (ph < pm.get("max_h_m", 0.22)) \
            & (d_out(pc[:, :2]) > top_w)
        cov = np.zeros(mask.shape, bool)
        ij = np.clip(((pc[keep, :2] - lo) / cell).astype(int), 0, shape - 1)
        cov[ij[:, 0], ij[:, 1]] = True
        cov = ndimage.binary_opening(ndimage.binary_closing(cov, iterations=2), iterations=max(1, int(pm.get("open_m", 0.3) / cell)))
        lab, nlab = ndimage.label(cov)
        if nlab:
            cov = np.isin(lab, 1 + np.nonzero(ndimage.sum(cov, lab, range(1, nlab + 1)) * cell * cell >= pm.get("min_area_m2", 2.0))[0])
        keep &= look(cov, pc[:, :2])
        if pmesh.has_vertex_colors() and pm.get("blown_lum", 0.8):   # 하얗게 날아간 덩어리 (노출 과다 · 하늘 비침) 는 버림 — 가는 흰 선(주차선)은 남김
            plum = np.asarray(pmesh.vertex_colors)[ptri].mean(1).mean(1)
            br = np.zeros(mask.shape, bool)
            bj = np.clip(((pc[keep & (plum > pm.get("blown_lum", 0.8)), :2] - lo) / cell).astype(int), 0, shape - 1)
            br[bj[:, 0], bj[:, 1]] = True
            br = ndimage.binary_dilation(ndimage.binary_opening(br, iterations=max(1, int(pm.get("blown_min_w_m", 0.4) / cell / 2))), iterations=2)
            st["perimeter_blown_m2"] = round(float(br.sum()) * cell * cell, 1)
            keep &= ~look(br, pc[:, :2])
        perim = _cut(pmesh, keep)
        if len(perim.triangles):
            if perim.has_vertex_colors() and floor.has_vertex_colors() and pm.get("match_floor", True):   # 다른 날 · 다른 카메라 → 전체 밝기만 바닥에 맞춤
                pcol = np.asarray(perim.vertex_colors)
                fl_ = float(np.median(np.asarray(floor.vertex_colors)[::20].mean(1)))
                gain = float(np.clip(fl_ / max(float(np.median(pcol.mean(1))), 1e-3), 0.6, 1.6))
                perim.vertex_colors = o3d.utility.Vector3dVector(np.clip(pcol * gain, 0, 1))
                st["perimeter_gain"] = round(gain, 2)
            perim.compute_vertex_normals()
            visuals["Perimeter"] = perim
            sidewalk_vis = grid_mesh(side_sel & ~ndimage.binary_dilation(cov, iterations=2), lo, cell, lambda xy: ground.height(xy) + h - 0.003)
            sidewalk_vis.paint_uniform_color([0.55, 0.55, 0.53])
        st.update(perimeter_tris=len(perim.triangles), perimeter_m2=round(float(cov.sum()) * cell * cell, 1),
                  perimeter_src_tris=len(ptri), perimeter_max_h_m=pm.get("max_h_m", 0.22))
    run.curb = curb                    # 숨겨도 차 모델 자리 정할 때는 연석을 피함 (build_usd)
    if e.get("show_curb", True):       # false = 연석 · 인도를 보이는 메쉬에서 뺌 (v0.9, 11.11 — 만든 테두리가 인위적). 충돌에는 그대로 둠 (Nav2 지도 · 경계는 r3 와 같게)
        visuals["Curb"] = curb
    if e.get("show_sidewalk", True):
        visuals["Sidewalk"] = sidewalk_vis
    if len(grass.triangles):
        visuals["Grass"] = grass
    if len(island_mesh.triangles):
        visuals["Island"] = island_mesh
    st["island_m2"] = round(float(island.sum()) * cell * cell, 1)

    # 4) 나무
    rng = np.random.default_rng(0)
    tvis, tcol = o3d.geometry.TriangleMesh(), o3d.geometry.TriangleMesh()
    for r in (trees if e.get("tree_models", True) else []):          # 11.01: 모델이 너무 인위적 → 끔 (스캔 나무 덩어리는 위에서 지움)
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
              grass_tris=len(grass.triangles), trees=len(trees) if e.get("tree_models", True) else 0, tree_tris=len(tvis.triangles),
              island_mode=isl.get("mode", "curb") if isl else None)

    mats = {}
    for part, key in (("Curb", "curb"), ("Sidewalk", "sidewalk"), ("Grass", "grass"), ("Island", "grass")):
        mm = (e.get("materials") or {}).get(key)
        if not mm or part not in visuals:
            continue
        t = dict(tile_m=mm.get("tile_m", 1.0), normal_strength=mm.get("normal_strength", 1.0), roughness_range=mm.get("roughness_range"))
        for k_ in ("color", "normal", "roughness"):
            path = os.path.expanduser(mm[k_]) if mm.get(k_) else None
            t[k_] = path if path and os.path.exists(path) else None
        if t["color"]:
            mats[part] = t
        else:
            print(f"      ! 테두리 재질 {key} 색 파일 없음 → 단색")
    return mats, st
