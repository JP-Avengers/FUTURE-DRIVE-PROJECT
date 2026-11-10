"""스캔된 주차 차 지우기 (v0.8). 차는 촬영 날마다 다르고 메쉬가 찌그러져 보여서 맵에서 뺀다 (11.06).

차 자리 = B outer/objects.csv 의 차 + config cars.extra (B 목록에 없는 덩어리를 메쉬 · 키프레임 사진으로 확인해 적은 것).
build_usd.py 가 색 투영 · 바닥 텍스처 다음에 부름 — 가림 판정에는 차가 있어야 차 색이 바닥에 안 찍힘.
차 자리의 바닥은 floor_texture fill_stalls 가 같은 목록으로 아스팔트로 채움.
"""
import csv

import numpy as np
from scipy.spatial import cKDTree
import open3d as o3d


def read(path, extra=None):
    """차 사각형 목록 [{x, y, yaw_deg, length_m, width_m}]. extra = [[x, y, yaw_deg, 길이, 폭], ...]."""
    rows = [r for r in csv.DictReader(open(path, encoding="utf-8")) if r["type"] == "car"] if path else []
    rows += [dict(x=e[0], y=e[1], yaw_deg=e[2], length_m=e[3], width_m=e[4]) for e in extra or []]
    return rows


def inside(rows, xy, margin):
    """점 xy (N,2) 가 차 사각형 + margin 안인가."""
    hit = np.zeros(len(xy), bool)
    for r in rows:
        yaw = np.radians(float(r["yaw_deg"]))
        d = xy - [float(r["x"]), float(r["y"])]
        a, b = d[:, 0] * np.cos(yaw) + d[:, 1] * np.sin(yaw), -d[:, 0] * np.sin(yaw) + d[:, 1] * np.cos(yaw)
        hit |= (np.abs(a) < float(r["length_m"]) / 2 + margin) & (np.abs(b) < float(r["width_m"]) / 2 + margin)
    return hit


def object_keep(objects, rows, margin, near_m, leftover_tris, ground=None, float_h_m=0.0):
    """물체 메쉬에서 남길 삼각형 bool. 차 사각형 안 + 그 둘레 near_m 안에 걸린 leftover_tris 미만 부스러기를 뺌.
    float_h_m 을 주면 가장 낮은 곳이 바닥 위 이보다 높은 조각(공중에 뜬 건물 처마 조각 등)도 뺌. → (keep, 부스러기 수, 뜬 조각 수)"""
    tri = np.asarray(objects.triangles)
    v = np.asarray(objects.vertices)
    c = v[tri].mean(1)[:, :2]
    keep = ~inside(rows, c, margin)
    rest = o3d.geometry.TriangleMesh(objects.vertices, o3d.utility.Vector3iVector(tri[keep]))
    ids, counts, _ = rest.cluster_connected_triangles()
    ids, counts = np.asarray(ids), np.asarray(counts)
    near = np.zeros(len(counts), bool)
    near[ids[inside(rows, c[keep], near_m)]] = True                     # 차 둘레에 닿은 조각
    drop = near[ids] & (counts[ids] < leftover_tris)
    n_float = 0
    if float_h_m and ground is not None:
        h = v[tri[keep]][:, :, 2].min(1) - ground.height(c[keep])
        low = np.full(len(counts), np.inf)
        np.minimum.at(low, ids, h)
        floating = (low > float_h_m)[ids]
        n_float = int((floating & ~drop).sum())
        drop |= floating
    idx = np.nonzero(keep)[0]
    keep[idx[drop]] = False
    return keep, int(drop.sum()) - n_float, n_float


def cut(mesh, keep):
    """keep 삼각형만 남긴 메쉬 (정점 색 · 법선 유지)."""
    out = o3d.geometry.TriangleMesh(mesh.vertices, o3d.utility.Vector3iVector(np.asarray(mesh.triangles)[keep]))
    if mesh.has_vertex_colors():
        out.vertex_colors = mesh.vertex_colors
    if mesh.has_vertex_normals():
        out.vertex_normals = mesh.vertex_normals
    out.remove_unreferenced_vertices()
    return out


PALETTE = [(0.85, 0.85, 0.83), (0.62, 0.63, 0.64), (0.35, 0.36, 0.38), (0.10, 0.10, 0.11), (0.18, 0.24, 0.36), (0.45, 0.12, 0.12)]


def _box(cx, cy, z0, L, W, H, yaw, color):
    b = o3d.geometry.TriangleMesh.create_box(L, W, H)
    b.translate((-L / 2, -W / 2, 0.0))
    c, s = np.cos(yaw), np.sin(yaw)
    b.rotate(np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), center=(0, 0, 0))
    b.translate((cx, cy, z0))
    t = np.asarray(b.triangles)                                         # 면마다 정점을 따로 (모서리가 둥글게 음영지지 않게)
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.asarray(b.vertices)[t].reshape(-1, 3)),
                                  o3d.utility.Vector3iVector(np.arange(len(t) * 3).reshape(-1, 3)))
    m.paint_uniform_color(color)
    return m


def _rect_dist(p, x, y, yaw, L, W):
    """점들 p (N,2) 에서 차 바닥 사각형까지 거리."""
    d = p - [x, y]
    a = np.abs(d @ [np.cos(yaw), np.sin(yaw)]) - L / 2
    b = np.abs(d @ [-np.sin(yaw), np.cos(yaw)]) - W / 2
    return np.hypot(np.maximum(a, 0), np.maximum(b, 0))


def _overlap(a, b, W):
    """차 바닥 사각형 둘 (x, y, yaw, L) 이 겹치는지 (분리축)."""
    def corners(x, y, yaw, L):
        u, v = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
        return np.array([[x, y] + su * u * L / 2 + sv * v * W / 2 for su in (-1, 1) for sv in (-1, 1)])
    A, Bc = corners(*a), corners(*b)
    for yaw in (a[2], b[2]):
        for n in (np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])):
            pa, pb = A @ n, Bc @ n
            if pa.max() <= pb.min() + 0.05 or pb.max() <= pa.min() + 0.05:
                return False
    return True


def slots(rows, extra, c, path_xy=None, scan_xy=None, block_xy=None):
    """차 모델 자리 [(x, y, yaw_rad, 길이, 높이)].
    path_xy(촬영 경로)가 있으면 path_clear_m 안에 들어오는 자리는 고침 — 로봇이 지나간 곳엔 차가 없음:
    덩어리 자리는 뺌, B 목록 차는 늘린 길이를 원래 길이로 되돌림.
    B 목록 차 = 그 자리 · 방향 (길이 · 높이는 실제 차 범위로 자름 — B 사각형은 스캔 덩어리라 짧거나 낮게 잡힌 것이 있음).
    extra 덩어리(B 목록에 없던 것) 중 넓이 min_extra_m2 이상 = 가장 가까운 B 차의 방향으로 주차칸 간격(slot_w_m)만큼 채움."""
    Lr, Hr = c.get("length_range_m", [4.2, 4.9]), c.get("height_range_m", [1.4, 1.8])
    out = []
    for r in rows:
        out.append((float(r["x"]), float(r["y"]), np.radians(float(r["yaw_deg"])), float(np.clip(float(r["length_m"]), *Lr)),
                    float(np.clip(float(r.get("height_m") or 1.5), *Hr))))
    if not rows:
        return out
    bxy = np.array([[float(r["x"]), float(r["y"])] for r in rows])
    for e in extra or []:
        x, y, yaw_d, L, W = map(float, e)
        if L * W < c.get("min_extra_m2", 6.0):
            continue
        j = int(np.argmin(np.hypot(*(bxy - [x, y]).T)))
        yaw = np.radians(float(rows[j]["yaw_deg"]))                    # 이웃 주차 차와 같은 방향
        ea = np.radians(yaw_d)
        u, v = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])   # 차 길이 · 옆 방향
        sw, sl = c.get("slot_w_m", 2.5), c.get("slot_l_m", 5.0)
        R = max(L, W)
        found = []
        for a in np.arange(-R, R + 1e-6, sl):
            for b in np.arange(-R, R + 1e-6, sw):
                p = np.array([x, y]) + a * u + b * v
                d = p - [x, y]
                pa, pb = d @ [np.cos(ea), np.sin(ea)], d @ [-np.sin(ea), np.cos(ea)]
                if abs(pa) <= L / 2 + 0.3 and abs(pb) <= W / 2 + 0.3:   # 덩어리 사각형 안
                    found.append(p)
        if not found:
            found = [np.array([x, y])]
        for p in found:
            if np.hypot(*(np.array([[o[0], o[1]] for o in out]) - p).T).min() < 2.0:   # 이미 있는 차와 겹치면 건너뜀
                continue
            out.append((float(p[0]), float(p[1]), yaw, float(np.mean(Lr)), float(np.mean(Hr))))
    if path_xy is None:
        return out
    W, P = c.get("width_m", 1.8), np.asarray(path_xy)[:, :2]
    tree = cKDTree(P)
    fit = shortened = 0
    lane_face = {}
    for i, (x, y, yaw, L, H) in enumerate(out):
        # 통로 쪽 면 맞추기: 스캔은 카메라가 본 쪽(통로 쪽 끝 2 m 안팎)만 있어서 B 사각형 · 늘린 길이가 통로로 나올 수 있음
        # → 통로 쪽 면을 지운 차 스캔의 통로 쪽 끝(99%)에 맞추고 차는 반대쪽으로. 스캔이 없으면 B 차는 통로 쪽 끝을 두고 안쪽으로만 늘림.
        # 맞출 축: 길(경로 주성분 방향)과 나란하지 않은 차 = 길이 축, 나란한 차(평행 주차) = 옆 축
        u, v = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
        j = tree.query([x, y])[1]
        near = P[tree.query_ball_point(P[j], 3.0)]
        tan = np.linalg.svd(near - near.mean(0), full_matrices=False)[2][0] if len(near) > 2 else u
        ax, half = (u, L / 2) if abs(u @ tan) < 0.7 else (v, W / 2)
        sgn = 1.0 if (P[j] - [x, y]) @ ax >= 0 else -1.0
        lane = half                                                      # 통로 쪽 면 (중심에서, sgn 방향)
        if scan_xy is not None and len(scan_xy):
            d = scan_xy - [x, y]
            m = (np.abs(d @ u) < L / 2 + 1.0) & (np.abs(d @ v) < W / 2 + 0.2)
            if m.sum() >= c.get("fit_min_pts", 50):
                lane = min(half, float(np.percentile(sgn * (d[m] @ ax), 99)))
                fit += 1
        elif i < len(rows) and ax is u:
            lane = min(half, float(rows[i]["length_m"]) / 2)
        if lane < half:
            x, y = np.array([x, y]) + sgn * ax * (lane - half)
            out[i] = (float(x), float(y), yaw, L, H)
        if ax is u:
            lane_face[i] = np.array([x, y]) + sgn * u * L / 2                # 길이 축을 맞춘 차: 통로 쪽 면 중심 (줄일 때 고정)
    Lmin = c.get("min_length_m", 3.5)
    bt = cKDTree(np.asarray(block_xy)[:, :2]) if block_xy is not None and len(block_xy) else None

    def hits(k, x, y, yaw, L):
        """다른 차 · 막는 점(경계벽 · 남은 스캔 물체) 과 겹치는지."""
        if any(_overlap((x, y, yaw, L), o[:4], W) for n, o in enumerate(out) if n != k and o is not None):
            return True
        if bt is None:
            return False
        idx = bt.query_ball_point([x, y], np.hypot(L, W) / 2)
        if not idx:
            return False
        d = bt.data[idx] - [x, y]
        u, v = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
        return int(((np.abs(d @ u) < L / 2) & (np.abs(d @ v) < W / 2)).sum()) >= c.get("block_min_pts", 3)

    dropped_blocked = 0
    for _ in range(3):                      # 겹치면 뒤에서 줄임 (통로 쪽 면 고정) — 최소 길이로도 겹치면 덩어리 자리는 뺌
        for i in range(len(out)):
            if out[i] is None or not hits(i, *out[i][:4]):
                continue
            x, y, yaw, L, H = out[i]
            if i in lane_face and L > Lmin + 1e-6:
                f, u = lane_face[i], np.array([np.cos(yaw), np.sin(yaw)])
                back = -np.sign((f - [x, y]) @ u)
                cx, cy, Ln = x, y, L
                for Ln in np.arange(L - 0.1, Lmin - 1e-6, -0.1):
                    cx, cy = f + back * u * Ln / 2
                    if not hits(i, cx, cy, yaw, Ln):
                        break
                out[i] = (float(cx), float(cy), yaw, float(Ln), H)
                shortened += 1
                if not hits(i, *out[i][:4]):
                    continue
            if i >= len(rows):
                out[i] = None
                dropped_blocked += 1
    out = [o for o in out if o is not None] if dropped_blocked else out
    nb = len(rows)                          # B 차는 빼지 않으므로 앞쪽 len(rows) 개가 그대로 B
    clear = c.get("path_clear_m")
    if not clear:
        slots.last = dict(scan_fit=fit, shortened=shortened, extra_blocked=dropped_blocked)
        return out
    kept, fixed, dropped = [], 0, 0
    for i, (x, y, yaw, L, H) in enumerate(out):
        if _rect_dist(P, x, y, yaw, L, W).min() >= clear:
            kept.append((x, y, yaw, L, H))
        elif i < nb:                        # B 차는 실제 차 — 빼지 않고 원래 사각형으로
            r = rows[i]
            kept.append((float(r["x"]), float(r["y"]), yaw, min(L, float(r["length_m"])), H))
            fixed += 1
        else:
            dropped += 1
    slots.last = dict(scan_fit=fit, shortened=shortened, extra_blocked=dropped_blocked, path_clear_m=clear, b_length_restored=fixed, extra_dropped=dropped)
    return kept


def models(rows, extra, ground, c, path_xy=None, scan_xy=None, block_xy=None):
    """단순 차 모델 (몸체 + 캐빈 + 바퀴 상자, 정점 색) → (보이는 메쉬, 충돌 메쉬 = 바닥부터 지붕까지 상자 하나씩, 대수).
    11.10: 차를 지운 v0.8 에서 라이다가 볼 물체가 줄어 AMCL 실패 → 같은 자리에 단순 모델 (D 요청: Visual · Collision 양쪽)."""
    rng = np.random.default_rng(c.get("seed", 0))
    W = c.get("width_m", 1.8)
    vis, col = o3d.geometry.TriangleMesh(), o3d.geometry.TriangleMesh()
    slots.last = {}
    sl = slots(rows, extra, c, path_xy, scan_xy, block_xy)
    models.slots = [[round(x, 3), round(y, 3), round(float(np.degrees(yaw)), 2), round(L, 3), round(H, 3)] for x, y, yaw, L, H in sl]
    for x, y, yaw, L, H in sl:
        z = float(ground.height(np.array([[x, y]]))[0]) if ground is not None else 0.0
        body = PALETTE[rng.integers(len(PALETTE))]
        clr = c.get("clearance_m", 0.18)
        belt = clr + 0.55 * (H - clr)                                   # 몸체 위 = 창 아래
        vis += _box(x, y, z + clr, L, W, belt - clr, yaw, body)
        cab_l = 0.55 * L
        off = -0.05 * L                                                 # 캐빈은 조금 뒤로
        vis += _box(x + off * np.cos(yaw), y + off * np.sin(yaw), z + belt, cab_l, W - 0.12, H - belt, yaw, (0.12, 0.13, 0.15))
        for sa in (-1, 1):                                              # 바퀴 4개 (차 밑이 비어 보이지 않게)
            for sb in (-1, 1):
                a, b = sa * 0.34 * L, sb * (W / 2 - 0.12)
                vis += _box(x + a * np.cos(yaw) - b * np.sin(yaw), y + a * np.sin(yaw) + b * np.cos(yaw), z, 0.62, 0.22, clr + 0.25, yaw, (0.05, 0.05, 0.05))
        col += _box(x, y, z, L, W, H, yaw, (0.5, 0.5, 0.5))
    vis.compute_vertex_normals()
    col.compute_vertex_normals()
    return vis, col, len(sl)
