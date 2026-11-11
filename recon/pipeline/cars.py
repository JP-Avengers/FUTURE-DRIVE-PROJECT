"""스캔된 주차 차 지우기 (v0.8). 차는 촬영 날마다 다르고 메쉬가 찌그러져 보여서 맵에서 뺀다 (11.06).

차 자리 = B outer/objects.csv 의 차 + config cars.extra (B 목록에 없는 덩어리를 메쉬 · 키프레임 사진으로 확인해 적은 것).
build_usd.py 가 색 투영 · 바닥 텍스처 다음에 부름 — 가림 판정에는 차가 있어야 차 색이 바닥에 안 찍힘.
차 자리의 바닥은 floor_texture fill_stalls 가 같은 목록으로 아스팔트로 채움.
"""
import csv

import json
import os

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


MANUAL_COLORS = {"white": (0.85, 0.85, 0.83), "silver": (0.62, 0.63, 0.64), "gray": (0.35, 0.36, 0.38), "black": (0.10, 0.10, 0.11),
                 "blue": (0.18, 0.24, 0.36), "red": (0.45, 0.12, 0.12)}   # car_editor.py 의 색 이름
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


def arrange(sl, stalls, c, path_xy=None, block_xy=None):
    """차 자리를 가지런히 (v0.9, 11.11 — 스캔 덩어리에서 나온 자리 · 방향이 들쭉날쭉). 보완(만든 것): 어느 칸에 차가 있었는지는 측정, 칸 안 자리는 만든 것.
    1) 주차칸(stalls.py 가 바닥 텍스처에서 찾은 사각형 [cx, cy, yaw, 폭, 길이, 줄, 기준선 쪽 x, y])에 든 차 = 칸 가운데 줄 · 칸 방향, 통로 쪽 끝에서 setback_m 들여 놓음
    2) 칸을 못 찾은 차 = 이웃끼리(중심 link_m 안 · 방향 link_deg 안) 묶어 같은 방향 · 같은 간격(pitch_m 이상) · 통로 쪽 면 한 줄로
    3) 옮긴 뒤 다른 차 · 막는 점(연석 · 경계벽 · 남은 물체) · 촬영 경로(path_clear_m)와 겹치면 그 차는 원래 자리로
    → (자리 목록, 통계)"""
    W = c.get("width_m", 1.8)
    a = c.get("arrange") if isinstance(c.get("arrange"), dict) else {}
    P = np.asarray(path_xy)[:, :2] if path_xy is not None else None
    tree = cKDTree(P) if P is not None else None
    out = [tuple(o) for o in sl]
    how = [""] * len(out)
    taken = set()
    S = np.asarray(stalls, float).reshape(-1, 8) if stalls is not None and len(stalls) else np.zeros((0, 8))
    pairs = []                                                                    # 1) 주차칸에 맞춤: 차 중심이 칸 안(+ stall_reach_m)인 쌍을 가까운 것부터
    for i, (x, y, yaw, L, H) in enumerate(out):                                   #    방향은 안 봄 — B 가 덩어리를 나눈 차는 방향이 칸과 어긋난 것이 있음 (칸이 더 믿을 만함)
        for j, (cx, cy, sy, sw, sl_, _, mx, my) in enumerate(S):
            u, v = np.array([np.cos(sy), np.sin(sy)]), np.array([-np.sin(sy), np.cos(sy)])
            d = np.array([x - cx, y - cy])
            if abs(d @ u) < sl_ / 2 + a.get("stall_reach_m", 1.2) and abs(d @ v) < sw / 2 + a.get("stall_reach_m", 1.2):
                pairs.append((float(np.hypot(*d)), i, j))
    match = {}
    for _, i, j in sorted(pairs):
        if i not in match and j not in taken:
            match[i] = j
            taken.add(j)
    for i, j in match.items():
        x, y, yaw, L, H = out[i]
        cx, cy, sy, sw, sl_, _, mx, my = S[j]
        u = np.array([np.cos(sy), np.sin(sy)])
        e0, e1 = np.array([mx, my]), np.array([mx, my]) + sl_ * u               # 칸의 두 끝 (기준선 쪽 · 반대쪽)
        lane, back = (e0, e1) if tree is None or tree.query(e0)[0] <= tree.query(e1)[0] else (e1, e0)
        inward = (back - lane) / max(np.linalg.norm(back - lane), 1e-9)
        sb = a.get("setback_m", 0.35)
        if sl_ >= 4.0:
            Ln = max(min(L, sl_ - 2 * sb), c.get("min_length_m", 3.5))
            cen = lane + inward * (sb + Ln / 2)
        else:                                                                     # 칸막이가 짧게만 보인 줄: 칸 가운데
            Ln, cen = L, np.array([cx, cy])
        ny = sy if np.cos(yaw - sy) >= 0 else sy + np.pi                          # 차 머리 방향은 그대로
        out[i] = (float(cen[0]), float(cen[1]), float(ny), float(Ln), H)
        how[i] = "stall"
    free = [i for i in range(len(out)) if not how[i]]                              # 2) 칸 없는 차: 이웃 묶음
    seen = set()
    for i in free:
        if i in seen:
            continue
        grp, todo = [], [i]
        while todo:
            q = todo.pop()
            if q in seen:
                continue
            seen.add(q)
            grp.append(q)
            for r in free:
                if r in seen:
                    continue
                dy = abs((out[q][2] - out[r][2] + np.pi / 2) % np.pi - np.pi / 2)
                if np.hypot(out[q][0] - out[r][0], out[q][1] - out[r][1]) < a.get("link_m", 3.6) and dy < np.radians(a.get("link_deg", 30)):
                    todo.append(r)
        if len(grp) < 2:
            continue
        ang = np.array([out[q][2] for q in grp])
        ax = 0.5 * np.arctan2(np.sin(2 * ang).mean(), np.cos(2 * ang).mean())      # 축 평균 (180° 뒤집힌 차도 같은 축)
        u, v = np.array([np.cos(ax), np.sin(ax)]), np.array([-np.sin(ax), np.cos(ax)])
        xy = np.array([[out[q][0], out[q][1]] for q in grp])
        m = xy.mean(0)
        lat, lon = (xy - m) @ v, (xy - m) @ u
        if np.ptp(lat) < np.ptp(lon):                                             # 앞뒤로 늘어선 묶음(평행 주차 · 두 줄)은 방향만 맞춤
            for q in grp:
                ny = ax if np.cos(out[q][2] - ax) >= 0 else ax + np.pi
                out[q] = (out[q][0], out[q][1], float(ny), out[q][3], out[q][4])
                how[q] = "group"
            continue
        o = np.argsort(lat)
        pitch = max(a.get("pitch_m", 2.5), float(np.ptp(lat)) / (len(grp) - 1))
        new_lat = (np.arange(len(grp)) - (len(grp) - 1) / 2) * pitch + lat.mean()
        sgn = 1.0
        if tree is not None:
            jn = tree.query(m)[1]
            sgn = 1.0 if (P[jn] - m) @ u >= 0 else -1.0
        Ls = np.array([out[q][3] for q in grp])
        face = float(np.median(sgn * lon + Ls / 2))                                # 통로 쪽 면을 한 줄로
        for rank, gi in enumerate(o):
            q = grp[gi]
            cen = m + new_lat[rank] * v + sgn * (face - Ls[gi] / 2) * u
            ny = ax if np.cos(out[q][2] - ax) >= 0 else ax + np.pi
            out[q] = (float(cen[0]), float(cen[1]), float(ny), out[q][3], out[q][4])
            how[q] = "group"
    # 2a) 남은 차: 가까운 긴 주차선(line_min_m 이상, line_reach_m 안)이 있으면 그 선에 직각으로, 선에서 setback_m 띄워 (칸막이가 안 찍힌 줄)
    LN = [np.asarray(q, float) for q in (c.get("_lines") or []) if np.hypot(q[2] - q[0], q[3] - q[1]) >= a.get("line_min_m", 6.0)]
    on_line = {}
    for i in [q for q in range(len(out)) if not how[q]]:
        x, y, yaw, L, H = out[i]
        best = None
        for li, q in enumerate(LN):
            d = (q[2:] - q[:2]) / np.linalg.norm(q[2:] - q[:2])
            n = np.array([-d[1], d[0]])
            t, r = (np.array([x, y]) - q[:2]) @ d, (np.array([x, y]) - q[:2]) @ n
            if -a.get("pitch_m", 2.5) < t < np.linalg.norm(q[2:] - q[:2]) + a.get("pitch_m", 2.5) and abs(r) < a.get("line_reach_m", 4.0):
                if best is None or abs(r) < abs(best[0]):
                    best = (r, t, li, d, n)
        if best is None:
            continue
        r, t, li, d, n = best
        sd = n * (1.0 if r >= 0 else -1.0)
        cen = LN[li][:2] + t * d + sd * (a.get("setback_m", 0.35) + L / 2)
        ax = float(np.arctan2(sd[1], sd[0]))
        out[i] = (float(cen[0]), float(cen[1]), ax if np.cos(yaw - ax) >= 0 else ax + np.pi, L, H)
        how[i] = "line"
        on_line.setdefault((li, r >= 0), []).append((t, i, d))
    for grp in on_line.values():                                                  # 같은 선 위의 차끼리 간격 pitch_m 이상
        grp.sort()
        ts = np.array([g_[0] for g_ in grp])
        for q in range(1, len(ts)):
            ts[q] = max(ts[q], ts[q - 1] + a.get("pitch_m", 2.5))
        ts += np.mean([g_[0] for g_ in grp]) - ts.mean()
        for (t0_, i, d), t1_ in zip(grp, ts):
            out[i] = (float(out[i][0] + (t1_ - t0_) * d[0]), float(out[i][1] + (t1_ - t0_) * d[1]), *out[i][2:])
    # 2b) 그래도 남은 차: far_stall_m 안의 빈 주차칸으로 (B 가 한 덩어리를 옆으로 나눠 놓은 차 — 실제로는 칸마다 한 대씩)
    for i in [q for q in range(len(out)) if not how[q]]:
        x, y, yaw, L, H = out[i]
        cand = [(float(np.hypot(x - S[j][0], y - S[j][1])), j) for j in range(len(S)) if j not in taken]
        cand = [q for q in cand if q[0] < a.get("far_stall_m", 6.5)]
        if not cand:
            continue
        j = min(cand)[1]
        taken.add(j)
        cx, cy, sy, sw, sl_, _, mx, my = S[j]
        u = np.array([np.cos(sy), np.sin(sy)])
        e0, e1 = np.array([mx, my]), np.array([mx, my]) + sl_ * u
        lane, back = (e0, e1) if tree is None or tree.query(e0)[0] <= tree.query(e1)[0] else (e1, e0)
        inward = (back - lane) / max(np.linalg.norm(back - lane), 1e-9)
        sb = a.get("setback_m", 0.35)
        Ln = max(min(L, sl_ - 2 * sb), c.get("min_length_m", 3.5)) if sl_ >= 4.0 else L
        cen = lane + inward * (sb + Ln / 2) if sl_ >= 4.0 else np.array([cx, cy])
        out[i] = (float(cen[0]), float(cen[1]), float(sy if np.cos(yaw - sy) >= 0 else sy + np.pi), float(Ln), H)
        how[i] = "stall"
    bt = cKDTree(np.asarray(block_xy)[:, :2]) if block_xy is not None and len(block_xy) else None

    def bad(k, cur=None):                                                         # 3) 겹침 검사
        cur = out if cur is None else cur
        x, y, yaw, L, H = cur[k]
        hit = [n for n, o in enumerate(cur) if n != k and _overlap((x, y, yaw, L), o[:4], W)]
        if hit:
            return "car %s" % hit
        if P is not None and c.get("path_clear_m") and _rect_dist(P, x, y, yaw, L, W).min() < c["path_clear_m"]:
            return "path"
        if bt is not None:
            idx = bt.query_ball_point([x, y], np.hypot(L, W) / 2)
            if idx:
                d = bt.data[idx] - [x, y]
                u, v = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
                if int(((np.abs(d @ u) < L / 2) & (np.abs(d @ v) < W / 2)).sum()) >= c.get("block_min_pts", 3):
                    return "block"
        return ""
    orig = [tuple(o) for o in sl]
    bad0 = [bad(k, orig) for k in range(len(orig))]                                # 원래 자리부터 겹쳤던 차 (경계벽에 걸친 북쪽 줄 등) 는 되돌려도 소용없음
    reverted = fixed = 0
    arrange.log = []
    Lmin = c.get("min_length_m", 3.5)
    for _ in range(3):
        for k in sorted(range(len(out)), key=lambda q: {"line": 0, "group": 1}.get(how[q], 2)):   # 덜 확실한 자리(선 · 묶음)부터 고침, 주차칸에 든 차는 마지막
            why = bad(k) if how[k] and not bad0[k] else ""
            if not why or why == "block":      # 연석 · 경계벽에 조금 걸치는 것은 둠 (원래 자리도 최소 길이로 줄인 뒤엔 그대로 둠)
                continue
            x, y, yaw, L, H = out[k]
            u = np.array([np.cos(yaw), np.sin(yaw)])
            ok = False
            if L > Lmin + 1e-6:                # (가) 통로 쪽 면은 두고 뒤에서 줄임
                sg = 1.0
                if tree is not None:
                    sg = 1.0 if (P[tree.query([x, y])[1]] - [x, y]) @ u >= 0 else -1.0
                f = np.array([x, y]) + sg * u * L / 2
                for Ln in np.arange(L - 0.1, Lmin - 1e-6, -0.1):
                    cx, cy = f - sg * u * Ln / 2
                    out[k] = (float(cx), float(cy), yaw, float(Ln), H)
                    if bad(k) in ("", "block"):
                        ok = True
                        break
            if not ok:                         # (나) 자리는 원래대로, 방향만 새 것
                out[k] = (orig[k][0], orig[k][1], yaw, orig[k][3], H)
                ok = bad(k) in ("", "block")
            if ok:
                fixed += 1
                continue
            arrange.log.append((k, how[k], why))
            out[k], how[k] = orig[k], ""       # (다) 원래대로
            reverted += 1
    st = dict(in_stall=how.count("stall"), in_group=how.count("group"), on_line=how.count("line"), unchanged=how.count(""), adjusted=fixed, reverted=reverted,
              moved_max_m=round(float(max(np.hypot(o[0] - s_[0], o[1] - s_[1]) for o, s_ in zip(out, sl))), 2) if out else 0.0,
              overlapping_before=int(sum(bool(b) for b in bad0)), overlapping_after=int(sum(bool(bad(k)) for k in range(len(out)))))
    arrange.how = how
    return out, st


def models(rows, extra, ground, c, path_xy=None, scan_xy=None, block_xy=None):
    """단순 차 모델 (몸체 + 캐빈 + 바퀴 상자, 정점 색) → (보이는 메쉬, 충돌 메쉬 = 바닥부터 지붕까지 상자 하나씩, 대수).
    11.10: 차를 지운 v0.8 에서 라이다가 볼 물체가 줄어 AMCL 실패 → 같은 자리에 단순 모델 (D 요청: Visual · Collision 양쪽)."""
    rng = np.random.default_rng(c.get("seed", 0))
    W = c.get("width_m", 1.8)
    vis, col = o3d.geometry.TriangleMesh(), o3d.geometry.TriangleMesh()
    slots.last = {}
    colors = None
    mf = c.get("manual_file")          # 손으로 놓은 차 (recon/tools/car_editor.py 로 만든 json) — 있으면 그대로 놓고 자동 자리 · 가지런히 · 빼기는 안 함
    if mf and not os.path.isabs(os.path.expanduser(mf)):
        mf = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "configs", mf)   # 상대 경로는 recon/configs 기준
    if mf and os.path.exists(os.path.expanduser(mf)):
        man = json.load(open(os.path.expanduser(mf), encoding="utf-8"))["cars"]
        sl = [(float(m["x"]), float(m["y"]), float(np.radians(m["yaw_deg"])), float(m.get("length", 4.6)), float(m.get("height", 1.5))) for m in man]
        colors = [MANUAL_COLORS.get(m.get("color"), None) for m in man]
        slots.last = dict(manual_file=os.path.basename(mf), manual=len(sl))
        if path_xy is not None and len(sl):
            P_ = np.asarray(path_xy)[:, :2]
            slots.last["manual_min_path_m"] = round(float(min(_rect_dist(P_, x, y, yaw, L, W).min() for x, y, yaw, L, H in sl)), 2)
            slots.last["manual_overlaps"] = int(sum(_overlap(a_[:4], b_[:4], W) for i_, a_ in enumerate(sl) for b_ in sl[i_ + 1:]))
        models.slots = [[round(x, 3), round(y, 3), round(float(np.degrees(yaw)), 2), round(L, 3), round(H, 3)] for x, y, yaw, L, H in sl]
    else:
        if mf:
            print(f"      ! 차 배치 파일 없음 → 자동 자리로: {mf}")
        sl = slots(rows, extra, c, path_xy, scan_xy, block_xy)
        if c.get("arrange"):
            sl, slots.last["arrange"] = arrange(sl, c.get("_stalls"), c, path_xy, block_xy)
    if c.get("drop_xy") and colors is None:               # 이 사각형 [x0, x1, y0, y1] 안의 차는 놓지 않음 (v0.9, 11.11 — 한 줄로 빽빽한 북쪽 · 동쪽 줄: 상자 차가 벽처럼 보임)
        keep_ = [o for o in sl if not any(b[0] <= o[0] <= b[1] and b[2] <= o[1] <= b[3] for b in c["drop_xy"])]
        slots.last["dropped_in_boxes"] = len(sl) - len(keep_)
        sl = keep_
    models.slots = [[round(x, 3), round(y, 3), round(float(np.degrees(yaw)), 2), round(L, 3), round(H, 3)] for x, y, yaw, L, H in sl]
    for k_car, (x, y, yaw, L, H) in enumerate(sl):
        z = float(ground.height(np.array([[x, y]]))[0]) if ground is not None else 0.0
        body = PALETTE[rng.integers(len(PALETTE))]
        if colors is not None and colors[k_car] is not None:
            body = colors[k_car]
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
