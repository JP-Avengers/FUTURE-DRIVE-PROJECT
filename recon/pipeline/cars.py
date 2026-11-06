"""스캔된 주차 차 지우기 (v0.8). 차는 촬영 날마다 다르고 메쉬가 찌그러져 보여서 맵에서 뺀다 (11.06).

차 자리 = B outer/objects.csv 의 차 + config cars.extra (B 목록에 없는 덩어리를 메쉬 · 키프레임 사진으로 확인해 적은 것).
build_usd.py 가 색 투영 · 바닥 텍스처 다음에 부름 — 가림 판정에는 차가 있어야 차 색이 바닥에 안 찍힘.
차 자리의 바닥은 floor_texture fill_stalls 가 같은 목록으로 아스팔트로 채움.
"""
import csv

import numpy as np
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
