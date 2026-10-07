"""키프레임 RGB 를 메쉬에 투영해 정점 색 만들기 + 키프레임 정합 점검.

B 맵 v0.2 인계 조건: 형상은 원본 depth 메쉬, 색은 키프레임 RGB 를 그 메쉬에 투영 (가림 처리 필수).
키프레임마다
  1) 메쉬를 그 카메라에서 광선으로 그린 깊이(z-buffer)보다 뒤에 있는 점 = 가려짐 → 안 씀
  2) 키프레임 depth 와도 비교 — 메쉬에 없는 물체(지나간 사람·차)나 자세 오차로 어긋난 색을 거름 (depth 없는 픽셀은 1만)
  3) 점수 = 면이 카메라를 향한 정도(cos) / 거리 → 점마다 점수 높은 top_k 키프레임 색을 점수² 가중 평균
     (가까운 키프레임 색 우선. 여러 장을 섞어 노출 차이로 생기는 얼룩을 줄임)
어느 키프레임에도 안 보인 점은 가장 가까운 색칠된 점의 색 (멀수록 바닥 평균 색 쪽으로).
"""
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


def raycasting_scene(mesh):
    rs = o3d.t.geometry.RaycastingScene()
    rs.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return rs


class _Cells:
    """점들을 위에서 본 cell 칸으로 묶어 두고, 카메라 앞쪽 칸의 점만 빠르게 꺼낸다."""

    def __init__(self, points, cell):
        lo = points[:, :2].min(0)
        ij = np.floor((points[:, :2] - lo) / cell).astype(int)
        ny = ij[:, 1].max() + 1
        key = ij[:, 0] * ny + ij[:, 1]
        self.order = np.argsort(key, kind="stable")
        keys, self.start, self.count = np.unique(key[self.order], return_index=True, return_counts=True)
        self.centers = lo + (np.c_[keys // ny, keys % ny] + 0.5) * cell
        self.cell = cell

    def near(self, C, fwd, max_dist, half_fov):
        d = self.centers - C[:2]
        dist = np.linalg.norm(d, axis=1)
        f = fwd[:2] / max(np.linalg.norm(fwd[:2]), 1e-9)
        ang = np.degrees(np.arccos(np.clip((d @ f) / np.maximum(dist, 1e-9), -1, 1)))
        sel = (dist < max_dist + self.cell) & ((ang < half_fov + 15) | (dist < 2 * self.cell))
        if not sel.any():
            return np.zeros(0, int)
        return np.concatenate([self.order[s:s + n] for s, n in zip(self.start[sel], self.count[sel])])


def _bilinear(img, u, v):
    x0, y0 = np.floor(u).astype(int), np.floor(v).astype(int)
    ax, ay = (u - x0)[:, None], (v - y0)[:, None]
    top = img[y0, x0] * (1 - ax) + img[y0, x0 + 1] * ax
    bot = img[y0 + 1, x0] * (1 - ax) + img[y0 + 1, x0 + 1] * ax
    return top * (1 - ay) + bot * ay


def project_colors(points, normals, frames, poses, occluder, c, require_depth=False, qc=False):
    """points·normals (N,3) 에 키프레임 색을 투영. occluder = 가림 판정용 RaycastingScene (보통 그 메쉬 자신).
    require_depth = 키프레임 depth 로 확인된 점만 (메쉬 밖 평면처럼 가림 물체 형상이 없는 곳).
    반환: (색 (N,3), 보인 점 bool (N,), 키프레임별 점검 목록 — qc=True 일 때)"""
    points, normals = np.asarray(points, float), np.asarray(normals, float)
    intr = frames.intrinsic
    W, H = intr.width, intr.height
    K = intr.intrinsic_matrix
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    half_fov = np.degrees(np.arctan(max(cx, W - cx) / fx))
    k, max_dist, border = c.get("top_k", 3), c.get("max_dist_m", 15.0), c.get("border_px", 4)
    tol0, tol1 = c.get("occlusion_tol_m", 0.05), c.get("occlusion_tol_per_m", 0.02)
    dtol0, dtol1 = c.get("depth_tol_m", 0.1), c.get("depth_tol_per_m", 0.05)
    cells = _Cells(points, 2.0)
    best_s = np.zeros((len(points), k), np.float32)
    best_c = np.zeros((len(points), k, 3), np.float32)
    rows = []
    Kt = o3d.core.Tensor(K)
    for f, P in enumerate(poses):
        C, R = P[:3, 3], P[:3, :3]
        rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(Kt, o3d.core.Tensor(np.linalg.inv(P)), W, H)
        zbuf = occluder.cast_rays(rays)["t_hit"].numpy()             # 광선 방향의 z 성분이 1 → t_hit = 카메라 z 깊이
        depth = np.asarray(o3d.io.read_image(frames.depth[f])).astype(np.float32) / frames.depth_scale
        if qc:
            ok = (depth > 0) & (depth < c.get("qc_max_depth_m", 8.0)) & np.isfinite(zbuf)
            diff = depth[ok] - zbuf[ok]
            rows.append(dict(id=frames.ids[f] if frames.ids else f, x=round(float(C[0]), 2), y=round(float(C[1]), 2),
                             pixels=int(ok.sum()),
                             median_abs_cm=round(100 * float(np.median(np.abs(diff))), 1) if len(diff) else None,
                             within_10cm_pct=round(100 * float((np.abs(diff) < 0.1).mean()), 1) if len(diff) else None,
                             bias_cm=round(100 * float(np.median(diff)), 1) if len(diff) else None))

        idx = cells.near(C, R[:, 2], max_dist, half_fov)
        if not len(idx):
            continue
        X = (points[idx] - C) @ R                                    # 카메라 좌표 (x 오른쪽, y 아래, z 앞)
        z = X[:, 2]
        m = z > 0.3
        idx, X, z = idx[m], X[m], z[m]
        u, v = fx * X[:, 0] / z + cx, fy * X[:, 1] / z + cy
        m = (u >= border) & (u < W - 1 - border) & (v >= border) & (v < H - 1 - border) & (np.linalg.norm(X, axis=1) < max_dist)
        idx, z, u, v = idx[m], z[m], u[m], v[m]
        view = C - points[idx]
        cos = np.sum(normals[idx] * view, axis=1) / np.linalg.norm(view, axis=1)
        ui, vi = np.rint(u).astype(int), np.rint(v).astype(int)
        dk = depth[vi, ui]
        m = (cos > c.get("min_cos", 0.1)) & (z <= zbuf[vi, ui] + tol0 + tol1 * z)
        m &= (np.abs(z - dk) < dtol0 + dtol1 * z) if require_depth else ((dk == 0) | (np.abs(z - dk) < dtol0 + dtol1 * z))
        if not m.any():
            continue
        idx, z, u, v, cos = idx[m], z[m], u[m], v[m], cos[m]
        img = np.asarray(o3d.io.read_image(frames.color[f])).astype(np.float32) / 255.0
        col = _bilinear(img, u, v)
        s = (cos / z).astype(np.float32)
        j = np.argmin(best_s[idx], axis=1)                            # top_k 중 가장 약한 자리와 비교해 교체
        better = s > best_s[idx, j]
        best_s[idx[better], j[better]] = s[better]
        best_c[idx[better], j[better]] = col[better]

    w = best_s ** 2
    tot = w.sum(axis=1)
    seen = tot > 0
    colors = np.zeros((len(points), 3))
    colors[seen] = (best_c[seen] * w[seen, :, None]).sum(axis=1) / tot[seen, None]
    if seen.any() and not seen.all():
        d, j = cKDTree(points[seen]).query(points[~seen])
        up = seen & (normals[:, 2] > 0.9)
        base = np.median(colors[up] if up.any() else colors[seen], axis=0)
        a = np.clip((d - 0.5) / 0.5, 0, 1)[:, None]                   # 0.5 m 까지는 가까운 색, 1 m 넘으면 바닥 평균 색
        colors[~seen] = (1 - a) * colors[seen][j] + a * base
    elif not seen.any():
        colors[:] = 0.6
    return np.clip(colors, 0, 1), seen, rows
