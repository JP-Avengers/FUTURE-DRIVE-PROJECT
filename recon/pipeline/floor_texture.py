"""바닥 한 장에 키프레임 사진을 텍스처로 굽기 (v0.6 노면 실사화). build_usd.py 가 colorize 다음에 부름.

정점 색(바닥 꼭짓점 5 cm 마다 색 하나)은 15 cm 주차선이 뭉개짐 → 바닥 전체를 cell_m(1 cm) 텍스처 한 장으로.
  1) 밝기 모형: 같은 카메라 묶음끼리 키프레임 이득 + 비네팅(r², r⁴) + 보는 각도(cos) 를 한 번에 최소제곱
     (10 cm 거친 격자에서 여러 장이 같이 본 칸으로). 잔차가 크면(지붕 카메라) 보정 안 함
  2) 굽기: 텍셀마다 가장 선명하게 본(gsd / 사진 가장자리 감쇠 가 작은) top_k 장을 (gsd_min/gsd)^power 로 섞음.
     여러 장을 똑같이 평균하면 자세 오차 3~4 cm 때문에 선이 두 줄로 번짐. 묶음은 순서대로 앞 묶음이 못 칠한 텍셀만
  3) 넓은 밝기 차이(촬영 때 그림자·햇빛) 줄이기 — 아스팔트에만, 약하게
  4) 주차칸 채우기 (fill_stalls): 차가 서 있던 자리 · 그 그림자 · 안 찍힌 곳을 주행 도로의 깨끗한 아스팔트 조각으로.
     찍힌 흰 주차선은 남김
결과: <out>/floor_tex/floor_color.jpg (+ bake_raw.png, bake_meta.npz, fill_mask.png), 바닥 UV 영역.
굽기(1·2)는 키프레임 1,773장에 20~30분 → reuse_bake 면 같은 설정으로 구운 bake_raw.png 를 다시 씀 (3·4 만 다시).
"""
import csv, hashlib, json, os, time

import numpy as np
import open3d as o3d
from PIL import Image
from scipy import ndimage

Image.MAX_IMAGE_PIXELS = None
LUMA = np.array([0.299, 0.587, 0.114], np.float32)


def _bilinear(img, u, v):
    x0, y0 = np.floor(u).astype(int), np.floor(v).astype(int)
    ax, ay = (u - x0)[:, None], (v - y0)[:, None]
    top = img[y0, x0] * (1 - ax) + img[y0, x0 + 1] * ax
    bot = img[y0 + 1, x0] * (1 - ax) + img[y0 + 1, x0 + 1] * ax
    return top * (1 - ay) + bot * ay


class Texels:
    """영역 [x0,x1]×[y0,y1] 의 cell 간격 텍셀. 배열 [iy, ix] (iy = +y 방향). z = 바닥 높이 지도."""

    def __init__(self, region, cell, ground):
        self.x0, self.x1, self.y0, self.y1 = region
        self.cell = cell
        self.nx = int(round((self.x1 - self.x0) / cell))
        self.ny = int(round((self.y1 - self.y0) / cell))
        self.xs = self.x0 + (np.arange(self.nx) + 0.5) * cell
        self.ys = self.y0 + (np.arange(self.ny) + 0.5) * cell
        self.z = np.empty((self.ny, self.nx), np.float32)
        for r0 in range(0, self.ny, 500):                       # 줄 묶음으로 (좌표 배열이 커지지 않게)
            X, Y = np.meshgrid(self.xs, self.ys[r0:r0 + 500])
            self.z[r0:r0 + 500] = ground.height(np.c_[X.ravel(), Y.ravel()]).reshape(X.shape)

    def window(self, C, fwd, max_dist, half_fov_deg):
        """카메라 앞 부채꼴의 경계 상자 → (iy 범위, ix 범위)"""
        f = fwd[:2] / max(np.linalg.norm(fwd[:2]), 1e-9)
        pts = [C[:2]]
        for a in np.radians(np.linspace(-half_fov_deg - 5, half_fov_deg + 5, 7)):
            pts.append(C[:2] + np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]) @ f * max_dist)
        pts = np.array(pts)
        ix0, ix1 = max(int((pts[:, 0].min() - self.x0) / self.cell), 0), min(int((pts[:, 0].max() - self.x0) / self.cell) + 1, self.nx)
        iy0, iy1 = max(int((pts[:, 1].min() - self.y0) / self.cell), 0), min(int((pts[:, 1].max() - self.y0) / self.cell) + 1, self.ny)
        return (iy0, iy1), (ix0, ix1)


def _zbuffer(occ, fr, P):
    K = o3d.core.Tensor(fr.intrinsic.intrinsic_matrix)
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(K, o3d.core.Tensor(np.linalg.inv(P)), fr.intrinsic.width, fr.intrinsic.height)
    return occ.cast_rays(rays)["t_hit"].numpy()


def _project(tex, fr, f, P, zbuf, lift, c, step=1):
    """키프레임 f 에서 보이는 텍셀 → (iy, ix, 색 sRGB 0~1, gsd m, 밝기 특징 [r², r⁴, cos], 가장자리 감쇠) 또는 None.
    가림 = 메쉬 z-buffer, 키프레임 depth 와도 비교 (지나간 차·사람). gsd = 바닥에서 화소 하나가 덮는 길이."""
    K = fr.intrinsic.intrinsic_matrix
    W, H = fr.intrinsic.width, fr.intrinsic.height
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    C, R = P[:3, 3], P[:3, :3]
    (iy0, iy1), (ix0, ix1) = tex.window(C, R[:, 2], c["max_dist_m"], np.degrees(np.arctan(max(cx, W - cx) / fx)))
    if iy1 <= iy0 or ix1 <= ix0:
        return None
    iy, ix = np.mgrid[iy0:iy1:step, ix0:ix1:step]
    iy, ix = iy.ravel(), ix.ravel()
    d = np.c_[tex.xs[ix], tex.ys[iy], tex.z[iy, ix] + lift] - C
    m = np.einsum("ij,ij->i", d[:, :2], d[:, :2]) < c["max_dist_m"] ** 2
    iy, ix, d = iy[m], ix[m], d[m]
    X = d @ R                                                   # 카메라 좌표 (x 오른쪽, y 아래, z 앞)
    z = X[:, 2]
    m = z > 0.3
    iy, ix, X, z, d = iy[m], ix[m], X[m], z[m], d[m]
    u, v = fx * X[:, 0] / z + cx, fy * X[:, 1] / z + cy
    b = c["border_px"]
    m = (u >= b) & (u < W - 1 - b) & (v >= b) & (v < H - 1 - b)
    iy, ix, u, v, z, d = iy[m], ix[m], u[m], v[m], z[m], d[m]
    if not len(iy):
        return None
    dist = np.linalg.norm(d, axis=1)
    cos = -d[:, 2] / dist                                       # 바닥 법선 ≈ +Z (지도 기울기 몇 도는 무시)
    depth = np.asarray(o3d.io.read_image(fr.depth[f])).astype(np.float32) / fr.depth_scale
    ui, vi = np.rint(u).astype(int), np.rint(v).astype(int)
    dk, zb = depth[vi, ui], zbuf[vi, ui]
    m = (cos > c["min_cos"]) & (z <= zb + 0.05 + 0.02 * z) & ((dk == 0) | (np.abs(z - dk) < 0.15 + 0.05 * z))
    if not m.any():
        return None
    iy, ix, u, v, cos, dist = iy[m], ix[m], u[m], v[m], cos[m], dist[m]
    img = np.asarray(Image.open(fr.color[f]).convert("RGB")).astype(np.float32) / 255.0
    col = _bilinear(img, u, v)
    gsd = (dist / (fx * cos)).astype(np.float32)
    r2 = ((u - cx) ** 2 + (v - cy) ** 2) / fx ** 2              # 광축에서 떨어진 정도 (비네팅)
    edge = np.minimum.reduce([u - b, W - 1 - b - u, v - b, H - 1 - b - v])
    feather = np.clip(edge / c["feather_px"], 0.02, 1).astype(np.float32)   # 사진 가장자리일수록 덜 씀 (경계가 칼같이 안 보이게)
    return iy, ix, col, gsd, np.c_[r2, r2 ** 2, cos].astype(np.float32), feather


def _frames_near(tex, poses, margin):
    C = np.array([P[:3, 3] for P in poses])
    return np.where((C[:, 0] > tex.x0 - margin) & (C[:, 0] < tex.x1 + margin) & (C[:, 1] > tex.y0 - margin) & (C[:, 1] < tex.y1 + margin))[0]


def _excluded(boxes, x, y):
    m = np.zeros(len(x), bool)
    for x0, x1, y0, y1 in boxes:
        m |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    return m


def solve_photometric(obs, n_frames, ridge=0.1, gain_range=(0.7, 1.4)):
    """obs = [(frame, cell, rgb, feat)] → (키프레임별 RGB 이득, 계수 β (3,3), feat 평균, 잔차 rms).
    log I = g_f + L_cell + (feat − 평균)·β 를 한 번에 최소제곱 (희소). 교대로 풀면 연속 키프레임끼리만 겹친 칸이 대부분이라
    날짜 간 밝기 차가 거의 안 움직임. g 에 작은 ridge, 3σ 넘는 관측은 빼고 한 번 더.
    관측이 100개 안 되는 키프레임은 이득 1 — 먼 바닥만 조금 본 키프레임에서 한 채널이 튀었음 (10.29)."""
    from scipy.sparse import csr_matrix, vstack
    from scipy.sparse.linalg import lsqr
    fi = np.concatenate([np.full(len(o[1]), o[0]) for o in obs])
    ci = np.concatenate([o[1] for o in obs])
    li = np.log(np.clip(np.concatenate([o[2] for o in obs]), 1e-3, 1))
    F = np.concatenate([o[3] for o in obs]).astype(np.float64)
    Fm = F.mean(0)
    F = F - Fm                                                  # β 가 전체 밝기를 바꾸지 않게 가운데로
    _, cinv, cnt = np.unique(ci, return_inverse=True, return_counts=True)
    keep = cnt[cinv] >= 2                                       # 두 장 이상 본 칸만
    fr_ids, finv = np.unique(fi, return_inverse=True)
    nf, nc, nb = len(fr_ids), cnt.size, F.shape[1]
    g, beta = np.zeros((n_frames, 3)), np.zeros((nb, 3))
    res = np.zeros((len(li), 3))
    use = keep.copy()
    for _ in range(2):
        idx = np.where(use)[0]
        n = len(idx)
        A = csr_matrix((np.c_[np.ones(n), np.ones(n), F[idx]].ravel(),
                        (np.repeat(np.arange(n), 2 + nb),
                         np.c_[finv[idx], nf + cinv[idx], nf + nc + np.arange(nb)[None].repeat(n, 0)].ravel())), shape=(n, nf + nc + nb))
        A = vstack([A, csr_matrix((np.full(nf, ridge), (np.arange(nf), np.arange(nf))), shape=(nf, nf + nc + nb))]).tocsr()
        for ch in range(3):
            x = lsqr(A, np.r_[li[idx, ch], np.zeros(nf)], atol=1e-8, btol=1e-8, iter_lim=5000)[0]
            g[fr_ids, ch], beta[:, ch] = x[:nf], x[nf + nc:]
            res[:, ch] = li[:, ch] - g[fi, ch] - x[nf:nf + nc][cinv] - F @ beta[:, ch]
        use = keep & (np.abs(res).max(1) < 3 * np.sqrt(np.mean(res[keep] ** 2)))
    g[np.bincount(fi[use], minlength=n_frames) < 100] = 0
    g[fr_ids] -= np.median(g[fr_ids], axis=0)                   # 전체 밝기는 그대로 (중앙 키프레임 = 이득 1)
    return np.clip(np.exp(-np.nan_to_num(g)), *gain_range), beta, Fm, float(np.sqrt(np.mean(res[use] ** 2)))


def bake(sets, floor, ground, occ, c, out_dir):
    """sets = [(이름, Frames, 씬 좌표 poses, 바닥 올림 m, 제외 구역)] 우선순위 순. floor = 바닥 메쉬 (정점 색 = 못 본 텍셀 대신).
    → (영역 [x0,x1,y0,y1], 통계). bake_raw.png (위 = +y) · bake_meta.npz 저장."""
    t0 = time.time()
    fv = np.asarray(floor.vertices)
    lo, hi = np.floor(fv[:, :2].min(0) * 4) / 4, np.ceil(fv[:, :2].max(0) * 4) / 4
    region = [float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1])]
    tex = Texels(region, c["cell_m"], ground)
    print(f"      바닥 텍스처: {tex.nx}×{tex.ny} 텍셀 ({c['cell_m'] * 100:g} cm) · 키프레임 묶음 {[s[0] for s in sets]}")
    sel = {s[0]: _frames_near(tex, s[2], c["max_dist_m"]) for s in sets}
    st = dict(region=region, cell_m=c["cell_m"], texels=[tex.nx, tex.ny], photometric={})

    corr = {s[0]: (np.ones((len(s[2]), 3)), np.zeros((3, 3)), np.zeros(3)) for s in sets}
    step = max(1, int(round(0.1 / c["cell_m"])))
    by_name = {s[0]: s for s in sets}
    for members in c.get("photometric_groups", []):             # 같은 카메라끼리 밝기 모형 하나 (비네팅·각도 계수 공유)
        members = [n for n in members if n in by_name]
        if not members:
            continue
        obs, off, n_tot = [], {}, 0
        for n in members:
            _, fr, poses, lift, excl = by_name[n]
            off[n] = n_tot
            for f in sel[n]:
                r = _project(tex, fr, f, poses[f], _zbuffer(occ, fr, poses[f]), lift, c, step=step)
                if r is None:
                    continue
                iy, ix, col, gsd, feat, _ = r
                m = (gsd < 0.04) & ~_excluded(excl, tex.xs[ix], tex.ys[iy])
                obs.append((n_tot + f, iy[m] * tex.nx + ix[m], col[m], feat[m]))
            n_tot += len(poses)
        gains, beta, Fm, rms = solve_photometric(obs, n_tot, gain_range=tuple(c.get("gain_range", (0.7, 1.4))))
        ok = rms <= c.get("photometric_max_rms", 0.06)
        st["photometric"]["+".join(members)] = dict(rms_log=round(rms, 4), used=bool(ok),
                                                    vignette_at_r2_025=round(float(np.exp(beta[0].mean() * 0.25 + beta[1].mean() * 0.0625)), 3),
                                                    cos_coef=[round(float(x), 3) for x in beta[2]])
        print(f"      밝기 모형 {'+'.join(members)}: 잔차 {rms:.3f} → {'보정' if ok else '보정 안 함 (모형이 안 맞음)'} · {time.time() - t0:.0f}s")
        if ok:
            for n in members:
                corr[n] = (gains[off[n]:off[n] + len(by_name[n][2])], beta, Fm)

    K = c.get("top_k", 3)
    best_g = np.full((tex.ny, tex.nx, K), np.inf, np.float16)  # 순위 = gsd / 가장자리 감쇠. 메모리 때문에 float16 (1 cm 바닥 전체 3,300만 텍셀)
    best_c = np.zeros((tex.ny, tex.nx, K, 3), np.float16)
    src = np.full((tex.ny, tex.nx), -1, np.int8)                # 어느 묶음이 칠했나 (주차칸 채우기 · 점검)
    for si, (n, fr, poses, lift, excl) in enumerate(sets):
        gains, beta, Fm = corr[n]
        filled_before = np.isfinite(best_g[..., 0])
        for f in sel[n]:
            r = _project(tex, fr, f, poses[f], _zbuffer(occ, fr, poses[f]), lift, c)
            if r is None:
                continue
            iy, ix, col, gsd, feat, feather = r
            m = ~filled_before[iy, ix] & ~_excluded(excl, tex.xs[ix], tex.ys[iy])
            if not m.any():
                continue
            iy, ix, col, gsd, feat, feather = iy[m], ix[m], col[m], gsd[m], feat[m], feather[m]
            col = np.clip(np.nan_to_num(col * gains[f] * np.clip(np.exp(-(feat - Fm) @ beta), 0.7, 1.4)), 0, 1)
            key = (gsd / feather).astype(np.float16)
            j = np.argmax(best_g[iy, ix], axis=1)               # 가장 흐린 자리와 비교해 교체
            better = key < best_g[iy, ix, j]
            iy, ix, j = iy[better], ix[better], j[better]
            best_g[iy, ix, j] = key[better]
            best_c[iy, ix, j] = col[better]
            src[iy, ix] = si
        print(f"      {n}: 키프레임 {len(sel[n])}장 · 칠한 텍셀 {np.isfinite(best_g[..., 0]).mean() * 100:.1f}% · {time.time() - t0:.0f}s")

    gmin = best_g.min(axis=2)
    seen = np.isfinite(gmin)
    col = np.zeros((tex.ny, tex.nx, 3), np.float32)
    wsum = np.zeros((tex.ny, tex.nx), np.float32)
    for j in range(K):                                          # 메모리 아끼려고 한 자리씩 ([..., j] 면 색 채널 축이 잡힘 — 10.29 빨간 톱니)
        fin = np.isfinite(best_g[:, :, j])
        w = np.zeros((tex.ny, tex.nx), np.float32)
        w[fin] = (gmin[fin].astype(np.float32) / np.maximum(best_g[:, :, j][fin].astype(np.float32), 1e-6)) ** c.get("power", 3.0)
        col += best_c[:, :, j].astype(np.float32) * w[..., None]
        wsum += w
    del best_c, best_g
    col /= np.maximum(wsum, 1e-9)[..., None]
    seen &= np.isfinite(col).all(-1)
    uy, ux = np.nonzero(~seen)                                  # 못 본 텍셀 = 바닥 정점 색 (v0.5 방식)
    col[uy, ux] = _vertex_color_sampler(floor, tex)(uy, ux)
    _save(col, os.path.join(out_dir, "bake_raw.png"))
    gsd = np.where(seen, gmin, np.nan).astype(np.float16)
    np.savez_compressed(os.path.join(out_dir, "bake_meta.npz"), src=src, gsd=gsd, region=np.array(region), cell=c["cell_m"],
                        sets=np.array([s[0] for s in sets]))
    q = np.nanpercentile(gsd[::4, ::4].astype(np.float32), [10, 50, 90]) * 100
    st.update(seen_pct=round(100 * float(seen.mean()), 1), gsd_cm_p10_50_90=[round(float(x), 2) for x in q],
              by_set_pct={s[0]: round(100 * float((src == i).mean()), 1) for i, s in enumerate(sets)}, bake_seconds=round(time.time() - t0, 1))
    return region, st


def _vertex_color_sampler(floor, tex, g=0.05):
    """바닥 정점 (g 격자 모서리, 정점 색) → 텍셀 위치 (iy, ix) 에서 이중선형으로 뽑는 함수."""
    v, c = np.asarray(floor.vertices), np.asarray(floor.vertex_colors)
    ix = np.rint((v[:, 0] - tex.x0) / g).astype(int)
    iy = np.rint((v[:, 1] - tex.y0) / g).astype(int)
    grid = np.full((int(np.ceil((tex.y1 - tex.y0) / g)) + 1, int(np.ceil((tex.x1 - tex.x0) / g)) + 1, 3), np.nan, np.float32)
    ok = (ix >= 0) & (ix < grid.shape[1]) & (iy >= 0) & (iy < grid.shape[0])
    grid[iy[ok], ix[ok]] = c[ok]
    hole = np.isnan(grid[..., 0])
    if hole.any() and (~hole).any():
        idx = ndimage.distance_transform_edt(hole, return_distances=False, return_indices=True)
        grid = grid[idx[0], idx[1]]

    def sample(ty, tx):
        fy, fx = (tex.ys[ty] - tex.y0) / g, (tex.xs[tx] - tex.x0) / g
        return np.stack([ndimage.map_coordinates(grid[..., ch], [fy, fx], order=1, mode="nearest") for ch in range(3)], -1)
    return sample


def _save(img, path, quality=None):
    im = Image.fromarray((np.clip(img[::-1], 0, 1) * 255 + 0.5).astype(np.uint8))   # 위 = +y
    im.save(path, **({"quality": quality} if quality else {}))


def finish(out_dir, floor, c, root):
    """bake_raw.png → (넓은 밝기 차이 줄이기) → (주차칸 채우기) → floor_color.jpg. 통계 반환."""
    img = np.asarray(Image.open(os.path.join(out_dir, "bake_raw.png")))[::-1].astype(np.float32) / 255   # [iy, ix]
    meta = np.load(os.path.join(out_dir, "bake_meta.npz"))
    src, gsd = meta["src"], meta["gsd"].astype(np.float32)
    x0, x1, y0, y1 = meta["region"]
    cell = float(meta["cell"])
    H, W = src.shape
    xs, ys = x0 + (np.arange(W) + 0.5) * cell, y0 + (np.arange(H) + 0.5) * cell
    k = max(1, int(round(0.1 / cell)))                          # 10 cm 칸 (넓은 밝기 계산용)
    Hc, Wc = H // k, W // k

    def coarse(a, fn):
        return fn(a[:Hc * k, :Wc * k].reshape(Hc, k, Wc, k, *a.shape[2:]), axis=(1, 3))

    def up(a):
        return ndimage.zoom(a, (H / Hc, W / Wc), order=1)[:H, :W]

    def smooth_valid(val, valid, sigma):
        den = ndimage.gaussian_filter(valid.astype(np.float32), sigma)
        return ndimage.gaussian_filter(np.where(valid, val, 0).astype(np.float32), sigma) / np.maximum(den, 1e-6), den

    # 바닥 구역 (B ground_zones: grid[ix, iy], 1 아스팔트 · 2 풀 · 3 붉은 보도) · 실제 바닥 메쉬가 깔린 곳
    fs = c.get("fill_stalls") or {}
    zones_path = os.path.join(root, fs["zones"]) if fs.get("zones") else None
    if zones_path and os.path.exists(zones_path):
        z = np.load(zones_path)
        zi = np.clip(((xs - z["lo"][0]) / float(z["cell"])).astype(int), 0, z["zone"].shape[0] - 1)
        zj = np.clip(((ys - z["lo"][1]) / float(z["cell"])).astype(int), 0, z["zone"].shape[1] - 1)
        asphalt = ndimage.binary_dilation(z["zone"] == 1, iterations=2)[zi[None, :], zj[:, None]]   # 10 cm 구역 경계가 거칠어 20 cm 넓게
        other = ndimage.binary_dilation(np.isin(z["zone"], [2, 3]), iterations=3)[zi[None, :], zj[:, None]]
    else:
        asphalt, other = np.ones((H, W), bool), np.zeros((H, W), bool)
    fv, g5 = np.asarray(floor.vertices), 0.05
    fp5 = np.zeros((int(np.ceil((y1 - y0) / g5)) + 1, int(np.ceil((x1 - x0) / g5)) + 1), bool)
    fp5[np.clip(((fv[:, 1] - y0) / g5).astype(int), 0, fp5.shape[0] - 1), np.clip(((fv[:, 0] - x0) / g5).astype(int), 0, fp5.shape[1] - 1)] = True
    on_floor = ndimage.binary_closing(fp5, iterations=2)[((ys - y0) / g5).astype(int)[:, None], ((xs - x0) / g5).astype(int)[None, :]]

    # (3) 넓은 밝기 차이 (촬영 때 그림자·햇빛): 아스팔트 10 cm 칸 밝기의 넓은 흐름만 전체 중앙값 쪽으로, 아스팔트에만
    lum = img @ LUMA
    med = coarse(lum, np.median)
    a_c = asphalt[::k, ::k][:Hc, :Wc]
    gl = float(np.median(med[a_c & (med > 0.05)]))
    asph_c = (med > gl * 0.5) & (med < gl * 1.3) & a_c          # 페인트·빈칸·물체 뺌
    fl = c.get("flatten") or {}
    ratio = np.ones((H, W), np.float32)
    if fl.get("mode") == "local":
        # 아스팔트 색을 한 가지로: sigma_m(0.3 m) 보다 넓은 밝기·색 흐름은 전체 아스팔트 중앙 색으로 바꾸고 그보다 작은 무늬
        # (알갱이 · 주차선 · 작은 얼룩)만 사진에서 살림. 회차·방향(해를 마주 봄 / 등짐)·날짜마다 다른 밝기가 이어 붙인 경계에서
        # 계단·물결 얼룩이 됨 — 1.2 m 로 약하게 줄이면 거의 그대로 남음 (10.30 위에서 본 그림)
        # 대상 = 바닥 위 회색 계열(채도 낮음) · 페인트보다 어두운 칸 (B 구역은 색으로 나눈 대략값이라 햇빛에 밝게 찍힌 아스팔트를
        # '기타'로 넣어 그 자리만 그대로 남았음 — 10.30). 풀 · 붉은 보도 둘레는 뺌
        med3 = coarse(img, np.median)
        floor_c = coarse(on_floor, np.all) & ~coarse(other, np.any)
        sat_c = med3.max(-1) - med3.min(-1)
        asph_c = floor_c & (sat_c < 0.08) & (med > gl * fl.get("low", 0.35)) & (med < gl * fl.get("high", 1.6))
        target = np.median(med3[asph_c & a_c], axis=0)
        soft = np.clip(ndimage.gaussian_filter(asph_c.astype(np.float32), 2) * 1.5, 0, 1)
        rc = np.ones((Hc, Wc, 3), np.float32)
        for ch in range(3):
            low, den = smooth_valid(med3[..., ch], asph_c, fl.get("sigma_m", 0.3) / (k * cell))
            r = np.clip(target[ch] / np.where(den > 0.02, low, target[ch]), *fl.get("range", (0.6, 1.6)))
            rc[..., ch] = 1 + (r - 1) * soft
        img = np.clip(img * np.stack([up(rc[..., ch]) for ch in range(3)], -1), 0, 1)
        ratio = up(rc.mean(-1))
        lum = img @ LUMA
        med = coarse(lum, np.median)
        gl = float(np.median(med[a_c & (med > 0.05)]))
        asph_c = (med > gl * 0.5) & (med < gl * 1.3) & a_c
    elif fl:
        low, den = smooth_valid(med, asph_c, fl.get("sigma_m", 1.2) / (k * cell))
        rc = np.clip((gl / np.where(den > 0.05, low, gl)) ** fl.get("strength", 0.5), *fl.get("range", (0.85, 1.2)))
        soft = np.clip(ndimage.gaussian_filter(asph_c.astype(np.float32), 3) * 1.5, 0, 1)
        ratio = up(1 + (rc - 1) * soft)
        img = np.clip(img * ratio[..., None], 0, 1)
        lum = img @ LUMA
        med = coarse(lum, np.median)
    st = dict(flatten_ratio_p5_p95=np.percentile(ratio[::k, ::k], [5, 95]).round(3).tolist())

    # (3b) 사진이 안 닿은 곳(바닥 정점 색) 다듬기: 정점 색은 5 cm 격자로 각지고, 사진 범위 경계는 키프레임 화각 모서리라
    #      계단처럼 끊겨 보임 (D 10.30 영상, 섬 가장자리) → 안쪽은 3 cm 흐리고 경계 둘레 seam_m 은 넓게 섞음
    seam = c.get("unseen_seam_m", 0.0)
    if seam and (src < 0).any():
        unseen = src < 0
        r = seam / cell
        band = ((ndimage.distance_transform_edt(unseen) < r) & unseen) | ((ndimage.distance_transform_edt(~unseen) < r) & ~unseen)
        for ch in range(3):
            soft3 = ndimage.gaussian_filter(img[..., ch], 0.03 / cell)
            wide = ndimage.gaussian_filter(img[..., ch], r / 2)
            img[..., ch] = np.where(band, wide, np.where(unseen, soft3, img[..., ch]))
        del band, soft3, wide
        lum = img @ LUMA
        st["unseen_seam_m"] = seam

    # (4) 주차칸 채우기
    obj_path = os.path.join(root, fs["objects"]) if fs.get("objects") else None
    if obj_path and os.path.exists(obj_path):
        cars = [r for r in csv.DictReader(open(obj_path, encoding="utf-8")) if r["type"] == "car"]
        core, rect = np.zeros((H, W), bool), np.zeros((H, W), bool)
        ml, mw = fs.get("margin_len_m", 0.3), fs.get("margin_side_m", 0.1)
        for r in cars:                                          # 차 사각형 (B 가 붙어 선 차를 2.5 m 단위로 나눈 것)
            cx, cy, yaw = float(r["x"]), float(r["y"]), np.radians(float(r["yaw_deg"]))
            L, Wd = float(r["length_m"]), float(r["width_m"])
            rad = np.hypot(L, Wd) / 2 + ml + 0.1
            i0, i1 = max(int((cx - rad - x0) / cell), 0), min(int((cx + rad - x0) / cell) + 1, W)
            j0, j1 = max(int((cy - rad - y0) / cell), 0), min(int((cy + rad - y0) / cell) + 1, H)
            if i1 <= i0 or j1 <= j0:
                continue
            X, Y = np.meshgrid(xs[i0:i1] - cx, ys[j0:j1] - cy)
            a, b = X * np.cos(yaw) + Y * np.sin(yaw), -X * np.sin(yaw) + Y * np.cos(yaw)   # 차 길이 · 폭 방향
            core[j0:j1, i0:i1] |= (np.abs(a) < L / 2) & (np.abs(b) < Wd / 2)
            rect[j0:j1, i0:i1] |= (np.abs(a) < L / 2 + ml) & (np.abs(b) < Wd / 2 + mw)
        local, _ = smooth_valid(med, asph_c, 1.0 / (k * cell))   # 둘레 아스팔트 밝기 (σ 1 m)
        local = up(np.where(np.isfinite(local), local, gl))
        dist_car = ndimage.distance_transform_edt(~rect[::k, ::k]) * k * cell
        near_car = up((dist_car < fs.get("shadow_m", 1.2)).astype(np.float32)) > 0.5
        near_unseen = up((dist_car < fs.get("unseen_m", 3.0)).astype(np.float32)) > 0.5
        sat = img.max(-1) - img.min(-1)
        dark = ((lum < 0.8 * local) & (sat < 0.1)) | (lum < 0.5 * local)    # 회색 계열로 어둡거나 아주 어두운 곳 (차 그림자 · 차 밑)
        paint = (lum > 1.3 * local) & (sat < 0.15) & (gsd < 0.08) & (src >= 0)   # 찍힌 흰 페인트 (길에서 먼 주차선은 gsd 3~8 cm)
        fillable = asphalt & ~other & on_floor                  # 풀·붉은 보도 둘레 30 cm 는 안 채움 (구역이 색으로 나눈 대략값)
        fill = (rect | (near_car & dark) | (near_unseen & (src < 0))) & fillable
        fill &= ~ndimage.binary_dilation(paint, iterations=3)   # 흰 주차선은 차 사각형 안이어도 남김
        fill = ndimage.binary_opening(ndimage.binary_closing(fill, structure=np.ones((5, 5))), structure=np.ones((3, 3))) & fillable
        img, n_patch = _fill_asphalt(img, fill, src, gsd, asphalt, local, lum, asph_c, coarse, up, smooth_valid, k, cell, fs)
        _save(fill.astype(np.float32)[::4, ::4], os.path.join(out_dir, "fill_mask_4cm.png"))
        st.update(cars=len(cars), filled_m2=round(float(fill.sum()) * cell * cell, 1),
                  filled_in_cars_m2=round(float((fill & rect).sum()) * cell * cell, 1), patches=n_patch)
    st["floor_m2"] = round(float(on_floor.sum()) * cell * cell, 1)

    # (5) 잔무늬 되살리기: 여러 장을 섞고 색을 통일하며 1~4 cm 알갱이 무늬가 실제 사진의 절반으로 약해짐
    #     (밝기 대비 표준편차: 본네트 사진 2 m 앞 2.2% · 텍스처 1.1~1.2%, 10.30) → 모자란 만큼만 CC0 색 사진의 같은 크기 무늬로 채움
    dt = c.get("detail") or {}
    dpath = os.path.expanduser(dt["color"]) if dt.get("color") else None
    if dpath and os.path.exists(dpath):
        def band(a):                                            # 1~4 텍셀 크기 무늬 (σ 0.7 ↔ 4 차이)
            return ndimage.gaussian_filter(a, 0.7) - ndimage.gaussian_filter(a, 4)
        n = int(round(dt.get("tile_m", 2.0) / cell))
        ref = np.asarray(Image.open(dpath).convert("L").resize((n, n), Image.LANCZOS)).astype(np.float32)
        ref = band(np.tile(ref, (3, 3)))[n:2 * n, n:2 * n]      # 반복 경계가 끊기지 않게 3×3 으로 이어 걸러서 가운데만
        ref /= ref.std()
        cur_rel = np.sqrt(ndimage.gaussian_filter(band(lum) ** 2, 0.5 / cell)) / np.maximum(ndimage.gaussian_filter(lum, 0.5 / cell), 1e-3)
        mask = up(ndimage.gaussian_filter(asph_c.astype(np.float32), 2)) * (lum < 1.3 * up(np.where(asph_c, med, gl)))   # 아스팔트에만, 페인트 빼고
        add = np.sqrt(np.clip(dt.get("target_rel_std", 0.022) ** 2 - cur_rel ** 2, 0, None)) * np.clip(mask, 0, 1)
        iy0, ix0 = int(np.floor(y0 / cell)), int(np.floor(x0 / cell))   # 세계 좌표에 고정 (판마다 같은 자리에 같은 무늬)
        D = ref[((np.arange(H) + iy0) % n)[:, None], ((np.arange(W) + ix0) % n)[None, :]]
        img = np.clip(img * (1 + add * D)[..., None], 0, 1)
        st["detail"] = dict(before_rel_std_p50=round(float(np.median(cur_rel[::50, ::50][mask[::50, ::50] > 0.5])) * 100, 2),
                            added_rel_std_p50=round(float(np.median(add[::50, ::50][mask[::50, ::50] > 0.5])) * 100, 2))
    path = os.path.join(out_dir, "floor_color.jpg")
    _save(img, path, quality=c.get("jpeg_quality", 92))
    return path, st


def _fill_asphalt(img, fill, src, gsd, asphalt, local, lum, asph_c, coarse, up, smooth_valid, k, cell, fs):
    """채울 무늬: 주행 경로 가까이서 가장 선명하게 찍힌 깨끗한 아스팔트 조각(patch_m)들을 60 cm 블록으로 무작위로 돌려 이어 붙임
    (20 cm 겹쳐 섞음) → 둘레 아스팔트 색(σ 0.6 m)에 맞춤 → 경계 5 cm 섞음."""
    rng = np.random.default_rng(0)
    H, W = fill.shape
    P = int(round(fs.get("patch_m", 1.2) / cell))
    good = (src == 0) & (gsd < 0.012) & asphalt & ~fill & (np.abs(lum - local) < 0.12 * local)
    cand = np.argwhere(ndimage.minimum_filter(coarse(good.astype(np.float32), np.median) > 0.99, size=P // k + 2))   # 창 전체가 깨끗한 칸
    rng.shuffle(cand)
    patches = []
    for cy, cx in cand:
        j, i = cy * k - P // 2, cx * k - P // 2
        if j < 0 or i < 0 or j + P > H or i + P > W or any(abs(cy - q[0]) < P // k and abs(cx - q[1]) < P // k for _, q in patches):
            continue
        p = img[j:j + P, i:i + P]
        if (p @ LUMA).std() > 0.06:
            continue
        patches.append((p / p.reshape(-1, 3).mean(0), (cy, cx)))   # 평균 1 로 (무늬만)
        if len(patches) >= fs.get("patches", 12):
            break
    if not patches or not fill.any():
        return img, len(patches)
    fill_c = coarse(fill, np.any)
    mean_c = coarse(img, np.mean)
    ref = asph_c & ~fill_c
    gcol = mean_c[ref].mean(0)
    target = []
    for ch in range(3):
        t, den = smooth_valid(mean_c[..., ch], ref, 0.6 / (k * cell))
        target.append(up(np.where(den > 0.02, t, gcol[ch])))
    target = np.stack(target, -1)
    B, O = int(round(0.6 / cell)), int(round(0.2 / cell))
    ramp = np.minimum(np.minimum(np.arange(B + O) + 1, (B + O) - np.arange(B + O)).astype(np.float32) / O, 1)
    wblk = ramp[:, None] * ramp[None, :]
    lab, _ = ndimage.label(ndimage.binary_dilation(fill, iterations=O))
    for sl in ndimage.find_objects(lab):
        j0, j1, i0, i1 = sl[0].start, sl[0].stop, sl[1].start, sl[1].stop
        acc = np.zeros((j1 - j0 + B + O, i1 - i0 + B + O, 3), np.float32)
        wsum = np.zeros(acc.shape[:2], np.float32)
        for bj in range(0, j1 - j0 + 1, B):
            for bi in range(0, i1 - i0 + 1, B):
                p = patches[rng.integers(len(patches))][0]
                oy, ox = rng.integers(0, P - (B + O), 2)
                acc[bj:bj + B + O, bi:bi + B + O] += np.rot90(p[oy:oy + B + O, ox:ox + B + O], rng.integers(4)) * wblk[..., None]
                wsum[bj:bj + B + O, bi:bi + B + O] += wblk
        tex = acc[:j1 - j0, :i1 - i0] / np.maximum(wsum[:j1 - j0, :i1 - i0], 1e-6)[..., None]
        alpha = np.clip(ndimage.distance_transform_edt(fill[j0:j1, i0:i1]) / (0.05 / cell), 0, 1)[..., None]
        img[j0:j1, i0:i1] = img[j0:j1, i0:i1] * (1 - alpha) + np.clip(tex * target[j0:j1, i0:i1], 0, 1) * alpha
    return img, len(patches)


def write_obj(d, floor, tex):
    """보기용 텍스처 메쉬 floor.obj + floor.mtl + 색 텍스처 (Open3D · view_ply 로 열림). 요철·거칠기는 USD 에만."""
    import shutil
    os.makedirs(d, exist_ok=True)
    v, t = np.asarray(floor.vertices), np.asarray(floor.triangles) + 1
    x0, x1, y0, y1 = tex["region"]
    name = os.path.basename(tex["color"])
    with open(os.path.join(d, "floor.obj"), "w") as fh:
        fh.write("mtllib floor.mtl\nusemtl floor\n")
        np.savetxt(fh, v, fmt="v %.4f %.4f %.4f")
        np.savetxt(fh, np.c_[(v[:, 0] - x0) / (x1 - x0), (v[:, 1] - y0) / (y1 - y0)], fmt="vt %.6f %.6f")
        np.savetxt(fh, np.repeat(t, 2, axis=1), fmt="f %d/%d %d/%d %d/%d")
    open(os.path.join(d, "floor.mtl"), "w").write(f"newmtl floor\nKa 1 1 1\nKd 1 1 1\nmap_Kd {name}\n")
    shutil.copyfile(tex["color"], os.path.join(d, name))


def run(cfg, out, floor, occ_mesh, ground, sets):
    """build_usd 에서 부름. floor = 바닥 메쉬 (색 투영 뒤, 정점 색 있음), occ_mesh = 가림 판정용 (바닥 + 물체).
    → (텍스처 정보 dict — usd_utils.write_scene materials 용, 통계)"""
    c = cfg["floor_texture"]
    d = os.path.join(out, "floor_tex")
    os.makedirs(d, exist_ok=True)
    # 같은 설정·입력으로 구운 결과가 있으면 굽기(20~30분)는 건너뜀 — 마무리(3·4)만 다시
    key = dict(cell_m=c["cell_m"], max_dist_m=c["max_dist_m"], top_k=c.get("top_k", 3), power=c.get("power", 3.0),
               border_px=c["border_px"], feather_px=c["feather_px"], min_cos=c["min_cos"], groups=c.get("photometric_groups"),
               max_rms=c.get("photometric_max_rms"), gain_range=c.get("gain_range"),
               sets=[(s[0], len(s[2]), round(float(s[3]), 4), s[4]) for s in sets],
               floor=hashlib.sha1(np.asarray(floor.vertices, np.float32).tobytes()).hexdigest()[:12])
    key_s = json.dumps(key, sort_keys=True)
    stamp = os.path.join(d, "bake_key.json")
    if c.get("reuse_bake") and os.path.exists(stamp) and open(stamp, encoding="utf-8").read() == key_s \
            and os.path.exists(os.path.join(d, "bake_raw.png")):
        st = json.load(open(os.path.join(d, "bake_stats.json"), encoding="utf-8"))
        region = st["region"]
        print("      바닥 텍스처: 같은 설정으로 구운 결과 다시 씀 (floor_tex/bake_raw.png)")
    else:
        occ = o3d.t.geometry.RaycastingScene()
        occ.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(occ_mesh))
        region, st = bake(sets, floor, ground, occ, c, d)
        del occ
        json.dump(st, open(os.path.join(d, "bake_stats.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        open(stamp, "w", encoding="utf-8").write(key_s)
    root = os.path.expanduser(cfg["input"]["root"])
    color, fst = finish(d, floor, c, root)
    st["finish"] = fst
    m = c.get("material", {})
    info = dict(color=color, region=region, tile_m=m.get("tile_m", 1.0), normal_strength=m.get("normal_strength", 1.0),
                normal=os.path.expanduser(m["normal"]) if m.get("normal") else None,
                roughness=os.path.expanduser(m["roughness"]) if m.get("roughness") else None)
    for k_ in ("normal", "roughness"):
        if info[k_] and not os.path.exists(info[k_]):
            print(f"      ! 재질 {k_} 파일 없음 → 빼고 씀: {info[k_]}")
            info[k_] = None
    return info, st
