"""주행 띠 바닥이 몇 겹인지 — 5 cm 격자마다 수직 광선을 z 1.0 에서 아래로 쏘고, 맞은 곳 바로 아래에서 다시 쏘기를 되풀이해
z -0.15~+0.15 안의 면을 모두 셈 (1 mm 안의 맞음은 같은 면 = 삼각형 모서리 중복). 겹 비율 · 겹 간격 · 바닥 높이 퍼짐을 출력.
--cmodel: C 파이프라인(recon build_usd) 충돌 바닥을 흉내 — 이 메쉬 바닥 정점으로 5 cm 바닥 높이 지도(칸 중앙값 → 3×3 중앙값)를 만들고
  single_floor 충돌 바닥(5 cm 격자, 높이 +0) 과 zones.ground_plane 판(25 cm 격자, 높이 −0.002) 사이 간격을 계산.
사용: in_floorlayers.py <mesh.ply> [--png out.png] [--cmodel] [--cell 0.05] [--x0 0 --x1 46 --y0 -0.6 --y1 0.9 --skip 18,27]"""
import sys, numpy as np, open3d as o3d
from scipy import ndimage
a = sys.argv; opt = lambda k, d: a[a.index(k) + 1] if k in a else d
CELL = float(opt('--cell', 0.05)); X0, X1 = float(opt('--x0', 0)), float(opt('--x1', 46)); Y0, Y1 = float(opt('--y0', -0.6)), float(opt('--y1', 0.9))
SK = [float(s) for s in opt('--skip', '18,27').split(',')] if opt('--skip', '18,27') else None; ZLO, ZHI, MERGE = -0.15, 0.15, 0.001
m = o3d.io.read_triangle_mesh(a[1]); V = np.asarray(m.vertices); T = np.asarray(m.triangles)
tz = V[T][:, :, 2]; txy = V[T][:, :, :2].mean(1)   # 띠 근처 삼각형만 남김 (z 0.3 위는 광선이 어차피 지나쳐 셈에 안 들어감 → 결과 같음, 빠름)
keep = (tz.min(1) < 0.3) & (tz.max(1) > -0.3) & (txy[:, 0] > X0 - 0.2) & (txy[:, 0] < X1 + 0.2) & (txy[:, 1] > Y0 - 0.2) & (txy[:, 1] < Y1 + 0.2)
sub = o3d.t.geometry.TriangleMesh(); sub.vertex.positions = o3d.core.Tensor(V.astype(np.float32)); sub.triangle.indices = o3d.core.Tensor(T[keep].astype(np.int32))
sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(sub)
xs = np.arange(X0 + CELL / 2, X1, CELL); ys = np.arange(Y0 + CELL / 2, Y1, CELL); GX, GY = np.meshgrid(xs, ys, indexing='ij')
use = np.ones(GX.shape, bool) if SK is None else ~((GX > SK[0]) & (GX < SK[1]))
n = GX.size; oz = np.full(n, 1.0); alive = np.ones(n, bool); hits = [[] for _ in range(n)]
for it in range(40):
    idx = np.where(alive)[0]
    if not len(idx): break
    r = np.c_[GX.ravel()[idx], GY.ravel()[idx], oz[idx], np.zeros((len(idx), 2)), -np.ones(len(idx))].astype(np.float32)
    res = sc.cast_rays(o3d.core.Tensor(r)); t = res['t_hit'].numpy(); nz = res['primitive_normals'].numpy()[:, 2]
    miss = ~np.isfinite(t); alive[idx[miss]] = False
    zh = oz[idx] - np.where(miss, 0, t)
    for k in np.where(~miss)[0]:
        if zh[k] < ZLO: alive[idx[k]] = False; continue
        if zh[k] <= ZHI: hits[idx[k]].append((zh[k], nz[k]))
    oz[idx[~miss]] = zh[~miss] - 1e-4
surf, seps, top, pat = np.zeros(n, int), [], np.full(n, np.nan), {}
for i, h in enumerate(hits):
    if not h: continue
    h = sorted(h, key=lambda q: -q[0]); s = [h[0]]
    for q in h[1:]:
        if s[-1][0] - q[0] > MERGE: s.append(q)
    surf[i] = len(s); top[i] = s[0][0]
    if use.ravel()[i] and len(s) >= 2:
        seps += [s[j][0] - s[j + 1][0] for j in range(len(s) - 1)]
        p = ''.join('u' if q[1] > 0 else 'd' for q in s); pat[p] = pat.get(p, 0) + 1
U = use.ravel(); nu = U.sum(); hit = surf[U] >= 1; seps = np.array(seps)
print(f'{a[1]}\n  띠 x {X0}~{X1} (뺀 곳 {SK}) · y {Y0}~{Y1} · 칸 {CELL*100:g} cm → 칸 {nu:,}')
print(f'  면이 잡힌 칸 {hit.mean():.1%} | 면 개수 분포 (잡힌 칸 중): ' + ' · '.join(f'{k}겹 {(surf[U][hit] == k).mean():.2%}' for k in range(1, 5)) + f' · 5겹 이상 {(surf[U][hit] >= 5).mean():.2%}')
print(f'  2겹 이상 칸 = 잡힌 칸의 {(surf[U][hit] >= 2).mean():.2%} ({(surf[U] >= 2).sum():,}칸)')
if len(seps):
    print(f'  겹 간격 (위 면 − 아래 면) 중앙 {np.median(seps)*1000:.1f} mm · 5~95% {np.percentile(seps,5)*1000:.1f}~{np.percentile(seps,95)*1000:.1f} mm | '
          + ' · '.join(f'{lo*1000:g}~{hi*1000:g}mm {((seps>=lo)&(seps<hi)).sum()}' for lo, hi in [(0.001, 0.003), (0.003, 0.01), (0.01, 0.03), (0.03, 0.1), (0.1, 1)]))
    print('  겹 무늬 (위→아래 면 법선 u 위 · d 아래):', dict(sorted(pat.items(), key=lambda q: -q[1])[:6]))
tz = top[U][hit]; print(f'  맨 위 바닥 면 높이 중앙 {np.median(tz)*1000:.1f} mm · 5~95% {np.percentile(tz,5)*1000:.1f}~{np.percentile(tz,95)*1000:.1f} mm · 1~99% {np.percentile(tz,1)*1000:.1f}~{np.percentile(tz,99)*1000:.1f} mm')
xb = np.arange(X0, X1 + 1e-9, 2.0); xc = GX.ravel()[U][hit]
print('  x 2 m 구간별 바닥 높이 중앙(mm) · 2겹 이상 비율:', ' '.join(f'[{lo:g}:{np.median(tz[(xc>=lo)&(xc<lo+2)])*1000:+.0f}/{(surf[U][hit][(xc>=lo)&(xc<lo+2)]>=2).mean():.0%}]' for lo in xb[:-1] if ((xc>=lo)&(xc<lo+2)).sum() > 20))
if '--cmodel' in a:   # C 충돌 바닥 두 장 사이 간격 예측
    nv = np.asarray((m.compute_vertex_normals() or m).vertex_normals)
    f = (nv[:, 2] > 0.9) & (np.abs(V[:, 2]) < 0.05) & (V[:, 0] > X0 - 2) & (V[:, 0] < X1 + 2) & (V[:, 1] > Y0 - 1) & (V[:, 1] < Y1 + 1)
    gc, lo = 0.05, np.array([X0 - 2, Y0 - 1]); ij = np.floor((V[f, :2] - lo) / gc).astype(int); shp = ij.max(0) + 2
    key = ij[:, 0] * shp[1] + ij[:, 1]; o = np.lexsort((V[f, 2], key)); ks, st, ct = np.unique(key[o], return_index=True, return_counts=True)
    G = np.full(shp, np.nan); G.flat[ks[ct >= 3]] = V[f, 2][o][st + ct // 2][ct >= 3]
    ok = ~np.isnan(G)   # 빈 칸 = 가장 가까운 잰 칸 값 (C 는 선형 보간 — 띠 안은 거의 다 재어져 차이 작음)
    near =ndimage.distance_transform_edt(~ok, return_distances=False, return_indices=True); G = G[near[0], near[1]]
    G = ndimage.median_filter(G, size=3)
    hgt = lambda xy: ndimage.map_coordinates(G, [(xy[:, 0] - lo[0]) / gc - 0.5, (xy[:, 1] - lo[1]) / gc - 0.5], order=1, mode='nearest')
    def plane_z(xy, cell, z0, off):   # Footprint.plane: 칸 모서리 정점 = 바닥 높이 + z0, 사각형을 (0,0)-(1,1) 대각선으로 나눈 삼각형
        q = (xy - off) / cell; i0 = np.floor(q); u, v = (q - i0).T; c = lambda di, dj: hgt(off + (i0 + [di, dj]) * cell) + z0
        z00, z10, z11, z01 = c(0, 0), c(1, 0), c(1, 1), c(0, 1)
        return np.where(u >= v, z00 + u * (z10 - z00) + v * (z11 - z10), z00 + v * (z01 - z00) + u * (z11 - z01))
    rng = np.random.default_rng(0); P = np.c_[rng.uniform(X0, X1, 400000), rng.uniform(Y0, Y1, 400000)]
    if SK: P = P[~((P[:, 0] > SK[0]) & (P[:, 0] < SK[1]))]
    for off in ([0.0, 0.0], [0.11, 0.07]):
        d = plane_z(P, 0.05, 0.0, lo) - plane_z(P, 0.25, -0.002, lo + off)
        print(f'  [C 흉내] 격자 원점 차 {off} m: single_floor 충돌 바닥 − ground_plane 판 간격 중앙 {np.median(d)*1000:.1f} mm · 5~95% {np.percentile(d,5)*1000:.1f}~{np.percentile(d,95)*1000:.1f} mm'
              f' · 2~7 mm 안 {((d>=0.002)&(d<=0.007)).mean():.0%} · 0 이하(판이 위) {(d<=0).mean():.1%}')
    print(f'  [C 흉내] 바닥 높이 지도 5 cm 칸 {ok.sum():,} (재어진 칸) · 높이 5~95% {np.percentile(G[ok],5)*1000:.1f}~{np.percentile(G[ok],95)*1000:.1f} mm')
if '--png' in a:
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(14, 7), gridspec_kw=dict(height_ratios=[1, 1, 1.4]))
    S = surf.reshape(GX.shape).astype(float); S[~use] = np.nan
    im = ax[0].imshow(S.T, origin='lower', extent=[X0, X1, Y0, Y1], aspect='auto', cmap='viridis', vmin=0, vmax=3); plt.colorbar(im, ax=ax[0], label='surfaces')
    ax[0].set_title(f'{a[1].split("/")[-1]}: surfaces within z -0.15..+0.15 per 5 cm cell (grey = skipped)')
    Z = top.reshape(GX.shape) * 1000; Z[~use] = np.nan
    im = ax[1].imshow(Z.T, origin='lower', extent=[X0, X1, Y0, Y1], aspect='auto', cmap='coolwarm', vmin=-30, vmax=30); plt.colorbar(im, ax=ax[1], label='top floor z (mm)')
    if len(seps): ax[2].hist(np.clip(seps * 1000, 0, 300), bins=150, log=True); ax[2].set_xlabel('separation between stacked surfaces (mm, clipped at 300)'); ax[2].set_ylabel('count')
    plt.tight_layout(); plt.savefig(opt('--png', 'x.png'), dpi=90); print('  →', opt('--png', 'x.png'))
