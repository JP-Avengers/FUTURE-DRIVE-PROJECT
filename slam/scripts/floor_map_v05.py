"""v0.5 바닥 높이 지도 (10cm 칸).
  1) 본네트 회차별 TSDF(depth 5m 안)의 위 향한 바닥 정점 → 칸마다 회차별 중앙값 → 회차 사이 중앙값 (지나간 차·사람에 강함)
  2) 본네트가 못 본 칸은 지붕 TSDF 바닥을 쓰되, 겹치는 칸에서 잰 (지붕 − 본네트) 차이를 매끈하게 빼서 맞춤
  3) 빈 칸은 둘레에서 보간, 지도 밖(--margin m)은 전체 평면으로 부드럽게 수렴하도록 외삽 (지도 밖 바닥 확장용)
  4) 본네트 칸은 3x3 중앙값, 차로(본네트 궤적 2.5m 안)는 σ 0.2m 가우시안으로 다듬음
출력은 C의 recon Ground 형식과 같다 (grid[ix, iy], lo=(x0, y0), cell; 높이 = 칸 중심 기준 이중선형):
  floor_height.npz : grid(float32), lo, cell, conf(uint8: 3 본네트 2회차↑, 2 본네트 1회차, 1 지붕(맞춤), 0 보간·외삽), nsess, spread(회차 사이 높이 폭 m)
  floor_height.png
사용: floor_map_v05.py <out_dir> <pkg_dir> --bonnet <mesh>... --roof <mesh>... [--cell 0.1] [--margin 40]"""
import argparse, json
import numpy as np, open3d as o3d
from pathlib import Path
from scipy import ndimage
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('pkg'); ap.add_argument('--bonnet', nargs='+', required=True); ap.add_argument('--roof', nargs='*', default=[])
ap.add_argument('--cell', type=float, default=0.1); ap.add_argument('--margin', type=float, default=40.0); ap.add_argument('--band', type=float, default=0.4); a = ap.parse_args()
O, P = Path(a.out), Path(a.pkg); O.mkdir(parents=True, exist_ok=True); cell = a.cell

def interp(pts, vals):
    lin, near = LinearNDInterpolator(pts, vals), NearestNDInterpolator(pts, vals)
    def f(q):
        z = lin(q); b = np.isnan(z); z[b] = near(q[b]); return z
    return f
T = np.vstack([np.loadtxt(P / f'trajectory_bonnet_{d}_camera_link.txt', comments='#', ndmin=2)[:, 1:4] for d in ('cw', 'ccw')])
T = T[::3]; est = interp(T[:, :2], T[:, 2] - 0.97)            # 본네트 카메라 아래 0.97 m ≈ 바닥

def floor_verts(path, band, ref):
    m = o3d.io.read_triangle_mesh(str(path)); m.compute_vertex_normals()
    V, N = np.asarray(m.vertices), np.asarray(m.vertex_normals); k = N[:, 2] > 0.9; V = V[k]
    return V[np.abs(V[:, 2] - ref(V[:, :2])) < band]

Bv = [floor_verts(f, a.band, est) for f in a.bonnet]
allxy = np.vstack([v[:, :2] for v in Bv])
x0, y0 = allxy.min(0) - a.margin; x1, y1 = allxy.max(0) + a.margin
nx, ny = int(np.ceil((x1 - x0) / cell)), int(np.ceil((y1 - y0) / cell)); lo = np.array([x0, y0])
def cellmed(V):
    ij = np.floor((V[:, :2] - lo) / cell).astype(int); key = ij[:, 0] * ny + ij[:, 1]; o = np.lexsort((V[:, 2], key))
    keys, st, cnt = np.unique(key[o], return_index=True, return_counts=True); g = np.full(nx * ny, np.nan)
    ok = cnt >= 2; g[keys[ok]] = V[o, 2][st + cnt // 2][ok]; return g.reshape(nx, ny)
S = np.stack([cellmed(v) for v in Bv]); nsess = np.isfinite(S).sum(0)
with np.errstate(all='ignore'):
    HB = np.nanmedian(S, 0); spread = np.nanmax(S, 0) - np.nanmin(S, 0)
spread[nsess < 2] = np.nan
print(f'격자 {nx}x{ny} ({cell}m), 본네트 회차 {len(Bv)}개 | 바닥 칸 {np.isfinite(HB).sum():,} (2회차↑ {(nsess >= 2).sum():,}) | 회차 사이 폭 중앙 {np.nanmedian(spread)*100:.1f}cm, 90% {np.nanpercentile(spread, 90)*100:.1f}cm')
# 외딴 칸 다듬기: 3x3 이웃 중앙값과 6cm 넘게 다르면 버림
def nanmed3(G):
    Pd = np.pad(G, 1, constant_values=np.nan); st = np.stack([Pd[i:i + G.shape[0], j:j + G.shape[1]] for i in range(3) for j in range(3)])
    with np.errstate(all='ignore'): return np.nanmedian(st, 0)
def smooth_fill(G, sig):
    w = np.isfinite(G).astype(float); s = ndimage.gaussian_filter(np.nan_to_num(G), sig / cell); sw = ndimage.gaussian_filter(w, sig / cell)
    with np.errstate(all='ignore'): return s / sw, sw
M3 = nanmed3(HB)
bad = np.isfinite(HB) & (np.abs(HB - M3) > 0.06); HB[bad] = np.nan; print(f'외딴 칸 {bad.sum():,}개 버림')

# 큰 규모 면(σ 3m) — 지붕 바닥 띠 기준과 외삽 바탕
coarse, w = smooth_fill(HB, 3.0)
far = ~np.isfinite(coarse) | (w < 1e-3)
if far.any():
    ok = np.isfinite(HB); ij = np.argwhere(ok)[::50]; f = interp(ij.astype(float), HB[ok][::50]); coarse[far] = f(np.argwhere(far).astype(float))
gx = lo[0] + (np.arange(nx) + 0.5) * cell; gy = lo[1] + (np.arange(ny) + 0.5) * cell
def cref(xy):
    return ndimage.map_coordinates(coarse, [(xy[:, 0] - lo[0]) / cell - 0.5, (xy[:, 1] - lo[1]) / cell - 0.5], order=1, mode='nearest')

H = HB.copy(); conf = np.where(nsess >= 2, 3, np.where(nsess == 1, 2, 0)).astype(np.uint8); conf[~np.isfinite(HB)] = 0
rep = dict(cell=cell, bonnet_cells=int(np.isfinite(HB).sum()), spread_med_cm=float(np.nanmedian(spread) * 100))
if a.roof:
    R = np.stack([cellmed(floor_verts(f, 0.35, cref)) for f in a.roof])
    with np.errstate(all='ignore'): HR = np.nanmedian(R, 0)
    d = HR - HB; ov = np.isfinite(d)
    off, wo = smooth_fill(np.where(ov, d, np.nan), 3.0)
    off2, _ = smooth_fill(np.where(ov, d, np.nan), 12.0); off = np.where(wo > 0.05, off, off2); off = np.nan_to_num(off, nan=float(np.nanmedian(d)))
    HRc = HR - off; use = ~np.isfinite(HB) & np.isfinite(HRc)
    HRs = ndimage.median_filter(np.nan_to_num(HRc, nan=-99), size=5); HRs[HRs < -50] = np.nan
    H[use] = np.where(np.isfinite(HRs[use]), HRs[use], HRc[use]); conf[use] = 1
    print(f'지붕: 겹친 칸 {ov.sum():,}, 지붕−본네트 중앙 {np.nanmedian(d)*100:+.1f}cm (|차| 90% {np.nanpercentile(np.abs(d[ov]), 90)*100:.1f}cm) → 본네트 없는 칸 {use.sum():,} 채움')
    rep.update(roof_cells=int(use.sum()), roof_minus_bonnet_med_cm=float(np.nanmedian(d) * 100))

# 빈 칸: 전체 평면 + 잔차. 둘레가 잘 둘러싸인 구멍은 잔차를 보간, 그 밖(지도 밖)은 잔차가 평면으로 부드럽게 줄어들게
meas = np.isfinite(H)
A_ = np.c_[np.repeat(gx, ny), np.tile(gy, nx), np.ones(nx * ny)]; okp = (conf >= 2).ravel()
pc_, *_ = np.linalg.lstsq(A_[okp][::10], H.ravel()[okp][::10], rcond=None); PL = (A_ @ pc_).reshape(nx, ny)
Rr = H - PL; Gr = Rr.copy()
for sig in (0.3, 1.0, 2.0):
    f_, w_ = smooth_fill(np.where(np.isfinite(Gr), Gr, np.nan), sig); fill = ~np.isfinite(Gr) & (w_ > 0.3); Gr[fill] = f_[fill]
wf = np.isfinite(Gr).astype(float); s4 = 4.0 / cell
num = ndimage.gaussian_filter(np.nan_to_num(Gr), s4); den = ndimage.gaussian_filter(wf, s4)
ext_r = num / np.maximum(den, 0.5); Gr[~np.isfinite(Gr)] = ext_r[~np.isfinite(Gr)]
G = PL + Gr
print(f'전체 평면: z = {pc_[0]:+.4f}x {pc_[1]:+.4f}y {pc_[2]:+.3f} (기울기 {np.degrees(np.arctan(np.hypot(pc_[0], pc_[1]))):.2f}°)')
# 측정 칸은 원래 값 유지하되 2회차↑ 칸끼리 3x3 중앙값으로 다듬기 (C의 smoothed(3)와 같은 효과)
Gs = ndimage.median_filter(G, size=3); G = np.where(conf >= 2, Gs, G)
# 차로(본네트 궤적 2.5m 안) 노면은 σ 0.2m로 한 번 더 다듬음 — 10cm 칸 잡음 제거 (차로에 연석 같은 계단은 없음)
from scipy.spatial import cKDTree
lane = (cKDTree(T[:, :2]).query(np.c_[np.repeat(gx, ny), np.tile(gy, nx)], distance_upper_bound=2.5)[0] <= 2.5).reshape(nx, ny)
G = np.where(lane, ndimage.gaussian_filter(G, 0.2 / cell), G); rep['lane_smoothed_cells'] = int(lane.sum())
np.savez_compressed(O / 'floor_height.npz', grid=G.astype(np.float32), lo=lo, cell=cell, conf=conf, nsess=nsess.astype(np.uint8), spread=np.nan_to_num(spread, nan=-1).astype(np.float32))
rep.update(shape=[nx, ny], lo=lo.tolist(), measured=int(meas.sum()), conf_counts={int(c): int((conf == c).sum()) for c in range(4)})
json.dump(rep, open(O / 'floor_height.json', 'w'), indent=1); print(json.dumps(rep, ensure_ascii=False))

# 그림: 큰 규모 면 대비 높이, 신뢰도
D = G - PL
fig, ax = plt.subplots(1, 2, figsize=(22, 10)); ext = [x0, x0 + nx * cell, y0, y0 + ny * cell]
im = ax[0].imshow(D.T, origin='lower', extent=ext, cmap='RdBu_r', vmin=-0.3, vmax=0.3); plt.colorbar(im, ax=ax[0], shrink=0.7); ax[0].set_title('floor height minus best plane (m)')
ax[1].imshow(conf.T, origin='lower', extent=ext, cmap='viridis', vmin=0, vmax=3); ax[1].set_title('conf: 3 bonnet>=2 sessions, 2 bonnet 1, 1 roof (aligned), 0 interpolated/extrapolated')
for a_ in ax: a_.plot(T[:, 0], T[:, 1], 'k.', ms=0.5); a_.set_aspect('equal')
plt.tight_layout(); plt.savefig(O / 'floor_height.png', dpi=80); print('→', O / 'floor_height.npz')
