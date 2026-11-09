"""바닥·장애물 점검 (D 기준 경로 기준): 형상 메시 바닥 높이 지도(0.25m 칸), 바닥 겹침, 바닥 출처(본네트/지붕),
경로 위 솟은 덩어리·떠 있는 덩어리, 점군의 외딴 점. 시뮬레이터 언덕·걸림이 B 맵에서 온 것인지 가르는 용도.
사용: floor_route_check.py <pkg_dir> <route_csv> <bonnet_mesh> <out_dir>"""
import sys, json
import numpy as np, open3d as o3d
from pathlib import Path
from scipy.spatial import cKDTree
from scipy import ndimage
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager
_kf = [f.name for f in font_manager.fontManager.ttflist if any(k in f.name for k in ('Nanum', 'Noto Sans CJK', 'Noto Sans KR', 'Malgun', 'Unifont'))]
if _kf: plt.rcParams['font.family'] = _kf[0]
plt.rcParams['axes.unicode_minus'] = False

P, RC, BM, O = map(Path, sys.argv[1:5]); O.mkdir(parents=True, exist_ok=True)
CELL = 0.25
R = np.loadtxt(RC, delimiter=',', skiprows=1); R = R[R[:, 0] == 0]; rs, rxy = R[:, 1], R[:, 2:4]
rt = cKDTree(rxy)

# 바닥 높이 추정: 가장 가까운 궤적 점의 카메라 높이 - 장착 높이 (본네트 약 0.97m, 지붕 약 1.55m)
T, off = [], []
for name, h in (('bonnet_cw', 0.97), ('bonnet_ccw', 0.97), ('roof_cw', 1.55), ('roof_ccw', 1.55)):
    t = np.loadtxt(P / f'trajectory_{name}_camera_link.txt', comments='#', ndmin=2)[:, 1:4]; T.append(t); off.append(np.full(len(t), h))
T = np.vstack(T); off = np.concatenate(off); tt = cKDTree(T[:, :2])
d_rt, _ = tt.query(rxy)
print(f'기준 경로 {len(rxy)}점, 길이 {rs[-1]:.1f}m | 경로→가장 가까운 촬영 궤적 거리 중앙 {np.median(d_rt):.2f}m, 최대 {d_rt.max():.2f}m')

def tris(path):
    m = o3d.io.read_triangle_mesh(str(path)); m.compute_triangle_normals()
    V = np.asarray(m.vertices); F = np.asarray(m.triangles); N = np.asarray(m.triangle_normals)
    C = (V[F[:, 0]] + V[F[:, 1]] + V[F[:, 2]]) / 3
    A = 0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
    _, j = tt.query(C[:, :2]); fe = T[j, 2] - off[j]
    return C, A, np.abs(N[:, 2]), C[:, 2] - fe

C, A, NZ, HE = tris(P / 'geometry_rawdepth_tsdf_5cm.ply')
fl = (NZ > 0.9) & (np.abs(HE) < 0.45)
x0, y0 = C[fl, 0].min() - 1, C[fl, 1].min() - 1; nx = int((C[fl, 0].max() + 1 - x0) / CELL) + 1; ny = int((C[fl, 1].max() + 1 - y0) / CELL) + 1
def cid(xy): ix = ((xy[:, 0] - x0) / CELL).astype(int); iy = ((xy[:, 1] - y0) / CELL).astype(int); ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny); return np.where(ok, iy * nx + ix, -1)

# 칸별 바닥: 면적 가중 중앙값 + 층(6cm 넘게 떨어진 높이 묶음) 수·간격
c = cid(C[fl, :2]); z = C[fl, 2]; a = A[fl]; o = np.lexsort((z, c)); c, z, a = c[o], z[o], a[o]
H = np.full(nx * ny, np.nan); NL = np.zeros(nx * ny, int); SEP = np.zeros(nx * ny); AREA = np.zeros(nx * ny)
st = np.r_[0, np.flatnonzero(np.diff(c)) + 1]; en = np.r_[st[1:], len(c)]
for s, e in zip(st, en):
    k = c[s]; zz, aa = z[s:e], a[s:e]; tot = aa.sum(); AREA[k] = tot
    H[k] = zz[np.searchsorted(np.cumsum(aa), tot / 2)]
    b = np.r_[0, np.flatnonzero(np.diff(zz) > 0.06) + 1, len(zz)]
    la = np.add.reduceat(aa, b[:-1]); lz = np.array([np.median(zz[b[i]:b[i + 1]]) for i in range(len(b) - 1)])
    sig = (la >= 0.15 * tot) & (la >= 0.004); NL[k] = sig.sum()
    if sig.sum() >= 2: SEP[k] = lz[sig].max() - lz[sig].min()
cov = AREA >= 0.008
H[~cov] = np.nan; HG = H.reshape(ny, nx)
# 큰 규모 바닥(배수 경사 등)을 가우시안(σ 3m)으로 → 그보다 작은 규모 출렁임 = 언덕·패임
w = np.isfinite(HG).astype(float); g = lambda X: ndimage.gaussian_filter(X, 3.0 / CELL, mode='constant')
HS = g(np.nan_to_num(HG)) / np.maximum(g(w), 1e-6); BUMP = HG - HS

# 바닥 출처: 본네트만으로 합친 메시에 바닥이 있는 칸
Cb, Ab, NZb, HEb = tris(BM); fb = (NZb > 0.9) & (np.abs(HEb) < 0.45)
BA = np.bincount(cid(Cb[fb, :2])[cid(Cb[fb, :2]) >= 0], Ab[fb][cid(Cb[fb, :2]) >= 0], nx * ny); BON = (BA >= 0.008).reshape(ny, nx)
del Cb, Ab, NZb, HEb

# 칸 중심과 경로 거리
gx, gy = np.meshgrid(x0 + (np.arange(nx) + 0.5) * CELL, y0 + (np.arange(ny) + 0.5) * CELL)
dR, jR = rt.query(np.c_[gx.ravel(), gy.ravel()]); dR = dR.reshape(ny, nx); sR = rs[jR].reshape(ny, nx)
COV = np.isfinite(HG); NLg = NL.reshape(ny, nx); SEPg = SEP.reshape(ny, nx)

def band(m, name):
    k = m & COV; n = k.sum(); b = np.abs(BUMP[k])
    r = dict(name=name, cells=int(m.sum()), floor_cov=round(float(n / max(m.sum(), 1)), 3), bonnet=round(float((k & BON).sum() / max(n, 1)), 3),
             multilayer=round(float((k & (NLg >= 2)).sum() / max(n, 1)), 3), sep_med_cm=round(float(np.median(SEPg[k & (NLg >= 2)]) * 100), 1) if (k & (NLg >= 2)).any() else 0,
             bump_gt5=round(float((b > 0.05).mean()), 3), bump_gt10=round(float((b > 0.10).mean()), 3), bump_p95_cm=round(float(np.percentile(b, 95) * 100), 1), bump_max_cm=round(float(b.max() * 100), 1))
    print(f"{name:16s}: 칸 {r['cells']:5d} | 바닥 있음 {r['floor_cov']:.1%} | 본네트 바닥 {r['bonnet']:.1%} | 바닥 겹침 {r['multilayer']:.1%} (간격 중앙 {r['sep_med_cm']}cm) | 출렁임 >5cm {r['bump_gt5']:.1%}, >10cm {r['bump_gt10']:.1%}, 95% {r['bump_p95_cm']}cm, 최대 {r['bump_max_cm']}cm")
    return r
S = dict(route=band(dR <= 1.0, '경로 ±1m'), corridor=band(dR <= 2.35, '경로 ±2.35m'), all=band(np.ones_like(COV), '지도 전체'))
for nm, m in (('본네트 바닥 칸', BON), ('지붕만 바닥 칸', ~BON)):
    k = m & COV & (dR <= 2.35); b = np.abs(BUMP[k]); print(f'  경로 ±2.35m 안 {nm}: {k.sum()}칸, 출렁임 >5cm {(b > 0.05).mean():.1%}, 95% {np.percentile(b, 95)*100:.1f}cm' if k.any() else f'  {nm}: 없음')

# 경로 단면: 바닥 높이·피치·롤 (로봇 폭 ±0.35m)
def hat(xy):
    k = cid(xy); v = np.full(len(xy), np.nan); ok = k >= 0; v[ok] = HG.ravel()[k[ok]]; return v
tan = np.gradient(rxy, axis=0); tan /= np.linalg.norm(tan, axis=1, keepdims=True); nrm = np.c_[-tan[:, 1], tan[:, 0]]
hc = hat(rxy); hl = hat(rxy + 0.35 * nrm); hr = hat(rxy - 0.35 * nrm)
hcs = np.array([np.nanmedian(HG.ravel()[cid(rxy[max(i-2,0):i+3])]) for i in range(len(rxy))])
pitch = np.degrees(np.arctan(np.gradient(hcs, rs))); roll = np.degrees(np.arctan((hl - hr) / 0.7))
bc = BUMP.ravel()[np.clip(cid(rxy), 0, None)]; bonr = BON.ravel()[np.clip(cid(rxy), 0, None)]
print(f'경로 위: 바닥 없음 {np.isnan(hc).mean():.1%}, 본네트 바닥 {np.mean(bonr):.1%} | 피치 |95%| {np.nanpercentile(np.abs(pitch),95):.1f}°, 최대 {np.nanmax(np.abs(pitch)):.1f}° | 롤 |95%| {np.nanpercentile(np.abs(roll),95):.1f}°, 최대 {np.nanmax(np.abs(roll)):.1f}°')

# 경로 ±1m 안 솟은 덩어리(바닥 위 6~50cm)와 떠 있는 덩어리(바닥 위 50cm~2.5m인데 아래 6~50cm가 비어 있음)
ka = cid(C[:, :2]); okk = ka >= 0; hf = np.full(len(C), np.nan); hf[okk] = C[okk, 2] - HG.ravel()[ka[okk]]
near = okk & (dR.ravel()[np.clip(ka, 0, None)] <= 1.0)
lo = near & (hf > 0.06) & (hf < 0.5); hi = near & (hf >= 0.5) & (hf < 2.5)
LO = np.zeros(nx * ny); np.maximum.at(LO, ka[lo], hf[lo]); LOA = np.bincount(ka[lo], A[lo], nx * ny)
HIA = np.bincount(ka[hi], A[hi], nx * ny); HIZ = np.full(nx * ny, np.inf); np.minimum.at(HIZ, ka[hi], hf[hi])
def comps(mask, name, val):
    lab, n = ndimage.label(mask.reshape(ny, nx), structure=np.ones((3, 3))); out = []
    for i in range(1, n + 1):
        k = np.flatnonzero(lab.ravel() == i); j = k[np.argmax(val[k])]
        out.append(dict(x=round(float(gx.ravel()[j]), 2), y=round(float(gy.ravel()[j]), 2), s=round(float(sR.ravel()[j]), 1), cells=int(len(k)), h_cm=round(float(val[j] * 100), 1),
                        bonnet=bool(BON.ravel()[k].any()), d_route=round(float(dR.ravel()[j]), 2)))
    out.sort(key=lambda r: -r['h_cm']); print(f'{name}: {len(out)}곳' + ''.join(f"\n  ({r['x']:+.1f}, {r['y']:+.1f}) 경로 {r['s']}m, 거리 {r['d_route']}m, 높이 {r['h_cm']}cm, {r['cells']}칸, {'본네트' if r['bonnet'] else '지붕만'}" for r in out[:12])); return out
bumps_on = comps((LO >= 0.06) & (LOA >= 0.003), '경로 ±1m 안 바닥 위 6~50cm 덩어리', LO)
lowsupport = ndimage.binary_dilation(((LOA >= 0.003)).reshape(ny, nx), np.ones((5, 5))).ravel()
flo = (HIA >= 0.003) & ~lowsupport
floats = comps(flo, '경로 ±1m 안 떠 있는 덩어리(아래 받침 없음, 바닥 위 0.5~2.5m)', np.where(np.isfinite(HIZ), HIZ, 0))

# 점군 외딴 점: 경로 ±1m, 바닥 위 0.3m 넘고 반경 0.5m 안 이웃 10개 미만
pc = np.asarray(o3d.io.read_point_cloud(str(P / 'cloud_3cm.ply')).points); kp = cid(pc[:, :2]); okp = kp >= 0
hp = np.full(len(pc), np.nan); hp[okp] = pc[okp, 2] - HG.ravel()[kp[okp]]
sel = okp & (dR.ravel()[np.clip(kp, 0, None)] <= 1.0) & (hp > 0.3) & (hp < 3)
cnt = cKDTree(pc).query_ball_point(pc[sel], 0.5, return_length=True); iso = (cnt < 10).sum()
print(f'점군: 경로 ±1m·바닥 위 0.3~3m 점 {sel.sum():,}개 중 외딴 점(0.5m 안 이웃 10개 미만) {iso:,}개')

# D가 적은 걸림·멈춤 자리
known = {'52.5m 걸림 (15cm 덩어리)': (-33.65, -6.65), '서쪽 끝 납작한 조각': (-36.8, -7.35), '첫 판 멈춤 12.4m': (-2.94, -17.18), '첫 판 멈춤 75m': (-14.15, 1.37)}
KN = {}
for nm, (x, y) in known.items():
    k = np.flatnonzero((np.hypot(gx.ravel() - x, gy.ravel() - y) <= 0.6)); kf = k[np.isfinite(HG.ravel()[k])]
    r = dict(bonnet=bool(BON.ravel()[k].any()), layers=int(NLg.ravel()[k].max()), sep_cm=round(float(SEPg.ravel()[k].max() * 100), 1),
             bump_cm=round(float(np.nanmax(np.abs(BUMP.ravel()[kf])) * 100), 1) if len(kf) else None, low_obj_cm=round(float(LO[k].max() * 100), 1))
    KN[nm] = r; print(f"  {nm} ({x}, {y}): 바닥 층 최대 {r['layers']} (간격 {r['sep_cm']}cm), 출렁임 최대 {r['bump_cm']}cm, 바닥 위 덩어리 {r['low_obj_cm']}cm, {'본네트' if r['bonnet'] else '지붕만'} 바닥")

# 그림
ext = [x0, x0 + nx * CELL, y0, y0 + ny * CELL]
fig, ax = plt.subplots(1, 2, figsize=(22, 10))
im = ax[0].imshow(BUMP, origin='lower', extent=ext, cmap='RdBu_r', vmin=-0.15, vmax=0.15); plt.colorbar(im, ax=ax[0], shrink=0.7, label='바닥 출렁임 (m, σ3m 매끈한 면 대비)')
ax[0].set_title('바닥 출렁임 + 경로 위 덩어리(△ 6~50cm, × 떠 있음) + D 걸림 자리(★)')
SRC = np.full((ny, nx, 3), 1.0); SRC[COV & BON] = (0.55, 0.8, 0.55); SRC[COV & ~BON] = (0.98, 0.7, 0.35); SRC[COV & (NLg >= 2) & (SEPg > 0.1)] = (0.85, 0.2, 0.85)
ax[1].imshow(SRC, origin='lower', extent=ext); ax[1].set_title('바닥 출처: 초록 본네트 / 주황 지붕만 / 보라 바닥 두 겹(간격 10cm 넘음)')
for a_ in ax:
    a_.plot(rxy[:, 0], rxy[:, 1], 'k-', lw=1.2)
    for i in range(0, len(rs), 40): a_.text(rxy[i, 0], rxy[i, 1], f'{rs[i]:.0f}', fontsize=8)
    for r in bumps_on: a_.plot(r['x'], r['y'], '^', ms=6 + min(r['h_cm'], 40) / 4, mfc='none', mec='k')
    for r in floats: a_.plot(r['x'], r['y'], 'x', ms=8, c='m')
    for nm, (x, y) in known.items(): a_.plot(x, y, '*', ms=14, c='gold', mec='k')
    a_.set_aspect('equal'); a_.set_xlabel('x (m)'); a_.set_ylabel('y (m)')
plt.tight_layout(); plt.savefig(O / 'floor_map.png', dpi=90); plt.close()
fig, ax = plt.subplots(3, 1, figsize=(16, 9), sharex=True)
ax[0].plot(rs, bc * 100, 'b-'); ax[0].axhline(5, c='r', ls=':'); ax[0].axhline(-5, c='r', ls=':'); ax[0].set_ylabel('출렁임 cm')
ax[1].plot(rs, pitch, 'g-', label='피치'); ax[1].plot(rs, roll, 'm-', label='롤 (±0.35m)'); ax[1].legend(); ax[1].set_ylabel('도')
ax[2].plot(rs, SEPg.ravel()[np.clip(cid(rxy), 0, None)] * 100, 'k-'); ax[2].set_ylabel('바닥 겹침 간격 cm'); ax[2].set_xlabel('기준 경로 s (m)')
for a_ in ax:
    for i in np.flatnonzero(~bonr): a_.axvspan(rs[i] - 0.125, rs[i] + 0.125, color='orange', alpha=0.25, lw=0)
plt.tight_layout(); plt.savefig(O / 'route_profile.png', dpi=90); plt.close()
json.dump(dict(summary=S, bumps_on_route=bumps_on, floating=floats, isolated_points=int(iso), known=KN), open(O / 'summary.json', 'w'), ensure_ascii=False, indent=1)
print('→', O)
