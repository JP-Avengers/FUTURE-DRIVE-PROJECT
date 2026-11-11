"""둘레 실험층 이음새 맞추기: 손 촬영(nr3 배치)의 연석 면·흰 선을 v0.5의 연석 면·흰 선에 맞추는 구역별 평행 이동.
- 특징: 연석 면(|nz|<0.6, 바닥 위 3~35cm), 흰 선(밝음, 수평, 바닥 ±8/15cm). v0.5는 cloud_3cm, 손 촬영은 nr3 TSDF 꼭짓점.
- 구역: 주차장 중심 기준 15° 부채꼴. 구역마다 이동 d(±3.5m, 10cm) 격자 탐색, 점수 = 손 촬영 특징점의 v0.5 특징까지 거리(0.6m에서 자름) 평균.
  최고 점수 +1cm 안 후보 중 이동이 가장 작은 것 선택(긴 연석 방향의 미끄러짐 억제). 점수 25% 이상 좋아질 때만 채택.
- 검증점(소화전·파란 안내판·남쪽 연석·남서 둥근 연석)이 있는 구역에서 이동이 0.5m를 넘으면 채택하지 않음(검증된 배치를 우선).
- 구역 이동을 각도 방향으로 부드럽게 이어 꼭짓점마다 적용.
사용: seam_snap.py <v05_cloud> <hand_tsdf.ply> <perimeter.ply> <floor.npz> <out_dir>"""
import sys, json, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial import cKDTree
VC, HT, PM, FL, O = sys.argv[1:6]; O = Path(O); O.mkdir(parents=True, exist_ok=True)
CEN = np.array([-22.0, -3.0]); SEC = 15; C = 0.1
F = np.load(FL); G, lo, c = F['grid'].astype(float), F['lo'], float(F['cell'])
fh = lambda xy: ndimage.map_coordinates(G, [(xy[:, 0] - lo[0]) / c - 0.5, (xy[:, 1] - lo[1]) / c - 0.5], order=1, mode='nearest')
def feats(X, N, Col, hand):
    h = X[:, 2] - fh(X); nz = np.abs(N[:, 2]); br = Col.mean(1)
    curb = (nz < 0.6) & (h > 0.03) & (h < 0.35)
    paint = (br > (0.75 if hand else 0.72)) & (nz > 0.85) & (np.abs(h) < (0.15 if hand else 0.08))
    return X[curb, :2], X[paint, :2]
pc = o3d.io.read_point_cloud(VC); pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=20))
vc, vp = feats(np.asarray(pc.points), np.asarray(pc.normals), np.asarray(pc.colors), False)
hm = o3d.io.read_triangle_mesh(HT); hm.compute_vertex_normals()
hc, hp = feats(np.asarray(hm.vertices), np.asarray(hm.vertex_normals), np.asarray(hm.vertex_colors), True)
print(f'특징: v0.5 연석 {len(vc):,} 흰 선 {len(vp):,} | 손 촬영 연석 {len(hc):,} 흰 선 {len(hp):,}')
allv = np.vstack([vc, vp]); glo = allv.min(0) - 6; n = ((allv.max(0) + 6 - glo) / C).astype(int) + 1
def dmap(P):
    m = np.ones(n, bool); ij = ((P - glo) / C).astype(int); ok = (ij >= 0).all(1) & (ij < n).all(1); m[ij[ok, 0], ij[ok, 1]] = False
    return ndimage.distance_transform_edt(m) * C
Dc, Dp = dmap(vc), dmap(vp)
def look(D, P):
    ij = ((P - glo) / C).astype(int); ok = (ij >= 0).all(1) & (ij < n).all(1); out = np.full(len(P), 0.6); out[ok] = np.minimum(D[ij[ok, 0], ij[ok, 1]], 0.6); return out
# 손 촬영 특징 중 v0.5 특징 4m 안(이음새 부근)만
hc = hc[look(np.minimum(Dc, Dp), hc) < 0.6] if False else hc[(lambda P: (lambda ij, ok: np.where(ok, np.minimum(Dc, Dp)[np.clip(ij[:, 0], 0, n[0] - 1), np.clip(ij[:, 1], 0, n[1] - 1)], 99))(((P - glo) / C).astype(int), ((((P - glo) / C).astype(int) >= 0).all(1) & (((P - glo) / C).astype(int) < n).all(1))))(hc) < 4]
hp = hp[(lambda P: (lambda ij, ok: np.where(ok, np.minimum(Dc, Dp)[np.clip(ij[:, 0], 0, n[0] - 1), np.clip(ij[:, 1], 0, n[1] - 1)], 99))(((P - glo) / C).astype(int), ((((P - glo) / C).astype(int) >= 0).all(1) & (((P - glo) / C).astype(int) < n).all(1))))(hp) < 4]
ang = lambda P: (np.degrees(np.arctan2(P[:, 1] - CEN[1], P[:, 0] - CEN[0])) + 360) % 360
VER = {'hydrant': (-29.74, -24.51), 'bluesign': (-38.9, -22.3), 'curbS': (-41.0, -20.6), 'arc': (-45.7, -18.0)}
ver_sec = {int(ang(np.array([p]))[0] // SEC) for p in VER.values()}
ds = np.arange(-3.5, 3.51, 0.1); DX, DY = np.meshgrid(ds, ds, indexing='ij'); cand = np.c_[DX.ravel(), DY.ravel()]
rng = np.random.default_rng(0); res = []
for k in range(360 // SEC):
    selc = hc[(ang(hc) // SEC) == k]; selp = hp[(ang(hp) // SEC) == k]
    if len(selc) + len(selp) < 300: res.append(dict(sec=k, n=int(len(selc) + len(selp)), d=[0, 0], used=False, note='특징 부족')); continue
    if len(selc) > 2500: selc = selc[rng.choice(len(selc), 2500, replace=False)]
    if len(selp) > 2500: selp = selp[rng.choice(len(selp), 2500, replace=False)]
    sc = np.array([(look(Dc, selc + d).sum() * 2 + look(Dp, selp + d).sum()) / (2 * len(selc) + len(selp)) for d in cand])
    s0 = sc[np.argmin(np.abs(cand).sum(1))]; b = sc.min(); near = cand[sc <= b + 0.01]; d = near[np.argmin(np.linalg.norm(near, axis=1))]
    spread = float(np.linalg.norm(near - near.mean(0), axis=1).max())
    used = (b < 0.75 * s0); note = ''
    if used and k in ver_sec and np.linalg.norm(d) > 0.5: used = False; note = '검증 구역(이동 0.5m 초과 → 기각)'
    res.append(dict(sec=k, n=int(len(selc) + len(selp)), d=[round(float(d[0]), 2), round(float(d[1]), 2)], score0=round(float(s0), 3), score=round(float(b), 3), spread=round(spread, 2), used=bool(used), note=note))
    print(f"구역 {k * SEC:3d}~{k * SEC + SEC:3d}° 특징 {res[-1]['n']:5d} | 이동 ({d[0]:+.1f},{d[1]:+.1f}) 점수 {s0:.3f}→{b:.3f} 후보 퍼짐 {spread:.1f}m {'채택' if used else '기각'} {note}")
D = np.array([r['d'] if r['used'] else [0, 0] for r in res], float); U = np.array([r['used'] for r in res])
# 기각 구역은 이웃 채택 구역 값으로 메우지 않음(0) — 단, 양옆이 모두 채택이면 평균
D2 = D.copy()
for k in range(len(D)):
    if not U[k] and U[k - 1] and U[(k + 1) % len(D)] and res[k].get('note', '') == '': D2[k] = (D[k - 1] + D[(k + 1) % len(D)]) / 2
def shift_at(P):
    a = ang(P) / SEC - 0.5; k0 = np.floor(a).astype(int) % len(D2); k1 = (k0 + 1) % len(D2); t = (a - np.floor(a))[:, None]
    return D2[k0] * (1 - t) + D2[k1] * t
m = o3d.io.read_triangle_mesh(PM); V = np.asarray(m.vertices).copy(); sh = shift_at(V[:, :2]); V[:, :2] += sh
m.vertices = o3d.utility.Vector3dVector(V); m.compute_vertex_normals(); o3d.io.write_triangle_mesh(str(O / 'perimeter_snapped.ply'), m)
for nm, p in VER.items(): print(f'  검증점 {nm}: 이음새 이동 {np.linalg.norm(shift_at(np.array([p]))[0]):.2f}m')
print(f'꼭짓점 이동량 중앙 {np.median(np.linalg.norm(sh, axis=1)):.2f}m, 95% {np.percentile(np.linalg.norm(sh, axis=1), 95):.2f}m')
json.dump(dict(sectors=res, center=CEN.tolist(), sec_deg=SEC), open(O / 'seam_snap.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
# 위에서 본 대조: v0.5 특징(흰색=흰 선, 하늘=연석) + 손 촬영 특징 이동 전(빨강)·후(초록)
X0, Y0, Wd, Hd = -62, -36, 820, 640; img = np.zeros((Hd, Wd, 3), np.uint8)
def dot(P, col):
    u = ((P[:, 0] - X0) / C).astype(int); v = Hd - 1 - ((P[:, 1] - Y0) / C).astype(int); ok = (u >= 0) & (u < Wd) & (v >= 0) & (v < Hd); img[v[ok], u[ok]] = col
dot(vp, (255, 255, 255)); dot(vc, (255, 200, 0)); hh = np.vstack([hc, hp]); dot(hh, (0, 0, 255)); dot(hh + shift_at(hh), (0, 255, 0))
for xg in range(-60, 21, 10): cv2.putText(img, str(xg), (int((xg - X0) / C) + 2, 12), 0, 0.4, (200, 200, 200), 1)
for yg in range(-30, 27, 10): cv2.putText(img, str(yg), (2, Hd - 1 - int((yg - Y0) / C)), 0, 0.4, (200, 200, 200), 1)
cv2.putText(img, 'white/cyan=v0.5 paint/curb  red=hand before  green=hand after', (100, Hd - 8), 0, 0.45, (255, 255, 255), 1)
cv2.imwrite(str(O / 'seam_snap_top.png'), img)
