"""남의 좌표계 메시(Polycam GLB/PLY/OBJ 등)를 우리 건물 좌표(x 복도 축, 바닥 z=0, Z 위)로 옮기기.
 (1) 표면을 넓이 비례로 고르게 뽑은 점과 면 법선으로 맨해튼 세 축을 찾음
 (2) 위 방향 후보 6개(세 축 × 부호) 각각: 가장 낮은 수평면 봉우리를 바닥으로 잡아 평면 맞춤으로 수평 · z=0,
     바닥 위 0.3~1.9 m 벽 점을 5 cm 격자로 찍어 기준 맵 벽 격자(거리장)와 90° 회전 4가지 × 이동(FFT 상관)으로 맞춤
 (3) 점수가 가장 높은 후보에서 요를 ±2° 미세 탐색 → 벽+바닥 점끼리 point-to-plane ICP(천장 · 잡동사니 제외)
 (4) 확인값: 크기 비(남/우리) 두 가지 — x 10 m 창마다 앞뒤 어긋남 기울기, 크기 허용 ICP — 와 벽 거리 잔차(중앙, 5 cm 안 비율)
     + 단면 폭 비(참고). 크기는 --apply-scale · --scale-value 를 줄 때만 적용(기본은 강체만)
 출력: <out>/<name>_reg.ply(건물 좌표 메시), <name>_reg.json(4x4 변환 · 후보 점수 · 크기 · 잔차), <name>_reg.png(위에서 본 벽 겹침)
사용: mc_register.py <foreign.glb|ply|obj> <out_dir> [--name polycam] [--ref <기준 메시, 기본 v0.3>] [--up auto|+x|-x|+y|-y|+z|-z (파일 축)] [--apply-scale (x 창 크기 비 적용)] [--scale-value 1.012 (크기 비 남/우리 를 직접 주고 적용)] [--npts 600000]"""
import sys, os, json, time, numpy as np, open3d as o3d, cv2
from pathlib import Path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mc_lib import load_mesh, save_ply, tri_normals
from scipy import ndimage, signal
from scipy.spatial import cKDTree

a = sys.argv; opt = lambda k, d: a[a.index(k) + 1] if k in a else d
SRC, OUT = a[1], Path(a[2]); OUT.mkdir(parents=True, exist_ok=True); NAME = opt('--name', Path(SRC).stem)
REF = os.path.expanduser(opt('--ref', '~/b_slam/deliver/corridor_v0.3/geometry_tsdf_2cm.ply')); UPOPT = opt('--up', 'auto'); NPTS = int(opt('--npts', 600000))
CELL, ZLO, ZHI, SIG = 0.05, 0.3, 1.9, 0.06
CACHE = Path.home() / 'b_slam/agents/compare/cache'; CACHE.mkdir(parents=True, exist_ok=True)
rng = np.random.default_rng(0); t0 = time.time(); log = lambda *s: print(f'[{time.time() - t0:5.0f}s]', *s, flush=True)


def surface_samples(V, T, n, seed=0, pca=False):
    """넓이 비례 표면 점 + 면 법선(Polycam 처럼 평면을 큰 삼각형으로 줄인 메시도 고르게).
    pca=True 면 법선을 반경 8 cm 이웃 점의 PCA 로 다시 구함(잔 삼각형 잡음에 강함, 부호는 면 법선 쪽)."""
    r = np.random.default_rng(seed); N, A = tri_normals(V, T); p = A / A.sum(); i = r.choice(len(T), n, p=p); u = r.random((n, 2)); s = u.sum(1) > 1; u[s] = 1 - u[s]
    t = T[i]; P = V[t[:, 0]] + u[:, :1] * (V[t[:, 1]] - V[t[:, 0]]) + u[:, 1:] * (V[t[:, 2]] - V[t[:, 0]]); Ni = N[i]
    if pca:
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P)); pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.08, max_nn=30))
        Np = np.asarray(pc.normals).copy(); Np *= np.where((Np * Ni).sum(1) < 0, -1.0, 1.0)[:, None]; Ni = Np
    return P, Ni, float(A.sum())


def ref_data():
    f = CACHE / f'ref2_{Path(REF).parent.name}_{Path(REF).stem}_{os.path.getsize(REF)}.npz'
    if f.exists(): return dict(np.load(f))
    log('기준 메시 읽기', REF); V, T, _ = load_mesh(REF); P, N, _ = surface_samples(V, T, 1500000, 1); del V, T
    w = (P[:, 2] > ZLO) & (P[:, 2] < ZHI) & (np.abs(N[:, 2]) < 0.3); W, WN = P[w], N[w]
    sel = ((P[:, 2] > 0.1) & (P[:, 2] < 2.5) & (np.abs(N[:, 2]) < 0.3)) | ((np.abs(P[:, 2]) < 0.1) & (np.abs(N[:, 2]) > 0.9))
    d = dict(wall=W, wall_n=WN, icp_p=P[sel], icp_n=N[sel]); np.savez(f, **d); return d


def manhattan(N, w=None):
    """면 법선(부호 무시)의 가장 빽빽한 방향 d1, 그에 수직인 평면에서 d2, d3=d1×d2."""
    k = np.arange(4000) + 0.5; th = np.arccos(1 - k / 4000); ph = np.pi * (1 + 5 ** 0.5) * k; G = np.c_[np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)]
    S = N[rng.choice(len(N), min(len(N), 80000), replace=False)]; c4 = np.cos(np.radians(4))
    def refine(d, perp=None):
        for _ in range(4):
            m = np.abs(S @ d) > np.cos(np.radians(8)); X = S[m] * np.sign(S[m] @ d)[:, None]; d = X.mean(0)
            if perp is not None: d -= (d @ perp) * perp
            d /= np.linalg.norm(d)
        return d
    dens = np.concatenate([(np.abs(S @ G[j:j + 500].T) > c4).sum(0) for j in range(0, len(G), 500)]); d1 = refine(G[np.argmax(dens)])
    b1 = np.cross(d1, [1, 0, 0] if abs(d1[0]) < 0.9 else [0, 1, 0]); b1 /= np.linalg.norm(b1); b2 = np.cross(d1, b1); an = np.radians(np.arange(0, 180, 0.5))
    C = np.outer(np.cos(an), b1) + np.outer(np.sin(an), b2); d2 = refine(C[np.argmax((np.abs(S @ C.T) > c4).sum(0))], d1); d3 = np.cross(d1, d2)
    frac = [float(np.mean(np.abs(S @ d) > np.cos(np.radians(8)))) for d in (d1, d2, d3)]
    return np.array([d1, d2, d3]), frac


def rz(th):
    c, s = np.cos(th), np.sin(th); M = np.eye(4); M[:2, :2] = [[c, -s], [s, c]]; return M


def tr(v): M = np.eye(4); M[:3, 3] = v; return M


def rot4(R): M = np.eye(4); M[:3, :3] = R; return M


def apply(M, P): return P @ M[:3, :3].T + M[:3, 3]


def applyN(M, N): R = M[:3, :3] / np.cbrt(np.linalg.det(M[:3, :3])); return N @ R.T


# ---------------------------------------------------------------- 기준 벽 거리장
R = ref_data(); RW = R['wall']; rmin = RW[:, :2].min(0) - 1.0; rmax = RW[:, :2].max(0) + 1.0; shp = np.ceil((rmax - rmin) / CELL).astype(int) + 1
occ = np.zeros(shp, bool); ij = ((RW[:, :2] - rmin) / CELL).astype(int); occ[ij[:, 0], ij[:, 1]] = True
SF = np.exp(-(ndimage.distance_transform_edt(~occ) * CELL) ** 2 / (2 * SIG ** 2)).astype(np.float32); NOCC = float(occ.sum())
RTREE = cKDTree(RW); log(f'기준 벽 점 {len(RW):,}, 격자 {shp}')


def match_plan(Pw, ths):
    """벽 점(평면 좌표)을 각 회전 th 로 돌려 기준 거리장과 FFT 상관 → (점수, th, 이동 xy) 최고값."""
    best = (-1, 0, None)
    for th in ths:
        c, s = np.cos(th), np.sin(th); Q = Pw @ np.array([[c, s], [-s, c]]); fmin = Q.min(0); jj = ((Q - fmin) / CELL).astype(int)
        F = np.zeros(jj.max(0) + 1, np.float32); F[jj[:, 0], jj[:, 1]] = 1; nF = F.sum()
        if nF < 200: continue
        cor = signal.fftconvolve(SF, F[::-1, ::-1], mode='full'); k = np.unravel_index(np.argmax(cor), cor.shape)
        sc = np.sqrt(cor[k] / nF * min(cor[k] / NOCC, 1.0))       # 정밀도(남의 벽 칸 중 기준 벽 근처) × 재현율(기준 벽 칸 중 설명된 몫)의 기하평균
        if sc > best[0]: best = (float(sc), th, rmin + (np.array(k) - (np.array(F.shape) - 1)) * CELL - fmin)
    return best


# ---------------------------------------------------------------- 남의 메시
V, T, C = load_mesh(SRC); log(f'{Path(SRC).name}: 꼭짓점 {len(V):,} 삼각형 {len(T):,}, 범위 {V.min(0).round(2)} ~ {V.max(0).round(2)}')
P, N, AREA = surface_samples(V, T, NPTS, pca=True); AX, frac = manhattan(N); log('맨해튼 축(파일 좌표)', AX.round(3).tolist(), '넓이 비율', np.round(frac, 3).tolist())
FILEAX = {'+x': [1, 0, 0], '-x': [-1, 0, 0], '+y': [0, 1, 0], '-y': [0, -1, 0], '+z': [0, 0, 1], '-z': [0, 0, -1]}
cands = []
for k in range(3):
    for sg in (1, -1):
        u = sg * AX[k]
        if UPOPT != 'auto' and u @ FILEAX[UPOPT] < np.cos(np.radians(20)): continue
        cands.append((k, sg, u))
hyp = []
for k, sg, u in cands:
    h1 = AX[(k + 1) % 3]; h2 = np.cross(u, h1); M0 = rot4(np.array([h1, h2, u]))     # u → +z
    Q = apply(M0, P); NQ = applyN(M0, N); hz = np.abs(NQ[:, 2]) > np.cos(np.radians(10))
    hb = np.arange(Q[:, 2].min() - 0.05, Q[:, 2].max() + 0.05, 0.02); hh, _ = np.histogram(Q[hz, 2], hb); hh = ndimage.uniform_filter1d(hh.astype(float), 3)
    pk = [j for j in range(1, len(hh) - 1) if hh[j] >= hh[j - 1] and hh[j] >= hh[j + 1] and hh[j] >= 0.25 * hh.max()]
    f0 = (hb[pk[0]] + hb[pk[0] + 1]) / 2; fl = Q[hz & (np.abs(Q[:, 2] - f0) < 0.04)]
    c = fl.mean(0); nf = np.linalg.svd(fl - c, full_matrices=False)[2][2]; nf *= np.sign(nf[2])
    ax_ = np.cross(nf, [0, 0, 1]); ang = np.arcsin(min(np.linalg.norm(ax_), 1.0)); Rt = cv2.Rodrigues(ax_ / max(np.linalg.norm(ax_), 1e-12) * ang)[0] if ang > 1e-9 else np.eye(3)
    M1 = tr([0, 0, -(Rt @ c)[2]]) @ rot4(Rt) @ M0; Q = apply(M1, P); NQ = applyN(M1, N)
    up_n = float(np.mean(NQ[hz & (np.abs(Q[:, 2]) < 0.04), 2] > 0))           # 바닥 면 법선이 위를 향한 비율(법선 방향이 일관된 메시면 1에 가까움)
    W = (Q[:, 2] > ZLO) & (Q[:, 2] < ZHI) & (np.abs(NQ[:, 2]) < 0.3)
    sc, th, txy = match_plan(Q[W, :2], [j * np.pi / 2 for j in range(4)])
    hyp.append(dict(axis=k, sign=sg, up_file=u.round(4).tolist(), floor_tilt_deg=float(np.degrees(ang)), floor_normal_up=up_n, walls=int(W.sum()), score=sc, th=th, txy=None if txy is None else txy.tolist(), M1=M1))
    log(f'위 후보 축{k}{"+-"[sg < 0]} (파일 {u.round(2)}): 바닥 기울기 {np.degrees(ang):.2f}°, 바닥 법선 위 {up_n:.0%}, 벽 점 {W.sum():,}, 맞춤 점수 {sc:.3f}, 회전 {np.degrees(th):.0f}°')
hyp.sort(key=lambda h: -h['score']); best = hyp[0]
note = ''
if len(hyp) > 1 and hyp[0]['score'] - hyp[1]['score'] < 0.05:
    note = f'위 후보 1·2위 점수 차 {hyp[0]["score"] - hyp[1]["score"]:.3f} < 0.05 — 바닥 법선 위 비율로 골랐음, 그림으로 확인 필요'
    if hyp[1]['floor_normal_up'] - hyp[0]['floor_normal_up'] > 0.3: best = hyp[1]
    log('주의:', note)
M1 = best['M1']; Q = apply(M1, P); NQ = applyN(M1, N); W = (Q[:, 2] > ZLO) & (Q[:, 2] < ZHI) & (np.abs(NQ[:, 2]) < 0.3)
fine = best['th'] + np.radians(np.arange(-2, 2.01, 0.25)); sc, th, txy = match_plan(Q[W, :2], fine)
Minit = tr([txy[0], txy[1], 0]) @ rz(th) @ M1; log(f'미세 요 {np.degrees(th - best["th"]):+.2f}°, 점수 {sc:.3f}, 이동 {np.round(txy, 2)}')

# ---------------------------------------------------------------- ICP (벽 + 바닥)
def icp_sets(Q, NQ):
    s = ((Q[:, 2] > 0.1) & (Q[:, 2] < 2.5) & (np.abs(NQ[:, 2]) < 0.3)) | ((np.abs(Q[:, 2]) < 0.1) & (np.abs(NQ[:, 2]) > 0.9)); return s
tgt = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(R['icp_p'])); tgt.normals = o3d.utility.Vector3dVector(R['icp_n']); tgt = tgt.voxel_down_sample(0.03)


def run_icp(Minit, dists=(0.4, 0.2, 0.1, 0.05)):
    M = Minit.copy()
    for dmax in dists:
        Q = apply(M, P); NQ = applyN(M, N); src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Q[icp_sets(Q, NQ)])).voxel_down_sample(0.04)
        r = o3d.pipelines.registration.registration_icp(src, tgt, dmax, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPlane(o3d.pipelines.registration.TukeyLoss(k=dmax / 2)),
                                                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
        M = r.transformation @ M
    return M, src, r


Mr, src, r = run_icp(Minit); log(f'ICP: fitness {r.fitness:.3f}, inlier rmse {r.inlier_rmse * 100:.1f} cm')


def scale_checks(M):
    Q = apply(M, P); NQ = applyN(M, N); src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Q[icp_sets(Q, NQ)])).voxel_down_sample(0.04)
    rs = o3d.pipelines.registration.registration_icp(src, tgt, 0.08, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=True), o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    s_pp = 1 / np.cbrt(np.linalg.det(rs.transformation[:3, :3]))
    # x 창마다 앞뒤 어긋남: 2 cm 거리장
    c2 = 0.02; o2 = rmin; sh2 = np.ceil((rmax - rmin) / c2).astype(int) + 1; oc2 = np.zeros(sh2, bool); ij = ((RW[:, :2] - o2) / c2).astype(int); oc2[ij[:, 0], ij[:, 1]] = True
    D2 = ndimage.distance_transform_edt(~oc2) * c2; S2 = np.exp(-D2 ** 2 / (2 * 0.04 ** 2))
    Wq = Q[(Q[:, 2] > ZLO) & (Q[:, 2] < ZHI) & (np.abs(NQ[:, 2]) < 0.3), :2]; dxs = np.arange(-0.8, 0.801, 0.02); rows = []
    for xc in np.arange(np.floor(RW[:, 0].min()) + 5, RW[:, 0].max() - 4, 2.5):
        q = Wq[np.abs(Wq[:, 0] - xc) < 5]
        if len(q) < 2000: continue
        q = q[rng.choice(len(q), min(len(q), 20000), replace=False)]; sc = []
        for dx in dxs:
            jj = np.floor((q + [dx, 0] - o2) / c2).astype(int); ok = (jj >= 0).all(1) & (jj < sh2).all(1); sc.append(S2[jj[ok, 0], jj[ok, 1]].sum() / len(q))
        sc = np.array(sc); b = int(np.argmax(sc)); dx = dxs[b]
        if 0 < b < len(sc) - 1:
            den = sc[b - 1] - 2 * sc[b] + sc[b + 1]; dx += 0.5 * c2 * (sc[b - 1] - sc[b + 1]) / den if den < 0 else 0
        rows.append(dict(xc=float(xc), dx=float(dx), peak=float(sc[b]), contrast=float(sc[b] - np.median(sc))))
    # 봉우리가 탐색 끝(±0.8 m)에 붙었거나 대비가 약한 창은 버리고, 남은 창에 RANSAC 직선(허용 6 cm) → 기울기 b 로 크기 비 1 - b
    ok = [x for x in rows if x['contrast'] > 0.08 and x['peak'] > 0.3 and abs(x['dx']) < 0.78]; s_x = None; resid = None; nin = 0
    if len(ok) >= 4:
        X = np.array([[x['xc'], x['dx']] for x in ok]); best = (0, None)
        for i in range(len(X)):
            for j in range(i + 1, len(X)):
                if X[j, 0] - X[i, 0] < 10: continue
                b_ = (X[j, 1] - X[i, 1]) / (X[j, 0] - X[i, 0]); inl = np.abs(X[:, 1] - (X[i, 1] + b_ * (X[:, 0] - X[i, 0]))) < 0.06
                if inl.sum() > best[0]: best = (int(inl.sum()), inl)
        if best[0] >= 4 and np.ptp(X[best[1], 0]) >= 15:
            Xi = X[best[1]]; b_, a_ = np.polyfit(Xi[:, 0], Xi[:, 1], 1); s_x = float(1 - b_); resid = float(np.sqrt(np.mean((Xi[:, 1] - (b_ * Xi[:, 0] + a_)) ** 2))); nin = best[0]
    # 단면 폭 비(in_width 방식: 높이 0.4~1.4 m 벽 점 y 봉우리 간격) — 1% 는 2.6 cm 라 벽면 재구성 방식 차이와 섞임, 참고만
    def width(y):
        h, e = np.histogram(y, bins=np.arange(-2.2, 2.6, 0.01)); cc = (e[:-1] + e[1:]) / 2; h = np.convolve(h, np.ones(3) / 3, 'same'); lo = cc < 0
        i, j = int(np.argmax(np.where(lo, h, 0))), int(np.argmax(np.where(~lo, h, 0))); sub = lambda k: float((cc[max(k - 4, 0):k + 5] * h[max(k - 4, 0):k + 5]).sum() / max(h[max(k - 4, 0):k + 5].sum(), 1e-9))
        return sub(j) - sub(i)
    sel = lambda X: X[(X[:, 2] > 0.4) & (X[:, 2] < 1.4) & (np.abs(X[:, 1]) < 2.6) & (X[:, 0] > -2) & (X[:, 0] < 48), 1]
    Wf = Q[(np.abs(NQ[:, 2]) < 0.3)]; wf, wr = width(sel(Wf)), width(sel(RW))
    return dict(scale_icp=float(s_pp), scale_x=s_x, x_windows=rows, x_used=nin, x_cand=len(ok), x_fit_rms_cm=None if resid is None else resid * 100, width_foreign=wf, width_ref=wr, scale_width=wf / wr)


def residual(M):
    Q = apply(M, P); NQ = applyN(M, N); Wq = Q[(Q[:, 2] > ZLO) & (Q[:, 2] < ZHI) & (np.abs(NQ[:, 2]) < 0.3)]
    q = Wq[rng.choice(len(Wq), min(len(Wq), 100000), replace=False)]; d0, j = RTREE.query(q, distance_upper_bound=0.5); ins = np.isfinite(d0)
    d = np.full(len(q), np.inf); d[ins] = np.abs(((q[ins] - RW[j[ins]]) * R['wall_n'][j[ins]]).sum(1))      # 가장 가까운 기준 벽 점의 면까지 거리
    fz = Q[(np.abs(Q[:, 2]) < 0.15) & (np.abs(NQ[:, 2]) > 0.95), 2]
    return dict(wall_med_cm=float(np.median(d[ins]) * 100), wall_in5=float(np.mean(d[ins] < 0.05)), wall_outside_50cm=float(1 - ins.mean()), floor_z_med_cm=float(np.median(fz) * 100) if len(fz) else None), Wq


chk = scale_checks(Mr); res, Wq = residual(Mr)
fs = lambda v, f='{:.4f}': '정하지 못함' if v is None else f.format(v)
log(f'크기 비(남/우리): x 창 기울기 {fs(chk["scale_x"])} (쓸 만한 창 {chk["x_cand"]}개 중 직선에 맞은 {chk["x_used"]}개, 직선 잔차 {fs(chk["x_fit_rms_cm"], "{:.1f}")} cm) | 크기 허용 ICP {chk["scale_icp"]:.4f} (1 쪽으로 치우침) | 단면 폭 비 {chk["scale_width"]:.4f} ({chk["width_foreign"]:.3f} / {chk["width_ref"]:.3f} m)')
log(f'벽 거리 잔차 중앙 {res["wall_med_cm"]:.1f} cm, 5 cm 안 {res["wall_in5"]:.0%}, 기준 벽 0.5 m 밖 {res["wall_outside_50cm"]:.0%}, 바닥 z 중앙 {res["floor_z_med_cm"]} cm')
Mfinal = Mr; applied = None
s = float(opt('--scale-value', 0)) or chk['scale_x']
if '--apply-scale' in a and s is None: log('크기 비를 x 창으로 정하지 못해 적용하지 않음(--scale-value 로 직접 줄 수 있음)')
if ('--apply-scale' in a or '--scale-value' in a) and s:
    c = apply(Mr, P)[icp_sets(apply(Mr, P), applyN(Mr, N))].mean(0)
    Ms = tr(c) @ np.diag([1 / s, 1 / s, 1 / s, 1]) @ tr(-c); Mfinal, src, r = run_icp(Ms @ Mr, (0.1, 0.05)); applied = s
    chk2 = scale_checks(Mfinal); res2, Wq = residual(Mfinal)
    log(f'크기 {1 / s:.4f} 배 적용 후: 벽 잔차 중앙 {res2["wall_med_cm"]:.1f} cm, 5 cm 안 {res2["wall_in5"]:.0%}, 남은 크기 비 x {fs(chk2["scale_x"])} · ICP {chk2["scale_icp"]:.4f}')
else: chk2 = res2 = None

# ---------------------------------------------------------------- 저장
Vo = apply(Mfinal, V); save_ply(OUT / f'{NAME}_reg.ply', Vo, T, C)
js = dict(src=os.path.abspath(SRC), ref=REF, name=NAME, area_m2=AREA, manhattan_axes_file=AX.tolist(), manhattan_area_frac=frac, up_option=UPOPT,
          hypotheses=[{k: v for k, v in h.items() if k != 'M1'} for h in hyp], chosen_up_file=best['up_file'], note=note, fine_yaw_score=sc,
          T_foreign_to_building=Mfinal.tolist(), T_rigid=Mr.tolist(), scale_applied=applied, icp_fitness=r.fitness, icp_rmse_cm=r.inlier_rmse * 100,
          scale_check=chk, residual=res, scale_check_after=chk2, residual_after=res2)
(OUT / f'{NAME}_reg.json').write_text(json.dumps(js, indent=1, ensure_ascii=False))
c = 0.05; img = np.full(shp[::-1].tolist() + [3], 255, np.uint8); img[occ.T] = (150, 150, 150)
jj = ((Wq[:, :2] - rmin) / c).astype(int); ok = (jj >= 0).all(1) & (jj < shp).all(1); img[jj[ok, 1], jj[ok, 0]] = (0, 0, 230); img = img[::-1].copy()
for xg in range(int(np.ceil(rmin[0] / 5) * 5), int(rmax[0]), 5): cv2.putText(img, f'{xg}', (int((xg - rmin[0]) / c), 14), 0, 0.45, (200, 100, 0), 1)
cv2.putText(img, f'{NAME}: red = registered walls, gray = reference walls ({Path(REF).parent.name})', (6, img.shape[0] - 8), 0, 0.5, (0, 0, 0), 1)
cv2.imwrite(str(OUT / f'{NAME}_reg.png'), img); log('저장', OUT / f'{NAME}_reg.ply')
