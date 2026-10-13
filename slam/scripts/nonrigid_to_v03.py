#!/usr/bin/env python3
"""10/9 새 맵(gn03·gm03)을 v0.3 좌표계(고정)에 비강체 정합. ccw_nonrigid.py(run20→run18)를 일반화.
 1) 초기값: exports/align_<run>/T_src_to_ref.npy (주차선 2D 강체 + 바닥 평면, align_paint2d.py·align_height.py)
 2) 앵커: 새 맵 키프레임마다 국소 정합
    - 높이·기울기: v0.3 바닥(map_v0.3/cloud_3cm.ply, v0.3 궤적 기준 바닥 높이)과 새 맵 바닥의 국소 평면 (반경 10m)
    - 수평 위치·방향: 주차선 2D ICP (반경 8m; 기준 exports/v03_paintref, 새 맵 exports/<run>_paintkf)
 3) 자세 최적화: 앵커(절대 자세) + 같은 map_id(DB) 안 키프레임 간 상대 자세(이웃 1·2·4) (soft-L1)
 4) 검증(강체 vs 비강체): 구역별 주차선 잔차, 동쪽 줄 칸막이 위치(줄 방향 어긋남), 바닥 높이 차
출력: exports/<run>nr/ (traj_camera_id_camera_poses.txt, traj_optimized_poses.txt, paint_xy.npy, nonrigid_report.txt)
사용: nonrigid_to_v03.py <run> [GLO,GHI = 새 맵 카메라-바닥 높이 범위, 기본 -1.25,-0.85] [--refcore N] [--out 이름]   (~/tsdf_env)
  --refcore N: 기준 주차선 중 10cm 칸에 N점 이상 모인 곳만 사용(원거리 depth로 번진 부채꼴 제거). 기본 0 = 전부
  --reflocal F: 기준 주차선 중 주변 3m 안 가장 촘촘한 10cm 칸의 F배 이상인 칸만 사용(부채꼴 제거, 관측 적은 구역 보존)
  --refextra 파일.npy: 기준 주차선에 더할 2D 점(v0.3 좌표, 다른 새 맵의 정합 결과)
  --capture M: 수평 앵커 ICP를 M m 대응 거리에서 시작(기본 1). 강체 초기값이 멀리 어긋난 구역용
  --ref_floor F.npy / --ref_paint P.npy: 기준을 v0.3 대신 다른 맵(같은 좌표계)의 바닥 점(Nx3)·주차선(Nx2)으로
  --init T.npy: 초기 강체 변환(기본 exports/align_<run>/T_src_to_ref.npy)
  --out: 출력 폴더 이름(기본 <run>nr)"""
import re, sys, sqlite3
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks

def opt(name, default):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default
pos = [a for i, a in enumerate(sys.argv[1:], 1) if not a.startswith('--') and not sys.argv[i - 1].startswith('--')]
run = pos[0]; GLO, GHI = map(float, (pos[1] if len(pos) > 1 else '-1.25,-0.85').split(','))
REFCORE = int(opt('--refcore', 0))
CAPTURE = float(opt('--capture', 1.0))  # 수평 앵커 ICP 첫 대응 거리(m). 1보다 크면 그 거리→절반→1m→… 로 좁히고, 허용 이동도 CAPTURE+0.5m
H = Path.home() / 'b_slam/exports'; V03 = Path.home() / 'b_slam/deliver/map_v0.3'
OUT = H / opt('--out', f'{run}nr'); OUT.mkdir(exist_ok=True)
RAD, RADH = 10.0, 8.0
REG = {'A': (-50, -29, -25, -14), 'B': (-34, -17, 3, 14), 'C': (-4, 7, -16, 11)}
EAST = (0, 6, -14, 6)  # 동쪽 줄 (v0.3 좌표, align_height.py와 같음)
rng = np.random.default_rng(0)
log = open(OUT / 'nonrigid_report.txt', 'w')
def say(*a):
    s = ' '.join(str(x) for x in a); print(s, flush=True); log.write(s + '\n')
def tf(T, X): return X @ T[:3, :3].T + T[:3, 3]
def near2(tree, c, r): return tree.query_ball_point(c, r)

# 기준(v0.3): 바닥 점 + 주차선 2D
pc = o3d.io.read_point_cloud(str(V03 / 'cloud_3cm.ply')); X0 = np.asarray(pc.points)
TR = np.vstack([np.loadtxt(V03 / f, comments='#', ndmin=2)[:, 1:4] for f in ('trajectory_camera_link.txt', 'trajectory_ccw_camera_link.txt')])
_, j = cKDTree(TR[:, :2]).query(X0[:, :2]); h = X0[:, 2] - TR[j, 2]
G0 = X0[(h > -1.85) & (h < -1.25)]; P0 = np.load(H / 'v03_paintref/paint_xy.npy')
if '--ref_floor' in sys.argv: G0 = np.load(opt('--ref_floor', '')); say(f'기준 바닥 바꿈: {opt("--ref_floor", "")}')   # 다른 기준 맵(같은 좌표계)의 바닥 점 Nx3
if '--ref_paint' in sys.argv: P0 = np.load(opt('--ref_paint', '')); say(f'기준 주차선 바꿈: {opt("--ref_paint", "")}')  # 다른 기준 맵의 주차선 Nx2
if REFCORE:
    cc = np.floor(P0 / 0.1).astype(np.int64); _, inv, cnt = np.unique(cc[:, 0] * 1000003 + cc[:, 1], return_inverse=True, return_counts=True)
    n_all = len(P0); P0 = P0[cnt[inv] >= REFCORE]; say(f'기준 주차선 고밀도 중심부만(10cm 칸 {REFCORE}점 이상): {n_all:,} → {len(P0):,}')
REFLOCAL = float(opt('--reflocal', 0))
if REFLOCAL:  # 주변 3m 안 가장 촘촘한 칸의 REFLOCAL배 이상인 칸만 (관측이 적은 구역의 주차선은 남김)
    from scipy.ndimage import maximum_filter
    lo_ = P0.min(0); ij = np.floor((P0 - lo_) / 0.1).astype(int); grid = np.zeros(ij.max(0) + 1, np.int32); np.add.at(grid, (ij[:, 0], ij[:, 1]), 1)
    lm = maximum_filter(grid, size=31); keep = grid[ij[:, 0], ij[:, 1]] >= np.maximum(REFLOCAL * lm[ij[:, 0], ij[:, 1]], 3)
    n_all = len(P0); P0 = P0[keep]; say(f'기준 주차선 국소 고밀도만(주변 3m 최고 칸의 {REFLOCAL:g}배 이상): {n_all:,} → {len(P0):,}')
    np.save(OUT / 'ref_paint_used.npy', P0)
if '--refextra' in sys.argv:  # 추가 기준 주차선(이미 v0.3 좌표에 맞춘 다른 새 맵, 예: exports/gn03nr3/paint_xy.npy)
    X_ = np.load(opt('--refextra', '')); n_all = len(P0); P0 = np.vstack([P0, X_]); say(f'추가 기준 주차선 {opt("--refextra", "")}: {n_all:,} + {len(X_):,}')
tG0, tG0z, tP0 = cKDTree(G0[:, :2]), None, cKDTree(P0)
# 새 맵: 바닥 점 + 주차선 2D (z는 가장 가까운 궤적의 바닥 높이로 채움)
pc = o3d.io.read_point_cloud(str(H / run / 'cloud_cloud.ply')); X1 = np.asarray(pc.points)
tro = np.loadtxt(H / run / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:4]; tt = cKDTree(tro[:, :2])
_, j = tt.query(X1[:, :2]); h = X1[:, 2] - tro[j, 2]; G1 = X1[(h > GLO) & (h < GHI)]
p1 = np.load(H / f'{run}_paintkf/paint_xy.npy'); _, j = tt.query(p1); P1 = np.c_[p1, tro[j, 2] + (GLO + GHI) / 2]
say(f'v0.3: 바닥 {len(G0):,}, 주차선 {len(P0):,} | {run}: 바닥 {len(G1):,}, 주차선 {len(P1):,}')

# 1) 초기 강체
Tg = np.load(opt('--init', str(H / f'align_{run}/T_src_to_ref.npy')))  # --init <4x4.npy>: 다른 초기 강체 변환
say(f'초기 강체: yaw {np.degrees(np.arctan2(Tg[1,0], Tg[0,0])):.2f}°, t=({Tg[0,3]:.2f},{Tg[1,3]:.2f},{Tg[2,3]:.2f})')

# 키프레임 자세(광학, ID순)와 map_id
tr = np.loadtxt(H / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); tr = tr[np.argsort(tr[:, 8])]
N = len(tr); Pc = np.zeros((N, 4, 4))
for k, r in enumerate(tr):
    Pc[k] = np.eye(4); Pc[k][:3, :3] = Rot.from_quat(r[4:8]).as_matrix(); Pc[k][:3, 3] = r[1:4]
mid = dict(sqlite3.connect(Path.home() / f'maps/{run}.db').execute('select id, map_id from Node').fetchall())
sess = np.array([mid[int(i)] for i in tr[:, 8]])
say(f'키프레임 {N}개, map_id {len(set(sess))}개 ' + str({int(m): int((sess == m).sum()) for m in sorted(set(sess))}))
P_init = np.array([Tg @ x for x in Pc])

# 2) 앵커
def plane(X):
    A = np.c_[X[:, 0], X[:, 1], np.ones(len(X))]; c = np.linalg.lstsq(A, X[:, 2], rcond=None)[0]
    r = X[:, 2] - A @ c; k = np.abs(r) < 3 * np.median(np.abs(r)) + 0.02
    return np.linalg.lstsq(A[k], X[k, 2], rcond=None)[0]
G1w, P1w = tf(Tg, G1), tf(Tg, P1); tG1w, tP1w = cKDTree(G1w[:, :2]), cKDTree(P1w[:, :2])
vert, horz = {}, {}
for k in range(N):
    c = P_init[k][:3, 3][:2]
    i0, i1 = near2(tG0, c, RAD), near2(tG1w, c, RAD)
    if len(i0) >= 500 and len(i1) >= 500:
        g0, g1 = G0[i0], G1w[i1]
        if len(g0) > 40000: g0 = g0[rng.choice(len(g0), 40000, replace=False)]
        if len(g1) > 40000: g1 = g1[rng.choice(len(g1), 40000, replace=False)]
        a0, a1 = plane(g0), plane(g1)
        n0 = np.array([-a0[0], -a0[1], 1]); n0 /= np.linalg.norm(n0); n1 = np.array([-a1[0], -a1[1], 1]); n1 /= np.linalg.norm(n1)
        ax = np.cross(n1, n0); s_ = np.linalg.norm(ax); Rt = np.eye(3) if s_ < 1e-9 else Rot.from_rotvec(ax / s_ * np.arcsin(min(s_, 1))).as_matrix()
        z1 = a1 @ [c[0], c[1], 1]; dz = (a0 @ [c[0], c[1], 1]) - z1
        Ct = np.eye(4); Ct[:3, :3] = Rt; Ct[:3, 3] = np.array([c[0], c[1], z1]) - Rt @ np.array([c[0], c[1], z1]); Ct[2, 3] += dz
        vert[k] = Ct @ P_init[k]
    F0 = P0[near2(tP0, c, RADH)]; F1 = P1w[near2(tP1w, c, RADH)][:, :2]
    if len(F0) < 150 or len(F1) < 150: continue
    if len(F0) > 20000: F0 = F0[rng.choice(len(F0), 20000, replace=False)]
    if len(F1) > 8000: F1 = F1[rng.choice(len(F1), 8000, replace=False)]
    t0 = cKDTree(F0); R = np.eye(2); t = np.zeros(2); ok = True
    for thr in ([CAPTURE] * 10 + [CAPTURE / 2] * 10 if CAPTURE > 1 else []) + [1.0] * 10 + [0.6] * 10 + [0.3] * 15:
        X = F1 @ R.T + t; dd, jj = t0.query(X); mm = dd < thr
        if mm.sum() < 60: ok = False; break
        A_, B_ = F1[mm], F0[jj[mm]]; ca, cb = A_.mean(0), B_.mean(0)
        U, _, Vt = np.linalg.svd((A_ - ca).T @ (B_ - cb)); Rn = (U @ Vt).T
        if np.linalg.det(Rn) < 0: Vt[1] *= -1; Rn = (U @ Vt).T
        R, t = Rn, cb - Rn @ ca
    if not ok: continue
    X = F1 @ R.T + t; frac = (t0.query(X)[0] < 0.2).mean()
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0])); shift = np.linalg.norm(R @ c + t - c)
    if frac < 0.5 or abs(yaw) > 4 or shift > max(1.5, CAPTURE + 0.5): continue
    Cj = np.eye(4); Cj[:2, :2] = R; Cj[:2, 3] = t
    horz[k] = (Cj @ P_init[k], frac)
say(f'높이 앵커 {len(vert)}/{N}개, 수평 앵커(주차선) {len(horz)}/{N}개 (일치율 중앙 {np.median([v[1] for v in horz.values()])*100:.0f}%)')
corr = np.array([np.linalg.norm(horz[k][0][:2, 3] - P_init[k][:2, 3]) for k in horz])
say(f'수평 앵커가 요구하는 이동: 중앙 {np.median(corr)*100:.0f}cm, 90% {np.percentile(corr,90)*100:.0f}cm, 최대 {corr.max()*100:.0f}cm')

# 3) 자세 최적화 (ccw_nonrigid.py와 같은 가중치)
def to_vec(T): return np.r_[Rot.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]
def to_mat(v): T = np.eye(4); T[:3, :3] = Rot.from_rotvec(v[:3]).as_matrix(); T[:3, 3] = v[3:]; return T
def err(Ta, Tb, st, sr):
    D = np.linalg.inv(Ta) @ Tb; return np.r_[Rot.from_matrix(D[:3, :3]).as_rotvec() / sr, D[:3, 3] / st]
def werr(M, P): return np.r_[Rot.from_matrix(P[:3, :3] @ M[:3, :3].T).as_rotvec(), P[:3, 3] - M[:3, 3]]
rel = []
for m in sorted(set(sess)):
    ks = np.nonzero(sess == m)[0]
    for a in range(len(ks)):
        for s in (1, 2, 4):
            if a + s < len(ks): rel.append((ks[a], ks[a + s], np.linalg.inv(Pc[ks[a]]) @ Pc[ks[a + s]]))
ST_R, SR_R, ST_A, SR_A = float(opt('--st_r', 0.03)), np.radians(0.3), 0.08, np.radians(0.5)  # --st_r: 키프레임 간 상대 이동 허용(m)
say(f'가중치: 상대 이동 {ST_R*100:g}cm, 앵커 수평 {ST_A*100:g}cm')
vk, hk = sorted(vert), sorted(horz)
def resid(x):
    Ps = [to_mat(x[6 * k:6 * k + 6]) for k in range(N)]; r = []
    for a, b, Z in rel: r.append(err(Z, np.linalg.inv(Ps[a]) @ Ps[b], ST_R, SR_R))
    for k in vk: e = werr(vert[k], Ps[k]); r.append(np.r_[e[0] / SR_A, e[1] / SR_A, e[5] / ST_A])
    for k in hk: e = werr(horz[k][0], Ps[k]); r.append(np.r_[e[2] / SR_A, e[3] / ST_A, e[4] / ST_A])
    for k in range(N): e = werr(P_init[k], Ps[k]); r.append(np.r_[e[2] / np.radians(5), e[3], e[4]])  # 약한 수평 유지(1m)
    return np.concatenate(r)
x0 = np.concatenate([to_vec(T) for T in P_init])
nres = 6 * len(rel) + 3 * (len(vk) + len(hk) + N); J = lil_matrix((nres, 6 * N), dtype=int); row = 0
for a, b, _ in rel: J[row:row + 6, 6 * a:6 * a + 6] = 1; J[row:row + 6, 6 * b:6 * b + 6] = 1; row += 6
for k in vk + hk + list(range(N)): J[row:row + 3, 6 * k:6 * k + 6] = 1; row += 3
sol = least_squares(resid, x0, jac_sparsity=J, loss='soft_l1', f_scale=1.0, max_nfev=40, x_scale='jac')
P_new = np.array([to_mat(sol.x[6 * k:6 * k + 6]) for k in range(N)])
mv = np.linalg.norm(P_new[:, :3, 3] - P_init[:, :3, 3], axis=1)
say(f'최적화: {sol.nfev}회, 비용 {sol.cost:.1f}, 자세 이동 중앙 {np.median(mv)*100:.0f}cm, 90% {np.percentile(mv,90)*100:.0f}cm, 최대 {mv.max()*100:.0f}cm')
ra = np.array([np.linalg.norm(werr(horz[k][0], P_new[k])[3:5]) for k in hk]); say(f'남은 수평 앵커 차이 중앙 {np.median(ra)*100:.1f}cm (90% {np.percentile(ra,90)*100:.1f}cm)')
rz = np.array([abs(werr(vert[k], P_new[k])[5]) for k in vk]); say(f'남은 높이 앵커 차이 중앙 {np.median(rz)*100:.1f}cm')
rr = np.array([np.linalg.norm((np.linalg.inv(Z) @ np.linalg.inv(P_new[a]) @ P_new[b])[:3, 3]) for a, b, Z in rel])
say(f'상대 자세 변형 중앙 {np.median(rr)*100:.2f}cm (90% {np.percentile(rr,90)*100:.1f}cm, 최대 {rr.max()*100:.1f}cm)')

# 저장 (광학 + camera_link)
lt = (H / run / 'keyframes/kf_calib' / f'{int(tr[0, 8])}.yaml').read_text()
L = np.eye(4); L[:3, :] = np.array([float(v) for v in re.search(r'local_transform:.*?data: \[([^\]]+)\]', lt, re.S).group(1).replace('\n', ' ').split(',')]).reshape(3, 4)
order = np.argsort(tr[:, 0])
with open(OUT / 'traj_camera_id_camera_poses.txt', 'w') as f, open(OUT / 'traj_optimized_poses.txt', 'w') as g:
    f.write('#timestamp x y z qx qy qz qw id\n'); g.write('#timestamp x y z qx qy qz qw\n')
    for k in order:
        q = Rot.from_matrix(P_new[k][:3, :3]).as_quat(); x = P_new[k][:3, 3]
        f.write(f'{tr[k,0]:.6f} {x[0]:.6f} {x[1]:.6f} {x[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} {int(tr[k,8])}\n')
        B = P_new[k] @ np.linalg.inv(L); q = Rot.from_matrix(B[:3, :3]).as_quat(); x = B[:3, 3]
        g.write(f'{tr[k,0]:.6f} {x[0]:.6f} {x[1]:.6f} {x[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n')
np.save(OUT / 'pose_delta.npy', np.einsum('nij,njk->nik', P_new, np.linalg.inv(Pc)))  # 키프레임별 (새 자세)·(원래 자세)^-1, 점 옮기기용
np.save(OUT / 'kf_xy_src.npy', Pc[:, :2, 3])
np.savez(OUT / 'anchors.npz', hk=np.array(hk), hreq=np.array([horz[k][0][:2, 3] for k in hk]).reshape(-1, 2), hfrac=np.array([horz[k][1] for k in hk]),
         init_xy=P_init[:, :2, 3], new_xy=P_new[:, :2, 3])  # 진단용

# 4) 검증: 새 맵 점을 가장 가까운 키프레임(원래 좌표 수평 기준)의 자세 변화로 옮김
tc = cKDTree(Pc[:, :2, 3])
def warp(X, Pfin):
    _, i = tc.query(X[:, :2]); D = np.einsum('nij,njk->nik', Pfin, np.linalg.inv(Pc))[i]
    return np.einsum('nij,nj->ni', D[:, :3, :3], X) + D[:, :3, 3]
def dividers(Q):
    """동쪽 줄 칸막이 위치: 줄 방향(±10° 탐색)으로 투영한 주차선 밀도의 봉우리"""
    s = (Q[:, 0] > EAST[0]) & (Q[:, 0] < EAST[1]) & (Q[:, 1] > EAST[2]) & (Q[:, 1] < EAST[3]); Q = Q[s]
    best = None
    for a in np.radians(np.arange(-10, 10.01, 0.5)):
        u = Q @ [np.sin(a), np.cos(a)]; hgm, e = np.histogram(u, np.arange(EAST[2] - 3, EAST[3] + 3, 0.05)); hs = gaussian_filter1d(hgm.astype(float), 1.5)
        sc = (hs ** 2).sum() / max(hs.sum(), 1) ** 2
        if best is None or sc > best[0]: best = (sc, a, hs, e)
    _, a, hs, e = best
    pk, _ = find_peaks(hs, distance=int(1.5 / 0.05), prominence=hs.max() * 0.15)
    return a, (e[pk] + 0.025)
ref_a, ref_pk = dividers(P0)
say(f'v0.3 동쪽 줄: 칸막이 {len(ref_pk)}개, 간격 중앙 {np.median(np.diff(ref_pk)):.2f}m, 방향 {np.degrees(ref_a):+.1f}°')
np.save(OUT / 'paint_xy.npy', warp(P1, P_new)[:, :2])
for name, Pf in (('강체(초기)', P_init), ('비강체(최적화)', P_new)):
    pw = warp(P1, Pf); gw = warp(G1, Pf); dd = tP0.query(pw[:, :2])[0]
    s = []
    for kk, r in REG.items():
        mk = (pw[:, 0] > r[0]) & (pw[:, 0] < r[1]) & (pw[:, 1] > r[2]) & (pw[:, 1] < r[3]) & (dd < 1.0)
        if mk.sum() > 100: s.append(f'{kk} {np.median(dd[mk])*100:.0f}cm')
    m1 = dd < 1.0
    d2, j2 = tG0.query(gw[:, :2]); m2 = d2 < 0.1
    say(f'[{name}] 주차선 잔차(1m 안 {m1.mean()*100:.0f}%, 중앙 {np.median(dd[m1])*100:.1f}cm): ' + ', '.join(s) + f' | 바닥 높이 차 중앙 {np.median(np.abs(G0[j2[m2],2]-gw[m2,2]))*100:.1f}cm')
    _, pk = dividers(pw[:, :2])
    if len(pk):
        dm = pk[:, None] - ref_pk[None]; jm = np.abs(dm).argmin(1); off = dm[np.arange(len(pk)), jm]; ok = np.abs(off) < 1.2
        say(f'   동쪽 줄 칸막이 {ok.sum()}개 대응, 줄 방향 어긋남(남→북, cm): ' + ' '.join(f'{o*100:+.0f}' for o in off[ok]) + f' | 최대 {np.abs(off[ok]).max()*100:.0f}cm')
log.close()
