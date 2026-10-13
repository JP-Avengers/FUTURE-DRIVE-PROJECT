#!/usr/bin/env python3
"""반시계 맵(run20)을 시계 방향 맵(run18, v0.2, 고정)에 비강체 정합.
 1) 초기값: 주차선 2D 강체 정합(run20_to_run18_paint2d.npy) + 바닥 높이 맞춤
 2) 앵커: 반시계 키프레임마다 반경 R 안에서 국소 정합
    - 높이·기울기: 두 맵의 국소 바닥 평면
    - 수평 위치·방향: 주차선 + 구조물(바닥 위 0.2~2.0m 점) 2D ICP
 3) 자세 최적화: 앵커(절대 자세) + 원래 키프레임 간 상대 자세(이웃 1·2·4개)를 함께 만족 (soft-L1)
 4) 검증: 반시계 점군을 새 자세로 옮겨 주차선 잔차(A/B/C), 바닥 높이 차
출력: exports/run20nr/ (traj_camera_id_camera_poses.txt, traj_optimized_poses.txt, nonrigid_report.txt)
(~/tsdf_env)"""
import re, sys
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import least_squares

H = Path.home() / 'b_slam/exports'; OUT = H / 'run20nr'; OUT.mkdir(exist_ok=True)
RAD = 10.0
REG = {'A': (-50, -29, -25, -14), 'B': (-34, -17, 3, 14), 'C': (-4, 7, -16, 11)}
log = open(OUT / 'nonrigid_report.txt', 'w')
def say(*a):
    s = ' '.join(str(x) for x in a); print(s, flush=True); log.write(s + '\n')

def split(run):
    """점군 → 바닥·주차선·구조물 (가장 가까운 궤적점 기준 높이)"""
    pc = o3d.io.read_point_cloud(str(H / run / 'cloud_cloud.ply')); P = np.asarray(pc.points); C = (np.asarray(pc.colors) * 255).astype(int)
    T = np.loadtxt(H / run / 'traj_optimized_poses.txt', ndmin=2)[:, 1:4]
    _, i = cKDTree(T[:, :2]).query(P[:, :2]); dz = T[i, 2] - P[:, 2]
    g = (dz > 1.0) & (dz < 2.2); paint = g & (C.mean(1) > 120) & (C.max(1) - C.min(1) < 40)
    s = (dz > -0.5) & (dz < 1.35)  # 바닥보다 약 0.2~2.0m 위 (카메라 1.55m 기준)
    return P[g], P[paint], P[s]

G0, P0, S0 = split('run18'); G1, P1, S1 = split('run20')
say(f'run18: 바닥 {len(G0)}, 주차선 {len(P0)}, 구조물 {len(S0)} | run20: 바닥 {len(G1)}, 주차선 {len(P1)}, 구조물 {len(S1)}')
tG0 = cKDTree(G0[:, :2])

# 1) 초기 강체 변환
p = np.load(H / 'run20_to_run18_paint2d.npy'); R2, t2 = p[:4].reshape(2, 2), p[4:]
Tg = np.eye(4); Tg[:2, :2] = R2; Tg[:2, 3] = t2
def tf(T, X): return X @ T[:3, :3].T + T[:3, 3]
g1 = tf(Tg, G1); d, j = tG0.query(g1[:, :2]); m = d < 0.1; Tg[2, 3] = np.median(G0[j[m], 2] - g1[m, 2])
say(f'초기 강체: yaw {np.degrees(np.arctan2(R2[1,0], R2[0,0])) % 360:.2f}°, t=({Tg[0,3]:.2f},{Tg[1,3]:.2f},{Tg[2,3]:.2f})')

# 반시계 키프레임 자세 (광학 좌표, 시간순), 세션 분리
tr = np.loadtxt(H / 'run20/traj_camera_id_camera_poses.txt', comments='#', ndmin=2); tr = tr[np.argsort(tr[:, 0])]
N = len(tr); Pc = np.zeros((N, 4, 4))
for k, r in enumerate(tr):
    Pc[k] = np.eye(4); Pc[k][:3, :3] = Rot.from_quat(r[4:8]).as_matrix(); Pc[k][:3, 3] = r[1:4]
sess = np.r_[0, np.cumsum(np.diff(tr[:, 0]) > 10)]
say(f'반시계 키프레임 {N}개, 세션 {sess.max() + 1}개')
P_init = np.array([Tg @ x for x in Pc])

# 2) 앵커 (국소 정합)
def plane(X):
    A = np.c_[X[:, 0], X[:, 1], np.ones(len(X))]; c = np.linalg.lstsq(A, X[:, 2], rcond=None)[0]
    r = X[:, 2] - A @ c; k = np.abs(r) < 3 * np.median(np.abs(r)) + 0.02
    return np.linalg.lstsq(A[k], X[k, 2], rcond=None)[0]

def feat2d(P, S, center, rng):
    a = P[np.linalg.norm(P[:, :2] - center, axis=1) < RAD][:, :2]; b = S[np.linalg.norm(S[:, :2] - center, axis=1) < RAD][:, :2]
    if len(b) > 6000: b = b[rng.choice(len(b), 6000, replace=False)]
    return np.r_[a, a, b]  # 주차선 가중 2

RADH = 8.0
vert, horz = {}, {}
G1w, P1w = tf(Tg, G1), tf(Tg, P1)
for k in range(N):
    c3 = P_init[k][:3, 3]; c = c3[:2]
    # 높이·기울기 앵커 (바닥 평면)
    gm0 = G0[np.linalg.norm(G0[:, :2] - c, axis=1) < RAD]; gm1 = G1w[np.linalg.norm(G1w[:, :2] - c, axis=1) < RAD]
    if len(gm0) >= 500 and len(gm1) >= 500:
        a0, a1 = plane(gm0), plane(gm1)
        n0 = np.array([-a0[0], -a0[1], 1]); n0 /= np.linalg.norm(n0); n1 = np.array([-a1[0], -a1[1], 1]); n1 /= np.linalg.norm(n1)
        ax = np.cross(n1, n0); s_ = np.linalg.norm(ax); Rt = np.eye(3) if s_ < 1e-9 else Rot.from_rotvec(ax / s_ * np.arcsin(min(s_, 1))).as_matrix()
        z1 = a1 @ [c[0], c[1], 1]; dz = (a0 @ [c[0], c[1], 1]) - z1
        Ct = np.eye(4); Ct[:3, :3] = Rt; Ct[:3, 3] = np.array([c[0], c[1], z1]) - Rt @ np.array([c[0], c[1], z1]); Ct[2, 3] += dz
        vert[k] = Ct @ P_init[k]
    # 수평 앵커 (주차선만: 보는 방향과 무관)
    F0 = P0[np.linalg.norm(P0[:, :2] - c, axis=1) < RADH][:, :2]; F1 = P1w[np.linalg.norm(P1w[:, :2] - c, axis=1) < RADH][:, :2]
    if len(F0) < 150 or len(F1) < 150: continue
    t0 = cKDTree(F0); R = np.eye(2); t = np.zeros(2); ok = True
    for thr in [1.0] * 10 + [0.6] * 10 + [0.3] * 15:
        X = F1 @ R.T + t; dd, jj = t0.query(X); mm = dd < thr
        if mm.sum() < 60: ok = False; break
        A_, B_ = F1[mm], F0[jj[mm]]; ca, cb = A_.mean(0), B_.mean(0)
        U, _, Vt = np.linalg.svd((A_ - ca).T @ (B_ - cb)); Rn = (U @ Vt).T
        if np.linalg.det(Rn) < 0: Vt[1] *= -1; Rn = (U @ Vt).T
        R, t = Rn, cb - Rn @ ca
    if not ok: continue
    X = F1 @ R.T + t; dd = t0.query(X)[0]; frac = (dd < 0.2).mean()
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0])); shift = np.linalg.norm(R @ c + t - c)
    if frac < 0.5 or abs(yaw) > 4 or shift > 1.5: continue
    Cj = np.eye(4); Cj[:2, :2] = R; Cj[:2, 3] = t
    horz[k] = (Cj @ P_init[k], frac)
say(f'높이 앵커 {len(vert)}/{N}개, 수평 앵커(주차선) {len(horz)}/{N}개 (일치율 중앙 {np.median([v[1] for v in horz.values()])*100:.0f}%)')
if horz:
    corr = np.array([np.linalg.norm(horz[k][0][:2, 3] - P_init[k][:2, 3]) for k in horz])
    say(f'수평 앵커가 요구하는 이동: 중앙 {np.median(corr)*100:.0f}cm, 최대 {corr.max()*100:.0f}cm')

# 3) 자세 최적화 (월드 좌표 성분별 제약)
def to_vec(T): return np.r_[Rot.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]
def to_mat(v): T = np.eye(4); T[:3, :3] = Rot.from_rotvec(v[:3]).as_matrix(); T[:3, 3] = v[3:]; return T
def err(Ta, Tb, st, sr):
    D = np.linalg.inv(Ta) @ Tb; return np.r_[Rot.from_matrix(D[:3, :3]).as_rotvec() / sr, D[:3, 3] / st]
def werr(M, P):  # 월드 기준 [회전 x,y,z, 이동 x,y,z]
    return np.r_[Rot.from_matrix(P[:3, :3] @ M[:3, :3].T).as_rotvec(), P[:3, 3] - M[:3, 3]]
rel = []
for k in range(N):
    for s in (1, 2, 4):
        if k + s < N and sess[k] == sess[k + s]: rel.append((k, k + s, np.linalg.inv(Pc[k]) @ Pc[k + s]))
ST_R, SR_R, ST_A, SR_A = 0.03, np.radians(0.3), 0.08, np.radians(0.5)
vk, hk = sorted(vert), sorted(horz)
def resid(x):
    Ps = [to_mat(x[6 * k:6 * k + 6]) for k in range(N)]; r = []
    for a, b, Z in rel: r.append(err(Z, np.linalg.inv(Ps[a]) @ Ps[b], ST_R, SR_R))
    for k in vk: e = werr(vert[k], Ps[k]); r.append(np.r_[e[0] / SR_A, e[1] / SR_A, e[5] / ST_A])
    for k in hk: e = werr(horz[k][0], Ps[k]); r.append(np.r_[e[2] / SR_A, e[3] / ST_A, e[4] / ST_A])
    for k in range(N): e = werr(P_init[k], Ps[k]); r.append(np.r_[e[2] / np.radians(5), e[3], e[4]])  # 약한 수평 유지(1m)
    return np.concatenate(r)
x0 = np.concatenate([to_vec(T) for T in P_init])
from scipy.sparse import lil_matrix
nres = 6 * len(rel) + 3 * (len(vk) + len(hk) + N); J = lil_matrix((nres, 6 * N), dtype=int); row = 0
for a, b, _ in rel: J[row:row + 6, 6 * a:6 * a + 6] = 1; J[row:row + 6, 6 * b:6 * b + 6] = 1; row += 6
for k in vk + hk + list(range(N)): J[row:row + 3, 6 * k:6 * k + 6] = 1; row += 3
sol = least_squares(resid, x0, jac_sparsity=J, loss='soft_l1', f_scale=1.0, max_nfev=40, x_scale='jac')
P_new = np.array([to_mat(sol.x[6 * k:6 * k + 6]) for k in range(N)])
mv = np.linalg.norm(P_new[:, :3, 3] - P_init[:, :3, 3], axis=1)
say(f'최적화: {sol.nfev}회, 비용 {sol.cost:.1f}, 자세 이동 중앙 {np.median(mv)*100:.0f}cm, 최대 {mv.max()*100:.0f}cm')
if hk:
    ra = np.array([np.linalg.norm(werr(horz[k][0], P_new[k])[3:5]) for k in hk]); say(f'남은 수평 앵커 차이 중앙 {np.median(ra)*100:.1f}cm (90% {np.percentile(ra,90)*100:.1f}cm)')
rz = np.array([abs(werr(vert[k], P_new[k])[5]) for k in vk]); say(f'남은 높이 앵커 차이 중앙 {np.median(rz)*100:.1f}cm')
rr = np.array([np.linalg.norm((np.linalg.inv(Z) @ np.linalg.inv(P_new[a]) @ P_new[b])[:3, 3]) for a, b, Z in rel])
say(f'상대 자세 변형 중앙 {np.median(rr)*100:.2f}cm (최대 {rr.max()*100:.1f}cm)')

# 저장 (광학 + camera_link)
lt = (H / 'run20/keyframes/kf_calib' / f'{int(tr[0, 8])}.yaml').read_text()
L = np.eye(4); L[:3, :] = np.array([float(v) for v in re.search(r'local_transform:.*?data: \[([^\]]+)\]', lt, re.S).group(1).replace('\n', ' ').split(',')]).reshape(3, 4)
with open(OUT / 'traj_camera_id_camera_poses.txt', 'w') as f, open(OUT / 'traj_optimized_poses.txt', 'w') as g:
    f.write('#timestamp x y z qx qy qz qw id\n'); g.write('#timestamp x y z qx qy qz qw\n')
    for k in range(N):
        q = Rot.from_matrix(P_new[k][:3, :3]).as_quat(); x = P_new[k][:3, 3]
        f.write(f'{tr[k,0]:.6f} {x[0]:.6f} {x[1]:.6f} {x[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} {int(tr[k,8])}\n')
        B = P_new[k] @ np.linalg.inv(L); q = Rot.from_matrix(B[:3, :3]).as_quat(); x = B[:3, 3]
        g.write(f'{tr[k,0]:.6f} {x[0]:.6f} {x[1]:.6f} {x[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n')

# 4) 검증: 반시계 점을 가장 가까운 키프레임의 자세 변화로 옮김 (원래 반시계 좌표 기준)
cen = Pc[:, :3, 3]; tc = cKDTree(cen)
def warp(X, Pfin):
    _, i = tc.query(X); D = np.einsum('nij,njk->nik', Pfin, np.linalg.inv(Pc))[i]
    return np.einsum('nij,nj->ni', D[:, :3, :3], X) + D[:, :3, 3]
tP0 = cKDTree(P0)
for name, Pf in (('강체(초기)', P_init), ('비강체(최적화)', P_new)):
    pw = warp(P1, Pf); gw = warp(G1, Pf); dd = cKDTree(P0[:, :2]).query(pw[:, :2])[0]
    s = []
    for kk, r in REG.items():
        mk = (pw[:, 0] > r[0]) & (pw[:, 0] < r[1]) & (pw[:, 1] > r[2]) & (pw[:, 1] < r[3]); s.append(f'{kk} {np.median(dd[mk])*100:.0f}cm')
    d2, j2 = tG0.query(gw[:, :2]); m2 = d2 < 0.1
    say(f'[{name}] 주차선 잔차: ' + ', '.join(s) + f' | 바닥 높이 차 중앙 {np.median(np.abs(G0[j2[m2],2]-gw[m2,2]))*100:.1f}cm')
log.close()
