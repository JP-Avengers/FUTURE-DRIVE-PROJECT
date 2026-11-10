"""손 촬영 묶음 맵 비강체 보정: 키프레임마다 수평 보정 (dx, dy, dθ)를 선형 최소제곱으로.
 조건: (1) 대응점 — 그 지점을 본 키프레임들을 옮기면 손 촬영 지점이 v0.5 지점으로(점: 2식, 선: 법선 1식)
       (2) 궤적 모양 — 이웃 키프레임 상대 위치 = dθ만큼 돌린 원래 상대 위치(σ_o), dθ 천천히(σ_θ/키프레임)
       (3) 세션 사이 — 2m 안 다른 세션 키프레임끼리 보정 비슷(σ_x, σ_xθ, 약하게), (4) 약한 0 사전(σ_p)
 교차 검증: 대응점 묶음(같은 지점의 여러 세션)마다 빼고 풀어 그 지점 예측 오차.
 출력: <out>/corr.npz(d, th), <out>/fems5c/ (보정된 내보내기: 궤적·점군·키프레임 링크, v0.5 좌표 → T=단위), loo.json, 그림
사용: nr_solve.py <T_ms.npy> <cps.json> <out_dir> [--run fems5]"""
import sys, json, os, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import lsqr
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; T = np.load(a[1]); CPS = json.load(open(a[2], encoding='utf-8')); O = Path(a[3]); O.mkdir(parents=True, exist_ok=True)
RUN = a[a.index('--run') + 1] if '--run' in a else 'fems5'
SIG_O, SIG_TH, SIG_X, SIG_XTH, SIG_P, SIG_PTH = 0.05, np.radians(0.3), 1.0, np.radians(3), 5.0, np.radians(10)
SESS = {'s0_feone': (1794125601.8, 1794125934.5), 's1_feit1': (1794124670.4, 1794124862.7), 's2_fecen': (1794126172.0, 1794126316.2),
        's3_fe422a': (1794124958.7, 1794125118.4), 's4_fe422b': (1794126402.2, 1794126508.4), 's5_feit1w': (1794125227.8, 1794125358.0)}
E = Path.home() / 'b_slam/exports'; cid = np.loadtxt(E / RUN / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); N = len(cid)
cw = cid[:, 1:4] @ T[:3, :3].T + T[:3, 3]; c2 = cw[:, :2]
fwd = np.array([(T[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix())[:2, 2] for r in cid]); fwd /= np.linalg.norm(fwd, axis=1, keepdims=True) + 1e-9
ses = np.full(N, '', object)
for k, (t0, t1) in SESS.items(): ses[(cid[:, 0] >= t0) & (cid[:, 0] <= t1)] = k
print(f'키프레임 {N}, 세션별', {k: int((ses == k).sum()) for k in SESS})
def observers(cp):
    s = cp['bev'].replace(RUN + '_', ''); p = np.array([cp['x'], cp['y']]); idx = np.where(ses == s)[0]
    v = p - c2[idx]; dist = np.linalg.norm(v, axis=1); cosang = (v * fwd[idx]).sum(1) / (dist + 1e-9)
    ok = idx[(dist < 6) & (dist > 0.5) & (cosang > np.cos(np.radians(50)))]
    return ok if len(ok) else idx[np.argsort(dist)[:3]]
def build(cps):
    rows, cols, vals, rhs = [], [], [], []; r = [0]
    def eq(cs, vs, b, w):
        for c_, v_ in zip(cs, vs): rows.append(r[0]); cols.append(c_); vals.append(v_ * w)
        rhs.append(b * w); r[0] += 1
    for cp in cps:
        K = observers(cp); p = np.array([cp['x'], cp['y']]); w = cp.get('w', 1.0) / np.sqrt(len(K)) / 0.2      # 대응점 σ≈0.2m
        for k in K:
            ry, rx = p[1] - c2[k, 1], p[0] - c2[k, 0]                       # d + θ J (p - c), J(x,y)=(-y,x)
            if cp['type'] == 'point':
                eq([3 * k, 3 * k + 2], [1, -ry], cp['dx'], w); eq([3 * k + 1, 3 * k + 2], [1, rx], cp['dy'], w)
            else:
                nx, ny = cp['n']; eq([3 * k, 3 * k + 1, 3 * k + 2], [nx, ny, -ry * nx + rx * ny], cp['off_n'], w)
    for s in SESS:                                                        # 궤적 모양·방향 부드러움(세션 안, 시간 순)
        idx = np.where(ses == s)[0]; idx = idx[np.argsort(cid[idx, 0])]
        for k0, k1 in zip(idx[:-1], idx[1:]):
            dv = c2[k1] - c2[k0]                                          # (d1 - d0) - θ0 J dv = 0
            eq([3 * k1, 3 * k0, 3 * k0 + 2], [1, -1, dv[1]], 0, 1 / SIG_O); eq([3 * k1 + 1, 3 * k0 + 1, 3 * k0 + 2], [1, -1, -dv[0]], 0, 1 / SIG_O)
            eq([3 * k1 + 2, 3 * k0 + 2], [1, -1], 0, 1 / SIG_TH)
    tr = cKDTree(c2)                                                      # 세션 사이 약한 결합
    for i, j in tr.query_pairs(2.0):
        if ses[i] == ses[j] or ses[i] == '' or ses[j] == '': continue
        eq([3 * i, 3 * j], [1, -1], 0, 1 / SIG_X); eq([3 * i + 1, 3 * j + 1], [1, -1], 0, 1 / SIG_X); eq([3 * i + 2, 3 * j + 2], [1, -1], 0, 1 / SIG_XTH)
    for k in range(N):
        eq([3 * k], [1], 0, 1 / SIG_P); eq([3 * k + 1], [1], 0, 1 / SIG_P); eq([3 * k + 2], [1], 0, 1 / SIG_PTH)
    A = coo_matrix((vals, (rows, cols)), shape=(r[0], 3 * N)).tocsr(); return A, np.array(rhs)
def solve(cps):
    A, b = build(cps); x = lsqr(A, b, atol=1e-10, btol=1e-10, iter_lim=20000)[0]; return x.reshape(N, 3)
def predict(u, cp):
    K = observers(cp); p = np.array([cp['x'], cp['y']])
    pr = np.mean([u[k, :2] + u[k, 2] * np.array([-(p[1] - c2[k, 1]), p[0] - c2[k, 0]]) for k in K], axis=0)
    return pr
def err(u, cp):
    pr = predict(u, cp)
    return float(np.hypot(pr[0] - cp['dx'], pr[1] - cp['dy'])) if cp['type'] == 'point' else float(abs(pr @ np.array(cp['n']) - cp['off_n']))
cps = [c for c in CPS if c.get('use', True) and ('dx' in c or 'off_n' in c)]
u = solve(cps)
print(f'\n대응점 {len(cps)}개로 풀이: 보정량 |d| 중앙 {np.median(np.linalg.norm(u[:, :2], axis=1)):.2f}m, 95% {np.percentile(np.linalg.norm(u[:, :2], axis=1), 95):.2f}m, |dθ| 95% {np.degrees(np.percentile(np.abs(u[:, 2]), 95)):.1f}°')
print('세션별 보정 중앙 |d|(m)·dθ(°):', {s: (round(float(np.median(np.linalg.norm(u[ses == s, :2], axis=1))), 2), round(float(np.degrees(np.median(u[ses == s, 2]))), 1)) for s in SESS})
groups = {}
for c in cps: groups.setdefault(c.get('group', c['name'].split('_')[0]), []).append(c)
res = []
print('\n대응점별: 맞춘 뒤 잔차 | 빼고 푼 예측 오차(교차 검증)')
for g, members in groups.items():
    rest = [c for c in cps if c not in members]; u_l = solve(rest)
    for c in members:
        e_fit, e_loo = err(u, c), err(u_l, c); before = float(np.hypot(c['dx'], c['dy'])) if c['type'] == 'point' else abs(c['off_n'])
        res.append(dict(name=c['name'], type=c['type'], w=c.get('w', 1.0), before=round(before, 2), fit=round(e_fit, 2), loo=round(e_loo, 2)))
        print(f"  {c['name']:10s} {c['type']:5s} w{c.get('w', 1.0):.1f} | 보정 전 {before:4.2f}m → 맞춘 뒤 {e_fit:4.2f}m | 교차 검증 {e_loo:4.2f}m")
lo_ = np.array([r_['loo'] for r_ in res]); print(f'교차 검증 오차: 중앙 {np.median(lo_):.2f}m, 최대 {lo_.max():.2f}m (보정 전 중앙 {np.median([r_["before"] for r_ in res]):.2f}m)')
json.dump(res, open(O / 'loo.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1); np.savez(O / 'corr.npz', d=u[:, :2], th=u[:, 2], ses=ses.astype(str))
# 보정된 내보내기 fems5c (v0.5 좌표, T=단위)
F = O / f'{RUN}c'; F.mkdir(exist_ok=True)
if not (F / 'keyframes').exists(): os.symlink(E / RUN / 'keyframes', F / 'keyframes')
Rw = np.array([T[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix() for r in cid]); Rz = Rot.from_euler('z', u[:, 2:3]).as_matrix()
cw2 = cw.copy(); cw2[:, :2] += u[:, :2]; R2 = Rz @ Rw
np.savetxt(F / 'traj_camera_id_camera_poses.txt', np.c_[cid[:, 0], cw2, Rot.from_matrix(R2).as_quat(), cid[:, 8]], fmt='%.6f', header='timestamp x y z qx qy qz qw id (v0.5 좌표, 비강체 보정)')
tr_ = np.loadtxt(E / RUN / 'traj_optimized_poses.txt', comments='#', ndmin=2); o_ = np.argsort(cid[:, 0])
pl = tr_[:, 1:4] @ T[:3, :3].T + T[:3, 3]; RL = np.array([T[:3, :3] @ Rot.from_quat(q).as_matrix() for q in tr_[:, 4:8]])
dl = np.stack([np.interp(tr_[:, 0], cid[o_, 0], u[o_, j]) for j in range(3)], 1)
pl[:, :2] += dl[:, :2]; RL = Rot.from_euler('z', dl[:, 2:3]).as_matrix() @ RL
np.savetxt(F / 'traj_optimized_poses.txt', np.c_[tr_[:, 0], pl, Rot.from_matrix(RL).as_quat()], fmt='%.6f', header='timestamp x y z qx qy qz qw (v0.5 좌표, 비강체 보정)')
pc = o3d.io.read_point_cloud(str(E / RUN / 'cloud_cloud.ply')); X = np.asarray(pc.points) @ T[:3, :3].T + T[:3, 3]
_, nk = cKDTree(cw).query(X); th_ = u[nk, 2]; rel = X[:, :2] - c2[nk]
X2 = X.copy(); X2[:, 0] = c2[nk, 0] + u[nk, 0] + np.cos(th_) * rel[:, 0] - np.sin(th_) * rel[:, 1]; X2[:, 1] = c2[nk, 1] + u[nk, 1] + np.sin(th_) * rel[:, 0] + np.cos(th_) * rel[:, 1]
pc.points = o3d.utility.Vector3dVector(X2); o3d.io.write_point_cloud(str(F / 'cloud_cloud.ply'), pc); np.save(O / 'T_identity.npy', np.eye(4))
# 그림: 보정 화살표(키프레임 5개마다, ×3)
X0, Y0, Wd, Hd, C = -70, -40, 950, 700, 0.1; img = np.zeros((Hd, Wd, 3), np.uint8)
cols = {'s0_feone': (255, 255, 255), 's1_feit1': (0, 200, 255), 's2_fecen': (0, 255, 0), 's3_fe422a': (255, 0, 255), 's4_fe422b': (255, 160, 0), 's5_feit1w': (0, 120, 255)}
for k in range(0, N, 5):
    if ses[k] == '': continue
    u0, v0 = int((c2[k, 0] - X0) / C), Hd - 1 - int((c2[k, 1] - Y0) / C); u1, v1 = int(u0 + 3 * u[k, 0] / C), int(v0 - 3 * u[k, 1] / C)
    cv2.arrowedLine(img, (u0, v0), (u1, v1), cols[ses[k]], 1, tipLength=0.2)
for c in cps:
    u0, v0 = int((c['x'] - X0) / C), Hd - 1 - int((c['y'] - Y0) / C); cv2.circle(img, (u0, v0), 5, (0, 0, 255), 2); cv2.putText(img, c['name'], (u0 + 6, v0), 0, 0.35, (0, 0, 255), 1)
for xg in range(-70, 25, 10): cv2.putText(img, str(xg), (int((xg - X0) / C) + 2, 12), 0, 0.4, (200, 200, 200), 1)
for yg in range(-40, 30, 10): cv2.putText(img, str(yg), (2, Hd - 1 - int((yg - Y0) / C)), 0, 0.4, (200, 200, 200), 1)
cv2.imwrite(str(O / 'corr_arrows.png'), img); print('→', O)
