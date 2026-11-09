"""사진측량 시험: depth가 닿지 않는 먼 건물·나무(8~80m)를 키프레임 컬러 사진 쌍으로 삼각측량 (카메라 자세는 맵 것을 그대로 씀).
  - 특징점: SIFT, depth가 8m 안으로 잡힌 점은 뺌(이미 메시에 있음)
  - 쌍: 카메라 사이 3~15m, 보는 방향 차 35° 안, 키프레임마다 가까운 짝 4개
  - 맞춤: 비율 검사 0.75 + 알려진 자세로 만든 기본행렬에서 Sampson 거리 1.5px 안
  - 3D 점: 두 사진 재투영 오차 2px 안, 교차각 1.5° 이상, 두 카메라에서 8~80m
  - 0.3m 복셀로 묶어 2쌍 이상이 본 점만 남김
  - 건물 정면 후보: 바닥 위 1.5~30m, 궤적에서 8m 넘게 떨어진 점을 위에서 본 1m 칸 무리로 나누고, 길이 5m 이상·두께 1.5m 이하인 무리를 선분으로 (나무 줄도 섞일 수 있음)
출력: far_points.ply(색), facades.csv(x1,y1,x2,y2,height_m,n), far_points.png
사용: far_triangulate.py <pkg_dir> <floor.npz> <out_dir> [--sets roof_cw,roof_ccw]"""
import argparse, re, json, csv
import numpy as np, cv2, open3d as o3d
from pathlib import Path
from scipy import ndimage
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
ap = argparse.ArgumentParser(); ap.add_argument('pkg'); ap.add_argument('floor'); ap.add_argument('out'); ap.add_argument('--sets', default='roof_cw,roof_ccw'); a = ap.parse_args()
P, O = Path(a.pkg), Path(a.out); O.mkdir(parents=True, exist_ok=True)
F = np.load(a.floor); G, lo, cell = F['grid'].astype(float), F['lo'], float(F['cell'])
def fh(xy): return ndimage.map_coordinates(G, [(xy[:, 0] - lo[0]) / cell - 0.5, (xy[:, 1] - lo[1]) / cell - 0.5], order=1, mode='nearest')
sift = cv2.SIFT_create(nfeatures=2500); KF = []
for s in a.sets.split(','):
    Kd = P / f'keyframes_{s}'
    for r in np.loadtxt(P / f'trajectory_{s}_camera_optical.txt', comments='#', ndmin=2):
        i = int(r[8]); ip = Kd / f'rgb/{i}.jpg'
        if not ip.exists(): continue
        k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (Kd / f'calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
        Km = np.array(k).reshape(3, 3); im = cv2.imread(str(ip)); g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        kp, de = sift.detectAndCompute(g, None)
        if de is None: continue
        uv = np.array([p.pt for p in kp], np.float32); D = cv2.imread(str(Kd / f'depth/{i}.png'), cv2.IMREAD_UNCHANGED)
        if D is not None:
            dz = D[np.clip(uv[:, 1].astype(int), 0, D.shape[0] - 1), np.clip(uv[:, 0].astype(int), 0, D.shape[1] - 1)] / 1000.0; far = ~((dz > 0.3) & (dz < 8.0))
            uv, de = uv[far], de[far]
        Rm = Rot.from_quat(r[4:8]).as_matrix(); c = r[1:4]
        KF.append(dict(id=i, set=s, K=Km, R=Rm, c=c, uv=uv, de=de, im=im, fwd=Rm[:, 2]))
print(f'키프레임 {len(KF)}장, 먼 특징점 평균 {np.mean([len(k["uv"]) for k in KF]):.0f}개')
C = np.array([k['c'] for k in KF]); Fw = np.array([k['fwd'] for k in KF]); tc = cKDTree(C)
pairs = set()
for i in range(len(KF)):
    cand = [j for j in tc.query_ball_point(C[i], 15.0) if j != i and np.linalg.norm(C[j] - C[i]) > 3.0 and Fw[i] @ Fw[j] > np.cos(np.radians(35))]
    cand.sort(key=lambda j: abs(np.linalg.norm(C[j] - C[i]) - 7.0))
    for j in cand[:4]: pairs.add((min(i, j), max(i, j)))
print(f'사진 쌍 {len(pairs)}개')
fl = cv2.FlannBasedMatcher(dict(algorithm=1, trees=4), dict(checks=48)); pts, cols, pid = [], [], []
def Pmat(k): return k['K'] @ np.c_[k['R'].T, -k['R'].T @ k['c']]
def skew(t): return np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
for n_, (i, j) in enumerate(sorted(pairs)):
    A, B = KF[i], KF[j]
    if len(A['de']) < 20 or len(B['de']) < 20: continue
    m = fl.knnMatch(A['de'], B['de'], k=2); good = [p[0] for p in m if len(p) == 2 and p[0].distance < 0.75 * p[1].distance]
    if len(good) < 8: continue
    ua = A['uv'][[g.queryIdx for g in good]].astype(float); ub = B['uv'][[g.trainIdx for g in good]].astype(float)
    Rab = B['R'].T @ A['R']; tab = B['R'].T @ (A['c'] - B['c']); E = skew(tab) @ Rab; Fm = np.linalg.inv(B['K']).T @ E @ np.linalg.inv(A['K'])
    xa = np.c_[ua, np.ones(len(ua))]; xb = np.c_[ub, np.ones(len(ub))]; Fx = xa @ Fm.T; Ftx = xb @ Fm
    sam = (np.sum(xb * Fx, 1) ** 2) / (Fx[:, 0] ** 2 + Fx[:, 1] ** 2 + Ftx[:, 0] ** 2 + Ftx[:, 1] ** 2); ok = sam < 1.5 ** 2
    if ok.sum() < 5: continue
    X = cv2.triangulatePoints(Pmat(A), Pmat(B), ua[ok].T, ub[ok].T); X = (X[:3] / X[3]).T
    def proj(k, X):
        xc = (X - k['c']) @ k['R']; return xc[:, 2], (xc[:, :2] / xc[:, 2:3]) * [k['K'][0, 0], k['K'][1, 1]] + [k['K'][0, 2], k['K'][1, 2]]
    za, pa = proj(A, X); zb, pb = proj(B, X)
    ra, rb = A['c'] - X, B['c'] - X; ang = np.degrees(np.arccos(np.clip((ra * rb).sum(1) / np.linalg.norm(ra, axis=1) / np.linalg.norm(rb, axis=1), -1, 1)))
    keep = (np.linalg.norm(pa - ua[ok], axis=1) < 2) & (np.linalg.norm(pb - ub[ok], axis=1) < 2) & (ang > 1.5) & (za > 8) & (zb > 8) & (za < 80) & (zb < 80)
    if not keep.any(): continue
    X = X[keep]; uu = ua[ok][keep].astype(int); pts.append(X); cols.append(A['im'][uu[:, 1], uu[:, 0]][:, ::-1] / 255.0); pid.append(np.full(len(X), n_))
X = np.vstack(pts); Cc = np.vstack(cols); Pi = np.concatenate(pid); h = X[:, 2] - fh(X[:, :2])
ok = (h > -1.0) & (h < 40); X, Cc, Pi, h = X[ok], Cc[ok], Pi[ok], h[ok]
vk = np.floor(X / 0.3).astype(np.int64); key = (vk[:, 0] * 1000003 + vk[:, 1]) * 1000003 + vk[:, 2]
u_, inv = np.unique(key, return_inverse=True); npair = np.zeros(len(u_), int)
for kk in np.unique(np.c_[inv, Pi], axis=0)[:, 0]: npair[kk] += 1
keep = npair[inv] >= 2; X, Cc, h = X[keep], Cc[keep], h[keep]
pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(X)); pc.colors = o3d.utility.Vector3dVector(Cc); pc = pc.voxel_down_sample(0.15)
o3d.io.write_point_cloud(str(O / 'far_points.ply'), pc); print(f'3D 점 {len(X):,} (2쌍 이상) → 복셀 {len(pc.points):,}')
# 건물 정면: 궤적에서 8m 넘게, 바닥 위 1.5~30m
T = np.vstack([np.loadtxt(P / f'trajectory_{s}_camera_link.txt', comments='#', ndmin=2)[:, 1:3] for s in ('bonnet_cw', 'bonnet_ccw', 'roof_cw', 'roof_ccw')])
dT = cKDTree(T).query(X[:, :2])[0]; B_ = X[(dT > 8) & (h > 1.5) & (h < 30)]; hb = h[(dT > 8) & (h > 1.5) & (h < 30)]
# 위에서 본 1m 칸 무리로 먼저 나누고(이웃 1칸까지 이음), 무리마다 주성분으로 길이·두께를 재서 얇고 긴 것만 정면으로
segs, clusters = [], []
if len(B_):
    gx0, gy0 = B_[:, 0].min() - 2, B_[:, 1].min() - 2; bi = ((B_[:, 0] - gx0) / 1.0).astype(int); bj = ((B_[:, 1] - gy0) / 1.0).astype(int)
    occ = np.zeros((bi.max() + 3, bj.max() + 3), bool); occ[bi, bj] = True; lab, nl = ndimage.label(ndimage.binary_dilation(occ), np.ones((3, 3))); lb = lab[bi, bj]
    for k in range(1, nl + 1):
        inl = np.flatnonzero(lb == k)
        if len(inl) < 40: continue
        Q2 = B_[inl, :2]; mu = Q2.mean(0); U, S_, Vt = np.linalg.svd(Q2 - mu, full_matrices=False); dv = Vt[0]; nrm = Vt[1]
        tt = (Q2 - mu) @ dv; ww = (Q2 - mu) @ nrm; t0, t1 = np.percentile(tt, [2, 98]); thick = 2 * ww.std()
        th_ = np.radians(-166.0); e_ = mu @ np.array([[np.cos(th_), np.sin(th_)], [-np.sin(th_), np.cos(th_)]])   # C 추정: 맵 → ENU 회전 −166° (맵 원점 기준 방향)
        brg = (np.degrees(np.arctan2(e_[0], e_[1])) + 360) % 360
        clusters.append(dict(cx=mu[0], cy=mu[1], length_m=t1 - t0, width_m=max(thick, 0.5), yaw_deg=np.degrees(np.arctan2(dv[1], dv[0])) % 180, height_m=float(np.percentile(hb[inl], 95)),
                             n=int(len(inl)), dist_to_path_m=float(np.median(cKDTree(T).query(Q2)[0])), bearing_deg=brg, dir='북북동동남남서서북'[int(((brg + 22.5) % 360) // 45) * 1:int(((brg + 22.5) % 360) // 45) * 1 + 1]))
        if t1 - t0 < 5 or thick > 1.5: continue
        segs.append(dict(x1=mu[0] + dv[0] * t0, y1=mu[1] + dv[1] * t0, x2=mu[0] + dv[0] * t1, y2=mu[1] + dv[1] * t1, height_m=float(np.percentile(hb[inl], 95)), n=int(len(inl)),
                         dist_to_path_m=float(np.median(cKDTree(T).query(Q2)[0])), thickness_m=float(thick)))
with open(O / 'facades.csv', 'w', newline='', encoding='utf-8') as f:
    w = csv.DictWriter(f, ['x1', 'y1', 'x2', 'y2', 'height_m', 'n', 'dist_to_path_m', 'thickness_m']); w.writeheader()
    for s_ in segs: w.writerow({k: round(v, 2) if isinstance(v, float) else v for k, v in s_.items()})
D8 = ['북', '북동', '동', '남동', '남', '남서', '서', '북서']
with open(O / 'far_clusters.csv', 'w', newline='', encoding='utf-8') as f:
    w = csv.DictWriter(f, ['cx', 'cy', 'length_m', 'width_m', 'yaw_deg', 'height_m', 'n', 'dist_to_path_m', 'bearing_deg', 'dir']); w.writeheader()
    for c_ in sorted(clusters, key=lambda c: -c['n']):
        c_['dir'] = D8[int(((c_['bearing_deg'] + 22.5) % 360) // 45)]; w.writerow({k: round(v, 2) if isinstance(v, float) else v for k, v in c_.items()})
for c_ in sorted(clusters, key=lambda c: -c['n'])[:10]: print(f"  무리 ({c_['cx']:+.1f},{c_['cy']:+.1f}) 실제 방향 {c_['dir']}, 길이 {c_['length_m']:.0f}m, 높이 {c_['height_m']:.1f}m, 점 {c_['n']}, 경로에서 {c_['dist_to_path_m']:.0f}m")
for s_ in segs: print(f"  정면 ({s_['x1']:+.1f},{s_['y1']:+.1f})→({s_['x2']:+.1f},{s_['y2']:+.1f}) 길이 {np.hypot(s_['x2']-s_['x1'], s_['y2']-s_['y1']):.1f}m, 높이 {s_['height_m']:.1f}m, 점 {s_['n']}, 경로에서 {s_['dist_to_path_m']:.0f}m")
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 2, figsize=(22, 10))
sc_ = ax[0].scatter(X[:, 0], X[:, 1], c=h, s=1, cmap='turbo', vmin=0, vmax=20); plt.colorbar(sc_, ax=ax[0], shrink=0.6, label='height above floor (m)')
ax[0].plot(T[:, 0], T[:, 1], 'k.', ms=0.3)
for s_ in segs: ax[0].plot([s_['x1'], s_['x2']], [s_['y1'], s_['y2']], 'm-', lw=3)
ax[0].set_aspect('equal'); ax[0].set_title('triangulated far points (top) + facade lines'); ax[1].scatter(X[:, 0], h, c=Cc, s=1); ax[1].set_title('side view x vs height'); ax[1].set_ylim(-2, 30)
plt.tight_layout(); plt.savefig(O / 'far_points.png', dpi=70); json.dump(dict(keyframes=len(KF), pairs=len(pairs), points=int(len(X)), facades=len(segs)), open(O / 'far_points.json', 'w'))
