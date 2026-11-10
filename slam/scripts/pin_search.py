"""출발점 고정 회전 탐색: 정합이 확인된 기준점(경로 s_pin m의 키프레임 위치)을 축으로 회차 전체(또는 --to m까지) 바닥 BEV를
요 ±Y°(0.5°)로 돌리고 ±R m(10cm) 이동해 v0.5 정사 영상과 선 무늬 정규화 상관 최대를 찾음. 각도별 최고 점수 곡선도 출력.
사용: pin_search.py <run> <T.npy> <out_dir> [--pin 0] [--to 1e9] [--Y 30] [--R 2] [--maxd 6]"""
import sys, re, json, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, tf, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
PIN, TO, YY, RR, MAXD = opt('--pin', 0), opt('--to', 1e9), opt('--Y', 30), opt('--R', 2), opt('--maxd', 6); C = 0.1
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'; T = np.load(tf)
F = np.load(P / 'floor_height.npz'); G = F['grid'].astype(np.float32); Z0 = float(np.median(G[np.isfinite(G)]))
Q = np.load(P / 'outer/ground_ortho.npz'); RGB, qlo = Q['rgb'], Q['lo']
def th(img, k=5): return img - ndimage.grey_opening(img, size=(k, k))
ref = RGB.astype(np.float32).mean(-1); rv = (RGB.sum(-1) > 0).astype(np.float32); reft = (th(ref) * rv).astype(np.float32)
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); K = E / run / 'keyframes'
tp = cid[:, 1:4]; s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tp, axis=0), axis=1))]
COV = opt('--cov', 0.3)
import open3d as o3d
from scipy.spatial import cKDTree
pc = o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).voxel_down_sample(0.05); X = np.asarray(pc.points) @ T[:3, :3].T + T[:3, 3]
xt = cKDTree(X[:, :2]); camw = tp @ T[:3, :3].T + T[:3, 3]; hloc = np.full(len(cid), np.nan)
for j, c in enumerate(camw):                                   # 키프레임마다 회차 자체 점군의 카메라 아래 바닥 높이
    idx = xt.query_ball_point(c[:2], 0.6)
    if len(idx) > 20: hloc[j] = np.percentile(X[idx, 2], 10)
ok_ = np.isfinite(hloc); hloc[~ok_] = np.interp(np.where(~ok_)[0], np.where(ok_)[0], hloc[ok_]); hloc = ndimage.median_filter(hloc, 5)
P_, V_ = [], []
for jj in np.where(s <= TO)[0]:
    r = cid[jj]; Z0 = hloc[jj]
    i = int(r[8]); txt = (K / f'kf_calib/{i}.yaml').read_text()
    k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', txt, re.S).group(1).replace('\n', ' ').split(',')]
    g = cv2.imread(str(K / f'kf_rgb/{i}.jpg'), cv2.IMREAD_GRAYSCALE).astype(np.float32); h, w = g.shape
    v, u = np.mgrid[h // 3:h:3, 0:w:3]; u = u.ravel(); v = v.ravel()
    Rw = T[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix(); cw = T[:3, :3] @ r[1:4] + T[:3, 3]
    d = np.c_[(u - k[2]) / k[0], (v - k[5]) / k[4], np.ones(len(u))] @ Rw.T; d /= np.linalg.norm(d, axis=1, keepdims=True)
    dn = d[:, 2] < -0.05; u, v, d = u[dn], v[dn], d[dn]; t = (Z0 - cw[2]) / d[:, 2]; ok = (t > 0.8) & (t < MAXD)
    val = g[v[ok], u[ok]]; P_.append((cw + t[ok, None] * d[ok])[:, :2]); V_.append(val / (np.median(val) + 1e-6) * 100)
pts = np.vstack(P_); vals = np.concatenate(V_)
pin = (T[:3, :3] @ tp[np.argmin(np.abs(s - PIN))] + T[:3, 3])[:2]
def bev(p, lo, n):
    ix = ((p[:, 0] - lo[0]) / C).astype(int); iy = ((p[:, 1] - lo[1]) / C).astype(int)
    m = (ix >= 0) & (ix < n[0]) & (iy >= 0) & (iy < n[1]); sm = np.zeros(n, np.float32); ct = np.zeros(n, np.float32)
    np.add.at(sm, (ix[m], iy[m]), vals[m]); np.add.at(ct, (ix[m], iy[m]), 1); return np.where(ct > 0, sm / np.maximum(ct, 1), 0).astype(np.float32), ct > 0
pad = int(RR / C); curve = []; best = None
for yaw in np.arange(-YY, YY + 0.01, 0.5):
    Rz = Rot.from_euler('z', yaw, degrees=True).as_matrix()[:2, :2]; pr = (pts - pin) @ Rz.T + pin
    lo = pr.min(0) - 0.5; n = tuple(((pr.max(0) + 0.5 - lo) / C).astype(int) + 1)
    B, Bm = bev(pr, lo, n); Bt = th(B) * Bm; M = Bm.astype(np.float32); nM = M.sum()
    tpl = np.where(Bm, Bt - Bt[Bm].mean(), 0).astype(np.float32)
    x0 = int(round((lo[0] - qlo[0]) / C)) - pad; y0 = int(round((lo[1] - qlo[1]) / C)) - pad
    W = np.zeros((n[0] + 2 * pad, n[1] + 2 * pad), np.float32); V = np.zeros_like(W)
    xs, ys = slice(max(x0, 0), min(x0 + W.shape[0], reft.shape[0])), slice(max(y0, 0), min(y0 + W.shape[1], reft.shape[1]))
    W[xs.start - x0:xs.stop - x0, ys.start - y0:ys.stop - y0] = reft[xs, ys]; V[xs.start - x0:xs.stop - x0, ys.start - y0:ys.stop - y0] = rv[xs, ys]
    num = cv2.matchTemplate(W, tpl, cv2.TM_CCORR); Ws = cv2.matchTemplate(W, M, cv2.TM_CCORR); W2 = cv2.matchTemplate(W * W, M, cv2.TM_CCORR)
    cov = cv2.matchTemplate(V, M, cv2.TM_CCORR) / nM; var = np.maximum(W2 - Ws * Ws / nM, 0)
    r_ = num / (np.sqrt(var) * np.linalg.norm(tpl) + 1e-3); r_[cov < COV] = -1
    mi = np.unravel_index(np.argmax(r_), r_.shape); sc = float(r_[mi]); dx, dy = (mi[0] - pad) * C, (mi[1] - pad) * C
    curve.append((yaw, sc, dx, dy, float(cov[mi])))
    if best is None or sc > best[1]: best = (yaw, sc, dx, dy, float(cov[mi]))
yaw, sc, dx, dy, cv_ = best
R3 = Rot.from_euler('z', yaw, degrees=True).as_matrix(); S = np.eye(4); S[:3, :3] = R3; S[:2, 3] = pin - R3[:2, :2] @ pin + [dx, dy]
np.save(O / 'T_src_to_ref.npy', S @ T)
cs = np.array([c[1] for c in curve]); far = [c[1] for c in curve if abs(c[0] - yaw) > 4] or [float('nan')]
print(f'{run} 고정점 {PIN:.0f}m ({pin[0]:+.1f},{pin[1]:+.1f}), 경로 ~{min(TO, s[-1]):.0f}m, 점 {len(pts):,}')
print(f'최고: 요 {yaw:+.1f}° 이동 ({dx:+.1f},{dy:+.1f})m r {sc:+.3f} (정사 영상 덮임 {cv_:.0%}) | 4° 넘게 떨어진 각도 최고 {max(far):+.3f}, 전체 중앙 {np.median(cs):+.3f}')
print('각도별(5°마다):', ' '.join(f'{c[0]:+.0f}:{c[1]:+.2f}' for c in curve if c[0] % 5 == 0))
(O / 'pin_search.json').write_text(json.dumps(dict(pin=PIN, yaw=yaw, dx=dx, dy=dy, r=sc, cover=cv_, second=max(far), curve=curve), indent=1), encoding='utf-8')
