"""대응점 고르기용 그림: (1) 손 촬영 회차의 컬러 BEV(10cm, 정합 T 좌표) (2) v0.5 바닥 정사 영상 — 둘 다 5m 격자·좌표 표시.
경로(검정 선)와 경로 거리 표시(10m마다)도 BEV에 그림. 위가 +y, 오른쪽이 +x.
사용: fe_bev.py <run> <T.npy> <out_dir> [--maxd 6] [--ortho]"""
import sys, re, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, tf, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True)
MAXD = float(a[a.index('--maxd') + 1]) if '--maxd' in a else 6.0; C = 0.1
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'; T = np.load(tf)
F = np.load(P / 'floor_height.npz'); G = F['grid'].astype(np.float32); Z0 = float(np.median(G[np.isfinite(G)]))
def grid_draw(img, lo, H, step=5.0):
    x0 = np.ceil(lo[0] / step) * step; y0 = np.ceil(lo[1] / step) * step
    for x in np.arange(x0, lo[0] + img.shape[1] * C, step):
        u = int((x - lo[0]) / C); cv2.line(img, (u, 0), (u, H - 1), (0, 140, 255) if x % 25 == 0 else (90, 90, 90), 1)
        if x % 10 == 0: cv2.putText(img, f'{x:.0f}', (u + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
    for y in np.arange(y0, lo[1] + H * C, step):
        v = H - 1 - int((y - lo[1]) / C); cv2.line(img, (0, v), (img.shape[1] - 1, v), (0, 140, 255) if y % 25 == 0 else (90, 90, 90), 1)
        if y % 10 == 0: cv2.putText(img, f'{y:.0f}', (2, v - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
if '--ortho' in a:
    Q = np.load(P / 'outer/ground_ortho.npz'); rgb, qlo = Q['rgb'], Q['lo']
    img = np.ascontiguousarray(np.transpose(rgb, (1, 0, 2))[::-1][:, :, ::-1]); grid_draw(img, qlo, img.shape[0])
    cv2.imwrite(str(O / 'ortho_grid.png'), img); print('정사 영상', img.shape, 'lo', qlo)
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); K = E / run / 'keyframes'
tp = cid[:, 1:4]; s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tp, axis=0), axis=1))]
import open3d as o3d
from scipy.spatial import cKDTree
pc = o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')).voxel_down_sample(0.05); X = np.asarray(pc.points) @ T[:3, :3].T + T[:3, 3]
xt = cKDTree(X[:, :2]); camw = cid[:, 1:4] @ T[:3, :3].T + T[:3, 3]; hloc = np.full(len(cid), np.nan)
for j, c in enumerate(camw):                                   # 키프레임마다 회차 자체 점군의 카메라 아래 바닥 높이(반경 0.6m, 하위 10%)
    idx = xt.query_ball_point(c[:2], 0.6)
    if len(idx) > 20: hloc[j] = np.percentile(X[idx, 2], 10)
ok_ = np.isfinite(hloc); hloc[~ok_] = np.interp(np.where(~ok_)[0], np.where(ok_)[0], hloc[ok_])
hloc = ndimage.median_filter(hloc, 5)
print(run, '카메라-자체 바닥 높이 중앙', round(float(np.median(camw[:, 2] - hloc)), 2))
P_, V_ = [], []
for jj, r in enumerate(cid):
    Z0 = hloc[jj]
    i = int(r[8]); txt = (K / f'kf_calib/{i}.yaml').read_text()
    k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', txt, re.S).group(1).replace('\n', ' ').split(',')]
    im = cv2.imread(str(K / f'kf_rgb/{i}.jpg')).astype(np.float32); h, w = im.shape[:2]
    v, u = np.mgrid[h // 3:h:2, 0:w:2]; u = u.ravel(); v = v.ravel()
    Rw = T[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix(); cw = T[:3, :3] @ r[1:4] + T[:3, 3]
    d = np.c_[(u - k[2]) / k[0], (v - k[5]) / k[4], np.ones(len(u))] @ Rw.T; d /= np.linalg.norm(d, axis=1, keepdims=True)
    dn = d[:, 2] < -0.05; u, v, d = u[dn], v[dn], d[dn]; t = (Z0 - cw[2]) / d[:, 2]; ok = (t > 0.8) & (t < MAXD)
    P_.append((cw + t[ok, None] * d[ok])[:, :2]); V_.append(np.c_[im[v[ok], u[ok]], t[ok]])
P_ = np.vstack(P_); V_ = np.vstack(V_)
lo = P_.min(0) - 2; n = ((P_.max(0) + 2 - lo) / C).astype(int) + 1
ix = ((P_[:, 0] - lo[0]) / C).astype(int); iy = ((P_[:, 1] - lo[1]) / C).astype(int)
# 칸마다 가장 가까이서 본 화소 색(번짐 줄임)
order = np.argsort(-V_[:, 3]); lin = ix * n[1] + iy; best = np.zeros(n[0] * n[1], int) - 1; best[lin[order]] = order
img = np.zeros((n[1], n[0], 3), np.uint8); m = best >= 0; bx, by = np.unravel_index(np.where(m)[0], (n[0], n[1]))
img[n[1] - 1 - by, bx] = np.clip(V_[best[m], :3], 0, 255).astype(np.uint8)
tw = (tp @ T[:3, :3].T + T[:3, 3])[:, :2]; pts = np.c_[((tw[:, 0] - lo[0]) / C), n[1] - 1 - ((tw[:, 1] - lo[1]) / C)].astype(np.int32)
cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, (255, 0, 255), 1)
for d_ in range(0, int(s[-1]) + 1, 10):
    j = int(np.argmin(np.abs(s - d_))); cv2.circle(img, tuple(pts[j]), 3, (255, 0, 255), -1); cv2.putText(img, f'{d_}m', tuple(pts[j] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
grid_draw(img, lo, n[1]); cv2.imwrite(str(O / f'{run}_bev.png'), img)
np.savez(O / f'{run}_bev.npz', lo=lo, cell=C, T=T); print(run, 'BEV', img.shape, 'lo', np.round(lo, 2), f'경로 {s[-1]:.0f}m')
