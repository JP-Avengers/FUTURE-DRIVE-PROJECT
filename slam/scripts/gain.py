"""반시계 메시가 v0.2 바닥 구멍을 얼마나 메우나. 변환 = 주차선 2D(yaw+xy) + 바닥 높이 맞춤 z."""
import numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial import cKDTree
from scipy import ndimage
H = Path.home()/'b_slam/exports'; D = Path.home()/'b_slam/deliver/map_v0.2'; res = 0.10
p = np.load(H/'run20_to_run18_paint2d.npy'); R2, t2 = p[:4].reshape(2, 2), p[4:]
V0 = np.asarray(o3d.io.read_triangle_mesh(str(D/'geometry_rawdepth_tsdf_5cm.ply')).vertices)
V1 = np.asarray(o3d.io.read_triangle_mesh(str(H/'run20/tsdf_rawdepth_5cm_min1m.ply')).vertices)
V1 = np.c_[V1[:, :2] @ R2.T + t2, V1[:, 2]]
T = np.loadtxt(D/'trajectory_camera_link.txt', ndmin=2); T = T[np.argsort(T[:, 0])][:, 1:4]
s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(T[:, :2], axis=0), axis=1))]; ss = np.arange(0, s[-1], 0.1)
Ti = np.c_[[np.interp(ss, s, T[:, k]) for k in range(3)]].T; tree = cKDTree(Ti[:, :2])
def ground(V):
    d, j = tree.query(V[:, :2]); dz = Ti[j, 2] - V[:, 2]; return V[(dz > 1.0) & (dz < 2.2)], d
G0, _ = ground(V0)
# z 맞춤: 두 바닥이 겹치는 곳의 높이 차 중앙값
g1all, _ = ground(V1); dd, jj = cKDTree(G0[:, :2]).query(g1all[:, :2]); m = dd < 0.1
dz = np.median(G0[jj[m], 2] - g1all[m, 2]); V1[:, 2] += dz; G1, _ = ground(V1)
print(f'반시계 메시 z 보정 {dz*100:+.1f}cm, 겹치는 바닥 높이 차 (보정 후) 중앙 {np.median(np.abs(G0[jj[m],2]-g1all[m,2]-dz))*100:.1f}cm')
lo = Ti[:, :2].min(0) - 12; hi = Ti[:, :2].max(0) + 12; W, Hh = np.ceil((hi - lo) / res).astype(int)
def occ(G):
    o = np.zeros((Hh, W), bool); ij = np.floor((G[:, :2] - lo) / res).astype(int); ok = (ij >= 0).all(1) & (ij[:, 0] < W) & (ij[:, 1] < Hh)
    o[Hh - 1 - ij[ok, 1], ij[ok, 0]] = True; return o
O0, O1 = occ(G0), occ(G1)
gy, gx = np.mgrid[0:Hh, 0:W]; dc = tree.query(np.c_[lo[0] + (gx.ravel() + .5) * res, lo[1] + (Hh - 1 - gy.ravel() + .5) * res])[0].reshape(Hh, W)
for a, b in ((0, 3), (3, 6)):
    band = (dc >= a) & (dc < b); h0 = band & ~O0; filled = h0 & O1
    lab, n = ndimage.label(h0); big = np.zeros_like(h0)
    sz = np.bincount(lab.ravel())[1:] * res * res
    for k in np.nonzero(sz >= 1)[0]: big |= lab == k + 1
    print(f'궤적 {a}~{b}m: v0.2 빈칸 {h0.sum()/band.sum()*100:.1f}% → 반시계 추가 시 {(h0 & ~O1).sum()/band.sum()*100:.1f}% '
          f'(메워지는 면적 {filled.sum()*res*res:.0f}㎡ / {h0.sum()*res*res:.0f}㎡; 1㎡ 이상 큰 구멍 중 {(big & O1).sum()/max(big.sum(),1)*100:.0f}% 메워짐)')
new = O1 & (dc >= 6); print(f'v0.2 궤적에서 6m 밖, 반시계만 본 새 바닥: {new.sum()*res*res:.0f}㎡')
img = np.full((Hh, W, 3), 255, np.uint8); band6 = dc < 6
img[band6 & O0] = (175, 175, 175); img[band6 & ~O0 & O1] = (0, 170, 0); img[band6 & ~O0 & ~O1] = (0, 0, 230); img[new] = (230, 160, 60)
img = cv2.resize(img, (W * 2, Hh * 2), interpolation=cv2.INTER_NEAREST)
for i, (c, tx) in enumerate((((175, 175, 175), 'v0.2 floor'), ((0, 170, 0), 'v0.2 hole filled by CCW'), ((0, 0, 230), 'still hole'), ((230, 160, 60), 'new floor only CCW saw (>6 m from CW path)'))):
    cv2.rectangle(img, (15, 15 + 24 * i), (33, 33 + 24 * i), c, -1); cv2.putText(img, tx, (40, 30 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
cv2.line(img, (15, img.shape[0] - 15), (215, img.shape[0] - 15), (0, 0, 0), 3); cv2.putText(img, '10 m', (90, img.shape[0] - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
cv2.imwrite('/home/owner7/b_slam/exports/tmp/ccw_gain.png', img)
