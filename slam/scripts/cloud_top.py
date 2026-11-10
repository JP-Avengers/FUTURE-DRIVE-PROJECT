"""점군 위에서 본 그림(10cm 칸, 칸마다 바닥 높이 ±0.3m 안의 점 색 평균) → <out>_raw.png + <out>.npz(lo)
fe_bev.py 출력과 같은 형식이라 cp_pair_ms.py 에 넣어 정사 영상·손 촬영 BEV와 나란히 볼 수 있음.
사용: cloud_top.py <cloud.ply> <out_prefix> [--T T.npy] [--floor]  (--floor: v0.5 바닥 높이 지도 기준 ±0.3m 점만)"""
import sys, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy import ndimage
a = sys.argv; pc = o3d.io.read_point_cloud(a[1]); OUT = a[2]; C = 0.1
X = np.asarray(pc.points); col = np.asarray(pc.colors)
if '--T' in a: T = np.load(a[a.index('--T') + 1]); X = X @ T[:3, :3].T + T[:3, 3]
if '--floor' in a:
    F = np.load(Path.home() / 'b_slam/deliver/map_v0.5/floor_height.npz'); G, lo_, c_ = F['grid'].astype(float), F['lo'], float(F['cell'])
    h = X[:, 2] - ndimage.map_coordinates(G, [(X[:, 0] - lo_[0]) / c_ - 0.5, (X[:, 1] - lo_[1]) / c_ - 0.5], order=1, mode='nearest')
    k = np.abs(h) < 0.3; X, col = X[k], col[k]
lo = X[:, :2].min(0) - 1; n = ((X[:, :2].max(0) + 1 - lo) / C).astype(int) + 1
ix = ((X[:, 0] - lo[0]) / C).astype(int); iy = ((X[:, 1] - lo[1]) / C).astype(int); lin = ix * n[1] + iy
cnt = np.bincount(lin, minlength=n[0] * n[1]); img = np.zeros((n[1], n[0], 3), np.uint8)
for ch in range(3):
    s_ = np.bincount(lin, weights=col[:, 2 - ch], minlength=n[0] * n[1]); v = np.where(cnt > 0, s_ / np.maximum(cnt, 1), 0).reshape(n[0], n[1])
    img[:, :, ch] = (np.clip(v, 0, 1) * 255).astype(np.uint8).T[::-1]
cv2.imwrite(OUT + '_bev_raw.png', img); np.savez(OUT + '_bev.npz', lo=lo, cell=C); print(OUT, img.shape, '점', len(X))
