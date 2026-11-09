"""대응점 고르기용 위에서 본 그림: 점군을 높이 색(바닥 근처 회색, 높을수록 진하게)으로, 5m 격자와 10m마다 좌표 숫자.
사용: topview_grid.py <cloud.ply> <out.png> [--floor auto|z] [--cell 0.1] [--T T.npy]"""
import sys, numpy as np, open3d as o3d, cv2
a = sys.argv; f, out = a[1], a[2]; cell = float(a[a.index('--cell') + 1]) if '--cell' in a else 0.1
P = np.asarray(o3d.io.read_point_cloud(f).voxel_down_sample(0.05).points)
if '--T' in a: T = np.load(a[a.index('--T') + 1]); P = P @ T[:3, :3].T + T[:3, 3]
z0 = np.percentile(P[:, 2], 15) if '--floor' not in a or a[a.index('--floor') + 1] == 'auto' else float(a[a.index('--floor') + 1])
h = P[:, 2] - z0; P = P[(h > -0.5) & (h < 6)]; h = h[(h > -0.5) & (h < 6)]
lo = np.floor(P[:, :2].min(0) / 5) * 5 - 5; hi = np.ceil(P[:, :2].max(0) / 5) * 5 + 5
W, H = ((hi - lo) / cell).astype(int)
img = np.full((H, W, 3), 255, np.uint8); ix = ((P[:, 0] - lo[0]) / cell).astype(int); iy = H - 1 - ((P[:, 1] - lo[1]) / cell).astype(int)
o = np.argsort(h); col = np.zeros((len(h), 3))
fl = h < 0.12; col[fl] = (200, 200, 200)
k = ~fl; t = np.clip(h[k] / 4, 0, 1); col[k] = np.c_[255 * (1 - t), 120 * (1 - t), 40 + 0 * t]  # BGR: 낮으면 파랑, 높으면 검정
img[iy[o], ix[o]] = col[o]
for x in np.arange(lo[0], hi[0] + 1e-6, 5):
    u = int((x - lo[0]) / cell); cv2.line(img, (u, 0), (u, H - 1), (0, 200, 0) if x % 10 == 0 else (180, 230, 180), 1)
    if x % 10 == 0: cv2.putText(img, f'{x:.0f}', (u + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 120, 0), 1)
for y in np.arange(lo[1], hi[1] + 1e-6, 5):
    v = H - 1 - int((y - lo[1]) / cell); cv2.line(img, (0, v), (W - 1, v), (0, 200, 0) if y % 10 == 0 else (180, 230, 180), 1)
    if y % 10 == 0: cv2.putText(img, f'{y:.0f}', (2, v - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 120, 0), 1)
cv2.imwrite(out, img); print(out, img.shape, 'lo', lo, 'hi', hi, 'z0', round(z0, 2))
