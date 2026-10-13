"""C 전달용 바닥 구멍 지도와 목록. 사용: holes_pkg.py <package_dir>"""
import sys, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial import cKDTree
from scipy import ndimage
D = Path(sys.argv[1]); res = 0.10
V = np.asarray(o3d.io.read_triangle_mesh(str(D/'geometry_rawdepth_tsdf_5cm.ply')).vertices)
T = np.loadtxt(D/'trajectory_camera_link.txt', ndmin=2); T = T[np.argsort(T[:, 0])][:, 1:4]
s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(T[:, :2], axis=0), axis=1))]; ss = np.arange(0, s[-1], 0.1)
Ti = np.c_[[np.interp(ss, s, T[:, k]) for k in range(3)]].T; tree = cKDTree(Ti[:, :2])
d, j = tree.query(V[:, :2]); dz = Ti[j, 2] - V[:, 2]; G = V[(dz > 1.0) & (dz < 2.2) & (d < 6.5)]
lo = Ti[:, :2].min(0) - 8; hi = Ti[:, :2].max(0) + 8; W, H = np.ceil((hi - lo) / res).astype(int)
occ = np.zeros((H, W), bool); ij = np.floor((G[:, :2] - lo) / res).astype(int); occ[H - 1 - ij[:, 1], ij[:, 0]] = True
gy, gx = np.mgrid[0:H, 0:W]; X = lo[0] + (gx + .5) * res; Y = lo[1] + (H - 1 - gy + .5) * res
dc = tree.query(np.c_[X.ravel(), Y.ravel()])[0].reshape(H, W)
img = np.full((H, W, 3), 255, np.uint8); rows = []
for (a, b), ok_c, h_c, name in (((0, 3), (150, 150, 150), (0, 0, 230), '차로(궤적 0~3m)'), ((3, 6), (205, 205, 205), (0, 150, 255), '주변(궤적 3~6m)')):
    band = (dc >= a) & (dc < b); img[band & occ] = ok_c; img[band & ~occ] = h_c
    lab, n = ndimage.label(band & ~occ); sz = np.bincount(lab.ravel())[1:] * res * res
    for k in np.nonzero(sz >= 1.0)[0]:
        m = lab == k + 1; rows.append((name, X[m].mean(), Y[m].mean(), sz[k]))
    print(f'{name}: 빈칸 {(band & ~occ).sum()/band.sum()*100:.1f}%, 1㎡ 이상 {int((sz>=1).sum())}개')
# 궤적, 축척 막대(10m), 범례
for p, q in zip(Ti[:-1:5], Ti[5::5]):
    cv2.line(img, (int((p[0]-lo[0])/res), int(H-1-(p[1]-lo[1])/res)), (int((q[0]-lo[0])/res), int(H-1-(q[1]-lo[1])/res)), (0, 150, 0), 1)
img = cv2.resize(img, (W * 2, H * 2), interpolation=cv2.INTER_NEAREST); H2 = H * 2
cv2.line(img, (20, H2 - 20), (20 + 200, H2 - 20), (0, 0, 0), 3); cv2.putText(img, '10 m', (90, H2 - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
for i, (c, t) in enumerate((((150, 150, 150), 'road floor OK (0-3 m from path)'), ((0, 0, 230), 'road floor HOLE'), ((205, 205, 205), 'side floor OK (3-6 m)'), ((0, 150, 255), 'side floor HOLE'), ((0, 150, 0), 'trajectory'))):
    cv2.rectangle(img, (20, 20 + 24 * i), (38, 38 + 24 * i), c, -1); cv2.putText(img, t, (46, 35 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
for name, x, y, a in rows:  # 1㎡ 이상 구멍 번호
    pass
rows.sort(key=lambda r: -r[3])
for i, (name, x, y, a) in enumerate(rows, 1):
    u, v = int((x - lo[0]) / res * 2), int((H - 1 - (y - lo[1]) / res) * 2)
    cv2.putText(img, str(i), (u - 6, v + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
cv2.imwrite(str(D/'floor_holes.png'), img)
with open(D/'floor_holes.csv', 'w') as f:
    f.write('no,area_type,center_x_m,center_y_m,area_m2\n')
    for i, (name, x, y, a) in enumerate(rows, 1): f.write(f'{i},{"road" if name.startswith("차로") else "side"},{x:.1f},{y:.1f},{a:.1f}\n')
print(f'1㎡ 이상 구멍 {len(rows)}개 → floor_holes.png, floor_holes.csv'); print(open(D/'floor_holes.csv').read())
