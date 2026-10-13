"""v0.3 floor_holes.csv 구멍별로 새 패키지 형상 메시 바닥이 채운 비율 (holes_pkg.py와 같은 방식: v0.3 시계 궤적 기준 바닥 높이 띠, 10cm 칸)."""
import sys
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
from scipy import ndimage
V3 = Path.home() / 'b_slam/deliver/map_v0.3'; NEW = Path(sys.argv[1]); res = 0.10
T = np.loadtxt(V3 / 'trajectory_camera_link.txt', ndmin=2); T = T[np.argsort(T[:, 0])][:, 1:4]
s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(T[:, :2], axis=0), axis=1))]; ss = np.arange(0, s[-1], 0.1)
Ti = np.c_[[np.interp(ss, s, T[:, k]) for k in range(3)]].T; tree = cKDTree(Ti[:, :2])
lo = Ti[:, :2].min(0) - 8; hi = Ti[:, :2].max(0) + 8; W, H = np.ceil((hi - lo) / res).astype(int)
def occ(mesh):
    V = np.asarray(o3d.io.read_triangle_mesh(str(mesh)).vertices); d, j = tree.query(V[:, :2]); dz = Ti[j, 2] - V[:, 2]; G = V[(dz > 1.0) & (dz < 2.2) & (d < 6.5)]
    m = np.zeros((H, W), bool); ij = np.floor((G[:, :2] - lo) / res).astype(int); m[H - 1 - ij[:, 1], ij[:, 0]] = True; return m
o3, o4 = occ(V3 / 'geometry_rawdepth_tsdf_5cm.ply'), occ(NEW / 'geometry_rawdepth_tsdf_5cm.ply')
gy, gx = np.mgrid[0:H, 0:W]; X = lo[0] + (gx + .5) * res; Y = lo[1] + (H - 1 - gy + .5) * res; dc = tree.query(np.c_[X.ravel(), Y.ravel()])[0].reshape(H, W)
holes = np.zeros((H, W), int)
for a, b in ((0, 3), (3, 6)):
    lab, n = ndimage.label((dc >= a) & (dc < b) & ~o3); sz = np.bincount(lab.ravel()); holes = np.where((lab > 0) & (sz[lab] * res * res >= 1.0), lab + (100000 if a else 0), holes)
ids = np.unique(holes[holes > 0]); cen = np.array([[X[holes == i].mean(), Y[holes == i].mean()] for i in ids]); area = np.array([(holes == i).sum() * res * res for i in ids])
print('| v0.3 번호 | 위치 (x, y) | 면적 | 채워진 비율 |'); print('|---|---|---|---|')
for l in (V3 / 'floor_holes.csv').read_text().splitlines()[1:]:
    n, typ, x, y, a = l.split(','); x, y, a = float(x), float(y), float(a)
    k = np.argmin(np.hypot(cen[:, 0] - x, cen[:, 1] - y) + 0.5 * np.abs(area - a)); m = holes == ids[k]
    print(f'| {n} | ({x:+.1f}, {y:+.1f}) | {a}㎡ | {(o4 & m).sum() / m.sum() * 100:.0f}% |')
