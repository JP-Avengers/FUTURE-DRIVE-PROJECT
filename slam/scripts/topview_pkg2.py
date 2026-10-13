#!/usr/bin/env python3
"""패키지 탑뷰: 점군(밝기=높이) + 궤적 7종(v0.3 시계·반시계, 10/9 시계·반시계, 10/10 R11·R14·R12), 10m 막대·범례.
사용: topview_pkg.py <pkg_dir>"""
import sys
from pathlib import Path
import numpy as np, open3d as o3d, cv2
D = Path(sys.argv[1]); s = 10
P = np.asarray(o3d.io.read_point_cloud(str(D / 'cloud_3cm.ply')).points)
TR = [('trajectory_camera_link.txt', (40, 160, 40), 'v0.3 CW (10/5)'), ('trajectory_ccw_camera_link.txt', (220, 120, 30), 'v0.3 CCW (10/5)'),
      ('trajectory_1009_cw_camera_link.txt', (0, 140, 255), '10/9 CW'), ('trajectory_1009_ccw_camera_link.txt', (200, 60, 200), '10/9 CCW'),
      ('trajectory_1010_cw_camera_link.txt', (0, 0, 220), '10/10 CW (R11)'), ('trajectory_1010_ccw_camera_link.txt', (150, 0, 60), '10/10 CCW (R14)'),
      ('trajectory_1010_ccw2_camera_link.txt', (120, 120, 0), '10/10 CCW (R12)')]
lo = P[:, :2].min(0) - 2; hi = P[:, :2].max(0) + 2; w, h = ((hi - lo) * s).astype(int) + 1
img = np.full((h + 40, w, 3), 255, np.uint8)
z = P[:, 2]; zn = np.clip((z - np.percentile(z, 2)) / (np.percentile(z, 98) - np.percentile(z, 2) + 1e-9), 0, 1)
u = ((P[:, 0] - lo[0]) * s).astype(int); v = (h - 1 - (P[:, 1] - lo[1]) * s).astype(int); g = (215 - 170 * zn).astype(np.uint8); o = np.argsort(z)
img[v[o], u[o]] = np.stack([g[o]] * 3, 1)
for k, (f, col, lab) in enumerate(TR):
    if not (D / f).exists(): continue
    T = np.loadtxt(D / f, comments='#', ndmin=2); T = T[np.argsort(T[:, 0])][:, 1:3]
    q = np.c_[(T[:, 0] - lo[0]) * s, h - 1 - (T[:, 1] - lo[1]) * s].astype(int)
    for a, b in zip(q[:-1], q[1:]):
        if np.hypot(*(a - b)) < 5 * s: cv2.line(img, tuple(a), tuple(b), col, 2)
    cv2.rectangle(img, (10, 10 + 22 * k), (30, 26 + 22 * k), col, -1); cv2.putText(img, lab, (36, 24 + 22 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
cv2.line(img, (10, h + 20), (10 + 10 * s, h + 20), (0, 0, 0), 3); cv2.putText(img, '10 m', (10, h + 36), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
cv2.imwrite(str(D / 'topview.png'), img); print(D / 'topview.png', img.shape)
