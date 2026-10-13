#!/usr/bin/env python3
"""paint_xy.npy 여러 개를 합쳐 탑뷰 PNG (1px=5cm, 5m 격자, 좌표 눈금). 사용: paint_npy_view.py <out.png> <run_dir_name>..."""
import sys
from pathlib import Path
import numpy as np, cv2
H = Path.home() / 'b_slam/exports'; out = sys.argv[1]
P = np.vstack([np.load(H / r / 'paint_xy.npy') for r in sys.argv[2:]])
s = 20; lo = np.floor(P.min(0)) - 1; hi = np.ceil(P.max(0)) + 1; w, h = ((hi - lo) * s).astype(int) + 1
img = np.full((h, w, 3), 255, np.uint8)
u = ((P[:, 0] - lo[0]) * s).astype(int); v = (h - 1 - (P[:, 1] - lo[1]) * s).astype(int); img[v, u] = 0
for x in np.arange(np.ceil(lo[0] / 5) * 5, hi[0], 5):
    X = int((x - lo[0]) * s); img[:, X] = (180, 180, 255); cv2.putText(img, f'{x:.0f}', (X + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 200), 1)
for y in np.arange(np.ceil(lo[1] / 5) * 5, hi[1], 5):
    Y = int(h - 1 - (y - lo[1]) * s); img[Y, :] = (180, 180, 255); cv2.putText(img, f'{y:.0f}', (2, Y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 200), 1)
cv2.imwrite(out, img); print(out, img.shape, '격자 5m, 숫자 = 맵 좌표(m)')
