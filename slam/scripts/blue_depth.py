"""v0.5 키프레임 파란 안내판 덩어리의 depth 기반 위치(광선 교차 결과와 대조)"""
import re, numpy as np, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
P = Path.home() / 'b_slam/deliver/map_v0.5'; O = Path.home() / 'b_slam/exports/landmarks'
def calib(p): return [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', p.read_text(), re.S).group(1).replace('\n', ' ').split(',')]
W = []
for f in sorted(O.glob('bluesign_v05_*.jpg')):
    _, _, s1, s2, i = f.stem.split('_'); s = f'{s1}_{s2}'; i = int(i)
    tr = np.loadtxt(P / f'trajectory_{s}_camera_optical.txt', comments='#', ndmin=2); r = tr[tr[:, 8] == i][0]
    im = cv2.imread(str(P / f'keyframes_{s}/rgb/{i}.jpg')); dp = cv2.imread(str(P / f'keyframes_{s}/depth/{i}.png'), -1).astype(float) / 1000; k = calib(P / f'keyframes_{s}/calib/{i}.yaml')
    h = cv2.cvtColor(im, cv2.COLOR_BGR2HSV); m = ((h[..., 0] > 95) & (h[..., 0] < 125) & (h[..., 1] > 110) & (h[..., 2] > 60)); ys, xs = np.nonzero(m)
    Rw = Rot.from_quat(r[4:8]).as_matrix(); c = r[1:4]; best = None
    for y, x in zip(ys, xs): pass
    z = dp[ys, xs]; ok = (z > 1) & (z < 20)
    # 덩어리별(가장 큰 덩어리 = 안내판으로 가정)
    n, lab, st, cen = cv2.connectedComponentsWithStats(m.astype(np.uint8)); q = 1 + np.argmax(st[1:, 4]); sel = (lab[ys, xs] == q) & ok
    if sel.sum() < 5: print(f, '깊이 없음'); continue
    zz = np.median(z[sel]); u, v = np.median(xs[sel]), np.median(ys[sel]); p = Rw @ np.array([(u - k[2]) / k[0] * zz, (v - k[5]) / k[4] * zz, zz]) + c
    W.append(p); print(f'{s} {i}: depth {zz:.2f}m (유효 {sel.sum()}px) → ({p[0]:+.2f},{p[1]:+.2f})')
W = np.array(W); print('depth 기반 중앙', np.median(W, 0).round(2), '흩어짐 최대', np.linalg.norm(W[:, :2] - np.median(W, 0)[:2], axis=1).max().round(2))
