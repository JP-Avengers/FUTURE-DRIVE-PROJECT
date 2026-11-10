"""v0.5 키프레임 지정 화소(set:kf:u:v,...)의 depth(주변 9x9 중앙) → 세계 위치. 사용: pick_depth.py <picks>"""
import sys, re, numpy as np, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
P = Path.home() / 'b_slam/deliver/map_v0.5'; W = []
def calib(p): return [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', p.read_text(), re.S).group(1).replace('\n', ' ').split(',')]
for t in sys.argv[1].split(','):
    s, i, u, v = t.split(':'); i, u, v = int(i), float(u), float(v)
    tr = np.loadtxt(P / f'trajectory_{s}_camera_optical.txt', comments='#', ndmin=2); r = tr[tr[:, 8] == i][0]
    dp = cv2.imread(str(P / f'keyframes_{s}/depth/{i}.png'), -1).astype(float) / 1000; k = calib(P / f'keyframes_{s}/calib/{i}.yaml')
    w = dp[int(v) - 4:int(v) + 5, int(u) - 4:int(u) + 5]; w = w[(w > 0.5) & (w < 25)]
    if len(w) < 5: print(s, i, 'depth 없음'); continue
    z = np.median(w); p = Rot.from_quat(r[4:8]).as_matrix() @ np.array([(u - k[2]) / k[0] * z, (v - k[5]) / k[4] * z, z]) + r[1:4]; W.append(p)
    print(f'{s} {i} ({u:.0f},{v:.0f}): depth {z:.2f}m (9x9 유효 {len(w)}) → ({p[0]:+.2f},{p[1]:+.2f},{p[2]:+.2f})')
W = np.array(W); print('depth 기반 중앙', np.median(W, 0).round(2))
