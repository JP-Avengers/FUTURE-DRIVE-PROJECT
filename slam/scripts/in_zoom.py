"""두 구간의 벽 점을 2cm/px 로 확대해 겹쳐 그림(A 회색, B 빨강) — x 구간별 띠. 사용: in_zoom.py <out.png> <runA> <legA> <runB> <legB> [--dx --dy] --win x0:x1 ..."""
import sys, re, numpy as np, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; E = Path.home() / 'b_slam/exports'; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
def load(run, leg, dx=0.0, dy=0.0):
    K = E / run / 'keyframes'; cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); cid = cid[np.argsort(cid[:, 0])]
    d0 = np.linalg.norm(cid[:, 1:3] - cid[0, 1:3], axis=1); kt = int(np.argmax(d0)); sel = {'all': np.arange(len(cid)), 'out': np.arange(0, kt + 1), 'ret': np.arange(kt + 1, len(cid))}[leg]; out = []
    for r in cid[sel]:
        i = int(r[8]); k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
        d = cv2.imread(str(K / f'kf_depth/{i}.png'), -1); v, u = np.mgrid[1:d.shape[0]:2, 1:d.shape[1]:2]; z = d[v, u].astype(np.float32) / 1000; m = (z > 0.4) & (z < 3.5)
        P = np.stack([(u[m] - k[2]) / k[0] * z[m], (v[m] - k[5]) / k[4] * z[m], z[m]], 1) @ Rot.from_quat(r[4:8]).as_matrix().T + r[1:4]
        out.append(P[(P[:, 2] > 0.3) & (P[:, 2] < 1.9), :2])
    return np.vstack(out) + np.array([dx, dy])
A = load(a[2], a[3]); B = load(a[4], a[5], opt('--dx', 0), opt('--dy', 0)); C = 0.02; rows = []
for w in [x for i, x in enumerate(a) if a[i - 1] == '--win']:
    x0, x1 = map(float, w.split(':')); y0, y1 = -2.2, 2.6; W_, H_ = int((x1 - x0) / C), int((y1 - y0) / C); img = np.full((H_, W_, 3), 255, np.uint8)
    for P, ch in ((A, 0), (B, 1)):
        m = (P[:, 0] >= x0) & (P[:, 0] < x1) & (P[:, 1] >= y0) & (P[:, 1] < y1); cnt = np.zeros((H_, W_), np.int32); np.add.at(cnt, (H_ - 1 - ((P[m, 1] - y0) / C).astype(int), ((P[m, 0] - x0) / C).astype(int)), 1); on = cnt >= 3
        if ch == 0: img[on] = (150, 150, 150)
        else: img[on] = np.where((img[on] == 150).all(1)[:, None], np.array([[90, 0, 140]]), np.array([[0, 0, 255]])).astype(np.uint8)       # 겹친 곳 = 자주색
    for xg in np.arange(np.ceil(x0), x1, 1.0): cv2.line(img, (int((xg - x0) / C), 0), (int((xg - x0) / C), 8), (0, 120, 255), 1); cv2.putText(img, f'{xg:.0f}', (int((xg - x0) / C) + 2, 20), 0, 0.45, (0, 120, 255), 1)
    rows.append(img)
w = max(r.shape[1] for r in rows); cv2.imwrite(a[1], np.vstack([np.pad(r, ((0, 6), (0, w - r.shape[1]), (0, 0)), constant_values=0) for r in rows]))
