"""v0.5 키프레임 소화전 관측(hydrant.json)을 광선 교차(수평 2D 최소제곱, 이상치 제외)로 위치 추정 → 대응점 JSON 추가.
사용: hydrant_tri.py <hydrant_dir> <cps_in.json> <cps_out.json>"""
import sys, re, json, numpy as np, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
H, CI, CO = Path(sys.argv[1]), sys.argv[2], sys.argv[3]; P = Path.home() / 'b_slam/deliver/map_v0.5'
J = json.load(open(H / 'hydrant.json')); hand = np.array(J['hand']['ms'])
def calib(p): return [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', p.read_text(), re.S).group(1).replace('\n', ' ').split(',')]
red = lambda bgr: (lambda h: (((h[..., 0] < 8) | (h[..., 0] > 170)) & (h[..., 1] > 120) & (h[..., 2] > 70)))(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV))
C_, D_ = [], []
for f in J['v05']:
    s, i = f['set'], f['kf']; tr = np.loadtxt(P / f'trajectory_{s}_camera_optical.txt', comments='#', ndmin=2); r = tr[tr[:, 8] == i][0]
    im = cv2.imread(str(P / f'keyframes_{s}/rgb/{i}.jpg')); k = calib(P / f'keyframes_{s}/calib/{i}.yaml')
    Rw = Rot.from_quat(r[4:8]).as_matrix(); c = r[1:4]; pc = Rw.T @ (np.array(f['world']) - c); u0, v0 = k[0] * pc[0] / pc[2] + k[2], k[4] * pc[1] / pc[2] + k[5]
    m = red(im).astype(np.uint8); n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    j = 1 + np.argmin([np.hypot(*(cen[q] - (u0, v0))) for q in range(1, n)]); u, v = cen[j]
    d = Rw @ np.array([(u - k[2]) / k[0], (v - k[5]) / k[4], 1.0]); d2 = d[:2] / np.linalg.norm(d[:2]); C_.append(c[:2]); D_.append(d2)
C_, D_ = np.array(C_), np.array(D_)
def solve(idx):
    A = np.zeros((2, 2)); b = np.zeros(2)
    for c, d in zip(C_[idx], D_[idx]): M = np.eye(2) - np.outer(d, d); A += M; b += M @ c
    return np.linalg.solve(A, b)
idx = np.arange(len(C_))
for _ in range(3):
    p = solve(idx); r_ = np.array([np.linalg.norm((np.eye(2) - np.outer(d, d)) @ (p - c)) for c, d in zip(C_, D_)])
    idx = np.where(r_ < max(0.3, 2.5 * np.median(r_[idx])))[0]
ang = np.degrees(np.ptp(np.arctan2(D_[idx, 1], D_[idx, 0])))
print(f'광선 {len(idx)}/{len(C_)}개 교차: ({p[0]:+.2f},{p[1]:+.2f}) | 광선 잔차 중앙 {np.median(r_[idx]):.2f}m 최대 {r_[idx].max():.2f}m | 광선 각도 폭 {ang:.0f}°')
dep = np.median(np.array([f['world'][:2] for f in J['v05']]), 0); print(f'  (depth 중앙값 위치 ({dep[0]:+.2f},{dep[1]:+.2f}) 와 {np.linalg.norm(p - dep):.2f}m 차이)')
for tag, h in J['hand'].items(): print(f'  손 촬영({tag}) ({h[0]:+.2f},{h[1]:+.2f}) → v0.5 어긋남 ({p[0] - h[0]:+.2f},{p[1] - h[1]:+.2f}) = {np.hypot(p[0] - h[0], p[1] - h[1]):.2f}m')
cps = json.load(open(CI, encoding='utf-8'))
cps.append(dict(name='hydrant', bev='fems5_s1_feit1', x=round(float(hand[0]), 2), y=round(float(hand[1]), 2), dx=round(float(p[0] - hand[0]), 2), dy=round(float(p[1] - hand[1]), 2), type='point', w=1.0, group='hydrant'))
json.dump(cps, open(CO, 'w', encoding='utf-8'), ensure_ascii=False, indent=1); json.dump(dict(v05=p.round(3).tolist(), rays=int(len(idx))), open(H / 'hydrant_tri.json', 'w'))
