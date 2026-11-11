"""v0.5 차량 키프레임에서 랜드마크 찾기: 추정 위치(손 촬영 nr2 배치)가 시야 안(거리 1~R m)인 키프레임 목록.
 --blue : 투영점 근처(반경 --rad px) 파란 덩어리 자동 → 화소 중심   --sheet : 투영점 표시 + 50px 격자 모음판(사람이 화소를 찍도록)
 --pick set:kf:u:v,... : 지정 화소로 광선 교차.   결과: 광선 교차(수평 2D, 이상치 제외) 위치·잔차, 손 촬영 배치별 어긋남, 대응점 항목
사용: lm_v05.py <name> <hand.json> <out_dir> [--R 15] [--rad 120] [--blue] [--sheet N] [--pick ...] [--cps_in a.json --cps_out b.json]"""
import sys, re, json, numpy as np, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; name, HJ, O = a[1], json.load(open(a[2])), Path(a[3]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: a[a.index(k) + 1] if k in a else d
R, RAD = float(opt('--R', 15)), float(opt('--rad', 120)); P = Path.home() / 'b_slam/deliver/map_v0.5'
c0 = np.array(HJ['hand']['nr2'])
def calib(p): return [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', p.read_text(), re.S).group(1).replace('\n', ' ').split(',')]
TR = {s: np.loadtxt(P / f'trajectory_{s}_camera_optical.txt', comments='#', ndmin=2) for s in ('bonnet_cw', 'bonnet_ccw', 'roof_cw', 'roof_ccw')}
def pose(s, i): r = TR[s][TR[s][:, 8] == i][0]; return Rot.from_quat(r[4:8]).as_matrix(), r[1:4]
vis = []
for s, tr in TR.items():
    for r in tr:
        Rw, c = Rot.from_quat(r[4:8]).as_matrix(), r[1:4]; pc = Rw.T @ (c0 - c)
        if not (1.0 < pc[2] < R): continue
        k = calib(P / f'keyframes_{s}/calib/{int(r[8])}.yaml'); u, v = k[0] * pc[0] / pc[2] + k[2], k[4] * pc[1] / pc[2] + k[5]
        if 20 < u < 620 and 20 < v < 460: vis.append((s, int(r[8]), u, v, pc[2]))
print(f'{name}: 추정 위치 ({c0[0]:+.2f},{c0[1]:+.2f}) 를 {R:g}m 안에서 본 v0.5 키프레임 {len(vis)}장', {s: sum(x[0] == s for x in vis) for s in TR})
obs = []
if '--blue' in a or '--red' in a:
    for s, i, u, v, z in vis:
        im = cv2.imread(str(P / f'keyframes_{s}/rgb/{i}.jpg')); h = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        if '--blue' in a: m = ((h[..., 0] > 95) & (h[..., 0] < 125) & (h[..., 1] > 110) & (h[..., 2] > 60))
        else: m = (((h[..., 0] < 7) | (h[..., 0] > 172)) & (h[..., 1] > 150) & (h[..., 2] > 70))
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m)
        cand = [q for q in range(1, n) if st[q, 4] >= (25 if '--blue' in a else 8) and np.hypot(*(cen[q] - (u, v))) < RAD]
        if not cand: continue
        q = max(cand, key=lambda q: st[q, 4]); obs.append((s, i, float(cen[q][0]), float(cen[q][1])))
        cv2.rectangle(im, (st[q, 0], st[q, 1]), (st[q, 0] + st[q, 2], st[q, 1] + st[q, 3]), (0, 255, 0), 2); cv2.circle(im, (int(u), int(v)), 6, (0, 0, 255), 2)
        cv2.putText(im, f'{s} {i}', (5, 20), 0, 0.6, (0, 255, 255), 2); cv2.imwrite(str(O / f'{name}_v05_{s}_{i}.jpg'), im)
if '--pick' in a:
    for t in opt('--pick', '').split(','):
        s, i, u, v = t.split(':'); obs.append((s, int(i), float(u), float(v)))
if '--sheet' in a:
    N = int(opt('--sheet', 9)); vs = sorted(vis, key=lambda x: x[4]); pick = [vs[int(q)] for q in np.linspace(0, len(vs) - 1, min(N, len(vs)))] if vs else []
    tiles = []
    for s, i, u, v, z in pick:
        im = cv2.imread(str(P / f'keyframes_{s}/rgb/{i}.jpg'))
        for g in range(0, 640, 50): cv2.line(im, (g, 0), (g, 479), (0, 255, 255), 1); cv2.putText(im, str(g), (g + 2, 12), 0, 0.35, (0, 255, 255), 1)
        for g in range(0, 480, 50): cv2.line(im, (0, g), (639, g), (0, 255, 255), 1); cv2.putText(im, str(g), (2, g - 2), 0, 0.35, (0, 255, 255), 1)
        cv2.circle(im, (int(u), int(v)), 8, (0, 0, 255), 2); cv2.putText(im, f'{s}:{i} z{z:.0f}m', (60, 30), 0, 0.7, (0, 0, 255), 2); tiles.append(im)
    tiles += [np.zeros((480, 640, 3), np.uint8)] * (-len(tiles) % 3)
    if tiles: cv2.imwrite(str(O / f'{name}_v05_sheet.jpg'), np.vstack([np.hstack(tiles[k:k + 3]) for k in range(0, len(tiles), 3)]))
if len(obs) >= 2:
    C_, D_ = [], []
    for s, i, u, v in obs:
        Rw, c = pose(s, i); k = calib(P / f'keyframes_{s}/calib/{i}.yaml'); d = Rw @ np.array([(u - k[2]) / k[0], (v - k[5]) / k[4], 1.0]); C_.append(c[:2]); D_.append(d[:2] / np.linalg.norm(d[:2]))
    C_, D_ = np.array(C_), np.array(D_); idx = np.arange(len(C_))
    def sol(ix):
        A = np.zeros((2, 2)); b = np.zeros(2)
        for c, d in zip(C_[ix], D_[ix]): M = np.eye(2) - np.outer(d, d); A += M; b += M @ c
        return np.linalg.solve(A, b)
    for _ in range(3):
        p = sol(idx); res = np.array([np.linalg.norm((np.eye(2) - np.outer(d, d)) @ (p - c)) for c, d in zip(C_, D_)]); idx = np.where(res < max(0.3, 2.5 * np.median(res[idx])))[0]
    ang = np.degrees(np.ptp(np.arctan2(D_[idx, 1], D_[idx, 0])))
    print(f'광선 {len(idx)}/{len(C_)}개 교차 → v0.5 ({p[0]:+.2f},{p[1]:+.2f}) | 잔차 중앙 {np.median(res[idx]):.2f}m 최대 {res[idx].max():.2f}m | 각도 폭 {ang:.0f}°')
    for tag, h in HJ['hand'].items(): print(f'   손 촬영({tag}) ({h[0]:+.2f},{h[1]:+.2f}) → 어긋남 ({p[0] - h[0]:+.2f},{p[1] - h[1]:+.2f}) = {np.hypot(p[0] - h[0], p[1] - h[1]):.2f}m')
    json.dump(dict(v05=p.round(3).tolist(), rays=int(len(idx)), angle=float(ang), resid_med=float(np.median(res[idx]))), open(O / f'{name}_v05.json', 'w'))
    if '--cps_in' in a:
        cps = json.load(open(opt('--cps_in', ''), encoding='utf-8')); h = HJ['hand']['ms']
        cps = [c for c in cps if c['name'] != name] + [dict(name=name, bev='fems5_' + HJ.get('sess', 's1_feit1'), x=round(h[0], 2), y=round(h[1], 2), dx=round(float(p[0] - h[0]), 2), dy=round(float(p[1] - h[1]), 2), type='point', w=1.0, group=name)]
        json.dump(cps, open(opt('--cps_out', ''), 'w', encoding='utf-8'), ensure_ascii=False, indent=1); print('   대응점 추가 →', opt('--cps_out', ''))
