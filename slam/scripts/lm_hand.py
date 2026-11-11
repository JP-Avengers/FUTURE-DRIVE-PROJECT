"""손 촬영 랜드마크 위치: 묶음 맵 IT1wall 세션(s1) 키프레임 중 [t0,t1]초(녹화 시작 기준)
 --blue : 파란 덩어리 자동(+depth)   --pick kf:u:v[,kf:u:v...] : 지정 화소(+주변 depth 중앙)
 배치 3가지(ms 강체 이전, nr1, nr2)의 세계 위치를 출력. --sheet 면 해당 구간 키프레임 모음판(격자 50px) 저장.
사용: lm_hand.py <name> <t0> <t1> <out_dir> [--blue] [--pick ...] [--sheet]"""
import sys, re, json, numpy as np, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; name, t0, t1, O = a[1], float(a[2]), float(a[3]), Path(a[4]); O.mkdir(parents=True, exist_ok=True)
# 회차별 녹화 시작 시각(동기 자료의 record 시각 - 경과 초) → 묶음 맵 세션
STARTS = {'s1_feit1': 1794124668.845110149, 's0_feone': 1794125588.140298765, 's3_fe422a': 1794124955.963293117,
          's4_fe422b': 1794126399.139781564, 's5_feit1w': 1794125224.199861750, 's2_fecen': 1794126170.003673757}
SESS = a[a.index('--sess') + 1] if '--sess' in a else 's1_feit1'
E = Path.home() / 'b_slam/exports'; T0 = STARTS[SESS]; K = E / 'fems5/keyframes'
PLS = a[a.index('--nr') + 1] if '--nr' in a else 'nr3'
def calib(p): return [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', p.read_text(), re.S).group(1).replace('\n', ' ').split(',')]
PL = {'ms': (E / 'fems5/traj_camera_id_camera_poses.txt', np.load(E / 'align_fems5_ms_from_fecen.npy')),
      'nr2': (E / f'{PLS}/fems5c/traj_camera_id_camera_poses.txt', np.eye(4))}     # 'nr2' 키 = 최신 비강체 배치(--nr, 기본 nr3)
CID = {k: np.loadtxt(f, comments='#', ndmin=2) for k, (f, _) in PL.items()}
c0 = CID['ms']; sel = np.where((c0[:, 0] > T0 + t0) & (c0[:, 0] < T0 + t1))[0]
def world(tag, j, u, v, z):
    r = CID[tag][j]; T = PL[tag][1]; k = calib(K / f'kf_calib/{int(r[8])}.yaml')
    p = np.array([(u - k[2]) / k[0] * z, (v - k[5]) / k[4] * z, z]); return T[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix() @ p + T[:3, :3] @ r[1:4] + T[:3, 3]
obs = []                                                   # (j, u, v, z)
if '--blue' in a or '--red' in a:
    for j in sel:
        i = int(c0[j, 8]); im = cv2.imread(str(K / f'kf_rgb/{i}.jpg')); dp = cv2.imread(str(K / f'kf_depth/{i}.png'), -1); h = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        if '--blue' in a: m = ((h[..., 0] > 95) & (h[..., 0] < 125) & (h[..., 1] > 110) & (h[..., 2] > 60))
        else: m = (((h[..., 0] < 7) | (h[..., 0] > 172)) & (h[..., 1] > 170) & (h[..., 2] > 90))      # 진한 빨강(벽돌 보도·단풍보다 채도 높음)
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, cen = cv2.connectedComponentsWithStats(m)
        if n < 2: continue
        q = 1 + np.argmax(st[1:, 4]);
        if st[q, 4] < 300: continue
        ys, xs = np.nonzero(lab == q); z = dp[ys, xs] / 1000.0; ok = (z > 0.3) & (z < 10)
        if ok.sum() < 50: continue
        obs.append((j, float(np.median(xs[ok])), float(np.median(ys[ok])), float(np.median(z[ok]))))
        cv2.rectangle(im, (st[q, 0], st[q, 1]), (st[q, 0] + st[q, 2], st[q, 1] + st[q, 3]), (0, 255, 0), 2); cv2.imwrite(str(O / f'{name}_hand_kf{i}.jpg'), im)
if '--pick' in a:
    for s in a[a.index('--pick') + 1].split(','):
        i, u, v = s.split(':'); i, u, v = int(i), float(u), float(v); j = int(np.where(c0[:, 8] == i)[0][0])
        dp = cv2.imread(str(K / f'kf_depth/{i}.png'), -1).astype(float) / 1000; w = dp[int(v) - 4:int(v) + 5, int(u) - 4:int(u) + 5]; w = w[(w > 0.3) & (w < 10)]
        obs.append((j, u, v, float(np.median(w))))
if '--sheet' in a:
    tiles = []
    for j in sel[::max(1, len(sel) // 12)][:12]:
        i = int(c0[j, 8]); im = cv2.imread(str(K / f'kf_rgb/{i}.jpg'))
        for g in range(0, 640, 50): cv2.line(im, (g, 0), (g, 479), (0, 255, 255), 1); cv2.putText(im, str(g), (g + 2, 12), 0, 0.35, (0, 255, 255), 1)
        for g in range(0, 480, 50): cv2.line(im, (0, g), (639, g), (0, 255, 255), 1); cv2.putText(im, str(g), (2, g - 2), 0, 0.35, (0, 255, 255), 1)
        cv2.putText(im, f'kf{i} t+{c0[j, 0] - T0:.1f}s', (60, 30), 0, 0.8, (0, 0, 255), 2); tiles.append(im)
    tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 3)
    cv2.imwrite(str(O / f'{name}_hand_sheet.jpg'), np.vstack([np.hstack(tiles[k:k + 3]) for k in range(0, len(tiles), 3)]))
out = {}
for tag in PL:
    W = np.array([world(tag, j, u, v, z) for j, u, v, z in obs]) if obs else np.zeros((0, 3))
    if len(W): out[tag] = np.median(W, 0).round(3).tolist(); print(f'{name} 손 촬영({tag}): ({out[tag][0]:+.2f},{out[tag][1]:+.2f},{out[tag][2]:+.2f}) 관측 {len(W)}, 흩어짐 최대 {np.linalg.norm(W[:, :2] - np.median(W, 0)[:2], axis=1).max():.2f}m')
for j in sel[::max(1, len(sel) // 6)]:
    p = CID['ms'][j, 1:4] @ PL['ms'][1][:3, :3].T + PL['ms'][1][:3, 3]; print(f'   카메라 kf{int(c0[j, 8])} t+{c0[j, 0] - T0:.1f}s ms ({p[0]:+.1f},{p[1]:+.1f}) nr2 ({CID["nr2"][j, 1]:+.1f},{CID["nr2"][j, 2]:+.1f})')
json.dump(dict(name=name, sess=SESS, hand=out, obs=[[int(c0[j, 8]), u, v, z] for j, u, v, z in obs]), open(O / f'{name}_hand.json', 'w'), indent=1)
