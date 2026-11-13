"""복도 방향(x) 일치 확인: 두 구간(회차·가는 길/오는 길)의 벽 요철(문틀·기둥·로비 입구처럼 벽 선에서 6cm 넘게 벗어난 점)의 x 분포를
창(8m)마다 ±1.5m 범위에서 맞대어(정규화 상관) x 어긋남을 잼. 입력은 in_manhattan.py 가 만든 건물 좌표 내보내기(<run>m).
사용: in_xcheck.py <runA> <all|out|ret> <runB> <all|out|ret> [--dx 0 --dy 0 (B에 더할 이동)] [--out png]"""
import sys, re, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; E = Path.home() / 'b_slam/exports'; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
def load(run, leg, dx=0.0, dy=0.0):
    K = E / run / 'keyframes'; cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); cid = cid[np.argsort(cid[:, 0])]
    d0 = np.linalg.norm(cid[:, 1:3] - cid[0, 1:3], axis=1); kt = int(np.argmax(d0)); sel = {'all': np.arange(len(cid)), 'out': np.arange(0, kt + 1), 'ret': np.arange(kt + 1, len(cid))}[leg]; out = []
    for r in cid[sel]:
        i = int(r[8]); k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
        d = cv2.imread(str(K / f'kf_depth/{i}.png'), -1); v, u = np.mgrid[2:d.shape[0]:4, 2:d.shape[1]:4]; z = d[v, u].astype(np.float32) / 1000; m = (z > 0.4) & (z < 4.0)
        P = np.stack([(u[m] - k[2]) / k[0] * z[m], (v[m] - k[5]) / k[4] * z[m], z[m]], 1) @ Rot.from_quat(r[4:8]).as_matrix().T + r[1:4]
        out.append(P[(P[:, 2] > 0.3) & (P[:, 2] < 1.9), :2])
    P = np.vstack(out) + np.array([dx, dy]); return np.unique(np.round(P / 0.03).astype(np.int32), axis=0) * 0.03, cid[sel, 1:3] + np.array([dx, dy])
A, ca = load(a[1], a[2]); B, cb = load(a[3], a[4], opt('--dx', 0), opt('--dy', 0))
h, e = np.histogram(np.r_[A[:, 1], B[:, 1]], bins=np.arange(-6, 12, 0.02)); h = ndimage.uniform_filter1d(h.astype(float), 3); ctr = (e[:-1] + e[1:]) / 2
p1 = int(np.argmax(h)); h2 = h.copy(); h2[np.abs(ctr - ctr[p1]) < 1.5] = 0; p2 = int(np.argmax(h2)); Y = sorted([ctr[p1], ctr[p2]])
print(f'벽 선 y = {Y[0]:+.2f} · {Y[1]:+.2f}m (폭 {Y[1] - Y[0]:.2f}m)')
x0 = min(A[:, 0].min(), B[:, 0].min()) - 2; nb = int((max(A[:, 0].max(), B[:, 0].max()) + 2 - x0) / 0.05) + 1
def prof(P):
    s = np.zeros((2, nb)); cov = np.zeros(nb)
    for w, yw in enumerate(Y):
        dev = np.abs(P[:, 1] - yw); m = (dev > 0.06) & (dev < 0.5); np.add.at(s[w], ((P[m, 0] - x0) / 0.05).astype(int), 1)
        np.add.at(cov, ((P[dev < 0.5, 0] - x0) / 0.05).astype(int), 1)
    return ndimage.uniform_filter1d(np.sqrt(s), 3, axis=1), cov > 0
sa, ca_ = prof(A); sb, cb_ = prof(B); xs = x0 + (np.arange(nb) + 0.5) * 0.05; res = []
for xc in np.arange(np.ceil((x0 + 6) / 4) * 4, x0 + nb * 0.05 - 6, 4.0):
    w = np.abs(xs - xc) < 4.0; best = []
    for sh in np.arange(-30, 31):                       # B를 sh*5cm 옮김
        sbs = np.roll(sb, sh, axis=1); cbs = np.roll(cb_, sh); m = w & ca_ & cbs
        if m.sum() < 80: best.append(-1); continue
        x_, y_ = sa[:, m].ravel(), sbs[:, m].ravel(); x_ = x_ - x_.mean(); y_ = y_ - y_.mean(); den = np.sqrt((x_ ** 2).sum() * (y_ ** 2).sum())
        best.append(float((x_ * y_).sum() / den) if den > 1e-6 else -1)
    best = np.array(best); b = int(np.argmax(best))
    if best[b] <= 0: continue
    far = best.copy(); far[max(b - 6, 0):b + 7] = -1; res.append((xc, (b - 30) * 0.05, best[b], far.max(), best[30]))
    print(f'  x {xc - 4:5.0f}~{xc + 4:3.0f}m: B를 {(b - 30) * 0.05:+.2f}m 옮기면 최고 상관 {best[b]:+.2f} (현재 {best[30]:+.2f}, 다른 봉우리 {far.max():+.2f})')
good = [r for r in res if r[2] > 0.35 and r[2] - r[3] > 0.1]
if good: g = np.array(good); print(f'뚜렷한 창 {len(good)}개: x 어긋남 중앙 {np.median(g[:, 1]):+.2f}m, 범위 {g[:, 1].min():+.2f}~{g[:, 1].max():+.2f}m')
else: print('뚜렷한 창 없음')
if '--out' in a:
    Xo = np.array([x0, min(A[:, 1].min(), B[:, 1].min()) - 1]); sz = (int(nb), int((max(A[:, 1].max(), B[:, 1].max()) + 1 - Xo[1]) / 0.05) + 1); img = np.full((sz[1], sz[0], 3), 255, np.uint8)
    for P, col in ((A, (150, 150, 150)), (B, (0, 0, 255))):
        u = ((P[:, 0] - Xo[0]) / 0.05).astype(int); v = sz[1] - 1 - ((P[:, 1] - Xo[1]) / 0.05).astype(int); ok = (u >= 0) & (u < sz[0]) & (v >= 0) & (v < sz[1]); img[v[ok], u[ok]] = col
    for xg in np.arange(np.ceil(Xo[0] / 5) * 5, Xo[0] + sz[0] * 0.05, 5): cv2.putText(img, f'{xg:.0f}', (int((xg - Xo[0]) / 0.05), 12), 0, 0.4, (0, 120, 255), 1)
    cv2.imwrite(a[a.index('--out') + 1], img)
