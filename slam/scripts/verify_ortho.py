"""정합 좌우 위치 검증(바닥 정사 영상): 키프레임 자세(정합 T 적용)에서 광선-바닥 교차 → v0.5 ground_ortho 색을 읽어 '이 자리에서 보일 바닥'을 합성,
실제 사진과 선 무늬(흰 톱햇) 상관을 잰다. T 를 ±0.6m·요 ±2° 흔들어 현재 T 가 최고 근처인지 판정.
사용: verify_ortho.py <run> <T.npy> <out_dir> [--kf 16] [--from 0] [--to 1e9]"""
import sys, re, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, tf, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
NK = int(opt('--kf', 16)); S0, S1 = opt('--from', 0), opt('--to', 1e9); ST = 2
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'; T = np.load(tf)
F = np.load(P / 'floor_height.npz'); G, lo, cell = F['grid'].astype(np.float32), F['lo'], float(F['cell'])
Q = np.load(P / 'outer/ground_ortho.npz'); RGB, qlo, qc = Q['rgb'], Q['lo'], float(Q['cell'])
Gq = RGB.astype(np.float32).mean(-1); valid_q = (RGB.sum(-1) > 0).astype(np.float32)
def fh(x, y): return ndimage.map_coordinates(G, [(x - lo[0]) / cell - 0.5, (y - lo[1]) / cell - 0.5], order=1, mode='nearest')
def oq(x, y, arr): return ndimage.map_coordinates(arr, [(x - qlo[0]) / qc - 0.5, (y - qlo[1]) / qc - 0.5], order=1, mode='constant', cval=0)
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); K = E / run / 'keyframes'
tp = cid[:, 1:4]; s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tp, axis=0), axis=1))]
idx = np.where((s >= S0) & (s <= S1))[0]; pi = idx[np.linspace(0, len(idx) - 1, NK).astype(int)]; pick = cid[pi]
def tophat(g, k=15): return cv2.subtract(g, cv2.morphologyEx(g, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))).astype(np.float32)
frames = []
for j, r in zip(pi, pick):
    i = int(r[8]); txt = (K / f'kf_calib/{i}.yaml').read_text()
    k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', txt, re.S).group(1).replace('\n', ' ').split(',')]
    im = cv2.imread(str(K / f'kf_rgb/{i}.jpg')); h, w = im.shape[:2]
    v, u = np.mgrid[0:h:ST, 0:w:ST]; dc = np.stack([(u - k[2]) / k[0], (v - k[5]) / k[4], np.ones(u.shape)], -1)
    frames.append((i, im, dc, Rot.from_quat(r[4:8]).as_matrix(), r[1:4], s[j], tophat(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)[::ST, ::ST])))
def synth(Tm, f):
    i, im, dc, Ro, c, sm, _ = f
    Rw = Tm[:3, :3] @ Ro; cw = Tm[:3, :3] @ c + Tm[:3, 3]; d = dc.reshape(-1, 3) @ Rw.T; d /= np.linalg.norm(d, axis=1, keepdims=True)
    z0 = fh(np.array([cw[0]]), np.array([cw[1]]))[0]; dz = np.minimum(d[:, 2], -1e-3); t = (z0 - cw[2]) / dz
    for _ in range(3): p = cw + t[:, None] * d; t = (fh(p[:, 0], p[:, 1]) - cw[2]) / dz
    p = cw + t[:, None] * d; ok = (d[:, 2] < -0.05) & (t > 0.8) & (t < 10)
    g = oq(p[:, 0], p[:, 1], Gq); vq = oq(p[:, 0], p[:, 1], valid_q) > 0.99; ok &= vq
    g[~ok] = 0; return g.reshape(dc.shape[:2]).astype(np.uint8), ok.reshape(dc.shape[:2])
def ncc(Tm, keep=False):
    num = den = 0.0; out = []
    for f in frames:
        g, ok = synth(Tm, f)
        if ok.sum() < 2000: continue
        a_ = tophat(g)[ok]; b_ = f[6][ok]; a_ -= a_.mean(); b_ -= b_.mean()
        c_ = (a_ * b_).sum() / (np.sqrt((a_ ** 2).sum() * (b_ ** 2).sum()) + 1e-9); num += c_ * ok.sum(); den += ok.sum()
        if keep:
            sy = cv2.resize(cv2.cvtColor(g, cv2.COLOR_GRAY2BGR), (f[1].shape[1], f[1].shape[0]), interpolation=cv2.INTER_NEAREST)
            ph = f[1].copy(); cv2.putText(ph, f'{run} kf{f[0]} {f[5]:.0f}m r={c_:+.2f}', (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            out.append(np.hstack([ph, sy]))
    return (num / den if den else float('nan')), out
def Tshift(dx, dy, dyaw):
    cen = (pick[:, 1:4] @ T[:3, :3].T + T[:3, 3]).mean(0); R = Rot.from_euler('z', dyaw, degrees=True).as_matrix()
    S = np.eye(4); S[:3, :3] = R; S[:3, 3] = cen - R @ cen + np.array([dx, dy, 0]); return S @ T
r0, ov = ncc(T, True)
res = [(dx, dy, yw, ncc(Tshift(dx, dy, yw))[0]) for yw in (-2, 0, 2) for dx in np.arange(-0.6, 0.61, 0.2) for dy in np.arange(-0.6, 0.61, 0.2)]
sc = np.array([t[3] for t in res]); b = res[int(np.nanargmax(sc))]
good = (b[3] - r0 < 0.03) or (abs(b[0]) <= 0.21 and abs(b[1]) <= 0.21 and b[2] == 0)
lines = [f'{run} 경로 {S0:.0f}~{min(S1, s[-1]):.0f}m, 키프레임 {len(frames)}장',
         f'현재 정합: 사진↔정사 합성 선 무늬 상관 r = {r0:+.3f}',
         f'흔든 49×3 경우: 중앙 {np.nanmedian(sc):+.3f}, 최고 {b[3]:+.3f} @ dx {b[0]:+.1f} dy {b[1]:+.1f} 요 {b[2]:+d}°',
         '판정: ' + ('현재 정합이 최고 근처 → 맞음' if good else '더 나은 위치 있음 → 어긋남') + (' (단, 상관 자체가 약함 < 0.15)' if b[3] < 0.15 else '')]
print('\n'.join(lines)); tag = f'{int(S0)}_{int(min(S1, 9999))}'
(O / f'{run}_ortho_{tag}.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
np.save(O / f'{run}_ortho_{tag}_grid.npy', np.array(res))
if ov:
    rows = [np.hstack(ov[j:j + 2] + [np.zeros_like(ov[0])] * (2 - len(ov[j:j + 2]))) for j in range(0, len(ov), 2)]
    cv2.imwrite(str(O / f'{run}_ortho_check.jpg'), np.vstack(rows)[::2, ::2], [cv2.IMWRITE_JPEG_QUALITY, 80])
