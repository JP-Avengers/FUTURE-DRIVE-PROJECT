"""세션별 확대 대조: 구역마다 [v0.5 정사 | 세션1 BEV | 세션2 BEV ...] (자료 있는 세션만), 1m 격자, 2m마다 좌표.
사용: cp_pair_ms.py <bev_dir> <run1,run2,...> <out_prefix> --reg 이름:cx:cy[:S] ... [--ppm 50]"""
import sys, numpy as np, cv2
from pathlib import Path
a = sys.argv; D, runs, OUT = Path(a[1]), a[2].split(','), a[3]; C = 0.1
PPM = int(a[a.index('--ppm') + 1]) if '--ppm' in a else 50
regs = [r.split(':') for i, r in enumerate(a) if i > 0 and a[i - 1] == '--reg']
Q = np.load(Path.home() / 'b_slam/deliver/map_v0.5/outer/ground_ortho.npz'); qlo = Q['lo']
ortho = np.ascontiguousarray(np.transpose(Q['rgb'], (1, 0, 2))[::-1][:, :, ::-1])
bevs = {r: (np.load(D / f'{r}_bev.npz')['lo'], cv2.imread(str(D / f'{r}_bev_raw.png'))) for r in runs}
def cut(img, lo, x0, y0, n):
    H = img.shape[0]; c0 = int(round((x0 - lo[0]) / C)); r1 = H - int(round((y0 - lo[1]) / C)); out = np.zeros((n, n, 3), np.uint8)
    rs, cs = slice(max(r1 - n, 0), min(r1, H)), slice(max(c0, 0), min(c0 + n, img.shape[1]))
    if rs.stop > rs.start and cs.stop > cs.start: out[rs.start - (r1 - n):rs.stop - (r1 - n), cs.start - c0:cs.stop - c0] = img[rs, cs]
    return out
def panel(img, x0, y0, S, title):
    t = cv2.resize(img, (int(S * PPM), int(S * PPM)), interpolation=cv2.INTER_NEAREST)
    for k in range(int(np.ceil(x0)), int(x0 + S) + 1):
        u = int((k - x0) * PPM); cv2.line(t, (u, 0), (u, t.shape[0] - 1), (0, 140, 255) if k % 2 == 0 else (70, 70, 70), 1)
        if k % 2 == 0: cv2.putText(t, str(k), (u + 2, 11), 0, 0.35, (0, 255, 255), 1)
    for k in range(int(np.ceil(y0)), int(y0 + S) + 1):
        v = t.shape[0] - 1 - int((k - y0) * PPM); cv2.line(t, (0, v), (t.shape[1] - 1, v), (0, 140, 255) if k % 2 == 0 else (70, 70, 70), 1)
        if k % 2 == 0: cv2.putText(t, str(k), (2, v - 2), 0, 0.35, (0, 255, 255), 1)
    cv2.putText(t, title, (4, t.shape[0] - 6), 0, 0.45, (255, 255, 0), 1); return t
for r in regs:
    nm, cx, cy = r[0], float(r[1]), float(r[2]); S = float(r[3]) if len(r) > 3 else 8.0; x0, y0 = cx - S / 2, cy - S / 2; n = int(round(S / C))
    ps = [panel(cut(ortho, qlo, x0, y0, n), x0, y0, S, f'{nm} v0.5')]
    for rn, (lo, im) in bevs.items():
        B = cut(im, lo, x0, y0, n)
        if (B.sum(2) > 0).mean() > 0.05: ps.append(panel(B, x0, y0, S, rn.split('_', 1)[-1]))
    sep = np.full((ps[0].shape[0], 6, 3), 255, np.uint8); row = [ps[0]]
    for p_ in ps[1:]: row += [sep, p_]
    cv2.imwrite(f'{OUT}_{nm}.png', np.hstack(row))
    print(nm, '세션', [p for p in runs if (cut(bevs[p][1], bevs[p][0], x0, y0, n).sum(2) > 0).mean() > 0.05])
