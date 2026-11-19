"""두 메쉬의 같은 벽 펼친 그림(in_wallraster.py npz)을 창마다 맞대어 높이 차이 찾기 — B 의 z 를 dz 만큼, x 를 dx 만큼 옮겨 가며
밝기 · 들어감 지도(넓은 흐름 0.1 m 가우시안을 뺀 잔무늬)의 정규화 상관이 가장 큰 곳. dz = B − A (같은 무늬가 B 에서 dz 만큼 높음).
창마다 가로 무늬(위아래 밝기 기울기) 높이도 따로 출력.
사용: in_wallmatch.py <A.npz> <B.npz> <out.png> x0:x1:z0:z1 [...] [--dz 0.2] [--dx 0.3] [--min 150]"""
import sys, numpy as np
from scipy import ndimage
a = sys.argv; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
A, B = np.load(a[1]), np.load(a[2]); C = float(A['cell']); x0g, z0g = float(A['x0']), float(A['z0'])
assert float(B['cell']) == C and float(B['x0']) == x0g and float(B['z0']) == z0g
wins = [[float(v) for v in w.split(':')] for w in a[4:] if w.count(':') == 3]; DZ, DX, MIN = opt('--dz', 0.2), opt('--dx', 0.3), opt('--min', 150)
def hp(img, ok, s=5):                     # 칸 비어 있는 곳을 뺀 가우시안 (정규화 합성곱) 을 뺀 잔무늬
    w = ndimage.gaussian_filter(ok.astype(float), s); v = ndimage.gaussian_filter(np.where(ok, img, 0), s); return np.where(ok, img - v / np.maximum(w, 1e-6), 0)
feat = {}
for nm, D in (('A', A), ('B', B)):
    ok = (D['cnt'] > 0) & np.isfinite(D['lum']) & np.isfinite(D['dep']); feat[nm] = (ok, hp(np.nan_to_num(D['lum']), ok), hp(np.nan_to_num(D['dep']), ok))
def ncc(p, q):
    p, q = p - p.mean(), q - q.mean(); return float((p * q).sum() / (np.sqrt((p * p).sum() * (q * q).sum()) + 1e-12))
sz, sx = int(round(DZ / C)), int(round(DX / C)); res = []
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, axs = plt.subplots(len(wins), 3, figsize=(15, 3.2 * len(wins)), squeeze=False)
for wi, (x0, x1, z0, z1) in enumerate(wins):
    i0, i1, k0, k1 = [int(round(v)) for v in ((x0 - x0g) / C, (x1 - x0g) / C, (z0 - z0g) / C, (z1 - z0g) / C)]
    okA, lA, dA = (f[i0:i1, k0:k1] for f in feat['A']); S = {}
    for di in range(-sx, sx + 1):
        for dk in range(-sz, sz + 1):
            sl = (slice(i0 + di, i1 + di), slice(k0 + dk, k1 + dk))
            if i0 + di < 0 or k0 + dk < 0 or i1 + di > feat['B'][0].shape[0] or k1 + dk > feat['B'][0].shape[1]: continue
            okB, lB, dB = (f[sl] for f in feat['B']); m = okA & okB
            if m.sum() < MIN: continue
            S[(di, dk)] = (ncc(lA[m], lB[m]), ncc(dA[m], dB[m]), int(m.sum()))
    if not S: print(f'창 x {x0}~{x1} z {z0}~{z1}: 겹친 칸 부족'); continue
    for j, name in ((0, '밝기'), (1, '들어감'), (None, '합')):
        sc = {k: (v[0] + v[1]) / 2 if j is None else v[j] for k, v in S.items()}; bk = max(sc, key=sc.get)
        prof = np.array([max([sc[(di, dk)] for di in range(-sx, sx + 1) if (di, dk) in sc] or [np.nan]) for dk in range(-sz, sz + 1)])
        far = [prof[q] for q in range(len(prof)) if abs(q - sz - bk[1]) * C > 0.06 and np.isfinite(prof[q])]; second = max(far) if far else np.nan
        kk = bk[1] + sz; sub = 0.0
        if 0 < kk < len(prof) - 1 and np.isfinite(prof[kk - 1:kk + 2]).all():
            y0, y1, y2 = prof[kk - 1:kk + 2]; den = y0 - 2 * y1 + y2; sub = 0.5 * (y0 - y2) / den if den < 0 else 0.0
        dz = (bk[1] + sub) * C; res.append((wi, name, dz, bk[0] * C, sc[bk], second, S[bk][2]))
        print(f'창 x {x0}~{x1} z {z0}~{z1} [{name}]: dz(B−A) {dz*100:+.1f} cm · dx {bk[0]*C*100:+.0f} cm | 상관 {sc[bk]:.2f} (dz 6 cm 밖 최고 {second:.2f}) · 겹친 칸 {S[bk][2]}')
        axs[wi, 0].plot((np.arange(-sz, sz + 1)) * C * 100, prof, label=name)
    axs[wi, 0].set_title(f'x {x0}-{x1} z {z0}-{z1}: best NCC vs dz (B-A, cm)', fontsize=8); axs[wi, 0].legend(fontsize=7); axs[wi, 0].grid(alpha=.3)
    # 가로 무늬 높이: 위아래 밝기 기울기의 x 평균 (A 그대로 · B 는 찾은 dx 만 옮김)
    bdx = int(round(res[-1][3] / C)); kl = max(k0 - sz, 0); zc = z0g + (np.arange(kl, k1 + sz) + 0.5) * C
    for nm, di, ax in (('A', 0, axs[wi, 1]), ('B', bdx, axs[wi, 2])):
        ok, L, _ = feat[nm]; L = np.where(ok, ndimage.gaussian_filter1d(np.nan_to_num((A if nm == 'A' else B)['lum']), 1, axis=1), np.nan)[i0 + di:i1 + di, kl:k1 + sz]
        g = np.nanmean(np.abs(np.diff(L, axis=1)), axis=0); g = np.r_[g, np.nan]
        pk = [q for q in range(1, len(g) - 1) if np.isfinite(g[q - 1:q + 2]).all() and g[q] >= g[q - 1] and g[q] >= g[q + 1] and g[q] > np.nanpercentile(g, 80)]
        pk = sorted(pk, key=lambda q: -g[q])[:4]; print(f'    {nm} 가로 무늬 높이(밝기 기울기 큰 순): ' + ', '.join(f'{zc[q] + C / 2:.2f} m ({g[q]:.3f})' for q in sorted(pk)))
        img = (A if nm == 'A' else B)['lum'][i0 + di:i1 + di, kl:k1 + sz]
        ax.imshow(img.T, origin='lower', extent=[x0 + di * C, x1 + di * C, zc[0] - C / 2, zc[-1] + C / 2], cmap='gray', vmin=0, vmax=1, aspect='equal'); ax.set_title(f'{nm} ({a[1 if nm == "A" else 2].split("/")[-1]})', fontsize=8)
        ax.axhspan(z0, z1, color='y', alpha=0.08)
plt.tight_layout(); plt.savefig(a[3], dpi=80)
for name in ('밝기', '들어감', '합'):
    v = [r[2] for r in res if r[1] == name]
    if v: print(f'[{name}] 창별 dz(B−A) cm: ' + ' '.join(f'{x*100:+.1f}' for x in v) + f' | 중앙 {np.median(v)*100:+.1f} cm')
print('→', a[3])
