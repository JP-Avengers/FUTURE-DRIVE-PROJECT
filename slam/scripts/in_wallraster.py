"""벽 펼친 그림 — 벽선(y 일정) ±band 안 정점을 (x, z) 2 cm 칸에 모아 밝기 · 들어감(벽 안쪽 +, m) 지도를 npz · PNG 로.
사용: in_wallraster.py <mesh.ply> <out_prefix> [--wall -1.10] [--band 0.15] [--x0 0 --x1 46] [--z0 0 --z1 3.2] [--cell 0.02]
  --wall 음수 = −y 쪽 벽 (y 가 더 작으면 벽 안쪽), 양수 = +y 쪽 벽."""
import sys, numpy as np, open3d as o3d
a = sys.argv; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
YW, B, C = opt('--wall', -1.10), opt('--band', 0.15), opt('--cell', 0.02); X0, X1, Z0, Z1 = opt('--x0', 0), opt('--x1', 46), opt('--z0', 0), opt('--z1', 3.2)
m = o3d.io.read_triangle_mesh(a[1]); V = np.asarray(m.vertices); Cc = np.asarray(m.vertex_colors)
s = np.sign(YW); d = (V[:, 1] - YW) * s           # 벽 안쪽(복도 밖) +
k = (np.abs(d) < B) & (V[:, 0] >= X0) & (V[:, 0] < X1) & (V[:, 2] >= Z0) & (V[:, 2] < Z1)
V, d, L = V[k], d[k], Cc[k] @ [0.299, 0.587, 0.114] if len(Cc) else np.zeros(k.sum())
nx, nz = int(round((X1 - X0) / C)), int(round((Z1 - Z0) / C))
ix = np.clip(((V[:, 0] - X0) / C).astype(int), 0, nx - 1); iz = np.clip(((V[:, 2] - Z0) / C).astype(int), 0, nz - 1); key = ix * nz + iz
cnt = np.bincount(key, minlength=nx * nz).reshape(nx, nz).astype(float)
lum = np.bincount(key, L, nx * nz).reshape(nx, nz) / np.maximum(cnt, 1)
o = np.lexsort((d, key)); ks, st, ct = np.unique(key[o], return_index=True, return_counts=True)   # 칸마다 들어감 중앙값
dep = np.full(nx * nz, np.nan); dep[ks] = d[o][st + ct // 2]; dep = dep.reshape(nx, nz); lum[cnt == 0] = np.nan
np.savez(a[2] + '.npz', lum=lum, dep=dep, cnt=cnt, x0=X0, z0=Z0, cell=C, wall=YW)
zc = Z0 + (np.arange(nz) + 0.5) * C; have = (cnt > 0).mean(0)
print(f'{a[1]} 벽 y={YW} ±{B}: 정점 {k.sum():,} | 칸이 찬 비율 z 0.2 m 구간별: ' + ' '.join(f'{z:.1f}:{have[(zc>=z)&(zc<z+0.2)].mean():.0%}' for z in np.arange(Z0, Z1, 0.2)))
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, ax = plt.subplots(2, 1, figsize=(30, 5.5))
ax[0].imshow(lum.T, origin='lower', extent=[X0, X1, Z0, Z1], aspect='equal', cmap='gray', vmin=0, vmax=1); ax[0].set_title(f'{a[1].split("/")[-1]} wall y={YW}: luminance')
ax[1].imshow(dep.T, origin='lower', extent=[X0, X1, Z0, Z1], aspect='equal', cmap='RdBu', vmin=-0.12, vmax=0.12); ax[1].set_title('relief (+ = recessed into wall, red)')
for x_ in ax: x_.set_xticks(np.arange(X0, X1 + 1, 1)); x_.set_yticks(np.arange(Z0, Z1 + 0.01, 0.5)); x_.grid(alpha=0.3); x_.tick_params(labelsize=6)
plt.tight_layout(); plt.savefig(a[2] + '.png', dpi=70)
