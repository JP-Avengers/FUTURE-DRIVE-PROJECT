"""정리 전후 비교 그림: 로봇 시점(높이 0.45m, 수평, 가로 화각 80°) 전 | 후 (전 그림에서 지운 면은 빨강) + 위에서 본 지운 조각 위치.
사용: dc_views.py <before.ply> <after.ply> <out_prefix> [--x 7,34,38] [--y 0.15]"""
import sys, numpy as np, open3d as o3d, cv2, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
a = sys.argv; B, A, OUT = a[1], a[2], a[3]; opt = lambda k, d: a[a.index(k) + 1] if k in a else d
XS = [float(v) for v in opt('--x', '7,34,38').split(',')]; YL = float(opt('--y', '0.15'))
def load(p):
    m = o3d.io.read_triangle_mesh(p); sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m)); return m, sc
mb, sb = load(B); Vb = np.asarray(mb.vertices); Tb = np.asarray(mb.triangles); Cb = np.asarray(mb.vertex_colors)
ma, sa = load(A); Va = np.asarray(ma.vertices); Ta = np.asarray(ma.triangles); Ca = np.asarray(ma.vertex_colors)
# 지운 삼각형 = 전 메시에서 중심이 후 메시 어느 삼각형 중심과도 1mm 안에 없는 것
from scipy.spatial import cKDTree
cb = Vb[Tb].mean(1); ca_ = Va[Ta].mean(1); d, _ = cKDTree(ca_).query(cb, distance_upper_bound=1e-3); gone = ~np.isfinite(d)
print('지운 삼각형', gone.sum())
W_, H_ = 480, 300; fx = (W_ / 2) / np.tan(np.radians(40)); uu, vv = np.meshgrid(np.arange(W_) - W_ / 2 + 0.5, np.arange(H_) - H_ / 2 + 0.5)
def shade(sc, T, C, o, dirs, mark=None):
    r = sc.cast_rays(o3d.core.Tensor(np.c_[np.tile(o, (len(dirs), 1)), dirs].astype(np.float32))); t = r['t_hit'].numpy(); h = np.isfinite(t) & (t < 12); pid = r['primitive_ids'].numpy()
    img = np.full((len(dirs), 3), (255, 200, 255), np.uint8); img[h] = np.clip(C[T[pid[h], 0]][:, ::-1] * 255, 0, 255).astype(np.uint8)
    if mark is not None: mk = np.zeros(len(dirs), bool); mk[h] = mark[pid[h]]; img[mk] = (0, 0, 255)
    return img.reshape(H_, W_, 3), h.mean()
rows = []
for x in XS:
    for sgn in (1, -1):
        dirs = np.stack([np.full(uu.shape, sgn * fx), -sgn * uu, -vv], -1).reshape(-1, 3); dirs /= np.linalg.norm(dirs, axis=1, keepdims=True); o = [x, YL, 0.45]
        ib, fb = shade(sb, Tb, Cb, o, dirs, gone); ia, fa = shade(sa, Ta, Ca, o, dirs)
        cv2.putText(ib, f'before x={x:.0f} {"+x" if sgn > 0 else "-x"} fill {fb:.1%}', (4, 16), 0, 0.5, (0, 0, 255), 1); cv2.putText(ia, f'after fill {fa:.1%}', (4, 16), 0, 0.5, (0, 0, 255), 1)
        rows.append(np.hstack([ib, ia]))
cv2.imwrite(OUT + '_robot.jpg', np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 85])
g = cb[gone]; fig, ax = plt.subplots(2, 1, figsize=(20, 6))
s = np.random.default_rng(0).choice(len(cb), 400000, replace=False); keep = s[(cb[s, 2] < 2.0)]
ax[0].scatter(cb[keep, 0], cb[keep, 1], s=0.2, c='0.75'); sc_ = ax[0].scatter(g[:, 0], g[:, 1], s=2, c=g[:, 2], cmap='jet', vmin=0, vmax=2.9); ax[0].set_title('removed (top view, color = z)'); ax[0].set_aspect('equal'); plt.colorbar(sc_, ax=ax[0])
ax[1].scatter(cb[s, 0], cb[s, 2], s=0.2, c='0.75'); ax[1].scatter(g[:, 0], g[:, 2], s=2, c='r'); ax[1].set_title('removed (side view x-z)'); ax[1].set_aspect('equal')
plt.tight_layout(); plt.savefig(OUT + '_top.png', dpi=70)
# 빨간 물체(소화기 후보) 중 지운 것
red = (Cb[:, 0] > 0.45) & (Cb[:, 1] < 0.25) & (Cb[:, 2] < 0.25); rt = red[Tb].all(1)
print(f'빨간 삼각형 {rt.sum():,} 중 지운 것 {(rt & gone).sum():,}; 지운 것 중 로비 x18~27 {np.sum(gone & (cb[:, 0] > 18) & (cb[:, 0] < 27)):,}')
