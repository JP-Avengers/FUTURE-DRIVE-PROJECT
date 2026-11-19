"""문 위 인방(문틀 윗면 밑면) 높이 — 벽 펼친 그림(in_wallraster.py npz, 바닥 기준 메쉬)에서 z 0.5~1.7 m 가 비어 있는 x 구간(≥0.5 m) = 문 자리로 잡고,
그 x 구간 안 · 벽면 뒤 0.5 m 까지의 아래 향한 면(법선 z < −0.8, z 1.6~2.6 m) 높이 중앙값을 메쉬마다 출력.
사용: in_doorhead.py <wall.npz> <mesh.ply>... [--wall -1.10] [--zmin 1.6] [--zmax 2.6]"""
import sys, numpy as np, open3d as o3d
a = sys.argv; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d; YW, ZL, ZH = opt('--wall', -1.10), opt('--zmin', 1.6), opt('--zmax', 2.6)
R = np.load(a[1]); C = float(R['cell']); x0 = float(R['x0']); z0 = float(R['z0']); cnt = R['cnt']
zc = z0 + (np.arange(cnt.shape[1]) + 0.5) * C; xc = x0 + (np.arange(cnt.shape[0]) + 0.5) * C
band = (zc > 0.5) & (zc < 1.7); cov = (cnt[:, band] > 0).mean(1); seen = (cnt[:, (zc > 0.2) & (zc < 1.7)] > 0).any(1)
gap = cov < 0.2; doors = []; i = 0
while i < len(gap):
    if gap[i]:
        j = i
        while j < len(gap) and gap[j]: j += 1
        if (j - i) * C >= 0.5 and (j - i) * C <= 2.2 and i > 5 and j < len(gap) - 5 and cov[max(i - 10, 0):i].mean() > 0.5 and cov[j:j + 10].mean() > 0.5: doors.append((xc[i], xc[j - 1]))
        i = j
    else: i += 1
print(f'{a[1]}: 문 자리 후보 {len(doors)}개 — ' + ' '.join(f'[{p:.2f}~{q:.2f}]' for p, q in doors))
meshes = [(p, o3d.io.read_triangle_mesh(p)) for p in a[2:] if p.endswith('.ply')]
for p, m in meshes: m.compute_vertex_normals()
print('문 x 구간 | ' + ' | '.join(p.split('/')[-1] for p, _ in meshes) + '   (아래 향한 면 z 중앙 · 25~75% · 정점 수)')
for p_, q_ in doors:
    row = []
    for p, m in meshes:
        V, N = np.asarray(m.vertices), np.asarray(m.vertex_normals); d = (V[:, 1] - YW) * np.sign(YW)
        k = (N[:, 2] < -0.8) & (V[:, 0] > p_ + 0.1) & (V[:, 0] < q_ - 0.1) & (d > -0.05) & (d < 0.5) & (V[:, 2] > ZL) & (V[:, 2] < ZH)
        row.append(f'{np.median(V[k,2]):.3f} ({np.percentile(V[k,2],25):.2f}~{np.percentile(V[k,2],75):.2f}, n {k.sum()})' if k.sum() >= 15 else f'- (n {k.sum()})')
    print(f'  {p_:5.2f}~{q_:5.2f} | ' + ' | '.join(row))
