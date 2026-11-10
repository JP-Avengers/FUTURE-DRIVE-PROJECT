"""v0.5 바닥 높이 지도(C Ground 형식) 갱신: 바닥 관측이 없던 칸(conf=0)만 손 촬영 바닥(수평 면, 기존 지도 ±0.3m 안)으로 채움.
새 칸은 conf=4(손 촬영) 표시. 다른 칸·키는 그대로. 형상 합치기(v0.5 + 덧붙임)도 함께.
사용: fe_floor_update.py <added.ply> <out_dir>"""
import sys, json, shutil, numpy as np, open3d as o3d, cv2
from pathlib import Path
a = sys.argv; M, O = a[1], Path(a[2]); O.mkdir(parents=True, exist_ok=True)
P = Path.home() / 'b_slam/deliver/map_v0.5'; F = dict(np.load(P / 'floor_height.npz'))
G, lo, c, conf = F['grid'].astype(np.float32), F['lo'], float(F['cell']), F['conf'].copy()
m = o3d.io.read_triangle_mesh(M); m.compute_vertex_normals(); V = np.asarray(m.vertices); N = np.asarray(m.vertex_normals)
ix = ((V[:, 0] - lo[0]) / c).astype(int); iy = ((V[:, 1] - lo[1]) / c).astype(int)
ok = (ix >= 0) & (ix < G.shape[0]) & (iy >= 0) & (iy < G.shape[1]); ix, iy, V, N = ix[ok], iy[ok], V[ok], N[ok]
g0 = G[ix, iy]; fl = (np.abs(N[:, 2]) > 0.9) & (np.abs(V[:, 2] - g0) < 0.3) & (conf[ix, iy] == 0)
lin = ix[fl] * G.shape[1] + iy[fl]; z = V[fl, 2]
order = np.argsort(lin); lin, z = lin[order], z[order]; u, st, cnt = np.unique(lin, return_index=True, return_counts=True)
med = np.array([np.median(z[s:s + k]) for s, k in zip(st, cnt)]); keep = cnt >= 3
u, med = u[keep], med[keep]; ux, uy = np.unravel_index(u, G.shape)
dz = med - G[ux, uy]; G2 = G.copy(); G2[ux, uy] = med; conf2 = conf.copy(); conf2[ux, uy] = 4
F['grid'] = G2.astype(F['grid'].dtype); F['conf'] = conf2
np.savez_compressed(O / 'floor_height.npz', **F)
info = json.loads((P / 'floor_height.json').read_text(encoding='utf-8')) if (P / 'floor_height.json').exists() else {}
info['v0.5e'] = dict(filled_cells=int(len(u)), conf4='손 촬영(11/8 feature_*) 바닥으로 채운 칸 — 기존 conf=0(관측 없음, 외삽)이던 곳',
                     dz_vs_old_cm=dict(p5=round(float(np.percentile(dz, 5)) * 100, 1), p50=round(float(np.median(dz)) * 100, 1), p95=round(float(np.percentile(dz, 95)) * 100, 1)))
(O / 'floor_height.json').write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding='utf-8')
img = np.zeros((G.shape[1], G.shape[0], 3), np.uint8); cc = conf2.T[::-1]
img[cc >= 1] = (150, 150, 150); img[cc == 0] = (40, 40, 40); img[cc == 4] = (0, 200, 0); cv2.imwrite(str(O / 'floor_conf_v05e.png'), img)
print(f'바닥 지도 칸 {G.size:,} 중 관측 없음(conf=0) {int((conf == 0).sum()):,} → 손 촬영으로 채움 {len(u):,}칸 ({len(u) * c * c:.0f}㎡). 기존 외삽값 대비 높이 차 중앙 {np.median(dz) * 100:+.1f}cm, 5~95% {np.percentile(dz, 5) * 100:+.1f}~{np.percentile(dz, 95) * 100:+.1f}cm')
base = o3d.io.read_triangle_mesh(str(P / 'geometry_clean_5cm.ply')); add = o3d.io.read_triangle_mesh(M)
if not base.has_vertex_colors(): base.vertex_colors = o3d.utility.Vector3dVector(np.full((len(base.vertices), 3), 0.6))
out = base + add; out.remove_duplicated_vertices(); out.compute_vertex_normals(); o3d.io.write_triangle_mesh(str(O / 'geometry_v05e.ply'), out)
print(f'형상: v0.5 {len(base.triangles):,} + 손 촬영 {len(add.triangles):,} = {len(out.triangles):,} 삼각형 → {O / "geometry_v05e.ply"}')
