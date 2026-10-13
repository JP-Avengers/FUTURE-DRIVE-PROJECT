#!/usr/bin/env python3
"""C 지적 항목 대리 검증.
 seam <base_mesh> <final_mesh> <이름> : 합치기에서 더해진 바닥 꼭짓점 중 기존(바탕) 바닥과 15cm 안에 붙은 것의 높이 차(이음새 단차)
 kf <pkg_dir> <묶음 접두(keyframes, keyframes_ccw, keyframes_bonnet_cw ...)> <traj 파일> : 키프레임 depth로 만든 바닥 점 − 형상 메시 바닥 높이
(~/tsdf_env)"""
import sys, re
from pathlib import Path
import numpy as np, open3d as o3d, cv2
from scipy.spatial import cKDTree
def floor_mesh(path):
    m = o3d.io.read_triangle_mesh(str(path)); m.compute_vertex_normals(); V, N = np.asarray(m.vertices), np.asarray(m.vertex_normals)
    return V, np.abs(N[:, 2]) > 0.9
mode = sys.argv[1]
if mode == 'seam':
    Vb, fb = floor_mesh(sys.argv[2]); Vf, ff = floor_mesh(sys.argv[3])
    new = np.isinf(cKDTree(Vb).query(Vf, distance_upper_bound=0.002)[0]) & ff
    B = Vb[fb]; d, j = cKDTree(B[:, :2]).query(Vf[new, :2]); k = d < 0.15
    dz = Vf[new][k, 2] - B[j[k], 2]; dz = dz[np.abs(dz) < 0.5]
    print(f'{sys.argv[4]:24s}: 이음새 바닥 꼭짓점 {len(dz):,} | 높이 차 |dz| 중앙 {np.median(np.abs(dz))*100:.1f}cm, 90% {np.percentile(np.abs(dz),90)*100:.1f}cm, 5cm 넘는 비율 {np.mean(np.abs(dz)>0.05)*100:.0f}%')
elif mode == 'kf':
    P, kd, tf = Path(sys.argv[2]), sys.argv[3], sys.argv[4]; K = P / kd
    V, f = floor_mesh(P / 'geometry_rawdepth_tsdf_5cm.ply'); F = V[f]; tF = cKDTree(F[:, :2])
    def qR(x, y, z, w): return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    tr = np.loadtxt(P / tf, comments='#', ndmin=2); out = []
    vv, uu = np.mgrid[240:480:6, 0:640:6]
    for r in tr[::3]:
        i = int(r[8]); dp = K / f'depth/{i}.png'
        if not dp.exists(): continue
        d = cv2.imread(str(dp), cv2.IMREAD_UNCHANGED).astype(float)[vv, uu] / 1000; a = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
        ok = (d > 1.0) & (d < 8); X = np.c_[(uu[ok] - a[2]) * d[ok] / a[0], (vv[ok] - a[5]) * d[ok] / a[4], d[ok]] @ qR(*r[4:8]).T + r[1:4]
        dd, j = tF.query(X[:, :2]); k = dd < 0.05; dz = X[k, 2] - F[j[k], 2]; dz = dz[np.abs(dz) < 0.3]
        if len(dz) > 50: out.append((i, np.median(dz)))
    o = np.array(out)
    print(f'{P.name}/{kd:22s}: 키프레임 {len(o)}장 | 키프레임 바닥 − 메시 바닥: 장별 중앙값의 중앙 {np.median(o[:,1])*100:+.1f}cm, |값| 90% {np.percentile(np.abs(o[:,1]),90)*100:.1f}cm, 5cm 넘는 장 {np.mean(np.abs(o[:,1])>0.05)*100:.0f}%')
    if len(sys.argv) > 5:
        lo, hi = map(int, sys.argv[5].split('-')); s = (o[:, 0] >= lo) & (o[:, 0] <= hi)
        print(f'    ID {lo}~{hi} ({s.sum()}장): 중앙 {np.median(o[s,1])*100:+.1f}cm')
if mode == 'seamcell':
    # 0.5m 칸마다 바탕 바닥과 더해진 바닥이 함께 있으면, 두 높이 중앙값의 차 = 그 칸의 단차(꼭짓점 잡음 제거)
    Vb, fb = floor_mesh(sys.argv[2]); Vf, ff = floor_mesh(sys.argv[3])
    new = np.isinf(cKDTree(Vb).query(Vf, distance_upper_bound=0.002)[0]) & ff; B, A = Vb[fb], Vf[new]
    def cells(X):
        c = np.floor(X[:, :2] / 0.5).astype(np.int64); k = c[:, 0] * 100003 + c[:, 1]; o = np.argsort(k); k, z = k[o], X[o, 2]
        u, st, n = np.unique(k, return_index=True, return_counts=True); return {a: np.median(z[s:s + m]) for a, s, m in zip(u, st, n) if m >= 5}
    cb, ca = cells(B), cells(A); ks = set(cb) & set(ca); st = np.array([ca[k] - cb[k] for k in ks]); st = st[np.abs(st) < 0.5]
    print(f'{sys.argv[4]:24s}: 경계 0.5m 칸 {len(st):,}개 | 칸별 단차 |중앙값 차| 중앙 {np.median(np.abs(st))*100:.1f}cm, 90% {np.percentile(np.abs(st),90)*100:.1f}cm, 5cm 넘는 칸 {np.mean(np.abs(st)>0.05)*100:.0f}%')
