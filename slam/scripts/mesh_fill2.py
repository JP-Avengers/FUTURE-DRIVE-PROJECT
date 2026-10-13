#!/usr/bin/env python3
"""mesh_fill2: mesh_fill.py + 바닥 겹침 규칙. 바닥 삼각형(|nz|>0.9)은 바탕에 바닥(|nz|>0.95)이 수평 15cm·높이 50cm 안에 있으면 넣지 않음(바닥 두 겹 방지).
(원래 설명) v0.3 형상 메시 = v0.2 메시(그대로) + 반시계 메시에서 v0.2 표면이 없는 부분만.
 - 반시계 메시: run20nr(정합된 자세)로 만든 원본 depth TSDF
 - 선택: 삼각형 무게중심에서 v0.2 꼭짓점까지 15cm보다 먼 삼각형
 - 조각별 높이 맞춤: 조각 둘레 1m 안에서 v0.2 바닥과 겹치는 반시계 꼭짓점들의 높이 차 중앙값만큼 z 이동
사용: mesh_fill.py <v02_mesh> <ccw_mesh> <out_mesh>   (~/tsdf_env)"""
import sys
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
a, b, out = sys.argv[1:4]
A = o3d.io.read_triangle_mesh(a); B = o3d.io.read_triangle_mesh(b)
VA, VB, FB = np.asarray(A.vertices), np.asarray(B.vertices), np.asarray(B.triangles)
tA = cKDTree(VA)
cen = VB[FB].mean(1); d = tA.query(cen, distance_upper_bound=0.15)[0]
keep = np.isinf(d)
# 바닥 겹침 규칙: 높이가 달라도 같은 자리에 바닥이 이미 있으면 바닥 삼각형은 넣지 않음
A.compute_vertex_normals(); B.compute_triangle_normals()
_NA = np.asarray(A.vertex_normals); _fa = np.abs(_NA[:, 2]) > 0.95; _GA = VA[_fa]; _tG = cKDTree(_GA[:, :2])
_ft = np.abs(np.asarray(B.triangle_normals)[:, 2]) > 0.9
import os as _os
_MINN = int(_os.environ.get('FILL_MINN', '1'))  # 반경 15cm 안 바탕 바닥 꼭짓점이 이 수 이상일 때만 '바닥 있음' (띄엄띄엄 찍힌 먼 바닥이 채움을 막지 않게)
_c = np.nonzero(keep & _ft)[0]; _nb = _tG.query_ball_point(cen[_c, :2], 0.15)
_hit = np.array([sum(abs(_GA[i, 2] - cen[c_, 2]) < 0.5 for i in nb) >= _MINN for c_, nb in zip(_c, _nb)], bool) if len(_c) else np.zeros(0, bool)
keep[_c[_hit]] = False
print(f'바닥 겹침 규칙: 바닥 삼각형 {len(_c):,} 중 {_hit.sum():,}개는 이미 바닥이 있는 자리라 제외')
F = FB[keep]
print(f'반시계 삼각형 {len(FB):,} 중 v0.2에 없는 부분 {keep.sum():,} ({keep.mean()*100:.0f}%)')
# 조각(연결 요소) 나누기
used = np.unique(F); remap = -np.ones(len(VB), int); remap[used] = np.arange(len(used))
Fl = remap[F]; n = len(used)
e = np.r_[Fl[:, [0, 1]], Fl[:, [1, 2]], Fl[:, [2, 0]]]
ncomp, lab = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)), directed=False)
V = VB[used].copy()
# 높이 맞춤: 반시계 꼭짓점 중 v0.2와 겹치는 것(5cm 안, 수평 기준)으로 국소 dz
A.compute_vertex_normals(); B.compute_vertex_normals()
fA = np.abs(np.asarray(A.vertex_normals)[:, 2]) > 0.95; fB = np.abs(np.asarray(B.vertex_normals)[:, 2]) > 0.95  # 바닥처럼 수평인 면
iA = np.nonzero(fA)[0]; tA2 = cKDTree(VA[iA, :2]); dB, jB = tA2.query(VB[:, :2]); jB = iA[np.minimum(jB, len(iA) - 1)]
ovl = fB & (dB < 0.05) & (np.abs(VA[jB, 2] - VB[:, 2]) < 0.4)  # 바닥끼리만 비교
print(f'바닥 겹침 꼭짓점 {ovl.sum():,}, 높이 차(v0.2-반시계) 중앙 {np.median(VA[jB[ovl],2]-VB[ovl,2])*100:+.1f}cm')
tOv = cKDTree(VB[ovl][:, :2]); dzOv = np.r_[VA[jB[ovl], 2] - VB[ovl, 2], np.nan]
dd, ii = tOv.query(V[:, :2], k=16, distance_upper_bound=1.0)  # 꼭짓점별 1m 안 겹침점 최대 16개
vdz = np.nanmedian(np.where(np.isinf(dd), np.nan, dzOv[np.minimum(ii, len(dzOv) - 1)]), axis=1)  # 없으면 nan
order = np.argsort(lab); bounds = np.searchsorted(lab[order], np.arange(ncomp + 1))
shifts = []
for c in range(ncomp):
    idx = order[bounds[c]:bounds[c + 1]]; z = vdz[idx]; z = z[~np.isnan(z)]
    if len(z) >= 5:
        dz = float(np.clip(np.median(z), -0.3, 0.3)); V[idx, 2] += dz; shifts.append(dz)
shifts = np.array(shifts)
print(f'조각 {ncomp}개, 높이 맞춤 적용 {len(shifts)}개: 이동 중앙 {np.median(np.abs(shifts))*100:.1f}cm, 90% {np.percentile(np.abs(shifts),90)*100:.1f}cm')
# 작은 조각(삼각형 30개 미만)은 바닥 조각(수평 + 궤적 기준 바닥 높이)이면 남기고, 공중에 뜬 것만 제외
TR = np.loadtxt(sys.argv[4] if len(sys.argv) > 4 else 'deliver/map_v0.2/trajectory_camera_link.txt', ndmin=2)[:, 1:4]
_, jt = cKDTree(TR[:, :2]).query(V[:, :2]); hz = TR[jt, 2] - V[:, 2]
nB = np.asarray(B.vertex_normals)[used]
floorv = (np.abs(nB[:, 2]) > 0.9) & (hz > 1.0) & (hz < 2.2)
tri_lab = lab[Fl[:, 0]]; cnt = np.bincount(tri_lab, minlength=ncomp)
ffrac = np.bincount(lab, weights=floorv.astype(float), minlength=ncomp) / np.maximum(np.bincount(lab, minlength=ncomp), 1)
okc = (cnt >= 30) | (ffrac >= 0.8); big = okc[tri_lab]
print(f'공중 파편 제외: 조각 {okc.sum()}/{ncomp}개 유지(작은 바닥 조각 {((cnt < 30) & (ffrac >= 0.8)).sum()}개 포함), 삼각형 {big.sum():,}/{len(Fl):,} 유지')
Fl = Fl[big]
P = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(Fl)); P.remove_unreferenced_vertices()
M = A + P; M.remove_duplicated_vertices(); M.compute_vertex_normals()
o3d.io.write_triangle_mesh(out, M)
print(f'결과: 삼각형 {len(M.triangles):,} (v0.2 {len(A.triangles):,} + 추가 {len(Fl):,}) → {out}')
