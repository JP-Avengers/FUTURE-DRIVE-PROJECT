"""v0.5 형상 메시 정리 (주행 통로 위주) + 바닥 한 장 + 지도 밖 바닥.
  1) 바닥 밑 면 삭제: 바닥 높이 지도보다 아래로 본네트 칸 5cm, 지붕 칸 15cm, 잰 적 없는 칸 80cm 넘게
  2) 스캔 바닥 층 삭제: 바닥 ±6cm 안의 수평 면 → 5)의 새 바닥 한 장으로 바꿈 (연석·벽 같은 세로 면은 남김)
     차로(본네트 궤적 2m 안)에서는 바닥 위 25cm 이하 수평 면도 삭제 (차로에 그 높이의 수평 면은 없음 — 겹친 바닥 조각)
  3) 차로 잡음 삭제: 본네트 키프레임 depth로 회차별 판정. 2개 이상 회차가 그 점 너머를 봤고(free) 그 점을 제대로 본 회차가
     1개 이하이면 지움 (지나간 사람·차 잔상, 공중 덩어리, 바닥 위 혹). 넓이 3㎡·높이 60cm 넘는 덩어리(주차 차 등)는 판정하지 않음
  4) 조각 정리: 어디서나 1~3삼각형 점과 바닥 위 15cm(본네트 칸은 25cm) 안의 납작한 조각(수평 60%↑),
     차로 안에서는 30삼각형·0.02㎡ 미만 조각, 바닥에서 25cm 넘게 뜬 0.5㎡ 미만 덩어리, 바닥 위 30cm 이하 1㎡ 미만 혹.
     차로 밖에서는 바닥 위 1m 넘게 뜬 0.3㎡ 미만·높이 폭 0.6m 미만 조각(하늘에 점처럼 박히는 것)만 지우고 나무·차·기둥은 남김
  5) 새 바닥: 바닥 높이 지도로 10cm 격자 한 장(스캔 범위 + 25㎡ 미만 구멍 + 0.5m), 나머지(지도 밖, 섬 안쪽)는 0.5m 격자 따로(ground_extended_0.5m.ply)
사용: clean_mesh_v05.py <in_mesh> <floor_height.npz> <pkg_dir> <out_dir> [--lane 2.0]"""
import argparse, json, re
import numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

ap = argparse.ArgumentParser(); ap.add_argument('mesh'); ap.add_argument('floor'); ap.add_argument('pkg'); ap.add_argument('out')
ap.add_argument('--lane', type=float, default=2.0); ap.add_argument('--route', default=None, help='D 기준 경로 csv(run,s,x,y) — 이 경로 2.5m 안도 차로로 봄')
a = ap.parse_args()
P, O = Path(a.pkg), Path(a.out); O.mkdir(parents=True, exist_ok=True); rep = {}
F = np.load(a.floor); G, lo, cell, conf = F['grid'].astype(float), F['lo'], float(F['cell']), F['conf']
def fh(xy): return ndimage.map_coordinates(G, [(xy[:, 0] - lo[0]) / cell - 0.5, (xy[:, 1] - lo[1]) / cell - 0.5], order=1, mode='nearest')
def fc(xy):
    i = np.clip(((xy[:, 0] - lo[0]) / cell).astype(int), 0, G.shape[0] - 1); j = np.clip(((xy[:, 1] - lo[1]) / cell).astype(int), 0, G.shape[1] - 1); return conf[i, j]

m = o3d.io.read_triangle_mesh(a.mesh); V = np.asarray(m.vertices); T = np.asarray(m.triangles); nT0 = len(T)
m.compute_triangle_normals(); NZ = np.asarray(m.triangle_normals)[:, 2]
hv = V[:, 2] - fh(V[:, :2]); cv_ = fc(V[:, :2])
Tr = {d: np.loadtxt(P / f'trajectory_bonnet_{d}_camera_link.txt', comments='#', ndmin=2) for d in ('cw', 'ccw')}
lane_t = cKDTree(np.vstack([t[:, 1:3] for t in Tr.values()]))
lane_v = lane_t.query(V[:, :2], distance_upper_bound=a.lane)[0] <= a.lane
if a.route:
    RR = np.loadtxt(a.route, delimiter=',', skiprows=1)[:, 2:4]; lane_v |= cKDTree(RR).query(V[:, :2], distance_upper_bound=2.5)[0] <= 2.5
th = hv[T]; tmin, tmax = th.min(1), th.max(1); lane_t3 = lane_v[T].all(1)
keep = np.ones(len(T), bool)
cmin_t = cv_[T].min(1); below = tmax < np.select([cmin_t >= 2, cmin_t == 1], [-0.05, -0.15], -0.80); keep &= ~below
floorlayer = (np.abs(NZ) > 0.8) & (np.abs(th).max(1) <= 0.06); keep &= ~floorlayer
lanelow = lane_t3 & (np.abs(NZ) > 0.8) & (tmin > 0.0) & (tmax <= 0.25); keep &= ~lanelow
rep.update(tris_in=int(nT0), below=int(below.sum()), floor_layer=int(floorlayer.sum()), lane_flat_6_15cm=int(lanelow.sum()))
print(f'삼각형 {nT0:,} | 바닥 밑 {below.sum():,} (본네트 칸 {(below & (cmin_t >= 2)).sum():,}, 지붕 칸 {(below & (cmin_t == 1)).sum():,}, 잰 적 없는 칸 {(below & (cmin_t == 0)).sum():,}) | 스캔 바닥 층 {floorlayer.sum():,} | 차로 바닥 위 25cm 이하 수평 {lanelow.sum():,}')

# 3) 차로 키프레임 free-space 판정 — 회차별로 세서, 2개 이상 회차가 '너머를 봤고' 그 물체를 제대로 본 회차가 1개 이하일 때만 지움
#    (주차 차는 그날 모든 회차가 봄 → 남음, 지나간 사람·잡음은 한 회차 이하). 넓이 3㎡ 넘는 덩어리(차 등)는 판정 안 함
def comps(mask):
    q = o3d.geometry.TriangleMesh(m.vertices, o3d.utility.Vector3iVector(T[mask])); c_, n_, a_ = q.cluster_connected_triangles(); return np.asarray(c_), np.asarray(n_), np.asarray(a_)
ik = np.flatnonzero(keep); cl0, n0, ar0 = comps(keep); hx0 = np.full(len(n0), -9.0); np.maximum.at(hx0, cl0, tmax[ik])
bigv = np.zeros(len(V), bool); bigv[T[ik[(ar0[cl0] > 3.0) & (hx0[cl0] > 0.6)]].ravel()] = True   # 크고 높은 덩어리(차 등)만 보호
used = np.zeros(len(V), bool); used[T[keep].ravel()] = True
lane2 = lane_v
cand = np.flatnonzero(used & lane2 & ~bigv & (hv > 0.03) & (hv < 2.5)); C = V[cand]; ct = cKDTree(C[:, :2])
E = Path.home() / 'b_slam/exports'; ses = {}
for si, s in enumerate(['gn04base_s0', 'gn04base_s1', 'gn04base_s2', 'gn04base_s3', 'gm05b_s0', 'gm05b_s1', 'gm05b_s2', 'gm05b_s3', 'gm05b_s4']):
    for t in np.loadtxt(E / s / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 0]: ses[round(t, 2)] = si
NS = 9; free = np.zeros((len(cand), NS), np.int16); seen = np.zeros((len(cand), NS), np.int16); nkf = 0
for d in ('cw', 'ccw'):
    K = P / f'keyframes_bonnet_{d}'
    for r in np.loadtxt(P / f'trajectory_bonnet_{d}_camera_optical.txt', comments='#', ndmin=2):
        i = int(r[8]); dp = K / f'depth/{i}.png'; si = ses.get(round(r[0], 2))
        if not dp.exists() or si is None: continue
        idx = np.array(ct.query_ball_point(r[1:3], 7.0), int)
        if len(idx) == 0: continue
        k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (K / f'calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
        Rm = Rot.from_quat(r[4:8]).as_matrix(); pc = (C[idx] - r[1:4]) @ Rm; z = pc[:, 2]; ok = (z > 0.8) & (z < 7.0)
        u = np.round(k[0] * pc[:, 0] / np.where(ok, z, 1) + k[2]).astype(int); v = np.round(k[4] * pc[:, 1] / np.where(ok, z, 1) + k[5]).astype(int)
        D = cv2.imread(str(dp), cv2.IMREAD_UNCHANGED); h_, w_ = D.shape; ok &= (u >= 1) & (u < w_ - 1) & (v >= 1) & (v < h_ - 1)
        if not ok.any(): continue
        Dm = cv2.medianBlur(D, 3).astype(float) / 1000.0; do = Dm[v[ok], u[ok]]; zz = z[ok]; ii = idx[ok]; val = do > 0.3
        tau = 0.10 + 0.02 * zz
        np.add.at(free[:, si], ii[val & (do > zz + tau)], 1); np.add.at(seen[:, si], ii[val & (np.abs(do - zz) <= tau)], 1); nkf += 1
fs = ((free >= 1) & (free > seen)).sum(1); ss = ((seen >= 2) & (seen >= free)).sum(1); ft, st_ = free.sum(1), seen.sum(1)
rm = (fs >= 2) & (ss <= 1) & (ft >= 3) & (ft > 2 * st_); gone = np.zeros(len(V), bool); gone[cand[rm]] = True
print(f'  판정 키프레임 회차 매칭 {nkf}장, 큰 덩어리 보호 점 {bigv.sum():,}')
carved = keep & gone[T].any(1); keep &= ~carved
rep.update(carve_keyframes=nkf, carve_candidates=int(len(cand)), carve_vertices=int(rm.sum()), carve_tris=int(carved.sum()))
print(f'키프레임 {nkf}장 판정: 후보 점 {len(cand):,} 중 {rm.sum():,}개 지움 → 삼각형 {carved.sum():,}')

# 4) 조각 정리
def sub(mask):
    q = o3d.geometry.TriangleMesh(m.vertices, o3d.utility.Vector3iVector(T[mask])); q.remove_unreferenced_vertices(); return q
idx_keep = np.flatnonzero(keep); q = o3d.geometry.TriangleMesh(m.vertices, o3d.utility.Vector3iVector(T[idx_keep]))
cl, ncl, area = q.cluster_connected_triangles(); cl = np.asarray(cl); ncl = np.asarray(ncl); area = np.asarray(area)
Tk = T[idx_keep]; thk = hv[Tk]; nzk = np.abs(NZ[idx_keep])
cmax = np.full(len(ncl), -9.0); np.maximum.at(cmax, cl, thk.max(1)); cmin = np.full(len(ncl), 9.0); np.minimum.at(cmin, cl, thk.min(1))
hor = np.bincount(cl, (nzk > 0.8).astype(float), len(ncl)) / np.maximum(ncl, 1)
inlane = np.bincount(cl, lane2[Tk].all(1).astype(float), len(ncl)) / np.maximum(ncl, 1) > 0.5
tiny = (ncl < 4) & (area < 0.003)                                               # 1~3삼각형 점 — 차·나무는 잘게 쪼개져 있어 이보다 크면 남김
small = tiny | (inlane & ((ncl < 30) | (area < 0.02)))                          # 차로 밖 작은 조각(나뭇잎 등)은 남김
bon = np.bincount(cl, (cv_[Tk].min(1) >= 2).astype(float), len(ncl)) / np.maximum(ncl, 1) > 0.9
flat = (hor > 0.6) & ((cmax < 0.15) | (bon & (cmax < 0.25) & (area < 5.0)))      # 바닥에 깔린 납작한 조각 (본네트 칸은 25cm까지)
floatc = inlane & (((cmin > 0.25) & (area < 0.5)) | ((cmax < 0.3) & (area < 1.0)))   # 차로 위에 뜬 작은 덩어리, 차로 바닥의 낮은 작은 혹
skyc = ~inlane & (cmin > 1.0) & (area < 0.3) & ((cmax - cmin) < 0.6)             # 차로 밖 하늘에 뜬 작은 조각 (나뭇잎·먼 잡음 — 화면에 점처럼 박힘). 세로로 긴 조각(기둥)은 남김
floatc = floatc | skyc
dropc = small | flat | floatc; dropt = dropc[cl]
rep.update(components=int(len(ncl)), drop_small=int(small.sum()), drop_flat=int((flat & ~small).sum()), drop_floating=int((floatc & ~small & ~flat).sum()), drop_comp_tris=int(dropt.sum()))
print(f'조각 {len(ncl):,}개: 작은 것 {small.sum():,}, 납작한 것 {(flat & ~small).sum():,}, 뜬 것 {(floatc & ~small & ~flat).sum():,} → 삼각형 {dropt.sum():,}')
noise_mask = np.zeros(len(T), bool); noise_mask[idx_keep[dropt]] = True; noise_mask |= carved | lanelow
for nm, cm in (('tiny', tiny), ('small_lane', small & ~tiny), ('flat', flat & ~small), ('floating_lane', floatc & ~small & ~flat)):
    mk = np.zeros(len(T), bool); mk[idx_keep[cm[cl]]] = True; o3d.io.write_triangle_mesh(str(O / f'removed_{nm}.ply'), sub(mk))
o3d.io.write_triangle_mesh(str(O / 'removed_carved.ply'), sub(carved)); o3d.io.write_triangle_mesh(str(O / 'removed_lanelow.ply'), sub(lanelow))
keep[idx_keep[dropt]] = False
o3d.io.write_triangle_mesh(str(O / 'removed_noise.ply'), sub(noise_mask)); o3d.io.write_triangle_mesh(str(O / 'removed_below.ply'), sub(below))
obj = sub(keep)

# 5) 새 바닥 한 장 (10cm) + 지도 밖 바닥 (0.5m)
fp = ndimage.binary_closing(conf >= 1, iterations=3); hl = ndimage.binary_fill_holes(fp) & ~fp; lab, nh = ndimage.label(hl)
hs = ndimage.sum(hl, lab, range(1, nh + 1)); fp |= np.isin(lab, 1 + np.flatnonzero(hs * cell * cell < 25.0))   # 25㎡ 미만 구멍만 채움 (섬 안쪽 화단은 비움)
fp = ndimage.binary_dilation(fp, iterations=5)
def grid_mesh(mask_cells, step, xs, ys, zf, dz=0.0):
    X, Y = np.meshgrid(xs, ys, indexing='ij'); Z = zf(np.c_[X.ravel(), Y.ravel()]).reshape(X.shape) + dz
    nxp, nyp = X.shape; vid = np.arange(nxp * nyp).reshape(nxp, nyp)
    q = mask_cells[:-1, :-1] & mask_cells[1:, :-1] & mask_cells[:-1, 1:] & mask_cells[1:, 1:]
    a0, a1, a2, a3 = vid[:-1, :-1][q], vid[1:, :-1][q], vid[1:, 1:][q], vid[:-1, 1:][q]
    tri = np.r_[np.c_[a0, a1, a2], np.c_[a0, a2, a3]]
    g = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.c_[X.ravel(), Y.ravel(), Z.ravel()]), o3d.utility.Vector3iVector(tri)); g.remove_unreferenced_vertices(); g.compute_vertex_normals(); return g
xs = lo[0] + (np.arange(G.shape[0]) + 0.5) * cell; ys = lo[1] + (np.arange(G.shape[1]) + 0.5) * cell
floor = grid_mesh(fp, cell, xs, ys, fh)
s5 = int(round(0.5 / cell)); xc, yc = xs[::s5], ys[::s5]; fpc = ndimage.binary_erosion(fp, iterations=3)[::s5, ::s5]
ext = grid_mesh(~fpc, 0.5, xc, yc, fh, dz=-0.01)
geo = obj + floor; geo.compute_vertex_normals()
o3d.io.write_triangle_mesh(str(O / 'geometry_v05.ply'), geo); o3d.io.write_triangle_mesh(str(O / 'ground_extended_0.5m.ply'), ext)
o3d.io.write_triangle_mesh(str(O / 'objects_only.ply'), obj)
rep.update(tris_objects=len(obj.triangles), tris_floor=len(floor.triangles), tris_out=len(geo.triangles), tris_ground_extended=len(ext.triangles), floor_area_m2=float(fp.sum() * cell * cell))
json.dump(rep, open(O / 'clean_report.json', 'w'), indent=1); print(json.dumps(rep, ensure_ascii=False))
